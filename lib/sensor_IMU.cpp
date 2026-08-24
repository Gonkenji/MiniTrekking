#include "sensor_imu.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <math.h>
#include <stdio.h>

#define SPI_PORT spi0
#define PIN_MISO 16 //ad0 - laranja
#define PIN_CS   17 //ncs - amarelo 
#define PIN_SCK  18 //scl - verde
#define PIN_MOSI 19 //sda - azul

// Constantes calculadas apenas uma vez na inicialização
static float offset_gy = 0.0f, offset_gz = 0.0f;
static float const_sin_r = 0.0f, const_cos_r = 1.0f, const_cos_p = 1.0f;
static float yaw_internal = 0.0f;

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void icm_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}

static void icm_write_register(uint8_t reg, uint8_t data) {
    reg &= 0x7F;
    uint8_t buf[2] = {reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

static void calibrar_sensores() {
    int32_t soma_ax = 0, soma_ay = 0, soma_az = 0;
    int32_t soma_gy = 0, soma_gz = 0; 
    const int amostras = 500;
    uint8_t buffer[12];

    for (int i = 0; i < amostras; i++) {
        // Lê Acelerômetro e Giroscópio (0x2D a 0x38)
        icm_read_registers(0x2D, buffer, 12);
        
        soma_ax += (int16_t)((buffer[0] << 8) | buffer[1]);
        soma_ay += (int16_t)((buffer[2] << 8) | buffer[3]);
        soma_az += (int16_t)((buffer[4] << 8) | buffer[5]);
        
        // buffer[6] e [7] são o Giroscópio X (não usado no Yaw)
        soma_gy += (int16_t)((buffer[8] << 8) | buffer[9]);
        soma_gz += (int16_t)((buffer[10] << 8) | buffer[11]);
        sleep_ms(2);
    }
    
    // 1. Offsets do Giroscópio
    offset_gy = (float)soma_gy / amostras;
    offset_gz = (float)soma_gz / amostras;

    // 2. Descobre a inclinação física de montagem (Robô parado)
    float ax_med = (float)soma_ax / amostras;
    float ay_med = (float)soma_ay / amostras;
    float az_med = (float)soma_az / amostras;

    float roll_rad = atan2(ay_med, az_med);
    float pitch_rad = atan2(-ax_med, sqrt(ay_med*ay_med + az_med*az_med));

    // 3. Pré-calcula a trigonometria
    const_sin_r = sin(roll_rad);
    const_cos_r = cos(roll_rad);
    const_cos_p = cos(pitch_rad);
}

void imu_init() {
    // 1. Corrigido para realmente operar em 1 MHz (estava em 5 MHz)
    spi_init(SPI_PORT, 1000 * 1000); 
    
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    
    sleep_ms(50);
    
    // --- VERIFICAÇÃO DE HARDWARE (WHO AM I) ---
    uint8_t who_am_i = 0;
    icm_read_registers(0x00, &who_am_i, 1);
    
    // Verifica no terminal se o sensor respondeu
    printf("SPI IMU: Lendo WHO_AM_I... Valor recebido: 0x%02X\n", who_am_i);
    
    // O valor correto para o ICM20948 é 0xEA
    if (who_am_i != 0xEA) {
        printf("ERRO CRÍTICO: Sensor não detectado. Valor recebido não é 0xEA!\n");
        // Trava o código aqui para você arrumar o hardware
        while(true) {
            printf("Hardware travado. Verifique a pinagem MISO, MOSI, SCK, CS e alimentação.\n");
            sleep_ms(1000);
        }
    }

    icm_write_register(0x06, 0x01); // Acorda o sensor
    sleep_ms(50);
    
    // --- DESATIVAÇÃO FORÇADA DO I2C INTERNO ---
    // Registrador 0x03 (USER_CTRL) -> bit 4 = I2C_IF_DIS (0x10)
    icm_write_register(0x03, 0x10);
    sleep_ms(10);
    
    // --- CONFIGURAÇÃO DO DLPF (FILTRO PASSA-BAIXA) ---
    icm_write_register(0x7F, 0x20); // Muda para o Banco 2
    icm_write_register(0x01, 0x19); // Configura GYRO_CONFIG_1
    icm_write_register(0x7F, 0x00); // Volta para o Banco 0

    calibrar_sensores();
}

void imu_update(float dt, float &yaw) {
    // 0x35 é o registrador GYRO_YOUT_H. Lemos apenas 4 bytes (Y e Z)
    uint8_t buffer_gyro[4]; 
    icm_read_registers(0x35, buffer_gyro, 4);
    
    // Os índices mudam para acomodar o novo buffer menor
    float raw_gy = (float)((int16_t)((buffer_gyro[0] << 8) | buffer_gyro[1]));
    float raw_gz = (float)((int16_t)((buffer_gyro[2] << 8) | buffer_gyro[3]));
    
    float gy = (raw_gy - offset_gy) / 131.0f;
    float gz = (raw_gz - offset_gz) / 131.0f;

    float yaw_rate_world = gz; 
    if (fabs(const_cos_p) > 0.01f) { 
        yaw_rate_world = gy * (const_sin_r / const_cos_p) + gz * (const_cos_r / const_cos_p);
    }

    yaw_internal = yaw_internal + (yaw_rate_world * dt);
    yaw = yaw_internal;
}