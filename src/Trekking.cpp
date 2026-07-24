#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "I2C.h"
#include "VL53L0X.h"

// ==========================================
// DEFINIÇÕES - SENSORES VL53L0X (ToF) - I2C0
// ==========================================
#define TOF_I2C_SDA_PIN 4
#define TOF_I2C_SCL_PIN 5

// Novos pinos XSHUT definidos
#define XSHUT_SENSOR_1 12 
#define XSHUT_SENSOR_2 13

#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31

VL53L0X sensor1;
VL53L0X sensor2;

// ==========================================
// DEFINIÇÕES - SENSOR TCS34725 (Cor) - I2C1
// ==========================================
#define TCS34725_ADDR 0x29
#define TCS_COMMAND_BIT 0x80
#define TCS_ENABLE      (0x00 | TCS_COMMAND_BIT)
#define TCS_ATIME       (0x01 | TCS_COMMAND_BIT)
#define TCS_CONTROL     (0x0F | TCS_COMMAND_BIT)
#define TCS_CDATAL      (0x14 | TCS_COMMAND_BIT)

#define COLOR_I2C_SDA_PIN 14
#define COLOR_I2C_SCL_PIN 15

// ==========================================
// DEFINIÇÕES - SENSOR IMU - SPI0
// ==========================================
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// ==========================================
// FUNÇÕES AUXILIARES - SPI / IMU
// ==========================================
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

void icm_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80; // MSB = 1 para leitura[cite: 3]
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}

void icm_write_register(uint8_t reg, uint8_t data) {
    reg &= 0x7F; // MSB = 0 para escrita[cite: 3]
    uint8_t buf[2] = {reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

// ==========================================
// FUNÇÕES AUXILIARES - I2C / TOF e COR
// ==========================================
void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(50);
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor->setMeasurementTimingBudget(50000); 
}

void tcs34725_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c1, TCS34725_ADDR, buffer, 2, false);
}

void tcs34725_init_basic() {
    tcs34725_write_reg(TCS_ENABLE, 0x01); 
    sleep_ms(3);
    tcs34725_write_reg(TCS_ENABLE, 0x03); 
    tcs34725_write_reg(TCS_ATIME, 0xD5);
    tcs34725_write_reg(TCS_CONTROL, 0x01);
}

// ==========================================
// PROGRAMA PRINCIPAL
// ==========================================
int main() {
    stdio_init_all();
    
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Sistema Completo (Dual ToF + Cor + IMU) Iniciado! ---\n");

    // 1. INICIALIZAÇÃO I2C1 (Sensor de Cor TCS34725)
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(COLOR_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(COLOR_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(COLOR_I2C_SDA_PIN);
    gpio_pull_up(COLOR_I2C_SCL_PIN);
    tcs34725_init_basic();
    printf("Sensor de Cor OK!\n");

    // 2. INICIALIZAÇÃO SPI0 (Sensor IMU)
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    
    sleep_ms(50);
    icm_write_register(0x06, 0x01); // Tira do Sleep e seleciona clock[cite: 3]
    sleep_ms(50);
    printf("Sensor IMU OK!\n");

    // 3. INICIALIZAÇÃO I2C0 (Sensores VL53L0X)
    gpio_pull_up(TOF_I2C_SDA_PIN);
    gpio_pull_up(TOF_I2C_SCL_PIN);
    I2C i2c_bus(i2c0, TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); 

    // 4. CONFIGURAÇÃO XSHUT
    gpio_init(XSHUT_SENSOR_1);
    gpio_set_dir(XSHUT_SENSOR_1, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_1, 0); 

    gpio_init(XSHUT_SENSOR_2);
    gpio_set_dir(XSHUT_SENSOR_2, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_2, 0); 

    printf("ToFs desligados. A aguardar estabilizacao...\n");
    sleep_ms(20); 

    // 5. INICIA SENSOR 1
    gpio_put(XSHUT_SENSOR_1, 1);
    sleep_ms(15); 
    sensor1.setBus(&i2c_bus);
    if (sensor1.init()) {
        sensor1.setAddress(ENDERECO_S1);
        configurarLongoAlcance(&sensor1);
        printf("Sensor ToF 1 OK! Novo endereco: 0x%02X\n", ENDERECO_S1);
    } else {
        printf("FALHA no Sensor ToF 1!\n");
    }

    // 6. INICIA SENSOR 2
    gpio_put(XSHUT_SENSOR_2, 1);
    sleep_ms(15); 
    sensor2.setBus(&i2c_bus);
    if (sensor2.init()) {
        sensor2.setAddress(ENDERECO_S2);
        configurarLongoAlcance(&sensor2);
        printf("Sensor ToF 2 OK! Novo endereco: 0x%02X\n", ENDERECO_S2);
    } else {
        printf("FALHA no Sensor ToF 2!\n");
    }

    sensor1.startContinuous();
    sensor2.startContinuous();

    // VARIÁVEIS DE ESTADO DO LOOP
    uint16_t dist_s1 = 1200;
    uint16_t dist_s2 = 1200;
    
    uint8_t color_data[8];
    uint16_t c = 0, r = 0, g = 0, b = 0;
    
    uint8_t imu_data[12];
    int16_t accel_x = 0, accel_y = 0, accel_z = 0;
    int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;

    uint32_t last_color_read = 0;
    uint32_t last_imu_read = 0;
    uint32_t last_terminal_print = 0;

    printf("\nA iniciar loop de controle principal...\n");

    // 7. LOOP PRINCIPAL (Arquitetura não-bloqueante)
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // --- LEITURA ToF (Contínua / ~33ms) ---
        uint16_t leitura_s1 = sensor1.readRangeContinuousMillimeters();
        if (!sensor1.timeoutOccurred()) {
            if (leitura_s1 == 8191 || leitura_s1 > 1200) dist_s1 = 1200;
            else if (leitura_s1 > 20) dist_s1 = leitura_s1;
        }

        uint16_t leitura_s2 = sensor2.readRangeContinuousMillimeters();
        if (!sensor2.timeoutOccurred()) {
            if (leitura_s2 == 8191 || leitura_s2 > 1200) dist_s2 = 1200;
            else if (leitura_s2 > 20) dist_s2 = leitura_s2;
        }

        // --- LEITURA SENSOR DE COR (A cada 110ms) ---
        if (current_time - last_color_read >= 110) {
            uint8_t reg = TCS_CDATAL;
            i2c_write_blocking(i2c1, TCS34725_ADDR, &reg, 1, true); 
            i2c_read_blocking(i2c1, TCS34725_ADDR, color_data, 8, false);

            c = (color_data[1] << 8) | color_data[0];
            r = (color_data[3] << 8) | color_data[2];
            g = (color_data[5] << 8) | color_data[4];
            b = (color_data[7] << 8) | color_data[6];

            last_color_read = current_time;
        }

        // --- LEITURA IMU SPI (A cada 10ms para malhas de controle de alta velocidade) ---
        if (current_time - last_imu_read >= 10) {
            icm_read_registers(0x2D, imu_data, 12);
            
            accel_x = (imu_data[0] << 8) | imu_data[1];
            accel_y = (imu_data[2] << 8) | imu_data[3];
            accel_z = (imu_data[4] << 8) | imu_data[5];
            
            gyro_x = (imu_data[6] << 8) | imu_data[7];
            gyro_y = (imu_data[8] << 8) | imu_data[9];
            gyro_z = (imu_data[10] << 8) | imu_data[11];
            
            last_imu_read = current_time;
        }

        // --- ATUALIZAÇÃO DO TERMINAL (A cada 100ms para manter a leitura humana possível) ---
        if (current_time - last_terminal_print >= 100) {
            printf("\033[H\033[J"); // Limpa o terminal (opcional) para melhor visualização contínua
            printf("=== DADOS DOS SENSORES ===\n");
            printf("ToF1 : %4d mm  |  ToF2 : %4d mm\n", dist_s1, dist_s2);
            printf("Cor  : C:%5u  R:%5u  G:%5u  B:%5u\n", c, r, g, b);
            printf("IMU  : AX:%6d  AY:%6d  AZ:%6d\n", accel_x, accel_y, accel_z);
            printf("       GX:%6d  GY:%6d  GZ:%6d\n", gyro_x, gyro_y, gyro_z);
            printf("==========================\n");
            
            last_terminal_print = current_time;
        }
    }

    return 0;
}