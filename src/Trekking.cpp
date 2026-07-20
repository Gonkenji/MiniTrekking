#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "I2C.h"
#include "VL53L0X.h"

extern "C" {
    #include "pico_spi_hal.h"
}

// --- DEFINIÇÃO DOS PINOS E ENDEREÇOS (ToF) ---
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define XSHUT_SENSOR_1 15
#define XSHUT_SENSOR_2 7

#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31

// Instâncias dos sensores ToF
VL53L0X sensor1;
VL53L0X sensor2;

// Função auxiliar para aplicar as configurações de longo alcance (ToF)
void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(50); // Timeout rápido
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor->setMeasurementTimingBudget(50000); // 33ms (30Hz)
}

int main() {
    stdio_init_all();
    
    // Aguarda a conexão do Serial Monitor via USB
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Iniciando Sistema Integrado (IMU + Dual ToF) ---\n");

    // ==========================================
    // 1. INICIALIZAÇÃO E CONFIGURAÇÃO DA IMU
    // ==========================================
    pico_spi_init();
    sleep_ms(100);

    ICM_20948_Serif_t pico_serif;
    pico_get_icm_serif(&pico_serif);

    ICM_20948_Device_t imu;
    imu._serif = &pico_serif;

    ICM_20948_init_struct(&imu);

    printf("Testando comunicacao com o ICM-20948...\n");
    ICM_20948_Status_e status = ICM_20948_check_id(&imu);
    if (status != ICM_20948_Stat_Ok) {
        printf("ERRO CRITICO: ICM-20948 nao encontrado no barramento SPI!\n");
        while (true) { sleep_ms(1000); } 
    }
    printf("ICM-20948 detectado com sucesso!\n");

    ICM_20948_sw_reset(&imu);
    sleep_ms(50);
    ICM_20948_sleep(&imu, false);
    ICM_20948_low_power(&imu, false);


    // ==========================================
    // 2. INICIALIZAÇÃO E CONFIGURAÇÃO DOS TOF
    // ==========================================
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    I2C i2c_bus(i2c0, I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); // 400kHz (Fast Mode)

    // Configuração dos pinos XSHUT (Desliga todos primeiro)
    gpio_init(XSHUT_SENSOR_1);
    gpio_set_dir(XSHUT_SENSOR_1, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_1, 0);

    gpio_init(XSHUT_SENSOR_2);
    gpio_set_dir(XSHUT_SENSOR_2, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_2, 0);

    printf("Sensores ToF desligados. Aguardando estabilizacao...\n");
    sleep_ms(20); 

    // Acorda e configura o Sensor 1
    printf("Iniciando Sensor ToF 1 (GP15)...\n");
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

    // Acorda e configura o Sensor 2
    printf("Iniciando Sensor ToF 2 (GP7)...\n");
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

    // Inicia leituras contínuas dos ToF
    printf("\nTodos os sensores prontos. Iniciando leituras unificadas...\n\n");
    sensor1.startContinuous();
    sensor2.startContinuous();

    uint16_t dist_s1 = 1200;
    uint16_t dist_s2 = 1200;


    // ==========================================
    // 3. LOOP PRINCIPAL
    // ==========================================
    while (true) {
        
        // --- 3.1. Leitura da IMU ---
        ICM_20948_AGMT_t imu_data; 
        status = ICM_20948_get_agmt(&imu, &imu_data);

        // --- 3.2. Leitura do ToF 1 ---
        uint16_t leitura_s1 = sensor1.readRangeContinuousMillimeters();
        if (!sensor1.timeoutOccurred()) {
            if (leitura_s1 == 8191 || leitura_s1 > 1200) dist_s1 = 1200;
            else if (leitura_s1 > 20) dist_s1 = leitura_s1;
        }

        // --- 3.3. Leitura do ToF 2 ---
        uint16_t leitura_s2 = sensor2.readRangeContinuousMillimeters();
        if (!sensor2.timeoutOccurred()) {
            if (leitura_s2 == 8191 || leitura_s2 > 1200) dist_s2 = 1200;
            else if (leitura_s2 > 20) dist_s2 = leitura_s2;
        }

        // --- 3.4. Impressão dos Dados ---
        if (status == ICM_20948_Stat_Ok) {
            printf("IMU -> Acel[X: %6d, Y: %6d, Z: %6d] | Giro[X: %6d, Y: %6d, Z: %6d]  ||  ",
                   imu_data.acc.axes.x, imu_data.acc.axes.y, imu_data.acc.axes.z,
                   imu_data.gyr.axes.x, imu_data.gyr.axes.y, imu_data.gyr.axes.z);
        } else {
            printf("IMU -> Erro na leitura dos dados  ||  ");
        }

        printf("ToF -> S1: %4d mm | S2: %4d mm\n", dist_s1, dist_s2);
        
        sleep_ms(50); 
    }

    return 0;
}