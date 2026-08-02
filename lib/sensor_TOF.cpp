#include "sensor_tof.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h> // Necessário para imprimir no terminal os erros
#include "I2C.h"
#include "VL53L0X.h"

// Envolve as bibliotecas em C puro para manter compatibilidade com o compilador C++
extern "C" {
    #include "VL53L1X_types.h"
    #include "VL53L1X_platform.h"
    #include "VL53L1X_calibration.h"
    #include "VL53L1X_api.h"
}

// Pinos e Endereços I2C0
#define TOF_I2C_SDA_PIN 4
#define TOF_I2C_SCL_PIN 5

// Pinos XSHUT
#define XSHUT_SENSOR_1 12 
#define XSHUT_SENSOR_2 13
#define XSHUT_SENSOR_3 11 // Novo sensor VL53L1X

// Endereços personalizados
#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31
#define ENDERECO_S3 0x32

#define RESULT_INTERRUPT_STATUS 0x13

// Instâncias
static VL53L0X sensor1;
static VL53L0X sensor2;
static uint16_t dev_s3 = 0x29; // O VL53L1X sempre acorda no endereço 0x29 (padrão de fábrica)
static I2C i2c_bus(i2c0, TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);

// Funções Auxiliares para o VL53L0X
static void configurarLongoAlcance_L0X(VL53L0X* sensor) {
    sensor->setTimeout(50);
    sensor->setSignalRateLimit(0.1); 
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor->setMeasurementTimingBudget(40000); 
}

static bool is_data_ready_L0X(VL53L0X* sensor) {
    uint8_t status = sensor->readReg(RESULT_INTERRUPT_STATUS);
    return (status & 0x07) != 0;
}

void tof_init() {
    gpio_pull_up(TOF_I2C_SDA_PIN);
    gpio_pull_up(TOF_I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); 

    // 1. DESLIGA TODOS OS SENSORES (Força o reset do barramento)
    gpio_init(XSHUT_SENSOR_1);
    gpio_set_dir(XSHUT_SENSOR_1, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_1, 0); 

    gpio_init(XSHUT_SENSOR_2);
    gpio_set_dir(XSHUT_SENSOR_2, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_2, 0); 

    gpio_init(XSHUT_SENSOR_3);
    gpio_set_dir(XSHUT_SENSOR_3, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_3, 0); 

    sleep_ms(20); 

    // 2. INICIA VL53L0X (SENSOR 1)
    gpio_put(XSHUT_SENSOR_1, 1);
    sleep_ms(15); 
    sensor1.setBus(&i2c_bus);
    if (sensor1.init()) {
        sensor1.setAddress(ENDERECO_S1);
        configurarLongoAlcance_L0X(&sensor1);
        sensor1.startContinuous();
    } else {
        printf("FALHA CRITICA no Sensor ToF 1 (L0X)!\n");
    }

    // 3. INICIA VL53L0X (SENSOR 2)
    gpio_put(XSHUT_SENSOR_2, 1);
    sleep_ms(15); 
    sensor2.setBus(&i2c_bus);
    if (sensor2.init()) {
        sensor2.setAddress(ENDERECO_S2);
        configurarLongoAlcance_L0X(&sensor2);
        sensor2.startContinuous();
    } else {
        printf("FALHA CRITICA no Sensor ToF 2 (L0X)!\n");
    }

    // 4. INICIA O NOVO VL53L1X (SENSOR 3)
    gpio_put(XSHUT_SENSOR_3, 1);
    sleep_ms(15); 
    
    // Tenta inicializar e vincula ao driver
    if (VL53L1X_I2C_Init(dev_s3, i2c0) == 0) {
        // Troca o endereço na memória RAM do sensor
        VL53L1X_SetI2CAddress(dev_s3, ENDERECO_S3 << 1); 
        dev_s3 = ENDERECO_S3; // Atualiza a variável para os próximos comandos
        
        // Rotina de configuração padrão do VL53L1X
        VL53L1X_SensorInit(dev_s3);
        VL53L1X_SetDistanceMode(dev_s3, 2); // 2 = Long Range (Até 4 metros)
        VL53L1X_SetTimingBudgetInMs(dev_s3, 40); // Sincronizado com os 40ms dos L0X
        VL53L1X_SetInterMeasurementInMs(dev_s3, 40); 
        VL53L1X_StartRanging(dev_s3);
        
        printf("Sensor ToF 3 (L1X) OK! Novo endereco: 0x%02X\n", ENDERECO_S3);
    } else {
        printf("FALHA CRITICA no Sensor ToF 3 (L1X)!\n");
    }
}

void tof_update(uint16_t &dist_s1, uint16_t &dist_s2, uint16_t &dist_s3) {
    // --- ATUALIZA O SENSOR 1 (VL53L0X) ---
    if (is_data_ready_L0X(&sensor1)) {
        uint16_t leitura_s1 = sensor1.readRangeContinuousMillimeters();
        if (!sensor1.timeoutOccurred()) {
            dist_s1 = (leitura_s1 == 8191 || leitura_s1 > 1200) ? 1200 : leitura_s1;
        }
    }

    // --- ATUALIZA O SENSOR 2 (VL53L0X) ---
    if (is_data_ready_L0X(&sensor2)) {
        uint16_t leitura_s2 = sensor2.readRangeContinuousMillimeters();
        if (!sensor2.timeoutOccurred()) {
            dist_s2 = (leitura_s2 == 8191 || leitura_s2 > 1200) ? 1200 : leitura_s2;
        }
    }

    // --- ATUALIZA O SENSOR 3 (VL53L1X) NÃO-BLOQUEANTE ---
    uint8_t s3_dataReady = 0;
    VL53L1X_CheckForDataReady(dev_s3, &s3_dataReady);
    
    if (s3_dataReady) {
        VL53L1X_Result_t resultado_s3; 
        
        VL53L1X_GetResult(dev_s3, &resultado_s3); 
        VL53L1X_ClearInterrupt(dev_s3); // Obrigatório no L1X para ele iniciar a próxima medição
        
        // Status 0: Totalmente válido
        // Status 4: Fase fora dos limites (Ocorre quando aponta para o infinito/vazio)
        // Status 7: Wrap target (Alvo de baixa reflexividade em longas distâncias)
        if (resultado_s3.status == 0 || resultado_s3.status == 4 || resultado_s3.status == 7) { 
            if (resultado_s3.distance > 20 && resultado_s3.distance < 4000) { 
                dist_s3 = resultado_s3.distance; 
            } else {
                dist_s3 = 4000;
            }
        } else {
            // Em caso de erro extremo de ruído, força o limite máximo de visão
            dist_s3 = 4000;
        }
    }
}