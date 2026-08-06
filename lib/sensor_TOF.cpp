#include "sensor_TOF.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h> 
#include "I2C.h"
#include "VL53L0X.h"
#include "VL53L1X.h"

#define TOF_I2C_SDA_PIN 4
#define TOF_I2C_SCL_PIN 5

// Pinos de Reset (XSHUT)
#define XSHUT_L1X 11 
#define XSHUT_S1 12 
#define XSHUT_S2 13

// Endereços únicos na rede I2C
#define ENDERECO_L1X 0x32
#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31

#define RESULT_INTERRUPT_STATUS 0x13

static VL53L0X sensor1;
static VL53L0X sensor2;
static uint16_t dev_l1x = 0x29; 

static I2C i2c_bus(i2c0, TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);
static uint8_t polaridade_l1x = 0;

// --- Funções Auxiliares VL53L1X ---
static inline bool is_data_ready_L1X(uint16_t dev) {
    uint8_t status;
    VL53L1X_RdByte(dev, 0x0031, &status);
    return (status & 0x01) == polaridade_l1x;
}

// --- Funções Auxiliares VL53L0X ---
static void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(50);
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor->setMeasurementTimingBudget(500000); 
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

    // 1. DESLIGA TODOS OS SENSORES (Reset do barramento para evitar conflito no 0x29)
    gpio_init(XSHUT_L1X);
    gpio_set_dir(XSHUT_L1X, GPIO_OUT);
    gpio_put(XSHUT_L1X, 0); 

    gpio_init(XSHUT_S1);
    gpio_set_dir(XSHUT_S1, GPIO_OUT);
    gpio_put(XSHUT_S1, 0); 

    gpio_init(XSHUT_S2);
    gpio_set_dir(XSHUT_S2, GPIO_OUT);
    gpio_put(XSHUT_S2, 0); 
    sleep_ms(20); 

    // 2. INICIA O VL53L1X
    gpio_put(XSHUT_L1X, 1);
    sleep_ms(15); 
    if (VL53L1X_I2C_Init(dev_l1x, i2c0) == 0) {
        VL53L1X_SetI2CAddress(dev_l1x, ENDERECO_L1X << 1); 
        dev_l1x = ENDERECO_L1X; 
        VL53L1X_SensorInit(dev_l1x);
        VL53L1X_SetDistanceMode(dev_l1x, 2); 
        VL53L1X_SetTimingBudgetInMs(dev_l1x, 40); 
        VL53L1X_SetInterMeasurementInMs(dev_l1x, 45); 
        VL53L1X_GetInterruptPolarity(dev_l1x, &polaridade_l1x);
        VL53L1X_StartRanging(dev_l1x);
        printf("ToF VL53L1X OK! Endereco: 0x%02X\n", ENDERECO_L1X);
    }

    // 3. INICIA O VL53L0X - S1
    gpio_put(XSHUT_S1, 1);
    sleep_ms(15); 
    sensor1.setBus(&i2c_bus);
    if (sensor1.init()) {
        sensor1.setAddress(ENDERECO_S1);
        configurarLongoAlcance(&sensor1);
        sensor1.startContinuous();
        printf("ToF S1 (VL53L0X) OK! Endereco: 0x%02X\n", ENDERECO_S1);
    }

    // 4. INICIA O VL53L0X - S2
    gpio_put(XSHUT_S2, 1);
    sleep_ms(15); 
    sensor2.setBus(&i2c_bus);
    if (sensor2.init()) {
        sensor2.setAddress(ENDERECO_S2);
        configurarLongoAlcance(&sensor2);
        sensor2.startContinuous();
        printf("ToF S2 (VL53L0X) OK! Endereco: 0x%02X\n", ENDERECO_S2);
    }
}

void tof_update(uint16_t &dist_l1x, uint16_t &dist_s1, uint16_t &dist_s2) {
    // --- Leitura VL53L1X ---
    if (is_data_ready_L1X(dev_l1x)) {
        uint16_t leitura_distancia;
        uint8_t status_range;

        VL53L1X_GetDistance(dev_l1x, &leitura_distancia);
        VL53L1X_GetRangeStatus(dev_l1x, &status_range);
        VL53L1X_ClearInterrupt(dev_l1x); 
        
        if (status_range == 0 || status_range == 4 || status_range == 7) { 
            if (leitura_distancia > 20 && leitura_distancia < 4000) { 
                dist_l1x = leitura_distancia; 
            } else {
                dist_l1x = 4000;
            }
        } else {
            dist_l1x = 4000;
        }
    }

    // --- Leitura VL53L0X (S1) ---
    if (is_data_ready_L0X(&sensor1)) {
        uint16_t leitura_s1 = sensor1.readRangeContinuousMillimeters();
        if (!sensor1.timeoutOccurred()) {
            if (leitura_s1 == 8191 || leitura_s1 > 1000) {
                dist_s1 = 1200;
            } else if (leitura_s1 > 20) {
                dist_s1 = leitura_s1;
            }
        }
    }

    // --- Leitura VL53L0X (S2) ---
    if (is_data_ready_L0X(&sensor2)) {
        uint16_t leitura_s2 = sensor2.readRangeContinuousMillimeters();
        if (!sensor2.timeoutOccurred()) {
            if (leitura_s2 == 8191 || leitura_s2 > 1000) {
                dist_s2 = 1200;
            } else if (leitura_s2 > 20) {
                dist_s2 = leitura_s2;
            }
        }
    }
}