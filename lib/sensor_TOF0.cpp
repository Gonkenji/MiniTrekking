#include "sensor_TOF0.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "I2C.h"
#include "VL53L0X.h"

// Pinos e Endereços
#define TOF_I2C_SDA_PIN 4
#define TOF_I2C_SCL_PIN 5
#define XSHUT_SENSOR_1 12 
#define XSHUT_SENSOR_2 13
#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31

// Registrador de status de interrupção do VL53L0X
#define RESULT_INTERRUPT_STATUS 0x13

static VL53L0X sensor1;
static VL53L0X sensor2;
static I2C i2c_bus(i2c0, TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);

static void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(50);
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // Timing budget mantido em 40ms, mas agora rodará em background
    sensor->setMeasurementTimingBudget(400000); 
}

// Função auxiliar para verificar se o hardware do sensor concluiu a medição
static bool is_data_ready(VL53L0X* sensor) {
    // O VL53L0X sinaliza que um novo dado está pronto quando os 3 primeiros bits não são zero
    uint8_t status = sensor->readReg(RESULT_INTERRUPT_STATUS);
    return (status & 0x07) != 0;
}

void tof_init() {
    gpio_pull_up(TOF_I2C_SDA_PIN);
    gpio_pull_up(TOF_I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); 

    gpio_init(XSHUT_SENSOR_1);
    gpio_set_dir(XSHUT_SENSOR_1, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_1, 0); 

    gpio_init(XSHUT_SENSOR_2);
    gpio_set_dir(XSHUT_SENSOR_2, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_2, 0); 
    sleep_ms(20); 

    // Inicializa Sensor 1
    gpio_put(XSHUT_SENSOR_1, 1);
    sleep_ms(15); 
    sensor1.setBus(&i2c_bus);
    if (sensor1.init()) {
        sensor1.setAddress(ENDERECO_S1);
        configurarLongoAlcance(&sensor1);
    }

    // Inicializa Sensor 2
    gpio_put(XSHUT_SENSOR_2, 1);
    sleep_ms(15); 
    sensor2.setBus(&i2c_bus);
    if (sensor2.init()) {
        sensor2.setAddress(ENDERECO_S2);
        configurarLongoAlcance(&sensor2);
    }

    // Inicia as leituras contínuas em background no hardware dos sensores
    sensor1.startContinuous();
    sensor2.startContinuous();
}

void tof_update(uint16_t &dist_s1, uint16_t &dist_s2) {
    // Apenas extrai o dado se o hardware sinalizar que a medição de 50ms acabou.
    // Isso impede que o processador do Pico fique parado esperando.
    
    if (is_data_ready(&sensor1)) {
        uint16_t leitura_s1 = sensor1.readRangeContinuousMillimeters();
        if (!sensor1.timeoutOccurred()) {
            if (leitura_s1 == 8191 || leitura_s1 > 1000) {
                dist_s1 = 1200;
            } else if (leitura_s1 > 20) {
                dist_s1 = leitura_s1;
            }
        }
    }

    if (is_data_ready(&sensor2)) {
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