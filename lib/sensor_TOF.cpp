#include "sensor_tof.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h> 
#include "I2C.h"
#include "VL53L1X.h"

#define TOF_I2C_SDA_PIN 4
#define TOF_I2C_SCL_PIN 5
#define XSHUT_SENSOR 11 
#define ENDERECO_SENSOR 0x32

static uint16_t dev_sensor = 0x29; 
static I2C i2c_bus(i2c0, TOF_I2C_SDA_PIN, TOF_I2C_SCL_PIN);

// Variável para armazenar a polaridade (evita ler isso via I2C a todo momento)
static uint8_t polaridade_l1x = 0;

// Função hiper-otimizada para checar dados (Equivalente ao antigo is_data_ready_L0X)
static inline bool is_data_ready_L1X(uint16_t dev) {
    uint8_t status;
    // Lê apenas 1 byte (Status da GPIO) para verificar se há nova medição
    VL53L1X_RdByte(dev, 0x0031, &status);
    return (status & 0x01) == polaridade_l1x;
}

void tof_init() {
    gpio_pull_up(TOF_I2C_SDA_PIN);
    gpio_pull_up(TOF_I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); 

    // 1. DESLIGA O SENSOR (Reset do barramento)
    gpio_init(XSHUT_SENSOR);
    gpio_set_dir(XSHUT_SENSOR, GPIO_OUT);
    gpio_put(XSHUT_SENSOR, 0); 
    sleep_ms(20); 

    // 2. INICIA O VL53L1X
    gpio_put(XSHUT_SENSOR, 1);
    sleep_ms(15); 
    
    if (VL53L1X_I2C_Init(dev_sensor, i2c0) == 0) {
        // Aplica o endereço I2C definitivo
        VL53L1X_SetI2CAddress(dev_sensor, ENDERECO_SENSOR << 1); 
        dev_sensor = ENDERECO_SENSOR; 
        
        // Rotina de configuração (Long Range, 40ms)
        VL53L1X_SensorInit(dev_sensor);
        VL53L1X_SetDistanceMode(dev_sensor, 2); 
        VL53L1X_SetTimingBudgetInMs(dev_sensor, 40); 
        VL53L1X_SetInterMeasurementInMs(dev_sensor, 40); 
        
        // CACHE DA POLARIDADE: Lê apenas uma vez aqui e usa a variável global no loop
        VL53L1X_GetInterruptPolarity(dev_sensor, &polaridade_l1x);

        // Dispara o laser para medições contínuas
        VL53L1X_StartRanging(dev_sensor);
        
        printf("Sensor ToF (VL53L1X) OK! Novo endereco: 0x%02X\n", ENDERECO_SENSOR);
    } else {
        printf("FALHA CRITICA no Sensor ToF (VL53L1X)!\n");
    }
}

void tof_update(uint16_t &dist_vl53l1x) {
    // A checagem agora consome o mínimo de tempo de I2C e não trava o loop
    if (is_data_ready_L1X(dev_sensor)) {
        uint16_t leitura_distancia;
        uint8_t status_range;

        // Ao invés da Struct inteira, puxamos cirurgicamente só a distância e o status de erro
        VL53L1X_GetDistance(dev_sensor, &leitura_distancia);
        VL53L1X_GetRangeStatus(dev_sensor, &status_range);
        
        // Obrigatório limpar a flag para ele disparar a próxima leitura
        VL53L1X_ClearInterrupt(dev_sensor); 
        
        // Tratamento de erros do L1X:
        // 0 = Válido
        // 4 = Fase fora dos limites (apontando pro nada/céu)
        // 7 = Wrap target (pouca reflexão)
        if (status_range == 0 || status_range == 4 || status_range == 7) { 
            if (leitura_distancia > 20 && leitura_distancia < 4000) { 
                dist_vl53l1x = leitura_distancia; 
            } else {
                dist_vl53l1x = 4000;
            }
        } else {
            dist_vl53l1x = 4000;
        }
    }
}