#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "I2C.h"
#include "VL53L0X.h"

// --- DEFINIÇÃO DOS PINOS ---
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define XSHUT_SENSOR_1 15
#define XSHUT_SENSOR_2 7

// --- NOVOS ENDEREÇOS I2C ---
// O endereço padrão de fábrica é 0x29. Vamos usar 0x30 e 0x31.
#define ENDERECO_S1 0x30
#define ENDERECO_S2 0x31

// Instâncias dos sensores
VL53L0X sensor1;
VL53L0X sensor2;

// Função auxiliar para aplicar as configurações de longo alcance
void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(50); // Timeout rápido para não travar o robô
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // Configura para ler a 30Hz (33ms)
    sensor->setMeasurementTimingBudget(50000); 
}

int main() {
    stdio_init_all();
    
    // Aguarda a ligação do Serial Monitor
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Sistema Dual ToF via XSHUT Iniciado! ---\n");

    // 1. INICIALIZAÇÃO DO BARRAMENTO I2C
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    I2C i2c_bus(i2c0, I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(400000); // 400kHz (Fast Mode) para leituras mais rápidas

    // 2. CONFIGURAÇÃO DOS PINOS XSHUT (Desliga todos primeiro)
    gpio_init(XSHUT_SENSOR_1);
    gpio_set_dir(XSHUT_SENSOR_1, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_1, 0); // GND = Desligado

    gpio_init(XSHUT_SENSOR_2);
    gpio_set_dir(XSHUT_SENSOR_2, GPIO_OUT);
    gpio_put(XSHUT_SENSOR_2, 0); // GND = Desligado

    printf("Sensores desligados. A aguardar estabilizacao...\n");
    sleep_ms(20); 

    // 3. ACORDA E CONFIGURA O SENSOR 1
    printf("A iniciar Sensor 1 (GP15)...\n");
    gpio_put(XSHUT_SENSOR_1, 1); // 3.3V = Liga o Sensor 1
    sleep_ms(15); // Dá tempo para o chip arrancar no endereço 0x29

    sensor1.setBus(&i2c_bus);
    if (sensor1.init()) {
        sensor1.setAddress(ENDERECO_S1); // Muda de 0x29 para 0x30
        configurarLongoAlcance(&sensor1);
        printf("Sensor 1 OK! Novo endereco: 0x%02X\n", ENDERECO_S1);
    } else {
        printf("FALHA no Sensor 1!\n");
    }

    // 4. ACORDA E CONFIGURA O SENSOR 2
    printf("A iniciar Sensor 2 (GP7)...\n");
    gpio_put(XSHUT_SENSOR_2, 1); // 3.3V = Liga o Sensor 2
    sleep_ms(15); // Dá tempo para o chip arrancar no endereço 0x29

    sensor2.setBus(&i2c_bus);
    if (sensor2.init()) {
        sensor2.setAddress(ENDERECO_S2); // Muda de 0x29 para 0x31
        configurarLongoAlcance(&sensor2);
        printf("Sensor 2 OK! Novo endereco: 0x%02X\n", ENDERECO_S2);
    } else {
        printf("FALHA no Sensor 2!\n");
    }

    // 5. INICIA O MODO CONTÍNUO (Sem bloquear a CPU)
    printf("\nA iniciar leituras em Modo Continuo...\n");
    sensor1.startContinuous();
    sensor2.startContinuous();

    // Variáveis de memória para reter o estado (Zero-Order Hold)
    uint16_t dist_s1 = 1200;
    uint16_t dist_s2 = 1200;

    // 6. LOOP PRINCIPAL
    while (true) {
        
        // Verifica se o Sensor 1 terminou a leitura de 33ms
        // Algumas bibliotecas usam isRangeComplete(), outras permitem ler direto.
        // Assumindo o readRangeContinuousMillimeters padrão:
        
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

        // Imprime os valores limpos e formatados
        printf("S1: %4d mm | S2: %4d mm\n", dist_s1, dist_s2);
        
        // No código final do robô de trekking, não coloque sleeps aqui!
        // Deixamos um pequeno sleep apenas para o monitor serial não encher a ecrã rápido demais nos testes.
        sleep_ms(30); 
    }

    return 0;
}