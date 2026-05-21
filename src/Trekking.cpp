#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "I2C.h"
#include "VL53L0X.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define MUX_ADDR 0x70
#define NUM_SENSORES 3 

// Quantos erros seguidos o sensor pode dar antes do sistema reiniciar ele
#define LIMITE_ERROS_CONSECUTIVOS 5 

void setMuxChannel(I2C* bus, uint8_t canal) {
    if (canal >= NUM_SENSORES) return; 
    bus->beginTransmission(MUX_ADDR);
    bus->write(1 << canal);
    bus->endTransmission();
}

void configurarLongoAlcance(VL53L0X* sensor) {
    sensor->setTimeout(500);
    sensor->setSignalRateLimit(0.1);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor->setMeasurementTimingBudget(200000); 
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Sistema Multi-ToF com Auto-Recuperacao Iniciado! ---\n");

    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    I2C i2c_bus(i2c0, I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(100000); // Mantido em 100kHz para maior estabilidade

    VL53L0X sensores[NUM_SENSORES];
    uint16_t distancias[NUM_SENSORES];
    
    // Novo: Array para contar quantas vezes cada sensor falhou em sequência
    uint8_t erros_consecutivos[NUM_SENSORES] = {0, 0, 0};

    // Inicialização original
    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
        printf("Iniciando S%d... ", i + 1);
        setMuxChannel(&i2c_bus, i);
        sensores[i].setBus(&i2c_bus);
        if (sensores[i].init()) {
            configurarLongoAlcance(&sensores[i]);
            printf("OK\n");
        } else {
            printf("FALHA\n");
            erros_consecutivos[i] = LIMITE_ERROS_CONSECUTIVOS; // Força tentar reiniciar no loop
        }
    }

    printf("Iniciando varredura...\n\n");

    while (true) {
        // 1. Fase de Leitura e Verificação de Saúde
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            setMuxChannel(&i2c_bus, i);
            
            // Tenta ler o sensor
            distancias[i] = sensores[i].readRangeSingleMillimeters();

            // Verifica se houve falha de comunicação (Timeout) ou valor absurdo de erro (8190/8191)
            if (sensores[i].timeoutOccurred() || distancias[i] > 8000) {
                erros_consecutivos[i]++;
            } else {
                // Leitura bem sucedida, zera o contador de erros
                erros_consecutivos[i] = 0; 
            }

            // AUTO-RECUPERAÇÃO: Se passou do limite, tenta ressuscitar
            if (erros_consecutivos[i] >= LIMITE_ERROS_CONSECUTIVOS) {
                printf("\n[ALERTA] Sensor %d travou! Tentando reiniciar... ", i + 1);
                
                // O Mux já está no canal dele, basta tentar o init de novo
                if (sensores[i].init()) {
                    configurarLongoAlcance(&sensores[i]);
                    erros_consecutivos[i] = 0; // Ressuscitou, zera os erros
                    printf("Sucesso! Sensor restaurado.\n");
                } else {
                    printf("Falhou. Tentaremos no proximo ciclo.\n");
                }
            }
        }

        // 2. Fase de Impressão
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            printf("S%d: ", i + 1);
            
            if (erros_consecutivos[i] > 0) {
                 printf("ERRO | "); // Mostra que está falhando antes de reiniciar
            } else if (distancias[i] > 2000) {
                printf(" 000 | "); // Funcionando, mas nada na frente
            } else {
                printf("%4d mm | ", distancias[i]); // Leitura válida
            }
        }
        printf("\n"); // Usa \r para sobrescrever a mesma linha e não poluir a tela
        
        sleep_ms(50);
    }

    return 0;
}