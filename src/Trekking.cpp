#include <stdio.h>
#include <cstdint>
#include "pico/stdlib.h"
#include "I2C.h"
#include "VL53L0X.h"

// Definindo os pinos de comunicação (Ajustado para GP4 e GP5)
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

int main() {
    // Inicializa a comunicação USB para exibir os dados no terminal
    stdio_init_all();

    // Pausa de 2 segundos para dar tempo de você abrir o monitor serial
    stdio_init_all();

    // A Pico vai ficar travada neste loop até você abrir o monitor serial
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    printf("\n--- Monitor Serial Conectado! ---\n");
    printf("Iniciando leitura do sensor VL53L0X...\n");
    printf("Iniciando leitura do sensor VL53L0X...\n");

    // 1. Cria a instância do proxy I2C
    // A Pico 2W possui dois controladores I2C internos (i2c0 e i2c1).
    // Os pinos GP4 e GP5 estão ligados fisicamente ao i2c0.
    I2C i2c_bus(i2c0, I2C_SDA_PIN, I2C_SCL_PIN);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_bus.begin();
    i2c_bus.setClock(100000); // Configura para 100kHz (Standard Mode)

    // 2. Cria o objeto do sensor
    VL53L0X sensor;
    
    // 3. O PASSO MAIS IMPORTANTE: Ensina ao sensor qual I2C ele deve usar
    sensor.setBus(&i2c_bus);

    printf("Inicializando o sensor...\n");
    if (!sensor.init()) {
        printf("Falha: Sensor não detectado. Verifique os fios e pinos!\n");
        while (1) { sleep_ms(100); } // Trava aqui em caso de erro
    }
    printf("Sensor inicializado com sucesso!\n");

    // 4. Configurações para estabilidade (Conforme recomendado pelo autor)
    sensor.setTimeout(500);                   // Tempo máximo de espera por leitura
    sensor.setMeasurementTimingBudget(50000); // 50ms para focar o laser (boa precisão)
    sensor.setSignalRateLimit(0.1);           // Melhora o alcance máximo

    // 5. Loop contínuo de leitura
    while (true) {
        uint16_t distancia = sensor.readRangeSingleMillimeters();
        
        if (sensor.timeoutOccurred()) {
            printf("Erro: Timeout na leitura!\n");
        } 
        else if (distancia > 2000) {
            printf("Fora de alcance\n");
        } 
        else {
            printf("Distancia: %d mm\n", distancia);
        }
        
        // Aguarda 100 milissegundos antes do próximo tiro do laser
        sleep_ms(100); 
    }

    return 0;
}