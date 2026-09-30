#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "PMW.hpp" 

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Fator de escala calculado com base nos seus testes físicos a 2.5cm de altura
const float PIXELS_POR_MM = 22.0f;

int main() {
    stdio_init_all();
    
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000);
    
    printf("\n============================================\n");
    printf("   Rastreamento em Milimetros - PMW3901\n");
    printf("============================================\n");

    spi_init(SPI_PORT, 2000 * 1000);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    PMW3901 fluxo_optico(SPI_PORT, PIN_CS);

    if (!fluxo_optico.init()) {
        printf("\n[ERRO CRITICO] Sensor PMW3901 nao respondeu.\n");
        while(true) sleep_ms(1000);
    }
    
    printf("Sensor Otimizado! Mova o robo na mesa para ver o deslocamento em mm.\n\n");

    int16_t dX = 0, dY = 0;
    uint8_t squal = 0;

    // Acumuladores agora são float para não perder precisão na divisão
    float pos_x_mm = 0.0f;
    float pos_y_mm = 0.0f;

    while (true) {
        fluxo_optico.readMotionCount(dX, dY, squal);
        
        // Só acumula deslocamento se a qualidade da imagem for minimamente aceitável
        if (squal > 15) { 
            // Converte o delta do ciclo instantaneamente para mm antes de somar
            pos_x_mm += (float)dX / PIXELS_POR_MM;
            pos_y_mm += (float)dY / PIXELS_POR_MM;
        }
        
        // %.2f exibe com duas casas decimais de precisão
        printf("Pos X: %7.2f mm | Pos Y: %7.2f mm | SQUAL: %3d\n", pos_x_mm, pos_y_mm, squal);
        
        sleep_ms(20); // Loop a 50Hz
    }

    return 0;
}