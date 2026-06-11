#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Definição dos pinos SPI mapeados para a Pico 2W
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Funções para controlar o pino CS (Chip Select)
static inline void cs_select() {
    asm volatile("nop \n nop \n nop"); // Atraso de alguns ciclos
    gpio_put(PIN_CS, 0);               // CS em LOW ativa o sensor
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);               // CS em HIGH desativa o sensor
    asm volatile("nop \n nop \n nop");
}

// Função para ler registradores do IMU via SPI
void icm_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    // No modo SPI, o bit mais significativo (MSB) do endereço deve ser 1 para operação de leitura
    reg |= 0x80; 
    
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);     // Envia o endereço do registrador
    spi_read_blocking(SPI_PORT, 0, buf, len);  // Lê a resposta do sensor
    cs_deselect();
}

int main() {
    stdio_init_all(); // Inicializa a saída padrão (USB/UART)

    // Inicializa o barramento SPI a 1 MHz (frequência segura para testes iniciais)
    spi_init(SPI_PORT, 1000 * 1000);
    
    // Configura os pinos de dados e clock para a função SPI
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Configura o pino CS como saída comum de GPIO
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Garante que começa desativado (HIGH)

    sleep_ms(1000); // Aguarda a estabilização da energia do sensor

    printf("Iniciando teste SPI com GY-ICM20948...\n");

    while (true) {
        uint8_t who_am_i;
        
        // Lê o registrador WHO_AM_I (endereço 0x00 no Bank 0 do ICM-20948)
        icm_read_registers(0x00, &who_am_i, 1);
        
        // O valor retornado esperado para o ICM-20948 é 0xEA
        printf("Registrador WHO_AM_I: 0x%02X\n", who_am_i);
        
        sleep_ms(1000);
    }
    
    return 0;
}