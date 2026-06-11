#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

// Função para LER registradores
void icm_read_registers(uint8_t reg, uint8_t *buf, uint16_t len) {
    reg |= 0x80; // MSB = 1 para leitura
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
}

// Função para ESCREVER em registradores
void icm_write_register(uint8_t reg, uint8_t data) {
    reg &= 0x7F; // MSB = 0 para escrita
    uint8_t buf[2] = {reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

int main() {
    stdio_init_all();

    // Inicializa o barramento SPI
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    sleep_ms(1000); 

    // --- CONFIGURAÇÃO INICIAL DO SENSOR ---
    // Registrador PWR_MGMT_1 (0x06)
    // Escrever 0x01 tira do modo Sleep e seleciona o melhor clock disponível
    icm_write_register(0x06, 0x01);
    sleep_ms(50); // Aguarda estabilizar

    printf("Iniciando leitura de dados do IMU...\n");

    // --- LOOP PRINCIPAL ---
    while (true) {
        uint8_t data[12];
        
        // O endereço do primeiro registrador de dados do acelerômetro é 0x2D (ACCEL_XOUT_H)
        // Vamos puxar 12 bytes em sequência: 6 do acelerômetro (X, Y, Z) e 6 do giroscópio (X, Y, Z)
        icm_read_registers(0x2D, data, 12);
        
        // Juntando os bytes High (H) e Low (L) em inteiros de 16 bits com sinal (int16_t)
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        
        int16_t gyro_x = (data[6] << 8) | data[7];
        int16_t gyro_y = (data[8] << 8) | data[9];
        int16_t gyro_z = (data[10] << 8) | data[11];
        
        // Imprime os valores na tela
        printf("Aceleração [X:%6d, Y:%6d, Z:%6d] | Giroscópio [X:%6d, Y:%6d, Z:%6d]\n", 
               accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
        
        sleep_ms(100); // Aguarda 100 milissegundos para não flodar o terminal (10x por segundo)
    }
    
    return 0;
}