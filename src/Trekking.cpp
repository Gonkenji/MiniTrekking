#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Endereço I2C padrão do TCS34725
#define TCS34725_ADDR 0x29

// Registradores do sensor (com Command Bit 0x80)
#define TCS_COMMAND_BIT 0x80
#define TCS_ENABLE      (0x00 | TCS_COMMAND_BIT)
#define TCS_ATIME       (0x01 | TCS_COMMAND_BIT)
#define TCS_CONTROL     (0x0F | TCS_COMMAND_BIT)
#define TCS_CDATAL      (0x14 | TCS_COMMAND_BIT) // Início dos dados

// Pinos I2C1 do Pico 2 W
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27

// Função para escrever nos registradores
void tcs34725_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c1, TCS34725_ADDR, buffer, 2, false);
}

// Configuração básica para teste
void tcs34725_init_basic() {
    tcs34725_write_reg(TCS_ENABLE, 0x01); // Liga o oscilador
    sleep_ms(3);
    tcs34725_write_reg(TCS_ENABLE, 0x03); // Ativa o ADC de cor
    
    // Configuração padrão: 101ms de tempo de integração e ganho 4x
    tcs34725_write_reg(TCS_ATIME, 0xD5);
    tcs34725_write_reg(TCS_CONTROL, 0x01);
}

int main() {
    stdio_init_all();

    // Inicializa o I2C1 a 400kHz
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    tcs34725_init_basic();
    printf("Teste Base do TCS34725 Iniciado.\n");

    uint8_t data[8];
    uint16_t c, r, g, b;

    while (true) {
        // Aponta para o primeiro registrador
        uint8_t reg = TCS_CDATAL;
        
        // Pede os 8 bytes de dados
        i2c_write_blocking(i2c1, TCS34725_ADDR, &reg, 1, true); 
        i2c_read_blocking(i2c1, TCS34725_ADDR, data, 8, false);

        // Junta os bytes (LSB e MSB) para formar inteiros de 16 bits
        c = (data[1] << 8) | data[0];
        r = (data[3] << 8) | data[2];
        g = (data[5] << 8) | data[4];
        b = (data[7] << 8) | data[6];

        // Imprime os valores brutos no terminal
        printf("RAW -> C: %5u | R: %5u | G: %5u | B: %5u\n", c, r, g, b);

        // Aguarda 110ms (um pouco mais que os 101ms do sensor) antes da próxima leitura
        sleep_ms(110); 
    }
    
    return 0;
}