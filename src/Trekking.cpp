#include <stdio.h>
#include <math.h> // Necessário para fabs()
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Endereço I2C padrão do TCS34725
#define TCS34725_ADDR 0x29

// Registradores do sensor (Com o Command Bit 0x80 já aplicado)
#define TCS_COMMAND_BIT 0x80
#define TCS_ENABLE      (0x00 | TCS_COMMAND_BIT)
#define TCS_ATIME       (0x01 | TCS_COMMAND_BIT)
#define TCS_CONTROL     (0x0F | TCS_COMMAND_BIT)
#define TCS_CDATAL      (0x14 | TCS_COMMAND_BIT) // Início dos dados (C, R, G, B)

// Pinos I2C1 (Ajuste conforme a sua montagem física)
#define I2C_SDA_PIN 26
#define I2C_SCL_PIN 27

// Função auxiliar para escrever registradores
void tcs34725_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c1, TCS34725_ADDR, buffer, 2, false);
}

// Inicialização otimizada para o robô em movimento (Alta Velocidade)
void tcs34725_init_high_speed() {
    // 1. Ligar o oscilador (Power ON)
    tcs34725_write_reg(TCS_ENABLE, 0x01); 
    sleep_ms(3); // Aguarda o oscilador estabilizar
    
    // 2. Ativar os conversores ADC de cor (Power ON + ADC Enable)
    tcs34725_write_reg(TCS_ENABLE, 0x03); 
    
    // 3. Tempo de Integração: 0xF6 = 24ms (Captura "fotos" rápidas para não borrar no movimento)
    tcs34725_write_reg(TCS_ATIME, 0xF6);
    
    // 4. Ganho do Amplificador: 0x02 = 16x (Compensa o tempo curto de exposição)
    // Se a leitura ficar muito escura com a saia de E.V.A., mude para 0x03 (60x)
    tcs34725_write_reg(TCS_CONTROL, 0x02);
}

int main() {
    // Inicializa a comunicação serial (USB/UART) para ver os prints
    stdio_init_all();

    // Inicializa o I2C1 em Fast Mode (400 kHz)
    i2c_init(i2c1, 400 * 1000);
    
    // Configura os pinos
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Configura o sensor
    tcs34725_init_high_speed();
    printf("TCS34725 Iniciado. Modo Alta Velocidade Ativo.\n");

    uint8_t data[8];
    uint16_t c, r, g, b;

    while (true) {
        // Aponta para o primeiro registrador de dados (Clear LSB)
        uint8_t reg = TCS_CDATAL;
        
        // Escreve sem soltar o barramento (nostop = true) e depois lê os 8 bytes seguidos
        i2c_write_blocking(i2c1, TCS34725_ADDR, &reg, 1, true); 
        i2c_read_blocking(i2c1, TCS34725_ADDR, data, 8, false);

        // Remonta os dados de 16 bits
        c = (data[1] << 8) | data[0];
        r = (data[3] << 8) | data[2];
        g = (data[5] << 8) | data[4];
        b = (data[7] << 8) | data[6];

        // Proteção contra divisão por zero e ruído extremo
        if (c == 0) {
            sleep_ms(25);
            continue; 
        }

        // ==========================================
        // FILTRO 1: Intensidade Total da Luz (C)
        // ==========================================
        // Abaixo desse valor, é grama, sombra ou asfalto preto.
        // *AJUSTE NA PISTA*: Leia a grama com a saia de isolamento e veja o valor máximo de C. 
        // Coloque um número acima do valor da grama e abaixo do valor da placa amarela.
        if (c < 800) {
            printf("Ignorado: Muito escuro (C: %u)\n", c);
            sleep_ms(25);
            continue; 
        }

        // ==========================================
        // CÁLCULO DAS PROPORÇÕES
        // ==========================================
        float p_red = (float)r / c;
        float p_green = (float)g / c;
        float p_blue = (float)b / c;

        // Calcula a distância entre vermelho e verde
        float diff_rg = fabs(p_red - p_green);

        // ==========================================
        // FILTRO 2: Regras do Amarelo
        // ==========================================
        // 1. Vermelho alto (> 35%)
        // 2. Verde alto (> 35%)
        // 3. Azul baixo (< 20%)
        // 4. Vermelho e Verde quase iguais (diferença menor que 10%) para barrar grama
        bool is_yellow = (p_red > 0.35) && 
                         (p_green > 0.35) && 
                         (p_blue < 0.20) && 
                         (diff_rg < 0.10);

        if (is_yellow) {
            // AQUI VOCÊ CHAMA A ROTINA DE PARAR/VIRAR O ROBÔ
            printf("\n========================================\n");
            printf(" => PLACA AMARELA DETECTADA! <=\n");
            printf(" Luz(C): %u | R: %.2f | G: %.2f | B: %.2f\n", c, p_red, p_green, p_blue);
            printf("========================================\n\n");
        } else {
            printf("Piso claro, mas nao e amarelo (C: %u | Diff R-G: %.2f)\n", c, diff_rg);
        }

        // Espera 25ms para não sobrecarregar o barramento e casar com o ATIME (24ms)
        sleep_ms(25); 
    }
    return 0;
}
