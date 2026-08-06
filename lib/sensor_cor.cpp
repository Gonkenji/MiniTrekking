#include "sensor_cor.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define TCS34725_ADDR 0x29
#define TCS_COMMAND_BIT 0x80
#define TCS_ENABLE      (0x00 | TCS_COMMAND_BIT)
#define TCS_ATIME       (0x01 | TCS_COMMAND_BIT)
#define TCS_CONTROL     (0x0F | TCS_COMMAND_BIT)
#define TCS_CDATAL      (0x14 | TCS_COMMAND_BIT)
#define cor_I2C_SDA_PIN 26 // roxo
#define cor_I2C_SCL_PIN 27 // branco 

static void tcs34725_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c1, TCS34725_ADDR, buffer, 2, false);
}

void color_init() {
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(cor_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(cor_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(cor_I2C_SDA_PIN);
    gpio_pull_up(cor_I2C_SCL_PIN);
    
    tcs34725_write_reg(TCS_ENABLE, 0x01); 
    sleep_ms(3);
    tcs34725_write_reg(TCS_ENABLE, 0x03); 
    
    // --- OTIMIZAÇÃO PARA ALTA VELOCIDADE ---
    // Opção 1: 0xFF = 2.4 ms (Mais rápido, exige muita iluminação)
    // Opção 2: 0xF6 = 24 ms (Equilíbrio ideal para robôs móveis)
    tcs34725_write_reg(TCS_ATIME, 0xF6); 

    // Aumenta o ganho para 16x (0x02) ou 60x (0x03) para compensar o tempo curto
    tcs34725_write_reg(TCS_CONTROL, 0x02); 
}

void color_update(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b) {
    uint8_t cor_data[8];
    uint8_t reg = TCS_CDATAL;
    
    i2c_write_blocking(i2c1, TCS34725_ADDR, &reg, 1, true); 
    i2c_read_blocking(i2c1, TCS34725_ADDR, cor_data, 8, false);

    c = (cor_data[1] << 8) | cor_data[0];
    r = (cor_data[3] << 8) | cor_data[2];
    g = (cor_data[5] << 8) | cor_data[4];
    b = (cor_data[7] << 8) | cor_data[6];
}

bool detectar_placa_amarela(uint16_t c, uint16_t r, uint16_t g, uint16_t b) {
    // 1. Proteção: Se estiver escuro demais, ignora para não dividir por zero
    if (c < 50) return false;

    // 2. Saturação: O metal, mesmo pintado, pode dar reflexo direto do LED.
    // Se o canal Clear bater no teto do sensor de 16-bits (ex: > 60000), 
    // o sensor está "cego" pelo reflexo.
    if (c > 60000) return false; 

    // 3. Normalização (Converte para porcentagem de 0.0 a 1.0)
    float pct_r = (float)r / c;
    float pct_g = (float)g / c;
    float pct_b = (float)b / c;

    // 4. Lógica do Amarelo
    // Os limiares (0.35, 0.20) precisarão ser ajustados lendo os 
    // dados do seu terminal ao colocar o robô sobre a placa, mas a lógica é:
    bool tem_vermelho = pct_r > 0.35f;
    bool tem_verde    = pct_g > 0.35f;
    bool sem_azul     = pct_b < 0.20f;

    return tem_vermelho && tem_verde && sem_azul;
}