#include "pico_spi_hal.h"

// Funções auxiliares para controle do Chip Select (CS)
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0); // Ativa o IMU
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1); // Desativa o IMU
    asm volatile("nop \n nop \n nop");
}

void pico_spi_init(void) {
    // Inicializa o barramento SPI a 7 MHz (limite estável e recomendado do ICM-20948)
    spi_init(SPI_PORT, 7000 * 1000); 

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Configuração exclusiva do pino CS
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // Começa alto (desativado)
}

ICM_20948_Status_e pico_spi_write(uint8_t reg, uint8_t *data, uint32_t len, void *user) {
    // Bit MSB em 0 para operação de escrita
    reg &= 0x7F; 

    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);    // Envia endereço
    spi_write_blocking(SPI_PORT, data, len);  // Envia payload
    cs_deselect();

    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e pico_spi_read(uint8_t reg, uint8_t *buff, uint32_t len, void *user) {
    // Bit MSB em 1 para operação de leitura
    reg |= 0x80; 

    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);    // Envia endereço
    spi_read_blocking(SPI_PORT, 0, buff, len);// Recebe payload
    cs_deselect();

    return ICM_20948_Stat_Ok;
}

void pico_get_icm_serif(ICM_20948_Serif_t *serif) {
    // Liga as funções de leitura e escrita do Pico SDK ao driver genérico
    serif->write = pico_spi_write;
    serif->read  = pico_spi_read;
    serif->user  = NULL;
}