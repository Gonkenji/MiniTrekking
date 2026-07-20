#ifndef PICO_SPI_HAL_H
#define PICO_SPI_HAL_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ICM_20948_C.h"

// Configuração dos pinos SPI mapeados (SPI0)
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Inicializa o barramento SPI e os pinos físicos
void pico_spi_init(void);

// Funções compatíveis com a assinatura esperada pela biblioteca
ICM_20948_Status_e pico_spi_write(uint8_t reg, uint8_t *data, uint32_t len, void *user);
ICM_20948_Status_e pico_spi_read(uint8_t reg, uint8_t *buff, uint32_t len, void *user);

// Função para injetar essas rotinas na estrutura da InvenSense
void pico_get_icm_serif(ICM_20948_Serif_t *serif);

#ifdef __cplusplus
}
#endif

#endif // PICO_SPI_HAL_H