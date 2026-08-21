#include "ICM20948_DMA.hpp"
#include <cstdio>

#define REG_BANK_SEL 0x7F
#define USER_CTRL    0x03
#define FIFO_EN_2    0x67
#define FIFO_COUNTH  0x70
#define FIFO_COUNTL  0x71
#define FIFO_R_W     0x72
#define PWR_MGMT_1   0x06

ICM20948::ICM20948(spi_inst_t* spi_port, uint miso, uint cs, uint sck, uint mosi) 
    : spi(spi_port), cs_pin(cs), dma_active(false), pending_samples(0) {
    
    // SPI elevado para 7 MHz para diminuir o tempo ocupado do barramento
    spi_init(spi, 7000 * 1000);
    spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); 
}

void ICM20948::selectBank(uint8_t bank) {
    uint8_t data[2] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, data, 2);
    gpio_put(cs_pin, 1);
}

void ICM20948::writeRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, buf, 2);
    gpio_put(cs_pin, 1);
}

uint8_t ICM20948::readRegister(uint8_t reg) {
    uint8_t reg_addr = reg | 0x80;
    uint8_t data = 0;
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &reg_addr, 1);
    spi_read_blocking(spi, 0x00, &data, 1);
    gpio_put(cs_pin, 1);
    return data;
}

// Leitura em Burst dos registradores H e L
uint16_t ICM20948::getFIFOCount() {
    uint8_t reg_addr = FIFO_COUNTH | 0x80;
    uint8_t data[2];
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &reg_addr, 1);
    spi_read_blocking(spi, 0x00, data, 2);
    gpio_put(cs_pin, 1);
    return (data[0] << 8) | data[1];
}

void ICM20948::setupDMA() {
    dma_tx = dma_claim_unused_channel(true);
    dma_rx = dma_claim_unused_channel(true);

    c_tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_dreq(&c_tx, spi_get_dreq(spi, true));
    channel_config_set_read_increment(&c_tx, false); 
    channel_config_set_write_increment(&c_tx, false);

    c_rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_dreq(&c_rx, spi_get_dreq(spi, false));
    channel_config_set_read_increment(&c_rx, false);
    channel_config_set_write_increment(&c_rx, true); 
}

void ICM20948::init() {
    setupDMA();

    selectBank(0);
    writeRegister(PWR_MGMT_1, 0x81);
    sleep_ms(50);
    writeRegister(PWR_MGMT_1, 0x01);
    sleep_ms(50);

    writeRegister(USER_CTRL, 0x40); 
    writeRegister(FIFO_EN_2, 0x1E); 

    // Garante que o banco 0 fique selecionado permanentemente 
    // para não precisarmos chamar selectBank() a cada ciclo do loop
    selectBank(0);
}

void ICM20948::startFIFODMARead(int max_samples) {
    if (dma_active) return; // Evita sobreposição se já estiver rodando

    uint16_t bytes_in_fifo = getFIFOCount();
    int available_samples = bytes_in_fifo / 12;
    
    if (available_samples == 0) return;
    
    if (max_samples > MAX_FIFO_SAMPLES) max_samples = MAX_FIFO_SAMPLES;
    pending_samples = available_samples > max_samples ? max_samples : available_samples;
    
    uint16_t bytes_to_read = pending_samples * 12;

    uint8_t reg_addr = FIFO_R_W | 0x80;
    gpio_put(cs_pin, 0); // Mantém o CS baixo até o final da leitura via DMA
    spi_write_blocking(spi, &reg_addr, 1);

    static const uint8_t dummy_byte = 0x00;
    dma_channel_configure(dma_tx, &c_tx,
                          &spi_get_hw(spi)->dr, 
                          &dummy_byte,          
                          bytes_to_read,
                          false);

    dma_channel_configure(dma_rx, &c_rx,
                          dma_rx_buffer, // Buffer pré-alocado
                          &spi_get_hw(spi)->dr, 
                          bytes_to_read,
                          false);

    dma_active = true;
    dma_start_channel_mask((1u << dma_rx) | (1u << dma_tx));
}

int ICM20948::checkAndGetFIFO(IMUData* buffer) {
    if (!dma_active) return 0;
    if (dma_channel_is_busy(dma_rx)) return 0; // A CPU não bloqueia, apenas retorna

    // Finaliza o barramento SPI
    gpio_put(cs_pin, 1);
    dma_active = false;

    // Processamento otimizado (usando as multiplicações)
    for (int i = 0; i < pending_samples; i++) {
        int offset = i * 12;
        
        int16_t ax = (dma_rx_buffer[offset + 0] << 8) | dma_rx_buffer[offset + 1];
        int16_t ay = (dma_rx_buffer[offset + 2] << 8) | dma_rx_buffer[offset + 3];
        int16_t az = (dma_rx_buffer[offset + 4] << 8) | dma_rx_buffer[offset + 5];
        
        int16_t gx = (dma_rx_buffer[offset + 6] << 8) | dma_rx_buffer[offset + 7];
        int16_t gy = (dma_rx_buffer[offset + 8] << 8) | dma_rx_buffer[offset + 9];
        int16_t gz = (dma_rx_buffer[offset + 10] << 8) | dma_rx_buffer[offset + 11];

        buffer[i].accelX = (float)ax * accel_mult;
        buffer[i].accelY = (float)ay * accel_mult;
        buffer[i].accelZ = (float)az * accel_mult;
        
        buffer[i].gyroX  = (float)gx * gyro_mult;
        buffer[i].gyroY  = (float)gy * gyro_mult;
        buffer[i].gyroZ  = (float)gz * gyro_mult;
    }

    return pending_samples;
}