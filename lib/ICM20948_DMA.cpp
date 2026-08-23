#include "ICM20948_DMA.hpp"
#include <cstdio>
#include <cmath>

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

void ICM20948::readRegisters(uint8_t reg, uint8_t* buf, uint16_t len) {
    uint8_t reg_addr = reg | 0x80;
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, &reg_addr, 1);
    spi_read_blocking(spi, 0x00, buf, len);
    gpio_put(cs_pin, 1);
}

void ICM20948::calibrate() {
    int32_t soma_ax = 0, soma_ay = 0, soma_az = 0;
    int32_t soma_gx = 0, soma_gy = 0, soma_gz = 0; 
    const int amostras = 500;
    uint8_t buffer[12];

    selectBank(0); 

    for (int i = 0; i < amostras; i++) {
        // Lê Acelerômetro e Giroscópio (0x2D a 0x38)
        readRegisters(0x2D, buffer, 12);
        
        soma_ax += (int16_t)((buffer[0] << 8) | buffer[1]);
        soma_ay += (int16_t)((buffer[2] << 8) | buffer[3]);
        soma_az += (int16_t)((buffer[4] << 8) | buffer[5]);
        
        soma_gx += (int16_t)((buffer[6] << 8) | buffer[7]);
        soma_gy += (int16_t)((buffer[8] << 8) | buffer[9]);
        soma_gz += (int16_t)((buffer[10] << 8) | buffer[11]);
        sleep_ms(2);
    }
    
    offset_gx = (float)soma_gx / amostras;
    offset_gy = (float)soma_gy / amostras;
    offset_gz = (float)soma_gz / amostras;

    float ax_med = (float)soma_ax / amostras;
    float ay_med = (float)soma_ay / amostras;
    float az_med = (float)soma_az / amostras;

    float roll_rad = atan2(ay_med, az_med);
    float pitch_rad = atan2(-ax_med, sqrt(ay_med*ay_med + az_med*az_med));

    const_sin_r = sin(roll_rad);
    const_cos_r = cos(roll_rad);
    const_cos_p = cos(pitch_rad);
}

int ICM20948::checkAndGetFIFO(IMUData* buffer) {
    if (!dma_active) return 0;
    if (dma_channel_is_busy(dma_rx)) return 0; // A CPU não bloqueia, apenas retorna

    // Finaliza o barramento SPI
    gpio_put(cs_pin, 1);
    dma_active = false;

    // Processamento otimizado (usando as multiplicações)
    // Substitua a lógica de conversão dentro do for de checkAndGetFIFO():
    for (int i = 0; i < pending_samples; i++) {
        int offset = i * 12;
        
        int16_t raw_ax = (dma_rx_buffer[offset + 0] << 8) | dma_rx_buffer[offset + 1];
        int16_t raw_ay = (dma_rx_buffer[offset + 2] << 8) | dma_rx_buffer[offset + 3];
        int16_t raw_az = (dma_rx_buffer[offset + 4] << 8) | dma_rx_buffer[offset + 5];
        
        int16_t raw_gx = (dma_rx_buffer[offset + 6] << 8) | dma_rx_buffer[offset + 7];
        int16_t raw_gy = (dma_rx_buffer[offset + 8] << 8) | dma_rx_buffer[offset + 9];
        int16_t raw_gz = (dma_rx_buffer[offset + 10] << 8) | dma_rx_buffer[offset + 11];
 
        // Aplica escala no acelerômetro
        buffer[i].accelX = (float)raw_ax * accel_mult;
        buffer[i].accelY = (float)raw_ay * accel_mult;
        buffer[i].accelZ = (float)raw_az * accel_mult;
        
        // Aplica offset e escala no giroscópio
        float gx = (raw_gx - offset_gx) * gyro_mult;
        float gy = (raw_gy - offset_gy) * gyro_mult;
        float gz = (raw_gz - offset_gz) * gyro_mult;

        buffer[i].gyroX = gx;
        buffer[i].gyroY = gy;
        
        // Aplica compensação de Tilt para o eixo Z (Yaw do mundo), idêntico ao SPI
        if (fabs(const_cos_p) > 0.01f) {
            buffer[i].gyroZ = gy * (const_sin_r / const_cos_p) + gz * (const_cos_r / const_cos_p);
        } else {
            buffer[i].gyroZ = gz;
        }
    }

    return pending_samples;
}