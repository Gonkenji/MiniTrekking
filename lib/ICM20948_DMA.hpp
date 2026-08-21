#ifndef ICM20948_DMA_HPP
#define ICM20948_DMA_HPP

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"

// Estrutura para os dados tratados do IMU
struct IMUData {
    float accelX, accelY, accelZ; 
    float gyroX, gyroY, gyroZ;    
};

class ICM20948 {
public:
    ICM20948(spi_inst_t* spi_port, uint miso, uint cs, uint sck, uint mosi);
    
    void init();
    
    // Inicia a transferência por DMA de forma não-bloqueante
    void startFIFODMARead(int max_samples);
    
    // Checa se o DMA terminou. Se sim, processa os dados e libera o CS
    // Retorna a quantidade de amostras processadas (0 se ainda ocupado)
    int checkAndGetFIFO(IMUData* buffer);

private:
    spi_inst_t* spi;
    uint cs_pin;
    
    int dma_tx;
    int dma_rx;
    dma_channel_config c_tx;
    dma_channel_config c_rx;

    // Estado do DMA e Buffer estático para evitar VLA e estouro de stack
    static const int MAX_FIFO_SAMPLES = 20; 
    uint8_t dma_rx_buffer[MAX_FIFO_SAMPLES * 12];
    bool dma_active;
    int pending_samples;

    void selectBank(uint8_t bank);
    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
    uint16_t getFIFOCount(); // Leitura em burst do contador
    void setupDMA();
    
    // Multiplicadores (substituem as divisões, CPU processa mais rápido)
    const float accel_mult = 1.0f / 16384.0f;
    const float gyro_mult  = 1.0f / 131.0f;
};

#endif // ICM20948_DMA_HPP