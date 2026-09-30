#ifndef PMW3901_HPP
#define PMW3901_HPP

#include "pico/stdlib.h"
#include "hardware/spi.h"

class PMW3901 {
public:
    // Construtor: recebe a porta SPI e o pino do Chip Select (CS)
    PMW3901(spi_inst_t* spi_port, uint cs_pin);

    // Inicializa o sensor, configura pinos e aplica a matriz de otimização
    bool init();

    // Lê os deslocamentos brutos e a qualidade da superfície (foco)
    void readMotionCount(int16_t &deltaX, int16_t &deltaY, uint8_t &squal);

private:
    spi_inst_t* _spi;
    uint _cs_pin;

    void cs_select();
    void cs_deselect();
    
    void writeRegister(uint8_t reg, uint8_t data);
    uint8_t readRegister(uint8_t reg);
    
    void sensorInit();
};

#endif // PMW3901_HPP