#ifndef I2C_H
#define I2C_H

/**
 * @file I2C.h
 * @brief Interface C++ simples para o periférico I2C do RP2040/RP2350.
 *
 * Esta classe funciona como um pequeno wrapper ao redor das funções do SDK
 * do Pico, oferecendo uma API semelhante ao Wire/Arduino para facilitar
 * exemplos e drivers de sensores.
 */
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cstddef>
#include <cstdint>

/**
 * @class I2C
 * @brief Wrapper minimalista para operações I2C (begin, write/read, etc.).
 */
class I2C {
public:
    /**
     * @brief Constrói o objeto I2C definindo instância e pinos.
     * @param i2c_instance Ponteiro para a instância do periférico (ex.: i2c0).
     * @param sda_pin GPIO usado como SDA.
     * @param scl_pin GPIO usado como SCL.
     */
    I2C(i2c_inst_t *i2c_instance, uint sda_pin, uint scl_pin);
    /** Inicia o periférico e configura GPIOs. */
    void begin();
    /** Desabilita o periférico e retorna GPIOs ao estado padrão. */
    void end();
    /** Ajusta a frequência do clock I2C (Hz). */
    void setClock(uint frequency);

    /** Inicia transmissão para um endereço de 7 bits. */
    void beginTransmission(uint8_t address);
    /** Finaliza a transmissão atual. Retorna status 0 em sucesso. */
    uint8_t endTransmission(void);
    /** Escreve um único byte no buffer de TX. */
    size_t write(uint8_t data);
    /** Escreve um bloco de bytes no buffer de TX. */
    size_t write(const uint8_t *data, size_t quantity);

    /** Solicita leitura de 'quantity' bytes de um endereço. */
    uint8_t requestFrom(uint8_t address, size_t quantity);
    /** Lê um byte do buffer de RX (ou -1 se vazio). */
    int read(void);
    /** Quantidade de bytes disponíveis no buffer de RX. */
    int available(void);

private:
    /** Ponteiro para a instância do periférico I2C. */
    i2c_inst_t *_i2c;
    /** Pino SDA configurado. */
    uint _sda;
    /** Pino SCL configurado. */
    uint _scl;
    /** Endereço de destino da transmissão atual. */
    uint8_t _txAddress;
    /** Buffer de recepção. */
    uint8_t _rxBuffer[32];
    uint8_t _rxBufferIndex;
    uint8_t _rxBufferLength;

    /** Buffer de transmissão. */
    uint8_t _txBuffer[32];
    uint8_t _txBufferIndex;
    uint8_t _txBufferLength;

    /** Indica se há uma transmissão em andamento. */
    bool _transmitting;
};

#endif // I2C_H
