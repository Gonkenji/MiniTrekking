#include "I2C.h"

I2C::I2C(i2c_inst_t *i2c_instance, uint sda_pin, uint scl_pin) {
    _i2c = i2c_instance;
    _sda = sda_pin;
    _scl = scl_pin;
    _rxBufferIndex = 0;
    _rxBufferLength = 0;
    _txBufferIndex = 0;
    _txBufferLength = 0;
    _transmitting = false;
}

void I2C::begin() {
    i2c_init(_i2c, 100 * 1000); // Default to 100kHz
    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_pull_up(_scl);
}

void I2C::end() {
    i2c_deinit(_i2c);
    gpio_set_function(_sda, GPIO_FUNC_NULL);
    gpio_set_function(_scl, GPIO_FUNC_NULL);
}

void I2C::setClock(uint frequency) {
    i2c_set_baudrate(_i2c, frequency);
}

void I2C::beginTransmission(uint8_t address) {
    _txAddress = address;
    _txBufferIndex = 0;
    _txBufferLength = 0;
    _transmitting = true;
}

size_t I2C::write(uint8_t data) {
    if (!_transmitting || _txBufferLength >= sizeof(_txBuffer)) {
        return 0;
    }
    _txBuffer[_txBufferLength++] = data;
    return 1;
}

size_t I2C::write(const uint8_t *data, size_t quantity) {
    if (!_transmitting) {
        return 0;
    }
    for (size_t i = 0; i < quantity; ++i) {
        if (!write(data[i])) {
            return i;
        }
    }
    return quantity;
}

uint8_t I2C::endTransmission(void) {
    if (!_transmitting) {
        return 4; // Not in a transmission
    }
    int ret = i2c_write_blocking(_i2c, _txAddress, _txBuffer, _txBufferLength, false);
    _transmitting = false;
    if (ret == PICO_ERROR_GENERIC) {
        return 2; // NACK on address
    } else if (ret < _txBufferLength) {
        return 3; // NACK on data
    }
    return 0; // Success
}


uint8_t I2C::requestFrom(uint8_t address, size_t quantity) {
    // Clamp quantity to the size of the buffer
    if (quantity > sizeof(_rxBuffer)) {
        quantity = sizeof(_rxBuffer);
    }

    // If there's anything in the transmit buffer, send it first.
    // This is typical for setting a register address before reading.
    if (_txBufferLength > 0) {
        int ret = i2c_write_blocking(_i2c, address, _txBuffer, _txBufferLength, true); // nostop = true
        if (ret < _txBufferLength) {
            // Error during write, abort
            _rxBufferLength = 0;
            _rxBufferIndex = 0;
            return 0;
        }
    }

    // Now, read the data
    int bytes_read = i2c_read_blocking(_i2c, address, _rxBuffer, quantity, false);

    if (bytes_read < 0) {
        _rxBufferLength = 0; // Error
    } else {
        _rxBufferLength = bytes_read;
    }
    _rxBufferIndex = 0;
    return _rxBufferLength;
}

int I2C::available(void) {
    return _rxBufferLength - _rxBufferIndex;
}

int I2C::read(void) {
    if (_rxBufferIndex < _rxBufferLength) {
        return _rxBuffer[_rxBufferIndex++];
    }
    return -1;
}
