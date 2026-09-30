#include "PMW.hpp"

PMW3901::PMW3901(spi_inst_t* spi_port, uint cs_pin) {
    _spi = spi_port;
    _cs_pin = cs_pin;
}

void PMW3901::cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

void PMW3901::cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(_cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

void PMW3901::writeRegister(uint8_t reg, uint8_t data) {
    reg |= 0x80; // MSB = 1 para escrita
    uint8_t buf[2] = {reg, data};
    cs_select();
    spi_write_blocking(_spi, buf, 2);
    cs_deselect();
    sleep_us(10);
}

uint8_t PMW3901::readRegister(uint8_t reg) {
    uint8_t data;
    reg &= 0x7F; // MSB = 0 para leitura
    cs_select();
    spi_write_blocking(_spi, &reg, 1);
    sleep_us(5); // tSRAD
    spi_read_blocking(_spi, 0x00, &data, 1);
    cs_deselect();
    sleep_us(1);
    return data;
}

bool PMW3901::init() {
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, 1);
    sleep_ms(50);

    // Força reset SPI togglando o CS
    cs_select();
    sleep_ms(1);
    cs_deselect();
    sleep_ms(1);

    // Verifica ID de hardware (O esperado é 0x49)
    uint8_t id = readRegister(0x00);
    if (id != 0x49) {
        return false;
    }

    sensorInit();
    return true;
}

void PMW3901::readMotionCount(int16_t &deltaX, int16_t &deltaY, uint8_t &squal) {
    // Ler o registrador 0x02 trava os dados para leitura sequencial
    readRegister(0x02); 
    
    uint8_t dx_l = readRegister(0x03);
    uint8_t dx_h = readRegister(0x04);
    uint8_t dy_l = readRegister(0x05);
    uint8_t dy_h = readRegister(0x06);
    squal = readRegister(0x07); // <--- AQUI ESTAVA O ERRO, SQUAL É 0x07!

    deltaX = (int16_t)((dx_h << 8) | dx_l);
    deltaY = (int16_t)((dy_h << 8) | dy_l);
}

void PMW3901::sensorInit() {
    // Power on reset
    writeRegister(0x3A, 0x5A);
    sleep_ms(5);

    // Leitura obrigatória para limpar registradores iniciais
    readRegister(0x02);
    readRegister(0x03);
    readRegister(0x04);
    readRegister(0x05);
    readRegister(0x06);
    sleep_ms(1);

    // === MATRIZ DE OTIMIZAÇÃO DA PIXART (Performance Registers) ===
    writeRegister(0x7F, 0x00);
    writeRegister(0x55, 0x01);
    writeRegister(0x50, 0x07);
    writeRegister(0x7f, 0x0e);
    writeRegister(0x43, 0x10);

    uint8_t v = readRegister(0x67);
    if (v & (1 << 7)) {
        writeRegister(0x48, 0x04);
    } else {
        writeRegister(0x48, 0x02);
    }

    writeRegister(0x7F, 0x00);
    writeRegister(0x51, 0x7b);
    writeRegister(0x50, 0x00);
    writeRegister(0x55, 0x00);
    writeRegister(0x7F, 0x0e);
    
    v = readRegister(0x73);
    if (v == 0) {
        uint8_t c1 = readRegister(0x70);
        if (c1 <= 28) c1 = c1 + 14;
        else c1 = c1 + 11;
        if (c1 > 0x3F) c1 = 0x3F;
        
        uint8_t c2 = readRegister(0x71);
        c2 = ((uint16_t)c2 * 45) / 100;
        
        writeRegister(0x7f, 0x00);
        writeRegister(0x61, 0xAD);
        writeRegister(0x51, 0x70);
        writeRegister(0x7f, 0x0e);
        writeRegister(0x70, c1);
        writeRegister(0x71, c2);
    }

    writeRegister(0x7F, 0x00);
    writeRegister(0x61, 0xAD);
    writeRegister(0x7F, 0x03);
    writeRegister(0x40, 0x00);
    writeRegister(0x7F, 0x05);
    writeRegister(0x41, 0xB3);
    writeRegister(0x43, 0xF1);
    writeRegister(0x45, 0x14);
    writeRegister(0x5B, 0x32);
    writeRegister(0x5F, 0x34);
    writeRegister(0x7B, 0x08);
    writeRegister(0x7F, 0x06);
    writeRegister(0x44, 0x1B);
    writeRegister(0x40, 0xBF);
    writeRegister(0x4E, 0x3F);
    writeRegister(0x7F, 0x08);
    writeRegister(0x65, 0x20);
    writeRegister(0x6A, 0x18);
    writeRegister(0x7F, 0x09);
    writeRegister(0x4F, 0xAF);
    writeRegister(0x5F, 0x40);
    writeRegister(0x48, 0x80);
    writeRegister(0x49, 0x80);
    writeRegister(0x57, 0x77);
    writeRegister(0x60, 0x78);
    writeRegister(0x61, 0x78);
    writeRegister(0x62, 0x08);
    writeRegister(0x63, 0x50);
    writeRegister(0x7F, 0x0A);
    writeRegister(0x45, 0x60);
    writeRegister(0x7F, 0x00);
    writeRegister(0x4D, 0x11);
    writeRegister(0x55, 0x80);
    writeRegister(0x74, 0x21);
    writeRegister(0x75, 0x1F);
    writeRegister(0x4A, 0x78);
    writeRegister(0x4B, 0x78);
    writeRegister(0x44, 0x08);
    writeRegister(0x45, 0x50);
    writeRegister(0x64, 0xFF);
    writeRegister(0x65, 0x1F);
    writeRegister(0x7F, 0x14);
    writeRegister(0x65, 0x67);
    writeRegister(0x66, 0x08);
    writeRegister(0x63, 0x70);
    writeRegister(0x7F, 0x15);
    writeRegister(0x48, 0x48);
    writeRegister(0x7F, 0x07);
    writeRegister(0x41, 0x0D);
    writeRegister(0x43, 0x14);
    writeRegister(0x4B, 0x0E);
    writeRegister(0x45, 0x0F);
    writeRegister(0x44, 0x42);
    writeRegister(0x4C, 0x80);
    writeRegister(0x7F, 0x10);
    writeRegister(0x5B, 0x02);
    writeRegister(0x7F, 0x07);
    writeRegister(0x40, 0x41);
    writeRegister(0x70, 0x00);

    sleep_ms(10); 

    writeRegister(0x32, 0x44);
    writeRegister(0x7F, 0x07);
    writeRegister(0x40, 0x40);
    writeRegister(0x7F, 0x06);
    writeRegister(0x62, 0xF0);
    writeRegister(0x63, 0x00);
    writeRegister(0x7F, 0x0D);
    writeRegister(0x48, 0xC0);
    writeRegister(0x6F, 0xD5);
    writeRegister(0x7F, 0x00);
    writeRegister(0x5B, 0xA0);
    writeRegister(0x4E, 0xA8);
    writeRegister(0x5A, 0x50);
    writeRegister(0x40, 0x80);
}