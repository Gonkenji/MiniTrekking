#pragma once
#include <cstdint>

void color_init();
void color_update(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b);
bool detectar_placa_amarela(uint16_t c, uint16_t r, uint16_t g, uint16_t b);