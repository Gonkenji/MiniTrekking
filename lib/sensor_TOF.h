#pragma once
#include <cstdint>

void tof_init();
// Atualizado para receber as variáveis dos três sensores simultaneamente
void tof_update(uint16_t &dist_l1x, uint16_t &dist_s1, uint16_t &dist_s2);