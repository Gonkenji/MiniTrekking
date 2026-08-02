#pragma once
#include <cstdint>

void tof_init();
void tof_update(uint16_t &dist_s1, uint16_t &dist_s2);