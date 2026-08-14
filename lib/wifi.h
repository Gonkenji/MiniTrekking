#ifndef WIFI_H
#define WIFI_H
#include <stdint.h>

extern bool rota_wifi_grid[40][40];

void wifi_init_ap();
void wifi_atualizar_dados(float x, float y, float yaw, uint16_t dist_l1x, uint16_t dist_s1, uint16_t dist_s2);

#endif // WIFI_H