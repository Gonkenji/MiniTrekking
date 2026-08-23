#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/critical_section.h"
#include <vector>
#include <utility>

// --- MECANISMOS DE IPC / SINCRONIZAÇÃO ---
extern mutex_t mutex_rota;
extern mutex_t mutex_sensores;
extern critical_section_t cs_estado;

// --- DADOS COMPARTILHADOS ---
// Odometria e Cinemática (Protegidos por cs_estado devido às ISRs)
extern volatile float pos_x_global;
extern volatile float pos_y_global;
extern volatile float yaw_global;

// Sensores e Status de Controle (Protegidos por mutex_sensores)
extern volatile uint16_t dist_l1x_global;
extern volatile uint16_t dist_s1_global;
extern volatile uint16_t dist_s2_global;
extern volatile float erro_dist_global;
extern volatile float erro_ang_global;

// Planejamento de Rota (Protegidos por mutex_rota)
extern std::vector<std::pair<int, int>> rota_atual_global;
extern volatile bool flag_recalcular_rota;

// Matrizes Globais
#include "mapeamento.h"
extern bool rota_wifi_grid[MAP_CELLS][MAP_CELLS];

// Assinaturas
void init_shared_sync();
void core1_main(); // Ponto de entrada do Core 1

#endif // SHARED_DATA_H