#ifndef MAPEAMENTO_H
#define MAPEAMENTO_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>

// --- CONFIGURAÇÕES DA ÁREA DE MAPEAMENTO ---
#define MAP_SIZE_MM 2000.0f
#define MAP_RESOLUTION_MM 50.0f 

#define MAP_CELLS ((int)(MAP_SIZE_MM / MAP_RESOLUTION_MM))

#define PESO_OCUPADO 8  
#define PESO_LIVRE -12    

extern int8_t mapa_grid[MAP_CELLS][MAP_CELLS];

// --- FUNÇÃO DE DESLOCAMENTO DE ORIGEM (CENTRALIZADA) ---
inline int coord_to_grid(float coord_mm) {
    // Coloca a origem 0,0 exatemente no centro da matriz
    int idx = (int)((coord_mm + (MAP_SIZE_MM / 2.0f)) / MAP_RESOLUTION_MM);
    return std::max(0, std::min(idx, MAP_CELLS - 1));
}

inline void atualizar_celula_linha(int x0, int y0, int x1, int y1, bool obstaculo_valido) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2;

    while (true) {
        if (x0 == x1 && y0 == y1) {
            if (obstaculo_valido) {
                int novo_valor = mapa_grid[x0][y0] + PESO_OCUPADO;
                mapa_grid[x0][y0] = (novo_valor > 127) ? 127 : (int8_t)novo_valor;
            }
            break;
        }
        
        int novo_valor = mapa_grid[x0][y0] + PESO_LIVRE;
        mapa_grid[x0][y0] = (novo_valor < -127) ? -127 : (int8_t)novo_valor;

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

inline void processar_leitura_tof(float rob_x, float rob_y, float rob_yaw_rad, uint16_t dist_mm, float angulo_sensor_rad, uint16_t sensor_max_range) {
    bool bateu_na_parede = true;
    
    // Se a distância lida for maior que o alcance confiável (ou for um código de erro como 4000)
    if (dist_mm >= sensor_max_range) { 
        bateu_na_parede = false;
        // Limita o traçado de espaço livre a 600mm.
        // Isso limpa a névoa imediatamente na frente do robô sem invadir paredes do outro lado do mapa.
        dist_mm = 600; 
    }

    // Calcula o ângulo global do laser
    float angulo_global_laser = rob_yaw_rad + angulo_sensor_rad;
    
    // Projeção polar para cartesiana
    float obs_x = rob_x + (dist_mm * cosf(angulo_global_laser));
    float obs_y = rob_y + (dist_mm * sinf(angulo_global_laser));

    // Conversão para a matriz do Occupancy Grid
    int grid_rob_x = coord_to_grid(rob_x);
    int grid_rob_y = coord_to_grid(rob_y);
    int grid_obs_x = coord_to_grid(obs_x);
    int grid_obs_y = coord_to_grid(obs_y);

    atualizar_celula_linha(grid_rob_x, grid_rob_y, grid_obs_x, grid_obs_y, bateu_na_parede);
}

#endif // MAPEAMENTO_H