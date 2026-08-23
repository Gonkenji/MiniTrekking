#include <cstring>
#include "pico/stdlib.h"
#include "shared_data.h"
#include "mapeamento.h"
#include "navegacao.h"
#include "wifi.h"

void core1_main() {
    // Inicialização da pilha de rede via Background architecture
    wifi_init_ap();

    uint32_t last_map_update = 0;
    float poi_x = 700.0f; 
    float poi_y = -500.0f;

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // 1. ATUALIZAÇÃO DO MAPA (A cada 100ms)
        if (current_time - last_map_update >= 100) {
            
            // Snapshot seguro da Cinemática e Sensores
            critical_section_enter_blocking(&cs_estado);
            float rob_x = pos_x_global;
            float rob_y = pos_y_global;
            float rob_yaw = yaw_global;
            critical_section_exit(&cs_estado);

            mutex_enter_blocking(&mutex_sensores);
            uint16_t tof_frente = dist_l1x_global;
            uint16_t tof_esq = dist_s1_global;
            uint16_t tof_dir = dist_s2_global;
            mutex_exit(&mutex_sensores);

            float yaw_rad = rob_yaw * ((float)M_PI / 180.0f);
            float offsets_l1x[3] = {-0.131f, 0.0f, 0.131f}; 
            float offsets_l0x[3] = {-0.218f, 0.0f, 0.218f}; 

            // Raycasting
            if (tof_frente > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(rob_x, rob_y, yaw_rad, tof_frente, offsets_l1x[i], 3000);
                }
            }
            if (tof_esq > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(rob_x, rob_y, yaw_rad, tof_esq, ((float)M_PI / 6.0f) + offsets_l0x[i], 400);
                }
            }
            if (tof_dir > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(rob_x, rob_y, yaw_rad, tof_dir, -(float)M_PI / 6.0f + offsets_l0x[i], 400);
                }
            }

            wifi_atualizar_dados(rob_x, rob_y, rob_yaw, tof_frente, tof_esq, tof_dir);

            // Verifica se a rota antiga foi obstruída pelas novas leituras
            mutex_enter_blocking(&mutex_rota);
            if (!rota_atual_global.empty() && verificar_rota_bloqueada(rota_atual_global)) {
                flag_recalcular_rota = true; 
            }
            mutex_exit(&mutex_rota);

            last_map_update = current_time;
        }

        // 2. PLANEJADOR A* (Acionado por flag)
        if (flag_recalcular_rota) {
            // Snapshot para o A* não travar mutexes na busca
            critical_section_enter_blocking(&cs_estado);
            int grid_start_x = coord_to_grid(pos_x_global);
            int grid_start_y = coord_to_grid(pos_y_global);
            critical_section_exit(&cs_estado);

            int grid_goal_x = coord_to_grid(poi_x);
            int grid_goal_y = coord_to_grid(poi_y);
            
            // O A* é pesado, roda fora das zonas críticas
            auto nova_rota = calcular_A_star(grid_start_x, grid_start_y, grid_goal_x, grid_goal_y);
            
            // Bloqueia para sobreescrever a rota oficial
            mutex_enter_blocking(&mutex_rota);
            rota_atual_global = nova_rota;
            
            memset(rota_wifi_grid, 0, sizeof(rota_wifi_grid));
            for(auto const& ponto : rota_atual_global) {
                rota_wifi_grid[ponto.first][ponto.second] = true; 
            }
            
            flag_recalcular_rota = false;
            mutex_exit(&mutex_rota);
        }

        // Previne estrangulamento da CPU do Core 1, deixando respiro para o LwIP threadsafe background
        sleep_ms(10); 
    }
}