#include <stdio.h>
#include <math.h>
#include <cstring> // Resolvido o problema do memset indefinido
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h"
#include "mapeamento.h" 
#include "wifi.h" 
#include "navegacao.h" // Inclusão do sistema de busca A*

int8_t mapa_grid[MAP_CELLS][MAP_CELLS] = {0};
bool rota_wifi_grid[40][40] = {false}; // Array global lido pelo WiFi para plotar caminho

const uint ENCODER_DIR_PIN_BASE = 20; 
const uint ENCODER_ESQ_PIN_BASE = 26; 

static PIO pio_global;
static uint sm_dir, sm_esq;

const float RAIO_RODA_MM = 18.0f;
const float REDUCAO_MOTOR = 50.0f;
const float PULSOS_POR_VOLTA_MOTOR = 14.0f;
const float PPR_TOTAL = REDUCAO_MOTOR * PULSOS_POR_VOLTA_MOTOR;
const float MM_POR_PULSO = (2.0f * (float)M_PI * RAIO_RODA_MM) / PPR_TOTAL;

volatile float yaw_global = 0.0f;
volatile int32_t contagem_dir = 0;
volatile int32_t contagem_esq = 0;

volatile float pos_x_global = 0.0f;
volatile float pos_y_global = 0.0f;

int32_t get_encoder_count(PIO pio, uint sm) {
    pio_sm_exec(pio, sm, pio_encode_in(pio_x, 32));
    pio_sm_exec(pio, sm, pio_encode_push(false, false));
    return (int32_t)pio_sm_get(pio, sm);
}

// TIMER DE ODOMETRIA
bool odometria_timer_callback(struct repeating_timer *t) {
    float yaw_radianos = yaw_global * ((float)M_PI / 180.0f);

    int32_t leitura_atual_dir = get_encoder_count(pio_global, sm_dir);
    int32_t leitura_atual_esq = -get_encoder_count(pio_global, sm_esq);
    
    int32_t delta_dir = leitura_atual_dir - contagem_dir;
    int32_t delta_esq = leitura_atual_esq - contagem_esq;
    
    contagem_dir = leitura_atual_dir;
    contagem_esq = leitura_atual_esq;

    float dist_mm_dir = (float)delta_dir * MM_POR_PULSO;
    float dist_mm_esq = (float)delta_esq * MM_POR_PULSO;
    float delta_centro_mm = (dist_mm_dir + dist_mm_esq) / 2.0f;

    pos_x_global += delta_centro_mm * cosf(yaw_radianos);
    pos_y_global += delta_centro_mm * sinf(yaw_radianos);

    return true; 
}

int main() {
    stdio_init_all();
    
    // Atraso de 2 segundos para estabilização da tensão e calibração estática do IMU
    sleep_ms(2000); 
    
    color_init();
    imu_init();
    tof_init();

    // Inicialização da PIO para os encoders
    pio_global = pio0;
    uint offset = pio_add_program(pio_global, &encoder_program);
    sm_dir = pio_claim_unused_sm(pio_global, true);
    sm_esq = pio_claim_unused_sm(pio_global, true);
    encoder_program_init(pio_global, sm_dir, offset, ENCODER_DIR_PIN_BASE);
    pio_sm_exec(pio_global, sm_dir, pio_encode_set(pio_x, 0)); 
    encoder_program_init(pio_global, sm_esq, offset, ENCODER_ESQ_PIN_BASE);
    pio_sm_exec(pio_global, sm_esq, pio_encode_set(pio_x, 0)); 

    // Ativa Timer de Odometria
    struct repeating_timer timer_odometria;
    add_repeating_timer_ms(-10, odometria_timer_callback, NULL, &timer_odometria);

    // Inicia a rede Wi-Fi e Servidor Web em Background
    wifi_init_ap();

    uint32_t last_sensor_read = 0;
    uint32_t last_color_read = 0;
    uint32_t last_map_update = 0;

    uint16_t dist_l1x = 4000, dist_s1 = 1200, dist_s2 = 1200;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    // --- VARIÁVEIS DE NAVEGAÇÃO ---
    std::vector<std::pair<int, int>> rota_atual;
    bool recalcular_rota = true;
    
    // Ponto de Interesse (POI) alvo - Exemplo
    float poi_x = 1000.0f; 
    float poi_y = 500.0f;

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // 1. LEITURAS I2C SEQUENCIAIS
        if (current_time - last_sensor_read >= 10) {
            float temp_yaw;
            imu_update(0.01f, temp_yaw); 
            yaw_global = temp_yaw;

            tof_update(dist_l1x, dist_s1, dist_s2);
            last_sensor_read = current_time;
        }

        // 2. SENSOR DE COR
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;
        }

        // 3. PROCESSAMENTO DE MAPA E ROTA
        if (current_time - last_map_update >= 100) {
            float yaw_rad = yaw_global * ((float)M_PI / 180.0f);

            float offsets_l1x[3] = {-0.131f, 0.0f, 0.131f}; // ~15° FOV
            float offsets_l0x[3] = {-0.218f, 0.0f, 0.218f}; // ~25° FOV

            if (dist_l1x > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_l1x, offsets_l1x[i], 1500);
                }
            }
            if (dist_s1 > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s1, ((float)M_PI / 6.0f) + offsets_l0x[i], 1200);
                }
            }
            if (dist_s2 > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s2, -(float)M_PI / 6.0f + offsets_l0x[i], 1200);
                }
            }

            // Checa se o A* precisa ser refeito por causa de um obstáculo novo lido agora
            if (!rota_atual.empty() && verificar_rota_bloqueada(rota_atual)) {
                recalcular_rota = true; 
            }

            // Atualiza os dados que serão despachados via interface Web
            wifi_atualizar_dados(pos_x_global, pos_y_global, yaw_global, dist_l1x, dist_s1, dist_s2);
            last_map_update = current_time;
        }

        // 4. PLANEJADOR A* 
        if (recalcular_rota) {
            int grid_start_x = coord_to_grid(pos_x_global);
            int grid_start_y = coord_to_grid(pos_y_global);
            int grid_goal_x = coord_to_grid(poi_x);
            int grid_goal_y = coord_to_grid(poi_y);
            
            rota_atual = calcular_A_star(grid_start_x, grid_start_y, grid_goal_x, grid_goal_y);
            
            // Passa a rota recalculada para a visualização web
            memset(rota_wifi_grid, 0, sizeof(rota_wifi_grid));
            for(auto const& ponto : rota_atual) {
                rota_wifi_grid[ponto.first][ponto.second] = true; 
            }
            
            recalcular_rota = false;
        }
    }

    return 0;
}