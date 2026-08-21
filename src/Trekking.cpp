#include <stdio.h>
#include <math.h>
#include <cstring>
#include <stdlib.h> // Adicionado para função abs()
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h"
#include "mapeamento.h" 
#include "wifi.h" 
#include "navegacao.h" 

int8_t mapa_grid[MAP_CELLS][MAP_CELLS] = {0};
bool rota_wifi_grid[MAP_CELLS][MAP_CELLS] = {false}; 

const uint ENCODER_DIR_PIN_BASE = 20; 
const uint ENCODER_ESQ_PIN_BASE = 26; 

// --- PINOS DA PONTE H ---
const uint PWM_A_PIN = 1;  // Motor Esquerda (A)
const uint DIR_A_PIN1 = 3;
const uint DIR_A_PIN2 = 2;

const uint PWM_B_PIN = 10; // Motor Direita (B)
const uint DIR_B_PIN1 = 8;
const uint DIR_B_PIN2 = 9;

static PIO pio_global;
static uint sm_dir, sm_esq;

const float RAIO_RODA_MM = 15.0f;
const float REDUCAO_MOTOR = 10.0f;
const float PULSOS_POR_VOLTA_MOTOR = 14.0f;
const float PPR_TOTAL = REDUCAO_MOTOR * PULSOS_POR_VOLTA_MOTOR;
const float MM_POR_PULSO = (2.0f * (float)M_PI * RAIO_RODA_MM) / PPR_TOTAL;

// --- CONSTANTES DO CONTROLE P (Ajuste conforme os testes) ---
const float KP_LINEAR = 0.8f;   
const float KP_ANGULAR = 45.0f; 
const int PWM_MAX_TESTE = 70;

volatile float yaw_global = 0.0f;
volatile int32_t contagem_dir = 0;
volatile int32_t contagem_esq = 0;

volatile float pos_x_global = 0.0f;
volatile float pos_y_global = 0.0f;

volatile float erro_dist_global = 0.0f;
volatile float erro_ang_global = 0.0f;

uint32_t last_control_update = 0;

// --- FUNÇÕES DOS MOTORES ---
void motores_init() {
    // Configura os pinos de direção como saída
    gpio_init(DIR_A_PIN1); gpio_set_dir(DIR_A_PIN1, GPIO_OUT);
    gpio_init(DIR_A_PIN2); gpio_set_dir(DIR_A_PIN2, GPIO_OUT);
    gpio_init(DIR_B_PIN1); gpio_set_dir(DIR_B_PIN1, GPIO_OUT);
    gpio_init(DIR_B_PIN2); gpio_set_dir(DIR_B_PIN2, GPIO_OUT);

    // Configura os pinos de PWM
    gpio_set_function(PWM_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_B_PIN, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(PWM_A_PIN);
    uint slice_b = pwm_gpio_to_slice_num(PWM_B_PIN);

   // Configuração básica do PWM 
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 255); // Resolução de 8 bits (0 a 255)
    
    // ADICIONE ESTA LINHA PARA SALVAR A PONTE H:
    pwm_config_set_clkdiv(&config, 100.0f); 
    
    pwm_init(slice_a, &config, true);
    pwm_init(slice_b, &config, true);
    
    // Garante que os motores comecem parados
    pwm_set_gpio_level(PWM_A_PIN, 0);
    pwm_set_gpio_level(PWM_B_PIN, 0);
}

// Função utilitária para quando for implementar o controle (ainda não utilizada no loop)
void set_motores(int pwm_esq, int pwm_dir) {
    // Sentido Motor Esquerdo (A)
    if (pwm_esq >= 0) {
        gpio_put(DIR_A_PIN1, 1);
        gpio_put(DIR_A_PIN2, 0);
    } else {
        gpio_put(DIR_A_PIN1, 0);
        gpio_put(DIR_A_PIN2, 1);
    }
    pwm_set_gpio_level(PWM_A_PIN, fabsf(pwm_esq));

    // Sentido Motor Direito (B)
    if (pwm_dir >= 0) {
        gpio_put(DIR_B_PIN1, 1);
        gpio_put(DIR_B_PIN2, 0);
    } else {
        gpio_put(DIR_B_PIN1, 0);
        gpio_put(DIR_B_PIN2, 1);
    }
    pwm_set_gpio_level(PWM_B_PIN, fabsf(pwm_dir));
}
// ---------------------------

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

// Atualize a função antes do main() para esta versão:
int calcular_erro_rota(float x_atual, float y_atual, float yaw_atual_graus, const std::vector<std::pair<int, int>>& rota, float &erro_d, float &erro_a, float &erro_lateral) {
    if (rota.empty()) {
        erro_d = 0.0f; erro_a = 0.0f; erro_lateral = 0.0f;
        return 0;
    }

    float min_dist_sq = 99999999.0f;
    int idx_mais_proximo = 0;

    // 1. Varre o vetor para achar a célula da rota mais próxima
    for (size_t i = 0; i < rota.size(); i++) {
        float px = grid_to_coord(rota[i].first);
        float py = grid_to_coord(rota[i].second);
        
        float dx = px - x_atual;
        float dy = py - y_atual;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            idx_mais_proximo = i;
        }
    }

    // Salva a distância real da roda para a linha do trajeto
    erro_lateral = sqrtf(min_dist_sq);

    // 2. Define o alvo de controle (Lookahead de 3 células à frente)
    int lookahead = std::min((int)rota.size() - 1, idx_mais_proximo + 3);
    
    float alvo_x = grid_to_coord(rota[lookahead].first);
    float alvo_y = grid_to_coord(rota[lookahead].second);

    // 3. Calcula os erros para controle dos motores
    float dx = alvo_x - x_atual;
    float dy = alvo_y - y_atual;
    
    erro_d = sqrtf(dx * dx + dy * dy);
    
    float yaw_rad = yaw_atual_graus * ((float)M_PI / 180.0f);
    float angulo_alvo = atan2f(dy, dx);
    erro_a = angulo_alvo - yaw_rad;
    
    // Normalização de ângulo
    while (erro_a > (float)M_PI) erro_a -= 2.0f * (float)M_PI;
    while (erro_a < -(float)M_PI) erro_a += 2.0f * (float)M_PI;

    // Retorna o índice para que a malha de controle saiba o que apagar
    return idx_mais_proximo;
}

int main() {
    stdio_init_all();
    
    // Atraso de 5 segundos para estabilização da tensão e calibração estática do IMU
    sleep_ms(15000); 
    
    color_init();
    imu_init();
    tof_init();
    motores_init(); // Inicializa os pinos e slices de PWM da Ponte H

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

    uint16_t dist_l1x = 2500, dist_s1 = 600, dist_s2 = 600;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    // --- VARIÁVEIS DE NAVEGAÇÃO ---
    std::vector<std::pair<int, int>> rota_atual;
    bool recalcular_rota = true;
    
    // Ponto de Interesse (POI) alvo - Exemplo
    float poi_x = 700.0f; 
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

            float offsets_l1x[3] = {-0.131f, 0.0f, 0.131f}; 
            float offsets_l0x[3] = {-0.218f, 0.0f, 0.218f}; 

            if (dist_l1x > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_l1x, offsets_l1x[i], 3000);
                }
            }
            if (dist_s1 > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s1, ((float)M_PI / 6.0f) + offsets_l0x[i], 400);
                }
            }
            if (dist_s2 > 0) {
                for(int i = 0; i < 3; i++) {
                    processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s2, -(float)M_PI / 6.0f + offsets_l0x[i], 400);
                }
            }

            if (!rota_atual.empty() && verificar_rota_bloqueada(rota_atual)) {
                recalcular_rota = true; 
            }

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
            
            memset(rota_wifi_grid, 0, sizeof(rota_wifi_grid));
            for(auto const& ponto : rota_atual) {
                rota_wifi_grid[ponto.first][ponto.second] = true; 
            }
            
            recalcular_rota = false;
        }

        // 5. MALHA DE CONTROLE (50ms)
        if (current_time - last_control_update >= 50) {
            if (!rota_atual.empty()) {
                float e_dist = 0.0f;
                float e_ang = 0.0f;
                float e_lateral = 0.0f; 

                int idx_closest = calcular_erro_rota(pos_x_global, pos_y_global, yaw_global, rota_atual, e_dist, e_ang, e_lateral);

                erro_dist_global = e_dist;
                erro_ang_global = e_ang;

                // --- ATUALIZAÇÃO DINÂMICA DO CAMINHO ---
                if (idx_closest > 0) {
                    rota_atual.erase(rota_atual.begin(), rota_atual.begin() + idx_closest);
                    
                    memset(rota_wifi_grid, 0, sizeof(rota_wifi_grid));
                    for(auto const& ponto : rota_atual) {
                        rota_wifi_grid[ponto.first][ponto.second] = true; 
                    }
                }

                if (e_lateral > 150.0f) {
                    recalcular_rota = true;
                }

                // --- INÍCIO DO CONTROLE P ---
                // Calcula a distância até o ÚLTIMO ponto da rota (objetivo final)
                float alvo_final_x = grid_to_coord(rota_atual.back().first);
                float alvo_final_y = grid_to_coord(rota_atual.back().second);
                float dist_objetivo_final = sqrtf(powf(alvo_final_x - pos_x_global, 2) + powf(alvo_final_y - pos_y_global, 2));

                if (dist_objetivo_final < 60.0f) {
                    // Chegou próximo ao objetivo final: freia o robô e limpa a rota
                    set_motores(0, 0);
                    rota_atual.clear();
                } else {
                    float v_linear = 0.0f;
                    
                    // Se o erro de ângulo for muito grande (> 45 graus), apenas rotaciona no próprio eixo
                    // antes de tentar ir para frente. Evita que o robô faça curvas muito abertas.
                    if (fabsf(e_ang) < 0.78f) { 
                        v_linear = e_dist * KP_LINEAR;
                    }
                    
                    float w_angular = e_ang * KP_ANGULAR;

                    // Cinemática inversa simples (V e W para PWM das rodas direita e esquerda)
                    int pwm_esq = (int)(v_linear - w_angular);
                    int pwm_dir = (int)(v_linear + w_angular);

                    // Saturação de Segurança (impede que passe da velocidade de teste)
                    if (pwm_esq > PWM_MAX_TESTE) pwm_esq = PWM_MAX_TESTE;
                    if (pwm_esq < -PWM_MAX_TESTE) pwm_esq = -PWM_MAX_TESTE;
                    
                    if (pwm_dir > PWM_MAX_TESTE) pwm_dir = PWM_MAX_TESTE;
                    if (pwm_dir < -PWM_MAX_TESTE) pwm_dir = -PWM_MAX_TESTE;

                    // Aplica na Ponte H
                    set_motores(pwm_esq, pwm_dir);
                }

            } else {
                erro_dist_global = 0.0f;
                erro_ang_global = 0.0f;
                set_motores(0, 0); // Rota vazia, garante motores parados
            }
            
            last_control_update = current_time;
        }
    }

    return 0;
}