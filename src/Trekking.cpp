#include <stdio.h>
#include <math.h>
#include <cstring>
#include <stdlib.h> // Para abs() e std::abs()
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "ICM20948_DMA.hpp"
#include "mapeamento.h" 
#include "wifi.h" 
#include "navegacao.h" 

// --- CLASSE DO FILTRO IMU ---
class SimpleIMUFilter {
private:
    IMUData last_valid;
    float alpha;            
    float max_gyro_delta;   
    bool first_run;

public:
    SimpleIMUFilter(float alpha_val = 0.3f, float max_delta = 60.0f) 
        : alpha(alpha_val), max_gyro_delta(max_delta), first_run(true) {
        last_valid = {0, 0, 0, 0, 0, 0};
    }

    IMUData apply(IMUData current) {
        if (first_run) {
            last_valid = current;
            first_run = false;
            return current;
        }

        if (std::abs(current.gyroX - last_valid.gyroX) > max_gyro_delta ||
            std::abs(current.gyroY - last_valid.gyroY) > max_gyro_delta ||
            std::abs(current.gyroZ - last_valid.gyroZ) > max_gyro_delta) {
            return last_valid;
        }

        IMUData filtered;
        filtered.accelX = last_valid.accelX + alpha * (current.accelX - last_valid.accelX);
        filtered.accelY = last_valid.accelY + alpha * (current.accelY - last_valid.accelY);
        filtered.accelZ = last_valid.accelZ + alpha * (current.accelZ - last_valid.accelZ);
        
        filtered.gyroX = last_valid.gyroX + alpha * (current.gyroX - last_valid.gyroX);
        filtered.gyroY = last_valid.gyroY + alpha * (current.gyroY - last_valid.gyroY);
        filtered.gyroZ = last_valid.gyroZ + alpha * (current.gyroZ - last_valid.gyroZ);

        last_valid = filtered;
        return filtered;
    }
};

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

// --- CONSTANTES DO CONTROLE P ---
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
    gpio_init(DIR_A_PIN1); gpio_set_dir(DIR_A_PIN1, GPIO_OUT);
    gpio_init(DIR_A_PIN2); gpio_set_dir(DIR_A_PIN2, GPIO_OUT);
    gpio_init(DIR_B_PIN1); gpio_set_dir(DIR_B_PIN1, GPIO_OUT);
    gpio_init(DIR_B_PIN2); gpio_set_dir(DIR_B_PIN2, GPIO_OUT);

    gpio_set_function(PWM_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PWM_B_PIN, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(PWM_A_PIN);
    uint slice_b = pwm_gpio_to_slice_num(PWM_B_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 255); 
    pwm_config_set_clkdiv(&config, 100.0f); 
    
    pwm_init(slice_a, &config, true);
    pwm_init(slice_b, &config, true);
    
    pwm_set_gpio_level(PWM_A_PIN, 0);
    pwm_set_gpio_level(PWM_B_PIN, 0);
}

void set_motores(int pwm_esq, int pwm_dir) {
    if (pwm_esq >= 0) {
        gpio_put(DIR_A_PIN1, 1);
        gpio_put(DIR_A_PIN2, 0);
    } else {
        gpio_put(DIR_A_PIN1, 0);
        gpio_put(DIR_A_PIN2, 1);
    }
    pwm_set_gpio_level(PWM_A_PIN, fabsf(pwm_esq));

    if (pwm_dir >= 0) {
        gpio_put(DIR_B_PIN1, 1);
        gpio_put(DIR_B_PIN2, 0);
    } else {
        gpio_put(DIR_B_PIN1, 0);
        gpio_put(DIR_B_PIN2, 1);
    }
    pwm_set_gpio_level(PWM_B_PIN, fabsf(pwm_dir));
}

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

int calcular_erro_rota(float x_atual, float y_atual, float yaw_atual_graus, const std::vector<std::pair<int, int>>& rota, float &erro_d, float &erro_a, float &erro_lateral) {
    if (rota.empty()) {
        erro_d = 0.0f; erro_a = 0.0f; erro_lateral = 0.0f;
        return 0;
    }

    float min_dist_sq = 99999999.0f;
    int idx_mais_proximo = 0;

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

    erro_lateral = sqrtf(min_dist_sq);
    int lookahead = std::min((int)rota.size() - 1, idx_mais_proximo + 3);
    
    float alvo_x = grid_to_coord(rota[lookahead].first);
    float alvo_y = grid_to_coord(rota[lookahead].second);

    float dx = alvo_x - x_atual;
    float dy = alvo_y - y_atual;
    
    erro_d = sqrtf(dx * dx + dy * dy);
    
    float yaw_rad = yaw_atual_graus * ((float)M_PI / 180.0f);
    float angulo_alvo = atan2f(dy, dx);
    erro_a = angulo_alvo - yaw_rad;
    
    while (erro_a > (float)M_PI) erro_a -= 2.0f * (float)M_PI;
    while (erro_a < -(float)M_PI) erro_a += 2.0f * (float)M_PI;

    return idx_mais_proximo;
}

int main() {
    stdio_init_all();
    
    // Atraso de 2 segundos para estabilização da tensão e calibração
    sleep_ms(2000); 
    
    color_init();
    tof_init();
    motores_init(); 
    
    // Inicialização do novo IMU com DMA
    ICM20948 imu(spi0, 16, 17, 18, 19);
    imu.init();
    SimpleIMUFilter imuFilter(0.3f, 60.0f);
    IMUData imu_buffer[10];
    
    // ---------------------------------------------------------
    // NOVA ROTINA DE CALIBRAÇÃO DO GIROSCÓPIO (BIAS)
    // ---------------------------------------------------------
    printf("Calibrando IMU (MANTENHA O ROBO PARADO)...\n");
    float gyroZ_bias = 0.0f;
    float soma_z = 0.0f;
    int amostras_calib = 0;
    
    // Aproveita 2 segundos do seu tempo de setup para coletar dados
    uint32_t start_calib = to_ms_since_boot(get_absolute_time());
    while(to_ms_since_boot(get_absolute_time()) - start_calib < 2000) {
        imu.startFIFODMARead(10);
        int lidos = imu.checkAndGetFIFO(imu_buffer);
        if (lidos > 0) {
            soma_z += imu_buffer[0].gyroZ;
            amostras_calib++;
        }
        sleep_ms(5); // Pequeno delay para preencher o FIFO
    }
    
    if (amostras_calib > 0) {
        gyroZ_bias = soma_z / amostras_calib;
        printf("Calibracao concluida. Bias Z: %.3f dps (Amostras: %d)\n", gyroZ_bias, amostras_calib);
    }
    // ---------------------------------------------------------

    uint32_t last_imu_time = to_ms_since_boot(get_absolute_time());

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

    wifi_init_ap();

    uint32_t last_sensor_read = 0;
    uint32_t last_color_read = 0;
    uint32_t last_map_update = 0;

    uint16_t dist_l1x = 2500, dist_s1 = 600, dist_s2 = 600;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    std::vector<std::pair<int, int>> rota_atual;
    bool recalcular_rota = true;
    
    float poi_x = 700.0f; 
    float poi_y = 500.0f;

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // 1. LEITURA CONTÍNUA E ASSÍNCRONA DO IMU (DMA)
        imu.startFIFODMARead(10);
        int lidos_imu = imu.checkAndGetFIFO(imu_buffer);
        
        if (lidos_imu > 0) {
            IMUData filtered_data = imuFilter.apply(imu_buffer[0]);
            
            // Subtrai o erro intrínseco lido na inicialização
            float true_gyroZ = filtered_data.gyroZ - gyroZ_bias;
            
            // Integração do giroscópio (eixo Z) para calcular o Yaw em graus
            float dt_sec = (current_time - last_imu_time) / 1000.0f;
            
            // Zona morta simples (agora atuando sobre o valor corrigido)
            if (std::abs(true_gyroZ) > 0.5f) {
                yaw_global += (true_gyroZ * dt_sec);
            }
            
            // Mantém o Yaw normalizado entre -180 e 180
            if (yaw_global > 180.0f) yaw_global -= 360.0f;
            if (yaw_global < -180.0f) yaw_global += 360.0f;

            last_imu_time = current_time;
        }

        // 2. LEITURAS I2C SEQUENCIAIS (Apenas ToF agora, IMU está em background)
        if (current_time - last_sensor_read >= 10) {
            tof_update(dist_l1x, dist_s1, dist_s2);
            last_sensor_read = current_time;
        }

        // 3. SENSOR DE COR
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;
        }

        // 4. PROCESSAMENTO DE MAPA E ROTA
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

        // 5. PLANEJADOR A* 
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

        // 6. MALHA DE CONTROLE (50ms)
        if (current_time - last_control_update >= 50) {
            if (!rota_atual.empty()) {
                float e_dist = 0.0f;
                float e_ang = 0.0f;
                float e_lateral = 0.0f; 

                int idx_closest = calcular_erro_rota(pos_x_global, pos_y_global, yaw_global, rota_atual, e_dist, e_ang, e_lateral);

                erro_dist_global = e_dist;
                erro_ang_global = e_ang;

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

                float alvo_final_x = grid_to_coord(rota_atual.back().first);
                float alvo_final_y = grid_to_coord(rota_atual.back().second);
                float dist_objetivo_final = sqrtf(powf(alvo_final_x - pos_x_global, 2) + powf(alvo_final_y - pos_y_global, 2));

                if (dist_objetivo_final < 60.0f) {
                    set_motores(0, 0);
                    rota_atual.clear();
                } else {
                    float v_linear = 0.0f;
                    
                    if (fabsf(e_ang) < 0.78f) { 
                        v_linear = e_dist * KP_LINEAR;
                    }
                    
                    float w_angular = e_ang * KP_ANGULAR;

                    int pwm_esq = (int)(v_linear - w_angular);
                    int pwm_dir = (int)(v_linear + w_angular);

                    if (pwm_esq > PWM_MAX_TESTE) pwm_esq = PWM_MAX_TESTE;
                    if (pwm_esq < -PWM_MAX_TESTE) pwm_esq = -PWM_MAX_TESTE;
                    
                    if (pwm_dir > PWM_MAX_TESTE) pwm_dir = PWM_MAX_TESTE;
                    if (pwm_dir < -PWM_MAX_TESTE) pwm_dir = -PWM_MAX_TESTE;

                    set_motores(pwm_esq, pwm_dir);
                }

            } else {
                erro_dist_global = 0.0f;
                erro_ang_global = 0.0f;
                set_motores(0, 0);
            }
            
            last_control_update = current_time;
        }
    }

    return 0;
}