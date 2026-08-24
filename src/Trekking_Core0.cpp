#include <stdio.h>
#include <math.h>
#include <cstring>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h" 
#include "navegacao.h" 
#include "shared_data.h"

// --- INSTANCIAÇÃO DAS VARIÁVEIS DO IPC ---
mutex_t mutex_rota;
mutex_t mutex_sensores;
critical_section_t cs_estado;

volatile float pos_x_global = 0.0f;
volatile float pos_y_global = 0.0f;
volatile float yaw_global = 0.0f;

volatile uint16_t dist_l1x_global = 2500;
volatile uint16_t dist_s1_global = 600;
volatile uint16_t dist_s2_global = 600;
volatile float erro_dist_global = 0.0f;
volatile float erro_ang_global = 0.0f;

std::vector<std::pair<int, int>> rota_atual_global;
volatile bool flag_recalcular_rota = true;
volatile size_t idx_rota_global = 0;

bool rota_wifi_grid[MAP_CELLS][MAP_CELLS] = {false};
int8_t mapa_grid[MAP_CELLS][MAP_CELLS] = {0}; 

void init_shared_sync() {
    mutex_init(&mutex_rota);
    mutex_init(&mutex_sensores);
    critical_section_init(&cs_estado);
}

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

volatile int32_t contagem_dir = 0;
volatile int32_t contagem_esq = 0;

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
    critical_section_enter_blocking(&cs_estado);
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
    critical_section_exit(&cs_estado);

    return true; 
}

int calcular_erro_rota_otimizado(float x_atual, float y_atual, float yaw_atual_graus, const std::vector<std::pair<int, int>>& rota, size_t &idx_atual, float &erro_d, float &erro_a, float &erro_lateral) {
    if (rota.empty() || idx_atual >= rota.size()) {
        erro_d = 0.0f; erro_a = 0.0f; erro_lateral = 0.0f;
        return 0;
    }

    float min_dist_sq = 99999999.0f;
    size_t melhor_idx = idx_atual;

    size_t limite_busca = std::min(rota.size(), idx_atual + 5);

    for (size_t i = idx_atual; i < limite_busca; i++) {
        float px = grid_to_coord(rota[i].first);
        float py = grid_to_coord(rota[i].second);
        
        float dx = px - x_atual;
        float dy = py - y_atual;
        float dist_sq = (dx * dx) + (dy * dy); 

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            melhor_idx = i;
        }
    }

    idx_atual = melhor_idx;
    erro_lateral = sqrtf(min_dist_sq);

    size_t lookahead = std::min(rota.size() - 1, idx_atual + 3);
    
    float alvo_x = grid_to_coord(rota[lookahead].first);
    float alvo_y = grid_to_coord(rota[lookahead].second);

    float dx = alvo_x - x_atual;
    float dy = alvo_y - y_atual;
    
    erro_d = sqrtf((dx * dx) + (dy * dy));
    
    float yaw_rad = yaw_atual_graus * ((float)M_PI / 180.0f);
    float angulo_alvo = atan2f(dy, dx);
    erro_a = angulo_alvo - yaw_rad;
    
    while (erro_a > (float)M_PI) erro_a -= 2.0f * (float)M_PI;
    while (erro_a < -(float)M_PI) erro_a += 2.0f * (float)M_PI;

    return 1; 
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    
    init_shared_sync();
    color_init();
    tof_init();
    motores_init(); 
    
    // Inicialização do IMU via SPI
    imu_init();
    
    // Inicialização da PIO para os encoders    
    pio_global = pio0;
    uint offset = pio_add_program(pio_global, &encoder_program);
    sm_dir = pio_claim_unused_sm(pio_global, true);
    sm_esq = pio_claim_unused_sm(pio_global, true);
    encoder_program_init(pio_global, sm_dir, offset, ENCODER_DIR_PIN_BASE);
    pio_sm_exec(pio_global, sm_dir, pio_encode_set(pio_x, 0)); 
    encoder_program_init(pio_global, sm_esq, offset, ENCODER_ESQ_PIN_BASE);
    pio_sm_exec(pio_global, sm_esq, pio_encode_set(pio_x, 0)); 

    struct repeating_timer timer_odometria;
    add_repeating_timer_ms(-10, odometria_timer_callback, NULL, &timer_odometria);

    multicore_launch_core1(core1_main);

    uint32_t last_sensor_read = 0;
    uint32_t last_color_read = 0;
    
    uint16_t d_l1x = 2500, d_s1 = 600, d_s2 = 600;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    uint32_t last_imu_read_ms = 0;
    uint64_t last_imu_time_us = to_us_since_boot(get_absolute_time());

    while (true) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // 1. LEITURA DO IMU LIMITADA A 500Hz (2ms)
    if (current_time - last_imu_read_ms >= 2) {
        uint64_t current_time_imu_us = to_us_since_boot(get_absolute_time());
        float dt_total_sec = (current_time_imu_us - last_imu_time_us) / 1000000.0f;
        last_imu_time_us = current_time_imu_us;

        float yaw_lido = 0.0f;
        imu_update(dt_total_sec, yaw_lido); 

        critical_section_enter_blocking(&cs_estado);
        yaw_global = yaw_lido;
        while (yaw_global > 180.0f) yaw_global -= 360.0f;
        while (yaw_global < -180.0f) yaw_global += 360.0f;
        critical_section_exit(&cs_estado);

        last_imu_read_ms = current_time;
    }

        // 2. LEITURAS I2C SEQUENCIAIS (ToF)
        if (current_time - last_sensor_read >= 10) {
            tof_update(d_l1x, d_s1, d_s2);
            
            mutex_enter_blocking(&mutex_sensores);
            dist_l1x_global = d_l1x;
            dist_s1_global = d_s1;
            dist_s2_global = d_s2;
            mutex_exit(&mutex_sensores);
            
            last_sensor_read = current_time;
        }

        // 3. SENSOR DE COR
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;
        }

        // 4. MALHA DE CONTROLE PID (10ms - 100Hz)
        if (current_time - last_control_update >= 10) {
            
            critical_section_enter_blocking(&cs_estado);
            float px = pos_x_global;
            float py = pos_y_global;
            float yaw = yaw_global;
            critical_section_exit(&cs_estado);

            mutex_enter_blocking(&mutex_rota);
            
            if (!rota_atual_global.empty() && idx_rota_global < rota_atual_global.size()) {
                float e_dist = 0.0f, e_ang = 0.0f, e_lateral = 0.0f; 

                size_t idx_local = idx_rota_global;
                calcular_erro_rota_otimizado(px, py, yaw, rota_atual_global, idx_local, e_dist, e_ang, e_lateral);
                idx_rota_global = idx_local; 

                erro_dist_global = e_dist;
                erro_ang_global = e_ang;

                if (e_lateral > 150.0f) {
                    flag_recalcular_rota = true; 
                }

                float alvo_final_x = grid_to_coord(rota_atual_global.back().first);
                float alvo_final_y = grid_to_coord(rota_atual_global.back().second);
                float dist_objetivo_final = sqrtf(powf(alvo_final_x - px, 2) + powf(alvo_final_y - py, 2));

                if (dist_objetivo_final < 60.0f) {
                    set_motores(0, 0);
                    idx_rota_global = rota_atual_global.size(); 
                } else {
                    float v_linear = (fabsf(e_ang) < 0.78f) ? e_dist * KP_LINEAR : 0.0f;
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
            mutex_exit(&mutex_rota);
            
            last_control_update = current_time;
        }
    }
    return 0;
}