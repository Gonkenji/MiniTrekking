#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h"
#include "mapeamento.h" 

int8_t mapa_grid[MAP_CELLS][MAP_CELLS] = {0};

const uint ENCODER_DIR_PIN_BASE = 20; 
const uint ENCODER_ESQ_PIN_BASE = 26; 

static PIO pio_global;
static uint sm_dir, sm_esq;

const float RAIO_RODA_MM = 16.0f; 
const float REDUCAO_MOTOR = 30.0f; 
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

// ======================================================================
// TIMER APENAS PARA MATEMÁTICA E PIO (Livre de travamentos de I2C)
// ======================================================================
bool odometria_timer_callback(struct repeating_timer *t) {
    // Apenas lê a variável global que o loop principal atualizou com segurança
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
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Sistema de Mapeamento Blindado Iniciado! ---\n");

    color_init();
    imu_init();
    tof_init();

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

    uint32_t last_sensor_read = 0;
    uint32_t last_color_read = 0;
    uint32_t last_map_update = 0;
    uint32_t last_terminal_print = 0;

    uint16_t dist_l1x = 4000, dist_s1 = 1200, dist_s2 = 1200;
    uint16_t c = 0, r = 0, g = 0, b = 0;
    
    uint8_t contador_amarelo = 0;
    bool sobre_a_placa = false;

    printf("\nRobo livre para mapeamento. O mapa sera impresso a cada 30 segundos...\n");

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // ======================================================================
        // 1. LEITURAS I2C SEQUENCIAIS (Garante que nunca haverá colisão no barramento)
        // ======================================================================
        if (current_time - last_sensor_read >= 10) {
            // Atualiza IMU primeiro
            float temp_yaw;
            imu_update(0.01f, temp_yaw); 
            yaw_global = temp_yaw;

            // Atualiza ToF em seguida, com a linha I2C 100% liberada
            tof_update(dist_l1x, dist_s1, dist_s2);
            
            last_sensor_read = current_time;
        }

        // 2. SENSOR DE COR (A cada 25ms)
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;
        }

        // 3. PROCESSAMENTO DE MAPA (A cada 100ms)
        if (current_time - last_map_update >= 100) {
            float yaw_rad = yaw_global * ((float)M_PI / 180.0f);

            // ToF Frontal (L1X): Ângulo 0
            if (dist_l1x >= 80) {
                // L1X é mais forte, confiamos nele até 1,5 metros (1500mm)
                processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_l1x, 0.0f, 1500);
            }
            
            // ToF Esquerdo (S1): +30 Graus (+PI/6 Radianos)
            if (dist_s1 >= 80) {
                processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s1, (float)M_PI / 6.0f, 800);
            }
            
            // ToF Direito (S2): -30 Graus (-PI/6 Radianos)
            if (dist_s2 >= 80) {
                processar_leitura_tof(pos_x_global, pos_y_global, yaw_rad, dist_s2, -(float)M_PI / 6.0f, 800);
            }

            last_map_update = current_time;
        }

        // 4. PRINT DO MAPA COMPLETO ESTABILIZADO (A cada 30 Segundos)
        if (current_time - last_terminal_print >= 30000) {
            printf("\033[H\033[J"); 
            
            // Buffer de 80 caracteres + finalizador nulo
            char linha_buffer[MAP_CELLS * 2 + 1]; 

            // Varre o mapa inteiro de cima para baixo
            for (int y = MAP_CELLS - 1; y >= 0; y--) { 
                int pos_buffer = 0; 
                
                // Varre o mapa inteiro da esquerda para a direita
                // Varre o mapa inteiro da esquerda para a direita
                for (int x = 0; x < MAP_CELLS; x++) {  
                    // Volta a usar coord_to_grid
                    if (x == coord_to_grid(pos_x_global) && y == coord_to_grid(pos_y_global)) {
                        linha_buffer[pos_buffer++] = 'R';
                        linha_buffer[pos_buffer++] = ' ';
                    } else if (mapa_grid[x][y] > 50) {
                        linha_buffer[pos_buffer++] = '#';
                        linha_buffer[pos_buffer++] = '#';
                    } else if (mapa_grid[x][y] < -10) {
                        linha_buffer[pos_buffer++] = '.';
                        linha_buffer[pos_buffer++] = ' ';
                    } else {
                        linha_buffer[pos_buffer++] = ' ';
                        linha_buffer[pos_buffer++] = ' ';
                    }
                }
                linha_buffer[pos_buffer] = '\0'; 

                printf("%s\n", linha_buffer);
                
                // Respiro crítico de 5ms mantido para evitar o travamento do I2C/USB
                sleep_ms(5); 
            }
            
            printf("----------------------------------------\n");
            printf("X: %8.1f mm | Y: %8.1f mm | Ang: %6.1f\n", pos_x_global, pos_y_global, yaw_global);
            printf("Mapa rodando estavel. Aguardando proximo ciclo...\n");
            
            stdio_flush(); 
            last_terminal_print = current_time;
        }
    }

    return 0;
}