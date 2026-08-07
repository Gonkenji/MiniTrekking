#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/pio.h"
#include "encoder.pio.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h"

// --- CONFIGURAÇÕES DOS ENCODERS ---
const uint ENCODER_DIR_PIN_BASE = 20; // Motor Direita  (GP20 e GP21)
const uint ENCODER_ESQ_PIN_BASE = 26; // Motor Esquerda (GP26 e GP27)

static PIO pio_global;
static uint sm_dir, sm_esq;

// --- VARIÁVEIS GLOBAIS DE ESTADO (VOLATILE) ---
// Devem ser 'volatile' pois são alteradas paralelamente pela interrupção do timer
volatile float yaw_global = 0.0f;
volatile int32_t contagem_dir = 0;
volatile int32_t contagem_esq = 0;
volatile int32_t delta_dir = 0;
volatile int32_t delta_esq = 0;

// Função auxiliar para solicitar a contagem à PIO
int32_t get_encoder_count(PIO pio, uint sm) {
    pio_sm_exec(pio, sm, pio_encode_in(pio_x, 32));
    pio_sm_exec(pio, sm, pio_encode_push(false, false));
    return (int32_t)pio_sm_get(pio, sm);
}

// --- TIMER DE FUSÃO SENSORIAL (ODOMETRIA + IMU) ---
// Callback do Timer de Hardware rodando a cada 10ms cravados
bool odometria_timer_callback(struct repeating_timer *t) {
    // 1. Atualiza o IMU
    float temp_yaw;
    imu_update(0.01f, temp_yaw); 
    yaw_global = temp_yaw;

    // 2. Atualiza os Encoders
    int32_t leitura_atual_dir = get_encoder_count(pio_global, sm_dir);
    int32_t leitura_atual_esq = -get_encoder_count(pio_global, sm_esq); // Inversão da roda esquerda
    
    // 3. Calcula os deltas de passos nos últimos 10ms
    delta_dir = leitura_atual_dir - contagem_dir;
    delta_esq = leitura_atual_esq - contagem_esq;
    
    // 4. Salva a contagem total
    contagem_dir = leitura_atual_dir;
    contagem_esq = leitura_atual_esq;

    return true; 
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    printf("\n--- Sistema Completo Iniciado! ---\n");

    color_init();
    printf("Sensor de Cor OK!\n");

    imu_init();
    printf("Sensor IMU OK!\n");

    tof_init();
    printf("Sensores ToF (1x L1X, 2x L0X) Inicializados!\n");

    // --- INICIALIZAÇÃO DOS ENCODERS (PIO) ---
    pio_global = pio0;
    uint offset = pio_add_program(pio_global, &encoder_program);

    sm_dir = pio_claim_unused_sm(pio_global, true);
    sm_esq = pio_claim_unused_sm(pio_global, true);

    encoder_program_init(pio_global, sm_dir, offset, ENCODER_DIR_PIN_BASE);
    pio_sm_exec(pio_global, sm_dir, pio_encode_set(pio_x, 0)); 

    encoder_program_init(pio_global, sm_esq, offset, ENCODER_ESQ_PIN_BASE);
    pio_sm_exec(pio_global, sm_esq, pio_encode_set(pio_x, 0)); 
    printf("Encoders PIO Inicializados!\n");

    // --- CONFIGURAÇÃO DO TIMER DE HARDWARE ---
    struct repeating_timer timer_odometria;
    add_repeating_timer_ms(-10, odometria_timer_callback, NULL, &timer_odometria);

    // --- VARIÁVEIS DO LOOP PRINCIPAL ---
    uint32_t last_tof_read = 0;
    uint16_t dist_l1x = 4000;
    uint16_t dist_s1 = 1200, dist_s2 = 1200;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    uint32_t last_color_read = 0;
    uint32_t last_terminal_print = 0;

    uint8_t contador_amarelo = 0;
    bool sobre_a_placa = false;
    
    printf("\nA iniciar loop de controle principal...\n");

    // --- LOOP PRINCIPAL ---
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // --- LEITURA ToF (Limitado a cada 10ms para não inundar o I2C) ---
        if (current_time - last_tof_read >= 10) {
            tof_update(dist_l1x, dist_s1, dist_s2);
            last_tof_read = current_time;
        }

        // --- LEITURA DO SENSOR DE COR (A cada 25ms) ---
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;

            // Validação de Debounce da placa amarela
            if (detectar_placa_amarela(c, r, g, b)) {
                contador_amarelo++;
                
                // Exige 3 leituras consecutivas (75ms garantidos de amarelo)
                if (contador_amarelo >= 3 && !sobre_a_placa) { 
                    sobre_a_placa = true;
                    printf("\n--- MARCAÇÃO ENCONTRADA! ---\n");
                    // Chamada para a matriz de mapeamento
                }
            } else {
                contador_amarelo = 0; 
                sobre_a_placa = false;
            }
        }

        // --- ATUALIZAÇÃO DO TERMINAL (A cada 100ms) ---
        if (current_time - last_terminal_print >= 100) {
            printf("\033[H\033[J"); // Limpa o terminal
            printf("=== DADOS DE NAVEGAÇÃO ===\n");
            
            // ToF
            printf("ToF L1X (Frontal) : %4d mm\n", dist_l1x);
            printf("ToF L0X (Esq/Dir) : %4d mm | %4d mm\n", dist_s1, dist_s2);
            
            // Cor
            printf("Sensor de Cor     : C:%4u R:%4u G:%4u B:%4u\n", c, r, g, b);
            
            // Odometria (Variáveis do Timer)
            printf("IMU Yaw           : %6.2f graus\n", yaw_global); 
            printf("Encoder Esquerda  : %5ld (Delta: %3ld)\n", contagem_esq, delta_esq);
            printf("Encoder Direita   : %5ld (Delta: %3ld)\n", contagem_dir, delta_dir);
            
            printf("==========================\n");
            
            last_terminal_print = current_time;
        }
    }

    return 0;
}