#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "sensor_tof.h"
#include "sensor_cor.h"
#include "sensor_imu.h"

// Variável global VOLATILE, pois será alterada pelo timer em paralelo
volatile float yaw_global = 0.0f;

// Callback do Timer de Hardware que rodará a cada 10ms cravados
bool imu_timer_callback(struct repeating_timer *t) {
    float temp_yaw;
    imu_update(0.01f, temp_yaw); 
    yaw_global = temp_yaw;
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

    // Configuração do Timer de Hardware para a IMU
    struct repeating_timer timer_imu;
    add_repeating_timer_ms(-10, imu_timer_callback, NULL, &timer_imu);

    // VARIÁVEIS DE ESTADO
    uint32_t last_tof_read = 0;
    uint16_t dist_l1x = 4000;
    uint16_t dist_s1 = 1200, dist_s2 = 1200;
    uint16_t c = 0, r = 0, g = 0, b = 0;

    uint32_t last_color_read = 0;
    uint32_t last_terminal_print = 0;

    uint8_t contador_amarelo = 0;
    bool sobre_a_placa = false;
    
    printf("\nA iniciar loop de controle principal...\n");

    // LOOP PRINCIPAL
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // --- LEITURA ToF (Desocupa o barramento I2C0 limitando as perguntas a cada 10ms) ---
        if (current_time - last_tof_read >= 10) {
            tof_update(dist_l1x, dist_s1, dist_s2);
            last_tof_read = current_time;
        }

        // --- LEITURA DO SENSOR DE COR (A cada 25ms) ---
        if (current_time - last_color_read >= 25) {
            color_update(c, r, g, b);
            last_color_read = current_time;

            // Validação de Debounce
            if (detectar_placa_amarela(c, r, g, b)) {
                contador_amarelo++;
                
                // Exige 3 leituras consecutivas (75ms garantidos de amarelo)
                if (contador_amarelo >= 3 && !sobre_a_placa) { 
                    sobre_a_placa = true;
                    printf("--- MARCAÇÃO ENCONTRADA! ---\n");
                    // AQUI entra a chamada para a matriz de mapeamento
                }
            } else {
                contador_amarelo = 0; 
                sobre_a_placa = false;
            }
        }

        // --- ATUALIZAÇÃO DO TERMINAL (A cada 100ms) ---
        if (current_time - last_terminal_print >= 100) {
            printf("\033[H\033[J"); 
            printf("=== DADOS DOS SENSORES ===\n");
            printf("ToF Frontal (L1X) : %4d mm\n", dist_l1x);
            printf("ToF Lado E  (L0X) : %4d mm  |  ToF Lado D (L0X) : %4d mm\n", dist_s1, dist_s2);
            printf("Cor  : C:%5u  R:%5u  G:%5u  B:%5u\n", c, r, g, b);
            
            // Lemos a variável global atualizada pelo timer
            printf("IMU  : yaw:%6.2f  \n", yaw_global); 
            
            printf("==========================\n");
            
            last_terminal_print = current_time;
        }
    }

    return 0;
}