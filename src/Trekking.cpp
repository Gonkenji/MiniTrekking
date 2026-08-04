#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h" // Necessário para acessar as funções de timer de hardware
#include "sensor_TOF.h"
#include "sensor_cor.h"
#include "sensor_imu.h"

// 1. Variável global VOLATILE, pois será alterada pelo timer em paralelo
volatile float yaw_global = 0.0f;

// 2. Callback do Timer de Hardware que rodará a cada 10ms cravados
bool imu_timer_callback(struct repeating_timer *t) {
    float temp_yaw;
    
    // O tempo dt é fixo em 0.01s (10ms), pois o timer garante essa precisão física
    imu_update(0.01f, temp_yaw); 
    
    // Atualiza a variável global de forma segura
    yaw_global = temp_yaw;
    
    // Retornar true informa ao sistema para continuar repetindo o timer
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
    printf("Sensor ToF VL53L1X OK!\n");

    // 3. Configuração do Timer de Hardware
    struct repeating_timer timer_imu;
    // O valor -10 garante que o tempo conte a partir do INÍCIO da interrupção.
    add_repeating_timer_ms(-10, imu_timer_callback, NULL, &timer_imu);

    // VARIÁVEIS DE ESTADO
    uint16_t dist_vl53l1x = 1200; // Variável única para o VL53L1X
    uint16_t c = 0, r = 0, g = 0, b = 0;

    uint32_t last_color_read = 0;
    uint32_t last_terminal_print = 0;

    uint8_t contador_amarelo = 0;
    bool sobre_a_placa = false;

    printf("\nA iniciar loop de controle principal...\n");

    // 4. LOOP PRINCIPAL
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // --- LEITURA ToF (Continua em background pelo hardware do sensor) ---
        tof_update(dist_vl53l1x); // Atualizado para utilizar apenas o VL53L1X

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
                    // AQUI entra a sua chamada para atualizar a matriz de mapeamento
                }
            } else {
                contador_amarelo = 0; // Zera se ver qualquer outra cor
                sobre_a_placa = false;
            }
        }

        // --- ATUALIZAÇÃO DO TERMINAL (A cada 100ms) ---
        if (current_time - last_terminal_print >= 100) {
            printf("\033[H\033[J"); 
            printf("=== DADOS DOS SENSORES ===\n");
            printf("ToF VL53L1X : %4d mm\n", dist_vl53l1x);
            printf("Cor  : C:%5u  R:%5u  G:%5u  B:%5u\n", c, r, g, b);
            
            // Lemos a variável global atualizada pelo timer
            printf("IMU  : yaw:%6.2f  \n", yaw_global); 
            
            printf("==========================\n");
            
            last_terminal_print = current_time;
        }
    }

    return 0;
}