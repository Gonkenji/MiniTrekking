#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
// Este cabeçalho será gerado automaticamente pelo CMake!
#include "encoder.pio.h" 

const uint ENCODER_PIN_A = 16;
const uint ENCODER_PIN_B = 17;

int32_t get_encoder_count(PIO pio, uint sm) {
    // 1. Copia o valor do registrador X para o ISR
    pio_sm_exec(pio, sm, pio_encode_in(pio_x, 32));
    
    // 2. Empurra (PUSH) o valor do ISR para a fila de leitura do C++
    pio_sm_exec(pio, sm, pio_encode_push(false, false));
    
    // 3. Lê o valor da fila e retorna
    return (int32_t)pio_sm_get(pio, sm);
}

int main() {
    stdio_init_all();

    // Escolhe o bloco PIO 0 e solicita uma máquina de estados (SM) livre
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);

    // Carrega o programa assembly na memória da PIO
    uint offset = pio_add_program(pio, &encoder_program);

    // Executa a função C de configuração (aquela dentro do % c-sdk do arquivo .pio)
    encoder_program_init(pio, sm, offset, ENCODER_PIN_A, ENCODER_PIN_B);

    // ADICIONE ESTA LINHA: Força o registrador X a começar em 0
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 0));

    while (true) {
        // Seu loop continua normal aqui
        int32_t current_position = -get_encoder_count(pio, sm);
        printf("Posição PIO (GP16/17): %ld\n", current_position);
        sleep_ms(100); 
    }
}