#include "ICM20948_DMA.hpp"
#include <cstdio>
#include <cmath> // Necessário para std::abs

// Estrutura do Filtro Simples (Outlier Rejection + EMA)
class SimpleIMUFilter {
private:
    IMUData last_valid;
    float alpha;            // Fator de suavização (0.0 a 1.0). Menor = mais suave, Maior = mais responsivo
    float max_gyro_delta;   // Variação máxima permitida entre leituras consecutivas (dps)
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

        // 1. Detecção de "Dado Trocado" (Outlier Rejection)
        // Se a variação brusca em qualquer eixo do giroscópio for maior que o limite, ignora a amostra
        if (std::abs(current.gyroX - last_valid.gyroX) > max_gyro_delta ||
            std::abs(current.gyroY - last_valid.gyroY) > max_gyro_delta ||
            std::abs(current.gyroZ - last_valid.gyroZ) > max_gyro_delta) {
            
            // Retorna o último dado válido sem atualizar (rejeita a anomalia)
            return last_valid;
        }

        // 2. Filtro Passa-Baixa (EMA) para suavização do ruído mecânico
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

int main() {
    stdio_init_all();
    
    ICM20948 imu(spi0, 16, 17, 18, 19);
    imu.init();
    
    IMUData raw_buffer[10];
    
    // Instancia o filtro (Alpha = 0.3, Limite de salto = 60 dps)
    SimpleIMUFilter imuFilter(0.3f, 60.0f);
    
    while (true) {
        // Dispara a leitura do hardware. A função retorna imediatamente.
        imu.startFIFODMARead(10);
        
        // ========================================================
        // A CPU ESTÁ LIVRE NESTE MOMENTO
        // ========================================================
        
        // Verifica continuamente se o DMA já encerrou
        int lidos = imu.checkAndGetFIFO(raw_buffer);
        
        if (lidos > 0) {
            // Passamos apenas a amostra mais recente [0] pelo filtro para o controle
            IMUData filtered_data = imuFilter.apply(raw_buffer[0]);

            printf("Amostras Extraidas de Background: %d\n", lidos);
            printf("Accel(g) Filtrado: X=%.2f, Y=%.2f, Z=%.2f\n", 
                   filtered_data.accelX, filtered_data.accelY, filtered_data.accelZ);
            printf("Giro(dps) Filtrado: X=%.2f, Y=%.2f, Z=%.2f\n", 
                   filtered_data.gyroX, filtered_data.gyroY, filtered_data.gyroZ);
            printf("---------------------\n");
        }
        
        sleep_ms(5); 
    }
}