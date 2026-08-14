#ifndef NAVEGACAO_H
#define NAVEGACAO_H

#include <vector>
#include <utility>

// Limiar: Valores maiores no mapa_grid serão desviados pelo A*
#define LIMIAR_OBSTACULO 20 

std::vector<std::pair<int, int>> calcular_A_star(int start_x, int start_y, int goal_x, int goal_y);
bool verificar_rota_bloqueada(const std::vector<std::pair<int, int>>& rota);

#endif // NAVEGACAO_H