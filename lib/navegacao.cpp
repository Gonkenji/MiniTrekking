#include "navegacao.h"
#include "mapeamento.h"
#include <queue>
#include <cmath>
#include <algorithm>

struct Node {
    int x, y;
    int g_cost, f_cost;
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost; 
    }
};

std::vector<std::pair<int, int>> calcular_A_star(int start_x, int start_y, int goal_x, int goal_y) {
    std::vector<std::pair<int, int>> caminho;
    
    if (start_x < 0 || start_x >= MAP_CELLS || start_y < 0 || start_y >= MAP_CELLS ||
        goal_x < 0 || goal_x >= MAP_CELLS || goal_y < 0 || goal_y >= MAP_CELLS) return caminho;
        
    if (mapa_grid[goal_x][goal_y] > LIMIAR_OBSTACULO) return caminho; 

    bool fechado[MAP_CELLS][MAP_CELLS] = {false};
    int custo_g[MAP_CELLS][MAP_CELLS];
    std::pair<int, int> pai[MAP_CELLS][MAP_CELLS];

    for (int i = 0; i < MAP_CELLS; i++) {
        for (int j = 0; j < MAP_CELLS; j++) {
            custo_g[i][j] = 999999;
        }
    }

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> aberta;
    aberta.push({start_x, start_y, 0, 0});
    custo_g[start_x][start_y] = 0;

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    int move_cost[] = {10, 10, 10, 10, 14, 14, 14, 14};

    bool encontrou = false;

    while (!aberta.empty()) {
        Node atual = aberta.top();
        aberta.pop();

        if (atual.x == goal_x && atual.y == goal_y) {
            encontrou = true;
            break;
        }

        if (fechado[atual.x][atual.y]) continue;
        fechado[atual.x][atual.y] = true;

        for (int i = 0; i < 8; i++) {
            int nx = atual.x + dx[i];
            int ny = atual.y + dy[i];

            if (nx >= 0 && nx < MAP_CELLS && ny >= 0 && ny < MAP_CELLS) {
                if (mapa_grid[nx][ny] > LIMIAR_OBSTACULO || fechado[nx][ny]) continue;

                int penalidade = (mapa_grid[nx][ny] > 0) ? 15 : 0;
                int novo_g = custo_g[atual.x][atual.y] + move_cost[i] + penalidade;

                if (novo_g < custo_g[nx][ny]) {
                    custo_g[nx][ny] = novo_g;
                    int h = (abs(nx - goal_x) + abs(ny - goal_y)) * 10; 
                    aberta.push({nx, ny, novo_g, novo_g + h});
                    pai[nx][ny] = {atual.x, atual.y};
                }
            }
        }
    }

    if (encontrou) {
        int cx = goal_x, cy = goal_y;
        while (cx != start_x || cy != start_y) {
            caminho.push_back({cx, cy});
            std::pair<int, int> p = pai[cx][cy];
            cx = p.first;
            cy = p.second;
        }
        std::reverse(caminho.begin(), caminho.end()); 
    }

    return caminho;
}

bool verificar_rota_bloqueada(const std::vector<std::pair<int, int>>& rota) {
    for (auto const& ponto : rota) {
        if (mapa_grid[ponto.first][ponto.second] > LIMIAR_OBSTACULO) {
            return true;
        }
    }
    return false;
}