#include "wifi.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "mapeamento.h"
#include <string.h>
#include <stdio.h>

extern volatile float pos_x_global;
extern volatile float pos_y_global;
extern volatile float yaw_global;

// Novas variáveis globais calculadas na malha de controle
extern volatile float erro_dist_global;
extern volatile float erro_ang_global;

// Importação obrigatória da matriz de rota populada pelo main.cpp
extern bool rota_wifi_grid[MAP_CELLS][MAP_CELLS];

static uint16_t tof_frente_global = 0;
static uint16_t tof_esq_global = 0;
static uint16_t tof_dir_global = 0;

void wifi_atualizar_dados(float x, float y, float yaw, uint16_t dist_l1x, uint16_t dist_s1, uint16_t dist_s2) {
    tof_frente_global = dist_l1x;
    tof_esq_global = dist_s1;
    tof_dir_global = dist_s2;
}

static err_t http_sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    tcp_close(tpcb);
    return ERR_OK;
}

static err_t http_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    char *request = (char *)p->payload;

    if (strncmp(request, "GET /favicon", 12) == 0) {
        const char *http_404 = "HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
        tcp_write(tpcb, http_404, strlen(http_404), 0);
        tcp_sent(tpcb, http_sent_callback);
        tcp_output(tpcb);
        pbuf_free(p);
        return ERR_OK;
    }

    if (strncmp(request, "GET", 3) == 0) {
        static char buffer[8192];
        buffer[0] = '\0'; 

        strcpy(buffer, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n"
                       "<!DOCTYPE html><html><head><title>Mapa do Trekking</title>"
                       "<meta http-equiv='refresh' content='2'>"
                       "<style>body{background:#1e1e1e;color:#0f0;font-family:monospace;text-align:center;}"
                       "pre{display:inline-block;text-align:left;line-height:16px;font-size:16px;letter-spacing:4px;}</style>"
                       "</head><body><h2>Mapeamento e Navegacao Global</h2><pre>\n");

        char *ptr = buffer + strlen(buffer);
        
        int rob_x = coord_to_grid(pos_x_global);
        int rob_y = coord_to_grid(pos_y_global);

        for (int y = MAP_CELLS - 1; y >= 0; y--) {
            for (int x = 0; x < MAP_CELLS; x++) {
                
                // Valida se a coordenada é exatamente a posição do robô (1x1)
                bool is_robot = (x == rob_x && y == rob_y);

                if (is_robot) {
                    *ptr++ = 'R'; 
                } else if (mapa_grid[x][y] > 50) {
                    *ptr++ = '#'; 
                } else if (rota_wifi_grid[x][y]) {
                    *ptr++ = '+'; 
                } else if (mapa_grid[x][y] < -10) {
                    *ptr++ = '.'; 
                } else {
                    *ptr++ = ' '; 
                }
            }
            *ptr++ = '\n'; 
        }
        *ptr = '\0'; 
        
        // Adição dos erros de distância e ângulo na renderização HTML
        sprintf(ptr, "</pre><p>X: %.1f mm | Y: %.1f mm | Ang: %.1f</p>"
                     "<p>ToF Frente: %u mm | ToF Esq: %u mm | ToF Dir: %u mm</p>"
                     "<p>Erro Dist: %.1f mm | Erro Ang: %.2f rad</p></body></html>\n",
                pos_x_global, pos_y_global, yaw_global,
                tof_frente_global, tof_esq_global, tof_dir_global,
                erro_dist_global, erro_ang_global);

        err_t err_write = tcp_write(tpcb, buffer, strlen(buffer), 0);
        if (err_write == ERR_OK) {
            tcp_sent(tpcb, http_sent_callback);
            tcp_output(tpcb);
        } else {
            tcp_close(tpcb);
        }
    } else {
        tcp_close(tpcb);
    }

    pbuf_free(p);
    return ERR_OK;
}

static err_t http_accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_recv_callback);
    return ERR_OK;
}

void wifi_init_ap() {
    if (cyw43_arch_init()) return;
    cyw43_arch_enable_ap_mode("Mudkip", "umapinha", CYW43_AUTH_WPA2_AES_PSK);
    cyw43_arch_lwip_begin();
    struct tcp_pcb *pcb = tcp_new();
    if (pcb != NULL) {
        tcp_bind(pcb, IP_ADDR_ANY, 80);
        pcb = tcp_listen(pcb);
        tcp_accept(pcb, http_accept_callback);
    }
    cyw43_arch_lwip_end();
}