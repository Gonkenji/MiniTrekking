#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// ======================================================================
// 1. CONFIGURAÇÕES DE SISTEMA (Bare-metal)
// ======================================================================
// Permite rodar o lwIP nativamente no loop principal (sem RTOS)
#define NO_SYS                      1

// Desativa APIs bloqueantes que exigiriam um sistema operacional
#define LWIP_SOCKET                 0
#define LWIP_COMPAT_SOCKETS         0
#define LWIP_NETCONN                0

// ======================================================================
// 2. GERENCIAMENTO DE MEMÓRIA (Otimizado para o mapa HTML)
// ======================================================================
// Simplifica o processamento de pacotes diretamente no hardware
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_NETIF_RX_SINGLE_PBUF   1

// Memória principal expandida para suportar a string de 8KB do seu mapa HTML
#define MEM_SIZE                    8192

// ======================================================================
// 3. PROTOCOLOS DE REDE (Enxutos)
// ======================================================================
#define LWIP_IPV4                   1
#define LWIP_ARP                    1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DHCP                   1 // Necessário para a arquitetura do Access Point
#define LWIP_ICMP                   1 // Mantido em 1 para você continuar conseguindo testar via "ping"

// Protocolos desativados (0) para poupar CPU e RAM, pois não são usados pelo robô
#define LWIP_DNS                    0 
#define LWIP_IGMP                   0 

// ======================================================================
// 4. TUNING DO TCP (Ajustado para envio em massa na porta 80)
// ======================================================================
#define LWIP_TCP_KEEPALIVE          1

// Tamanho máximo do segmento (padrão Wi-Fi)
#define TCP_MSS                     1460 

// Janelas e buffers dimensionados para comportar o envio do HTML sem gargalos
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

// Aumenta o limite de blocos de memória para suportar a fila de envio (corrige o sanity_check)
#define MEMP_NUM_TCP_SEG            32

#endif /* _LWIPOPTS_H */