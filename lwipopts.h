#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Στο Pico SDK bare-metal, το NO_SYS ΠΡΕΠΕΙ να είναι 1
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0

#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        4000
#define MEMP_NUM_TCP_SEG                32
#define MEMP_NUM_ARP_QUEUE              10
#define PBUF_POOL_SIZE                  16

#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_UDP                        1
#define LWIP_TCP                        0
#define LWIP_DHCP                       1

// Checksum offloading / calculation
#define CHECKSUM_GEN_IP                 1
#define CHECKSUM_GEN_UDP                1
#define CHECKSUM_CHECK_IP               1
#define CHECKSUM_CHECK_UDP              1

#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0

#endif // _LWIPOPTS_H