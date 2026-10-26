#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* lwIP configuration: NO_SYS=0 (FreeRTOS), BSD sockets enabled. */

#define NO_SYS                          0
#define LWIP_SOCKET                     1
#define LWIP_NETCONN                    1

#define LWIP_SO_RCVTIMEO                1
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 1

#define TCPIP_THREAD_STACKSIZE          2048
#define TCPIP_THREAD_PRIO               3
#define DEFAULT_THREAD_STACKSIZE        1024
#define DEFAULT_RAW_RECVMBOX_SIZE       8
#define TCPIP_MBOX_SIZE                 8
#define DEFAULT_UDP_RECVMBOX_SIZE       TCPIP_MBOX_SIZE
#define DEFAULT_TCP_RECVMBOX_SIZE       TCPIP_MBOX_SIZE
#define DEFAULT_ACCEPTMBOX_SIZE         TCPIP_MBOX_SIZE

#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        4000
#define MEMP_NUM_TCP_SEG                32
#define MEMP_NUM_ARP_QUEUE              10
#define PBUF_POOL_SIZE                  24

#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_IPV4                       1
#define LWIP_UDP                        1
#define LWIP_TCP                        1
#define LWIP_DHCP                       1
#define LWIP_DNS                        1
#define LWIP_TCP_KEEPALIVE              1

#define TCP_MSS                         1460
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_BUF                     (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))

#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETIF_TX_SINGLE_PBUF       1

#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DHCP_DOES_ACD_CHECK        0

#define LWIP_CHKSUM_ALGORITHM           3

#define MEM_STATS                       0
#define SYS_STATS                       0
#define MEMP_STATS                      0
#define LINK_STATS                      0

#define LWIP_TIMEVAL_PRIVATE            0

#endif /* _LWIPOPTS_H */
