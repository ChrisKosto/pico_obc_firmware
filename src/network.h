#ifndef NETWORK_H
#define NETWORK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "obc_context.h"

typedef enum {
    NETWORK_SOCKET_OK = 0,
    NETWORK_SOCKET_CREATE_FAILED,
    NETWORK_SOCKET_BIND_FAILED
} network_socket_result_t;

bool network_init(void);
bool network_wifi_link_up(void);
bool network_connect_at_boot(uint32_t timeout_ms);
network_socket_result_t network_open_socket(int *socket_out);
void network_reconnect(void);
void network_send_to_gs(obc_context_t *ctx, const char *message, size_t length);
void network_poll_telecommand(obc_context_t *ctx);

#endif /* NETWORK_H */
