#include "network.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#include "obc_config.h"
#include "reset_cause.h"
#include "tc_parser.h"

bool network_init(void) {
    if (cyw43_arch_init()) {
        return false;
    }
    cyw43_arch_enable_sta_mode();
    
    // Απενεργοποίηση εξοικονόμησης ενέργειας στο Wi-Fi για να μην έχουμε lag
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    return true;
}

bool network_wifi_link_up(void) {
    return cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

bool network_connect_at_boot(uint32_t timeout_ms) {
    if (cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD,
                                      CYW43_AUTH_WPA2_AES_PSK) != 0) {
        return false;
    }

    const uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!network_wifi_link_up()) {
        if ((to_ms_since_boot(get_absolute_time()) - start) >= timeout_ms) {
            return false;
        }
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return true;
}

network_socket_result_t network_open_socket(int *socket_out) {
    *socket_out = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (*socket_out < 0) {
        return NETWORK_SOCKET_CREATE_FAILED;
    }

    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_len = sizeof(local);
    local.sin_family = AF_INET;
    local.sin_port = lwip_htons(UDP_PORT_PICO);
    local.sin_addr.s_addr = lwip_htonl(INADDR_ANY);

    if (lwip_bind(*socket_out, (struct sockaddr *)&local, sizeof(local)) < 0) {
        return NETWORK_SOCKET_BIND_FAILED;
    }

    // Το αφήνουμε στα 500ms όπως ήταν αρχικά
    int receive_timeout_ms = 500;
    lwip_setsockopt(*socket_out, SOL_SOCKET, SO_RCVTIMEO,
                    &receive_timeout_ms, sizeof(receive_timeout_ms));
    return NETWORK_SOCKET_OK;
}

void network_reconnect(void) {
    cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
}

void network_send_to_gs(obc_context_t *ctx, const char *message, size_t length) {
    if (ctx->socket < 0 || !network_wifi_link_up()) {
        return;
    }

    struct sockaddr_in destination;
    memset(&destination, 0, sizeof(destination));
    destination.sin_len = sizeof(destination);
    destination.sin_family = AF_INET;
    destination.sin_port = lwip_htons(UDP_PORT_GS);
    destination.sin_addr.s_addr = ipaddr_addr(GS_IP);

    int sent = lwip_sendto(ctx->socket, message, length, 0,
                           (struct sockaddr *)&destination, sizeof(destination));
    if (sent < 0 || (size_t)sent != length) {
        printf("[NET] UDP send failed or short send\n");
    }
}

static void network_send_tc_ack(obc_context_t *ctx, bool have_command,
                                uint8_t id, uint8_t value, uint8_t status) {
    char message[40];
    int length;
    if (have_command) {
        length = snprintf(message, sizeof(message), "$TM:ACK,%u,%u,%u\n",
                          (unsigned)id, (unsigned)value, (unsigned)status);
    } else {
        length = snprintf(message, sizeof(message), "$TM:ACK,--,--,%u\n",
                          (unsigned)status);
    }

    if (length > 0 && (size_t)length < sizeof(message)) {
        network_send_to_gs(ctx, message, (size_t)length);
    }
}

void network_poll_telecommand(obc_context_t *ctx) {
    char buffer[64];
    const uint32_t ground_station_address = ipaddr_addr(GS_IP);
    struct sockaddr_in source;
    socklen_t source_length = sizeof(source);

    // Χρήση MSG_DONTWAIT για να μην μπλοκάρει το socket και καθυστερεί η τηλεμετρία
    int received = lwip_recvfrom(ctx->socket, buffer, sizeof(buffer) - 1u, MSG_DONTWAIT,
                                 (struct sockaddr *)&source, &source_length);
    if (received <= 0) {
        return;
    }
    buffer[received] = '\0';

    if (source.sin_addr.s_addr != ground_station_address) {
        printf("[TC] ignored packet from unexpected source %s\n",
               ip4addr_ntoa((const ip4_addr_t *)&source.sin_addr));
        return;
    }

    uint8_t id = 0;
    uint8_t value = 0;
    if (!tc_parse(buffer, (size_t)received, &id, &value)) {
        printf("[TC] malformed command (%d bytes)\n", received);
        network_send_tc_ack(ctx, false, 0, 0, TC_ACK_REJ_MALFORMED);
        return;
    }

    uint8_t status = TC_ACK_ACCEPTED;
    tc_cmd_t command = TC_NONE;
    bool do_ping = false;
    bool do_reboot = false;

    if      (id == 10 && value == 0) do_ping = true;
    else if (id == 20 && value == 0) command = TC_SET_NOMINAL;
    else if (id == 20 && value == 1) command = TC_SET_SAFE;
    else if (id == 99 && value == 1) do_reboot = true;
    else                             status = TC_ACK_REJ_UNKNOWN;

    if (command != TC_NONE && xQueueSend(ctx->tc_queue, &command, 0) != pdTRUE) {
        status = TC_ACK_REJ_QUEUE_FULL;
    }

    printf("[TC] id=%u val=%u -> status %u\n",
           (unsigned)id, (unsigned)value, (unsigned)status);
    network_send_tc_ack(ctx, true, id, value, status);

    if (status != TC_ACK_ACCEPTED) {
        return;
    }
    if (do_ping) {
        const char alive[] = "$TM:PUS17,ALIVE\n";
        network_send_to_gs(ctx, alive, strlen(alive));
    }
    if (do_reboot) {
        reset_cause_record(BOOT_COMMANDED);
        ctx->reboot_commanded = true;
    }
}