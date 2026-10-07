#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define WIFI_SSID "CUBESAT-GROUND-STATION"
#define WIFI_PASSWORD "12345678"
#define UDP_TARGET_PORT 5000
#define UDP_LOCAL_PORT 5001

#pragma pack(push, 1)
typedef struct {
    uint16_t packet_id;        
    uint16_t packet_seq_ctrl;  
    uint16_t packet_data_len;  
} ccsds_primary_header_t;

typedef struct {
    ccsds_primary_header_t header; 
    uint32_t uptime_ms;            
    int16_t  core_temp_centi_c;    
    uint8_t  flight_mode;          
    uint16_t crc16;                
} ccsds_telemetry_packet_t;
#pragma pack(pop)

typedef enum {
    STATE_NORMAL = 0,
    STATE_SAFE_MODE
} obc_flight_mode_t;

static obc_flight_mode_t current_mode = STATE_NORMAL;
static uint16_t tlm_sequence_counter = 0;
static struct udp_pcb *udp_client_pcb = NULL;
static ip_addr_t esp32_ip;

static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc = crc << 1;
        }
    }
    return crc;
}

// Λήψη εντολών ασύρματα μέσω UDP
static void udp_rx_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        char cmd_buffer[64];
        size_t len = p->tot_len < sizeof(cmd_buffer) - 1 ? p->tot_len : sizeof(cmd_buffer) - 1;
        pbuf_copy_partial(p, cmd_buffer, len, 0);
        cmd_buffer[len] = '\0';

        if (strncmp(cmd_buffer, "$TC:", 4) == 0) {
            unsigned int cmd_id = 0, param = 0;
            if (sscanf(cmd_buffer + 4, "%x,%x", &cmd_id, &param) >= 1) {
                switch (cmd_id) {
                    case 0x10:
                        printf("[TC ACK] PONG\n");
                        break;
                    case 0x20:
                        current_mode = (param == 0x01) ? STATE_SAFE_MODE : STATE_NORMAL;
                        printf("[TC EXEC] MODE CHANGED\n");
                        break;
                }
            }
        }
        pbuf_free(p);
    }
}

static void task_housekeeping_downlink(uint32_t now_ms) {
    tlm_sequence_counter++;

    uint16_t raw_adc = adc_read();
    float voltage = raw_adc * (3.3f / (1 << 12));
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    ccsds_telemetry_packet_t pkt;
    pkt.header.packet_id = 0x0842; 
    pkt.header.packet_seq_ctrl = 0xC000 | (tlm_sequence_counter & 0x3FFF);
    pkt.header.packet_data_len = (sizeof(ccsds_telemetry_packet_t) - sizeof(ccsds_primary_header_t)) - 1;

    pkt.uptime_ms = now_ms;
    pkt.core_temp_centi_c = (int16_t)(temp_c * 100);
    pkt.flight_mode = (current_mode == STATE_NORMAL) ? 0 : 1;
    pkt.crc16 = calculate_crc16((uint8_t*)&pkt, sizeof(ccsds_telemetry_packet_t) - 2);

    // Μετατροπή σε hex string
    char hex_str[128];
    strcpy(hex_str, "$CCSDS:");
    uint8_t *raw = (uint8_t*)&pkt;
    for (size_t i = 0; i < sizeof(ccsds_telemetry_packet_t); i++) {
        sprintf(hex_str + 7 + (i * 2), "%02X", raw[i]);
    }
    strcat(hex_str, "\n");

    // Αποστολή ασύρματα στο ESP32
    if (udp_client_pcb != NULL) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, strlen(hex_str), PBUF_RAM);
        if (p != NULL) {
            memcpy(p->payload, hex_str, strlen(hex_str));
            udp_sendto(udp_client_pcb, p, &esp32_ip, UDP_TARGET_PORT);
            pbuf_free(p);
        }
    }
}

static void task_optical_beacon(uint32_t now_ms) {
    static uint32_t last_toggle_time = 0;
    static bool led_state = false;

    uint32_t interval = (current_mode == STATE_NORMAL) ? (led_state ? 100 : 900) : 200;
    if (now_ms - last_toggle_time >= interval) {
        led_state = !led_state;
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        last_toggle_time = now_ms;
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Σύνδεση στο Wi-Fi του ESP32
    printf("Connecting to CubeSat AP...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        printf("Failed to connect!\n");
        return -1;
    }
    printf("Connected to Ground Station AP!\n");

    ipaddr_aton("192.168.4.1", &esp32_ip);

    // Άνοιγμα UDP socket
    udp_client_pcb = udp_new();
    udp_bind(udp_client_pcb, IP_ADDR_ANY, UDP_LOCAL_PORT);
    udp_recv(udp_client_pcb, udp_rx_callback, NULL);

    watchdog_enable(3500, 1);

    uint32_t last_tlm_time = 0;
    uint32_t last_wdog_time = 0;

    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        cyw43_arch_poll(); // Διαχείριση δικτύου Wi-Fi
        task_optical_beacon(current_time);

        if (current_time - last_wdog_time >= 400) {
            watchdog_update();
            last_wdog_time = current_time;
        }

        if (current_time - last_tlm_time >= 1000) {
            task_housekeeping_downlink(current_time);
            last_tlm_time = current_time;
        }

        sleep_ms(5);
    }

    return 0;
}