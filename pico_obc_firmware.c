#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"

#pragma pack(push, 1)

// CCSDS 133.0-B-2 Primary Header (6 bytes)
typedef struct {
    uint16_t packet_id;        
    uint16_t packet_seq_ctrl;  
    uint16_t packet_data_len;  
} ccsds_primary_header_t;

// Πλήρες πακέτο Housekeeping Telemetry
typedef struct {
    ccsds_primary_header_t header; 
    uint32_t uptime_ms;            
    int16_t  core_temp_centi_c;    
    uint16_t crc16;                
} ccsds_telemetry_packet_t;

#pragma pack(pop)

typedef enum {
    STATE_NORMAL = 0,
    STATE_SAFE_MODE
} obc_flight_mode_t;

static obc_flight_mode_t current_mode = STATE_NORMAL;
static uint16_t tlm_sequence_counter = 0;

static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// -------------------------------------------------------------
// TASK 1: Downlink Frame
// -------------------------------------------------------------
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
    pkt.crc16 = calculate_crc16((uint8_t*)&pkt, sizeof(ccsds_telemetry_packet_t) - 2);

    printf("[DOWNLINK FRAME] ");
    uint8_t *raw = (uint8_t*)&pkt;
    for (size_t i = 0; i < sizeof(ccsds_telemetry_packet_t); i++) {
        printf("%02X ", raw[i]);
    }
    printf("| Temp: %.2f C | Mode: %s\n", 
           temp_c, (current_mode == STATE_NORMAL) ? "NORMAL" : "SAFE_MODE");
}

// -------------------------------------------------------------
// TASK 2: Telecommand Parser (Uplink) - Non Blocking!
// -------------------------------------------------------------
static void task_telecommand_parser(void) {
    static char cmd_buffer[32];
    static size_t idx = 0;

    int ch = getchar_timeout_us(0); // Δεν περιμένει καθόλου αν δεν υπάρχει χαρακτήρας
    while (ch != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (idx > 0) {
                cmd_buffer[idx] = '\0';
                
                // Επεξεργασία εντολής
                if (strcmp(cmd_buffer, "PING") == 0) {
                    printf("\n>>> [TC ACK] PONG! System alive and responding. <<<\n\n");
                } else if (strcmp(cmd_buffer, "SAFE") == 0) {
                    current_mode = STATE_SAFE_MODE;
                    printf("\n>>> [TC EXEC] State forced to: SAFE_MODE <<<\n\n");
                } else if (strcmp(cmd_buffer, "NORM") == 0) {
                    current_mode = STATE_NORMAL;
                    printf("\n>>> [TC EXEC] State forced to: NORMAL <<<\n\n");
                } else {
                    printf("\n>>> [TC REJECT] Unknown command: '%s' <<<\n\n", cmd_buffer);
                }
                idx = 0; // reset buffer
            }
        } else if (idx < sizeof(cmd_buffer) - 1) {
            cmd_buffer[idx++] = (char)ch;
        }
        ch = getchar_timeout_us(0);
    }
}

// -------------------------------------------------------------
// MAIN
// -------------------------------------------------------------
int main() {
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    sleep_ms(3000);
    watchdog_enable(2500, 1);

    printf("\n========================================================\n");
    printf("   PICO 2 W - OBC FLIGHT SOFTWARE (WITH TC UPLINK)      \n");
    printf("   Commands available: PING | SAFE | NORM              \n");
    printf("========================================================\n\n");

    uint32_t last_tlm_time = 0;
    uint32_t last_wdog_time = 0;

    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // 1. Έλεγχος για εντολές από τη Γη συνεχώς
        task_telecommand_parser();

        // 2. Watchdog kick κάθε 400ms
        if (current_time - last_wdog_time >= 400) {
            watchdog_update();
            last_wdog_time = current_time;
        }

        // 3. Telemetry downlink κάθε 1000ms
        if (current_time - last_tlm_time >= 1000) {
            task_housekeeping_downlink(current_time);
            last_tlm_time = current_time;
        }

        sleep_ms(5); // Πολύ μικρό delay για άμεση απόκριση στο πληκτρολόγιο
    }

    return 0;
}