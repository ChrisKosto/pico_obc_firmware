#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
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
    ccsds_primary_header_t header; // 6 bytes
    
    // Payload (6 bytes)
    uint32_t uptime_ms;            
    int16_t  core_temp_centi_c;    
    
    // Checksum (2 bytes)
    uint16_t crc16;                
} ccsds_telemetry_packet_t;

#pragma pack(pop)

// Καταστάσεις του OBC (Flight State Machine)
typedef enum {
    STATE_BOOT = 0,
    STATE_NORMAL,
    STATE_SAFE_MODE
} obc_flight_mode_t;

// Global System State
static obc_flight_mode_t current_mode = STATE_BOOT;
static uint16_t tlm_sequence_counter = 0;

// CRC-16/CCITT calculation
static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1) ^ 0x1021; // polynomial
            }
        }
    }
    return crc;
}

// -------------------------------------------------------------
// TASK 1: Housekeeping Telemetry (Εκτελείται κάθε 1000ms)
// -------------------------------------------------------------
static void task_housekeeping_downlink(uint32_t now_ms) {
    tlm_sequence_counter++;

    // Διάβασμα αισθητήρα
    uint16_t raw_adc = adc_read();
    float voltage = raw_adc * (3.3f / (1 << 12));
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    // FDIR Logic: Αν ανέβει υπερβολικά η θερμοκρασία, αλλάζουμε Flight Mode
    if (temp_c > 35.0f && current_mode != STATE_SAFE_MODE) {
        current_mode = STATE_SAFE_MODE;
        printf("\n[FDIR EVENT] Critical Temp threshold crossed! Switching to SAFE_MODE\n");
    }

    // Δόμηση πακέτου CCSDS
    ccsds_telemetry_packet_t pkt;
    pkt.header.packet_id = 0x0842; 
    pkt.header.packet_seq_ctrl = 0xC000 | (tlm_sequence_counter & 0x3FFF);
    pkt.header.packet_data_len = (sizeof(ccsds_telemetry_packet_t) - sizeof(ccsds_primary_header_t)) - 1;

    pkt.uptime_ms = now_ms;
    pkt.core_temp_centi_c = (int16_t)(temp_c * 100);
    pkt.crc16 = calculate_crc16((uint8_t*)&pkt, sizeof(ccsds_telemetry_packet_t) - 2);

    // Εκπομπή πλαισίου
    printf("[DOWNLINK FRAME] ");
    uint8_t *raw = (uint8_t*)&pkt;
    for (size_t i = 0; i < sizeof(ccsds_telemetry_packet_t); i++) {
        printf("%02X ", raw[i]);
    }
    printf("| Temp: %.2f C | Mode: %s\n", 
           temp_c, (current_mode == STATE_NORMAL) ? "NORMAL" : "SAFE_MODE");
}

// -------------------------------------------------------------
// TASK 2: Hardware Watchdog Kick (Εκτελείται κάθε 400ms)
// -------------------------------------------------------------
static void task_watchdog_feed(void) {
    watchdog_update();
}

// -------------------------------------------------------------
// TASK 3: Health Status Beacon (Εκτελείται κάθε 5000ms)
// -------------------------------------------------------------
static void task_health_beacon(uint32_t now_ms) {
    printf(">>> [BEACON] OBC Health: ALIVE | Uptime: %lu s | Packets Sent: %u <<<\n",
           (unsigned long)(now_ms / 1000), tlm_sequence_counter);
}

// -------------------------------------------------------------
// MAIN: Time-Triggered Scheduler Loop
// -------------------------------------------------------------
int main() {
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    bool was_watchdog_reset = watchdog_caused_reboot();

    // 3 δευτερόλεπτα αναμονή για το USB CDC
    sleep_ms(3000);

    // Watchdog timeout στα 2500ms
    watchdog_enable(2500, 1);

    printf("\n========================================================\n");
    printf("   PICO 2 W (RP2350) - TIME-TRIGGERED FLIGHT SOFTWARE   \n");
    printf("   Architecture: Non-blocking Cyclic Executive          \n");
    if (was_watchdog_reset) {
        printf("   [WARN] SYSTEM REBOOT TRIGGERED BY WATCHDOG TIMEOUT!  \n");
    } else {
        printf("   [INFO] SYSTEM REBOOT: NORMAL COLD START              \n");
    }
    printf("========================================================\n\n");

    current_mode = STATE_NORMAL;

    // Χρονικά σημεία για τον scheduler
    uint32_t last_tlm_time = 0;
    uint32_t last_wdog_time = 0;
    uint32_t last_beacon_time = 0;

    while (1) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Εκτέλεση Task 2 (Watchdog) κάθε 400 ms
        if (current_time - last_wdog_time >= 400) {
            task_watchdog_feed();
            last_wdog_time = current_time;
        }

        // Εκτέλεση Task 1 (Housekeeping) κάθε 1000 ms
        if (current_time - last_tlm_time >= 1000) {
            task_housekeeping_downlink(current_time);
            last_tlm_time = current_time;
        }

        // Εκτέλεση Task 3 (System Beacon) κάθε 5000 ms
        if (current_time - last_beacon_time >= 5000) {
            task_health_beacon(current_time);
            last_beacon_time = current_time;
        }

        // Μικρό sleep 10ms για μείωση CPU churn
        sleep_ms(10);
    }

    return 0;
}