#include <stdio.h>
#include <string.h>

#include "telemetry.h"

static int failures = 0;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

int main(void) {
    CHECK(telemetry_crc16((const uint8_t *)"123456789", 9) == 0x29B1);

    obc_state_t state = {
        .mpu_temp_c = 45.67f,
        .mcu_temp_c = 33.21f,
        .roll = 1.25f,
        .pitch = -1.75f,
        .yaw = 180.0f,
        .mode = FSM_MODE_NOMINAL,
        .fdir_alarm = FDIR_OK
    };

    ccsds_telemetry_packet_t packet;
    telemetry_build_packet(&packet, &state, 123456u, BOOT_POWER_ON, 0x4001u);

    CHECK(packet.header.packet_id == 0x0801);
    CHECK(packet.header.packet_seq_ctrl == 0xC001);
    CHECK(packet.header.packet_data_len == 18);
    CHECK(packet.uptime_ms == 123456u);
    CHECK(packet.mpu_temp_centi_c == 4567);
    CHECK(packet.mcu_temp_centi_c == 3321);
    CHECK(packet.roll_centi_deg == 125);
    CHECK(packet.pitch_centi_deg == -175);
    CHECK(packet.yaw_centi_deg == 18000);
    CHECK(packet.flight_mode == FSM_MODE_NOMINAL);
    CHECK(packet.boot_cause == BOOT_POWER_ON);
    CHECK(packet.crc16 == telemetry_crc16((const uint8_t *)&packet, sizeof(packet) - 2u));

    char encoded[TELEMETRY_ASCII_CAPACITY];
    size_t length = telemetry_encode_ascii(&packet, encoded, sizeof(encoded));
    CHECK(length == 58u);
    CHECK(strlen(encoded) == length);
    CHECK(strncmp(encoded, "$CCSDS:", 7) == 0);
    CHECK(encoded[length - 1u] == '\n');
    CHECK(telemetry_encode_ascii(&packet, encoded, 10u) == 0u);

    state.roll = 1000.0f;
    state.pitch = -1000.0f;
    telemetry_build_packet(&packet, &state, 0u, BOOT_POWER_ON, 0u);
    CHECK(packet.roll_centi_deg == 32767);
    CHECK(packet.pitch_centi_deg == -32768);

    if (failures == 0) {
        printf("telemetry: all tests passed\n");
        return 0;
    }
    printf("telemetry: %d failure(s)\n", failures);
    return 1;
}
