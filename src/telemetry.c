#include "telemetry.h"

uint16_t telemetry_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static int16_t to_centi(float value) {
    float scaled = value * 100.0f;
    if (scaled >  32767.0f) scaled =  32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)scaled;
}

void telemetry_build_packet(ccsds_telemetry_packet_t *packet,
                            const obc_state_t *state,
                            uint32_t uptime_ms,
                            uint8_t boot_cause,
                            uint16_t sequence) {
    packet->header.packet_id       = 0x0801;
    packet->header.packet_seq_ctrl = (uint16_t)(0xC000 | (sequence & 0x3FFF));
    packet->header.packet_data_len = (uint16_t)(sizeof(*packet) - sizeof(packet->header) - 1);
    packet->uptime_ms         = uptime_ms;
    packet->mpu_temp_centi_c  = to_centi(state->mpu_temp_c);
    packet->mcu_temp_centi_c  = to_centi(state->mcu_temp_c);
    packet->roll_centi_deg    = to_centi(state->roll);
    packet->pitch_centi_deg   = to_centi(state->pitch);
    packet->yaw_centi_deg     = to_centi(state->yaw);
    packet->flight_mode       = (uint8_t)state->mode;
    packet->boot_cause        = boot_cause;
    packet->fdir_alarm        = state->fdir_alarm;
    packet->crc16 = telemetry_crc16((const uint8_t *)packet, sizeof(*packet) - 2);
}

size_t telemetry_encode_ascii(const ccsds_telemetry_packet_t *packet,
                              char *output,
                              size_t output_size) {
    static const char HEXCH[] = "0123456789ABCDEF";
    const size_t required = 7u + 2u * sizeof(*packet) + 1u;
    if (output_size < required + 1u) {
        return 0;
    }

    size_t pos = 0;
    const char prefix[] = "$CCSDS:";
    for (size_t i = 0; i < sizeof(prefix) - 1u; i++) {
        output[pos++] = prefix[i];
    }

    const uint8_t *bytes = (const uint8_t *)packet;
    for (size_t i = 0; i < sizeof(*packet); i++) {
        output[pos++] = HEXCH[bytes[i] >> 4];
        output[pos++] = HEXCH[bytes[i] & 0x0F];
    }
    output[pos++] = '\n';
    output[pos] = '\0';
    return pos;
}
