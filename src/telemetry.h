#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#include "obc_types.h"

#define TELEMETRY_ASCII_CAPACITY (7u + 2u * sizeof(ccsds_telemetry_packet_t) + 2u)

uint16_t telemetry_crc16(const uint8_t *data, size_t length);
void telemetry_build_packet(ccsds_telemetry_packet_t *packet,
                            const obc_state_t *state,
                            uint32_t uptime_ms,
                            uint8_t boot_cause,
                            uint16_t sequence);
size_t telemetry_encode_ascii(const ccsds_telemetry_packet_t *packet,
                              char *output,
                              size_t output_size);

#endif /* TELEMETRY_H */
