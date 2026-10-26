/*
 * tc_parser.h - Strict parser for ASCII telecommands.
 *
 * Format: "$TC:<id>,<val>" optionally followed by CR/LF characters.
 * <id> and <val> are decimal numbers of 1..3 digits in the range 0..255.
 *
 * Pure C with no SDK/RTOS dependency, so it can be unit-tested on the host
 * (see tests/test_tc_parser.c).
 */
#ifndef TC_PARSER_H
#define TC_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline bool tc_parse_u8(const char *buf, size_t len, size_t *pos, uint8_t *out)
{
    unsigned value  = 0;
    size_t   digits = 0;

    while (*pos < len && buf[*pos] >= '0' && buf[*pos] <= '9') {
        value = value * 10u + (unsigned)(buf[*pos] - '0');
        if (++digits > 3u || value > 255u) {
            return false;
        }
        (*pos)++;
    }
    if (digits == 0u) {
        return false;
    }
    *out = (uint8_t)value;
    return true;
}

/* Returns true and fills id/val only if the whole buffer is a valid telecommand. */
static inline bool tc_parse(const char *buf, size_t len, uint8_t *id, uint8_t *val)
{
    static const char prefix[] = "$TC:";
    const size_t prefix_len = sizeof(prefix) - 1u;

    if (buf == NULL || id == NULL || val == NULL || len < prefix_len) {
        return false;
    }
    if (memcmp(buf, prefix, prefix_len) != 0) {
        return false;
    }

    size_t  pos = prefix_len;
    uint8_t parsed_id;
    uint8_t parsed_val;

    if (!tc_parse_u8(buf, len, &pos, &parsed_id)) {
        return false;
    }
    if (pos >= len || buf[pos] != ',') {
        return false;
    }
    pos++;
    if (!tc_parse_u8(buf, len, &pos, &parsed_val)) {
        return false;
    }
    while (pos < len && (buf[pos] == '\r' || buf[pos] == '\n')) {
        pos++;
    }
    if (pos != len) {
        return false;   /* trailing garbage or embedded NUL */
    }

    *id  = parsed_id;
    *val = parsed_val;
    return true;
}

#endif /* TC_PARSER_H */
