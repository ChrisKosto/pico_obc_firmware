/* Host-side unit tests for tc_parser.h.
 * Build & run:  gcc -Wall -Wextra -I. tests/test_tc_parser.c -o /tmp/test_tc && /tmp/test_tc
 */
#include <stdio.h>
#include "tc_parser.h"

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

static bool parse_str(const char *s, uint8_t *id, uint8_t *val)
{
    return tc_parse(s, strlen(s), id, val);
}

int main(void)
{
    uint8_t id = 0xAA, val = 0xBB;

    /* valid commands used by the ground station */
    CHECK(parse_str("$TC:10,00", &id, &val) && id == 10 && val == 0);
    CHECK(parse_str("$TC:20,00\n", &id, &val) && id == 20 && val == 0);
    CHECK(parse_str("$TC:20,01\r\n", &id, &val) && id == 20 && val == 1);
    CHECK(parse_str("$TC:99,01", &id, &val) && id == 99 && val == 1);
    CHECK(parse_str("$TC:255,255", &id, &val) && id == 255 && val == 255);
    CHECK(parse_str("$TC:1,2", &id, &val) && id == 1 && val == 2);

    /* malformed input must be rejected and must not modify the outputs */
    id = 0xAA; val = 0xBB;
    CHECK(!parse_str("", &id, &val));
    CHECK(!parse_str("$TC:", &id, &val));
    CHECK(!parse_str("TC:10,00", &id, &val));
    CHECK(!parse_str("$TC:10", &id, &val));
    CHECK(!parse_str("$TC:10,", &id, &val));
    CHECK(!parse_str("$TC:,00", &id, &val));
    CHECK(!parse_str("$TC:10;00", &id, &val));
    CHECK(!parse_str("$TC:256,00", &id, &val));
    CHECK(!parse_str("$TC:10,999", &id, &val));
    CHECK(!parse_str("$TC:0010,00", &id, &val));
    CHECK(!parse_str("$TC:-1,00", &id, &val));
    CHECK(!parse_str("$TC:1a,00", &id, &val));
    CHECK(!parse_str("xx$TC:10,00", &id, &val));        /* the old strstr() would accept this */
    CHECK(!parse_str("$TC:10,00 garbage", &id, &val));
    CHECK(!parse_str("$TC:10,00\n$TC:99,01", &id, &val));
    CHECK(id == 0xAA && val == 0xBB);

    /* embedded NUL */
    CHECK(!tc_parse("$TC:10,00\0x", 11, &id, &val));

    /* NULL arguments */
    CHECK(!tc_parse(NULL, 5, &id, &val));
    CHECK(!tc_parse("$TC:10,00", 9, NULL, &val));

    if (failures == 0) {
        printf("tc_parser: all tests passed\n");
        return 0;
    }
    printf("tc_parser: %d failure(s)\n", failures);
    return 1;
}
