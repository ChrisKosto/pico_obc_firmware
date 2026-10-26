#ifndef MOCK_HARDWARE_I2C_H
#define MOCK_HARDWARE_I2C_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct { unsigned unused; } i2c_inst_t;
extern i2c_inst_t mock_i2c0;
#define i2c0 (&mock_i2c0)
unsigned i2c_init(i2c_inst_t *instance, unsigned baudrate);
int i2c_write_timeout_us(i2c_inst_t *instance, uint8_t address,
                         const uint8_t *src, size_t len, bool nostop,
                         unsigned timeout_us);
int i2c_read_timeout_us(i2c_inst_t *instance, uint8_t address,
                        uint8_t *dst, size_t len, bool nostop,
                        unsigned timeout_us);
#endif
