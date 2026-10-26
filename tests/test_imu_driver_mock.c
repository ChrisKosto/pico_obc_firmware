/* Host-only simulated I2C tests: not equivalent to a Pico hardware test. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/i2c.h"
#include "mpu6050.h"

i2c_inst_t mock_i2c0;
static uint8_t regs[256], cursor;
static int mode, read_count;
/* mode: 1=noisy gyro, 2=bad gravity, 3=failed transfer,
 *       4=readback mismatch, 5=all-zero buffer */

static void reset_mock(uint8_t who) {
    memset(regs, 0, sizeof(regs));
    regs[0x75] = who;
    cursor = 0;
    mode = 0;
    read_count = 0;
}
unsigned i2c_init(i2c_inst_t *p, unsigned baud) {(void)p;return baud;}
void gpio_set_function(unsigned pin, unsigned mode_value) {(void)pin;(void)mode_value;}
void gpio_pull_up(unsigned pin) {(void)pin;}
void vTaskDelay(uint32_t ticks) {(void)ticks;}
int i2c_write_timeout_us(i2c_inst_t *p, uint8_t addr,
                         const uint8_t *src, size_t n, bool nostop,
                         unsigned timeout) {
    (void)p;(void)nostop;(void)timeout;
    assert(addr == 0x68 && src != NULL);
    if (mode == 3) return -1;
    if (n == 1) {cursor=src[0];return 1;}
    if (n == 2) {regs[src[0]]=src[1];return 2;}
    return -1;
}
static void be16(uint8_t *p, int16_t v) {
    uint16_t u=(uint16_t)v;
    p[0]=(uint8_t)(u>>8);p[1]=(uint8_t)u;
}
int i2c_read_timeout_us(i2c_inst_t *p, uint8_t addr,
                        uint8_t *dst, size_t n, bool nostop,
                        unsigned timeout) {
    (void)p;(void)nostop;(void)timeout;
    assert(addr == 0x68 && dst != NULL);
    if (mode == 3) return -1;
    if (n == 1) {
        *dst=(mode==4 && cursor==0x1B) ? 0x00 : regs[cursor];
        return 1;
    }
    if (n == 14 && cursor == 0x3B) {
        memset(dst, 0, 14);
        if (mode == 5) return 14;
        be16(&dst[4], mode==2 ? 12000 : 16384);
        be16(&dst[6], 2200);
        const int16_t gyro = (mode==1 && (read_count%2)==0) ? 1100 : -196;
        be16(&dst[12],gyro);
        ++read_count;
        return 14;
    }
    return -1;
}

int main(void) {
    float offset = 123.0f;
    mpu6050_sample_t sample;
    reset_mock(0x72);
    mpu6050_bus_init();
    assert(mpu6050_init());
    assert(mpu6050_calibrate_gyro(&offset));
    assert(fabsf(offset + 196.0f / 65.5f) < 0.0001f);
    assert(mpu6050_read_sample(offset, &sample));
    assert(fabsf(sample.gz_dps) < 0.0001f);
    printf("PASS provisional 0x72: readback + calibration + bias subtraction\n");

    reset_mock(0x68);
    assert(mpu6050_init());
    printf("PASS standard 0x68 identity\n");

    reset_mock(0x79);
    assert(!mpu6050_init());
    assert(!mpu6050_read_sample(0.0f, &sample));
    printf("PASS unknown identity rejected\n");

    reset_mock(0x72);
    mode=4;
    assert(!mpu6050_init());
    printf("PASS register readback mismatch rejected\n");

    reset_mock(0x72);
    assert(mpu6050_init());
    mode=1;
    offset=2.0f;
    assert(!mpu6050_calibrate_gyro(&offset));
    assert(offset==0.0f);
    printf("PASS noisy calibration rejected, old bias invalidated\n");

    reset_mock(0x72);
    assert(mpu6050_init());
    mode=2;
    assert(!mpu6050_calibrate_gyro(&offset));
    printf("PASS implausible gravity rejected\n");

    reset_mock(0x72);
    assert(mpu6050_init());
    mode=3;
    assert(!mpu6050_read_sample(0.0f, &sample));
    assert(!mpu6050_calibrate_gyro(&offset));
    printf("PASS I2C failure rejected\n");

    reset_mock(0x72);
    assert(mpu6050_init());
    mode=5;
    assert(!mpu6050_calibrate_gyro(&offset));
    printf("PASS all-zero burst rejected\n");
    puts("ALL IMU MOCK TESTS PASSED");
    return 0;
}
