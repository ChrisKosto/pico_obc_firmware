#include "mpu6050.h"

#include <math.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"

#include "FreeRTOS.h"
#include "task.h"

#include "obc_config.h"

#define I2C_PORT                i2c0
#define REG_PWR_MGMT_1          0x6B
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_XOUT          0x3B
#define GYRO_FS_500             0x08

void mpu6050_bus_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
}

bool mpu6050_init(void) {
    uint8_t wake[2] = { REG_PWR_MGMT_1, 0x00 };
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, wake, 2, false, I2C_TIMEOUT_US) < 0) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg[2] = { REG_GYRO_CONFIG, GYRO_FS_500 };
    return i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, cfg, 2, false, I2C_TIMEOUT_US) >= 0;
}

static bool mpu6050_read_burst(uint8_t data[14]) {
    uint8_t reg = REG_ACCEL_XOUT;
    if (i2c_write_timeout_us(I2C_PORT, MPU6050_ADDR, &reg, 1, true, I2C_TIMEOUT_US) < 0) {
        return false;
    }
    return i2c_read_timeout_us(I2C_PORT, MPU6050_ADDR, data, 14, false, I2C_TIMEOUT_US) == 14;
}

bool mpu6050_read_sample(float gz_offset, mpu6050_sample_t *sample) {
    uint8_t data[14];
    if (!mpu6050_read_burst(data)) {
        return false;
    }

    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);
    int16_t rt = (int16_t)((data[6] << 8) | data[7]);
    int16_t gz = (int16_t)((data[12] << 8) | data[13]);

    sample->temperature_c = (rt / 340.0f) + 36.53f;
    sample->roll_deg = atan2f((float)ay, (float)az) * 57.2957795f;
    sample->pitch_deg = atan2f(-(float)ax,
        sqrtf((float)ay * (float)ay + (float)az * (float)az)) * 57.2957795f;
    sample->gz_dps = ((float)gz / GYRO_SCALE) - gz_offset;
    return true;
}

void mpu6050_calibrate_gyro(float *gz_offset) {
    float sum_gz = 0.0f;
    int valid = 0;

    for (int i = 0; i < 150; i++) {
        uint8_t data[14];
        if (mpu6050_read_burst(data)) {
            int16_t gz = (int16_t)((data[12] << 8) | data[13]);
            sum_gz += (float)gz / GYRO_SCALE;
            valid++;
        }
        watchdog_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    *gz_offset = (valid > 0) ? (sum_gz / (float)valid) : 0.0f;
}
