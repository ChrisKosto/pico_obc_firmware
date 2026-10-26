/*
 * GY-521 / MPU-6050-style IMU driver.
 * WHO_AM_I=0x72 is treated as an UNKNOWN, PROVISIONAL device, not a verified
 * MPU-6050. The compatibility path is an explicit lab-only opt-in.
 * Every accepted device must pass register readback and stationary calibration.
 */
#include "mpu6050.h"

#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include "obc_config.h"

#define IMU_I2C                 i2c0
#define REG_CONFIG              0x1Au
#define REG_GYRO_CONFIG         0x1Bu
#define REG_ACCEL_CONFIG        0x1Cu
#define REG_ACCEL_XOUT_H        0x3Bu
#define REG_PWR_MGMT_1          0x6Bu
#define REG_PWR_MGMT_2          0x6Cu
#define REG_WHO_AM_I            0x75u

/* Common MPU-60x0-compatible register settings, verified by read-back. */
#define SET_PWR_1               0x01u  /* Awake, PLL reference */
#define SET_PWR_2               0x00u  /* All axes enabled */
#define SET_CONFIG              0x03u  /* DLPF_CFG=3 */
#define SET_GYRO_CONFIG         0x08u  /* FS_SEL=1, +/- 500 dps */
#define SET_ACCEL_CONFIG        0x00u  /* AFS_SEL=0, +/- 2g */
#define ACCEL_LSB_PER_G         16384.0f
#define RAD_TO_DEG              57.2957795f
#define CALIBRATION_SAMPLES     150u
#define CALIBRATION_MAX_SD_DPS  0.85f  /* Lab threshold; tune from measured noise */
#define CALIBRATION_MAX_SD_G    0.06f  /* Axis-wise variation during calibration */
#define CALIBRATION_MAX_BIAS    20.0f  /* Large bias -> suspect wrong scale/part */

static bool s_probed = false;
static uint8_t s_identity = 0;

void mpu6050_bus_init(void) {
    i2c_init(IMU_I2C, 400u * 1000u);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
}

static bool read_reg(uint8_t reg, uint8_t *out) {
    if (out == NULL) return false;
    int rc = i2c_write_timeout_us(IMU_I2C, MPU6050_ADDR,
                                  &reg, 1u, true, I2C_TIMEOUT_US);
    if (rc != 1) {
        printf("[IMU] register select 0x%02X failed rc=%d\n", reg, rc);
        return false;
    }
    rc = i2c_read_timeout_us(IMU_I2C, MPU6050_ADDR,
                             out, 1u, false, I2C_TIMEOUT_US);
    if (rc != 1) {
        printf("[IMU] register read 0x%02X failed rc=%d\n", reg, rc);
        return false;
    }
    return true;
}

static bool write_verify_reg(uint8_t reg, uint8_t value, uint8_t mask) {
    const uint8_t data[2] = {reg, value};
    const int rc = i2c_write_timeout_us(IMU_I2C, MPU6050_ADDR,
                                        data, sizeof(data), false,
                                        I2C_TIMEOUT_US);
    if (rc != (int)sizeof(data)) {
        printf("[IMU] register write 0x%02X failed rc=%d\n", reg, rc);
        return false;
    }
    uint8_t got = 0;
    if (!read_reg(reg, &got)) return false;
    if ((got & mask) != (value & mask)) {
        printf("[IMU] register 0x%02X verify failed: wrote=0x%02X read=0x%02X mask=0x%02X\n",
               reg, value, got, mask);
        return false;
    }
    printf("[IMU] register 0x%02X=0x%02X verified\n", reg, got);
    return true;
}

/* One atomic register-pointer + data transaction on the same I2C peripheral. */
static bool read_raw(uint8_t out[14]) {
    if (out == NULL) return false;
    const uint8_t reg = REG_ACCEL_XOUT_H;
    if (i2c_write_timeout_us(IMU_I2C, MPU6050_ADDR, &reg, 1u,
                             true, I2C_TIMEOUT_US) != 1) return false;
    return i2c_read_timeout_us(IMU_I2C, MPU6050_ADDR, out, 14u,
                               false, I2C_TIMEOUT_US) == 14;
}

static int16_t i16be(const uint8_t *p) {
    return (int16_t)((uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static bool decode_raw(const uint8_t data[14], float *ax_g, float *ay_g,
                       float *az_g, float *gz_dps, int16_t *temp_raw) {
    bool all_zero = true, all_ff = true;
    for (unsigned i = 0; i < 14u; ++i) {
        if (data[i] != 0u) all_zero = false;
        if (data[i] != 0xFFu) all_ff = false;
    }
    if (all_zero || all_ff) return false;
    *ax_g = (float)i16be(&data[0]) / ACCEL_LSB_PER_G;
    *ay_g = (float)i16be(&data[2]) / ACCEL_LSB_PER_G;
    *az_g = (float)i16be(&data[4]) / ACCEL_LSB_PER_G;
    *gz_dps = (float)i16be(&data[12]) / GYRO_SCALE;
    *temp_raw = i16be(&data[6]);
    const float g2 = (*ax_g * *ax_g) + (*ay_g * *ay_g) + (*az_g * *az_g);
    return isfinite(g2) && isfinite(*gz_dps) && g2 >= 0.04f && g2 <= 16.0f;
}

bool mpu6050_init(void) {
    s_probed = false;
    printf("[IMU] probe: I2C0 addr=0x%02X SDA=GP%d SCL=GP%d\n",
           MPU6050_ADDR, PIN_SDA, PIN_SCL);

    /* Require a stable identity over separate transactions. */
    uint8_t who = 0;
    for (unsigned i = 0; i < 3u; ++i) {
        uint8_t identity = 0;
        if (!read_reg(REG_WHO_AM_I, &identity)) return false;
        if (i > 0u && identity != who) {
            printf("[IMU] WHO_AM_I unstable: 0x%02X vs 0x%02X\n", who, identity);
            return false;
        }
        who = identity;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("[IMU] stable WHO_AM_I=0x%02X\n", who);
    if (who == 0x68u) {
        printf("[IMU] standard MPU6050 identity\n");
    } else if (who == 0x72u && MPU6050_ALLOW_PROVISIONAL_ID_72) {
        printf("[IMU] WARNING: 0x72 is UNKNOWN silicon; lab compatibility trial only\n");
    } else {
        printf("[IMU] unsupported identity; IMU remains invalid\n");
        return false;
    }

    /* Verify the overlapping register map, not just the identity value. */
    if (!write_verify_reg(REG_PWR_MGMT_1, SET_PWR_1, 0x7Fu)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!write_verify_reg(REG_PWR_MGMT_2, SET_PWR_2, 0x3Fu) ||
        !write_verify_reg(REG_CONFIG, SET_CONFIG, 0x07u) ||
        !write_verify_reg(REG_GYRO_CONFIG, SET_GYRO_CONFIG, 0x18u) ||
        !write_verify_reg(REG_ACCEL_CONFIG, SET_ACCEL_CONFIG, 0x18u)) return false;

    /* Verify the data register burst is populated and gives plausible gravity. */
    unsigned plausible = 0;
    for (unsigned i = 0; i < 5u; ++i) {
        uint8_t data[14];
        float ax, ay, az, gz;
        int16_t temp;
        if (read_raw(data) && decode_raw(data, &ax, &ay, &az, &gz, &temp)) {
            const float g = sqrtf(ax*ax + ay*ay + az*az);
            if (g > 0.65f && g < 1.35f && fabsf(gz) < 100.0f) plausible++;
            if (i == 0u) {
                printf("[IMU] preview: accel=%.3fg gyroZ=%+.3fdps rawTemp=%d\n",
                       (double)g, (double)gz, (int)temp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (plausible < 4u) {
        printf("[IMU] compatibility check failed: %u/5 plausible stationary samples\n",
               plausible);
        return false;
    }
    s_identity = who;
    s_probed = true;
    printf("[IMU] register map + stationary data checks PASS (ID=0x%02X)\n", who);
    return true;
}

bool mpu6050_read_sample(float gz_offset, mpu6050_sample_t *sample) {
    if (!s_probed || sample == NULL || !isfinite(gz_offset)) return false;
    uint8_t raw[14];
    float ax, ay, az, gz;
    int16_t temp_raw;
    if (!read_raw(raw) || !decode_raw(raw, &ax, &ay, &az, &gz, &temp_raw))
        return false;

    /* These conversion factors are MPU6050-family ASSUMPTIONS for ID 0x72.
     * Lab characterization against a reference is still required. */
    sample->temperature_c = (float)temp_raw / 340.0f + 36.53f;
    sample->roll_deg = atan2f(ay, az) * RAD_TO_DEG;
    sample->pitch_deg = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
    sample->gz_dps = gz - gz_offset;
    return isfinite(sample->temperature_c) &&
           isfinite(sample->roll_deg) && isfinite(sample->pitch_deg) &&
           isfinite(sample->gz_dps);
}

typedef struct {
    unsigned n;
    float mean;
    float m2;
} running_stat_t;

static void update_stat(running_stat_t *s, float x) {
    ++s->n;
    float delta = x - s->mean;
    s->mean += delta / (float)s->n;
    s->m2 += delta * (x - s->mean);
}
static float stddev(const running_stat_t *s) {
    return s->n > 1u ? sqrtf(s->m2 / (float)(s->n - 1u)) : INFINITY;
}

bool mpu6050_calibrate_gyro(float *gz_offset) {
    if (gz_offset == NULL || !s_probed) return false;
    /* Zero bias on failure must NEVER be treated as successful calibration. */
    *gz_offset = 0.0f;
    running_stat_t sx = {0}, sy = {0}, sz = {0}, sgyro = {0};
    printf("[IMU] calibration: %u stationary samples (DO NOT MOVE)\n",
           CALIBRATION_SAMPLES);
    for (unsigned i = 0; i < CALIBRATION_SAMPLES; ++i) {
        uint8_t raw[14];
        float ax, ay, az, gz;
        int16_t rt;
        if (!read_raw(raw) || !decode_raw(raw, &ax, &ay, &az, &gz, &rt)) {
            printf("[IMU] calibration failure: I2C/data at sample %u\n", i);
            return false;
        }
        const float g = sqrtf(ax*ax + ay*ay + az*az);
        if (g < 0.85f || g > 1.15f) {
            printf("[IMU] calibration failure: gravity %.3fg at sample %u\n",
                   (double)g, i);
            return false;
        }
        update_stat(&sx, ax); update_stat(&sy, ay); update_stat(&sz, az);
        update_stat(&sgyro, gz);
        if (i == 0u || i == 74u || i == CALIBRATION_SAMPLES - 1u) {
            printf("[IMU] calib %u: norm=%.3fg gyroZ=%+.4fdps\n",
                   i, (double)g, (double)gz);
        }
        /* Bounded calibration; watchdog remains owned by the supervisor. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    const float gx = stddev(&sx), gy = stddev(&sy), gz = stddev(&sz);
    const float gyro_noise = stddev(&sgyro);
    printf("[IMU] calib stats: bias=%+.4f dps sigmaZ=%.4f dps accelSigma=(%.4f, %.4f, %.4f)g\n",
           (double)sgyro.mean, (double)gyro_noise,
           (double)gx, (double)gy, (double)gz);
    if (!isfinite(sgyro.mean) || !isfinite(gyro_noise) ||
        fabsf(sgyro.mean) > CALIBRATION_MAX_BIAS ||
        gyro_noise > CALIBRATION_MAX_SD_DPS ||
        gx > CALIBRATION_MAX_SD_G || gy > CALIBRATION_MAX_SD_G ||
        gz > CALIBRATION_MAX_SD_G) {
        printf("[IMU] calibration REJECTED: moving/noisy/unknown device\n");
        return false;
    }
    *gz_offset = sgyro.mean;
    printf("[IMU] CALIBRATION PASS id=0x%02X biasZ=%+.4f dps (provisional=%u)\n",
           s_identity, (double)*gz_offset, (unsigned)(s_identity == 0x72u));
    return true;
}