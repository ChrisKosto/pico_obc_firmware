#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temperature_c;
    float roll_deg;
    float pitch_deg;
    float gz_dps;
} mpu6050_sample_t;

void mpu6050_bus_init(void);
bool mpu6050_init(void);
bool mpu6050_read_sample(float gz_offset, mpu6050_sample_t *sample);
/* Returns false on missing samples, excessive motion, or invalid sensor data.
 * A failed calibration MUST NOT silently assign zero bias.
 */
bool mpu6050_calibrate_gyro(float *gz_offset);

#endif /* MPU6050_H */