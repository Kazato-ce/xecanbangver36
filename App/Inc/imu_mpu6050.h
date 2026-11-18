#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool imu_init(I2C_HandleTypeDef *hi2c);
bool imu_calibrate(uint16_t sample_count);
bool imu_read(float dt_s);

float imu_get_pitch_deg(void);
float imu_get_gyro_rate_dps(void);
bool  imu_is_calibrated(void);
uint32_t imu_get_error_count(void);
void imu_notify_data_ready(void);
bool imu_diagnostics(uint8_t *out_who_am_i, float *out_bias, float *out_ax, float *out_ay, float *out_az, float *out_gx, float *out_gy, float *out_gz);

#endif /* IMU_MPU6050_H */
