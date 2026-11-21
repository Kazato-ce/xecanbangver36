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
float imu_get_yaw_deg(void);
float imu_get_gyro_z_dps(void);
void  imu_reset_yaw(void);
bool  imu_is_calibrated(void);
uint32_t imu_get_error_count(void);
void imu_notify_data_ready(void);

#endif /* IMU_MPU6050_H */
