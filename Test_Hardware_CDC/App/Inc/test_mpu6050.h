#ifndef TEST_MPU6050_H
#define TEST_MPU6050_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool  test_mpu_init(I2C_HandleTypeDef *hi2c);
bool  test_mpu_check_whoami(uint8_t *whoami);
bool  test_mpu_read_all(float *ax_g, float *ay_g, float *az_g, float *gx_dps, float *gy_dps, float *gz_dps, float *temp_c);
float test_mpu_update_pitch(float dt_s);
float test_mpu_get_pitch(void);
bool  test_mpu_calibrate(uint16_t samples, float *bias_x, float *bias_y, float *bias_z);
bool  test_mpu_is_int_active(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_MPU6050_H */
