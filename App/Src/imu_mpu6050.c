#include "imu_mpu6050.h"
#include "board_config.h"
#include "config.h"
#include <math.h>

#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_INT_PIN_CFG     0x37
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

#define MPU_I2C_ADDR            (IMU_I2C_ADDR_7BIT << 1)

static I2C_HandleTypeDef *s_hi2c = NULL;
static bool s_calibrated = false;
static float s_gyro_bias = 0.0f;
static float s_pitch_deg = 0.0f;
static float s_gyro_rate_dps = 0.0f;
static uint32_t s_consecutive_errors = 0;
static volatile bool s_data_ready_flag = false;

static bool mpu_write_reg(uint8_t reg, uint8_t val)
{
    if (!s_hi2c) return false;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(s_hi2c, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
    return (status == HAL_OK);
}

static bool mpu_read_bytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!s_hi2c) return false;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(s_hi2c, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
    return (status == HAL_OK);
}

bool imu_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_calibrated = false;
    s_consecutive_errors = 0;
    s_pitch_deg = 0.0f;
    s_gyro_rate_dps = 0.0f;

    uint8_t who_am_i = 0;
    if (!mpu_read_bytes(MPU_REG_WHO_AM_I, &who_am_i, 1) || (who_am_i != 0x68 && who_am_i != 0x72 && who_am_i != 0x70)) {
        s_consecutive_errors++;
        return false;
    }

    /* Reset device */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(50);

    /* Wake up, select PLL with X-axis gyroscope reference */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* Config DLPF: ~42 Hz bandwidth */
    mpu_write_reg(MPU_REG_CONFIG, 0x03);

    /* Sample rate divider = 4 -> 1 kHz / (1 + 4) = 200 Hz */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x04);

    /* Gyro scale: ±500 deg/s (65.5 LSB/(deg/s)) */
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x08);

    /* Accel scale: ±2g (16384 LSB/g) */
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    /* Interrupt pin: Active High, Push-Pull, 50us pulse */
    mpu_write_reg(MPU_REG_INT_PIN_CFG, 0x00);

    /* Enable Data Ready interrupt */
    mpu_write_reg(MPU_REG_INT_ENABLE, 0x01);

    return true;
}

void imu_notify_data_ready(void)
{
    s_data_ready_flag = true;
}

bool imu_calibrate(uint16_t sample_count)
{
    if (!s_hi2c || sample_count == 0) return false;

    float sum_gyro = 0.0f;
    uint16_t valid_samples = 0;
    uint8_t buf[14];

    for (uint16_t i = 0; i < sample_count; i++) {
        if (mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
            int16_t raw_gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t raw_gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t raw_gyro = (IMU_TILT_AXIS == 0) ? raw_gyro_y : raw_gyro_x;
            sum_gyro += (float)raw_gyro / 65.5f;
            valid_samples++;
        }
        HAL_Delay(2);
    }

    if (valid_samples < (sample_count * 3 / 4)) {
        s_calibrated = false;
        return false;
    }

    s_gyro_bias = sum_gyro / (float)valid_samples;
    s_calibrated = true;
    s_consecutive_errors = 0;
    return true;
}

bool imu_read(float dt_s)
{
    if (!s_hi2c) return false;
    if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = CONTROL_DT_S;

    uint8_t buf[14];
    if (!mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
        s_consecutive_errors++;
        return false;
    }

    s_consecutive_errors = 0;

    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);

    float ax = (float)raw_ax / 16384.0f;
    float ay = (float)raw_ay / 16384.0f;
    float az = (float)raw_az / 16384.0f;

    float gyro_raw_dps;
    float acc_angle_deg;

    if (IMU_TILT_AXIS == 0) {
        /* Pitch derived from ax and az, gyro from gy */
        acc_angle_deg = atan2f(-ax, az) * (180.0f / 3.14159265f);
        gyro_raw_dps = ((float)raw_gy / 65.5f) - s_gyro_bias;
    } else {
        /* Pitch derived from ay and az, gyro from gx */
        acc_angle_deg = atan2f(ay, az) * (180.0f / 3.14159265f);
        gyro_raw_dps = ((float)raw_gx / 65.5f) - s_gyro_bias;
    }

    acc_angle_deg *= IMU_TILT_SIGN;
    gyro_raw_dps  *= IMU_GYRO_SIGN;

    s_gyro_rate_dps = gyro_raw_dps;

    /* Complementary Filter */
    const float alpha = IMU_FILTER_ALPHA;
    s_pitch_deg = alpha * (s_pitch_deg + s_gyro_rate_dps * dt_s) + (1.0f - alpha) * acc_angle_deg;

    return true;
}

float imu_get_pitch_deg(void)
{
    return s_pitch_deg;
}

float imu_get_gyro_rate_dps(void)
{
    return s_gyro_rate_dps;
}

bool imu_is_calibrated(void)
{
    return s_calibrated;
}

uint32_t imu_get_error_count(void)
{
    return s_consecutive_errors;
}

bool imu_diagnostics(uint8_t *out_who_am_i, float *out_bias, float *out_ax, float *out_ay, float *out_az, float *out_gx, float *out_gy, float *out_gz)
{
    if (!s_hi2c) return false;

    uint8_t who = 0;
    bool who_ok = mpu_read_bytes(MPU_REG_WHO_AM_I, &who, 1);
    if (out_who_am_i) *out_who_am_i = who;
    if (out_bias) *out_bias = s_gyro_bias;

    uint8_t buf[14];
    if (mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
        int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
        int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

        if (out_ax) *out_ax = (float)raw_ax / 16384.0f;
        if (out_ay) *out_ay = (float)raw_ay / 16384.0f;
        if (out_az) *out_az = (float)raw_az / 16384.0f;
        if (out_gx) *out_gx = (float)raw_gx / 65.5f;
        if (out_gy) *out_gy = (float)raw_gy / 65.5f;
        if (out_gz) *out_gz = (float)raw_gz / 65.5f;
        return who_ok && (who == 0x68 || who == 0x70 || who == 0x72);
    }
    return false;
}
