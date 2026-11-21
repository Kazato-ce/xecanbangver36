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
#define IMU_YAW_SIGN            1.0f

static I2C_HandleTypeDef *s_hi2c = NULL;
static bool s_calibrated = false;
static float s_gyro_bias = 0.0f;
static float s_gyro_z_bias = 0.0f;
static float s_pitch_deg = 0.0f;
static float s_gyro_rate_dps = 0.0f;
static float s_gyro_z_dps = 0.0f;
static float s_yaw_deg = 0.0f;
static uint32_t s_consecutive_errors = 0;
static volatile bool s_data_ready_flag = false;

static bool mpu_write_reg(uint8_t reg, uint8_t val)
{
    if (!s_hi2c) return false;
    uint8_t buf[2] = { reg, val };
    return (HAL_I2C_Master_Transmit(s_hi2c, MPU_I2C_ADDR, buf, 2, 20) == HAL_OK);
}

static bool mpu_read_bytes(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!s_hi2c || !data) return false;
    if (HAL_I2C_Master_Transmit(s_hi2c, MPU_I2C_ADDR, &reg, 1, 20) != HAL_OK) return false;
    return (HAL_I2C_Master_Receive(s_hi2c, MPU_I2C_ADDR, data, len, 50) == HAL_OK);
}

bool imu_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_calibrated = false;
    s_consecutive_errors = 0;
    s_pitch_deg = 0.0f;
    s_gyro_rate_dps = 0.0f;
    s_gyro_z_dps = 0.0f;
    s_yaw_deg = 0.0f;

    /* 1. Kiểm tra WHO_AM_I (0x68 là default AD0=0, 0x69 là AD0=1, 0x70/0x72 là bản clone MPU6500) */
    uint8_t who_am_i = 0;
    if (!mpu_read_bytes(MPU_REG_WHO_AM_I, &who_am_i, 1) ||
        (who_am_i != 0x68 && who_am_i != 0x69 && who_am_i != 0x70 && who_am_i != 0x72 && who_am_i != 0x98)) {
        s_consecutive_errors++;
        return false;
    }

    /* 2. Reset device */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 3. Đánh thức và chọn Clock Source PLL Gyro X */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(15);

    /* 4. Cấu hình Sample Rate Divider = 0 (1 kHz) */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x00);

    /* 5. Cấu hình DLPF: ~42 Hz bandwidth */
    mpu_write_reg(MPU_REG_CONFIG, 0x03);

    /* 6. Gyro scale: ±500 deg/s (65.5 LSB/(deg/s)) */
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x08);

    /* 7. Accel scale: ±4g (8192 LSB/g) */
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x08);

    /* 8. Cấu hình chân ngắt INT (Active High, Push-Pull, 50us pulse) */
    mpu_write_reg(MPU_REG_INT_PIN_CFG, 0x00);

    /* 9. Bật ngắt Data Ready */
    mpu_write_reg(MPU_REG_INT_ENABLE, 0x01);

    /* 10. Đọc mẫu gia tốc khởi tạo góc ngay khi boot */
    uint8_t raw[14];
    if (mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, raw, 14)) {
        int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);
        float ax = (float)raw_ax / 8192.0f;
        float ay = (float)raw_ay / 8192.0f;
        float az = (float)raw_az / 8192.0f;
        float acc_angle = (IMU_TILT_AXIS == 0) ? (atan2f(-ax, az) * 57.2957795f) : (atan2f(ay, az) * 57.2957795f);
        s_pitch_deg = acc_angle * IMU_TILT_SIGN;
    }

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
    float sum_gyro_z = 0.0f;
    uint16_t valid_samples = 0;
    uint8_t buf[14];

    for (uint16_t i = 0; i < sample_count; i++) {
        if (mpu_read_bytes(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
            int16_t raw_gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
            int16_t raw_gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
            int16_t raw_gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
            int16_t raw_gyro = (IMU_TILT_AXIS == 0) ? raw_gyro_y : raw_gyro_x;
            sum_gyro += (float)raw_gyro / 65.5f;
            sum_gyro_z += (float)raw_gyro_z / 65.5f;
            valid_samples++;
        }
        HAL_Delay(2);
    }

    uint16_t minimum_samples = (uint16_t)(sample_count - (sample_count / 4U));
    if (valid_samples < minimum_samples) {
        s_calibrated = false;
        return false;
    }

    s_gyro_bias = sum_gyro / (float)valid_samples;
    s_gyro_z_bias = sum_gyro_z / (float)valid_samples;
    s_yaw_deg = 0.0f;
    s_gyro_z_dps = 0.0f;
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
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    float ax = (float)raw_ax / 8192.0f;
    float ay = (float)raw_ay / 8192.0f;
    float az = (float)raw_az / 8192.0f;

    float gyro_raw_dps;
    float acc_angle_deg;

    if (IMU_TILT_AXIS == 0) {
        /* Pitch từ ax và az, gyro từ gy */
        acc_angle_deg = atan2f(-ax, az) * 57.2957795f;
        gyro_raw_dps  = ((float)raw_gy / 65.5f) - s_gyro_bias;
    } else {
        /* Pitch từ ay và az, gyro từ gx */
        acc_angle_deg = atan2f(ay, az) * 57.2957795f;
        gyro_raw_dps  = ((float)raw_gx / 65.5f) - s_gyro_bias;
    }

    acc_angle_deg *= IMU_TILT_SIGN;
    gyro_raw_dps  *= IMU_GYRO_SIGN;

    s_gyro_rate_dps = gyro_raw_dps;

    /* Tích hợp góc quay Yaw Z */
    float gz_dps = ((float)raw_gz / 65.5f) - s_gyro_z_bias;
    if (fabsf(gz_dps) <= 0.25f) {
        gz_dps = 0.0f;
    }
    gz_dps *= IMU_YAW_SIGN;
    s_gyro_z_dps = gz_dps;
    s_yaw_deg += gz_dps * dt_s;

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

float imu_get_yaw_deg(void)
{
    return s_yaw_deg;
}

float imu_get_gyro_z_dps(void)
{
    return s_gyro_z_dps;
}

void imu_reset_yaw(void)
{
    s_yaw_deg = 0.0f;
}

bool imu_is_calibrated(void)
{
    return s_calibrated;
}

uint32_t imu_get_error_count(void)
{
    return s_consecutive_errors;
}
