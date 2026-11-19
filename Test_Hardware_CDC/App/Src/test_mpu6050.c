#include "test_mpu6050.h"
#include <math.h>

#define MPU_ADDR_7BIT           0x68
#define MPU_ADDR_8BIT           (MPU_ADDR_7BIT << 1)

#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_INT_ENABLE          0x38
#define REG_ACCEL_XOUT_H        0x3B
#define REG_PWR_MGMT_1          0x6B
#define REG_WHO_AM_I            0x75

#define ACCEL_SCALE_4G          8192.0f   /* LSB per g for ±4g */
#define GYRO_SCALE_500DPS       65.5f     /* LSB per deg/s for ±500dps */

static I2C_HandleTypeDef *s_hi2c = NULL;
static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;
static float s_pitch_deg = 0.0f;
static bool  s_initialized = false;

static bool mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return (HAL_I2C_Master_Transmit(s_hi2c, MPU_ADDR_8BIT, buf, 2, 20) == HAL_OK);
}

static bool mpu_read_reg(uint8_t reg, uint8_t *val)
{
    if (HAL_I2C_Master_Transmit(s_hi2c, MPU_ADDR_8BIT, &reg, 1, 20) != HAL_OK) return false;
    return (HAL_I2C_Master_Receive(s_hi2c, MPU_ADDR_8BIT, val, 1, 20) == HAL_OK);
}

static bool mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Master_Transmit(s_hi2c, MPU_ADDR_8BIT, &reg, 1, 20) != HAL_OK) return false;
    return (HAL_I2C_Master_Receive(s_hi2c, MPU_ADDR_8BIT, buf, len, 50) == HAL_OK);
}

bool test_mpu_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_initialized = false;
    s_pitch_deg = 0.0f;

    /* 1. Kiểm tra WHO_AM_I */
    uint8_t whoami = 0;
    if (!test_mpu_check_whoami(&whoami)) {
        return false;
    }

    /* 2. Reset MPU6050 */
    mpu_write_reg(REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    /* 3. Đánh thức và chọn Clock Source: PLL Gyro X (0x01) */
    mpu_write_reg(REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    /* 4. Cấu hình Sample Rate Divider = 0 (1 kHz) */
    mpu_write_reg(REG_SMPLRT_DIV, 0x00);

    /* 5. Cấu hình DLPF: 42 Hz (0x03) */
    mpu_write_reg(REG_CONFIG, 0x03);

    /* 6. Cấu hình Gyro: ±500 dps (0x08) */
    mpu_write_reg(REG_GYRO_CONFIG, 0x08);

    /* 7. Cấu hình Accel: ±4g (0x08) */
    mpu_write_reg(REG_ACCEL_CONFIG, 0x08);

    /* 8. Bật ngắt Data Ready (0x01) */
    mpu_write_reg(REG_INT_ENABLE, 0x01);

    s_initialized = true;
    return true;
}

bool test_mpu_check_whoami(uint8_t *whoami)
{
    if (!s_hi2c) return false;
    uint8_t val = 0;
    if (!mpu_read_reg(REG_WHO_AM_I, &val)) return false;
    if (whoami) *whoami = val;
    return (val == 0x68 || val == 0x69 || val == 0x70 || val == 0x72);
}

bool test_mpu_read_all(float *ax_g, float *ay_g, float *az_g, float *gx_dps, float *gy_dps, float *gz_dps, float *temp_c)
{
    if (!s_initialized) return false;
    uint8_t raw[14];
    if (!mpu_read_bytes(REG_ACCEL_XOUT_H, raw, 14)) return false;

    int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t raw_t  = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t raw_gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

    if (ax_g) *ax_g = (float)raw_ax / ACCEL_SCALE_4G;
    if (ay_g) *ay_g = (float)raw_ay / ACCEL_SCALE_4G;
    if (az_g) *az_g = (float)raw_az / ACCEL_SCALE_4G;

    if (gx_dps) *gx_dps = ((float)raw_gx / GYRO_SCALE_500DPS) - s_gyro_bias_x;
    if (gy_dps) *gy_dps = ((float)raw_gy / GYRO_SCALE_500DPS) - s_gyro_bias_y;
    if (gz_dps) *gz_dps = ((float)raw_gz / GYRO_SCALE_500DPS) - s_gyro_bias_z;

    if (temp_c) *temp_c = ((float)raw_t / 340.0f) + 36.53f;

    return true;
}

float test_mpu_update_pitch(float dt_s)
{
    float ax, ay, az, gx, gy, gz;
    if (!test_mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz, NULL)) {
        return s_pitch_deg;
    }

    /* Góc Pitch từ gia tốc kế (trục X và Z) */
    float pitch_acc = atan2f(-ax, az) * 57.2957795f;

    /* Bộ lọc bù (Complementary filter) kết hợp tốc độ góc Gyro Y */
    s_pitch_deg = 0.98f * (s_pitch_deg + gy * dt_s) + 0.02f * pitch_acc;
    return s_pitch_deg;
}

float test_mpu_get_pitch(void)
{
    return s_pitch_deg;
}

bool test_mpu_calibrate(uint16_t samples, float *bias_x, float *bias_y, float *bias_z)
{
    if (!s_initialized || samples == 0) return false;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++) {
        uint8_t raw[14];
        if (mpu_read_bytes(REG_ACCEL_XOUT_H, raw, 14)) {
            int16_t raw_gx = (int16_t)((raw[8] << 8) | raw[9]);
            int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
            int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

            sum_x += (float)raw_gx / GYRO_SCALE_500DPS;
            sum_y += (float)raw_gy / GYRO_SCALE_500DPS;
            sum_z += (float)raw_gz / GYRO_SCALE_500DPS;
            valid++;
        }
        HAL_Delay(3);
    }

    if (valid < (samples / 2)) return false;

    s_gyro_bias_x = sum_x / (float)valid;
    s_gyro_bias_y = sum_y / (float)valid;
    s_gyro_bias_z = sum_z / (float)valid;

    if (bias_x) *bias_x = s_gyro_bias_x;
    if (bias_y) *bias_y = s_gyro_bias_y;
    if (bias_z) *bias_z = s_gyro_bias_z;

    return true;
}

bool test_mpu_is_int_active(void)
{
    /* INT pin nối PB3 */
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET);
}
