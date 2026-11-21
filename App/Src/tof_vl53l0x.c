#include "tof_vl53l0x.h"
#include "board_config.h"
#include "config.h"
#include <math.h>

#define VL53L0X_ADDR_7BIT           0x29
#define VL53L0X_ADDR                (VL53L0X_ADDR_7BIT << 1)

#define REG_IDENTIFICATION_MODEL_ID 0xC0
#define REG_SYSRANGE_START          0x00
#define REG_RESULT_RANGE_STATUS     0x14

#define TOF_MOUNT_ANGLE_DEG         20.0f
#define TOF_CLIFF_LIMIT_MM          80.0f
#define TOF_CLIFF_CONFIRM_TICKS     3U

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint16_t s_distance_mm = 8190;
static bool s_is_valid = false;
static bool s_new_sample = false;
static uint8_t s_cliff_hits = 0;
static bool s_cliff_detected = false;
static uint32_t s_last_read_ms = 0;
static bool s_measuring = false;

static bool tof_read_reg8(uint8_t reg, uint8_t *val)
{
    if (!s_hi2c) return false;
    return (HAL_I2C_Mem_Read(s_hi2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 5) == HAL_OK);
}

static bool tof_write_reg8(uint8_t reg, uint8_t val)
{
    if (!s_hi2c) return false;
    return (HAL_I2C_Mem_Write(s_hi2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 5) == HAL_OK);
}

bool tof_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_distance_mm = 8190;
    s_is_valid = false;
    s_new_sample = false;
    s_cliff_hits = 0;
    s_cliff_detected = false;
    s_last_read_ms = 0;
    s_measuring = false;

    if (!s_hi2c) return false;

    /* Kiểm tra Model ID (0xEE) */
    uint8_t id = 0;
    if (!tof_read_reg8(REG_IDENTIFICATION_MODEL_ID, &id) || id != 0xEE) {
        return false;
    }

    /* Khởi tạo cơ bản: Standard 2V8 mode, I2C standard mode */
    tof_write_reg8(0x88, 0x00);
    tof_write_reg8(0x80, 0x01);
    tof_write_reg8(0xFF, 0x01);
    tof_write_reg8(0x00, 0x00);

    return true;
}

void tof_update(uint32_t current_time_ms)
{
    if (!s_hi2c) return;

    if (!s_measuring) {
        /* Bắt đầu đo đơn (Single shot) mỗi TOF_READ_INTERVAL_MS */
        if ((current_time_ms - s_last_read_ms) >= TOF_READ_INTERVAL_MS) {
            s_last_read_ms = current_time_ms;
            /* Ghi 0x01 vào SYSRANGE_START để bắt đầu đo */
            if (tof_write_reg8(REG_SYSRANGE_START, 0x01)) {
                s_measuring = true;
            }
        }
    } else {
        /* Timeout 50ms nếu đo thất bại */
        if ((current_time_ms - s_last_read_ms) > 50) {
            s_measuring = false;
            return;
        }

        /* Kiểm tra cờ Data Ready */
        uint8_t status = 0;
        if (tof_read_reg8(REG_RESULT_RANGE_STATUS, &status)) {
            if (status & 0x01) { /* Measurement finished */
                uint8_t buf[2];
                if (HAL_I2C_Mem_Read(s_hi2c, VL53L0X_ADDR, 0x14 + 10, I2C_MEMADD_SIZE_8BIT, buf, 2, 5) == HAL_OK) {
                    uint16_t range = (uint16_t)((buf[0] << 8) | buf[1]);
                    s_distance_mm = range;
                    s_is_valid = true;
                    s_new_sample = true;
                }
                /* Xóa cờ ngắt */
                tof_write_reg8(0x0B, 0x01);
                s_measuring = false;
            }
        } else {
            s_measuring = false;
        }
    }
}

uint16_t tof_get_distance_mm(void)
{
    return s_distance_mm;
}

bool tof_is_valid(void)
{
    return s_is_valid;
}

void tof_evaluate_cliff(float pitch_deg)
{
    if (!s_new_sample) return;
    s_new_sample = false;

    bool raw_cliff = false;
    if (!s_is_valid || s_distance_mm >= 8000U) {
        raw_cliff = true;
    } else {
        float rad = (TOF_MOUNT_ANGLE_DEG + pitch_deg) * 0.01745329f;
        float cos_a = cosf(rad);
        if (cos_a < 0.25f) cos_a = 0.25f;
        float vertical_mm = (float)s_distance_mm * cos_a;
        if (vertical_mm > TOF_CLIFF_LIMIT_MM) {
            raw_cliff = true;
        }
    }

    if (raw_cliff) {
        if (s_cliff_hits < TOF_CLIFF_CONFIRM_TICKS) {
            s_cliff_hits++;
        }
    } else {
        s_cliff_hits = 0;
    }

    s_cliff_detected = (s_cliff_hits >= TOF_CLIFF_CONFIRM_TICKS);
}

bool tof_is_cliff_detected(void)
{
    return s_cliff_detected;
}
