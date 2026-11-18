#include "tof_vl53l0x.h"
#include "board_config.h"
#include "config.h"

#define VL53L0X_ADDR_7BIT           0x29
#define VL53L0X_ADDR                (VL53L0X_ADDR_7BIT << 1)

#define REG_IDENTIFICATION_MODEL_ID 0xC0
#define REG_SYSRANGE_START          0x00
#define REG_RESULT_RANGE_STATUS     0x14

static I2C_HandleTypeDef *s_hi2c = NULL;
static uint16_t s_distance_mm = 8190;
static bool s_is_valid = false;
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
    s_measuring = false;
    s_last_read_ms = 0;

    uint8_t model_id = 0;
    if (!tof_read_reg8(REG_IDENTIFICATION_MODEL_ID, &model_id) || model_id != 0xEE) {
        return false; /* VL53L0X Model ID không khớp hoặc không có phản hồi trên I2C2 */
    }

    /* Đưa sensor vào chế độ sẵn sàng */
    tof_write_reg8(0x88, 0x00);
    tof_write_reg8(0x80, 0x01);
    tof_write_reg8(0xFF, 0x01);
    tof_write_reg8(0x00, 0x00);
    tof_write_reg8(0x91, 0x3C);
    tof_write_reg8(0x00, 0x01);
    tof_write_reg8(0xFF, 0x00);
    tof_write_reg8(0x80, 0x00);

    return true;
}

void tof_update(uint32_t current_time_ms)
{
    if (!s_hi2c) return;

    if (!s_measuring) {
        if ((current_time_ms - s_last_read_ms) >= TOF_READ_INTERVAL_MS) {
            /* Kích hoạt đo khoảng cách Single Shot */
            tof_write_reg8(REG_SYSRANGE_START, 0x01);
            s_measuring = true;
            s_last_read_ms = current_time_ms;
        }
    } else {
        /* Kiểm tra cờ Data Ready */
        uint8_t status = 0;
        if (tof_read_reg8(REG_RESULT_RANGE_STATUS, &status)) {
            if (status & 0x01) { /* Measurement finished */
                uint8_t buf[2];
                if (HAL_I2C_Mem_Read(s_hi2c, VL53L0X_ADDR, 0x14 + 10, I2C_MEMADD_SIZE_8BIT, buf, 2, 5) == HAL_OK) {
                    uint16_t range = (uint16_t)((buf[0] << 8) | buf[1]);
                    if (range < 8000) {
                        s_distance_mm = range;
                        s_is_valid = true;
                    }
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
