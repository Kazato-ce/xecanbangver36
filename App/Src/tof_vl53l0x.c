#include "tof_vl53l0x.h"
#include "config.h"

#define VL53L0X_ADDR_7BIT              0x29
#define VL53L0X_ADDR                   (VL53L0X_ADDR_7BIT << 1)

#define REG_IDENTIFICATION_MODEL_ID    0xC0
#define REG_SYSRANGE_START             0x00
#define REG_RESULT_INTERRUPT_STATUS    0x13
#define REG_RESULT_RANGE_STATUS        0x14
#define REG_SYSTEM_INTERRUPT_CLEAR     0x0B
#define REG_RESULT_RANGE_MM            0x1E

#define RANGE_STATUS_SIGNAL_FAIL       4U
#define RANGE_STATUS_OUT_OF_BOUNDS    7U

static I2C_HandleTypeDef *s_hi2c = NULL;
static volatile uint16_t s_distance_mm = 8190U;
static volatile bool s_is_valid = false;
static volatile bool s_new_sample = false;
static volatile bool s_cliff_sample_candidate = false;
static volatile uint8_t s_cliff_hits = 0U;
static volatile uint8_t s_safe_hits = 0U;
static volatile bool s_cliff_detected = false;
static uint32_t s_last_read_ms = 0U;
static uint32_t s_last_init_attempt_ms = 0U;
static bool s_measuring = false;
static bool s_initialized = false;

static void tof_invalidate_sample(void)
{
    s_is_valid = false;
    s_new_sample = false;
    s_cliff_sample_candidate = false;
    s_cliff_hits = 0U;
    s_safe_hits = 0U;
}

static bool tof_read_reg8(uint8_t reg, uint8_t *val)
{
    if (!s_hi2c) return false;
    return (HAL_I2C_Mem_Read(s_hi2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             val, 1U, 5U) == HAL_OK);
}

static bool tof_write_reg8(uint8_t reg, uint8_t val)
{
    if (!s_hi2c) return false;
    return (HAL_I2C_Mem_Write(s_hi2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &val, 1U, 5U) == HAL_OK);
}

bool tof_init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_initialized = false;
    s_last_read_ms = 0U;
    s_last_init_attempt_ms = HAL_GetTick();
    s_measuring = false;
    tof_invalidate_sample();

    if (!s_hi2c) return false;

    uint8_t id = 0U;
    if (!tof_read_reg8(REG_IDENTIFICATION_MODEL_ID, &id) || id != 0xEEU) {
        return false;
    }

    if (!tof_write_reg8(0x88U, 0x00U) ||
        !tof_write_reg8(0x80U, 0x01U) ||
        !tof_write_reg8(0xFFU, 0x01U) ||
        !tof_write_reg8(0x00U, 0x00U)) {
        tof_invalidate_sample();
        return false;
    }

    s_initialized = true;
    return true;
}

void tof_update(uint32_t current_time_ms)
{
    if (!s_hi2c) return;

    if (!s_initialized) {
        if ((current_time_ms - s_last_init_attempt_ms) >= TOF_INIT_RETRY_MS) {
            (void)tof_init(s_hi2c);
        }
        return;
    }

    if (!s_measuring) {
        if ((current_time_ms - s_last_read_ms) >= TOF_READ_INTERVAL_MS) {
            s_last_read_ms = current_time_ms;
            if (tof_write_reg8(REG_SYSRANGE_START, 0x01U)) {
                s_measuring = true;
            } else {
                tof_invalidate_sample();
            }
        }
        return;
    }

    if ((current_time_ms - s_last_read_ms) > TOF_MEASUREMENT_TIMEOUT_MS) {
        s_measuring = false;
        tof_invalidate_sample();
        return;
    }

    uint8_t interrupt_status = 0U;
    if (!tof_read_reg8(REG_RESULT_INTERRUPT_STATUS, &interrupt_status)) {
        s_measuring = false;
        tof_invalidate_sample();
        return;
    }

    if ((interrupt_status & 0x07U) == 0U) {
        return;
    }

    uint8_t range_status = 0U;
    uint8_t range_bytes[2] = { 0U, 0U };
    bool read_ok = tof_read_reg8(REG_RESULT_RANGE_STATUS, &range_status);
    if (read_ok) {
        read_ok = (HAL_I2C_Mem_Read(s_hi2c, VL53L0X_ADDR, REG_RESULT_RANGE_MM,
                                    I2C_MEMADD_SIZE_8BIT, range_bytes, 2U, 5U) == HAL_OK);
    }
    bool clear_ok = tof_write_reg8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01U);
    s_measuring = false;

    if (!read_ok || !clear_ok) {
        tof_invalidate_sample();
        return;
    }

    uint16_t range_mm = (uint16_t)(((uint16_t)range_bytes[0] << 8U) | range_bytes[1]);
    uint8_t range_status_code = (uint8_t)((range_status >> 3U) & 0x0FU);
    bool no_reflection = (range_status_code == RANGE_STATUS_SIGNAL_FAIL) ||
                         (range_status_code == RANGE_STATUS_OUT_OF_BOUNDS) ||
                         (range_mm == 8190U);
    bool valid_range = (range_status_code == 0U) &&
                       (range_mm > 0U) &&
                       (range_mm < TOF_MAX_VALID_RANGE_MM);

    if (valid_range) {
        s_distance_mm = range_mm;
    }
    s_is_valid = valid_range;
    s_cliff_sample_candidate = no_reflection ||
                               ((range_status_code == 0U) &&
                                (range_mm > TOF_CLIFF_LIMIT_MM) &&
                                (range_mm != UINT16_MAX));
    s_new_sample = true;
}

bool tof_is_available(void)
{
    return s_initialized;
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
    (void)pitch_deg;
    if (!s_new_sample) return;
    s_new_sample = false;

    if (s_cliff_sample_candidate) {
        s_safe_hits = 0U;
        if (s_cliff_hits < TOF_CLIFF_CONFIRM_SAMPLES) {
            s_cliff_hits++;
        }
        if (s_cliff_hits >= TOF_CLIFF_CONFIRM_SAMPLES) {
            s_cliff_detected = true;
        }
    } else {
        s_cliff_hits = 0U;
        if (s_cliff_detected) {
            if (s_safe_hits < TOF_CLIFF_CONFIRM_SAMPLES) {
                s_safe_hits++;
            }
            if (s_safe_hits >= TOF_CLIFF_CONFIRM_SAMPLES) {
                s_cliff_detected = false;
                s_safe_hits = 0U;
            }
        } else {
            s_safe_hits = 0U;
        }
    }
}

bool tof_is_cliff_detected(void)
{
    return s_cliff_detected;
}
