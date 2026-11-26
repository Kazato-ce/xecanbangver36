#include "stepper_tmc2208.h"
#include "board_config.h"
#include "config.h"
#include <math.h>

typedef struct {
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    bool          dir_invert;
    float         target_steps_s;
    float         ramp_steps_s;
    float         applied_steps_s;
    bool          logical_fwd;
    volatile uint32_t dds_acc;
    volatile uint32_t dds_inc;
} MotorState_t;

static MotorState_t s_motors[MOTOR_COUNT];
static volatile bool s_is_enabled = false;
static volatile bool s_estop_tripped = false;
volatile int32_t g_motor_steps_left = 0;
volatile int32_t g_motor_steps_right = 0;

static inline float clampf(float v, float min_val, float max_val)
{
    return (v < min_val) ? min_val : ((v > max_val) ? max_val : v);
}

static void motor_set_dir(MotorState_t *m, bool fwd)
{
    const bool level_high = (fwd != m->dir_invert);
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, level_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    m->logical_fwd = fwd;
}

void stepper_init(void)
{
    /* Rule 1: Khởi tạo PA0 = HIGH (Driver tắt) */
    HAL_GPIO_WritePin(BOARD_MOTOR_EN_PORT, BOARD_MOTOR_EN_PIN, BOARD_MOTOR_EN_DISABLE_LEVEL);

    /* Left Motor */
    s_motors[MOTOR_LEFT].step_port = BOARD_MOTOR_L_STEP_PORT;
    s_motors[MOTOR_LEFT].step_pin  = BOARD_MOTOR_L_STEP_PIN;
    s_motors[MOTOR_LEFT].dir_port  = BOARD_MOTOR_L_DIR_PORT;
    s_motors[MOTOR_LEFT].dir_pin   = BOARD_MOTOR_L_DIR_PIN;
    s_motors[MOTOR_LEFT].dir_invert = (MOTOR_L_DIR_INVERT != 0);

    /* Right Motor */
    s_motors[MOTOR_RIGHT].step_port = BOARD_MOTOR_R_STEP_PORT;
    s_motors[MOTOR_RIGHT].step_pin  = BOARD_MOTOR_R_STEP_PIN;
    s_motors[MOTOR_RIGHT].dir_port  = BOARD_MOTOR_R_DIR_PORT;
    s_motors[MOTOR_RIGHT].dir_pin   = BOARD_MOTOR_R_DIR_PIN;
    s_motors[MOTOR_RIGHT].dir_invert = (MOTOR_R_DIR_INVERT != 0);

    s_is_enabled = false;
    s_estop_tripped = false;
    g_motor_steps_left = 0;
    g_motor_steps_right = 0;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorState_t *m = &s_motors[i];
        motor_set_dir(m, true);
        HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
        m->target_steps_s = 0.0f;
        m->ramp_steps_s = 0.0f;
        m->applied_steps_s = 0.0f;
        m->dds_acc = 0;
        m->dds_inc = 0;
    }
}

void stepper_enable(bool enable)
{
    if (enable == s_is_enabled) return;

    if (enable) {
        s_estop_tripped = false;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            s_motors[i].target_steps_s = 0.0f;
            s_motors[i].ramp_steps_s = 0.0f;
            s_motors[i].applied_steps_s = 0.0f;
            s_motors[i].dds_acc = 0;
            s_motors[i].dds_inc = 0;
        }
        /* PA0 LOW = Bật driver */
        HAL_GPIO_WritePin(BOARD_MOTOR_EN_PORT, BOARD_MOTOR_EN_PIN, BOARD_MOTOR_EN_ACTIVE_LEVEL);
        s_is_enabled = true;
    } else {
        stepper_stop();
    }
}

bool stepper_is_enabled(void)
{
    return s_is_enabled;
}

void stepper_stop(void)
{
    /* Tắt driver ngay: PA0 = HIGH */
    HAL_GPIO_WritePin(BOARD_MOTOR_EN_PORT, BOARD_MOTOR_EN_PIN, BOARD_MOTOR_EN_DISABLE_LEVEL);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorState_t *m = &s_motors[i];
        HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
        m->target_steps_s = 0.0f;
        m->ramp_steps_s = 0.0f;
        m->applied_steps_s = 0.0f;
        m->dds_inc = 0;
        m->dds_acc = 0;
    }
    s_is_enabled = false;
}

void stepper_emergency_stop_isr(void)
{
    /* Ghi trực tiếp thanh ghi BSRR để ngắt driver trong ISR */
    BOARD_MOTOR_EN_PORT->BSRR = BOARD_MOTOR_EN_PIN; /* High = Disable */
    s_motors[MOTOR_LEFT].dds_inc = 0;
    s_motors[MOTOR_RIGHT].dds_inc = 0;
    s_is_enabled = false;
    s_estop_tripped = true;
}

void stepper_set_speed(MotorId_t motor, float steps_per_sec_signed)
{
    if (motor >= MOTOR_COUNT || !isfinite(steps_per_sec_signed)) return;
    s_motors[motor].target_steps_s = steps_per_sec_signed;
}

static void apply_step_rate(MotorState_t *m, float rate_signed)
{
    float abs_rate = fabsf(rate_signed);
    if (abs_rate < 18.0f) { /* Deadband chống rung giật & tiếng rè ở tốc độ siêu chậm quanh 0 */
        m->dds_inc = 0;
        m->applied_steps_s = 0.0f;
        return;
    }
    if (abs_rate > STEP_FREQ_MAX_HZ) {
        abs_rate = STEP_FREQ_MAX_HZ;
    }

    const bool fwd = (rate_signed >= 0.0f);
    if (fwd != m->logical_fwd) {
        motor_set_dir(m, fwd);
    }

    /* DDS phase increment = rate * 2^32 / f_isr */
    uint32_t inc = (uint32_t)(abs_rate * 4294967296.0f / (float)STEP_TICK_FREQ_HZ);
    m->dds_inc = inc;
    m->applied_steps_s = fwd ? abs_rate : -abs_rate;
}

void stepper_update_ramp(float dt_s)
{
    if (!s_is_enabled || s_estop_tripped) return;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f) dt_s = CONTROL_DT_S;

    const float max_step_change = STEP_ACCEL_MAX_HZ_PER_S * dt_s;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        MotorState_t *m = &s_motors[i];
        float target = clampf(m->target_steps_s, -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);
        m->ramp_steps_s += clampf(target - m->ramp_steps_s, -max_step_change, max_step_change);
        apply_step_rate(m, m->ramp_steps_s);
    }
}

float stepper_get_speed(MotorId_t motor)
{
    return (motor < MOTOR_COUNT) ? s_motors[motor].applied_steps_s : 0.0f;
}

void motor_get_steps(int32_t *left, int32_t *right)
{
    if (left) {
        *left = g_motor_steps_left;
    }
    if (right) {
        *right = g_motor_steps_right;
    }
}

static void motor_count_step(volatile int32_t *steps, bool forward)
{
    if (forward) {
        if (*steps == INT32_MAX) {
            *steps = INT32_MIN;
        } else {
            (*steps)++;
        }
    } else {
        if (*steps == INT32_MIN) {
            *steps = INT32_MAX;
        } else {
            (*steps)--;
        }
    }
}

/* 20 kHz interrupt routine called from TIM7 */
void stepper_step_isr(void)
{
    /* Đưa cả 2 chân STEP về LOW trước (xung thấp) */
    BOARD_MOTOR_L_STEP_PORT->BSRR = (uint32_t)BOARD_MOTOR_L_STEP_PIN << 16U;
    BOARD_MOTOR_R_STEP_PORT->BSRR = (uint32_t)BOARD_MOTOR_R_STEP_PIN << 16U;

    if (!s_is_enabled || s_estop_tripped) return;

    uint32_t set_pins_l = 0;
    uint32_t set_pins_r = 0;

    /* Motor Trái */
    uint32_t prev_l = s_motors[MOTOR_LEFT].dds_acc;
    s_motors[MOTOR_LEFT].dds_acc += s_motors[MOTOR_LEFT].dds_inc;
    if (s_motors[MOTOR_LEFT].dds_acc < prev_l) {
        set_pins_l = BOARD_MOTOR_L_STEP_PIN;
    }

    /* Motor Phải */
    uint32_t prev_r = s_motors[MOTOR_RIGHT].dds_acc;
    s_motors[MOTOR_RIGHT].dds_acc += s_motors[MOTOR_RIGHT].dds_inc;
    if (s_motors[MOTOR_RIGHT].dds_acc < prev_r) {
        set_pins_r = BOARD_MOTOR_R_STEP_PIN;
    }

    if (set_pins_l) {
        BOARD_MOTOR_L_STEP_PORT->BSRR = set_pins_l;
        motor_count_step(&g_motor_steps_left, s_motors[MOTOR_LEFT].logical_fwd);
    }
    if (set_pins_r) {
        BOARD_MOTOR_R_STEP_PORT->BSRR = set_pins_r;
        motor_count_step(&g_motor_steps_right, s_motors[MOTOR_RIGHT].logical_fwd);
    }
}
