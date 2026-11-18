#include "balance.h"
#include "config.h"
#include <math.h>

/* Dynamic PID gains with default values from config.h */
static float s_kp_angle = PID_ANGLE_KP;
static float s_kd_angle = PID_ANGLE_KD;
static float s_ki_angle = PID_ANGLE_KI;

static float s_kp_speed = PID_SPEED_KP;
static float s_ki_speed = PID_SPEED_KI;

static float s_kp_steer = PID_STEER_KP;

static float s_target_offset_deg = BALANCE_TARGET_OFFSET_DEG;
static float s_target_pitch_deg = BALANCE_TARGET_OFFSET_DEG;
static float s_gyro_filtered = 0.0f;
static float s_integral_speed = 0.0f;
static float s_integral_angle = 0.0f;
static float s_out_left_steps = 0.0f;
static float s_out_right_steps = 0.0f;

static inline float clampf(float val, float min_val, float max_val)
{
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

void balance_init(void)
{
    s_kp_angle = PID_ANGLE_KP;
    s_kd_angle = PID_ANGLE_KD;
    s_ki_angle = PID_ANGLE_KI;

    s_kp_speed = PID_SPEED_KP;
    s_ki_speed = PID_SPEED_KI;
    s_kp_steer = PID_STEER_KP;

    s_target_offset_deg = BALANCE_TARGET_OFFSET_DEG;
    balance_reset();
}

void balance_reset(void)
{
    s_target_pitch_deg = s_target_offset_deg;
    s_gyro_filtered = 0.0f;
    s_integral_speed = 0.0f;
    s_integral_angle = 0.0f;
    s_out_left_steps = 0.0f;
    s_out_right_steps = 0.0f;
}

void balance_set_pid(float kp_angle, float kd_angle, float ki_angle, float kp_speed, float ki_speed, float kp_steer)
{
    if (isfinite(kp_angle) && kp_angle >= 0.0f) s_kp_angle = kp_angle;
    if (isfinite(kd_angle) && kd_angle >= 0.0f) s_kd_angle = kd_angle;
    if (isfinite(ki_angle) && ki_angle >= 0.0f) s_ki_angle = ki_angle;

    if (isfinite(kp_speed) && kp_speed >= 0.0f) s_kp_speed = kp_speed;
    if (isfinite(ki_speed) && ki_speed >= 0.0f) s_ki_speed = ki_speed;

    if (isfinite(kp_steer) && kp_steer >= 0.0f) s_kp_steer = kp_steer;
}

void balance_get_pid(float *kp_angle, float *kd_angle, float *ki_angle, float *kp_speed, float *ki_speed, float *kp_steer)
{
    if (kp_angle) *kp_angle = s_kp_angle;
    if (kd_angle) *kd_angle = s_kd_angle;
    if (ki_angle) *ki_angle = s_ki_angle;

    if (kp_speed) *kp_speed = s_kp_speed;
    if (ki_speed) *ki_speed = s_ki_speed;

    if (kp_steer) *kp_steer = s_kp_steer;
}

void balance_set_target_offset(float offset_deg)
{
    if (isfinite(offset_deg)) {
        s_target_offset_deg = clampf(offset_deg, -15.0f, 15.0f);
    }
}

float balance_get_target_pitch_deg(void)
{
    return s_target_pitch_deg;
}

float balance_get_target_offset(void)
{
    return s_target_offset_deg;
}

void balance_update(float pitch_deg, float gyro_rate_dps, float cmd_drive, float cmd_steer, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = CONTROL_DT_S;

    /* 1. Vòng điều khiển vận tốc (Outer loop):
     * Tính góc nghiêng mục tiêu dựa trên sai lệch giữa vận tốc mong muốn và vận tốc bánh xe hiện tại */
    float speed_err = cmd_drive - 0.5f * (s_out_left_steps + s_out_right_steps) / (float)MOTOR_STEPS_PER_REV;
    s_integral_speed += speed_err * dt_s;
    s_integral_speed = clampf(s_integral_speed, -5.0f, 5.0f);

    float speed_tilt = (s_kp_speed * speed_err) + (s_ki_speed * s_integral_speed);
    speed_tilt = clampf(speed_tilt, -6.0f, 6.0f);
    s_target_pitch_deg = s_target_offset_deg + speed_tilt;

    /* 2. Vòng điều khiển cân bằng góc nghiêng (Inner loop):
     * Lọc thông thấp bậc 1 (LPF) cho tốc độ góc Gyro để triệt tiêu nhiễu cao tần gây rè/kêu động cơ */
    s_gyro_filtered = (0.75f * s_gyro_filtered) + (0.25f * gyro_rate_dps);

    float angle_err = pitch_deg - s_target_pitch_deg;

    /* Deadband vi rung (micro-jitter): triệt tiêu rung lắc tần số cao khi đứng yên */
    if (fabsf(angle_err) < 0.06f) {
        angle_err = 0.0f;
    }

    s_integral_angle += angle_err * dt_s;
    s_integral_angle = clampf(s_integral_angle, -10.0f, 10.0f);

    float balance_output = (s_kp_angle * angle_err) +
                           (s_kd_angle * s_gyro_filtered) +
                           (s_ki_angle * s_integral_angle);

    /* Quy đổi đầu ra sang tần số xung bước (steps/sec) */
    float base_steps_s = balance_output * (float)MOTOR_STEPS_PER_REV / 360.0f * 15.0f;

    /* 3. Phối hợp điều khiển quay xe (Steering) */
    float steer_steps_s = cmd_steer * s_kp_steer * (float)MOTOR_STEPS_PER_REV / 1000.0f;

    s_out_left_steps  = base_steps_s - steer_steps_s;
    s_out_right_steps = base_steps_s + steer_steps_s;

    /* Giới hạn an toàn */
    s_out_left_steps  = clampf(s_out_left_steps,  -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);
    s_out_right_steps = clampf(s_out_right_steps, -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);
}

void balance_get_motor_speeds(float *left_steps_s, float *right_steps_s)
{
    if (left_steps_s)  *left_steps_s  = s_out_left_steps;
    if (right_steps_s) *right_steps_s = s_out_right_steps;
}
