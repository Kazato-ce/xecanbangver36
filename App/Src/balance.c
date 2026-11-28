#include "balance.h"
#include "config.h"
#include "stepper_tmc2208.h"
#include <math.h>

/* ====================================================================
 * Hệ Thống Cân Bằng Mô Phỏng Theo Thuật Toán Chuẩn BalancingWii (Mahowik)
 * Kiến trúc: Vòng ngoài PI Vận Tốc + Vòng trong PD Góc Tích Phân Gia Tốc
 * ==================================================================== */

/* Các hệ số điều khiển (có thể tinh chỉnh trực tiếp qua Web / UART) */
static float s_kp_angle = PID_ANGLE_KP;   /* Mặc định ~ 18.0 */
static float s_kd_angle = PID_ANGLE_KD;   /* Mặc định ~ 0.85 */
static float s_ki_angle = PID_ANGLE_KI;   /* Mặc định ~ 0.00 */

static float s_kp_speed = PID_SPEED_KP;   /* Mặc định ~ 0.35 */
static float s_ki_speed = PID_SPEED_KI;   /* Mặc định ~ 0.02 */
static float s_kp_pos   = 0.002f;         /* Hệ số giữ vị trí đứng yên (Position Hold) */

static float s_kp_steer = PID_STEER_KP;   /* Hệ số bẻ lái (Steering) */

static float s_target_offset_deg = PID_DEFAULT_TRIM_DEG;  /* Góc trim cân bằng tự nhiên */
static float s_target_pitch_deg  = PID_DEFAULT_TRIM_DEG;  /* Góc nghiêng mục tiêu tức thời */

/* Biến trạng thái nội bộ thuật toán BalancingWii */
static float s_actual_averaged_speed = 0.0f; /* Tốc độ trung bình qua lọc thông thấp */
static float s_speed_error_i = 0.0f;         /* Tích phân sai số tốc độ */
static float s_position_error = 0.0f;        /* Tích phân quãng đường khi giữ vị trí */
static float s_speed = 0.0f;                 /* Vận tốc tích lũy từ gia tốc góc */
static float s_integral_angle = 0.0f;

static float s_out_left_steps = 0.0f;
static float s_out_right_steps = 0.0f;
static float s_out_left_filtered = 0.0f;
static float s_out_right_filtered = 0.0f;
static float s_gyro_rate_filtered = 0.0f;
static float s_cmd_drive_slew = 0.0f;
static float s_cmd_drive_filtered = 0.0f;
static float s_cmd_steer_slew = 0.0f;
static float s_cmd_steer_filtered = 0.0f;
static bool s_drive_reversal_bleed_active = false;

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
    s_kp_pos   = 0.002f;
    s_kp_steer = PID_STEER_KP;

    s_target_offset_deg = PID_DEFAULT_TRIM_DEG;
    balance_reset();
}

void balance_reset(void)
{
    s_target_pitch_deg = s_target_offset_deg;
    s_actual_averaged_speed = 0.0f;
    s_speed_error_i = 0.0f;
    s_position_error = 0.0f;
    s_speed = 0.0f;
    s_integral_angle = 0.0f;
    s_out_left_steps = 0.0f;
    s_out_right_steps = 0.0f;
    s_out_left_filtered = 0.0f;
    s_out_right_filtered = 0.0f;
    s_gyro_rate_filtered = 0.0f;
    s_cmd_drive_slew = 0.0f;
    s_cmd_drive_filtered = 0.0f;
    s_cmd_steer_slew = 0.0f;
    s_cmd_steer_filtered = 0.0f;
    s_drive_reversal_bleed_active = false;
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
        s_target_offset_deg = clampf(offset_deg, -8.0f, 8.0f);
    }
}

float balance_get_target_offset(void)
{
    return s_target_offset_deg;
}

float balance_get_target_pitch_deg(void)
{
    return s_target_pitch_deg;
}

void balance_update(float pitch_deg, float gyro_rate_dps, float cmd_drive, float cmd_steer, float dt_s, bool fast_drive_release)
{
    if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = CONTROL_DT_S;

    /* stepper_get_speed() reports applied pulse rate in signed steps/sec. */
    float actual_speed = 0.5f * (stepper_get_speed(MOTOR_LEFT) +
                                 stepper_get_speed(MOTOR_RIGHT));
    s_actual_averaged_speed = s_actual_averaged_speed * 0.92f + actual_speed * 0.08f;

    /* 0. Giới hạn vector lệnh và làm mượt drive/steer qua hai tầng slew-rate + EMA. */
    float norm_drive = clampf(cmd_drive, -1.0f, 1.0f);
    float norm_steer = clampf(cmd_steer, -1.0f, 1.0f);

    /* Giới hạn vector hình tròn (Circle clamp) khi di chuyển chéo */
    float mag = sqrtf(norm_drive * norm_drive + norm_steer * norm_steer);
    if (mag > 1.0f) {
        norm_drive /= mag;
        norm_steer /= mag;
    }

    bool reversal_requested =
        (fabsf(norm_drive) > 0.05f) &&
        (fabsf(s_cmd_drive_filtered) > 0.05f) &&
        (norm_drive * s_cmd_drive_filtered < 0.0f);
    if (reversal_requested && !s_drive_reversal_bleed_active) {
        s_speed_error_i *= 0.2f;
        s_drive_reversal_bleed_active = true;
    } else if (!reversal_requested) {
        s_drive_reversal_bleed_active = false;
    }

    if ((norm_drive < 0.0f && actual_speed > BRAKE_BEFORE_REV_SPEED_HZ) ||
        (norm_drive > 0.0f && actual_speed < -BRAKE_BEFORE_REV_SPEED_HZ)) {
        norm_drive = 0.0f;
    }

    bool drive_reversing = !fast_drive_release &&
        ((norm_drive * s_cmd_drive_slew < 0.0f) ||
         ((fabsf(actual_speed) > 50.0f) &&
          (norm_drive * actual_speed < 0.0f)));

    if (drive_reversing) {
        float max_drive_change = DRIVE_REVERSAL_SLEW_PER_S * dt_s;
        s_cmd_drive_slew += clampf(norm_drive - s_cmd_drive_slew,
                                   -max_drive_change, max_drive_change);
        s_speed_error_i *= 0.92f;
    } else {
        s_cmd_drive_slew = norm_drive;
    }

    float max_steer_change = STEER_SLEW_PER_S * dt_s;
    s_cmd_steer_slew += clampf(norm_steer - s_cmd_steer_slew,
                               -max_steer_change, max_steer_change);

    if (DRIVE_CMD_EMA_TAU_S <= 0.0f) {
        s_cmd_drive_filtered = s_cmd_drive_slew;
    } else {
        float drive_ema_alpha = (dt_s > 0.0f)
                              ? (dt_s / (DRIVE_CMD_EMA_TAU_S + dt_s))
                              : 1.0f;
        s_cmd_drive_filtered += drive_ema_alpha *
                                (s_cmd_drive_slew - s_cmd_drive_filtered);
    }

    if (STEER_CMD_EMA_TAU_S <= 0.0f) {
        s_cmd_steer_filtered = s_cmd_steer_slew;
    } else {
        float steer_ema_alpha = (dt_s > 0.0f)
                              ? (dt_s / (STEER_CMD_EMA_TAU_S + dt_s))
                              : 1.0f;
        s_cmd_steer_filtered += steer_ema_alpha *
                                (s_cmd_steer_slew - s_cmd_steer_filtered);
    }

    /* Quy đổi lệnh qua MANUAL_DRIVE_MAX sang toàn dải tốc độ hành trình. */
    float drive_speed_fraction = clampf(s_cmd_drive_filtered / MANUAL_DRIVE_MAX,
                                        -1.0f, 1.0f);
    float target_speed = drive_speed_fraction * (float)STEP_CRUISE_MAX_HZ;

    /* 3. Chế độ Giữ vị trí (BalancingWii Position Hold Mode):
     * Khi xe đứng yên không có lệnh di chuyển (target_speed = 0, steer = 0),
     * tự động tích lũy quãng đường trôi để tạo góc phản kháng đưa xe về chỗ cũ */
    if (fabsf(target_speed) < 50.0f && fabsf(s_cmd_steer_filtered) < 0.05f) {
        s_position_error += actual_speed * dt_s;
        s_position_error = clampf(s_position_error, -4000.0f, 4000.0f);
    } else {
        s_position_error = 0.0f;
    }

    /* 4. Vòng ngoài PI Vận Tốc & Tạo Góc Nghiêng Di Chuyển (Drive Pitch Tilt):
     * Xe 2 bánh tự cân bằng muốn tiến/lùi phải nghiêng người theo hướng đó.
     * s_cmd_drive_filtered (-1.0 .. +1.0) chủ động tạo góc nghiêng drive_tilt để xe lướt đi */
    float drive_tilt = s_cmd_drive_filtered *
                       ((s_cmd_drive_filtered >= 0.0f)
                            ? DRIVE_TILT_FWD_DEG
                            : DRIVE_TILT_REV_DEG);

    float speed_err = target_speed - s_actual_averaged_speed - (s_position_error * s_kp_pos);

    s_speed_error_i += speed_err * dt_s;
    s_speed_error_i = clampf(s_speed_error_i, -100000.0f, 100000.0f);

    float target_pitch = s_target_offset_deg
                       + drive_tilt
                       + (s_kp_speed * 0.001f * speed_err)
                       + (s_ki_speed * 0.001f * s_speed_error_i);
    target_pitch = clampf(target_pitch,
                          -MAX_TARGET_ANGLE_REV_DEG,
                          MAX_TARGET_ANGLE_FWD_DEG);
    s_target_pitch_deg = target_pitch;

    /* 5. Vòng trong PD Góc Nghiêng với tích phân gia tốc (BalancingWii Core):
     * Lọc thông thấp Gyro Rate (cắt triệt để nhiễu cơ học tần số cao 50-200Hz gây tiếng rít) */
    s_gyro_rate_filtered = s_gyro_rate_filtered * 0.70f + gyro_rate_dps * 0.30f;

    float angle_err = pitch_deg - s_target_pitch_deg;

    s_integral_angle += angle_err * dt_s;
    s_integral_angle = clampf(s_integral_angle, -5.0f, 5.0f);

    /* Tính gia tốc bánh xe cần thiết (steps/s^2) */
    float acceleration = (s_kp_angle * 125.0f * angle_err)
                       + (s_kd_angle * 85.0f * s_gyro_rate_filtered)
                       + (s_ki_angle * 20.0f * s_integral_angle);

    /* Tích phân gia tốc thành vận tốc bước */
    s_speed += acceleration * dt_s;
    s_speed = clampf(s_speed, -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);

    /* Deadband quanh vị trí cân bằng: giảm chấn vi rung khi xe đang đứng thẳng */
    if (fabsf(angle_err) < 0.25f && fabsf(s_speed) < 30.0f && fabsf(target_speed) < 10.0f) {
        s_speed *= 0.85f;
    }

    /* 6. Hệ số suy giảm động ở góc lớn (BalancingWii Dynamic K - dynK):
     * Khi góc nghiêng quá lớn (gần ngưỡng ngã), giảm bớt tốc độ tối đa
     * để duy trì mô-men xoắn bước cực đại chống trượt bước */
    float abs_angle = fabsf(pitch_deg);
    float dyn_k = 1.0f;
    if (abs_angle < 25.0f) {
        dyn_k = 1.0f;
    } else if (abs_angle < FALL_ANGLE_THRESHOLD_DEG) {
        dyn_k = (FALL_ANGLE_THRESHOLD_DEG - abs_angle) / (FALL_ANGLE_THRESHOLD_DEG - 25.0f) * 0.85f + 0.15f;
    } else {
        dyn_k = 0.0f;
    }

    float output_speed = s_speed * dyn_k;

    /* 7. Bộ trộn rẽ thích ứng 8 hướng (Adaptive 8-way Steering Mixer):
     * - Khi xe đứng yên hoặc tốc độ thấp: lực quay tối đa để quay tại chỗ nhạy bén
     * - Khi xe đang tiến/lùi với tốc độ cao: giảm bớt độ nhạy rẽ để xe lượn chéo mềm mại,
     *   tránh lực ly tâm làm lật ngang xe */
    float steer_adaptive = 1.0f / (1.0f + 0.5f * fabsf(s_cmd_drive_filtered));
    float steer_hz = s_cmd_steer_filtered * steer_adaptive * s_kp_steer
                   * (float)STEP_CRUISE_MAX_HZ;
    steer_hz = clampf(steer_hz, -STEER_MAX_HZ, STEER_MAX_HZ);
    if (fabsf(s_cmd_drive_filtered) < 0.15f) {
        steer_hz *= 0.75f;
    }

    float balance_hz = output_speed;
    float steer_budget_hz = (float)STEP_CRUISE_MAX_HZ - fabsf(balance_hz);
    if (steer_budget_hz < 0.0f) {
        steer_budget_hz = 0.0f;
    }
    steer_hz = clampf(steer_hz, -steer_budget_hz, steer_budget_hz);

    s_out_left_steps  = balance_hz - steer_hz;
    s_out_right_steps = balance_hz + steer_hz;

    /* Giới hạn an toàn */
    s_out_left_steps  = clampf(s_out_left_steps,  -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);
    s_out_right_steps = clampf(s_out_right_steps, -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);

    /* Bộ lọc làm mịn tốc độ đầu ra (Output smoothing filter - triệt tiêu tiếng ồn & rít) */
    s_out_left_filtered  = s_out_left_filtered  * 0.35f + s_out_left_steps  * 0.65f;
    s_out_right_filtered = s_out_right_filtered * 0.35f + s_out_right_steps * 0.65f;
}

void balance_get_motor_speeds(float *left_steps_s, float *right_steps_s)
{
    if (left_steps_s)  *left_steps_s  = s_out_left_filtered;
    if (right_steps_s) *right_steps_s = s_out_right_filtered;
}
