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

static float s_target_offset_deg = 0.45f;  /* Góc trim cân bằng tự nhiên */
static float s_target_pitch_deg  = 0.45f;  /* Góc nghiêng mục tiêu tức thời */

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

    s_target_offset_deg = 0.0f;
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

void balance_update(float pitch_deg, float gyro_rate_dps, float cmd_drive, float cmd_steer, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = CONTROL_DT_S;

    /* 1. Lấy tốc độ bánh xe thực tế hiện tại (steps/sec) */
    float actual_speed = 0.5f * (stepper_get_speed(MOTOR_LEFT) + stepper_get_speed(MOTOR_RIGHT));

    /* 2. Bộ lọc thông thấp tốc độ (BalancingWii: 0.92 * avg + 0.08 * cur) */
    s_actual_averaged_speed = s_actual_averaged_speed * 0.92f + actual_speed * 0.08f;

    /* Quy đổi lệnh lái cmd_drive (-1.0 .. +1.0) sang tốc độ mục tiêu */
    float target_speed = cmd_drive * (float)STEP_CRUISE_MAX_HZ;

    /* 3. Chế độ Giữ vị trí (BalancingWii Position Hold Mode):
     * Khi xe đứng yên không có lệnh di chuyển (target_speed = 0, steer = 0),
     * tự động tích lũy quãng đường trôi để tạo góc phản kháng đưa xe về chỗ cũ */
    if (fabsf(target_speed) < 50.0f && fabsf(cmd_steer) < 0.05f) {
        s_position_error += actual_speed * dt_s;
        s_position_error = clampf(s_position_error, -4000.0f, 4000.0f);
    } else {
        s_position_error = 0.0f;
    }

    /* 4. Vòng ngoài PI Vận Tốc & Tạo Góc Nghiêng Di Chuyển (Drive Pitch Tilt):
     * Xe 2 bánh tự cân bằng muốn tiến/lùi phải nghiêng người theo hướng đó.
     * cmd_drive (-1.0 .. +1.0) chủ động tạo góc nghiêng drive_tilt để xe lướt đi */
    float drive_tilt = cmd_drive * 5.5f;

    float speed_err = target_speed - s_actual_averaged_speed - (s_position_error * s_kp_pos);

    s_speed_error_i += speed_err * dt_s;
    s_speed_error_i = clampf(s_speed_error_i, -8000.0f, 8000.0f);

    float target_pitch = s_target_offset_deg
                       + drive_tilt
                       + (s_kp_speed * 0.001f * speed_err)
                       + (s_ki_speed * 0.001f * s_speed_error_i);
    target_pitch = clampf(target_pitch, -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG);
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

    /* Trộn thành phần ga đẩy dứt khoát khi có lệnh tiến/lùi */
    float drive_throttle = cmd_drive * (float)STEP_CRUISE_MAX_HZ * 0.35f;
    float output_speed = (s_speed * dyn_k) + drive_throttle;

    /* 7. Bộ trộn rẽ (Steering Mixer) & Chống bão hòa (Anti-saturation):
     * Cắt giảm tốc độ tuyến tính nếu tổng tốc độ và rẽ vượt quá giới hạn xung tối đa */
    float steering = cmd_steer * s_kp_steer * (float)STEP_CRUISE_MAX_HZ;
    float max_spd = STEP_FREQ_MAX_HZ;

    if ((fabsf(output_speed) + fabsf(steering)) > max_spd) {
        output_speed = (output_speed > 0.0f) ? (max_spd - fabsf(steering)) : (-max_spd + fabsf(steering));
    }

    s_out_left_steps  = output_speed - steering;
    s_out_right_steps = output_speed + steering;

    /* Giới hạn an toàn */
    s_out_left_steps  = clampf(s_out_left_steps,  -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);
    s_out_right_steps = clampf(s_out_right_steps, -STEP_FREQ_MAX_HZ, STEP_FREQ_MAX_HZ);

    /* Bộ lọc làm mịn tốc độ đầu ra (Output smoothing filter - triệt tiêu tiếng ồn & rít) */
    s_out_left_filtered  = s_out_left_filtered  * 0.70f + s_out_left_steps  * 0.30f;
    s_out_right_filtered = s_out_right_filtered * 0.70f + s_out_right_steps * 0.30f;
}

void balance_get_motor_speeds(float *left_steps_s, float *right_steps_s)
{
    if (left_steps_s)  *left_steps_s  = s_out_left_filtered;
    if (right_steps_s) *right_steps_s = s_out_right_filtered;
}
