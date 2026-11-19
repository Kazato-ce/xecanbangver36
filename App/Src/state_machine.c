#include "state_machine.h"
#include "stepper_tmc2208.h"
#include "balance.h"
#include "imu_mpu6050.h"
#include "config.h"
#include <math.h>

static VehicleState_t s_state = STATE_BOOT;
static FaultReason_t  s_fault_reason = FAULT_NONE;
static uint32_t       s_upright_stable_count = 0;
static uint32_t       s_recovery_count = 0;
static bool           s_auto_arm_enabled = (AUTO_BALANCE_ON_GROUND != 0);

static float          s_test_left_rpm = 0.0f;
static float          s_test_right_rpm = 0.0f;
static uint32_t       s_test_end_tick = 0;

void sm_init(void)
{
    s_state = STATE_BOOT;
    s_fault_reason = FAULT_NONE;
    s_upright_stable_count = 0;
    s_recovery_count = 0;
    s_auto_arm_enabled = (AUTO_BALANCE_ON_GROUND != 0);
    stepper_stop(); /* PA0 = HIGH: Driver tat an toan luc khoi dong */
}

void sm_update(float pitch_deg, bool radio_connected, bool arm_cmd, bool disarm_cmd, bool estop_cmd, uint32_t imu_errors)
{
    (void)radio_connected; /* Tự chủ cân bằng khi đặt xuống đất, không bắt buộc sóng */

    /* Emergency stop hoặc lệnh DISARM luôn được ưu tiên */
    if (estop_cmd) {
        stepper_stop();
        s_fault_reason = FAULT_ESTOP_CMD;
        s_state = STATE_FAULT;
        return;
    }

    /* Kiểm tra lỗi IMU liên tiếp */
    if (imu_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
        stepper_stop();
        s_fault_reason = FAULT_IMU_ERROR;
        s_state = STATE_FAULT;
        return;
    }

    float tilt_err = fabsf(pitch_deg - balance_get_target_offset());

    switch (s_state) {
        case STATE_BOOT:
            stepper_stop();
            /* Boot xong kiểm tra trạng thái hiệu chuẩn IMU */
            if (imu_is_calibrated()) {
                s_state = STATE_DISARMED;
            } else {
                s_state = STATE_CALIBRATING;
            }
            break;

        case STATE_CALIBRATING:
            stepper_stop();
            /* Sau khi calibration hoàn tất từ app.c / IMU, chuyển sang DISARMED */
            if (imu_is_calibrated()) {
                s_state = STATE_DISARMED;
            }
            break;

        case STATE_DISARMED:
            stepper_stop(); /* PA0 = HIGH: Driver tat, dong co ngat luc giu xe */
            s_fault_reason = FAULT_NONE;

            /* Neu bat che do AUTO_ARM (s_auto_arm_enabled = true):
             * Can giu xe thang dung quanh goc can bang (<= 3.5 do) lien tuc 1.0 giay (200 ticks):
             * Moi tu dong bat driver can bang */
            if (s_auto_arm_enabled) {
                if (tilt_err <= ARM_UPRIGHT_ANGLE_DEG) {
                    s_upright_stable_count++;
                    if (s_upright_stable_count >= AUTO_ARM_STABLE_TICKS) {
                        balance_reset();
                        stepper_enable(true); /* PA0 = LOW: Bat driver */
                        s_state = STATE_ARMED;
                        s_upright_stable_count = 0;
                    }
                } else {
                    s_upright_stable_count = 0;
                }
            } else {
                s_upright_stable_count = 0;
            }

            /* Lenh ARM chu dong tu USB CDC / UART1 / Radio NRF:
             * Cho phep kich hoat khi xe dung tuong doi thang quanh goc can bang (<= 10.5 do) */
            if (arm_cmd && tilt_err <= (ARM_UPRIGHT_ANGLE_DEG * 3.0f)) {
                balance_reset();
                stepper_enable(true); /* PA0 = LOW */
                s_state = STATE_ARMED;
                s_upright_stable_count = 0;
            }
            break;

        case STATE_ARMED:
            /* Kiểm tra ngã (Failsafe an toàn 1):
             * Khi xe nghiêng quá 32 độ so voi goc can bang -> NGẮT ĐỘNG CƠ NGAY LẬP TỨC */
            if (tilt_err > FALL_ANGLE_THRESHOLD_DEG) {
                stepper_stop(); /* PA0 = HIGH: Tắt driver */
                s_fault_reason = FAULT_FALL_OVER;
                s_state = STATE_FAULT;
                s_recovery_count = 0;
                break;
            }

            /* Người dùng gửi lệnh DISARM chủ động */
            if (disarm_cmd) {
                stepper_stop();
                s_state = STATE_DISARMED;
                s_upright_stable_count = 0;
                break;
            }
            break;

        case STATE_FAULT:
            stepper_stop(); /* Đảm bảo driver luôn tắt an toàn */
            /* TỰ ĐỘNG PHỤC HỒI KHI DỰNG LẠI XE:
             * Khi người dùng nhấc xe lên dựng thẳng lại quanh goc can bang trong 500ms:
             * Tự động chuyển về DISARMED để sẵn sàng tự cân bằng tiếp! */
            if (tilt_err <= ARM_UPRIGHT_ANGLE_DEG) {
                s_recovery_count++;
                if (s_recovery_count >= AUTO_RECOVER_TICKS || disarm_cmd) {
                    s_fault_reason = FAULT_NONE;
                    s_state = STATE_DISARMED;
                    s_recovery_count = 0;
                    s_upright_stable_count = 0;
                }
            } else {
                s_recovery_count = 0;
            }
            break;

        case STATE_TEST_MOTOR:
            if (disarm_cmd || (HAL_GetTick() >= s_test_end_tick)) {
                stepper_stop();
                s_test_left_rpm = 0.0f;
                s_test_right_rpm = 0.0f;
                s_state = STATE_DISARMED;
            }
            break;
    }
}

void sm_start_motor_test(float left_rpm, float right_rpm, uint32_t duration_ms)
{
    s_test_left_rpm = left_rpm;
    s_test_right_rpm = right_rpm;
    s_test_end_tick = HAL_GetTick() + duration_ms;
    s_state = STATE_TEST_MOTOR;
    stepper_enable(true);
}

void sm_stop_motor_test(void)
{
    stepper_stop();
    s_test_left_rpm = 0.0f;
    s_test_right_rpm = 0.0f;
    s_state = STATE_DISARMED;
}

bool sm_is_motor_test_active(void)
{
    return (s_state == STATE_TEST_MOTOR);
}

void sm_get_motor_test_speeds(float *left_rpm, float *right_rpm)
{
    if (left_rpm)  *left_rpm  = s_test_left_rpm;
    if (right_rpm) *right_rpm = s_test_right_rpm;
}

VehicleState_t sm_get_state(void)
{
    return s_state;
}

FaultReason_t sm_get_fault_reason(void)
{
    return s_fault_reason;
}

const char* sm_get_state_str(VehicleState_t st)
{
    switch (st) {
        case STATE_BOOT:        return "BOOT";
        case STATE_CALIBRATING: return "CALIBRATING";
        case STATE_DISARMED:    return "DISARMED";
        case STATE_ARMED:       return "ARMED";
        case STATE_FAULT:       return "FAULT";
        case STATE_TEST_MOTOR:  return "TEST_MOTOR";
        default:                return "UNKNOWN";
    }
}

void sm_set_auto_arm(bool enable)
{
    s_auto_arm_enabled = enable;
    s_upright_stable_count = 0;
}

bool sm_get_auto_arm(void)
{
    return s_auto_arm_enabled;
}

float sm_get_auto_arm_countdown_s(void)
{
    if (s_state != STATE_DISARMED) return 0.0f;
    if (s_upright_stable_count >= AUTO_ARM_STABLE_TICKS) return 0.0f;
    uint32_t rem = AUTO_ARM_STABLE_TICKS - s_upright_stable_count;
    return (float)rem * CONTROL_DT_S;
}

uint32_t sm_get_upright_stable_count(void)
{
    return s_upright_stable_count;
}
