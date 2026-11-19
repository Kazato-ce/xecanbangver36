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

void sm_init(void)
{
    s_state = STATE_BOOT;
    s_fault_reason = FAULT_NONE;
    s_upright_stable_count = 0;
    s_recovery_count = 0;
    stepper_stop(); /* PA0 = HIGH: Driver tắt an toàn lúc khởi động */
}

void sm_update(float pitch_deg, bool radio_connected, bool arm_cmd, bool disarm_cmd, bool estop_cmd, uint32_t imu_errors)
{
    (void)radio_connected; /* Tự chủ cân bằng khi đặt xe xuống sàn */

    /* Emergency stop hoặc lệnh DISARM luôn được ưu tiên cao nhất */
    if (estop_cmd) {
        stepper_stop();
        s_fault_reason = FAULT_ESTOP_CMD;
        s_state = STATE_FAULT;
        s_upright_stable_count = 0;
        return;
    }

    /* Kiểm tra lỗi IMU liên tiếp */
    if (imu_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
        stepper_stop();
        s_fault_reason = FAULT_IMU_ERROR;
        s_state = STATE_FAULT;
        s_upright_stable_count = 0;
        return;
    }

    /* Tính độ lệch góc so với điểm cân bằng tự nhiên của xe */
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
            /* Sau khi calibration hoàn tất từ IMU, chuyển sang DISARMED */
            if (imu_is_calibrated()) {
                s_state = STATE_DISARMED;
            }
            break;

        case STATE_DISARMED:
            stepper_stop(); /* PA0 = HIGH: Driver tắt, động cơ không bị gông cứng */
            s_fault_reason = FAULT_NONE;

            /* TỰ ĐỘNG CHUYỂN SANG ARM KHI GIỮ Ở GÓC CÂN BẰNG 5 GIÂY (1000 chu kỳ @ 200Hz):
             * Người dùng dựng thẳng xe quanh điểm cân bằng (<= 4.5 độ) liên tục 5 giây:
             * Xe sẽ tự động bật driver TMC2208 và bắt đầu tự cân bằng! */
            if (tilt_err <= ARM_UPRIGHT_ANGLE_DEG) {
                s_upright_stable_count++;
                if (s_upright_stable_count >= AUTO_ARM_STABLE_TICKS) {
                    balance_reset();
                    stepper_enable(true); /* PA0 = LOW: Bật driver động cơ */
                    s_state = STATE_ARMED;
                    s_upright_stable_count = 0;
                }
            } else {
                s_upright_stable_count = 0; /* Lệch quá góc cân bằng -> đếm lại */
            }

            /* Lệnh ARM chủ động từ Remote NRF24 / ESP32 Super Mini / Python:
             * Cho phép kích hoạt ngay lập tức nếu xe đang dựng tương đối thẳng */
            if (arm_cmd && (tilt_err <= (ARM_UPRIGHT_ANGLE_DEG * 2.5f))) {
                balance_reset();
                stepper_enable(true); /* PA0 = LOW */
                s_state = STATE_ARMED;
                s_upright_stable_count = 0;
            }
            break;

        case STATE_ARMED:
            /* Failsafe 1: Kiểm tra xe bị ngã (> 35 độ) -> NGẮT ĐỘNG CƠ NGAY LẬP TỨC */
            if (tilt_err > FALL_ANGLE_THRESHOLD_DEG) {
                stepper_stop(); /* PA0 = HIGH: Tắt driver */
                s_fault_reason = FAULT_FALL_OVER;
                s_state = STATE_FAULT;
                s_recovery_count = 0;
                s_upright_stable_count = 0;
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
             * Khi xe ngã, người dùng nhấc xe lên dựng thẳng lại quanh góc cân bằng trong 500ms:
             * Tự động xóa lỗi và chuyển về DISARMED để chuẩn bị đếm 5s tự cân bằng tiếp! */
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
                if (disarm_cmd) {
                    s_fault_reason = FAULT_NONE;
                    s_state = STATE_DISARMED;
                    s_recovery_count = 0;
                    s_upright_stable_count = 0;
                }
            }
            break;
    }
}

VehicleState_t sm_get_state(void)
{
    return s_state;
}

FaultReason_t sm_get_fault_reason(void)
{
    return s_fault_reason;
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

const char* sm_get_state_str(VehicleState_t st)
{
    switch (st) {
        case STATE_BOOT:        return "BOOT";
        case STATE_CALIBRATING: return "CALIBRATING";
        case STATE_DISARMED:    return "DISARMED";
        case STATE_ARMED:       return "ARMED";
        case STATE_FAULT:       return "FAULT";
        default:                return "UNKNOWN";
    }
}
