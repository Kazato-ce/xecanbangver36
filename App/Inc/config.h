#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
 * Loop Frequencies & Timing
 * ==================================================================== */
#define CONTROL_LOOP_FREQ_HZ        200           /* 200 Hz = 5 ms (TIM6) */
#define CONTROL_DT_S                0.005f        /* dt = 1 / 200 Hz */
#define STEP_TICK_FREQ_HZ           20000         /* 20 kHz (TIM7) DDS pulse generator */

/* ====================================================================
 * Stepper & Physical Parameters
 * ==================================================================== */
#define MOTOR_STEP_ANGLE_DEG        1.8f          /* 1.8 deg/step (200 steps/rev) */
#define MOTOR_MICROSTEPS            16            /* TMC2208 hardware microstep setting */
#define MOTOR_STEPS_PER_REV         (200 * MOTOR_MICROSTEPS) /* 3200 steps/rev */

#define WHEEL_DIAMETER_M            0.065f        /* 65 mm wheel diameter */
#define WHEEL_DISTANCE_M            0.180f        /* Distance between 2 wheels in meters */

/* Motor Direction Inversion (Đối xứng 2 bên sườn xe) */
#define MOTOR_L_DIR_INVERT          0
#define MOTOR_R_DIR_INVERT          1

/* Speed & Acceleration Limits */
#define STEP_FREQ_MAX_HZ            30000.0f      /* Max pulses/second limit */
#define STEP_CRUISE_MAX_HZ          8000.0f       /* Tốc độ hành trình di chuyển tối đa (steps/sec) */
#define STEP_ACCEL_MAX_HZ_PER_S     80000.0f      /* Max pulse acceleration */
#define MAX_TARGET_ANGLE_DEG        12.0f         /* Góc nghiêng tối đa khi di chuyển (tương đương BalancingWii) */

/* ====================================================================
 * Safety & Thresholds (Tối ưu cho Cân Bằng Thực Tế Khi Đặt Xuống Đất)
 * ==================================================================== */
#define AUTO_BALANCE_ON_GROUND      0             /* 0: Khoi dong o che do DISARM an toan (PA0=HIGH), khong tu quay banh khi cam USB. Gui 'ARM' de chay */
#define AUTO_ARM_STABLE_TICKS       200           /* Neu bat AUTO_ARM: Can giu thang 200 chu ky (1.0 giay) tren san moi bat can bang */
#define AUTO_RECOVER_TICKS          100           /* Khi nga dung lai 500 ms -> Ve DISARMED */

#define FALL_ANGLE_THRESHOLD_DEG    32.0f         /* Góc nghiêng > 32 độ -> NGÃ -> TẮT ĐỘNG CƠ AN TOÀN */
#define ARM_UPRIGHT_ANGLE_DEG       3.5f          /* Xe dựng thẳng trong khoảng ±3.5 độ để kích hoạt */
#define IMU_MAX_CONSECUTIVE_ERRORS  5             /* Lỗi đọc IMU liên tiếp tối đa -> FAULT */
#define RADIO_TIMEOUT_MS            1000          /* Mất sóng NRF quá thời gian này */

/* ====================================================================
 * IMU (MPU6050) Orientation & Filtering
 * ==================================================================== */
#define IMU_I2C_ADDR_7BIT           0x68          /* AD0=GND: 0x68, AD0=VCC: 0x69 */
#define IMU_TILT_AXIS               0             /* 0: Pitch từ Accel X/Z, 1: Pitch từ Accel Y/Z */
#define IMU_TILT_SIGN               (1.0f)        /* Đổi dấu góc nghiêng nếu module lắp ngược */
#define IMU_GYRO_SIGN               (1.0f)        /* Đổi dấu tốc độ góc gyro */
#define IMU_FILTER_ALPHA            0.985f        /* Bộ lọc bù triệt tiêu rung chấn tiếp sàn (98.5% Gyro + 1.5% Accel) */

/* ====================================================================
 * Control System / Cascade PID Gains (Tối ưu cho tiếp xúc mặt đất)
 * ==================================================================== */
#define BALANCE_TARGET_OFFSET_DEG   (-2.98f)      /* Góc cân bằng vật lý thực tế của xe (bù lệch trọng tâm khung) */

#define PID_ANGLE_KP                18.0f         /* Vòng trong: Độ nhạy phản hồi góc nghiêng (êm, giảm giật) */
#define PID_ANGLE_KD                0.75f         /* Vòng trong: Giảm chấn Gyro (lọc mượt, triệt tiêu tiếng kêu rè) */
#define PID_ANGLE_KI                0.0f          /* Vòng trong: Tích phân góc nghiêng */

#define PID_SPEED_KP                0.40f         /* Vòng ngoài: Triệt tiêu trôi dạt (Zero-Drift) trên sàn */
#define PID_SPEED_KI                0.025f        /* Vòng ngoài: Tích phân vận tốc để giữ vị trí cố định */

#define PID_STEER_KP                1.2f          /* Hệ số quay xe */

/* ====================================================================
 * NRF24L01 Radio Configuration
 * ==================================================================== */
#define RADIO_CHANNEL               76            /* Tần số 2476 MHz */
#define RADIO_PAYLOAD_LEN           16            /* Bytes per packet */
#define RADIO_TX_ADDR               0xE7E7E7E7E7ULL
#define RADIO_RX_ADDR               0xE7E7E7E7E7ULL

/* ====================================================================
 * Obstacle Sensors (VL53L0X & HC-SR05)
 * ==================================================================== */
#define TOF_READ_INTERVAL_MS        50            /* Đọc ToF mỗi 50 ms (20 Hz) */
#define US_PING_INTERVAL_MS         40            /* Chuyển cảm biến siêu âm mỗi 40 ms */
#define OBSTACLE_SLOWDOWN_DIST_CM   30            /* Khoảng cách bắt đầu giảm tốc */
#define OBSTACLE_STOP_DIST_CM       12            /* Khoảng cách dừng khẩn cấp */

#endif /* CONFIG_H */
