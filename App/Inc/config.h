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
#define WHEEL_DISTANCE_M            0.200f        /* Distance between 2 wheels in meters */

#define ROBOT_WIDTH_CM              26.0f        /* Bề ngang phủ bì cả hai bánh */
#define ROBOT_LENGTH_CM             10.0f
#define AVOID_SIDE_SAFE_CM          28.0f        /* Ngưỡng phát hiện vật cản hai bên */
#define AVOID_SIDE_STEER_MAX        0.12f
#define AVOID_CRUISE_DRIVE          0.36f
#define AVOID_TAIL_CLEAR_STEPS      2200.0f       /* Chạy thẳng thêm khoảng 15 cm để bánh sau vượt đuôi vật cản */
#define AVOID_TAIL_CLEAR_MS         850U          /* Giới hạn thời gian chờ đệm thoát đuôi */
#define AVOID_PHASE_TIMEOUT_MS      4000U         /* Timeout an toàn khi cảm biến chưa báo thoáng */
#define AVOID_YAW_KP                0.015f
#define AVOID_ALIGN_DURATION_MS     300U          /* Bù hướng ngắn sau khi vượt vật cản */

#define MANUAL_DRIVE_GAIN           1.55f
#define MANUAL_DRIVE_MIN_EFFECTIVE  0.24f
#define MANUAL_DRIVE_MAX            0.85f

/* Motor Direction Inversion (Đối xứng 2 bên sườn xe) */
#define MOTOR_L_DIR_INVERT          0
#define MOTOR_R_DIR_INVERT          1

/* Speed & Acceleration Limits */
#define STEP_FREQ_MAX_HZ            15000.0f      /* Max pulses/second limit */
#define STEP_CRUISE_MAX_HZ          9000.0f       /* Tốc độ hành trình di chuyển tối đa (steps/sec) */
#define STEP_ACCEL_MAX_HZ_PER_S     40000.0f      /* Max pulse acceleration (làm mịn gia tốc, triệt tiêu tiếng gằn máy) */
#define MAX_TARGET_ANGLE_DEG        9.8f          /* Góc nghiêng tối đa khi di chuyển (tương đương BalancingWii) */

/* ====================================================================
 * Safety & Thresholds (Tự động ARM khi giữ ở góc cân bằng 5s)
 * ==================================================================== */
#define FALL_ANGLE_THRESHOLD_DEG    45.0f         /* Góc nghiêng tối đa -> ngã -> tắt động cơ */
#define ARM_UPRIGHT_ANGLE_DEG       8.0f         /* Ngưỡng góc cân bằng (±12.0 độ quanh điểm cân bằng) */
#define AUTO_ARM_HOLD_TIME_S        2.0f          /* Giữ xe ở góc cân bằng 5 giây -> TỰ ĐỘNG CHUYỂN SANG ARM */
#define AUTO_ARM_STABLE_TICKS       ((uint32_t)(AUTO_ARM_HOLD_TIME_S * CONTROL_LOOP_FREQ_HZ)) /* 5s * 200Hz = 1000 chu kỳ */
#define AUTO_RECOVER_TICKS          100           /* Khi ngã, dựng lại 500 ms -> Tự động về DISARMED */
#define IMU_MAX_CONSECUTIVE_ERRORS  5             /* Lỗi đọc IMU liên tiếp tối đa -> FAULT */
#define RADIO_TIMEOUT_MS            1000          /* Mất sóng NRF quá thời gian này */
#define CONTROL_LINK_TIMEOUT_MS     400           /* Mất cả lệnh UART và NRF */

/* ====================================================================
 * IMU (MPU6050) Orientation & Filtering
 * ==================================================================== */
#define IMU_I2C_ADDR_7BIT           0x68          /* AD0=GND: 0x68, AD0=VCC: 0x69 */
#define IMU_TILT_AXIS               0             /* 0: Pitch từ Accel X/Z, 1: Pitch từ Accel Y/Z */
#define IMU_TILT_SIGN               (1.0f)        /* Đổi dấu góc nghiêng nếu module lắp ngược */
#define IMU_GYRO_SIGN               (1.0f)        /* Đổi dấu tốc độ góc gyro */
#define IMU_FILTER_ALPHA            0.98f         /* Hệ số Complementary Filter */

/* ====================================================================
 * Control System / Balance PID Gains (Placeholder - TODO Tune)
 * ==================================================================== */
/* TODO tune: Người dùng thay thế hệ số từ mô phỏng */
#define PID_ANGLE_KP                30.0f         /* Độ nhạy góc nghiêng (Kp Góc = 30) */
#define PID_ANGLE_KD                1.90f         /* Giảm chấn theo tốc độ góc (Kd Góc = 1.9) */
#define PID_ANGLE_KI                0.000001f     /* Tích phân góc nghiêng (Ki Góc = 0.000001) */

#define PID_SPEED_KP                0.35f         /* Vòng ngoài điều khiển vận tốc xe (Kp Vận Tốc = 0.35) */
#define PID_SPEED_KI                0.02f         /* Tích phân vận tốc (Ki Vận Tốc = 0.02) */

#define PID_STEER_KP                1.2f          /* Hệ số quay xe (Kp Lái = 1.2) */

#define PID_DEFAULT_TRIM_DEG        (-0.45f)      /* Trim Góc Đặt (Điểm 0 trọng tâm cân bằng tự nhiên = -0.45°) */

/* ====================================================================
 * NRF24L01 Radio Configuration
 * ==================================================================== */
#define RADIO_CHANNEL               76            /* Tần số 2476 MHz */
#define RADIO_PAYLOAD_LEN           17            /* Bytes per packet (sizeof(RadioPacket_t)) */
#define RADIO_TX_ADDR               0xE7E7E7E7E7ULL
#define RADIO_RX_ADDR               0xE7E7E7E7E7ULL
//
/* ====================================================================
 * Obstacle Sensors (VL53L0X & HC-SR05)
 * ==================================================================== */
#define TOF_READ_INTERVAL_MS        50            /* Đọc ToF mỗi 50 ms (20 Hz) */
#define TOF_MEASUREMENT_TIMEOUT_MS  50            /* Timeout cho một phép đo ToF */
#define TOF_INIT_RETRY_MS           500U          /* Thử khởi tạo lại ToF nếu lần đầu lỗi */
#define TOF_MAX_VALID_RANGE_MM      2000U         /* Bỏ các mã range ngoài vùng sử dụng */
#define TOF_CLIFF_LIMIT_MM          450U          /* Khoảng cách tối đa tới mặt sàn an toàn */
#define TOF_CLIFF_CONFIRM_SAMPLES   3U            /* Số mẫu ToF liên tiếp xác nhận mép */
#define US_PING_INTERVAL_MS         45U           /* Khoảng tối thiểu giữa các xung TRIG tuần tự */
#define US_ECHO_RISE_TIMEOUT_US     20000U        /* Timeout tối đa chờ cạnh lên Echo (~340 cm) */
#define US_ECHO_FALL_TIMEOUT_US     20000U        /* Timeout tối đa chờ cạnh xuống Echo (~340 cm) */
#define US_SAMPLE_MAX_AGE_MS        400U          /* Mẫu siêu âm cũ hơn được coi là lỗi */
#define OBSTACLE_SLOWDOWN_DIST_CM   40.0f         /* Ngưỡng phản xạ tránh vật cản phía trước */
#define OBSTACLE_STOP_DIST_CM       12            /* Khoảng cách dừng khẩn cấp */
#define DRIVER_ASSIST_STEER_MAX     0.15f         /* Giới hạn bù lái khi né vật cản */
#define CLIFF_EXTRA_BACKUP_MS       400U          /* Lùi thêm sau khi ToF xác nhận đã trở lại mặt sàn */

#endif /* CONFIG_H */
