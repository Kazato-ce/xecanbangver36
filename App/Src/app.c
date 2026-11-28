#include "app.h"
#include "board_config.h"
#include "config.h"
#include "state_machine.h"
#include "stepper_tmc2208.h"
#include "imu_mpu6050.h"
#include "balance.h"
#include "radio_nrf24.h"
#include "tof_vl53l0x.h"
#include "ultrasonic_hcsr05.h"
#include "telemetry_uart.h"
#include "protocol.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static TIM_HandleTypeDef s_htim6;
static TIM_HandleTypeDef s_htim7;
static SPI_HandleTypeDef s_hspi2;
static volatile bool s_tof_available = false;
static bool s_cliff_retreat_active = false;
static volatile bool s_adas_enabled = false;

static float app_clampf(float value, float min_value, float max_value)
{
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}

static void app_apply_manual_drive_gain(float *cmd_drive)
{
    if (!cmd_drive || fabsf(*cmd_drive) <= 0.02f) {
        return;
    }

    float magnitude = fabsf(*cmd_drive) * MANUAL_DRIVE_GAIN;
    if (magnitude < MANUAL_DRIVE_MIN_EFFECTIVE) {
        magnitude = MANUAL_DRIVE_MIN_EFFECTIVE;
    }
    magnitude = app_clampf(magnitude, 0.0f, MANUAL_DRIVE_MAX);
    *cmd_drive = copysignf(magnitude, *cmd_drive);
}

static bool app_get_clearance_cm(UltrasonicId_t id, uint32_t now_ms, float *clearance_cm)
{
    UltrasonicSample_t sample = ultrasonic_get_sample(id);
    if ((sample.timestamp_ms == 0U) ||
        ((now_ms - sample.timestamp_ms) > US_SAMPLE_MAX_AGE_MS)) {
        return false;
    }

    switch (sample.status) {
        case US_SAMPLE_VALID:
            *clearance_cm = (float)sample.distance_cm;
            return true;
        case US_SAMPLE_TOO_CLOSE:
            *clearance_cm = 0.0f;
            return true;
        case US_SAMPLE_OUT_OF_RANGE:
            *clearance_cm = 400.0f;
            return true;
        case US_SAMPLE_NO_DATA:
        case US_SAMPLE_TIMEOUT_OR_TOO_CLOSE:
        default:
            return false;
    }
}

static void app_apply_adas_speed_limit(float *cmd_drive, uint32_t now_ms)
{
    if (!cmd_drive || *cmd_drive <= 0.0f) {
        return;
    }

    float front_cm = 0.0f;
    bool front_valid = app_get_clearance_cm(US_FRONT, now_ms, &front_cm);
    float left_cm = 0.0f;
    float right_cm = 0.0f;
    bool left_valid = app_get_clearance_cm(US_LEFT, now_ms, &left_cm);
    bool right_valid = app_get_clearance_cm(US_RIGHT, now_ms, &right_cm);

    if ((front_valid && front_cm <= ADAS_STOP_DIST_CM) ||
        (left_valid && left_cm <= ADAS_SIDE_STOP_CM) ||
        (right_valid && right_cm <= ADAS_SIDE_STOP_CM)) {
        *cmd_drive = 0.0f;
        return;
    }

    if (front_valid && front_cm <= ADAS_SLOW_DIST_CM) {
        float speed_scale = (front_cm - ADAS_STOP_DIST_CM) /
                            (ADAS_SLOW_DIST_CM - ADAS_STOP_DIST_CM);
        *cmd_drive *= app_clampf(speed_scale, 0.0f, 1.0f);
    }
}

static void app_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void app_spi2_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();

    s_hspi2.Instance = SPI2;
    s_hspi2.Init.Mode = SPI_MODE_MASTER;
    s_hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    s_hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    s_hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_hspi2.Init.NSS = SPI_NSS_SOFT;
    s_hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    s_hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    s_hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&s_hspi2);
}

static void app_timers_init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_apb1_clock = (RCC->CFGR & RCC_CFGR_PPRE1) ? (pclk1 * 2) : pclk1;

    /* TIM6: Vòng điều khiển cân bằng 200 Hz (5 ms)
     * Prescaler đưa về clock 100 kHz (10 us/tick)
     * Period = 499 -> 100 kHz / 500 = 200 Hz */
    uint32_t psc6 = (tim_apb1_clock / 100000U) - 1U;
    s_htim6.Instance = TIM6;
    s_htim6.Init.Prescaler = psc6;
    s_htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim6.Init.Period = 499;
    s_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_htim6);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&s_htim6);

    /* TIM7: Bộ phát xung DDS 20 kHz cho 2 động cơ
     * Prescaler đưa về clock 2 MHz (0.5 us/tick)
     * Period = 99 -> 2 MHz / 100 = 20 kHz */
    uint32_t psc7 = (tim_apb1_clock / 2000000U) - 1U;
    s_htim7.Instance = TIM7;
    s_htim7.Init.Prescaler = psc7;
    s_htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim7.Init.Period = 99;
    s_htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_htim7);

    HAL_NVIC_SetPriority(TIM7_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&s_htim7);
}

void app_init(void)
{
    app_dwt_init();

    /* 1. Khởi tạo Stepper ngay lập tức để giữ PA0 = HIGH (tắt driver) */
    stepper_init();

    /* 2. Khởi tạo SPI2 và các ngoại vi phần cứng */
    app_spi2_init();

    /* 3. Khởi tạo UART giao tiếp với ESP32 Super Mini (Telemetry & Web PID tuning) */
    telemetry_uart_init();

    /* 4. Khởi tạo các module */
    sm_init();
    balance_init();
    ultrasonic_init();
    s_tof_available = tof_init(&hi2c2);
    radio_init(&s_hspi2);

    /* 5. Khởi tạo MPU6050 & hiệu chuẩn bias khi xe đứng yên */
    if (imu_init(&hi2c1)) {
        /* Lấy 300 mẫu hiệu chuẩn (~600 ms) */
        imu_calibrate(300);
    }
    /* 6. Khởi động Timer điều khiển và phát xung */
    app_timers_init();
}

void app_loop(void)
{
    uint32_t now_ms = HAL_GetTick();

    /* Xử lý các tác vụ non-blocking trong nền */
    radio_update(now_ms);
    ultrasonic_update(now_ms);
    tof_update(now_ms);
    s_tof_available = tof_is_available();
    telemetry_uart_update(now_ms);
}

/* Ngắt chu kỳ cố định 200 Hz từ TIM6 */
void app_timer_control_tick(void)
{
    bool requested_adas_enabled = radio_get_adas_enabled();
    if (s_adas_enabled && !requested_adas_enabled) {
        s_cliff_retreat_active = false;
    }
    s_adas_enabled = requested_adas_enabled;

    /* Đọc IMU với dt cố định 5 ms */
    imu_read(CONTROL_DT_S);

    float pitch = imu_get_pitch_deg();
    if (s_adas_enabled && s_tof_available) {
        tof_evaluate_cliff(pitch);
    }
    bool cliff_detected = s_adas_enabled && tof_is_cliff_detected();
    float gyro_rate = imu_get_gyro_rate_dps();
    uint32_t imu_err = imu_get_error_count();

    /* Nhận lệnh từ Radio NRF24 */
    float nrf_drive = 0.0f;
    float nrf_steer = 0.0f;
    bool nrf_arm_req = false;
    bool nrf_disarm_req = false;
    bool nrf_estop_req = false;

    radio_get_commands(&nrf_drive, &nrf_steer, &nrf_arm_req, &nrf_disarm_req, &nrf_estop_req);
    bool radio_recent = (radio_get_last_packet_age_ms() <= CONTROL_LINK_TIMEOUT_MS);

    /* Nhận lệnh từ ESP32 Super Mini qua UART */
    float uart_drive = 0.0f;
    float uart_steer = 0.0f;
    bool uart_has_drive = telemetry_uart_get_drive_cmd(&uart_drive, &uart_steer);
    bool uart_arm_req = telemetry_uart_get_arm_request();
    bool uart_disarm_req = telemetry_uart_get_disarm_request();

    /* Tổng hợp lệnh ARM / DISARM / ESTOP từ cả hai nguồn (NRF và ESP32 Super Mini) */
    bool arm_req = nrf_arm_req || uart_arm_req;
    bool disarm_req = nrf_disarm_req || uart_disarm_req;
    bool estop_req = nrf_estop_req;

    /* Tổng hợp lệnh di chuyển DRIVE & STEER:
     * - Nếu UART vừa gửi lệnh lái -> dùng lệnh UART
     * - Ngược lại nếu gói NRF24 còn mới -> dùng lệnh NRF24
     * - Nếu cả hai không điều khiển -> tốc độ = 0 */
    float cmd_drive = 0.0f;
    float cmd_steer = 0.0f;

    if (uart_has_drive) {
        cmd_drive = uart_drive;
        cmd_steer = uart_steer;
    } else if (radio_recent) {
        cmd_drive = nrf_drive;
        cmd_steer = nrf_steer;
    }
    bool control_link_ok = radio_recent || uart_has_drive;

    /* Cập nhật Máy trạng thái */
    sm_update(pitch, radio_recent, arm_req, disarm_req, estop_req, imu_err);

    bool is_armed = (sm_get_state() == STATE_ARMED);

    if (is_armed) {
        bool fast_adas_drive_release = false;

        if (!radio_recent) {
            cmd_drive = 0.0f;
            cmd_steer = 0.0f;
            s_cliff_retreat_active = false;
        }

        app_apply_manual_drive_gain(&cmd_drive);

        if (!radio_recent || !control_link_ok) {
            cmd_drive = 0.0f;
            cmd_steer = 0.0f;
        }

        if (s_adas_enabled) {
            bool cliff_control_active = cliff_detected || s_cliff_retreat_active;
            float drive_before_adas = cmd_drive;

            /* Phản xạ chống rơi mép bàn / rơi vực (ToF Anti-Cliff Protection). */
            if (cliff_detected) {
                s_cliff_retreat_active = true;
                cmd_drive = ADAS_CLIFF_BACK_DRIVE;
                cmd_steer = 0.0f;
            } else if (s_cliff_retreat_active) {
                if (!s_tof_available || !tof_is_valid()) {
                    cmd_drive = ADAS_CLIFF_BACK_DRIVE;
                    cmd_steer = 0.0f;
                } else {
                    s_cliff_retreat_active = false;
                    cmd_drive = 0.0f;
                }
            }

            if (control_link_ok && cmd_drive > 0.0f) {
                app_apply_adas_speed_limit(&cmd_drive, HAL_GetTick());
            }

            fast_adas_drive_release = !cliff_control_active &&
                                      (fabsf(cmd_drive) < fabsf(drive_before_adas));
        } else {
            s_cliff_retreat_active = false;
        }

        /* Tính toán cân bằng */
        cmd_drive = -cmd_drive;
        cmd_steer = -cmd_steer;
        balance_update(pitch, gyro_rate, cmd_drive, cmd_steer, CONTROL_DT_S,
                       fast_adas_drive_release);

        float left_spd = 0.0f;
        float right_spd = 0.0f;
        balance_get_motor_speeds(&left_spd, &right_spd);

        stepper_set_speed(MOTOR_LEFT, left_spd);
        stepper_set_speed(MOTOR_RIGHT, right_spd);
        stepper_update_ramp(CONTROL_DT_S);
    } else {
        stepper_stop();
        s_cliff_retreat_active = false;
    }
}

/* Ngắt tần số cao 20 kHz từ TIM7 */
void app_timer_step_tick(void)
{
    stepper_step_isr();
}

/* Ngắt ngoài EXTI từ MPU6050 INT (PB3) */
void app_gpio_exti_callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BOARD_MPU_INT_PIN) {
        imu_notify_data_ready();
    }
}
