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

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static TIM_HandleTypeDef s_htim6;
static TIM_HandleTypeDef s_htim7;
static SPI_HandleTypeDef s_hspi2;

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
    tof_init(&hi2c2);
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
    telemetry_uart_update(now_ms);

    /* Cập nhật Telemetry nạp sẵn vào ACK Payload của NRF24 mỗi 50 ms */
    static uint32_t s_last_nrf_telem_ms = 0;
    if ((now_ms - s_last_nrf_telem_ms) >= 50) {
        s_last_nrf_telem_ms = now_ms;
        float pitch = imu_get_pitch_deg();
        float rate  = imu_get_gyro_rate_dps();
        float left_rpm  = stepper_get_speed(MOTOR_LEFT)  * 60.0f / (float)MOTOR_STEPS_PER_REV;
        float right_rpm = stepper_get_speed(MOTOR_RIGHT) * 60.0f / (float)MOTOR_STEPS_PER_REV;
        uint16_t dist = ultrasonic_get_dist_cm(US_FRONT);
        radio_load_ack_telemetry(sm_get_state(), sm_get_fault_reason(), pitch, rate, left_rpm, right_rpm, dist, 14.8f);
    }
}

/* Ngắt chu kỳ cố định 200 Hz từ TIM6 */
void app_timer_control_tick(void)
{
    /* Đọc IMU với dt cố định 5 ms */
    imu_read(CONTROL_DT_S);

    float pitch = imu_get_pitch_deg();
    float gyro_rate = imu_get_gyro_rate_dps();
    uint32_t imu_err = imu_get_error_count();

    /* Nhận lệnh từ Radio NRF24 */
    float nrf_drive = 0.0f;
    float nrf_steer = 0.0f;
    bool nrf_arm_req = false;
    bool nrf_disarm_req = false;
    bool nrf_estop_req = false;

    radio_get_commands(&nrf_drive, &nrf_steer, &nrf_arm_req, &nrf_disarm_req, &nrf_estop_req);
    bool radio_ok = radio_is_connected();

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
     * - Nếu ESP32 Super Mini (UART) vừa gửi lệnh lái (trong 600ms) -> dùng lệnh UART
     * - Ngược lại nếu NRF24 đang có sóng kết nối -> dùng lệnh NRF24
     * - Nếu cả hai không điều khiển -> tốc độ = 0 */
    float cmd_drive = 0.0f;
    float cmd_steer = 0.0f;

    if (uart_has_drive) {
        cmd_drive = uart_drive;
        cmd_steer = uart_steer;
    } else if (radio_ok) {
        cmd_drive = nrf_drive;
        cmd_steer = nrf_steer;
    }

    /* Coi liên kết điều khiển hợp lệ nếu có sóng NRF hoặc đang dùng UART */
    bool control_link_ok = radio_ok || uart_has_drive || true;

    /* Cập nhật Máy trạng thái */
    sm_update(pitch, control_link_ok, arm_req, disarm_req, estop_req, imu_err);

    if (sm_get_state() == STATE_ARMED) {
        /* Giảm tốc độ tiến nếu phát hiện vật cản phía trước */
        if (cmd_drive > 0.0f && ultrasonic_is_obstacle_detected()) {
            uint16_t dist_front = ultrasonic_get_dist_cm(US_FRONT);
            if (dist_front < OBSTACLE_STOP_DIST_CM) {
                cmd_drive = 0.0f;
            } else if (dist_front < OBSTACLE_SLOWDOWN_DIST_CM) {
                cmd_drive *= 0.5f;
            }
        }

        /* Tính toán cân bằng */
        balance_update(pitch, gyro_rate, cmd_drive, cmd_steer, CONTROL_DT_S);

        float left_spd = 0.0f;
        float right_spd = 0.0f;
        balance_get_motor_speeds(&left_spd, &right_spd);

        stepper_set_speed(MOTOR_LEFT, left_spd);
        stepper_set_speed(MOTOR_RIGHT, right_spd);
        stepper_update_ramp(CONTROL_DT_S);
    } else {
        stepper_stop();
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
