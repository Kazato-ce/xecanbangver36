#include "telemetry_uart.h"
#include "board_config.h"
#include "config.h"
#include "balance.h"
#include "state_machine.h"
#include "imu_mpu6050.h"
#include "stepper_tmc2208.h"
#include "ultrasonic_hcsr05.h"
#include "tof_vl53l0x.h"
#include "radio_nrf24.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define UART_RX_BUFFER_SIZE     256
#define UART_TX_BUFFER_SIZE     512
#define TELEMETRY_INTERVAL_MS   50    /* 20 Hz telemetry stream to ESP32 */

UART_HandleTypeDef huart1;
static uint8_t s_rx_byte = 0;
static volatile uint8_t s_rx_ring[UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;

static char s_rx_line[160];
static uint16_t s_rx_line_len = 0;

static uint32_t s_last_telemetry_ms = 0;
static uint32_t s_telemetry_seq = 0;

static bool s_req_arm = false;
static bool s_req_disarm = false;

static volatile float s_uart_drive_cmd = 0.0f;
static volatile float s_uart_steer_cmd = 0.0f;
static volatile uint32_t s_uart_last_drive_ms = 0;
static volatile uint32_t s_last_supermini_rx_ms = 0;

static void usart1_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    /* PA9 -> TX, PA10 -> RX */
    gpio.Pin = BOARD_ESP_UART_TX_PIN | BOARD_ESP_UART_RX_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART1) {
        usart1_gpio_init();
    }
}

void telemetry_uart_init(void)
{
    usart1_gpio_init();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = BOARD_ESP_UART_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Đặt mức ưu tiên IRQ = 3 (cao hơn TIM6 ưu tiên 4 để không bao giờ bị nghẽn ORE) */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    s_rx_head = 0;
    s_rx_tail = 0;
    s_rx_line_len = 0;
    s_last_telemetry_ms = 0;
    s_telemetry_seq = 0;

    /* Start non-blocking 1-byte receive interrupt */
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        telemetry_uart_rx_isr();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* Xóa triệt để các cờ lỗi phần cứng ORE, NE, FE, PE tránh treo vĩnh viễn */
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        volatile uint32_t dummy = huart1.Instance->SR;
        dummy = huart1.Instance->DR;
        (void)dummy;
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        huart1.RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
}

void telemetry_feed_rx_byte(uint8_t byte)
{
    s_last_supermini_rx_ms = HAL_GetTick();
    uint16_t next = (uint16_t)((s_rx_head + 1) % UART_RX_BUFFER_SIZE);
    if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next;
    }
}

void telemetry_uart_rx_isr(void)
{
    telemetry_feed_rx_byte(s_rx_byte);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

static void uart_send_string(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0) return;

    /* Tự phục hồi state nếu trước đó bị lỗi hoặc timeout */
    if (huart1.gState != HAL_UART_STATE_READY) {
        huart1.gState = HAL_UART_STATE_READY;
    }

    /* Timeout 80ms: Đủ thời gian truyền cả chuỗi mà không bị cắt cụt do ngắt TIM7/TIM6 */
    HAL_UART_Transmit(&huart1, (const uint8_t*)str, len, 80);
}

static void process_json_command(const char *line)
{
    /* Format: {"type":"set_pid","seq":1,"kp":18.0,"ki":0.0,"kd":0.85,"target":0.0} */
    if (strstr(line, "\"type\":\"set_pid\"") || strstr(line, "\"type\": \"set_pid\"")) {
        float cur_kp, cur_kd, cur_ki, cur_kp_s, cur_ki_s, cur_steer;
        balance_get_pid(&cur_kp, &cur_kd, &cur_ki, &cur_kp_s, &cur_ki_s, &cur_steer);
        float kp = cur_kp, kd = cur_kd, ki = cur_ki;
        float kp_s = cur_kp_s, ki_s = cur_ki_s, steer = cur_steer;
        float target = balance_get_target_offset();
        uint32_t seq = 0;

        const char *p_seq = strstr(line, "\"seq\":");
        if (p_seq) seq = (uint32_t)strtoul(p_seq + 6, NULL, 10);

        const char *p_kp = strstr(line, "\"kp\":");
        const char *p_kd = strstr(line, "\"kd\":");
        const char *p_ki = strstr(line, "\"ki\":");
        const char *p_kps = strstr(line, "\"kp_s\":");
        const char *p_kis = strstr(line, "\"ki_s\":");
        const char *p_str = strstr(line, "\"steer\":");
        const char *p_tgt = strstr(line, "\"target\":");
        const char *p_trm = strstr(line, "\"trim\":");

        if (p_kp) kp = strtof(p_kp + 5, NULL);
        if (p_kd) kd = strtof(p_kd + 5, NULL);
        if (p_ki) ki = strtof(p_ki + 5, NULL);
        if (p_kps) kp_s = strtof(p_kps + 7, NULL);
        if (p_kis) ki_s = strtof(p_kis + 7, NULL);
        if (p_str) steer = strtof(p_str + 8, NULL);
        if (p_tgt) target = strtof(p_tgt + 9, NULL);
        else if (p_trm) target = strtof(p_trm + 7, NULL);

        balance_set_pid(kp, kd, ki, kp_s, ki_s, steer);
        balance_set_target_offset(target);

        char resp[96];
        snprintf(resp, sizeof(resp), "{\"type\":\"ack\",\"seq\":%lu,\"ok\":true,\"message\":\"PID updated\"}\r\n", (unsigned long)seq);
        uart_send_string(resp);
    }
    else if (strstr(line, "\"action\":\"arm_request\"") || strstr(line, "\"action\":\"arm\"")) {
        s_req_arm = true;
        s_req_disarm = false;
        uart_send_string("{\"type\":\"ack\",\"ok\":true,\"message\":\"arm requested\"}\r\n");
    }
    else if (strstr(line, "\"action\":\"disarm\"")) {
        s_req_disarm = true;
        s_req_arm = false;
        uart_send_string("{\"type\":\"ack\",\"ok\":true,\"message\":\"disarmed\"}\r\n");
    }
    else if (strstr(line, "\"action\":\"drive\"") || strstr(line, "\"type\":\"drive\"")) {
        const char *p_spd = strstr(line, "\"speed\":");
        const char *p_str = strstr(line, "\"steer\":");
        float spd = 0.0f, str = 0.0f;
        if (p_spd) spd = strtof(p_spd + 8, NULL);
        if (p_str) str = strtof(p_str + 8, NULL);
        if (fabsf(spd) > 2.0f) spd /= 1000.0f;
        if (fabsf(str) > 2.0f) str /= 1000.0f;
        s_uart_drive_cmd = spd;
        s_uart_steer_cmd = str;
        s_uart_last_drive_ms = HAL_GetTick();
    }
    else if (strstr(line, "\"type\":\"heartbeat\"")) {
        /* Heartbeat acknowledged */
    }
}

static void process_text_command(char *line)
{
    /* Format: SET_PID <kp> <kd> <ki> [kp_s] [ki_s] [steer] */
    if (strncmp(line, "SET_PID", 7) == 0) {
        float kp, kd, ki;
        float kp_s = PID_SPEED_KP, ki_s = PID_SPEED_KI, steer = PID_STEER_KP;
        int n = sscanf(line + 7, "%f %f %f %f %f %f", &kp, &kd, &ki, &kp_s, &ki_s, &steer);
        if (n >= 3) {
            balance_set_pid(kp, kd, ki, kp_s, ki_s, steer);
            char resp[96];
            snprintf(resp, sizeof(resp), "OK PID kp=%.2f kd=%.2f ki=%.2f kp_s=%.2f ki_s=%.2f steer=%.2f\r\n",
                     kp, kd, ki, kp_s, ki_s, steer);
            uart_send_string(resp);
        } else {
            uart_send_string("ERR Syntax: SET_PID <kp> <kd> <ki> [kp_s] [ki_s] [steer]\r\n");
        }
    }
    else if (strncmp(line, "GET_PID", 7) == 0) {
        float kp, kd, ki, kp_s, ki_s, steer;
        balance_get_pid(&kp, &kd, &ki, &kp_s, &ki_s, &steer);
        char resp[96];
        snprintf(resp, sizeof(resp), "PID kp=%.2f kd=%.2f ki=%.2f kp_s=%.2f ki_s=%.2f steer=%.2f\r\n",
                 kp, kd, ki, kp_s, ki_s, steer);
        uart_send_string(resp);
    }
    else if (strncmp(line, "ARM", 3) == 0) {
        s_req_arm = true;
        s_req_disarm = false;
        uart_send_string("OK ARM requested\r\n");
    }
    else if (strncmp(line, "DISARM", 6) == 0) {
        s_req_disarm = true;
        s_req_arm = false;
        uart_send_string("OK DISARM\r\n");
    }
    else if (strncmp(line, "DRIVE", 5) == 0) {
        float spd = 0.0f, str = 0.0f;
        int n = sscanf(line + 5, "%f %f", &spd, &str);
        if (n >= 1) {
            if (fabsf(spd) > 2.0f) spd /= 1000.0f;
            if (fabsf(str) > 2.0f) str /= 1000.0f;
            s_uart_drive_cmd = spd;
            s_uart_steer_cmd = (n >= 2) ? str : 0.0f;
            s_uart_last_drive_ms = HAL_GetTick();
        }
    }
    else if (strncmp(line, "STOP", 4) == 0) {
        s_uart_drive_cmd = 0.0f;
        s_uart_steer_cmd = 0.0f;
        s_uart_last_drive_ms = HAL_GetTick();
    }
}

static void parse_rx_line(char *line)
{
    /* Bỏ khoảng trắng ở đầu */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    if (*line == '{') {
        process_json_command(line);
    } else {
        process_text_command(line);
    }
}

static void send_telemetry(void)
{
    float pitch = imu_get_pitch_deg();
    float rate  = imu_get_gyro_rate_dps();
    float target_pitch = balance_get_target_pitch_deg();

    float left_spd_hz  = stepper_get_speed(MOTOR_LEFT);
    float right_spd_hz = stepper_get_speed(MOTOR_RIGHT);

    /* Quy đổi sang RPM ước lượng: rpm = steps_per_sec / (3200) * 60 */
    float left_rpm  = left_spd_hz  * 60.0f / (float)MOTOR_STEPS_PER_REV;
    float right_rpm = right_spd_hz * 60.0f / (float)MOTOR_STEPS_PER_REV;

    float kp, kd, ki, kp_s, ki_s, steer;
    balance_get_pid(&kp, &kd, &ki, &kp_s, &ki_s, &steer);

    uint16_t dist_f = ultrasonic_get_dist_cm(US_FRONT);
    uint16_t tof_mm = tof_get_distance_mm();

    VehicleState_t state = sm_get_state();
    FaultReason_t fault = sm_get_fault_reason();

    bool mpu_ok = imu_is_calibrated() && (imu_get_error_count() == 0);
    uint32_t imu_err = imu_get_error_count();
    bool nrf_hw = radio_is_hardware_ok();
    bool nrf_link = radio_is_connected();
    bool spm_ok = telemetry_uart_is_supermini_linked();
    float trim = balance_get_target_offset();
    bool upright = (fabsf(pitch - trim) <= ARM_UPRIGHT_ANGLE_DEG);
    float arm_cd = sm_get_auto_arm_countdown_s();

    char buf[360];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"telemetry\",\"seq\":%lu,\"state\":%d,\"fault\":%d,\"pitch\":%.2f,\"target\":%.2f,\"rate\":%.2f,"
        "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,\"kp_s\":%.2f,\"ki_s\":%.2f,\"steer\":%.2f,\"trim\":%.2f,"
        "\"dist_f\":%u,\"tof_mm\":%u,\"bat\":14.8,"
        "\"mpu_ok\":%s,\"imu_err\":%lu,\"nrf_hw\":%s,\"nrf_link\":%s,\"supermini_ok\":%s,\"upright\":%s,\"arm_cd\":%.1f}\r\n",
        (unsigned long)s_telemetry_seq++, (int)state, (int)fault,
        pitch, target_pitch, rate,
        left_rpm, right_rpm, kp, kd, ki, kp_s, ki_s, steer, trim,
        (unsigned)dist_f, (unsigned)tof_mm,
        mpu_ok ? "true" : "false", (unsigned long)imu_err,
        nrf_hw ? "true" : "false", nrf_link ? "true" : "false",
        spm_ok ? "true" : "false", upright ? "true" : "false", arm_cd);

    uart_send_string(buf);
}

void telemetry_uart_update(uint32_t current_time_ms)
{
    /* Tự động kích hoạt lại RX interrupt nếu bị treo hoặc mất cờ */
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        volatile uint32_t dummy = huart1.Instance->SR;
        dummy = huart1.Instance->DR;
        (void)dummy;
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        huart1.RxState = HAL_UART_STATE_READY;
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }

    /* Đọc ký tự từ ring buffer */
    while (s_rx_head != s_rx_tail) {
        char ch = (char)s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1) % UART_RX_BUFFER_SIZE);

        if (ch == '\n' || ch == '\r') {
            if (s_rx_line_len > 0) {
                s_rx_line[s_rx_line_len] = '\0';
                parse_rx_line(s_rx_line);
                s_rx_line_len = 0;
            }
        } else {
            if (s_rx_line_len < (sizeof(s_rx_line) - 1)) {
                s_rx_line[s_rx_line_len++] = ch;
            }
        }
    }

    /* Gửi telemetry định kỳ 20 Hz (50 ms) lên ESP32 Super Mini */
    if ((current_time_ms - s_last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {
        s_last_telemetry_ms = current_time_ms;
        send_telemetry();
    }
}

bool telemetry_uart_get_arm_request(void)
{
    if (s_req_arm) {
        s_req_arm = false;
        return true;
    }
    return false;
}

bool telemetry_uart_get_disarm_request(void)
{
    if (s_req_disarm) {
        s_req_disarm = false;
        return true;
    }
    return false;
}

bool telemetry_uart_get_drive_cmd(float *drive, float *steer)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_uart_last_drive_ms) < 600) {
        if (drive) *drive = s_uart_drive_cmd;
        if (steer) *steer = s_uart_steer_cmd;
        return true;
    }
    return false;
}

bool telemetry_uart_is_supermini_linked(void)
{
    return (s_last_supermini_rx_ms > 0 && ((HAL_GetTick() - s_last_supermini_rx_ms) < 2000));
}
