#include "telemetry_uart.h"
#include "board_config.h"
#include "config.h"
#include "balance.h"
#include "state_machine.h"
#include "imu_mpu6050.h"
#include "stepper_tmc2208.h"
#include "radio_nrf24.h"
#include "ultrasonic_hcsr05.h"
#include "tof_vl53l0x.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
static uint8_t  s_telemetry_mode = 0; /* 0: JSON, 1: PLOT, 2: TEXT */

static bool s_req_arm = false;
static bool s_req_disarm = false;

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

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    s_rx_head = 0;
    s_rx_tail = 0;
    s_rx_line_len = 0;
    s_last_telemetry_ms = 0;
    s_telemetry_seq = 0;
    s_telemetry_mode = 0;

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
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
}

void telemetry_feed_rx_byte(uint8_t byte)
{
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

void telemetry_uart_send_string(const char *str)
{
    if (str && *str) {
        uint16_t len = (uint16_t)strlen(str);

        /* 1. Transmit via USART1 (PA9 TX, PA10 RX) */
        HAL_UART_Transmit(&huart1, (uint8_t*)str, len, 50);

        /* 2. Transmit via USB CDC Virtual COM Port (PA11 DM, PA12 DP) */
        uint32_t t0 = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t*)str, len) == USBD_BUSY) {
            if ((HAL_GetTick() - t0) > 5) break; /* Timeout 5ms neu USB chua san sang */
        }
    }
}

static void uart_send_string(const char *str)
{
    telemetry_uart_send_string(str);
}

static void process_json_command(const char *line)
{
    /* Format: {"type":"set_pid","seq":1,"kp":18.0,"ki":0.0,"kd":0.85,"target":0.0} */
    if (strstr(line, "\"type\":\"set_pid\"") || strstr(line, "\"type\": \"set_pid\"")) {
        float kp = 0, ki = 0, kd = 0, target = 0;
        uint32_t seq = 0;

        const char *p_seq = strstr(line, "\"seq\":");
        if (p_seq) seq = (uint32_t)strtoul(p_seq + 6, NULL, 10);

        const char *p_kp = strstr(line, "\"kp\":");
        const char *p_ki = strstr(line, "\"ki\":");
        const char *p_kd = strstr(line, "\"kd\":");
        const char *p_tgt = strstr(line, "\"target\":");

        float cur_kp, cur_kd, cur_ki, cur_kp_s, cur_ki_s, cur_steer;
        balance_get_pid(&cur_kp, &cur_kd, &cur_ki, &cur_kp_s, &cur_ki_s, &cur_steer);

        if (p_kp) kp = strtof(p_kp + 5, NULL); else kp = cur_kp;
        if (p_kd) kd = strtof(p_kd + 5, NULL); else kd = cur_kd;
        if (p_ki) ki = strtof(p_ki + 5, NULL); else ki = cur_ki;
        if (p_tgt) target = strtof(p_tgt + 9, NULL);

        balance_set_pid(kp, kd, ki, cur_kp_s, cur_ki_s, cur_steer);
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
    else if (strncmp(line, "SET_TARGET", 10) == 0 || strncmp(line, "SET_OFFSET", 10) == 0) {
        float tgt = 0.0f;
        int n = sscanf(line + 10, "%f", &tgt);
        if (n == 1) {
            balance_set_target_offset(tgt);
            char resp[80];
            snprintf(resp, sizeof(resp), "OK Goc can bang moi: Target Offset = %.2f deg\r\n", tgt);
            uart_send_string(resp);
        } else {
            uart_send_string("ERR Cu phap: SET_TARGET <goc_deg> (VD: SET_TARGET -2.98)\r\n");
        }
    }
    else if (strncmp(line, "GET_TARGET", 10) == 0) {
        char resp[80];
        snprintf(resp, sizeof(resp), "Goc can bang hien tai: Target Offset = %.2f deg\r\n", balance_get_target_offset());
        uart_send_string(resp);
    }
    else if (strncmp(line, "ARM", 3) == 0) {
        s_req_arm = true;
        s_req_disarm = false;
        uart_send_string("OK ARM requested (Dung thang xe <= 10 do de bat can bang)\r\n");
    }
    else if (strncmp(line, "DISARM", 6) == 0) {
        s_req_disarm = true;
        s_req_arm = false;
        uart_send_string("OK DISARM (Dong co da tat an toan)\r\n");
    }
    else if (strncmp(line, "AUTO_ARM ON", 11) == 0 || strncmp(line, "AUTO_ARM 1", 10) == 0) {
        sm_set_auto_arm(true);
        uart_send_string("OK AUTO_ARM BAT: Xe se tu dong can bang khi duoc giu thang 1.0s tren san.\r\n");
    }
    else if (strncmp(line, "AUTO_ARM OFF", 12) == 0 || strncmp(line, "AUTO_ARM 0", 10) == 0) {
        sm_set_auto_arm(false);
        uart_send_string("OK AUTO_ARM TAT: Dong co se khong tu quay. Gui lenh 'ARM' khi muon can bang.\r\n");
    }
    else if (strncmp(line, "ESTOP", 5) == 0) {
        s_req_disarm = true;
        s_req_arm = false;
        stepper_emergency_stop_isr();
        uart_send_string("OK ESTOP triggered\r\n");
    }
    else if (strncmp(line, "TEST_MPU", 8) == 0 || strncmp(line, "CHECK_MPU", 9) == 0) {
        uint8_t who = 0;
        float bias = 0, ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        bool ok = imu_diagnostics(&who, &bias, &ax, &ay, &az, &gx, &gy, &gz);
        float pitch = imu_get_pitch_deg();
        float rate  = imu_get_gyro_rate_dps();
        uint32_t errs = imu_get_error_count();

        char rep[512];
        snprintf(rep, sizeof(rep),
            "\r\n--- [1] KIEM TRA MPU6050 (I2C1 PB6/PB7) ---\r\n"
            "WHO_AM_I : 0x%02X (%s)\r\n"
            "Gia toc  : Ax = %+5.2fg | Ay = %+5.2fg | Az = %+5.2fg\r\n"
            "Gyro     : Gx = %+5.1f | Gy = %+5.1f | Gz = %+5.1f dps\r\n"
            "Goc Pitch: %+6.2f deg | Gyro Rate: %+6.2f dps\r\n"
            "Gyro Bias: %+6.2f dps | So loi I2C: %lu\r\n"
            "KET LUAN : %s\r\n"
            "-------------------------------------------\r\n",
            who, (who == 0x68 || who == 0x70 || who == 0x72) ? "DUNG CHIP MPU6050" : "SAI ID / MAT KET NOI",
            ax, ay, az, gx, gy, gz,
            pitch, rate, bias, (unsigned long)errs,
            ok ? "[PASS] MPU6050 HOAT DONG TOT! Nghieng bo mach se thay Pitch thay doi." :
                 "[FAIL] LOI I2C MPU6050! Kiem tra PB6=SCL, PB7=SDA, 3.3V, GND, AD0 noi GND.");
        uart_send_string(rep);
    }
    else if (strncmp(line, "TEST_NRF", 8) == 0 || strncmp(line, "CHECK_NRF", 9) == 0) {
        uint8_t st = 0, cfg = 0, ch = 0;
        bool ok = radio_check_hardware(&st, &cfg, &ch);
        bool conn = radio_is_connected();

        char rep[340];
        snprintf(rep, sizeof(rep),
            "\r\n--- [2] KIEM TRA NRF24L01 (SPI2 PB13/14/15) ---\r\n"
            "STATUS reg : 0x%02X\r\n"
            "CONFIG reg : 0x%02X\r\n"
            "Kenh RF    : %u (Tan so: %u MHz)\r\n"
            "Ket noi    : %s\r\n"
            "KET LUAN   : %s\r\n"
            "-----------------------------------------------\r\n",
            st, cfg, ch, 2400 + ch,
            conn ? "DANG NHAN TIN HIEU DIEU KHIEN (CONNECTED)" : "CHUA CO TIN HIEU TAY CAM",
            ok ? "[PASS] NRF24L01 KET NOI SPI2 TOT!" :
                 "[FAIL] KHONG PHAN HOI SPI2! Kiem tra PB13(SCK), PB14(MISO), PB15(MOSI), PB9(CSN), PB8(CE).");
        uart_send_string(rep);
    }
    else if (strncmp(line, "TEST_MOTOR", 10) == 0) {
        float l_rpm = 0.0f, r_rpm = 0.0f;
        float dur_s = 4.0f;
        int n = sscanf(line + 10, "%f %f %f", &l_rpm, &r_rpm, &dur_s);
        if (n >= 2) {
            if (dur_s <= 0.0f || dur_s > 30.0f) dur_s = 4.0f;
            sm_start_motor_test(l_rpm, r_rpm, (uint32_t)(dur_s * 1000.0f));
            char rep[180];
            snprintf(rep, sizeof(rep),
                "OK TEST_MOTOR Trai=%.1f RPM, Phai=%.1f RPM trong %.1f giay (PA0=LOW)\r\n"
                "-> 2 banh dang quay theo lenh. Gui 'DISARM' de dung ngay.\r\n",
                l_rpm, r_rpm, dur_s);
            uart_send_string(rep);
        } else {
            uart_send_string("ERR Cu phap: TEST_MOTOR <rpm_trai> <rpm_phai> [thoi_gian_giay]\r\nVD: TEST_MOTOR 35 35 5\r\n");
        }
    }
    else if (strncmp(line, "TEST_STEPPER", 12) == 0) {
        sm_start_motor_test(35.0f, 35.0f, 5000);
        uart_send_string(
            "\r\n--- [3] TEST 2 DONG CO BUOC TMC2208 (5 GIAY) ---\r\n"
            "Chan EN   : PA0 -> LOW (Driver BAT)\r\n"
            "Motor Trai: PA1 (STEP), PA2 (DIR) -> 35 RPM\r\n"
            "Motor Phai: PA3 (STEP), PA4 (DIR) -> 35 RPM\r\n"
            "-> Kiem tra 2 banh xe quay nhe nhang trong 5s!\r\n"
            "Luu y: Can cap nguon 12V/Pin vao chan VMOT cua Driver.\r\n"
            "Gui 'DISARM' de dung bat cu luc nao.\r\n"
            "-------------------------------------------------\r\n"
        );
    }
    else if (strncmp(line, "STATUS", 6) == 0 || strncmp(line, "CHECK", 5) == 0) {
        float pitch = imu_get_pitch_deg();
        float rate  = imu_get_gyro_rate_dps();
        VehicleState_t st = sm_get_state();
        FaultReason_t flt = sm_get_fault_reason();
        bool en = stepper_is_enabled();
        float left_spd = stepper_get_speed(MOTOR_LEFT);
        float right_spd = stepper_get_speed(MOTOR_RIGHT);
        float kp, kd, ki, kp_s, ki_s, steer;
        balance_get_pid(&kp, &kd, &ki, &kp_s, &ki_s, &steer);

        uint8_t nrf_st = 0, nrf_cfg = 0, nrf_ch = 0;
        bool nrf_ok = radio_check_hardware(&nrf_st, &nrf_cfg, &nrf_ch);
        uint8_t who = 0;
        bool imu_ok = imu_diagnostics(&who, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

        char rep[750];
        snprintf(rep, sizeof(rep),
            "\r\n================ BANG TONG QUAN HE THONG (STATUS) ================\r\n"
            "Trang thai xe : %s (id=%d) | Ma loi: %d | Auto-Arm: %s\r\n"
            "MPU6050 (I2C1): %s (ID=0x%02X) | Pitch: %+.2f deg | Gyro: %+.2f dps\r\n"
            "Goc can bang  : Offset = %+.2f deg (Target Pitch: %+.2f deg)\r\n"
            "2x TMC2208    : Driver %s (PA0=%s)\r\n"
            "Toc do dong co: Trai = %.1f Hz (%.1f RPM) | Phai = %.1f Hz (%.1f RPM)\r\n"
            "NRF24L01(SPI2): %s (Kenh %u, %s)\r\n"
            "PID can bang  : Kp=%.2f | Kd=%.2f | Ki=%.2f (Kp_s=%.2f, Ki_s=%.3f)\r\n"
            "Cam bien kc   : ToF = %u mm | US Front = %u cm\r\n"
            "==================================================================\r\n",
            sm_get_state_str(st), (int)st, (int)flt, sm_get_auto_arm() ? "BAT" : "TAT",
            imu_ok ? "OK" : "LOI KET NOI", who, pitch, rate,
            balance_get_target_offset(), balance_get_target_pitch_deg(),
            en ? "DANG BAT (ENABLED)" : "DANG TAT (DISABLED)", en ? "LOW" : "HIGH",
            left_spd, left_spd * 60.0f / (float)MOTOR_STEPS_PER_REV,
            right_spd, right_spd * 60.0f / (float)MOTOR_STEPS_PER_REV,
            nrf_ok ? "OK" : "LOI SPI", nrf_ch, radio_is_connected() ? "CO SONG" : "MAT SONG",
            kp, kd, ki, kp_s, ki_s,
            tof_get_distance_mm(), ultrasonic_get_dist_cm(US_FRONT));
        uart_send_string(rep);
    }
    else if (strncmp(line, "PLOT", 4) == 0) {
        s_telemetry_mode = 1;
        uart_send_string("OK Mode Arduino Serial Plotter (PITCH,RATE,RPM_L,RPM_R,TARGET)\r\n");
    }
    else if (strncmp(line, "TEXT", 4) == 0) {
        s_telemetry_mode = 2;
        uart_send_string("OK Mode Text Monitor\r\n");
    }
    else if (strncmp(line, "JSON", 4) == 0) {
        s_telemetry_mode = 0;
        uart_send_string("OK Mode JSON Telemetry (ESP32/Python GCS)\r\n");
    }
    else if (strncmp(line, "HELP", 4) == 0 || strncmp(line, "?", 1) == 0 || strncmp(line, "MENU", 4) == 0) {
        uart_send_string(
            "\r\n================ DANH SACH LENH DIEU KHIEN & TEST (USB/UART) ================\r\n"
            "TEST_MPU       : Kiem tra MPU6050 (WHO_AM_I, Accel X/Y/Z, Gyro, Pitch, Loi I2C)\r\n"
            "TEST_STEPPER   : Test tu dong 2 dong co TMC2208 chay 35 RPM trong 5 giay\r\n"
            "TEST_MOTOR L R : Chay motor theo toc do RPM tuy chon (VD: TEST_MOTOR 40 40 5)\r\n"
            "TEST_NRF       : Kiem tra module song khong day NRF24L01 (SPI2 registers)\r\n"
            "STATUS         : Xem bao cao toan dien (Goc, Dong co, Loi, PID, Nguon, NRF)\r\n"
            "SET_TARGET <goc>: Chinh goc can bang mong muon (VD: SET_TARGET -2.98)\r\n"
            "GET_TARGET     : Xem goc can bang mong muon hien tai\r\n"
            "ARM            : Kich hoat can bang 2 banh (khi xe dung thang quanh goc can bang)\r\n"
            "DISARM         : Ngat dong co an toan lap tuc (PA0=HIGH)\r\n"
            "AUTO_ARM ON/OFF: Bat/tat tu dong can bang khi dung thang 1s tren san\r\n"
            "ESTOP          : Dung khan cap\r\n"
            "PLOT           : Che do ve do thi tren Arduino Serial Plotter\r\n"
            "TEXT           : Che do xem text truc quan theo tung dong\r\n"
            "JSON           : Che do JSON cho Python GCS / Web\r\n"
            "GET_PID        : Xem thong so PID hien tai\r\n"
            "SET_PID        : Cai dat PID (VD: SET_PID 20.0 0.95 0.0)\r\n"
            "==============================================================================\r\n"
        );
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
    uint16_t dist_l = ultrasonic_get_dist_cm(US_LEFT);
    uint16_t dist_r = ultrasonic_get_dist_cm(US_RIGHT);
    uint16_t tof_mm = tof_get_distance_mm();

    VehicleState_t state = sm_get_state();
    FaultReason_t fault = sm_get_fault_reason();

    char buf[300];
    if (s_telemetry_mode == 1) {
        /* Chế độ vẽ đồ thị Arduino Serial Plotter */
        snprintf(buf, sizeof(buf), "PITCH:%.2f,RATE:%.2f,RPM_L:%.1f,RPM_R:%.1f,TARGET:%.2f\r\n",
                 pitch, rate, left_rpm, right_rpm, target_pitch);
    } else if (s_telemetry_mode == 2) {
        /* Chế độ xem text rõ ràng từng dòng */
        snprintf(buf, sizeof(buf), "[DATA] Pitch:%+6.2f deg | Rate:%+6.2f dps | State:%-11s | RPM_L:%+6.1f | RPM_R:%+6.1f\r\n",
                 pitch, rate, sm_get_state_str(state), left_rpm, right_rpm);
    } else {
        /* Chế độ mặc định JSON */
        snprintf(buf, sizeof(buf),
            "{\"type\":\"telemetry\",\"seq\":%lu,\"state\":%d,\"fault\":%d,\"pitch\":%.2f,\"target\":%.2f,\"rate\":%.2f,"
            "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,"
            "\"dist_f\":%u,\"dist_l\":%u,\"dist_r\":%u,\"tof_mm\":%u,\"bat\":14.8}\r\n",
            (unsigned long)s_telemetry_seq++, (int)state, (int)fault,
            pitch, target_pitch, rate,
            left_rpm, right_rpm, kp, kd, ki,
            (unsigned)dist_f, (unsigned)dist_l, (unsigned)dist_r, (unsigned)tof_mm);
    }

    uart_send_string(buf);
}

void telemetry_uart_update(uint32_t current_time_ms)
{
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
