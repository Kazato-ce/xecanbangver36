#include "ultrasonic_hcsr05.h"
#include "board_config.h"
#include "config.h"

typedef enum {
    US_STATE_IDLE = 0,
    US_STATE_TRIG_PULSE,
    US_STATE_WAIT_ECHO_RISE,
    US_STATE_WAIT_ECHO_FALL,
    US_STATE_COOLDOWN
} UltrasonicState_t;

typedef struct {
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
    volatile uint16_t dist_cm;
    volatile UltrasonicSampleStatus_t sample_status;
    volatile uint32_t sample_time_ms;
    uint16_t      hist[3];
    uint8_t       hist_idx;
    uint8_t       hist_count;
} UltrasonicSensor_t;

static UltrasonicSensor_t s_sensors[US_SENSOR_COUNT];
static UltrasonicId_t s_active_sensor = US_FRONT;
static UltrasonicState_t s_state = US_STATE_IDLE;
static uint32_t s_state_start_cyc = 0;
static uint32_t s_echo_rise_cyc = 0;
static uint32_t s_last_ping_ms = 0;
static bool s_dwt_enabled = false;

static inline uint32_t dwt_get_cyc(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t dwt_cycles_to_us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}

static inline uint32_t us_to_dwt_cycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000U);
}

static uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t min = a;
    uint16_t max = a;

    if (b < min) min = b;
    if (c < min) min = c;
    if (b > max) max = b;
    if (c > max) max = c;

    return (uint16_t)((uint32_t)a + b + c - min - max);
}

static void ultrasonic_publish_sample(UltrasonicSensor_t *s,
                                      UltrasonicSampleStatus_t status,
                                      uint16_t distance_cm,
                                      uint32_t current_time_ms)
{
    s->sample_status = US_SAMPLE_TIMEOUT_OR_TOO_CLOSE;
    s->dist_cm = distance_cm;
    s->sample_time_ms = current_time_ms;
    s->sample_status = status;
}

static void ultrasonic_push_sample(UltrasonicSensor_t *s,
                                   UltrasonicSampleStatus_t status,
                                   uint16_t raw_cm,
                                   uint32_t current_time_ms)
{
    if (status == US_SAMPLE_VALID && raw_cm < 2U) {
        status = US_SAMPLE_TOO_CLOSE;
    } else if (status == US_SAMPLE_VALID && raw_cm > 400U) {
        status = US_SAMPLE_OUT_OF_RANGE;
    }

    if (status != US_SAMPLE_VALID) {
        s->hist_count = 0;
        s->hist_idx = 0;
        uint16_t distance_cm = (status == US_SAMPLE_OUT_OF_RANGE) ? 999U : 0U;
        ultrasonic_publish_sample(s, status, distance_cm, current_time_ms);
        return;
    }

    s->hist[s->hist_idx] = raw_cm;
    s->hist_idx = (uint8_t)((s->hist_idx + 1U) % 3U);
    if (s->hist_count < 3U) {
        s->hist_count++;
    }

    uint16_t filtered_cm = raw_cm;
    if (raw_cm > OBSTACLE_STOP_DIST_CM && s->hist_count == 2U) {
        filtered_cm = (s->hist[0] < s->hist[1]) ? s->hist[0] : s->hist[1];
    } else if (raw_cm > OBSTACLE_STOP_DIST_CM && s->hist_count == 3U) {
        filtered_cm = median3_u16(s->hist[0], s->hist[1], s->hist[2]);
    }

    ultrasonic_publish_sample(s, US_SAMPLE_VALID, filtered_cm, current_time_ms);
}

void ultrasonic_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_enabled = ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) &&
                    ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);

    /* Front: TRIG PC0, ECHO PC1 */
    s_sensors[US_FRONT].trig_port = BOARD_US_FRONT_TRIG_PORT;
    s_sensors[US_FRONT].trig_pin  = BOARD_US_FRONT_TRIG_PIN;
    s_sensors[US_FRONT].echo_port = BOARD_US_FRONT_ECHO_PORT;
    s_sensors[US_FRONT].echo_pin  = BOARD_US_FRONT_ECHO_PIN;
    s_sensors[US_FRONT].dist_cm   = 999U;
    s_sensors[US_FRONT].sample_status = US_SAMPLE_NO_DATA;
    s_sensors[US_FRONT].sample_time_ms = 0U;

    /* Left: TRIG PC2, ECHO PC3 */
    s_sensors[US_LEFT].trig_port = BOARD_US_LEFT_TRIG_PORT;
    s_sensors[US_LEFT].trig_pin  = BOARD_US_LEFT_TRIG_PIN;
    s_sensors[US_LEFT].echo_port = BOARD_US_LEFT_ECHO_PORT;
    s_sensors[US_LEFT].echo_pin  = BOARD_US_LEFT_ECHO_PIN;
    s_sensors[US_LEFT].dist_cm   = 999U;
    s_sensors[US_LEFT].sample_status = US_SAMPLE_NO_DATA;
    s_sensors[US_LEFT].sample_time_ms = 0U;

    /* Right: TRIG PC4, ECHO PC5 */
    s_sensors[US_RIGHT].trig_port = BOARD_US_RIGHT_TRIG_PORT;
    s_sensors[US_RIGHT].trig_pin  = BOARD_US_RIGHT_TRIG_PIN;
    s_sensors[US_RIGHT].echo_port = BOARD_US_RIGHT_ECHO_PORT;
    s_sensors[US_RIGHT].echo_pin  = BOARD_US_RIGHT_ECHO_PIN;
    s_sensors[US_RIGHT].dist_cm   = 999U;
    s_sensors[US_RIGHT].sample_status = US_SAMPLE_NO_DATA;
    s_sensors[US_RIGHT].sample_time_ms = 0U;

    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        s_sensors[i].hist[0] = 999U;
        s_sensors[i].hist[1] = 999U;
        s_sensors[i].hist[2] = 999U;
        s_sensors[i].hist_idx = 0;
        s_sensors[i].hist_count = 0;
        HAL_GPIO_WritePin(s_sensors[i].trig_port, s_sensors[i].trig_pin, GPIO_PIN_RESET);
    }

    s_state = US_STATE_IDLE;
    s_active_sensor = US_FRONT;
    s_last_ping_ms = 0;
}

void ultrasonic_update(uint32_t current_time_ms)
{
    if (!s_dwt_enabled) {
        return;
    }

    UltrasonicSensor_t *s = &s_sensors[s_active_sensor];
    uint32_t now_cyc = dwt_get_cyc();

    switch (s_state) {
        case US_STATE_IDLE:
            if ((current_time_ms - s_last_ping_ms) >= US_PING_INTERVAL_MS) {
                /* Bắt đầu xung TRIG mức cao */
                HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
                s_last_ping_ms = current_time_ms;
                s_state_start_cyc = now_cyc;
                s_state = US_STATE_TRIG_PULSE;
            }
            break;

        case US_STATE_TRIG_PULSE:
            /* Giữ xung TRIG tối thiểu 12 us */
            if ((now_cyc - s_state_start_cyc) >= us_to_dwt_cycles(12)) {
                HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_RESET);
                s_state_start_cyc = now_cyc;
                s_state = US_STATE_WAIT_ECHO_RISE;
            }
            break;

        case US_STATE_WAIT_ECHO_RISE:
            /* Chờ cạnh lên của ECHO */
            if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_SET) {
                s_echo_rise_cyc = now_cyc;
                s_state_start_cyc = now_cyc;
                s_state = US_STATE_WAIT_ECHO_FALL;
            } else if ((now_cyc - s_state_start_cyc) >=
                       us_to_dwt_cycles(US_ECHO_RISE_TIMEOUT_US)) {
                /* Chờ lâu hơn để không bỏ lỡ cạnh lên Echo. */
                ultrasonic_push_sample(s, US_SAMPLE_OUT_OF_RANGE, 999U, current_time_ms);
                s_state = US_STATE_COOLDOWN;
                s_state_start_cyc = now_cyc;
            }
            break;

        case US_STATE_WAIT_ECHO_FALL:
            /* Chờ cạnh xuống của ECHO */
            if (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET) {
                uint32_t echo_us = dwt_cycles_to_us(now_cyc - s_echo_rise_cyc);
                /* Khoảng cách (cm) = thời gian xung (us) / 58 */
                uint16_t dist = (uint16_t)(echo_us / 58);
                ultrasonic_push_sample(s, US_SAMPLE_VALID, dist, current_time_ms);
                s_state = US_STATE_COOLDOWN;
                s_state_start_cyc = now_cyc;
            } else if ((now_cyc - s_state_start_cyc) >=
                       us_to_dwt_cycles(US_ECHO_FALL_TIMEOUT_US)) {
                /* Echo bị giữ mức cao quá thời gian đo: không coi là đường thông thoáng. */
                ultrasonic_push_sample(s, US_SAMPLE_TIMEOUT_OR_TOO_CLOSE, 0U, current_time_ms);
                s_state = US_STATE_COOLDOWN;
                s_state_start_cyc = now_cyc;
            }
            break;

        case US_STATE_COOLDOWN:
            /* Nghỉ ngắn trước khi chuyển sang cảm biến kế tiếp */
            if (((now_cyc - s_state_start_cyc) >= us_to_dwt_cycles(1000)) &&
                (HAL_GPIO_ReadPin(s->echo_port, s->echo_pin) == GPIO_PIN_RESET)) {
                s_active_sensor = (s_active_sensor + 1) % US_SENSOR_COUNT;
                s_state = US_STATE_IDLE;
            }
            break;
    }
}

uint16_t ultrasonic_get_dist_cm(UltrasonicId_t id)
{
    return (id < US_SENSOR_COUNT) ? s_sensors[id].dist_cm : 999;
}

UltrasonicSample_t ultrasonic_get_sample(UltrasonicId_t id)
{
    UltrasonicSample_t sample = { 999U, US_SAMPLE_NO_DATA, 0U };
    if (id < US_SENSOR_COUNT) {
        sample.status = s_sensors[id].sample_status;
        sample.distance_cm = s_sensors[id].dist_cm;
        sample.timestamp_ms = s_sensors[id].sample_time_ms;
    }
    return sample;
}

void ultrasonic_get_all_dist_cm(uint16_t *front, uint16_t *left, uint16_t *right)
{
    if (front != NULL) *front = s_sensors[US_FRONT].dist_cm;
    if (left != NULL) *left = s_sensors[US_LEFT].dist_cm;
    if (right != NULL) *right = s_sensors[US_RIGHT].dist_cm;
}

bool ultrasonic_is_obstacle_detected(void)
{
    for (uint8_t i = 0U; i < US_SENSOR_COUNT; i++) {
        UltrasonicSample_t sample = ultrasonic_get_sample((UltrasonicId_t)i);
        if (sample.status == US_SAMPLE_TOO_CLOSE ||
            sample.status == US_SAMPLE_TIMEOUT_OR_TOO_CLOSE ||
            (sample.status == US_SAMPLE_VALID &&
             sample.distance_cm < OBSTACLE_SLOWDOWN_DIST_CM)) {
            return true;
        }
    }
    return false;
}
