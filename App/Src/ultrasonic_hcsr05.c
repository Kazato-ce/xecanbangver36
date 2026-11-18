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
    uint16_t      dist_cm;
} UltrasonicSensor_t;

static UltrasonicSensor_t s_sensors[US_SENSOR_COUNT];
static UltrasonicId_t s_active_sensor = US_FRONT;
static UltrasonicState_t s_state = US_STATE_IDLE;
static uint32_t s_state_start_cyc = 0;
static uint32_t s_echo_rise_cyc = 0;
static uint32_t s_last_ping_ms = 0;

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

void ultrasonic_init(void)
{
    /* Front: TRIG PC0, ECHO PC1 */
    s_sensors[US_FRONT].trig_port = BOARD_US_FRONT_TRIG_PORT;
    s_sensors[US_FRONT].trig_pin  = BOARD_US_FRONT_TRIG_PIN;
    s_sensors[US_FRONT].echo_port = BOARD_US_FRONT_ECHO_PORT;
    s_sensors[US_FRONT].echo_pin  = BOARD_US_FRONT_ECHO_PIN;
    s_sensors[US_FRONT].dist_cm   = 999;

    /* Left: TRIG PC2, ECHO PC3 */
    s_sensors[US_LEFT].trig_port = BOARD_US_LEFT_TRIG_PORT;
    s_sensors[US_LEFT].trig_pin  = BOARD_US_LEFT_TRIG_PIN;
    s_sensors[US_LEFT].echo_port = BOARD_US_LEFT_ECHO_PORT;
    s_sensors[US_LEFT].echo_pin  = BOARD_US_LEFT_ECHO_PIN;
    s_sensors[US_LEFT].dist_cm   = 999;

    /* Right: TRIG PC4, ECHO PC5 */
    s_sensors[US_RIGHT].trig_port = BOARD_US_RIGHT_TRIG_PORT;
    s_sensors[US_RIGHT].trig_pin  = BOARD_US_RIGHT_TRIG_PIN;
    s_sensors[US_RIGHT].echo_port = BOARD_US_RIGHT_ECHO_PORT;
    s_sensors[US_RIGHT].echo_pin  = BOARD_US_RIGHT_ECHO_PIN;
    s_sensors[US_RIGHT].dist_cm   = 999;

    for (int i = 0; i < US_SENSOR_COUNT; i++) {
        HAL_GPIO_WritePin(s_sensors[i].trig_port, s_sensors[i].trig_pin, GPIO_PIN_RESET);
    }

    s_state = US_STATE_IDLE;
    s_active_sensor = US_FRONT;
    s_last_ping_ms = 0;
}

void ultrasonic_update(uint32_t current_time_ms)
{
    UltrasonicSensor_t *s = &s_sensors[s_active_sensor];
    uint32_t now_cyc = dwt_get_cyc();

    switch (s_state) {
        case US_STATE_IDLE:
            if ((current_time_ms - s_last_ping_ms) >= US_PING_INTERVAL_MS) {
                /* Bắt đầu xung TRIG mức cao */
                HAL_GPIO_WritePin(s->trig_port, s->trig_pin, GPIO_PIN_SET);
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
            } else if ((now_cyc - s_state_start_cyc) >= us_to_dwt_cycles(3000)) {
                /* Timeout 3ms không thấy cạnh lên */
                s->dist_cm = 999;
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
                if (dist > 400) dist = 999;
                s->dist_cm = dist;
                s_state = US_STATE_COOLDOWN;
                s_state_start_cyc = now_cyc;
            } else if ((now_cyc - s_state_start_cyc) >= us_to_dwt_cycles(25000)) {
                /* Timeout ~25ms tương đương ~4m */
                s->dist_cm = 999;
                s_state = US_STATE_COOLDOWN;
                s_state_start_cyc = now_cyc;
            }
            break;

        case US_STATE_COOLDOWN:
            /* Nghỉ ngắn trước khi chuyển sang cảm biến kế tiếp */
            if ((now_cyc - s_state_start_cyc) >= us_to_dwt_cycles(1000)) {
                s_active_sensor = (s_active_sensor + 1) % US_SENSOR_COUNT;
                s_last_ping_ms = current_time_ms;
                s_state = US_STATE_IDLE;
            }
            break;
    }
}

uint16_t ultrasonic_get_dist_cm(UltrasonicId_t id)
{
    return (id < US_SENSOR_COUNT) ? s_sensors[id].dist_cm : 999;
}

bool ultrasonic_is_obstacle_detected(void)
{
    return (s_sensors[US_FRONT].dist_cm < OBSTACLE_SLOWDOWN_DIST_CM ||
            s_sensors[US_LEFT].dist_cm  < OBSTACLE_SLOWDOWN_DIST_CM ||
            s_sensors[US_RIGHT].dist_cm < OBSTACLE_SLOWDOWN_DIST_CM);
}
