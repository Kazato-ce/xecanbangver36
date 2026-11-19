#include "test_tmc2208.h"

#define PIN_EN          GPIO_PIN_0
#define PIN_STEP_L      GPIO_PIN_1
#define PIN_DIR_L       GPIO_PIN_2
#define PIN_STEP_R      GPIO_PIN_3
#define PIN_DIR_R       GPIO_PIN_4

#define PORT_MOTORS     GPIOA

static bool s_enabled = false;

static inline void dwt_delay_us(uint32_t us)
{
    uint32_t ticks = us * (168000000U / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

void test_stepper_init(void)
{
    /* Bật DWT Cycle Counter nếu chưa bật */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};

    /* Cấu hình chân EN: PA0 (Mặc định HIGH = Tắt an toàn) */
    gpio.Pin = PIN_EN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PORT_MOTORS, &gpio);
    HAL_GPIO_WritePin(PORT_MOTORS, PIN_EN, GPIO_PIN_SET);
    s_enabled = false;

    /* Cấu hình các chân STEP và DIR: PA1, PA2, PA3, PA4 */
    gpio.Pin = PIN_STEP_L | PIN_DIR_L | PIN_STEP_R | PIN_DIR_R;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(PORT_MOTORS, &gpio);

    HAL_GPIO_WritePin(PORT_MOTORS, PIN_STEP_L | PIN_STEP_R, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT_MOTORS, PIN_DIR_L | PIN_DIR_R, GPIO_PIN_RESET);
}

void test_stepper_enable(bool enable)
{
    s_enabled = enable;
    if (enable) {
        HAL_GPIO_WritePin(PORT_MOTORS, PIN_EN, GPIO_PIN_RESET); /* PA0 = LOW -> BẬT Driver */
    } else {
        HAL_GPIO_WritePin(PORT_MOTORS, PIN_EN, GPIO_PIN_SET);   /* PA0 = HIGH -> TẮT Driver */
    }
    dwt_delay_us(100);
}

bool test_stepper_is_enabled(void)
{
    return s_enabled;
}

void test_stepper_stop(void)
{
    test_stepper_enable(false);
    HAL_GPIO_WritePin(PORT_MOTORS, PIN_STEP_L | PIN_STEP_R, GPIO_PIN_RESET);
}

void test_stepper_spin_motor(uint8_t motor, int8_t dir, uint32_t steps, uint32_t step_delay_us)
{
    if (!s_enabled) {
        test_stepper_enable(true);
    }

    uint16_t pin_step = (motor == MOTOR_LEFT) ? PIN_STEP_L : PIN_STEP_R;
    uint16_t pin_dir  = (motor == MOTOR_LEFT) ? PIN_DIR_L  : PIN_DIR_R;

    /* Đặt chiều quay */
    GPIO_PinState dir_state = (dir >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(PORT_MOTORS, pin_dir, dir_state);
    dwt_delay_us(20); /* Thời gian chờ DIR ổn định trước xung STEP */

    for (uint32_t i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(PORT_MOTORS, pin_step, GPIO_PIN_SET);
        dwt_delay_us(3); /* Độ rộng xung tối thiểu TMC2208 > 100ns */
        HAL_GPIO_WritePin(PORT_MOTORS, pin_step, GPIO_PIN_RESET);
        dwt_delay_us(step_delay_us);
    }
}

void test_stepper_spin_both(int8_t left_dir, int8_t right_dir, uint32_t steps, uint32_t step_delay_us)
{
    if (!s_enabled) {
        test_stepper_enable(true);
    }

    /* Đặt chiều quay 2 bánh đối xứng (bánh phải lắp ngược hướng với bánh trái) */
    HAL_GPIO_WritePin(PORT_MOTORS, PIN_DIR_L, (left_dir >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PORT_MOTORS, PIN_DIR_R, (right_dir >= 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    dwt_delay_us(20);

    for (uint32_t i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(PORT_MOTORS, PIN_STEP_L | PIN_STEP_R, GPIO_PIN_SET);
        dwt_delay_us(3);
        HAL_GPIO_WritePin(PORT_MOTORS, PIN_STEP_L | PIN_STEP_R, GPIO_PIN_RESET);
        dwt_delay_us(step_delay_us);
    }
}

void test_stepper_ramp_speed(uint8_t motor, uint32_t start_us, uint32_t min_us, uint32_t total_steps)
{
    if (!s_enabled) {
        test_stepper_enable(true);
    }

    uint16_t pin_step = (motor == MOTOR_LEFT) ? PIN_STEP_L : PIN_STEP_R;
    uint16_t pin_dir  = (motor == MOTOR_LEFT) ? PIN_DIR_L  : PIN_DIR_R;

    HAL_GPIO_WritePin(PORT_MOTORS, pin_dir, GPIO_PIN_SET);
    dwt_delay_us(20);

    uint32_t cur_us = start_us;
    uint32_t ramp_steps = total_steps / 2;
    uint32_t step_diff = (start_us - min_us) / (ramp_steps > 0 ? ramp_steps : 1);

    for (uint32_t i = 0; i < total_steps; i++) {
        HAL_GPIO_WritePin(PORT_MOTORS, pin_step, GPIO_PIN_SET);
        dwt_delay_us(3);
        HAL_GPIO_WritePin(PORT_MOTORS, pin_step, GPIO_PIN_RESET);
        dwt_delay_us(cur_us);

        if (i < ramp_steps) {
            if (cur_us > min_us + step_diff) cur_us -= step_diff;
        } else {
            if (cur_us < start_us - step_diff) cur_us += step_diff;
        }
    }
}
