#ifndef APP_H
#define APP_H

#include "main.h"

void app_init(void);
void app_loop(void);

/* Interrupt Handlers Hooks */
void app_timer_control_tick(void); /* TIM6 200 Hz */
void app_timer_step_tick(void);    /* TIM7 20 kHz */
void app_gpio_exti_callback(uint16_t GPIO_Pin);

#endif /* APP_H */
