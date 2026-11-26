#ifndef STEPPER_TMC2208_H
#define STEPPER_TMC2208_H

#include <stdint.h>
#include <stdbool.h>

extern volatile int32_t g_motor_steps_left;
extern volatile int32_t g_motor_steps_right;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} MotorId_t;

void stepper_init(void);
void stepper_enable(bool enable);
bool stepper_is_enabled(void);
void stepper_stop(void);
void stepper_emergency_stop_isr(void);

void stepper_set_speed(MotorId_t motor, float steps_per_sec_signed);
void stepper_update_ramp(float dt_s);
float stepper_get_speed(MotorId_t motor);
void motor_get_steps(int32_t *left, int32_t *right);

/* 20 kHz ISR step pulse generator called from TIM7 */
void stepper_step_isr(void);

#endif /* STEPPER_TMC2208_H */
