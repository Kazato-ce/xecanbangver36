#ifndef TEST_TMC2208_H
#define TEST_TMC2208_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_LEFT              0
#define MOTOR_RIGHT             1

#define MOTOR_DIR_FORWARD       1
#define MOTOR_DIR_BACKWARD     -1

#ifdef __cplusplus
extern "C" {
#endif

void test_stepper_init(void);
void test_stepper_enable(bool enable);
bool test_stepper_is_enabled(void);
void test_stepper_spin_motor(uint8_t motor, int8_t dir, uint32_t steps, uint32_t step_delay_us);
void test_stepper_spin_both(int8_t left_dir, int8_t right_dir, uint32_t steps, uint32_t step_delay_us);
void test_stepper_ramp_speed(uint8_t motor, uint32_t start_us, uint32_t min_us, uint32_t total_steps);
void test_stepper_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_TMC2208_H */
