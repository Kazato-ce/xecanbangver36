#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include <stdbool.h>

void balance_init(void);
void balance_reset(void);
void balance_update(float pitch_deg, float gyro_rate_dps, float cmd_drive, float cmd_steer, float dt_s, bool fast_drive_release);
void balance_get_motor_speeds(float *left_steps_s, float *right_steps_s);

/* Dynamic real-time PID tuning functions (e.g. from ESP32 Super Mini Web UI) */
void balance_set_pid(float kp_angle, float kd_angle, float ki_angle, float kp_speed, float ki_speed, float kp_steer);
void balance_get_pid(float *kp_angle, float *kd_angle, float *ki_angle, float *kp_speed, float *ki_speed, float *kp_steer);
void balance_set_target_offset(float offset_deg);
float balance_get_target_offset(void);
float balance_get_target_pitch_deg(void);

#endif /* BALANCE_H */
