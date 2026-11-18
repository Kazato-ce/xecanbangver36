#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATE_BOOT = 0,
    STATE_CALIBRATING,
    STATE_DISARMED,
    STATE_ARMED,
    STATE_FAULT,
    STATE_TEST_MOTOR
} VehicleState_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_FALL_OVER,
    FAULT_IMU_ERROR,
    FAULT_RADIO_TIMEOUT,
    FAULT_ESTOP_CMD
} FaultReason_t;

void sm_init(void);
void sm_update(float pitch_deg, bool radio_connected, bool arm_cmd, bool disarm_cmd, bool estop_cmd, uint32_t imu_errors);
VehicleState_t sm_get_state(void);
FaultReason_t sm_get_fault_reason(void);
const char* sm_get_state_str(VehicleState_t st);

void sm_start_motor_test(float left_rpm, float right_rpm, uint32_t duration_ms);
void sm_stop_motor_test(void);
bool sm_is_motor_test_active(void);
void sm_get_motor_test_speeds(float *left_rpm, float *right_rpm);

void sm_set_auto_arm(bool enable);
bool sm_get_auto_arm(void);

#endif /* STATE_MACHINE_H */
