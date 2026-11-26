#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATE_BOOT = 0,
    STATE_CALIBRATING,
    STATE_DISARMED,
    STATE_ARMED,
    STATE_FAULT
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
float sm_get_auto_arm_countdown_s(void);
uint32_t sm_get_upright_stable_count(void);
const char* sm_get_state_str(VehicleState_t st);

#endif /* STATE_MACHINE_H */
