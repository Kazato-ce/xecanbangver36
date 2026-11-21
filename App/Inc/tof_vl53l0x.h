#ifndef TOF_VL53L0X_H
#define TOF_VL53L0X_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool tof_init(I2C_HandleTypeDef *hi2c);
void tof_update(uint32_t current_time_ms);
uint16_t tof_get_distance_mm(void);
bool tof_is_valid(void);
void tof_evaluate_cliff(float pitch_deg);
bool tof_is_cliff_detected(void);

#endif /* TOF_VL53L0X_H */
