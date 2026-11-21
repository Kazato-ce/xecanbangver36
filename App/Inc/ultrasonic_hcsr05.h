#ifndef ULTRASONIC_HCSR05_H
#define ULTRASONIC_HCSR05_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    US_FRONT = 0,
    US_LEFT  = 1,
    US_RIGHT = 2,
    US_SENSOR_COUNT = 3
} UltrasonicId_t;

void ultrasonic_init(void);
void ultrasonic_update(uint32_t current_time_ms);
uint16_t ultrasonic_get_dist_cm(UltrasonicId_t id);
void ultrasonic_get_all_dist_cm(uint16_t *front, uint16_t *left, uint16_t *right);
bool ultrasonic_is_obstacle_detected(void);

#endif /* ULTRASONIC_HCSR05_H */
