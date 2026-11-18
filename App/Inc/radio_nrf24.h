#ifndef RADIO_NRF24_H
#define RADIO_NRF24_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool radio_init(SPI_HandleTypeDef *hspi);
void radio_update(uint32_t current_time_ms);
bool radio_is_connected(void);
bool radio_check_hardware(uint8_t *out_status, uint8_t *out_config, uint8_t *out_rf_ch);

void radio_get_commands(float *drive_norm, float *steer_norm, bool *arm_req, bool *disarm_req, bool *estop_req);

#endif /* RADIO_NRF24_H */
