#ifndef RADIO_NRF24_H
#define RADIO_NRF24_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

bool radio_init(SPI_HandleTypeDef *hspi);
void radio_update(uint32_t current_time_ms);
bool radio_is_connected(void);
bool radio_check_hardware(uint8_t *status, uint8_t *config, uint8_t *channel);

void radio_get_commands(float *drive_norm, float *steer_norm, bool *arm_req, bool *disarm_req, bool *estop_req);

/* Nạp gói dữ liệu phản hồi Telemetry vào ACK Payload của NRF24 */
void radio_load_ack_telemetry(uint8_t state, uint8_t fault, float pitch_deg, float rate_dps, float left_rpm, float right_rpm, uint16_t dist_cm, float bat_v);

#endif /* RADIO_NRF24_H */
