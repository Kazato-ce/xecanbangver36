#ifndef TELEMETRY_UART_H
#define TELEMETRY_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart1;

void telemetry_uart_init(void);
void telemetry_uart_update(uint32_t current_time_ms);
void telemetry_uart_rx_isr(void);
void telemetry_feed_rx_byte(uint8_t byte);

/* Check if external arm/disarm command was requested via UART */
bool telemetry_uart_get_arm_request(void);
bool telemetry_uart_get_disarm_request(void);

/* Check and retrieve drive/steer command from ESP32 Super Mini via UART */
bool telemetry_uart_get_drive_cmd(float *drive, float *steer);

/* Check if ESP32 Super Mini is currently communicating with STM32 via UART1 */
bool telemetry_uart_is_supermini_linked(void);
uint32_t telemetry_uart_get_tx_drop_count(void);

#endif /* TELEMETRY_UART_H */
