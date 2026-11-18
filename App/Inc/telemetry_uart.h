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
void telemetry_uart_send_string(const char *str);

/* Check if external arm/disarm command was requested via UART */
bool telemetry_uart_get_arm_request(void);
bool telemetry_uart_get_disarm_request(void);

#endif /* TELEMETRY_UART_H */
