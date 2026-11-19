#ifndef TEST_NRF24_H
#define TEST_NRF24_H

#include "main.h"
#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool     test_nrf_init(SPI_HandleTypeDef *hspi);
bool     test_nrf_check_spi(uint8_t *status, uint8_t *config, uint8_t *ch, uint8_t *aw);
bool     test_nrf_read_packet(RadioPacket_t *packet);
uint32_t test_nrf_get_rx_count(void);
void     test_nrf_flush(void);
uint8_t  test_nrf_read_reg(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* TEST_NRF24_H */
