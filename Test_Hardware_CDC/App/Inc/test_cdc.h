#ifndef TEST_CDC_H
#define TEST_CDC_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void cdc_init(void);
void cdc_rx_feed_byte(uint8_t byte);
bool cdc_available(void);
char cdc_read_char(void);
void cdc_flush_rx(void);
int  cdc_printf(const char *fmt, ...);
void cdc_print(const char *str);
bool cdc_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_CDC_H */
