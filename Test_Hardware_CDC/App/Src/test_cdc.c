#include "test_cdc.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define CDC_RX_RING_SIZE 512

static volatile uint8_t  s_rx_ring[CDC_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint32_t s_last_rx_tick = 0;

void cdc_init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
    s_last_rx_tick = 0;
}

void cdc_rx_feed_byte(uint8_t byte)
{
    s_last_rx_tick = HAL_GetTick();
    uint16_t next = (uint16_t)((s_rx_head + 1) % CDC_RX_RING_SIZE);
    if (next != s_rx_tail) {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next;
    }
}

bool cdc_available(void)
{
    return (s_rx_head != s_rx_tail);
}

char cdc_read_char(void)
{
    if (s_rx_head == s_rx_tail) return '\0';
    char ch = (char)s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1) % CDC_RX_RING_SIZE);
    return ch;
}

void cdc_flush_rx(void)
{
    s_rx_tail = s_rx_head;
}

bool cdc_is_connected(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return false;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    return (hcdc != NULL);
}

int cdc_printf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return 0;
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;

    /* Chờ đường truyền USB sẵn sàng (Timeout 30ms) */
    uint32_t start = HAL_GetTick();
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc == NULL) return 0;

    while (hcdc->TxState != 0) {
        if ((HAL_GetTick() - start) > 30) {
            return 0; /* Tránh treo MCU nếu USB không cắm vào máy tính */
        }
    }

    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)len);
    return len;
}

void cdc_print(const char *str)
{
    if (!str) return;
    cdc_printf("%s", str);
}
