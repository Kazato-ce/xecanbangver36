#include "test_nrf24.h"
#include <string.h>

#define NRF_CMD_R_REGISTER      0x00
#define NRF_CMD_W_REGISTER      0x20
#define NRF_CMD_R_RX_PAYLOAD    0x61
#define NRF_CMD_FLUSH_TX        0xE1
#define NRF_CMD_FLUSH_RX        0xE2
#define NRF_CMD_NOP             0xFF

#define NRF_REG_CONFIG          0x00
#define NRF_REG_EN_AA           0x01
#define NRF_REG_EN_RXADDR       0x02
#define NRF_REG_SETUP_AW        0x03
#define NRF_REG_SETUP_RETR      0x04
#define NRF_REG_RF_CH           0x05
#define NRF_REG_RF_SETUP        0x06
#define NRF_REG_STATUS          0x07
#define NRF_REG_RX_ADDR_P0      0x0A
#define NRF_REG_TX_ADDR         0x10
#define NRF_REG_RX_PW_P0        0x11
#define NRF_REG_FIFO_STATUS     0x17

/* Chân kết nối NRF24L01 theo AGENTS.md:
 * SPI2: SCK=PB13, MISO=PB14, MOSI=PB15
 * CE=PB8, CSN=PB9
 */
#define NRF_PORT_CE_CSN         GPIOB
#define NRF_PIN_CE              GPIO_PIN_8
#define NRF_PIN_CSN             GPIO_PIN_9

#define TEST_RADIO_CHANNEL      76

static SPI_HandleTypeDef *s_hspi = NULL;
static uint32_t s_rx_packet_count = 0;

static inline void nrf_delay_us(uint32_t us)
{
    uint32_t ticks = us * (168000000U / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

static inline void csn_low(void)
{
    HAL_GPIO_WritePin(NRF_PORT_CE_CSN, NRF_PIN_CSN, GPIO_PIN_RESET);
    nrf_delay_us(2);
}

static inline void csn_high(void)
{
    nrf_delay_us(2);
    HAL_GPIO_WritePin(NRF_PORT_CE_CSN, NRF_PIN_CSN, GPIO_PIN_SET);
    nrf_delay_us(2);
}

static inline void ce_low(void)  { HAL_GPIO_WritePin(NRF_PORT_CE_CSN, NRF_PIN_CE, GPIO_PIN_RESET); }
static inline void ce_high(void) { HAL_GPIO_WritePin(NRF_PORT_CE_CSN, NRF_PIN_CE, GPIO_PIN_SET); }

uint8_t test_nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { NRF_CMD_R_REGISTER | (reg & 0x1F), 0xFF };
    uint8_t rx[2] = { 0, 0 };
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, 5);
    csn_high();
    return rx[1];
}

static void test_nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { NRF_CMD_W_REGISTER | (reg & 0x1F), val };
    csn_low();
    HAL_SPI_Transmit(s_hspi, tx, 2, 5);
    csn_high();
}

static void test_nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[36];
    if (len > 32) len = 32;
    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], buf, len);
    csn_low();
    HAL_SPI_Transmit(s_hspi, tx, len + 1, 10);
    csn_high();
}

static void test_nrf_read_buf(uint8_t cmd, uint8_t *buf, uint8_t len)
{
    uint8_t tx[36] = {0};
    uint8_t rx[36] = {0};
    if (len > 32) len = 32;
    tx[0] = cmd;
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, len + 1, 10);
    csn_high();
    memcpy(buf, &rx[1], len);
}

static void test_nrf_cmd(uint8_t cmd)
{
    csn_low();
    HAL_SPI_Transmit(s_hspi, &cmd, 1, 5);
    csn_high();
}

void test_nrf_flush(void)
{
    test_nrf_cmd(NRF_CMD_FLUSH_RX);
    test_nrf_cmd(NRF_CMD_FLUSH_TX);
    test_nrf_write_reg(NRF_REG_STATUS, 0x70);
}

bool test_nrf_check_spi(uint8_t *status, uint8_t *config, uint8_t *ch, uint8_t *aw)
{
    if (!s_hspi) return false;
    uint8_t st = test_nrf_read_reg(NRF_REG_STATUS);
    uint8_t cfg = test_nrf_read_reg(NRF_REG_CONFIG);
    uint8_t c = test_nrf_read_reg(NRF_REG_RF_CH);
    uint8_t a = test_nrf_read_reg(NRF_REG_SETUP_AW);

    if (status) *status = st;
    if (config) *config = cfg;
    if (ch)     *ch = c;
    if (aw)     *aw = a;

    return (st != 0x00 && st != 0xFF);
}

bool test_nrf_init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;
    s_rx_packet_count = 0;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = NRF_PIN_CE | NRF_PIN_CSN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF_PORT_CE_CSN, &gpio);

    ce_low();
    csn_high();
    HAL_Delay(100);

    /* Kiểm tra SPI */
    test_nrf_write_reg(NRF_REG_SETUP_AW, 0x03);
    uint8_t aw = test_nrf_read_reg(NRF_REG_SETUP_AW);
    if (aw != 0x03) {
        return false;
    }

    /* Bật Auto-Ack Pipe 0 chuẩn Enhanced ShockBurst (ESB) */
    test_nrf_write_reg(NRF_REG_EN_AA, 0x01);

    /* Enable RX Pipe 0 */
    test_nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);

    /* 5 bytes address */
    test_nrf_write_reg(NRF_REG_SETUP_AW, 0x03);

    /* Setup Retransmit */
    test_nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);

    /* Kênh 76 (2476 MHz) */
    test_nrf_write_reg(NRF_REG_RF_CH, TEST_RADIO_CHANNEL);

    /* 1 Mbps, 0 dBm */
    test_nrf_write_reg(NRF_REG_RF_SETUP, 0x06);

    /* Payload size cố định 16 bytes */
    test_nrf_write_reg(NRF_REG_RX_PW_P0, sizeof(RadioPacket_t));

    /* Địa chỉ pipe 0: 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 */
    const uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
    test_nrf_write_buf(NRF_REG_RX_ADDR_P0, addr, 5);
    test_nrf_write_buf(NRF_REG_TX_ADDR, addr, 5);

    /* Flush FIFO */
    test_nrf_flush();

    /* Tắt tính năng Dynamic Payload */
    test_nrf_write_reg(0x1D, 0x00);
    test_nrf_write_reg(0x1C, 0x00);

    /* Power up, PRX mode, 2-byte CRC */
    test_nrf_write_reg(NRF_REG_CONFIG, 0x0F);

    ce_high(); /* Bắt đầu thu sóng */
    HAL_Delay(2);

    return true;
}

bool test_nrf_read_packet(RadioPacket_t *packet)
{
    if (!s_hspi || !packet) return false;

    /* Kiểm tra có gói tin trong RX FIFO không (Bit 0 = RX_EMPTY) */
    uint8_t fifo_st = test_nrf_read_reg(NRF_REG_FIFO_STATUS);
    if (fifo_st & 0x01) {
        return false; /* Không có gói tin */
    }

    test_nrf_read_buf(NRF_CMD_R_RX_PAYLOAD, (uint8_t*)packet, sizeof(RadioPacket_t));
    test_nrf_write_reg(NRF_REG_STATUS, 0x70); /* Clear IRQ flags */

    if (packet->magic == PROTOCOL_MAGIC_BYTE && packet->version == PROTOCOL_VERSION) {
        uint16_t expected_cs = protocol_calc_checksum((const uint8_t*)packet, sizeof(RadioPacket_t) - 2);
        if (expected_cs == packet->checksum) {
            s_rx_packet_count++;
            return true;
        }
    }

    return false;
}

uint32_t test_nrf_get_rx_count(void)
{
    return s_rx_packet_count;
}
