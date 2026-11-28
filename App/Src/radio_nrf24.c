#include "radio_nrf24.h"
#include "board_config.h"
#include "config.h"
#include "protocol.h"
#include <string.h>

/* NRF24L01 Commands & Registers */
#define NRF_CMD_R_REGISTER          0x00
#define NRF_CMD_W_REGISTER          0x20
#define NRF_CMD_R_RX_PAYLOAD        0x61
#define NRF_CMD_W_TX_PAYLOAD        0xA0
#define NRF_CMD_FLUSH_TX            0xE1
#define NRF_CMD_FLUSH_RX            0xE2
#define NRF_CMD_NOP                 0xFF

#define NRF_REG_CONFIG              0x00
#define NRF_REG_EN_AA               0x01
#define NRF_REG_EN_RXADDR           0x02
#define NRF_REG_SETUP_AW            0x03
#define NRF_REG_SETUP_RETR          0x04
#define NRF_REG_RF_CH               0x05
#define NRF_REG_RF_SETUP            0x06
#define NRF_REG_STATUS              0x07
#define NRF_REG_RX_ADDR_P0          0x0A
#define NRF_REG_TX_ADDR             0x10
#define NRF_REG_RX_PW_P0            0x11
#define NRF_REG_FIFO_STATUS         0x17

static SPI_HandleTypeDef *s_hspi = NULL;
static uint32_t s_last_packet_ms = 0;
static bool s_connected = false;
static bool s_initialized = false;

static float s_cmd_drive = 0.0f;
static float s_cmd_steer = 0.0f;
static bool  s_cmd_arm = false;
static bool  s_cmd_disarm = false;
static bool  s_cmd_estop = false;
static volatile bool s_adas_enabled = false;

static uint32_t s_packet_count = 0;
static uint16_t s_last_seq = 0;
static int16_t  s_last_raw_drive = 0;
static int16_t  s_last_raw_steer = 0;
static uint8_t  s_last_raw_status = 0;

static inline void nrf_delay_us(uint32_t us)
{
    uint32_t ticks = us * (168000000U / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

static inline void csn_low(void)
{
    HAL_GPIO_WritePin(BOARD_NRF_CSN_PORT, BOARD_NRF_CSN_PIN, GPIO_PIN_RESET);
    nrf_delay_us(2);
}

static inline void csn_high(void)
{
    nrf_delay_us(2);
    HAL_GPIO_WritePin(BOARD_NRF_CSN_PORT, BOARD_NRF_CSN_PIN, GPIO_PIN_SET);
    nrf_delay_us(2);
}

static inline void ce_low(void)   { HAL_GPIO_WritePin(BOARD_NRF_CE_PORT,  BOARD_NRF_CE_PIN,  GPIO_PIN_RESET); }
static inline void ce_high(void)  { HAL_GPIO_WritePin(BOARD_NRF_CE_PORT,  BOARD_NRF_CE_PIN,  GPIO_PIN_SET); }

/* Xóa triệt để các cờ RXNE và OVR còn sót lại trong SPI2 trước khi truyền nhận dữ liệu */
static inline void spi_clear_flags(void)
{
    if (!s_hspi || !s_hspi->Instance) return;
    __IO uint32_t tmpreg;
    while (__HAL_SPI_GET_FLAG(s_hspi, SPI_FLAG_RXNE)) {
        tmpreg = s_hspi->Instance->DR;
        (void)tmpreg;
    }
    if (__HAL_SPI_GET_FLAG(s_hspi, SPI_FLAG_OVR)) {
        tmpreg = s_hspi->Instance->DR;
        tmpreg = s_hspi->Instance->SR;
        (void)tmpreg;
    }
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { NRF_CMD_R_REGISTER | (reg & 0x1F), 0xFF };
    uint8_t rx[2] = { 0, 0 };
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, 5);
    csn_high();
    return rx[1];
}

static void nrf_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { NRF_CMD_W_REGISTER | (reg & 0x1F), val };
    uint8_t rx[2] = { 0, 0 };
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, 5);
    csn_high();
}

static void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[36] = {0};
    uint8_t rx[36] = {0};
    if (len > 32) len = 32;
    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], buf, len);
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, len + 1, 10);
    csn_high();
}

static void nrf_read_buf(uint8_t cmd, uint8_t *buf, uint8_t len)
{
    uint8_t tx[36];
    uint8_t rx[36] = {0};
    if (len > 32) len = 32;
    tx[0] = cmd;
    memset(&tx[1], 0xFF, len);
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, len + 1, 10);
    csn_high();
    memcpy(buf, &rx[1], len);
}

static void nrf_send_cmd(uint8_t cmd)
{
    uint8_t rx = 0;
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, &cmd, &rx, 1, 5);
    csn_high();
}

static inline uint8_t nrf_get_status(void)
{
    uint8_t cmd = NRF_CMD_NOP;
    uint8_t rx = 0;
    spi_clear_flags();
    csn_low();
    HAL_SPI_TransmitReceive(s_hspi, &cmd, &rx, 1, 2);
    csn_high();
    return rx;
}

static void radio_apply_config(void)
{
    ce_low();
    csn_high();
    nrf_delay_us(100);

    /* Flush TX & RX FIFO */
    nrf_send_cmd(NRF_CMD_FLUSH_RX);
    nrf_send_cmd(NRF_CMD_FLUSH_TX);

    /* Xóa cờ ngắt */
    nrf_write_reg(NRF_REG_STATUS, 0x70);

    /* Bật Auto-Ack Pipe 0: Chuẩn Enhanced ShockBurst (ESB) tương thích RF24 */
    nrf_write_reg(NRF_REG_EN_AA, 0x01);

    /* Enable RX Pipe 0 */
    nrf_write_reg(NRF_REG_EN_RXADDR, 0x01);

    /* Address width = 5 bytes */
    nrf_write_reg(NRF_REG_SETUP_AW, 0x03);

    /* Tự động truyền lại: không cần vì STM32 ở vai trò PRX */
    nrf_write_reg(NRF_REG_SETUP_RETR, 0x00);

    /* RF Channel: Kênh 76 (2476 MHz) */
    nrf_write_reg(NRF_REG_RF_CH, RADIO_CHANNEL);

    /* 1 Mbps, 0 dBm (RF_PWR=11, RF_DR_HIGH=0, RF_DR_LOW=0) */
    nrf_write_reg(NRF_REG_RF_SETUP, 0x06);

    /* RX Payload width cố định cho Pipe 0 */
    nrf_write_reg(NRF_REG_RX_PW_P0, sizeof(RadioPacket_t));

    /* RX & TX Address Pipe 0 */
    const uint8_t rx_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
    nrf_write_buf(NRF_REG_RX_ADDR_P0, rx_addr, 5);
    nrf_write_buf(NRF_REG_TX_ADDR, rx_addr, 5);

    /* Tắt tính năng Dynamic Payload và ACK Payload để giữ chế độ 1-chiều siêu ổn định */
    nrf_write_reg(0x1D, 0x00); /* FEATURE = 0 */
    nrf_write_reg(0x1C, 0x00); /* DYNPD = 0 */

    /* Clear interrupts */
    nrf_write_reg(NRF_REG_STATUS, 0x70);

    /* Power up, PRX mode, 2-byte CRC (EN_CRC=1, CRCO=1, PWR_UP=1, PRIM_RX=1) */
    nrf_write_reg(NRF_REG_CONFIG, 0x0F);

    /* Flush lại lần nữa */
    nrf_send_cmd(NRF_CMD_FLUSH_RX);
    nrf_send_cmd(NRF_CMD_FLUSH_TX);
    nrf_write_reg(NRF_REG_STATUS, 0x70);

    ce_high(); /* Bắt đầu lắng nghe RX liên tục */
    nrf_delay_us(150);
}

bool radio_init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;
    s_connected = false;
    s_last_packet_ms = 0;
    s_initialized = false;
    s_packet_count = 0;
    s_last_seq = 0;
    s_last_raw_drive = 0;
    s_last_raw_steer = 0;
    s_last_raw_status = 0;
    s_adas_enabled = false;

    ce_low();
    csn_high();
    HAL_Delay(100); /* Chờ nguồn NRF24 PA+LNA ổn định */

    spi_clear_flags();

    /* Kiểm tra giao tiếp SPI qua thanh ghi SETUP_AW và RF_CH */
    nrf_write_reg(NRF_REG_SETUP_AW, 0x03);
    uint8_t aw = nrf_read_reg(NRF_REG_SETUP_AW);

    nrf_write_reg(NRF_REG_RF_CH, RADIO_CHANNEL);
    uint8_t ch = nrf_read_reg(NRF_REG_RF_CH);

    if (aw != 0x03 || ch != RADIO_CHANNEL) {
        s_initialized = false;
        return false; /* Chưa gắn module hoặc mất kết nối SPI */
    }

    radio_apply_config();
    s_initialized = true;
    s_last_raw_status = nrf_get_status();
    return true;
}

void radio_update(uint32_t current_time_ms)
{
    if (!s_hspi) return;

    /* Tự động dò và khởi tạo lại nếu module NRF24 chưa được phát hiện lúc boot */
    if (!s_initialized) {
        static uint32_t s_last_reinit_ms = 0;
        if ((current_time_ms - s_last_reinit_ms) >= 1000) {
            s_last_reinit_ms = current_time_ms;
            spi_clear_flags();
            nrf_write_reg(NRF_REG_SETUP_AW, 0x03);
            uint8_t aw = nrf_read_reg(NRF_REG_SETUP_AW);
            nrf_write_reg(NRF_REG_RF_CH, RADIO_CHANNEL);
            uint8_t ch = nrf_read_reg(NRF_REG_RF_CH);
            if (aw == 0x03 && ch == RADIO_CHANNEL) {
                radio_apply_config();
                s_initialized = true;
                s_last_raw_status = nrf_get_status();
            }
        }
        return;
    }

    /* Giới hạn tần số quét SPI ở mức ~500 Hz (2000 us) để không làm nghẽn bus SPI và triệt tiêu nhiễu cao tần cho khối RF */
    static uint32_t s_last_poll_cyc = 0;
    uint32_t now_cyc = DWT->CYCCNT;
    if ((now_cyc - s_last_poll_cyc) < (2000U * (168000000U / 1000000U))) {
        return;
    }
    s_last_poll_cyc = now_cyc;

    /* Kiểm tra nhanh trạng thái phần cứng và hàng đợi gói tin qua lệnh NOP */
    uint8_t status = nrf_get_status();
    s_last_raw_status = status;
    static uint8_t s_status_err_count = 0;

    /* 0xFF nghĩa là đường MISO bị treo mức cao (đứt dây hoặc mất nguồn module) */
    if (status == 0xFF) {
        if (++s_status_err_count >= 50) {
            s_initialized = false;
            s_connected = false;
        }
        return;
    }
    s_status_err_count = 0;

    /* Kiểm tra xem có dữ liệu trong RX FIFO không:
     * - Bit 6 (RX_DR): Cờ ngắt có dữ liệu mới nhận
     * - Bits 3:1 (RX_P_NO): 000..101 = Pipe 0..5 có payload; 111 (0x07) = RX FIFO rỗng */
    uint8_t rx_pipe = (status >> 1) & 0x07;
    if ((status & 0x40) || (rx_pipe < 6)) {
        uint8_t rx_guard = 0;
        while (rx_guard++ < 4) {
            RadioPacket_t packet;
            nrf_read_buf(NRF_CMD_R_RX_PAYLOAD, (uint8_t*)&packet, sizeof(RadioPacket_t));

            /* Kiểm tra tính toàn vẹn của gói tin */
            if (packet.magic == PROTOCOL_MAGIC_BYTE && packet.version == PROTOCOL_VERSION) {
                uint16_t expected_cs = protocol_calc_checksum((const uint8_t*)&packet, sizeof(RadioPacket_t) - 2);
                if (expected_cs == packet.checksum) {
                    s_last_packet_ms = current_time_ms;
                    s_connected = true;
                    s_packet_count++;
                    s_last_seq = packet.seq;
                    s_last_raw_drive = packet.drive_cmd;
                    s_last_raw_steer = packet.steer_cmd;

                    if ((packet.reserved[0] & PROTOCOL_ADAS_STATE_VALID) != 0U) {
                        s_adas_enabled = ((packet.reserved[0] & PROTOCOL_ADAS_ENABLED) != 0U);
                    }

                    s_cmd_drive = (float)packet.drive_cmd / 1000.0f; /* Normalization -1.0 .. +1.0 */
                    s_cmd_steer = (float)packet.steer_cmd / 1000.0f;

                    if (packet.cmd == CMD_ARM) {
                        s_cmd_arm = true;
                        s_cmd_disarm = false;
                        s_cmd_estop = false;
                    } else if (packet.cmd == CMD_DISARM) {
                        s_cmd_disarm = true;
                        s_cmd_arm = false;
                    } else if (packet.cmd == CMD_ESTOP || (packet.flags & 0x01)) {
                        s_cmd_estop = true;
                    }
                }
            }

            uint8_t fifo_st = nrf_read_reg(NRF_REG_FIFO_STATUS);
            if (fifo_st == 0x00 || fifo_st == 0xFF || (fifo_st & 0x01)) {
                /* FIFO rỗng (RX_EMPTY) hoặc lỗi SPI */
                break;
            }
        }

        if (rx_guard >= 4) {
            nrf_send_cmd(NRF_CMD_FLUSH_RX);
        }

        /* Xóa cờ ngắt RX_DR (và cả TX_DS, MAX_RT nếu có) */
        nrf_write_reg(NRF_REG_STATUS, 0x70);
    }

    /* Kiểm tra timeout mất sóng */
    if (s_connected && ((current_time_ms - s_last_packet_ms) > RADIO_TIMEOUT_MS)) {
        s_connected = false;
        s_cmd_drive = 0.0f;
        s_cmd_steer = 0.0f;
    }
}

bool radio_is_connected(void)
{
    return s_connected;
}

bool radio_is_hardware_ok(void)
{
    if (!s_hspi || !s_initialized) return false;
    return (s_last_raw_status != 0xFF);
}

uint32_t radio_get_packet_count(void)
{
    return s_packet_count;
}

uint16_t radio_get_last_seq(void)
{
    return s_last_seq;
}

uint32_t radio_get_last_packet_age_ms(void)
{
    if (s_last_packet_ms == 0) return 999999;
    return HAL_GetTick() - s_last_packet_ms;
}

void radio_get_raw_commands(int16_t *drive_cmd, int16_t *steer_cmd)
{
    if (drive_cmd) *drive_cmd = s_last_raw_drive;
    if (steer_cmd) *steer_cmd = s_last_raw_steer;
}

uint8_t radio_get_last_status(void)
{
    return s_last_raw_status;
}

bool radio_get_adas_enabled(void)
{
    return s_adas_enabled;
}

bool radio_check_hardware(uint8_t *status, uint8_t *config, uint8_t *channel)
{
    if (!s_hspi || !s_initialized) return false;
    uint8_t st = nrf_read_reg(NRF_REG_STATUS);
    uint8_t cfg = nrf_read_reg(NRF_REG_CONFIG);
    uint8_t ch = nrf_read_reg(NRF_REG_RF_CH);
    if (status) *status = st;
    if (config) *config = cfg;
    if (channel) *channel = ch;
    return (st != 0xFF && ch == RADIO_CHANNEL);
}

void radio_get_commands(float *drive_norm, float *steer_norm, bool *arm_req, bool *disarm_req, bool *estop_req)
{
    if (drive_norm) *drive_norm = s_cmd_drive;
    if (steer_norm) *steer_norm = s_cmd_steer;
    if (arm_req)    { *arm_req = s_cmd_arm; s_cmd_arm = false; }
    if (disarm_req) { *disarm_req = s_cmd_disarm; s_cmd_disarm = false; }
    if (estop_req)  { *estop_req = s_cmd_estop; }
}

void radio_load_ack_telemetry(uint8_t state, uint8_t fault, float pitch_deg, float rate_dps, float left_rpm, float right_rpm, uint16_t dist_cm, float bat_v, uint8_t diag_flags, uint8_t imu_err_cnt)
{
    /* Trong chế độ 1-chiều thuần túy: Không phát ACK Payload ngược lại để tiết kiệm pin tối đa cho xe */
    (void)state; (void)fault; (void)pitch_deg; (void)rate_dps;
    (void)left_rpm; (void)right_rpm; (void)dist_cm; (void)bat_v;
    (void)diag_flags; (void)imu_err_cnt;
}
