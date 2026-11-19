#include "test_suite.h"
#include "test_cdc.h"
#include "test_mpu6050.h"
#include "test_tmc2208.h"
#include "test_nrf24.h"
#include <stdio.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

static SPI_HandleTypeDef s_hspi2;

typedef enum {
    STATE_MENU = 0,
    STATE_STREAM_MPU,
    STATE_MONITOR_NRF
} TestState_t;

static TestState_t s_test_state = STATE_MENU;
static uint32_t    s_last_stream_tick = 0;

static void spi2_init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();

    s_hspi2.Instance = SPI2;
    s_hspi2.Init.Mode = SPI_MODE_MASTER;
    s_hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    s_hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    s_hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_hspi2.Init.NSS = SPI_NSS_SOFT;
    s_hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    s_hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    s_hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&s_hspi2);
}

void print_banner(void)
{
    cdc_print("\r\n\r\n");
    cdc_print("======================================================================\r\n");
    cdc_print("   BỘ CHẨN ĐOÁN PHẦN CỨNG XE CÂN BẰNG STM32F405 (USB CDC)\r\n");
    cdc_print("   Kiểm tra: MPU6050 (I2C1) | TMC2208 (GPIO) | NRF24L01 (SPI2)\r\n");
    cdc_print("======================================================================\r\n");
    cdc_print(" [1] QUÉT TỔNG THỂ PHẦN CỨNG (Scan 3 module trong 1 giây)\r\n");
    cdc_print(" [2] STREAM GÓC NGHIÊNG REAL-TIME (MPU6050 Live Pitch & Gyro)\r\n");
    cdc_print(" [3] TEST ĐỘNG CƠ TRÁI (PA1 STEP, PA2 DIR: Quay 1 vòng tới/lui)\r\n");
    cdc_print(" [4] TEST ĐỘNG CƠ PHẢI (PA3 STEP, PA4 DIR: Quay 1 vòng tới/lui)\r\n");
    cdc_print(" [5] TEST CẢ 2 ĐỘNG CƠ (Cùng quay tới 2 vòng -> cùng quay lui)\r\n");
    cdc_print(" [6] BẬT DRIVER ĐỘNG CƠ (PA0 = LOW -> Động cơ gông cứng giữ vị trí)\r\n");
    cdc_print(" [7] TẮT DRIVER ĐỘNG CƠ (PA0 = HIGH -> Động cơ ngắt lực an toàn)\r\n");
    cdc_print(" [8] BẮT SÓNG NRF24 (Xem trực tiếp gói tin từ ESP32 DevKit TX)\r\n");
    cdc_print(" [9] HIỆU CHUẨN GYRO MPU6050 (Đặt xe đứng yên 3 giây)\r\n");
    cdc_print(" [0] DỪNG TẤT CẢ / QUAY VỀ MENU CHÍNH\r\n");
    cdc_print("======================================================================\r\n");
    cdc_print("Nhập phím [0 - 9] từ bàn phím để chọn: ");
}

static void run_auto_scan(void)
{
    cdc_print("\r\n\r\n--- ĐANG QUÉT TỔNG THỂ PHẦN CỨNG XE CÂN BẰNG ---\r\n");

    /* 1. MPU6050 */
    uint8_t whoami = 0;
    bool mpu_ok = test_mpu_check_whoami(&whoami);
    if (mpu_ok) {
        float ax, ay, az, gx, gy, gz, t;
        test_mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz, &t);
        float pitch = test_mpu_update_pitch(0.01f);
        cdc_printf(" [1] MPU6050 (I2C1 PB6/PB7) : [PASS 🟢] WHO_AM_I=0x%02X | Pitch: %+.2f deg | Temp: %.1f C\r\n", whoami, pitch, t);
    } else {
        cdc_print(" [1] MPU6050 (I2C1 PB6/PB7) : [FAIL 🔴] Không thấy phản hồi I2C! Kiểm tra dây SDA(PB7), SCL(PB6), 3.3V, GND\r\n");
    }

    /* 2. NRF24L01 */
    uint8_t st = 0, cfg = 0, ch = 0, aw = 0;
    bool nrf_ok = test_nrf_check_spi(&st, &cfg, &ch, &aw);
    if (nrf_ok) {
        cdc_printf(" [2] NRF24L01 (SPI2)        : [PASS 🟢] STATUS=0x%02X | Kênh: %d | AW: %d bytes | CFG: 0x%02X\r\n", st, ch, aw + 2, cfg);
        cdc_print("                              -> Kết nối SPI2 (PB13/14/15, CSN=PB9, CE=PB8) HOÀN HẢO!\r\n");
    } else {
        cdc_printf(" [2] NRF24L01 (SPI2)        : [FAIL 🔴] Mã STATUS: 0x%02X (Mất nguồn 3.3V hoặc đứt dây SPI2)\r\n", st);
    }

    /* 3. TMC2208 GPIOs */
    bool en = test_stepper_is_enabled();
    cdc_printf(" [3] TMC2208 Stepper Pins   : [INFO 🟡] PA0(EN)=%s | Bánh Trái: STEP=PA1, DIR=PA2 | Bánh Phải: STEP=PA3, DIR=PA4\r\n",
               en ? "LOW (Đang BẬT)" : "HIGH (Đang TẮT an toàn)");

    cdc_print("----------------------------------------------------------------------\r\n");
    if (mpu_ok && nrf_ok) {
        cdc_print(" -> KẾT LUẬN: TOÀN BỘ PHẦN CỨNG GIAO TIẾP HOẠT ĐỘNG TỐT 100%!\r\n");
    } else {
        cdc_print(" -> KẾT LUẬN: CÓ THIẾT BỊ BỊ LỖI KẾT NỐI, VUI LÒNG KIỂM TRA DÂY CẮM Ở TRÊN!\r\n");
    }
    cdc_print("----------------------------------------------------------------------\r\n");
    cdc_print("Bấm phím bất kỳ để quay lại Menu...");
}

void test_suite_init(void)
{
    cdc_init();
    test_stepper_init(); /* PA0 = HIGH ngay khi boot */

    spi2_init();
    test_nrf_init(&s_hspi2);
    test_mpu_init(&hi2c1);

    s_test_state = STATE_MENU;
    s_last_stream_tick = 0;
}

void test_suite_loop(void)
{
    uint32_t now = HAL_GetTick();

    /* Đọc phím bấm từ USB CDC */
    if (cdc_available()) {
        char ch = cdc_read_char();

        if (s_test_state != STATE_MENU) {
            /* Nếu đang ở chế độ stream/monitor, bấm phím bất kỳ sẽ dừng và về Menu */
            s_test_state = STATE_MENU;
            test_stepper_stop();
            print_banner();
            return;
        }

        switch (ch) {
            case '1':
                run_auto_scan();
                break;

            case '2':
                cdc_print("\r\n\r\n=== STREAM GÓC NGHIÊNG MPU6050 THỜI GIAN THỰC ===\r\n");
                cdc_print("Nghiêng xe qua lại để quan sát góc Pitch thay đổi. [BẤM PHÍM BẤT KỲ ĐỂ DỪNG]\r\n\r\n");
                s_test_state = STATE_STREAM_MPU;
                s_last_stream_tick = now;
                break;

            case '3':
                cdc_print("\r\n[CẢNH BÁO: BÁNH XE PHẢI NHẤC KHỎI MẶT ĐẤT]\r\n");
                cdc_print(" -> Đang quay ĐỘNG CƠ TRÁI (PA1 STEP, PA2 DIR): 1 vòng tiến (3200 xung)... ");
                test_stepper_enable(true);
                test_stepper_spin_motor(MOTOR_LEFT, MOTOR_DIR_FORWARD, 3200, 300);
                HAL_Delay(300);
                cdc_print("1 vòng lùi... ");
                test_stepper_spin_motor(MOTOR_LEFT, MOTOR_DIR_BACKWARD, 3200, 300);
                test_stepper_enable(false);
                cdc_print("HOÀN TẤT! [PASS 🟢]\r\n");
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '4':
                cdc_print("\r\n[CẢNH BÁO: BÁNH XE PHẢI NHẤC KHỎI MẶT ĐẤT]\r\n");
                cdc_print(" -> Đang quay ĐỘNG CƠ PHẢI (PA3 STEP, PA4 DIR): 1 vòng tiến (3200 xung)... ");
                test_stepper_enable(true);
                test_stepper_spin_motor(MOTOR_RIGHT, MOTOR_DIR_FORWARD, 3200, 300);
                HAL_Delay(300);
                cdc_print("1 vòng lùi... ");
                test_stepper_spin_motor(MOTOR_RIGHT, MOTOR_DIR_BACKWARD, 3200, 300);
                test_stepper_enable(false);
                cdc_print("HOÀN TẤT! [PASS 🟢]\r\n");
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '5':
                cdc_print("\r\n[CẢNH BÁO: BÁNH XE PHẢI NHẤC KHỎI MẶT ĐẤT]\r\n");
                cdc_print(" -> Đang quay CẢ 2 ĐỘNG CƠ TIẾN (2 vòng = 6400 xung)... ");
                test_stepper_enable(true);
                test_stepper_spin_both(MOTOR_DIR_FORWARD, MOTOR_DIR_FORWARD, 6400, 250);
                HAL_Delay(400);
                cdc_print("QUAY LÙI (2 vòng)... ");
                test_stepper_spin_both(MOTOR_DIR_BACKWARD, MOTOR_DIR_BACKWARD, 6400, 250);
                test_stepper_enable(false);
                cdc_print("HOÀN TẤT! [PASS 🟢]\r\n");
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '6':
                test_stepper_enable(true);
                cdc_print("\r\n[OK] ĐÃ BẬT DRIVER TMC2208 (PA0 = LOW)!\r\n");
                cdc_print(" -> Cả 2 động cơ đang giữ mô-men xoắn (gông cứng). Dùng tay xoay thử để cảm nhận lực giữ.\r\n");
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '7':
                test_stepper_enable(false);
                cdc_print("\r\n[OK] ĐÃ TẮT DRIVER TMC2208 (PA0 = HIGH)!\r\n");
                cdc_print(" -> Động cơ ngắt điện hoàn toàn, xoay tự do không sinh nhiệt.\r\n");
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '8':
                cdc_print("\r\n\r\n=== GIÁM SÁT GÓI TIN NRF24L01 THỜI GIAN THỰC ===\r\n");
                cdc_print("Lắng nghe gói tin từ ESP32 DevKit TX (Kênh 76 @ 1Mbps, Auto-ACK). [BẤM PHÍM BẤT KỲ ĐỂ DỪNG]\r\n\r\n");
                s_test_state = STATE_MONITOR_NRF;
                break;

            case '9':
                cdc_print("\r\n[MPU6050] Đang hiệu chuẩn Gyro bias (giữ xe đứng yên 3 giây)... ");
                float bx, by, bz;
                if (test_mpu_calibrate(500, &bx, &by, &bz)) {
                    cdc_printf("XONG! Bias: X=%.2f, Y=%.2f, Z=%.2f dps [PASS 🟢]\r\n", bx, by, bz);
                } else {
                    cdc_print("THẤT BẠI (Lỗi I2C hoặc xe bị rung lắc) [FAIL 🔴]\r\n");
                }
                cdc_print("\r\nChọn lệnh tiếp theo [0-9]: ");
                break;

            case '0':
            case ' ':
                test_stepper_stop();
                print_banner();
                break;

            case '?':
                print_banner();
                break;

            default:
                break;
        }
    }

    /* Xử lý chế độ STREAM REAL-TIME (Mỗi 100ms) */
    if (s_test_state == STATE_STREAM_MPU) {
        if ((now - s_last_stream_tick) >= 100) {
            s_last_stream_tick = now;
            float ax, ay, az, gx, gy, gz, t;
            test_mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz, &t);
            float pitch = test_mpu_update_pitch(0.1f);
            bool en = test_stepper_is_enabled();
            uint32_t rx_pkts = test_nrf_get_rx_count();

            cdc_printf(" [LIVE] PITCH: %+.2f deg | GYRO_Y: %+.2f dps | TEMP: %.1f C | DRIVER: %s | NRF_RX: #%lu\r\n",
                       pitch, gy, t, en ? "ON" : "OFF", (unsigned long)rx_pkts);
        }
    }

    /* Xử lý chế độ BẮT SÓNG NRF24 REAL-TIME */
    if (s_test_state == STATE_MONITOR_NRF) {
        RadioPacket_t pkt;
        if (test_nrf_read_packet(&pkt)) {
            const char *cmd_name = (pkt.cmd == CMD_ARM) ? "ARM" : ((pkt.cmd == CMD_DISARM) ? "DISARM" : ((pkt.cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
            cdc_printf(" [NRF24 RX] Gói #%u | Lệnh: %-6s | Drive: %+5d | Steer: %+5d | Flags: 0x%02X | CRC: OK 🟢\r\n",
                       pkt.seq, cmd_name, pkt.drive_cmd, pkt.steer_cmd, pkt.flags);
        }
    }
}
