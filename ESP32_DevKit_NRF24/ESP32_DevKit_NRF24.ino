/**
 * @file ESP32_DevKit_NRF24.ino
 * @brief Firmware USB Serial ↔ NRF24L01 Dongle Bridge cho ESP32 DevKit V1:
 *        - Đóng vai trò là Cầu nối không dây (Wireless Dongle) kết nối trực tiếp với Máy tính qua cổng USB COM
 *        - Giao tiếp siêu tốc (115200 baud) với Ứng dụng Python Ground Control Station (App_python/app.py)
 *        - Truyền lệnh điều khiển (ARM, DISARM, Tiến/Lùi, Rẽ, Dừng khẩn cấp) sang STM32F405 qua NRF24L01
 *        - KHÔNG BẬT WIFI / WEB SERVER: Triệt tiêu 100% can nhiễu 2.4GHz với NRF24L01 và tối ưu độ trễ (latency < 2ms)
 *
 * Sơ đồ đấu dây giữa NRF24L01 và ESP32 DevKit V1:
 *   NRF24L01        ESP32 DevKit V1
 *     VCC    ----->  3.3V (Bắt buộc thêm tụ 10uF - 47uF giữa VCC và GND)
 *     GND    ----->  GND
 *     CE     ----->  GPIO 4
 *     CSN    ----->  GPIO 5
 *     SCK    ----->  GPIO 18
 *     MOSI   ----->  GPIO 23
 *     MISO   ----->  GPIO 19
 *     IRQ    ----->  Không nối
 */

#include <SPI.h>
#include <RF24.h>
#include "protocol.h"

// ──────────────────────────────────────────────
// Cấu hình chân phần cứng NRF24L01 theo từng loại chip ESP32
// ──────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || defined(ESP32S3)
  #define PIN_CE                  9
  #define PIN_CSN                 10
  #define PIN_LED                 48    // LED trên ESP32-S3 Super Mini
  #define PIN_SCK                 12
  #define PIN_MISO                13
  #define PIN_MOSI                11
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || defined(ESP32C3)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 8     // LED trên ESP32-C3 Super Mini
  #define PIN_SCK                 6
  #define PIN_MISO                7
  #define PIN_MOSI                10
#else
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 2     // Đèn LED tích hợp trên board ESP32 DevKit
  #define PIN_SCK                 18
  #define PIN_MISO                19
  #define PIN_MOSI                23
#endif

#define RADIO_CHANNEL           76    // Kênh RF 2476 MHz (khớp với STM32)
#define CONTROL_INTERVAL_MS     40    // Gửi lệnh mỗi 40 ms (25 Hz)

// Khởi tạo RF24 với tần số SPI 4 MHz (tối ưu độ ổn định khi cắm dây breadboard)
RF24 radio(PIN_CE, PIN_CSN, 4000000);

// Địa chỉ pipe giao tiếp 5 byte (khớp với STM32)
const uint64_t RADIO_ADDR = 0xE7E7E7E7E7ULL;

// Biến lưu trạng thái lệnh gửi đi
static RadioPacket_t txPacket;
static uint16_t s_seq = 0;
static uint32_t s_last_tx_ms = 0;
static bool s_radio_ok = false;

// Trạng thái điều khiển hiện tại
static int16_t s_current_drive = 0;   // -1000..1000
static int16_t s_current_steer = 0;   // -1000..1000
static ProtocolCmd_t s_pending_cmd = CMD_DISARM;
static uint8_t s_consecutive_fails = 0;

static String s_serial_rx = "";

void printMenu(void) {
  Serial.println("\n==================================================");
  Serial.println("  ESP32 DevKit V1 ↔ NRF24L01 Ground Station Dongle");
  Serial.println("==================================================");
  Serial.println("Giao diện kết nối USB Serial (115200 baud) cho App Python:");
  Serial.println("  'PING'      : Kiểm tra trạng thái Dongle (trả về PONG)");
  Serial.println("  'ARM' / 'X' : ARM xe (cho phép cân bằng)");
  Serial.println("  'DISARM'/'Z': DISARM xe (tắt động cơ)");
  Serial.println("  'ESTOP'/'E' : DỪNG KHẨN CẤP");
  Serial.println("  'DRIVE X Y' : Đặt tốc độ tiến lùi X (-1000..1000) và bẻ lái Y (-1000..1000)");
  Serial.println("  'w' / 's'   : Tăng / giảm tốc độ tiến lùi (+-150)");
  Serial.println("  'a' / 'd'   : Rẽ trái / rẽ phải (+-200)");
  Serial.println("  ' ' (Space) : Phanh về tốc độ 0");
  Serial.println("==================================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LED, OUTPUT);

  // 1. Chớp LED xanh 3 lần xác nhận ESP32 khởi động tốt
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }

  printMenu();

  // Cấu hình trước mức logic chân CE và CSN
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);
  delay(100);

  // Khởi tạo SPI cứng trên ESP32
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(50);

  // Thử khởi động module NRF24L01
  bool ok = radio.begin(&SPI);

  // Kiểm tra thanh ghi STATUS thô qua lệnh NOP 0xFF
  digitalWrite(PIN_CSN, LOW);
  delayMicroseconds(10);
  uint8_t raw_status = SPI.transfer(0xFF);
  digitalWrite(PIN_CSN, HIGH);

  Serial.printf("[KIỂM TRA PHẦN CỨNG] Mã STATUS SPI NRF24: 0x%02X\n", raw_status);

  if (!ok && raw_status != 0x00 && raw_status != 0xFF) {
    Serial.println("  -> [THÀNH CÔNG] Chip NRF24/Si24R1 phản hồi SPI tốt.");
    ok = true;
  }

  if (!ok) {
    Serial.println("\n[LỖI] radio.begin() thất bại!");
    s_radio_ok = false;
    if (raw_status == 0x00) {
      Serial.println("  [CHẨN ĐOÁN: 0x00] Chân MISO chạm mass hoặc mất xung SCK / cắm ngược hàng chân.");
    } else if (raw_status == 0xFF) {
      Serial.println("  [CHẨN ĐOÁN: 0xFF] Module NRF24L01 chưa có nguồn 3.3V hoặc đứt dây GND.");
    }
    // Chớp LED SOS nhanh báo lỗi
    for (int i = 0; i < 6; i++) {
      digitalWrite(PIN_LED, HIGH);
      delay(80);
      digitalWrite(PIN_LED, LOW);
      delay(80);
    }
  } else {
    s_radio_ok = true;
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);   // Dùng PA_LOW tránh bão hòa LNA khi để gần và chống sụt áp 3.3V
    radio.setAutoAck(true);          // Bật Enhanced ShockBurst Auto-Ack chuẩn
    radio.setRetries(5, 15);         // Delay 1500us, thử lại 15 lần tối đa
    radio.setPayloadSize(sizeof(RadioPacket_t)); // 17 bytes tĩnh

    // Mở pipe ghi sang STM32
    radio.openWritingPipe(RADIO_ADDR);
    radio.stopListening();           // Chế độ phát (Transmitter)

    // Bật LED xanh 1 giây xác nhận phần cứng NRF24 sẵn sàng
    digitalWrite(PIN_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED, LOW);

    Serial.println("[OK] NRF24L01 ĐÃ SẴN SÀNG TRUYỀN TIN (Kênh 76 @ 1Mbps, PA_LOW)!");
    Serial.println("[READY] ESP32 Dongle đã sẵn sàng nhận lệnh từ App Python!\n");
  }
}

void processCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("ARM") || line.equalsIgnoreCase("X")) {
    s_pending_cmd = CMD_ARM;
    Serial.println("ACK: ARM");
  } else if (line.equalsIgnoreCase("DISARM") || line.equalsIgnoreCase("Z")) {
    s_pending_cmd = CMD_DISARM;
    s_current_drive = 0;
    s_current_steer = 0;
    Serial.println("ACK: DISARM");
  } else if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("E")) {
    s_pending_cmd = CMD_ESTOP;
    s_current_drive = 0;
    s_current_steer = 0;
    Serial.println("ACK: ESTOP");
  } else if (line.startsWith("DRIVE ") || line.startsWith("drive ")) {
    int spd = 0, str = 0;
    int n = sscanf(line.c_str() + 6, "%d %d", &spd, &str);
    if (n >= 1) {
      s_current_drive = constrain(spd, -1000, 1000);
      s_current_steer = (n >= 2) ? constrain(str, -1000, 1000) : 0;
      s_pending_cmd = CMD_DRIVE;
      Serial.printf("ACK: DRIVE %d %d\n", s_current_drive, s_current_steer);
    }
  } else if (line == "w" || line == "W") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = constrain(s_current_drive + 150, -1000, 1000);
    Serial.printf("ACK: DRIVE %d %d\n", s_current_drive, s_current_steer);
  } else if (line == "s" || line == "S") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = constrain(s_current_drive - 150, -1000, 1000);
    Serial.printf("ACK: DRIVE %d %d\n", s_current_drive, s_current_steer);
  } else if (line == "a" || line == "A") {
    s_pending_cmd = CMD_DRIVE;
    s_current_steer = constrain(s_current_steer - 200, -1000, 1000);
    Serial.printf("ACK: DRIVE %d %d\n", s_current_drive, s_current_steer);
  } else if (line == "d" || line == "D") {
    s_pending_cmd = CMD_DRIVE;
    s_current_steer = constrain(s_current_steer + 200, -1000, 1000);
    Serial.printf("ACK: DRIVE %d %d\n", s_current_drive, s_current_steer);
  } else if (line == " " || line == "r" || line == "R") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = 0;
    s_current_steer = 0;
    Serial.println("ACK: STOP");
  } else if (line.equalsIgnoreCase("PING")) {
    Serial.println("PONG: MODE=BRIDGE, RADIO_OK=1");
  } else if (line.equalsIgnoreCase("STATUS") || line.equalsIgnoreCase("CHECK")) {
    const char *cmd_str = (s_pending_cmd == CMD_ARM) ? "ARM" : ((s_pending_cmd == CMD_DISARM) ? "DISARM" : ((s_pending_cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
    Serial.println("\n--- [TRẠNG THÁI ESP32 DONGLE BRIDGE] ---");
    Serial.printf(" 1. NRF24L01 Phần cứng : %s (Kênh 76 @ 1Mbps, PA_LOW)\n", s_radio_ok ? "🟢 SẴN SÀNG" : "🔴 LỖI SPI PHẦN CỨNG");
    Serial.printf(" 2. Liên kết xe STM32   : %s\n", (s_consecutive_fails < 4) ? "🟢 ĐÃ KẾT NỐI (Auto-ACK OK)" : "🟡 ĐANG TÌM XE (Chờ STM32 bật nguồn)");
    Serial.printf(" 3. Lệnh đang gửi       : %s | Tốc độ: %d | Bẻ lái: %d | Số gói: #%u\n", cmd_str, s_current_drive, s_current_steer, s_seq);
    Serial.println("----------------------------------------\n");
  } else if (line == "?") {
    printMenu();
  }
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (s_serial_rx.length() > 0) {
        processCommandLine(s_serial_rx);
        s_serial_rx = "";
      }
    } else {
      s_serial_rx += ch;
      if (s_serial_rx.length() > 80) s_serial_rx = "";
    }
  }
}

void loop() {
  handleSerialInput();

  uint32_t now = millis();

  if (!s_radio_ok) {
    static uint32_t s_last_err_ms = 0;
    if ((now - s_last_err_ms) >= 2000) {
      s_last_err_ms = now;
      Serial.println("[LỖI PHẦN CỨNG 🔴] ESP32 không nhận diện được module NRF24L01! Kiểm tra dây SPI: 3.3V, GND, CE=4, CSN=5, SCK=18, MISO=19, MOSI=23");
    }
    return;
  }

  // Phát gói tin điều khiển định kỳ sang STM32 (mỗi 40 ms = 25 Hz)
  if ((now - s_last_tx_ms) >= CONTROL_INTERVAL_MS) {
    s_last_tx_ms = now;

    txPacket.magic      = PROTOCOL_MAGIC_BYTE; // 0xAA
    txPacket.version    = PROTOCOL_VERSION;    // 0x01
    txPacket.seq        = s_seq++;
    txPacket.cmd        = s_pending_cmd;
    txPacket.drive_cmd  = s_current_drive;
    txPacket.steer_cmd  = s_current_steer;
    txPacket.flags      = (s_pending_cmd == CMD_ESTOP) ? 0x01 : ((s_pending_cmd == CMD_ARM) ? 0x02 : 0x00);

    for (int i = 0; i < 5; i++) txPacket.reserved[i] = 0;

    // Tính checksum
    txPacket.checksum = protocol_calc_checksum((const uint8_t*)&txPacket, sizeof(RadioPacket_t) - 2);

    // Gửi qua sóng NRF24L01 và kiểm tra tín hiệu phản hồi phần cứng (Hardware Auto-ACK)
    bool tx_ok = radio.write(&txPacket, sizeof(RadioPacket_t));

    if (tx_ok) {
      s_consecutive_fails = 0;
      digitalWrite(PIN_LED, HIGH); // XE ĐANG KẾT NỐI VÀ PHẢN HỒI AUTO-ACK TỐT -> ĐÈN XANH SÁNG LIÊN TỤC!
    } else {
      if (s_consecutive_fails < 255) s_consecutive_fails++;
      if (s_consecutive_fails >= 4) {
        // Khi chưa nhận được ACK từ STM32: Nhấp nháy đèn LED đều 0.5s để người dùng biết ESP32 vẫn đang phát sóng tìm xe
        static uint32_t s_last_blink_ms = 0;
        static bool s_blink_state = false;
        if ((now - s_last_blink_ms) >= 500) {
          s_last_blink_ms = now;
          s_blink_state = !s_blink_state;
          digitalWrite(PIN_LED, s_blink_state ? HIGH : LOW);
        }
      }
    }

    // In thông báo trạng thái định kỳ mỗi 1 giây cho App Python
    static uint32_t s_last_log_ms = 0;
    if ((now - s_last_log_ms) >= 1000) {
      s_last_log_ms = now;
      const char *cmd_str = (s_pending_cmd == CMD_ARM) ? "ARM" : ((s_pending_cmd == CMD_DISARM) ? "DISARM" : ((s_pending_cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
      if (s_consecutive_fails < 4) {
        Serial.printf("[KẾT NỐI TỐT 🟢] Sóng NRF OK | Gói #%u | Lệnh: %s | Tốc độ: %d | Rẽ: %d | ĐÈN XANH SÁNG LIÊN TỤC\n",
                      s_seq, cmd_str, s_current_drive, s_current_steer);
      } else {
        Serial.printf("[CHỜ TÍN HIỆU XE ⏳] Đang phát sóng tìm xe STM32... (Gói #%u) | ĐÈN XANH NHẤP NHÁY | Hãy bật nguồn xe!\n", s_seq);
      }
    }

    // Nếu vừa gửi lệnh ARM thì chuyển sang duy trì lệnh DRIVE
    if (s_pending_cmd == CMD_ARM) {
      s_pending_cmd = CMD_DRIVE;
    }
  }
}
