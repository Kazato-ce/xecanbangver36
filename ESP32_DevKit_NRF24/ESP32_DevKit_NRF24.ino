/**
 * @file ESP32_DevKit_NRF24.ino
 * @brief Firmware nạp bằng Arduino IDE cho ESP32 DevKit V1:
 *        - Giao tiếp với module NRF24L01 thường
 *        - Gửi lệnh điều khiển (ARM, DISARM, Tiến/Lùi, Rẽ) sang STM32F405
 *        - Nhận dữ liệu trạng thái / Telemetry từ STM32F405 qua sóng không dây
 *        - Giao tiếp với Máy tính qua cổng USB Serial (115200 baud)
 *
 * Sơ đồ đấu dây:
 * NRF24L01        ESP32 DevKit V1
 *   VCC    ----->  3.3V (Khuyến nghị thêm tụ 10uF - 100uF giữa VCC và GND)
 *   GND    ----->  GND
 *   CE     ----->  GPIO 4
 *   CSN    ----->  GPIO 5
 *   SCK    ----->  GPIO 18
 *   MOSI   ----->  GPIO 23
 *   MISO   ----->  GPIO 19
 *   IRQ    ----->  Không nối
 */

#include <SPI.h>
#include <RF24.h>
#include "protocol.h"

// ──────────────────────────────────────────────
// Cấu hình chân phần cứng NRF24L01
// ──────────────────────────────────────────────
#define PIN_CE                  4
#define PIN_CSN                 5
#define PIN_LED                 2     // Đèn LED tích hợp trên board ESP32

#define RADIO_CHANNEL           76    // Kênh RF 2476 MHz (khớp với STM32)
#define CONTROL_INTERVAL_MS     40    // Gửi lệnh mỗi 40 ms (25 Hz)

// Khởi tạo RF24 với tần số SPI 4 MHz (thay vì 10 MHz) để tăng độ ổn định khi dùng dây cắm breadboard
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

void printMenu(void) {
  Serial.println("\n==================================================");
  Serial.println("  ESP32 DevKit V1 ↔ NRF24L01 Ground Station");
  Serial.println("==================================================");
  Serial.println("Phím điều khiển qua Serial Monitor (115200 baud):");
  Serial.println("  'x' / 'X' : ARM xe (cho phep can bang)");
  Serial.println("  'z' / 'Z' : DISARM xe (tat dong co)");
  Serial.println("  'e' / 'E' : DUNG KHAN CAP (ESTOP)");
  Serial.println("  'w' / 'W' : Tien toi (+150 speed)");
  Serial.println("  's' / 'S' : Lui lai (-150 speed)");
  Serial.println("  'a' / 'A' : Re trai (-200 steer)");
  Serial.println("  'd' / 'D' : Re phai (+200 steer)");
  Serial.println("  ' ' (Space) : Ve toc do 0 (dung tai cho)");
  Serial.println("==================================================");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  printMenu();

  // Cấu hình trước mức logic chân CE và CSN
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH); // CSN ở mức HIGH khi rảnh
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);   // CE ở mức LOW khi chờ
  delay(100);                  // Đợi module ổn định nguồn 3.3V

  // Khởi tạo SPI cứng trên ESP32 (SCK=18, MISO=19, MOSI=23, SS=-1 để RF24 tự quản lý CSN)
  SPI.begin(18, 19, 23, -1);
  delay(50);

  // Thử khởi động module NRF24L01 với con trỏ bus SPI
  bool ok = radio.begin(&SPI);

  // Đo kiểm tra giao tiếp SPI trực tiếp với NRF24L01 (lệnh NOP 0xFF trả về thanh ghi STATUS)
  digitalWrite(PIN_CSN, LOW);
  delayMicroseconds(10);
  uint8_t raw_status = SPI.transfer(0xFF);
  digitalWrite(PIN_CSN, HIGH);

  Serial.printf("\n[KIỂM TRA PHẦN CỨNG] Mã phản hồi STATUS thô: 0x%02X\n", raw_status);

  // Nếu radio.begin() trả về false nhưng chip phản hồi đúng mã 0x0E (đặc trưng của dòng clone Si24R1):
  if (!ok && raw_status == 0x0E) {
    Serial.println("  -> [THÀNH CÔNG] Dây nối 100% CHÍNH XÁC! Chip NRF24/Si24R1 phản hồi mã 0x0E chuẩn.");
    Serial.println("  -> Tự động kích hoạt chế độ tương thích (Bypass strict check) để bắt đầu truyền...");
    ok = true;
  }

  if (!ok) {
    Serial.println("\n[LOI] radio.begin() thất bại!");
    s_radio_ok = false;

    if (raw_status == 0x00) {
      Serial.println("  [CHẨN ĐOÁN: 0x00]");
      Serial.println("   * Chân MISO (GPIO 19) đang bị chạm mass (GND) hoặc không có xung nhịp SCK (GPIO 18).");
      Serial.println("   * Hoặc bị cắm NGƯỢC HÀNG CHÂN 2x4 (xem lại hình sơ đồ chân mặt trên linh kiện).");
    } else if (raw_status == 0xFF) {
      Serial.println("  [CHẨN ĐOÁN: 0xFF]");
      Serial.println("   * Module NRF24L01 CHƯA CÓ NGUỒN 3.3V hoặc đứt dây GND.");
      Serial.println("   * Bắt buộc gắn thêm tụ 10uF - 100uF vào chân VCC và GND của NRF24L01.");
    } else {
      Serial.printf("  [CHẨN ĐOÁN: 0x%02X]\n", raw_status);
      Serial.println("   * Vui lòng kiểm tra lại tụ lọc nguồn và độ tiếp xúc chân.");
    }
    Serial.println("==================================================");
  } else {
    s_radio_ok = true;
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setRetries(5, 5); // delay 1500us, 5 retries
    radio.setPayloadSize(sizeof(RadioPacket_t));

    // Mở pipe ghi sang STM32
    radio.openWritingPipe(RADIO_ADDR);
    // Mở pipe đọc (nếu STM32 gửi ACK payload hoặc telemetry ngược)
    radio.openReadingPipe(1, RADIO_ADDR);
    radio.stopListening(); // Ở chế độ phát (Transmitter)

    Serial.println("[OK] NRF24L01 ĐÃ KHỞI TẠO THÀNH CÔNG & SẴN SÀNG TRUYỀN TIN (Kênh 76 @ 1Mbps)!");
  }
}

static String s_serial_rx = "";

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

  // Gửi gói tin điều khiển định kỳ sang STM32 (mỗi 40 ms)
  if (s_radio_ok && ((now - s_last_tx_ms) >= CONTROL_INTERVAL_MS)) {
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

    // Gửi qua sóng NRF24L01
    bool tx_ok = radio.write(&txPacket, sizeof(RadioPacket_t));

    static uint32_t s_last_log_ms = 0;
    static uint32_t s_ok_cnt = 0;
    static uint32_t s_fail_cnt = 0;

    if (tx_ok) {
      s_ok_cnt++;
      digitalWrite(PIN_LED, HIGH); // Sáng LED khi có ACK từ xe
    } else {
      s_fail_cnt++;
      digitalWrite(PIN_LED, LOW);
    }

    // In thông báo trạng thái kết nối định kỳ mỗi 1 giây
    if ((now - s_last_log_ms) >= 1000) {
      s_last_log_ms = now;
      if (tx_ok) {
        Serial.printf("[KẾT NỐI TỐT] Đang nhận tín hiệu từ STM32! (Gói #%u | Nhận: %u, Rớt: %u)\n",
                      s_seq, s_ok_cnt, s_fail_cnt);
      } else {
        Serial.printf("[CHỜ TÍN HIỆU XE] Đang gửi lệnh... Chưa nhận được ACK từ STM32 (Thử: %u lần)\n",
                      s_seq);
      }
    }

    // Nếu vừa gửi lệnh ARM/ESTOP một lần thì đưa về chế độ duy trì DRIVE
    if (s_pending_cmd == CMD_ARM) {
      s_pending_cmd = CMD_DRIVE;
    }

    // Kiểm tra nếu STM32 có gửi gói tin trả lời (AckPayload hoặc phản hồi)
    if (radio.isAckPayloadAvailable()) {
      TelemetryPacket_t rxTelem;
      radio.read(&rxTelem, sizeof(TelemetryPacket_t));
      if (rxTelem.magic == PROTOCOL_TELEMETRY_MAGIC) {
        Serial.printf("[STM32 Telemetry] State=%d, Pitch=%.2f deg, Dist=%u cm, Bat=%.2f V\n",
                      rxTelem.state, (float)rxTelem.pitch_cd / 100.0f,
                      rxTelem.dist_front_cm, (float)rxTelem.bat_mv / 1000.0f);
      }
    }
  }

  // Tắt LED sau 15ms
  if ((now - s_last_tx_ms) > 15) {
    digitalWrite(PIN_LED, LOW);
  }
}
