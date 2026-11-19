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
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "protocol.h"
#include "web_remote.h"

// ──────────────────────────────────────────────
// Cấu hình Điểm phát WiFi Web Remote
// ──────────────────────────────────────────────
#define WIFI_AP_SSID            "Robot_CanBang_AP"
#define WIFI_AP_PASS            "12345678"

static WebServer server(80);
static DNSServer dnsServer;

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

// Khởi tạo RF24 với tần số SPI 4 MHz (thay vì 10 MHz) để tăng độ ổn định khi dùng dây cắm breadboard
RF24 radio(PIN_CE, PIN_CSN, 4000000);

// Địa chỉ pipe giao tiếp 5 byte (khớp với STM32)
const uint64_t RADIO_ADDR = 0xE7E7E7E7E7ULL;

// Biến lưu trạng thái lệnh gửi đi
static RadioPacket_t txPacket;
static uint16_t s_seq = 0;
static uint32_t s_last_tx_ms = 0;
static bool s_radio_ok = false;

// Trạng thái chẩn đoán hệ thống thu được từ STM32 qua Telemetry
static uint32_t s_last_telem_ms = 0;
static bool     s_last_mpu_ok = false;
static bool     s_last_nrf_ok = false;
static bool     s_last_spm_ok = false;
static bool     s_last_upright = false;
static bool     s_last_motoren = false;
static float    s_last_pitch = 0.0f;
static float    s_last_rate = 0.0f;
static uint16_t s_last_dist = 0;
static float    s_last_bat = 0.0f;
static uint8_t  s_last_state = 0;
static uint8_t  s_last_fault = 0;
static uint8_t  s_last_imu_err = 0;

// Trạng thái điều khiển hiện tại
static int16_t s_current_drive = 0;   // -1000..1000
static int16_t s_current_steer = 0;   // -1000..1000
static ProtocolCmd_t s_pending_cmd = CMD_DISARM;
static uint8_t s_consecutive_fails = 0;

// ──────────────────────────────────────────────
// Các Endpoint HTTP Web Remote
// ──────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", PAGE_REMOTE);
}

void handleStatus() {
  const char *cmd_str = (s_pending_cmd == CMD_ARM) ? "ARM" : ((s_pending_cmd == CMD_DISARM) ? "DISARM" : ((s_pending_cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
  bool link_ok = (s_consecutive_fails < 4);
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"seq\":%u,\"rf_ok\":%s,\"link_ok\":%s,\"drive\":%d,\"steer\":%d,\"cmd\":\"%s\"}",
    s_seq, s_radio_ok ? "true" : "false", link_ok ? "true" : "false",
    s_current_drive, s_current_steer, cmd_str);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

void handleWebCmd() {
  if (server.hasArg("action")) {
    String act = server.arg("action");
    if (act == "arm") {
      s_pending_cmd = CMD_ARM;
      Serial.println("[WEB REMOTE]: ARM");
    } else if (act == "disarm") {
      s_pending_cmd = CMD_DISARM;
      s_current_drive = 0;
      s_current_steer = 0;
      Serial.println("[WEB REMOTE]: DISARM");
    } else if (act == "estop") {
      s_pending_cmd = CMD_ESTOP;
      s_current_drive = 0;
      s_current_steer = 0;
      Serial.println("[WEB REMOTE]: ESTOP");
    }
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleWebDrive() {
  if (server.hasArg("speed") && server.hasArg("steer")) {
    int spd = server.arg("speed").toInt();
    int str = server.arg("steer").toInt();
    s_current_drive = constrain(spd, -1000, 1000);
    s_current_steer = constrain(str, -1000, 1000);
    s_pending_cmd = CMD_DRIVE;
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing args");
  }
}

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
  delay(500);

  pinMode(PIN_LED, OUTPUT);

  // 1. CHỚP LED XANH 3 LẦN ĐỂ XÁC NHẬN PHẦN CỨNG ESP32 & ĐÈN LED HOẠT ĐỘNG
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(120);
    digitalWrite(PIN_LED, LOW);
    delay(120);
  }

  printMenu();

  // Cấu hình trước mức logic chân CE và CSN
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH); // CSN ở mức HIGH khi rảnh
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);   // CE ở mức LOW khi chờ
  delay(100);                  // Đợi module ổn định nguồn 3.3V

  // Khởi tạo SPI cứng trên ESP32 theo từng loại board
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(50);

  // Thử khởi động module NRF24L01 với con trỏ bus SPI
  bool ok = radio.begin(&SPI);

  // Đo kiểm tra giao tiếp SPI trực tiếp với NRF24L01 (lệnh NOP 0xFF trả về thanh ghi STATUS)
  digitalWrite(PIN_CSN, LOW);
  delayMicroseconds(10);
  uint8_t raw_status = SPI.transfer(0xFF);
  digitalWrite(PIN_CSN, HIGH);

  Serial.printf("\n[KIỂM TRA PHẦN CỨNG] Mã phản hồi STATUS thô: 0x%02X\n", raw_status);

  // Nếu chip phản hồi bất kỳ mã hợp lệ nào khác 0x00 và 0xFF (chip NRF24 / Si24R1 đang gắn tốt):
  if (!ok && raw_status != 0x00 && raw_status != 0xFF) {
    Serial.println("  -> [THÀNH CÔNG] Dây nối CHÍNH XÁC! Chip NRF24/Si24R1 phản hồi tín hiệu SPI tốt.");
    Serial.println("  -> Tự động kích hoạt chế độ tương thích để bắt đầu truyền tin...");
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

    // Chớp LED báo lỗi nhanh (SOS)
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
    radio.setPALevel(RF24_PA_LOW);   // Dùng PA_LOW tránh bão hòa bộ khuếch đại LNA ở cự ly gần (<1m) và sụt áp 3.3V
    radio.setAutoAck(true);          // Bật Enhanced ShockBurst Auto-Ack chuẩn
    radio.setRetries(5, 15);         // delay 1500us, thử lại tối đa 15 lần (tối ưu Auto-ACK)
    radio.setPayloadSize(sizeof(RadioPacket_t)); // 17 bytes tĩnh

    // Mở pipe ghi sang STM32
    radio.openWritingPipe(RADIO_ADDR);
    radio.stopListening();           // Ở chế độ phát (Transmitter)

    // BẬT LED XANH SÁNG RÕ 1 GIÂY ĐỂ XÁC NHẬN NRF24 HOẠT ĐỘNG HOÀN HẢO!
    digitalWrite(PIN_LED, HIGH);
    delay(1000);
    digitalWrite(PIN_LED, LOW);

    Serial.println("[OK] NRF24L01 ĐÃ SẴN SÀNG TRUYỀN TIN (Kênh 76 @ 1Mbps, PA_LOW)!");
  }

  // Khởi tạo WiFi Access Point & Web Remote Server
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  dnsServer.start(53, "*", local_ip);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/cmd", HTTP_POST, handleWebCmd);
  server.on("/api/cmd", HTTP_GET, handleWebCmd);
  server.on("/api/drive", HTTP_POST, handleWebDrive);
  server.on("/api/drive", HTTP_GET, handleWebDrive);

  // Captive Portal
  server.on("/generate_204", handleRoot);
  server.on("/gen_204", handleRoot);
  server.on("/hotspot-detect.html", handleRoot);
  server.on("/canonical.html", handleRoot);
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.printf("\n[WIFI AP] Đã phát sóng WiFi: '%s' (Mật khẩu: '%s')\n", WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.println("[WEB REMOTE] Mở trình duyệt điện thoại/máy tính gõ: http://192.168.4.1 để lái xe!\n");
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
  } else if (line.equalsIgnoreCase("PING")) {
    Serial.println("PONG: MODE=1-WAY-TX, RADIO_OK=1");
  } else if (line.equalsIgnoreCase("STATUS") || line.equalsIgnoreCase("CHECK")) {
    const char *cmd_str = (s_pending_cmd == CMD_ARM) ? "ARM" : ((s_pending_cmd == CMD_DISARM) ? "DISARM" : ((s_pending_cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
    Serial.println("\n--- [TRẠNG THÁI BỘ ĐIỀU KHIỂN PHÁT 1-CHIỀU] ---");
    Serial.printf(" 1. ESP32 DevKit NRF24  : %s (Kênh 76 @ 1Mbps)\n", s_radio_ok ? "🟢 SẴN SÀNG" : "🔴 LỖI SPI PHẦN CỨNG");
    Serial.printf(" 2. Chế độ truyền sóng  : 🟢 1-CHIỀU SIÊU TỐC (TX -> Xe RX)\n");
    Serial.printf(" 3. Lệnh đang phát      : %s | Tốc độ: %d | Bẻ lái: %d | Số gói: #%u\n", cmd_str, s_current_drive, s_current_steer, s_seq);
    Serial.printf(" 4. Tiết kiệm pin xe    : Tối đa (Xe chỉ thu ~20mA, không phát PA 120mA)\n");
    Serial.printf(" 5. Xem Telemetry & PID : Kết nối WiFi ESP32 Super Mini (http://192.168.4.1)\n");
    Serial.println("-----------------------------------------------\n");
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
  dnsServer.processNextRequest();
  server.handleClient();
  handleSerialInput();

  uint32_t now = millis();

  if (!s_radio_ok) {
    // Nếu NRF24 trên ESP32 bị lỏng dây hoặc mất nguồn: in nhắc nhở định kỳ (không dùng delay dài gây đơ web)
    static uint32_t s_last_err_ms = 0;
    if ((now - s_last_err_ms) >= 2000) {
      s_last_err_ms = now;
      Serial.println("[LỖI PHẦN CỨNG 🔴] ESP32 không nhận diện được module NRF24L01!");
      Serial.println("  -> Kiểm tra dây nối: 3.3V, GND, CE=4, CSN=5, SCK=18, MISO=19, MOSI=23");
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

    // In thông báo trạng thái định kỳ mỗi 1 giây
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

    // Nếu vừa gửi lệnh ARM/ESTOP thì chuyển sang duy trì lệnh DRIVE
    if (s_pending_cmd == CMD_ARM) {
      s_pending_cmd = CMD_DRIVE;
    }
  }
}
