/**
 * @file ESP32_DevKit_NRF24.ino
 * @brief Firmware USB Serial COM & NRF24L01 Dongle Bridge tích hợp Cần gạt Joystick (Hỗ trợ cả 1 Cần Đôi hoặc 2 Cần Rời):
 *        - Đóng vai trò là Cầu nối không dây (Wireless Dongle) kết nối trực tiếp với Máy tính (App Python)
 *        - ĐỒNG THỜI tích hợp Joystick: Cho phép điều khiển xe độc lập khi KHÔNG CÓ MÁY TÍNH
 *        - CHẾ ĐỘ 2 CẦN RIÊNG BIỆT (RC Mode 2 - Chuẩn tay cầm RC/Gamepad):
 *          + CẦN 1: Điều khiển TỐC ĐỘ (Tiến / Lùi) + Nút SW1: ARM / DISARM
 *          + CẦN 2: Điều khiển HƯỚNG ĐI (Bẻ Lái Trái / Phải) + Nút SW2: Đổi cấp tốc độ (35% / 65% / 100%)
 *        - Tối ưu hóa chu kỳ phát sóng 20ms (50 Hz) và phát tức thời khi có lệnh/sự kiện (< 2ms delay)
 *        - Cơ chế Hardware Auto-ACK đảm bảo độ tin cậy và chống nghẽn đường truyền
 *
 * Sơ đồ đấu nối giữa NRF24L01 và ESP32 DevKit V1:
 *   NRF24L01        ESP32 DevKit V1
 *     VCC    ----->  3.3V (Khuyến nghị thêm tụ 10uF - 47uF giữa VCC và GND)
 *     GND    ----->  GND
 *     CE     ----->  GPIO 4
 *     CSN    ----->  GPIO 5
 *     SCK    ----->  GPIO 18
 *     MOSI   ----->  GPIO 23
 *     MISO   ----->  GPIO 19
 *
 * Sơ đồ đấu nối 2 Cần gạt Joystick (1 Cần Tốc Độ + 1 Cần Hướng Đi):
 *   [CẦN 1 - TỐC ĐỘ TIẾN / LÙI]   ESP32 DevKit V1
 *     VCC                  ----->  3.3V (Chung 3.3V)
 *     GND                  ----->  GND (Chung GND)
 *     VRX / VRY (Trục dọc) ----->  GPIO 34 (ADC1_CH6 - Input Only)
 *     SW1 (Nút ấn Cần 1)   ----->  GPIO 32 (Ấn ngắn: ARM / DISARM | Ấn giữ >1s: ESTOP)
 *
 *   [CẦN 2 - HƯỚNG ĐI BẺ LÁI]    ESP32 DevKit V1
 *     VCC                  ----->  3.3V (Chung 3.3V)
 *     GND                  ----->  GND (Chung GND)
 *     VRX / VRY (Trục ngang)---->  GPIO 35 (ADC1_CH7 - Input Only)
 *     SW2 (Nút ấn Cần 2)   ----->  GPIO 33 (Ấn ngắn: Đổi tốc độ 35%/65%/100% | Ấn giữ: ESTOP)
 */

#include <SPI.h>
#include <RF24.h>
#include "protocol.h"

// ──────────────────────────────────────────────
// Cấu hình chân phần cứng NRF24L01 & Joystick theo từng loại chip ESP32
// ──────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || defined(ESP32S3)
  #define PIN_CE                  9
  #define PIN_CSN                 10
  #define PIN_LED                 48    // LED trên ESP32-S3 Super Mini
  #define PIN_SCK                 12
  #define PIN_MISO                13
  #define PIN_MOSI                11
  #define PIN_JOY_SPEED           1     // Cần 1: Trục Tốc độ Tiến/Lùi (ADC1_CH0)
  #define PIN_JOY_STEER           2     // Cần 2: Trục Hướng đi Bẻ lái (ADC1_CH1)
  #define PIN_JOY_SW1             4     // Nút nhấn Cần 1: ARM / DISARM
  #define PIN_JOY_SW2             5     // Nút nhấn Cần 2: Đổi cấp tốc độ & ESTOP
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || defined(ESP32C3)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 8     // LED trên ESP32-C3 Super Mini
  #define PIN_SCK                 6
  #define PIN_MISO                7
  #define PIN_MOSI                10
  #define PIN_JOY_SPEED           0     // Cần 1: Trục Tốc độ Tiến/Lùi (ADC1_CH0)
  #define PIN_JOY_STEER           1     // Cần 2: Trục Hướng đi Bẻ lái (ADC1_CH1)
  #define PIN_JOY_SW1             2     // Nút nhấn Cần 1: ARM / DISARM
  #define PIN_JOY_SW2             3     // Nút nhấn Cần 2: Đổi cấp tốc độ & ESTOP
#else
  // Cấu hình chuẩn ESP32 DevKit V1 (30 chân / 36 chân WROOM-32)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 2     // Đèn LED tích hợp trên board ESP32 DevKit
  #define PIN_SCK                 18
  #define PIN_MISO                19
  #define PIN_MOSI                23
  #define PIN_JOY_SPEED           34    // Cần 1: Trục Tốc độ Tiến/Lùi (GPIO 34, ADC1_CH6)
  #define PIN_JOY_STEER           35    // Cần 2: Trục Hướng đi Bẻ lái (GPIO 35, ADC1_CH7)
  #define PIN_JOY_SW1             32    // Nút nhấn Cần 1: ARM / DISARM (SW1 nối GPIO 32)
  #define PIN_JOY_SW2             33    // Nút nhấn Cần 2: Đổi cấp tốc độ 35%/65%/100% & ESTOP (SW2 nối GPIO 33)
#endif

// Tương thích ngược với tên chân cũ (1 cần 2 trục)
#define PIN_JOY_VRX             PIN_JOY_SPEED
#define PIN_JOY_VRY             PIN_JOY_STEER
#define PIN_JOY_SW              PIN_JOY_SW1

// Tùy chọn đảo chiều nếu người dùng lắp ngược hướng cần gạt Joystick
#define JOY_INVERT_DRIVE        false   // Đặt true nếu gạt tới xe bị lùi
#define JOY_INVERT_STEER        false   // Đặt true nếu gạt phải xe rẽ trái
#define JOY_DEADBAND            220     // Vùng chết chống trôi lò xo (~ +-220 trên thang 4096)

#define RADIO_CHANNEL           76      // Kênh RF 2476 MHz (khớp với STM32)
#define CONTROL_INTERVAL_MS     20      // 50 Hz (20 ms) - Siêu mượt và tức thời!

// Khởi tạo RF24 với tần số SPI 2 MHz (Tối ưu chống nhiễu trên dây cắm Dupont)
RF24 radio(PIN_CE, PIN_CSN, 2000000);

// Địa chỉ pipe giao tiếp 5 byte (khớp với STM32)
const uint64_t RADIO_ADDR = 0xE7E7E7E7E7ULL;

// Biến lưu trạng thái lệnh gửi đi
static RadioPacket_t txPacket;
static uint16_t s_seq = 0;
static uint32_t s_last_tx_ms = 0;
static bool s_radio_ok = false;
static bool s_force_tx_immediate = false;

// Trạng thái điều khiển hiện tại
static int16_t s_current_drive = 0;     // -1000..1000
static int16_t s_current_steer = 0;     // -1000..1000
static ProtocolCmd_t s_pending_cmd = CMD_DISARM;
static uint8_t s_consecutive_fails = 0;

// Trạng thái Joystick (Hỗ trợ cả 1 Cần Đôi và 2 Cần Rời)
static bool s_joystick_detected = false;
static bool s_joy_speed_detected = false;
static bool s_joy_steer_detected = false;
static int  s_joy_center_x = 2048;       // Tâm Cần 1 (Tốc độ)
static int  s_joy_center_y = 2048;       // Tâm Cần 2 (Hướng đi)
static bool s_joy_active = false;
static uint32_t s_sw1_press_time_ms = 0;
static bool s_sw1_last_state = HIGH;
static uint32_t s_sw2_press_time_ms = 0;
static bool s_sw2_last_state = HIGH;
static uint8_t  s_speed_gear = 2;        // 1: 35% (Chậm), 2: 65% (Vừa), 3: 100% (Tối đa)

static String s_serial_rx = "";

const char* get_direction_8way(int drive, int steer) {
  if (abs(drive) < 100 && abs(steer) < 100) return "TĨNH (STOP)";
  if (drive > 150 && abs(steer) < 150) return "▲ TIẾN THẲNG";
  if (drive < -150 && abs(steer) < 150) return "▼ LÙI THẲNG";
  if (abs(drive) < 150 && steer < -150) return "◄ QUAY TRÁI";
  if (abs(drive) < 150 && steer > 150) return "► QUAY PHẢI";
  if (drive > 100 && steer < -100) return "↖ TIẾN - CHÉO TRÁI";
  if (drive > 100 && steer > 100) return "↗ TIẾN - CHÉO PHẢI";
  if (drive < -100 && steer < -100) return "↙ LÙI - CHÉO TRÁI";
  if (drive < -100 && steer > 100) return "↘ LÙI - CHÉO PHẢI";
  return "DI CHUYỂN";
}

void printMenu(void) {
  Serial.println("\n========================================================");
  Serial.println("  ESP32 DevKit V1 ↔ NRF24L01 Dongle & 2-Joystick Controller");
  Serial.println("========================================================");
  Serial.println("1. Kết nối 2 Cần gạt Joystick (RC Mode 2 - Chuẩn tay cầm):");
  Serial.println("   [CẦN 1 - TỐC ĐỘ]: VRX/VRY -> GPIO 34 (Tiến / Lùi)");
  Serial.println("                     SW1     -> GPIO 32 (Ấn ngắn: ARM/DISARM | Giữ: ESTOP)");
  Serial.println("   [CẦN 2 - HƯỚNG ]: VRX/VRY -> GPIO 35 (Bẻ Lái Trái / Phải)");
  Serial.println("                     SW2     -> GPIO 33 (Ấn ngắn: Đổi tốc độ 35%/65%/100%)");
  Serial.println("2. Lệnh qua USB Serial COM (115200 baud cho App Python):");
  Serial.println("   'ARM' / 'X' : ARM xe (bật cân bằng)");
  Serial.println("   'DISARM'/'Z': DISARM xe (tắt động cơ)");
  Serial.println("   'ESTOP'/'E' : DỪNG KHẨN CẤP");
  Serial.println("   'DRIVE X Y' : Đặt tốc độ X (-1000..1000) và bẻ lái Y (-1000..1000)");
  Serial.println("   'w' / 's'   : Tiến / Lùi");
  Serial.println("   'a' / 'd'   : Rẽ Trái / Rẽ Phải");
  Serial.println("   'q' / 'e'   : Tiến-Trái / Tiến-Phải (Đi chéo)");
  Serial.println("   'z' / 'c'   : Lùi-Trái  / Lùi-Phải  (Đi chéo)");
  Serial.println("   ' ' (Space) : Phanh dừng tốc độ 0");
  Serial.println("   'JOY CAL'   : Hiệu chuẩn lại tâm 2 Cần Joystick");
  Serial.println("========================================================\n");
}

void calibrateJoystick(void) {
  long sum_speed = 0;
  long sum_steer = 0;
  const int SAMPLES = 32;

  for (int i = 0; i < SAMPLES; i++) {
    sum_speed += analogRead(PIN_JOY_SPEED);
    sum_steer += analogRead(PIN_JOY_STEER);
    delay(5);
  }

  int avg_speed = sum_speed / SAMPLES;
  int avg_steer = sum_steer / SAMPLES;

  // Hiệu chuẩn Cần 1 (Tốc độ - GPIO 34)
  if (avg_speed > 1000 && avg_speed < 3000) {
    s_joy_speed_detected = true;
    s_joy_center_x = avg_speed;
  } else {
    s_joy_speed_detected = false;
  }

  // Hiệu chuẩn Cần 2 (Hướng đi - GPIO 35)
  if (avg_steer > 1000 && avg_steer < 3000) {
    s_joy_steer_detected = true;
    s_joy_center_y = avg_steer;
  } else {
    s_joy_steer_detected = false;
  }

  s_joystick_detected = s_joy_speed_detected || s_joy_steer_detected;

  if (s_joy_speed_detected && s_joy_steer_detected) {
    Serial.println("[2 JOYSTICKS 🟢] Nhận diện ĐỦ 2 CẦN RIÊNG BIỆT!");
    Serial.printf("   + Cần 1 (Tốc độ Tiến/Lùi - GPIO 34): Tâm = %d\n", avg_speed);
    Serial.printf("   + Cần 2 (Hướng đi Bẻ Lái  - GPIO 35): Tâm = %d\n", avg_steer);
    Serial.printf("   + Cấp tốc độ khởi động: %d (65%% Vừa)\n", s_speed_gear);
  } else if (s_joy_speed_detected) {
    Serial.printf("[1 JOYSTICK 🟢] Nhận diện Cần Tốc độ (GPIO 34, Tâm = %d)\n", avg_speed);
  } else if (s_joy_steer_detected) {
    Serial.printf("[1 JOYSTICK 🟢] Nhận diện Cần Hướng đi (GPIO 35, Tâm = %d)\n", avg_steer);
  } else {
    Serial.printf("[JOYSTICK ℹ️] Không phát hiện Joystick (Speed=%d, Steer=%d) -> Sử dụng USB Serial COM làm kênh chính.\n", avg_speed, avg_steer);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_JOY_SW1, INPUT_PULLUP);
  pinMode(PIN_JOY_SW2, INPUT_PULLUP);

  // Chớp LED xác nhận khởi động
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(80);
    digitalWrite(PIN_LED, LOW);
    delay(80);
  }
    digitalWrite(PIN_LED, HIGH);
    delay(80);
    digitalWrite(PIN_LED, LOW);
    delay(80);
  }

  printMenu();

  // Hiệu chuẩn Joystick
  calibrateJoystick();

  // Cấu hình chân CE và CSN cho NRF24
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);
  delay(50);

  // Khởi tạo SPI cứng cho ESP32
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  delay(50);

  // Thử khởi động module NRF24L01 (tự động thử lại 5 lần)
  bool ok = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (radio.begin(&SPI) || radio.begin()) {
      ok = true;
      break;
    }
    delay(100);
  }

  if (!ok && radio.isChipConnected()) {
    ok = true;
  }

  if (!ok) {
    Serial.println("\n[LỖI 🔴] radio.begin() thất bại! Kiểm tra chân SPI NRF24:");
    Serial.println("   VCC -> 3.3V (Có tụ 10uF-100uF)");
    Serial.println("   GND -> GND");
    Serial.println("   CE  -> GPIO 4");
    Serial.println("   CSN -> GPIO 5");
    Serial.println("   SCK -> GPIO 18");
    Serial.println("   MISO-> GPIO 19");
    Serial.println("   MOSI-> GPIO 23\n");
    s_radio_ok = false;
  } else {
    s_radio_ok = true;
    radio.setChannel(RADIO_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_LOW);               // PA_LOW ổn định, không sụt áp nguồn 3.3V USB
    radio.setAutoAck(true);
    radio.setRetries(3, 5);                     // Delay 1000us, thử lại 5 lần (~5ms max) -> cực kỳ ổn định & tin cậy!
    radio.setPayloadSize(sizeof(RadioPacket_t)); // 17 bytes cố định

    radio.openWritingPipe(RADIO_ADDR);
    radio.stopListening();                      // Chế độ phát (Transmitter)
    radio.flush_tx();

    digitalWrite(PIN_LED, HIGH);
    delay(500);
    digitalWrite(PIN_LED, LOW);

    Serial.println("[OK 🟢] Sóng NRF24L01 sẵn sàng (Kênh 76 @ 1Mbps, 50Hz, Auto-ACK)!\n");
  }
}

void processCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("ARM") || line.equalsIgnoreCase("X")) {
    s_pending_cmd = CMD_ARM;
    s_force_tx_immediate = true;
    Serial.println("ACK: ARM");
  } else if (line.equalsIgnoreCase("DISARM") || line.equalsIgnoreCase("Z")) {
    s_pending_cmd = CMD_DISARM;
    s_current_drive = 0;
    s_current_steer = 0;
    s_force_tx_immediate = true;
    Serial.println("ACK: DISARM");
  } else if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("E")) {
    s_pending_cmd = CMD_ESTOP;
    s_current_drive = 0;
    s_current_steer = 0;
    s_force_tx_immediate = true;
    Serial.println("ACK: ESTOP");
  } else if (line.startsWith("DRIVE ") || line.startsWith("drive ")) {
    int spd = 0, str = 0;
    int n = sscanf(line.c_str() + 6, "%d %d", &spd, &str);
    if (n >= 1) {
      s_current_drive = constrain(spd, -1000, 1000);
      s_current_steer = (n >= 2) ? constrain(str, -1000, 1000) : 0;
      s_pending_cmd = CMD_DRIVE;
      s_force_tx_immediate = true;
      Serial.printf("ACK: DRIVE %d %d (%s)\n", s_current_drive, s_current_steer, get_direction_8way(s_current_drive, s_current_steer));
    }
  } else if (line == "w" || line == "W") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = constrain(s_current_drive + 200, -1000, 1000);
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_current_drive, s_current_steer, get_direction_8way(s_current_drive, s_current_steer));
  } else if (line == "s" || line == "S") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = constrain(s_current_drive - 200, -1000, 1000);
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_current_drive, s_current_steer, get_direction_8way(s_current_drive, s_current_steer));
  } else if (line == "a" || line == "A") {
    s_pending_cmd = CMD_DRIVE;
    s_current_steer = constrain(s_current_steer - 250, -1000, 1000);
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_current_drive, s_current_steer, get_direction_8way(s_current_drive, s_current_steer));
  } else if (line == "d" || line == "D") {
    s_pending_cmd = CMD_DRIVE;
    s_current_steer = constrain(s_current_steer + 250, -1000, 1000);
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_current_drive, s_current_steer, get_direction_8way(s_current_drive, s_current_steer));
  } else if (line == "q" || line == "Q") {
    // ↖ Tiến - Chéo Trái
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = 300;
    s_current_steer = -250;
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE ↖ TIẾN-TRÁI (%d, %d)\n", s_current_drive, s_current_steer);
  } else if (line == "e" || line == "E_DRIVE") {
    // ↗ Tiến - Chéo Phải
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = 300;
    s_current_steer = 250;
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE ↗ TIẾN-PHẢI (%d, %d)\n", s_current_drive, s_current_steer);
  } else if (line == "z" || line == "Z_DRIVE") {
    // ↙ Lùi - Chéo Trái
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = -300;
    s_current_steer = -250;
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE ↙ LÙI-TRÁI (%d, %d)\n", s_current_drive, s_current_steer);
  } else if (line == "c" || line == "C") {
    // ↘ Lùi - Chéo Phải
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = -300;
    s_current_steer = 250;
    s_force_tx_immediate = true;
    Serial.printf("ACK: DRIVE ↘ LÙI-PHẢI (%d, %d)\n", s_current_drive, s_current_steer);
  } else if (line == " " || line == "r" || line == "R") {
    s_pending_cmd = CMD_DRIVE;
    s_current_drive = 0;
    s_current_steer = 0;
    s_force_tx_immediate = true;
    Serial.println("ACK: STOP");
  } else if (line.equalsIgnoreCase("JOY CAL")) {
    calibrateJoystick();
  } else if (line.equalsIgnoreCase("PING")) {
    Serial.println("PONG: MODE=BRIDGE_JOYSTICK, RADIO_OK=1");
  } else if (line.equalsIgnoreCase("STATUS") || line.equalsIgnoreCase("CHECK")) {
    Serial.println("\n--- [TRẠNG THÁI ESP32 DONGLE & 2 JOYSTICKS] ---");
    Serial.printf(" 1. NRF24L01 Phần cứng : %s (Kênh 76 @ 1Mbps)\n", s_radio_ok ? "🟢 SẴN SÀNG" : "🔴 LỖI SPI");
    Serial.printf(" 2. Sóng xe STM32       : %s\n", (s_consecutive_fails < 4) ? "🟢 ĐÃ KẾT NỐI (Auto-ACK OK)" : "🟡 CHỜ XE BẬT NGUỒN");
    Serial.printf(" 3. Cần 1 (Tốc độ)      : %s (GPIO 34, Tâm = %d)\n", s_joy_speed_detected ? "🟢 ĐÃ KẾT NỐI" : "⚪ CHƯA CẮM", s_joy_center_x);
    Serial.printf(" 4. Cần 2 (Hướng đi)    : %s (GPIO 35, Tâm = %d)\n", s_joy_steer_detected ? "🟢 ĐÃ KẾT NỐI" : "⚪ CHƯA CẮM", s_joy_center_y);
    Serial.printf(" 5. Cấp tốc độ hiện tại : Mức %d (%d%%)\n", s_speed_gear, (s_speed_gear == 1) ? 35 : ((s_speed_gear == 2) ? 65 : 100));
    Serial.printf(" 6. Hướng & Lệnh        : %s | Tốc độ=%d | Bẻ lái=%d\n", get_direction_8way(s_current_drive, s_current_steer), s_current_drive, s_current_steer);
    Serial.println("------------------------------------------------\n");
  } else if (line == "?") {
    printMenu();
  }
}

void handleJoystick(void) {
  if (!s_joystick_detected) return;

  uint32_t now = millis();

  // 1. Xử lý nút nhấn SW1 (Cần 1 - Tốc độ - GPIO 32)
  bool sw1_state = digitalRead(PIN_JOY_SW1);
  if (sw1_state == LOW && s_sw1_last_state == HIGH) {
    s_sw1_press_time_ms = now;
  } else if (sw1_state == HIGH && s_sw1_last_state == LOW) {
    uint32_t press_duration = now - s_sw1_press_time_ms;
    if (press_duration >= 50 && press_duration < 1000) {
      // Ấn ngắn: Đổi trạng thái ARM <-> DISARM
      if (s_pending_cmd == CMD_ARM || s_pending_cmd == CMD_DRIVE) {
        s_pending_cmd = CMD_DISARM;
        s_current_drive = 0;
        s_current_steer = 0;
        Serial.println("[NÚT CẦN 1] -> DISARM XE (Tắt động cơ)");
      } else {
        s_pending_cmd = CMD_ARM;
        Serial.println("[NÚT CẦN 1] -> ARM XE (Bật cân bằng)");
      }
      s_force_tx_immediate = true;
    } else if (press_duration >= 1000) {
      // Ấn giữ > 1 giây: DỪNG KHẨN CẤP
      s_pending_cmd = CMD_ESTOP;
      s_current_drive = 0;
      s_current_steer = 0;
      s_force_tx_immediate = true;
      Serial.println("[NÚT CẦN 1] -> DỪNG KHẨN CẤP (ESTOP)!");
    }
  }
  s_sw1_last_state = sw1_state;

  // 2. Xử lý nút nhấn SW2 (Cần 2 - Hướng đi - GPIO 33)
  bool sw2_state = digitalRead(PIN_JOY_SW2);
  if (sw2_state == LOW && s_sw2_last_state == HIGH) {
    s_sw2_press_time_ms = now;
  } else if (sw2_state == HIGH && s_sw2_last_state == LOW) {
    uint32_t press_duration2 = now - s_sw2_press_time_ms;
    if (press_duration2 >= 50 && press_duration2 < 1000) {
      // Ấn ngắn: Chuyển đổi 3 Cấp tốc độ (35% -> 65% -> 100%)
      s_speed_gear = (s_speed_gear % 3) + 1;
      int pct = (s_speed_gear == 1) ? 35 : ((s_speed_gear == 2) ? 65 : 100);
      Serial.printf("[NÚT CẦN 2] -> ĐỔI CẤP TỐC ĐỘ: Mức %d (%d%%)\n", s_speed_gear, pct);
      for (int k = 0; k < s_speed_gear; k++) {
        digitalWrite(PIN_LED, HIGH);
        delay(60);
        digitalWrite(PIN_LED, LOW);
        delay(60);
      }
    } else if (press_duration2 >= 1000) {
      s_pending_cmd = CMD_ESTOP;
      s_current_drive = 0;
      s_current_steer = 0;
      s_force_tx_immediate = true;
      Serial.println("[NÚT CẦN 2] -> DỪNG KHẨN CẤP (ESTOP)!");
    }
  }
  s_sw2_last_state = sw2_state;

  // 3. Đọc 4 mẫu analog để triệt tiêu nhiễu jitter ADC
  int raw_x = 0, raw_y = 0;
  for (int i = 0; i < 4; i++) {
    raw_x += analogRead(PIN_JOY_SPEED);
    raw_y += analogRead(PIN_JOY_STEER);
  }
  raw_x /= 4;
  raw_y /= 4;

  int diff_x = raw_x - s_joy_center_x; // Cần 1: Trục Tiến / Lùi
  int diff_y = raw_y - s_joy_center_y; // Cần 2: Trục Bẻ Lái

  if (JOY_INVERT_DRIVE) diff_x = -diff_x;
  if (JOY_INVERT_STEER) diff_y = -diff_y;

  int joy_drive = 0;
  int joy_steer = 0;

  if (s_joy_speed_detected && abs(diff_x) > JOY_DEADBAND) {
    if (diff_x > 0) {
      joy_drive = map(diff_x, JOY_DEADBAND, 2048, 0, 1000);
    } else {
      joy_drive = map(diff_x, -JOY_DEADBAND, -2048, 0, -1000);
    }
  }

  if (s_joy_steer_detected && abs(diff_y) > JOY_DEADBAND) {
    if (diff_y > 0) {
      joy_steer = map(diff_y, JOY_DEADBAND, 2048, 0, 1000);
    } else {
      joy_steer = map(diff_y, -JOY_DEADBAND, -2048, 0, -1000);
    }
  }

  // Áp dụng tỉ lệ Cấp tốc độ (Speed Gear) cho trục Tiến/Lùi
  float gear_scale = (s_speed_gear == 1) ? 0.35f : ((s_speed_gear == 2) ? 0.65f : 1.0f);
  joy_drive = (int)(joy_drive * gear_scale);

  joy_drive = constrain(joy_drive, -1000, 1000);
  joy_steer = constrain(joy_steer, -1000, 1000);

  // Giới hạn vector hình tròn (Circle clamp) khi vừa đẩy ga vừa bẻ lái
  float mag = sqrtf((float)joy_drive * joy_drive + (float)joy_steer * joy_steer);
  if (mag > 1000.0f) {
    joy_drive = (int)(joy_drive * 1000.0f / mag);
    joy_steer = (int)(joy_steer * 1000.0f / mag);
  }

  // Nếu người dùng đang gạt cần ra khỏi Deadband
  if (abs(joy_drive) > 0 || abs(joy_steer) > 0) {
    s_joy_active = true;
    s_current_drive = joy_drive;
    s_current_steer = joy_steer;
    if (s_pending_cmd != CMD_ESTOP) {
      s_pending_cmd = CMD_DRIVE;
    }
  } else if (s_joy_active) {
    // Vừa nhả cả 2 cần về vị trí trung tâm (Deadband)
    s_joy_active = false;
    s_current_drive = 0;
    s_current_steer = 0;
    s_force_tx_immediate = true; // Gửi ngay lệnh dừng 0
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
  handleJoystick();

  uint32_t now = millis();

  if (!s_radio_ok) {
    static uint32_t s_last_err_ms = 0;
    if ((now - s_last_err_ms) >= 2000) {
      s_last_err_ms = now;
      if (radio.begin(&SPI) || radio.begin()) {
        s_radio_ok = true;
        radio.setChannel(RADIO_CHANNEL);
        radio.setDataRate(RF24_1MBPS);
        radio.setPALevel(RF24_PA_LOW);
        radio.setAutoAck(true);
        radio.setRetries(3, 5);
        radio.setPayloadSize(sizeof(RadioPacket_t));
        radio.openWritingPipe(RADIO_ADDR);
        radio.stopListening();
        radio.flush_tx();
        Serial.println("\n[TỰ KẾT NỐI 🟢] Đã nhận diện lại module NRF24L01 thành công!\n");
      } else {
        Serial.println("[LỖI NRF24 🔴] ESP32 không nhận diện module NRF! Kiểm tra: 3.3V, GND, CE=4, CSN=5, SCK=18, MISO=19, MOSI=23");
      }
    }
    return;
  }

  // Gửi định kỳ mỗi 20 ms (50 Hz) HOẶC phát tức thời khi có sự kiện thay đổi lệnh/hướng
  if ((now - s_last_tx_ms) >= CONTROL_INTERVAL_MS || s_force_tx_immediate) {
    s_last_tx_ms = now;
    s_force_tx_immediate = false;

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

    // Gửi qua sóng NRF24L01 (Hardware Auto-ACK)
    bool tx_ok = radio.write(&txPacket, sizeof(RadioPacket_t));

    if (tx_ok) {
      s_consecutive_fails = 0;
      digitalWrite(PIN_LED, HIGH); // Auto-ACK thành công: LED sáng liên tục
    } else {
      if (s_consecutive_fails < 255) s_consecutive_fails++;
      if (s_consecutive_fails % 8 == 0) {
        radio.flush_tx(); // Xóa sạch bộ đệm truyền bị nghẽn để các gói tiếp theo không bị delay
      }
      if (s_consecutive_fails >= 4) {
        static uint32_t s_last_blink_ms = 0;
        static bool s_blink_state = false;
        if ((now - s_last_blink_ms) >= 400) {
          s_last_blink_ms = now;
          s_blink_state = !s_blink_state;
          digitalWrite(PIN_LED, s_blink_state ? HIGH : LOW);
        }
      }
    }

    // In thông báo định kỳ mỗi 1 giây
    static uint32_t s_last_log_ms = 0;
    if ((now - s_last_log_ms) >= 1000) {
      s_last_log_ms = now;
      const char *cmd_str = (s_pending_cmd == CMD_ARM) ? "ARM" : ((s_pending_cmd == CMD_DISARM) ? "DISARM" : ((s_pending_cmd == CMD_ESTOP) ? "ESTOP" : "DRIVE"));
      const char *dir_str = get_direction_8way(s_current_drive, s_current_steer);

      if (s_consecutive_fails < 4) {
        Serial.printf("[SÓNG TỐT 🟢] Gói #%u | %s | Hướng: %s | Tốc độ=%d, Rẽ=%d (Auto-ACK OK)\n",
                      s_seq, cmd_str, dir_str, s_current_drive, s_current_steer);
      } else {
        Serial.printf("[ĐANG PHÁT 📡] Đã phát sóng gói #%u | %s | Hướng: %s | Tốc độ=%d | Chờ xe STM32 ACK...\n",
                      s_seq, cmd_str, dir_str, s_current_drive);
      }
    }

    // Nếu vừa gửi lệnh ARM thì tự động duy trì lệnh DRIVE
    if (s_pending_cmd == CMD_ARM) {
      s_pending_cmd = CMD_DRIVE;
    }
  }
}
