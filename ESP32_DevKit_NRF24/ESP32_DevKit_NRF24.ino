/**
 * @file ESP32_DevKit_NRF24.ino
 * @brief Firmware USB Serial COM & NRF24L01 Dongle Bridge tích hợp 2 Cần gạt Joystick:
 *        - Đóng vai trò là Cầu nối không dây (Wireless Dongle) kết nối trực tiếp với Máy tính (App Python)
 *        - ĐỒNG THỜI tích hợp 2 Cần gạt Joystick: Cho phép điều khiển xe độc lập khi KHÔNG CÓ MÁY TÍNH
 *        - CẤU HÌNH 2 CẦN:
 *          + [CẦN 1]: ĐIỀU KHIỂN HƯỚNG DI CHUYỂN 8 HƯỚNG (Tiến, Lùi, Rẽ Trái, Rẽ Phải, 4 góc chéo) + Nút SW1: ARM / DISARM
 *          + [CẦN 2]: ĐIỀU CHỈNH TĂNG / GIẢM TỐC ĐỘ (Đẩy lên = Tăng ga 15%..100%, Kéo xuống = Giảm ga) + Nút SW2: Reset 50%
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
 * Sơ đồ đấu nối 2 Cần gạt Joystick vào ESP32 DevKit V1:
 *   [CẦN 1 - ĐIỀU KHIỂN 8 HƯỚNG: TIẾN / LÙI & BẺ LÁI]
 *     VCC                  ----->  3.3V (Chung 3.3V)
 *     GND                  ----->  GND  (Chung GND)
 *     VRX (Trục Tiến/Lùi)  ----->  GPIO 34 (ADC1_CH6 - Input Only)
 *     VRY (Trục Bẻ Lái)    ----->  GPIO 35 (ADC1_CH7 - Input Only)
 *     SW1 (Nút nhấn Cần 1) ----->  GPIO 32 (Ấn ngắn: ARM / DISARM | Ấn giữ >1s: ESTOP)
 *
 *   [CẦN 2 - ĐIỀU CHỈNH TĂNG / GIẢM TỐC ĐỘ GA]
 *     VCC                  ----->  3.3V (Chung 3.3V)
 *     GND                  ----->  GND  (Chung GND)
 *     VRX / VRY (Trục dọc) ----->  GPIO 39 / VN (ADC1_CH3 - Input Only)
 *     SW2 (Nút nhấn Cần 2) ----->  GPIO 33 (Ấn ngắn: Reset tốc độ về 50% | Ấn giữ: ESTOP)
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
  // Cần 1 (8 Hướng di chuyển)
  #define PIN_JOY1_VRX            1     // Cần 1: Trục Tiến / Lùi (ADC1_CH0)
  #define PIN_JOY1_VRY            2     // Cần 1: Trục Bẻ Lái (ADC1_CH1)
  #define PIN_JOY1_SW             4     // Nút nhấn Cần 1: ARM / DISARM
  // Cần 2 (Tăng / Giảm Tốc độ)
  #define PIN_JOY2_VRX            5     // Cần 2: Trục Tăng / Giảm tốc độ
  #define PIN_JOY2_SW             6     // Nút nhấn Cần 2: Reset tốc độ & ESTOP
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || defined(ESP32C3)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 8     // LED trên ESP32-C3 Super Mini
  #define PIN_SCK                 6
  #define PIN_MISO                7
  #define PIN_MOSI                10
  // Cần 1 (8 Hướng di chuyển)
  #define PIN_JOY1_VRX            0     // Cần 1: Trục Tiến / Lùi (ADC1_CH0)
  #define PIN_JOY1_VRY            1     // Cần 1: Trục Bẻ Lái (ADC1_CH1)
  #define PIN_JOY1_SW             2     // Nút nhấn Cần 1: ARM / DISARM
  // Cần 2 (Tăng / Giảm Tốc độ)
  #define PIN_JOY2_VRX            3     // Cần 2: Trục Tăng / Giảm tốc độ
  #define PIN_JOY2_SW             4     // Nút nhấn Cần 2: Reset tốc độ & ESTOP
#else
  // Cấu hình chuẩn ESP32 DevKit V1 (30 chân / 36 chân WROOM-32)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 2     // Đèn LED tích hợp trên board ESP32 DevKit
  #define PIN_SCK                 18
  #define PIN_MISO                19
  #define PIN_MOSI                23
  // [CẦN 1]: ĐIỀU KHIỂN TIẾN / LÙI & BẺ LÁI TRÁI / PHẢI (8 HƯỚNG)
  #define PIN_JOY1_VRX            34    // Trục Tiến / Lùi (VRX1 - GPIO 34, ADC1_CH6)
  #define PIN_JOY1_VRY            35    // Trục Bẻ Lái Trái / Phải (VRY1 - GPIO 35, ADC1_CH7)
  #define PIN_JOY1_SW             32    // Nút nhấn Cần 1: ARM / DISARM (SW1 nối GPIO 32)
  // [CẦN 2]: TĂNG / GIẢM TỐC ĐỘ (SPEED THROTTLE LIMITER)
  #define PIN_JOY2_VRX            39    // Trục Tăng / Giảm tốc độ (VRX2 - GPIO 39 / VN, ADC1_CH3)
  #define PIN_JOY2_SW             33    // Nút nhấn Cần 2: Reset tốc độ về 50% & ESTOP (SW2 nối GPIO 33)
#endif

// Tương thích ngược với tên chân cũ (1 cần 2 trục)
#define PIN_JOY_VRX             PIN_JOY1_VRX
#define PIN_JOY_VRY             PIN_JOY1_VRY
#define PIN_JOY_SW              PIN_JOY1_SW
#define PIN_JOY_SPEED           PIN_JOY1_VRX
#define PIN_JOY_STEER           PIN_JOY1_VRY
#define PIN_JOY_SW1             PIN_JOY1_SW
#define PIN_JOY_SW2             PIN_JOY2_SW

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

// Trạng thái 2 Cần Joystick
static bool s_joystick_detected = false;
static bool s_joy1_detected = false;
static bool s_joy2_detected = false;
static int  s_joy1_center_x = 2048;      // Tâm Cần 1 (Tiến / Lùi)
static int  s_joy1_center_y = 2048;      // Tâm Cần 1 (Bẻ Lái)
static int  s_joy2_center_x = 2048;      // Tâm Cần 2 (Tăng / Giảm tốc độ)
static bool s_joy_active = false;
static uint32_t s_sw1_press_time_ms = 0;
static bool s_sw1_last_state = HIGH;
static uint32_t s_sw2_press_time_ms = 0;
static bool s_sw2_last_state = HIGH;
static int  s_speed_limit_pct = 60;      // Tốc độ tối đa hiện tại (15% .. 100%, mặc định 60%)

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
  Serial.println("  ESP32 DevKit V1 ↔ 2-Joystick Controller (8-Way + Speed)");
  Serial.println("========================================================");
  Serial.println("1. Kết nối 2 Cần gạt Joystick (1 Cần 8 Hướng + 1 Cần Tốc Độ):");
  Serial.println("   [CẦN 1 - ĐIỀU KHIỂN 8 HƯỚNG]:");
  Serial.println("     - VRX -> GPIO 34 (Tiến / Lùi)");
  Serial.println("     - VRY -> GPIO 35 (Bẻ Lái Trái / Phải)");
  Serial.println("     - SW1 -> GPIO 32 (Ấn ngắn: ARM/DISARM | Giữ >1s: ESTOP)");
  Serial.println("   [CẦN 2 - TĂNG / GIẢM TỐC ĐỘ]:");
  Serial.println("     - VRX -> GPIO 39 (Đẩy lên: Tăng tốc | Kéo xuống: Giảm tốc)");
  Serial.println("     - SW2 -> GPIO 33 (Ấn ngắn: Reset tốc độ về 50%)");
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
  long sum1_x = 0;
  long sum1_y = 0;
  long sum2_x = 0;
  const int SAMPLES = 32;

  for (int i = 0; i < SAMPLES; i++) {
    sum1_x += analogRead(PIN_JOY1_VRX);
    sum1_y += analogRead(PIN_JOY1_VRY);
    sum2_x += analogRead(PIN_JOY2_VRX);
    delay(5);
  }

  int avg1_x = sum1_x / SAMPLES;
  int avg1_y = sum1_y / SAMPLES;
  int avg2_x = sum2_x / SAMPLES;

  // Hiệu chuẩn Cần 1 (Tiến/Lùi & Bẻ Lái - GPIO 34 & 35)
  if (avg1_x > 1000 && avg1_x < 3000 && avg1_y > 1000 && avg1_y < 3000) {
    s_joy1_detected = true;
    s_joy1_center_x = avg1_x;
    s_joy1_center_y = avg1_y;
  } else {
    s_joy1_detected = false;
  }

  // Hiệu chuẩn Cần 2 (Tăng / Giảm Tốc Độ - GPIO 39)
  if (avg2_x > 1000 && avg2_x < 3000) {
    s_joy2_detected = true;
    s_joy2_center_x = avg2_x;
  } else {
    s_joy2_detected = false;
  }

  s_joystick_detected = s_joy1_detected;

  if (s_joy1_detected && s_joy2_detected) {
    Serial.println("[2 JOYSTICKS 🟢] Nhận diện ĐỦ 2 CẦN!");
    Serial.printf("   + Cần 1 (8 Hướng Lái): X=%d (Tiến/Lùi), Y=%d (Bẻ Lái) - GPIO 34 & 35\n", avg1_x, avg1_y);
    Serial.printf("   + Cần 2 (Tăng/Giảm Ga): Tâm=%d - GPIO 39\n", avg2_x);
    Serial.printf("   + Tốc độ giới hạn hiện tại: %d%%\n", s_speed_limit_pct);
  } else if (s_joy1_detected) {
    Serial.printf("[1 JOYSTICK 🟢] Nhận diện Cần 1 (8 Hướng: X=%d, Y=%d), Cần 2 chưa cắm (Tốc độ mặc định %d%%)\n", avg1_x, avg1_y, s_speed_limit_pct);
  } else {
    Serial.printf("[JOYSTICK ℹ️] Không phát hiện Joystick -> Sử dụng USB Serial COM làm kênh chính.\n");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_JOY1_SW, INPUT_PULLUP);
  pinMode(PIN_JOY2_SW, INPUT_PULLUP);

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
    Serial.printf(" 3. Cần 1 (8 Hướng Lái) : %s (X_center=%d, Y_center=%d)\n", s_joy1_detected ? "🟢 ĐÃ KẾT NỐI" : "⚪ CHƯA CẮM", s_joy1_center_x, s_joy1_center_y);
    Serial.printf(" 4. Cần 2 (Tăng/Giảm Ga): %s (GPIO 39, Tâm = %d)\n", s_joy2_detected ? "🟢 ĐÃ KẾT NỐI" : "⚪ CHƯA CẮM", s_joy2_center_x);
    Serial.printf(" 5. Tốc độ tối đa hiện tại: %d%%\n", s_speed_limit_pct);
    Serial.printf(" 6. Hướng & Lệnh        : %s | Tốc độ=%d | Bẻ lái=%d\n", get_direction_8way(s_current_drive, s_current_steer), s_current_drive, s_current_steer);
    Serial.println("------------------------------------------------\n");
  } else if (line == "?") {
    printMenu();
  }
}

void handleJoystick(void) {
  if (!s_joystick_detected && !s_joy2_detected) return;

  uint32_t now = millis();

  // 1. Xử lý nút nhấn SW1 (Cần 1 - ARM/DISARM - GPIO 32)
  bool sw1_state = digitalRead(PIN_JOY1_SW);
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

  // 2. Xử lý nút nhấn SW2 (Cần 2 - Reset tốc độ - GPIO 33)
  bool sw2_state = digitalRead(PIN_JOY2_SW);
  if (sw2_state == LOW && s_sw2_last_state == HIGH) {
    s_sw2_press_time_ms = now;
  } else if (sw2_state == HIGH && s_sw2_last_state == LOW) {
    uint32_t press_duration2 = now - s_sw2_press_time_ms;
    if (press_duration2 >= 50 && press_duration2 < 1000) {
      // Ấn ngắn: Đặt lại tốc độ về 50%
      s_speed_limit_pct = 50;
      Serial.println("[NÚT CẦN 2] -> Đã Reset tốc độ về mức chuẩn: 50%!");
      digitalWrite(PIN_LED, HIGH);
      delay(80);
      digitalWrite(PIN_LED, LOW);
    } else if (press_duration2 >= 1000) {
      s_pending_cmd = CMD_ESTOP;
      s_current_drive = 0;
      s_current_steer = 0;
      s_force_tx_immediate = true;
      Serial.println("[NÚT CẦN 2] -> DỪNG KHẨN CẤP (ESTOP)!");
    }
  }
  s_sw2_last_state = sw2_state;

  // 3. Xử lý CẦN 2: TĂNG / GIẢM TỐC ĐỘ (Đẩy lên = Tăng ga, Kéo xuống = Giảm ga)
  if (s_joy2_detected) {
    int raw_spd_stick = 0;
    for (int i = 0; i < 4; i++) {
      raw_spd_stick += analogRead(PIN_JOY2_VRX);
    }
    raw_spd_stick /= 4;

    int diff_spd = raw_spd_stick - s_joy2_center_x;
    static uint32_t s_last_speed_adjust_ms = 0;

    if (abs(diff_spd) > JOY_DEADBAND) {
      // Điều chỉnh tốc độ mượt mà mỗi 60ms
      if ((now - s_last_speed_adjust_ms) >= 60) {
        s_last_speed_adjust_ms = now;
        int step = (abs(diff_spd) > 1200) ? 3 : 1; // Gạt mạnh tăng/giảm nhanh hơn
        if (diff_spd > 0) {
          s_speed_limit_pct = min(100, s_speed_limit_pct + step);
        } else {
          s_speed_limit_pct = max(15, s_speed_limit_pct - step);
        }
        static int s_last_printed_pct = 0;
        if (abs(s_speed_limit_pct - s_last_printed_pct) >= 5) {
          s_last_printed_pct = s_speed_limit_pct;
          Serial.printf("[CẦN 2 🎚️] Tốc độ tối đa hiện tại: %d%%\n", s_speed_limit_pct);
        }
      }
    }
  }

  // 4. Xử lý CẦN 1: ĐIỀU KHIỂN TIẾN/LÙI & BẺ LÁI (8 HƯỚNG)
  if (!s_joy1_detected) return;

  int raw_x = 0, raw_y = 0;
  for (int i = 0; i < 4; i++) {
    raw_x += analogRead(PIN_JOY1_VRX);
    raw_y += analogRead(PIN_JOY1_VRY);
  }
  raw_x /= 4;
  raw_y /= 4;

  int diff_x = raw_x - s_joy1_center_x; // Cần 1: Trục Tiến / Lùi
  int diff_y = raw_y - s_joy1_center_y; // Cần 1: Trục Bẻ Lái Trái / Phải

  if (JOY_INVERT_DRIVE) diff_x = -diff_x;
  if (JOY_INVERT_STEER) diff_y = -diff_y;

  int joy_drive = 0;
  int joy_steer = 0;

  if (abs(diff_x) > JOY_DEADBAND) {
    if (diff_x > 0) {
      joy_drive = map(diff_x, JOY_DEADBAND, 2048, 0, 1000);
    } else {
      joy_drive = map(diff_x, -JOY_DEADBAND, -2048, 0, -1000);
    }
  }

  if (abs(diff_y) > JOY_DEADBAND) {
    if (diff_y > 0) {
      joy_steer = map(diff_y, JOY_DEADBAND, 2048, 0, 1000);
    } else {
      joy_steer = map(diff_y, -JOY_DEADBAND, -2048, 0, -1000);
    }
  }

  // Áp dụng trần tốc độ do CẦN 2 quy định (15% .. 100%)
  float speed_factor = (float)s_speed_limit_pct / 100.0f;
  joy_drive = (int)(joy_drive * speed_factor);
  joy_drive = constrain(joy_drive, -1000, 1000);
  joy_steer = constrain(joy_steer, -1000, 1000);

  // Giới hạn vector hình tròn (Circle clamp) khi đi chéo
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
    // Vừa nhả cần về vị trí trung tâm (Deadband)
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
