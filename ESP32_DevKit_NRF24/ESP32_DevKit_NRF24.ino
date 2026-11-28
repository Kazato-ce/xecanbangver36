/**
 * @file ESP32_DevKit_NRF24.ino
 * @brief Firmware USB Serial COM & NRF24L01 Dongle Bridge tích hợp Joystick 2 trục
 *        - Phân quyền độc lập giữa GCS (App Python) và Tay cầm Joystick vật lý
 *        - Đồng bộ trạng thái 1 chiều từ ESP32 Dongle lên GCS qua USB Serial (EVT & STATE)
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
  #define PIN_LED                 48
  #define PIN_SCK                 12
  #define PIN_MISO                13
  #define PIN_MOSI                11
  #define PIN_JOY_VRX             1
  #define PIN_JOY_VRY             2
  #define PIN_JOY_SW              3
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV) || defined(ESP32C3)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 8
  #define PIN_SCK                 6
  #define PIN_MISO                7
  #define PIN_MOSI                10
  #define PIN_JOY_VRX             0
  #define PIN_JOY_VRY             1
  #define PIN_JOY_SW              2
#else
  // Cấu hình chuẩn ESP32 DevKit V1 (30 chân / 36 chân WROOM-32)
  #define PIN_CE                  4
  #define PIN_CSN                 5
  #define PIN_LED                 2
  #define PIN_SCK                 18
  #define PIN_MISO                19
  #define PIN_MOSI                23
  #define PIN_JOY_VRX             34    // Trục Tiến / Lùi
  #define PIN_JOY_VRY             35    // Trục Bẻ Lái
  #define PIN_JOY_SW              32    // Nút bấm ấn trục Joystick
#endif

// Tùy chọn cấu hình Joystick
// Thay thế các macro cấu hình Joystick ở đầu file ESP32_DevKit_NRF24.ino:
#define JOY_INVERT_DRIVE         false
#define JOY_INVERT_STEER         false
#define JOY_DEADBAND             230
#define JOY_PRIORITY_ACTIVATE_DEADBAND 360
#define JOY_PRIORITY_RELEASE_DEADBAND  280
#define JOY_MAX_CMD_FWD          380
#define JOY_MAX_CMD_REV          265
#define JOY_MAX_STEER_CMD        320
#define JOY_ACCEL_PER_S          640.0f
#define JOY_DECEL_PER_S          950.0f  // Tốc độ giảm ga khi nhả cần (hãm mượt trong ~0.34s)
#define JOY_STEER_ACCEL_PER_S    660.0f  // Tốc độ tăng lái từ từ
#define JOY_STEER_DECEL_PER_S    1150.0f // Tốc độ trả lái về 0
#define JOY_PRIORITY_HOLD_MS     500U
#define ENABLE_HARDWARE_JOYSTICK true

// Cấu hình RF & Đồng bộ Serial lên GCS
#define RADIO_CHANNEL            76
#define CONTROL_INTERVAL_MS      20     // 50 Hz phát NRF24
#define GCS_COMMAND_TIMEOUT_MS   400U   // Hết hạn lệnh PC sau 400ms
#define STATE_SYNC_INTERVAL_MS   250U   // 4 Hz gửi snapshot STATE lên GCS

enum ControlSource_t {
  SRC_NONE = 0,
  SRC_JOYSTICK,
  SRC_GCS
};

enum SystemMode_t {
  MODE_DISARMED = 0,
  MODE_ARMED,
  MODE_ESTOP
};

RF24 radio(PIN_CE, PIN_CSN, 2000000);
const uint64_t RADIO_ADDR = 0xE7E7E7E7E7ULL;

// Trạng thái gói tin NRF24 & hệ thống
static RadioPacket_t txPacket;
static uint16_t s_seq = 0;
static uint32_t s_last_tx_ms = 0;
static bool s_radio_ok = false;
static bool s_force_tx_immediate = false;
static bool s_adas_enabled = false;
static uint8_t s_consecutive_fails = 0;

static SystemMode_t s_system_mode = MODE_ARMED;
static ProtocolCmd_t s_pending_cmd = CMD_ARM;
static int16_t s_current_drive = 0;
static int16_t s_current_steer = 0;
static ControlSource_t s_active_source = SRC_NONE;

// Bộ đệm riêng cho kênh GCS (App Python)
static int16_t s_gcs_drive = 0;
static int16_t s_gcs_steer = 0;
static bool s_gcs_active = false;
static uint32_t s_last_gcs_command_ms = 0;

// Bộ đệm riêng cho kênh Joystick vật lý
static bool s_joystick_detected = false;
static int s_joy_center_x = 2048;
static int s_joy_center_y = 2048;
static int16_t s_joy_drive = 0;
static int16_t s_joy_steer = 0;
static bool s_joy_active = false;
static bool s_joy_input_active = false;
static uint32_t s_last_joy_active_ms = 0;
static uint32_t s_sw_press_time_ms = 0;
static bool s_sw_last_state = HIGH;

// Biến theo dõi đồng bộ trạng thái lên GCS
static uint32_t s_last_state_sync_ms = 0;
static SystemMode_t s_last_synced_mode = (SystemMode_t)255;
static bool s_last_synced_adas = false;
static ControlSource_t s_last_synced_source = (ControlSource_t)255;
static bool s_last_synced_link = false;

static String s_serial_rx = "";

const char* get_mode_name(SystemMode_t mode) {
  switch (mode) {
    case MODE_ARMED:    return "ARMED";
    case MODE_ESTOP:    return "ESTOP";
    default:            return "DISARMED";
  }
}

const char* get_source_name(ControlSource_t src) {
  switch (src) {
    case SRC_JOYSTICK:  return "JOYSTICK";
    case SRC_GCS:       return "GCS_PC";
    default:            return "IDLE";
  }
}

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

// Gửi đồng bộ trạng thái tức thời (EVT) và định kỳ (STATE) lên GCS
void syncStateToGcs(uint32_t now, bool force_snapshot) {
  bool link_ok = s_radio_ok && (s_consecutive_fails < 4);
  bool changed = false;

  if (s_system_mode != s_last_synced_mode) {
    s_last_synced_mode = s_system_mode;
    Serial.printf("EVT:STATE=%s\n", get_mode_name(s_system_mode));
    changed = true;
  }

  if (s_adas_enabled != s_last_synced_adas) {
    s_last_synced_adas = s_adas_enabled;
    Serial.printf("EVT:ADAS=%d\n", s_adas_enabled ? 1 : 0);
    changed = true;
  }

  if (s_active_source != s_last_synced_source) {
    s_last_synced_source = s_active_source;
    Serial.printf("EVT:SRC=%s\n", get_source_name(s_active_source));
    changed = true;
  }

  if (link_ok != s_last_synced_link) {
    s_last_synced_link = link_ok;
    changed = true;
  }

  if (changed || force_snapshot || (now - s_last_state_sync_ms) >= STATE_SYNC_INTERVAL_MS) {
    s_last_state_sync_ms = now;
    Serial.printf("STATE:MODE=%s,ADAS=%d,SRC=%s,DRIVE=%d,STEER=%d,LINK=%d\n",
                  get_mode_name(s_system_mode),
                  s_adas_enabled ? 1 : 0,
                  get_source_name(s_active_source),
                  s_current_drive,
                  s_current_steer,
                  link_ok ? 1 : 0);
  }
}

void setSystemMode(SystemMode_t new_mode) {
  s_system_mode = new_mode;
  if (new_mode == MODE_ARMED) {
    s_pending_cmd = CMD_ARM;
  } else {
    s_pending_cmd = (new_mode == MODE_ESTOP) ? CMD_ESTOP : CMD_DISARM;
    s_gcs_drive = 0;
    s_gcs_steer = 0;
    s_gcs_active = false;
    s_joy_drive = 0;
    s_joy_steer = 0;
    s_joy_active = false;
    s_current_drive = 0;
    s_current_steer = 0;
    s_active_source = SRC_NONE;
  }
  s_force_tx_immediate = true;
  syncStateToGcs(millis(), true);
}

void printMenu(void) {
  Serial.println("\n========================================================");
  Serial.println("  ESP32 DevKit V1 <-> NRF24L01 Dongle & 8-Way Joystick");
  Serial.println("========================================================");
  Serial.println("1. Cần gạt Joystick:");
  Serial.println("   - VRX -> GPIO 34 | VRY -> GPIO 35 | SW -> GPIO 32");
  Serial.println("   - Ấn ngắn SW: ARM/DISARM | Ấn giữ >1s SW: ESTOP");
  Serial.println("2. Lệnh qua USB Serial COM (115200 baud):");
  Serial.println("   'ARM' / 'X'  : ARM xe | 'DISARM'/'Z': DISARM | 'ESTOP'/'E': ESTOP");
  Serial.println("   'ADAS:1/0'   : Bật/tắt ADAS");
  Serial.println("   'DRIVE X Y'  : Đặt tốc độ X (-1000..1000), lái Y (-1000..1000)");
  Serial.println("   'SYNC'       : Yêu cầu gửi ngay gói STATE đồng bộ giao diện");
  Serial.println("   'JOY CAL'    : Hiệu chuẩn lại tâm Joystick");
  Serial.println("========================================================\n");
}

void calibrateJoystick(void) {
  if (!ENABLE_HARDWARE_JOYSTICK) {
    s_joystick_detected = false;
    return;
  }

  long sum_x = 0;
  long sum_y = 0;
  const int SAMPLES = 32;

  for (int i = 0; i < SAMPLES; i++) {
    sum_x += analogRead(PIN_JOY_VRX);
    sum_y += analogRead(PIN_JOY_VRY);
    delay(5);
  }

  int avg_x = sum_x / SAMPLES;
  int avg_y = sum_y / SAMPLES;

  if (avg_x > 1200 && avg_x < 2800 && avg_y > 1200 && avg_y < 2800) {
    s_joystick_detected = true;
    s_joy_center_x = avg_x;
    s_joy_center_y = avg_y;
    Serial.printf("[JOYSTICK] Đã nhận diện Cần gạt! Tâm chuẩn: X=%d, Y=%d\n", avg_x, avg_y);
  } else {
    s_joystick_detected = false;
    Serial.printf("[JOYSTICK] Không phát hiện Joystick (X=%d, Y=%d) -> Dùng USB Serial COM.\n", avg_x, avg_y);
  }
}

void setup() {
  memset(&txPacket, 0, sizeof(txPacket));
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);

  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(80);
    digitalWrite(PIN_LED, LOW);
    delay(80);
  }

  printMenu();
  calibrateJoystick();

  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);
  delay(50);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  delay(50);

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
    Serial.println("\n[LỖI] radio.begin() thất bại! Kiểm tra dây SPI NRF24L01.\n");
    s_radio_ok = false;
  } else {
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

    digitalWrite(PIN_LED, HIGH);
    delay(300);
    digitalWrite(PIN_LED, LOW);

    Serial.println("[OK] Sóng NRF24L01 sẵn sàng (Kênh 76 @ 1Mbps, 50Hz, Auto-ACK)!\n");
  }

  syncStateToGcs(millis(), true);
}

void setGcsDriveCommand(int16_t drive, int16_t steer, bool active) {
  // Không cho phép lệnh DRIVE tự mở khóa nếu hệ thống đang DISARM hoặc ESTOP
  if (s_system_mode != MODE_ARMED) {
    s_gcs_drive = 0;
    s_gcs_steer = 0;
    s_gcs_active = false;
    return;
  }

  s_gcs_drive = constrain(drive, -1000, 1000);
  s_gcs_steer = constrain(steer, -1000, 1000);
  s_gcs_active = active;
  if (active) {
    s_last_gcs_command_ms = millis();
  }
  if (!s_joy_active) {
    s_force_tx_immediate = true;
  }
}

void processCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("ADAS:1")) {
    s_adas_enabled = true;
    s_force_tx_immediate = true;
    Serial.println("ACK: ADAS ON");
    syncStateToGcs(millis(), true);
  } else if (line.equalsIgnoreCase("ADAS:0")) {
    s_adas_enabled = false;
    s_force_tx_immediate = true;
    Serial.println("ACK: ADAS OFF");
    syncStateToGcs(millis(), true);
  } else if (line.equalsIgnoreCase("ARM") || line.equalsIgnoreCase("X")) {
    Serial.println("ACK: ARM");
    setSystemMode(MODE_ARMED);
  } else if (line.equalsIgnoreCase("DISARM") || line.equalsIgnoreCase("Z")) {
    Serial.println("ACK: DISARM");
    setSystemMode(MODE_DISARMED);
  } else if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("E")) {
    Serial.println("ACK: ESTOP");
    setSystemMode(MODE_ESTOP);
  } else if (line.startsWith("DRIVE ") || line.startsWith("drive ")) {
    int spd = 0, str = 0;
    int n = sscanf(line.c_str() + 6, "%d %d", &spd, &str);
    if (n >= 1) {
      int16_t d = constrain(spd, -1000, 1000);
      int16_t s = (n >= 2) ? constrain(str, -1000, 1000) : 0;
      setGcsDriveCommand(d, s, true);
      Serial.printf("ACK: DRIVE %d %d (%s)\n", d, s, get_direction_8way(d, s));
    }
  } else if (line == "w" || line == "W") {
    setGcsDriveCommand(s_gcs_drive + 200, s_gcs_steer, true);
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_gcs_drive, s_gcs_steer, get_direction_8way(s_gcs_drive, s_gcs_steer));
  } else if (line == "s" || line == "S") {
    setGcsDriveCommand(s_gcs_drive - 200, s_gcs_steer, true);
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_gcs_drive, s_gcs_steer, get_direction_8way(s_gcs_drive, s_gcs_steer));
  } else if (line == "a" || line == "A") {
    setGcsDriveCommand(s_gcs_drive, s_gcs_steer - 250, true);
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_gcs_drive, s_gcs_steer, get_direction_8way(s_gcs_drive, s_gcs_steer));
  } else if (line == "d" || line == "D") {
    setGcsDriveCommand(s_gcs_drive, s_gcs_steer + 250, true);
    Serial.printf("ACK: DRIVE %d %d (%s)\n", s_gcs_drive, s_gcs_steer, get_direction_8way(s_gcs_drive, s_gcs_steer));
  } else if (line == "q" || line == "Q") {
    setGcsDriveCommand(300, -250, true);
    Serial.printf("ACK: DRIVE TIẾN-TRÁI (%d, %d)\n", s_gcs_drive, s_gcs_steer);
  } else if (line == "e" || line == "E_DRIVE") {
    setGcsDriveCommand(300, 250, true);
    Serial.printf("ACK: DRIVE TIẾN-PHẢI (%d, %d)\n", s_gcs_drive, s_gcs_steer);
  } else if (line == "z" || line == "Z_DRIVE") {
    setGcsDriveCommand(-300, -250, true);
    Serial.printf("ACK: DRIVE LÙI-TRÁI (%d, %d)\n", s_gcs_drive, s_gcs_steer);
  } else if (line == "c" || line == "C") {
    setGcsDriveCommand(-300, 250, true);
    Serial.printf("ACK: DRIVE LÙI-PHẢI (%d, %d)\n", s_gcs_drive, s_gcs_steer);
  } else if (line == " " || line == "r" || line == "R") {
    setGcsDriveCommand(0, 0, false);
    Serial.println("ACK: STOP");
  } else if (line.equalsIgnoreCase("SYNC") || line.equalsIgnoreCase("STATE?")) {
    syncStateToGcs(millis(), true);
  } else if (line.equalsIgnoreCase("JOY CAL")) {
    calibrateJoystick();
  } else if (line.equalsIgnoreCase("PING")) {
    Serial.println("PONG: MODE=BRIDGE_JOYSTICK, RADIO_OK=1");
    syncStateToGcs(millis(), true);
  } else if (line.equalsIgnoreCase("STATUS") || line.equalsIgnoreCase("CHECK")) {
    Serial.println("\n--- [TRẠNG THÁI ESP32 DONGLE & JOYSTICK] ---");
    Serial.printf(" 1. Chế độ hệ thống     : %s\n", get_mode_name(s_system_mode));
    Serial.printf(" 2. NRF24L01 Phần cứng : %s (Kênh 76 @ 1Mbps)\n", s_radio_ok ? "SẴN SÀNG" : "LỖI SPI");
    Serial.printf(" 3. Sóng xe STM32       : %s\n", (s_consecutive_fails < 4) ? "ĐÃ KẾT NỐI (Auto-ACK OK)" : "CHỜ XE BẬT NGUỒN");
    Serial.printf(" 4. Nguồn đang cầm lái : %s\n", get_source_name(s_active_source));
    Serial.printf(" 5. Lệnh phát thực tế  : %s | Drive=%d | Steer=%d\n", get_direction_8way(s_current_drive, s_current_steer), s_current_drive, s_current_steer);
    Serial.println("--------------------------------------------\n");
    syncStateToGcs(millis(), true);
  } else if (line == "?") {
    printMenu();
  }
}

// Hàm phụ trợ làm mượt tăng/giảm tốc theo dt
static float rampCommand(float current, float target, float accel_rate, float decel_rate, float dt) {
  float diff = target - current;
  if (fabsf(diff) < 0.5f) return target;

  // Chọn tốc độ tăng ga (chậm, êm) hay giảm ga/đảo chiều (nhanh hơn để phanh gọn)
  bool is_accelerating = (fabsf(target) > fabsf(current)) && ((target * current) >= 0.0f);
  float rate = is_accelerating ? accel_rate : decel_rate;
  float max_step = rate * dt;

  if (diff > max_step) return current + max_step;
  if (diff < -max_step) return current - max_step;
  return target;
}

void handleJoystick(void) {
  if (!ENABLE_HARDWARE_JOYSTICK || !s_joystick_detected) return;

  uint32_t now = millis();
  static uint32_t s_last_joy_step_ms = 0;
  float dt = (s_last_joy_step_ms == 0U) ? 0.02f : ((now - s_last_joy_step_ms) / 1000.0f);
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.1f)   dt = 0.1f;
  s_last_joy_step_ms = now;

  // 1. Xử lý nút nhấn SW của Joystick
  bool sw_state = digitalRead(PIN_JOY_SW);
  if (sw_state == LOW && s_sw_last_state == HIGH) {
    s_sw_press_time_ms = now;
  } else if (sw_state == HIGH && s_sw_last_state == LOW) {
    uint32_t press_duration = now - s_sw_press_time_ms;
    if (press_duration >= 50 && press_duration < 1000) {
      if (s_system_mode == MODE_ARMED) {
        Serial.println("[NÚT JOYSTICK] -> DISARM XE");
        setSystemMode(MODE_DISARMED);
      } else {
        Serial.println("[NÚT JOYSTICK] -> ARM XE");
        setSystemMode(MODE_ARMED);
      }
    } else if (press_duration >= 1000) {
      Serial.println("[NÚT JOYSTICK] -> DỪNG KHẨN CẤP (ESTOP)!");
      setSystemMode(MODE_ESTOP);
    }
  }
  s_sw_last_state = sw_state;

  // 2. Đọc 4 mẫu analog chống nhiễu ADC
  int raw_x = 0, raw_y = 0;
  for (int i = 0; i < 4; i++) {
    raw_x += analogRead(PIN_JOY_VRX);
    raw_y += analogRead(PIN_JOY_VRY);
  }
  raw_x /= 4;
  raw_y /= 4;

  int diff_x = raw_x - s_joy_center_x;
  int diff_y = raw_y - s_joy_center_y;
  if (JOY_INVERT_DRIVE) diff_x = -diff_x;
  if (JOY_INVERT_STEER) diff_y = -diff_y;

  int max_joy_deflection = max(abs(diff_x), abs(diff_y));
  if (!s_joy_input_active &&
      max_joy_deflection > JOY_PRIORITY_ACTIVATE_DEADBAND) {
    s_joy_input_active = true;
  } else if (s_joy_input_active &&
             max_joy_deflection < JOY_PRIORITY_RELEASE_DEADBAND) {
    s_joy_input_active = false;
  }

  // diff_x is post-inversion: positive maps to forward and receives the forward limit.
  float target_drive = 0.0f;
  if (abs(diff_x) > JOY_DEADBAND) {
    float sign = (diff_x > 0) ? 1.0f : -1.0f;
    float norm = (float)(abs(diff_x) - JOY_DEADBAND) / (float)(2048 - JOY_DEADBAND);
    if (norm > 1.0f) norm = 1.0f;
    float curved = norm * (0.65f + 0.35f * norm); // Vùng gần tâm mịn hơn
    float drive_limit = (diff_x > 0) ? (float)JOY_MAX_CMD_FWD
                                    : (float)JOY_MAX_CMD_REV;
    target_drive = sign * curved * drive_limit;
  }

  float target_steer = 0.0f;
  if (abs(diff_y) > JOY_DEADBAND) {
    float sign = (diff_y > 0) ? 1.0f : -1.0f;
    float norm = (float)(abs(diff_y) - JOY_DEADBAND) / (float)(2048 - JOY_DEADBAND);
    if (norm > 1.0f) norm = 1.0f;
    float curved = norm * (0.65f + 0.35f * norm);
    target_steer = sign * curved * (float)JOY_MAX_STEER_CMD;
  }

  // Giới hạn vector theo ellipse để giữ trần riêng cho drive tiến/lùi và steer.
  float drive_limit = (target_drive >= 0.0f) ? (float)JOY_MAX_CMD_FWD
                                              : (float)JOY_MAX_CMD_REV;
  float normalized_drive = target_drive / drive_limit;
  float normalized_steer = target_steer / (float)JOY_MAX_STEER_CMD;
  float magnitude = sqrtf(normalized_drive * normalized_drive +
                          normalized_steer * normalized_steer);
  if (magnitude > 1.0f) {
    target_drive /= magnitude;
    target_steer /= magnitude;
  }

  // 4. Tăng/giảm tốc từ từ (Ramp) giống cảm giác điều khiển qua GCS
  static float s_joy_drive_f = 0.0f;
  static float s_joy_steer_f = 0.0f;

  if (s_system_mode != MODE_ARMED) {
    s_joy_drive_f = 0.0f;
    s_joy_steer_f = 0.0f;
    s_joy_drive = 0;
    s_joy_steer = 0;
    s_joy_active = false;
    s_joy_input_active = false;
    return;
  }

  if (!s_joy_input_active) {
    target_drive = 0.0f;
    target_steer = 0.0f;
  }

  s_joy_drive_f = rampCommand(s_joy_drive_f, target_drive, JOY_ACCEL_PER_S, JOY_DECEL_PER_S, dt);
  s_joy_steer_f = rampCommand(s_joy_steer_f, target_steer, JOY_STEER_ACCEL_PER_S, JOY_STEER_DECEL_PER_S, dt);

  int16_t out_drive = (int16_t)roundf(s_joy_drive_f);
  int16_t out_steer = (int16_t)roundf(s_joy_steer_f);

  // Giữ quyền Joystick cho tới khi lệnh giảm hẳn về 0 để phanh êm
  if (out_drive != 0 || out_steer != 0 || fabsf(target_drive) > 0.0f || fabsf(target_steer) > 0.0f) {
    if (!s_joy_active) {
      s_force_tx_immediate = true;
    }
    s_joy_active = true;
    s_joy_drive = out_drive;
    s_joy_steer = out_steer;
    s_last_joy_active_ms = now;
  } else if (s_joy_active) {
    s_joy_active = false;
    s_joy_drive = 0;
    s_joy_steer = 0;
    s_last_joy_active_ms = now;
    s_force_tx_immediate = true;
  }
}

void arbitrateControl(uint32_t now) {
  if (s_system_mode == MODE_ESTOP) {
    s_pending_cmd = CMD_ESTOP;
    s_current_drive = 0;
    s_current_steer = 0;
    s_active_source = SRC_NONE;
    return;
  }

  if (s_system_mode == MODE_DISARMED) {
    s_pending_cmd = CMD_DISARM;
    s_current_drive = 0;
    s_current_steer = 0;
    s_active_source = SRC_NONE;
    return;
  }

  // Kiểm tra hết hạn gói tin từ GCS
  if (s_gcs_active && (now - s_last_gcs_command_ms) >= GCS_COMMAND_TIMEOUT_MS) {
    s_gcs_active = false;
    s_gcs_drive = 0;
    s_gcs_steer = 0;
    s_force_tx_immediate = true;
  }

  bool joy_hold_window = (s_last_joy_active_ms != 0U) &&
                         ((now - s_last_joy_active_ms) < JOY_PRIORITY_HOLD_MS);

  if (s_joy_active) {
    s_active_source = SRC_JOYSTICK;
    s_current_drive = s_joy_drive;
    s_current_steer = s_joy_steer;
    if (s_pending_cmd != CMD_ARM) s_pending_cmd = CMD_DRIVE;
  } else if (joy_hold_window) {
    s_active_source = SRC_JOYSTICK;
    s_current_drive = 0;
    s_current_steer = 0;
    if (s_pending_cmd != CMD_ARM) s_pending_cmd = CMD_DRIVE;
  } else if (s_gcs_active) {
    s_active_source = SRC_GCS;
    s_current_drive = s_gcs_drive;
    s_current_steer = s_gcs_steer;
    if (s_pending_cmd != CMD_ARM) s_pending_cmd = CMD_DRIVE;
  } else {
    s_active_source = SRC_NONE;
    s_current_drive = 0;
    s_current_steer = 0;
    if (s_pending_cmd != CMD_ARM) s_pending_cmd = CMD_DRIVE;
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
  arbitrateControl(now);
  syncStateToGcs(now, false);

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
        Serial.println("\n[TỰ KẾT NỐI] Đã nhận diện lại module NRF24L01 thành công!\n");
      }
    }
    return;
  }

  if ((now - s_last_tx_ms) >= CONTROL_INTERVAL_MS || s_force_tx_immediate) {
    s_last_tx_ms = now;
    s_force_tx_immediate = false;

    txPacket.magic      = PROTOCOL_MAGIC_BYTE;
    txPacket.version    = PROTOCOL_VERSION;
    txPacket.seq        = s_seq++;
    txPacket.cmd        = s_pending_cmd;
    txPacket.drive_cmd  = s_current_drive;
    txPacket.steer_cmd  = s_current_steer;
    txPacket.flags      = (s_pending_cmd == CMD_ESTOP) ? 0x01 : ((s_pending_cmd == CMD_ARM) ? 0x02 : 0x00);

    txPacket.reserved[0] = PROTOCOL_ADAS_STATE_VALID |
                           (s_adas_enabled ? PROTOCOL_ADAS_ENABLED : 0U);
    for (int i = 1; i < 5; i++) txPacket.reserved[i] = 0;

    txPacket.checksum = protocol_calc_checksum((const uint8_t*)&txPacket, sizeof(RadioPacket_t) - 2);

    bool tx_ok = radio.write(&txPacket, sizeof(RadioPacket_t));

    if (tx_ok) {
      s_consecutive_fails = 0;
      digitalWrite(PIN_LED, HIGH);
    } else {
      if (s_consecutive_fails < 255) s_consecutive_fails++;
      if (s_consecutive_fails % 8 == 0) {
        radio.flush_tx();
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

    if (s_pending_cmd == CMD_ARM) {
      s_pending_cmd = CMD_DRIVE;
    }
  }
}