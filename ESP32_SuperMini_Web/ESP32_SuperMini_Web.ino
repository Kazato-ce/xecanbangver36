/**
 * @file ESP32_SuperMini_Web.ino
 * @brief Firmware nạp bằng Arduino IDE cho ESP32-S3 Super Mini / ESP32 Super Mini:
 *        - Đọc dữ liệu Telemetry từ STM32F405 (PA9 TX, PA10 RX) qua UART
 *        - Phát Web Server trên mạng WiFi AP (192.168.4.1) và WiFi STA mạng nhà
 *        - Dashboard chuyên nghiệp: đọc góc nghiêng Pitch/Rate thời gian thực, đồ thị Canvas
 *        - Hiển thị đầy đủ thông số PID hiện tại đang chạy trên STM32 và tinh chỉnh PID thời gian thực
 *
 * Sơ đồ đấu dây chuẩn:
 *   STM32F405 PA9  (TX)  ----->  ESP32 RXD (Mặc định GPIO 44 trên ESP32-S3, GPIO 16 trên ESP32 thường)
 *   STM32F405 PA10 (RX)  <-----  ESP32 TXD (Mặc định GPIO 43 trên ESP32-S3, GPIO 17 trên ESP32 thường)
 *   STM32F405 GND        <=====> ESP32 GND (BẮT BUỘC NỐI CHUNG MASS GND)
 *
 * LƯU Ý KHI NẠP TRÊN ARDUINO IDE CHO ESP32-S3:
 *   - Tools -> USB CDC On Boot: Enabled (để cổng USB-C nhận Serial Monitor trên máy tính)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "config.h"
#include "web_index.h"

WebServer server(80);
DNSServer dnsServer;

// Cổng UART giao tiếp riêng biệt với STM32F405
HardwareSerial StmSerial(1);

// Debug Serial an toàn (chỉ in khi USB CDC bật, tuyệt đối không in ra chân STM32)
#if ARDUINO_USB_CDC_ON_BOOT
  #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
#else
  #define DBG_PRINTF(...)  do {} while(0)
  #define DBG_PRINTLN(...) do {} while(0)
  #define DBG_PRINT(...)   do {} while(0)
#endif

struct TelemetryData {
  float    pitch = 0.0f;
  float    target = 0.0f;
  float    rate = 0.0f;
  float    rpm_l = 0.0f;
  float    rpm_r = 0.0f;
  int      state = 0;       // 0:BOOT, 1:CALIB, 2:DISARMED, 3:ARMED, 4:FAULT
  int      fault = 0;
  float    kp = 30.0f;
  float    kd = 1.90f;
  float    ki = 0.000001f;
  float    kp_s = 0.35f;
  float    ki_s = 0.55f;
  float    steer = 1.20f;
  float    trim = -0.45f;
  uint16_t dist_f = 999;
  uint16_t dist_l = 999;
  uint16_t dist_r = 999;
  uint16_t tof_mm = 8190;
  float    bat = 14.8f;
  bool     mpu_ok = false;
  uint32_t imu_err = 0;
  bool     nrf_hw = false;
  bool     nrf_link = false;
  uint32_t nrf_pkts = 0;
  uint16_t nrf_seq = 0;
  uint32_t nrf_age = 999999;
  int16_t  nrf_drv = 0;
  int16_t  nrf_str = 0;
  uint8_t  nrf_st = 0;
  bool     supermini_ok = false;
  bool     upright = false;
  uint32_t last_rx_ms = 0;
} telem;

String rxBuffer = "";
static uint32_t s_rx_bytes_total = 0;
static uint8_t  s_current_rx_pin = STM32_RX_PIN;
static uint8_t  s_current_tx_pin = STM32_TX_PIN;
static bool     s_pin_locked = false;
static uint32_t s_last_pin_swap_ms = 0;

// ──────────────────────────────────────────────
// Xử lý gói tin Telemetry nhận từ STM32
// ──────────────────────────────────────────────
void parseTelemetryFrame(const String& line) {
  if (line.indexOf("\"pitch\":") >= 0) {
    int p_pitch  = line.indexOf("\"pitch\":");
    int p_target = line.indexOf("\"target\":");
    int p_rate   = line.indexOf("\"rate\":");
    int p_state  = line.indexOf("\"state\":");
    int p_rpml   = line.indexOf("\"rpm_l\":");
    int p_rpmr   = line.indexOf("\"rpm_r\":");
    int p_kp     = line.indexOf("\"kp\":");
    int p_kd     = line.indexOf("\"kd\":");
    int p_ki     = line.indexOf("\"ki\":");
    int p_kps    = line.indexOf("\"kp_s\":");
    int p_kis    = line.indexOf("\"ki_s\":");
    int p_str    = line.indexOf("\"steer\":");
    int p_trm    = line.indexOf("\"trim\":");
    int p_distf  = line.indexOf("\"dist_f\":");
    int p_tof    = line.indexOf("\"tof_mm\":");
    int p_mpu    = line.indexOf("\"mpu_ok\":");
    int p_imue   = line.indexOf("\"imu_err\":");
    int p_nrfhw  = line.indexOf("\"nrf_hw\":");
    int p_nrflk  = line.indexOf("\"nrf_link\":");
    int p_nrfpk  = line.indexOf("\"nrf_pkts\":");
    int p_nrfsq  = line.indexOf("\"nrf_seq\":");
    int p_nrfag  = line.indexOf("\"nrf_age\":");
    int p_nrfdv  = line.indexOf("\"nrf_drv\":");
    int p_nrfst  = line.indexOf("\"nrf_str\":");
    int p_nrfraw = line.indexOf("\"nrf_st\":");
    int p_spm    = line.indexOf("\"supermini_ok\":");
    int p_upr    = line.indexOf("\"upright\":");

    if (p_pitch >= 0)  telem.pitch  = line.substring(p_pitch + 8).toFloat();
    if (p_target >= 0) telem.target = line.substring(p_target + 9).toFloat();
    if (p_rate >= 0)   telem.rate   = line.substring(p_rate + 7).toFloat();
    if (p_state >= 0)  telem.state  = line.substring(p_state + 8).toInt();
    if (p_rpml >= 0)   telem.rpm_l  = line.substring(p_rpml + 8).toFloat();
    if (p_rpmr >= 0)   telem.rpm_r  = line.substring(p_rpmr + 8).toFloat();
    if (p_kp >= 0)     telem.kp     = line.substring(p_kp + 5).toFloat();
    if (p_kd >= 0)     telem.kd     = line.substring(p_kd + 5).toFloat();
    if (p_ki >= 0)     telem.ki     = line.substring(p_ki + 5).toFloat();
    if (p_kps >= 0)    telem.kp_s   = line.substring(p_kps + 7).toFloat();
    if (p_kis >= 0)    telem.ki_s   = line.substring(p_kis + 7).toFloat();
    if (p_str >= 0)    telem.steer  = line.substring(p_str + 8).toFloat();
    if (p_trm >= 0)    telem.trim   = line.substring(p_trm + 7).toFloat();
    if (p_distf >= 0)  telem.dist_f = line.substring(p_distf + 9).toInt();
    if (p_tof >= 0)    telem.tof_mm = line.substring(p_tof + 9).toInt();
    if (p_mpu >= 0)    telem.mpu_ok = (line.substring(p_mpu + 9, p_mpu + 14).indexOf("true") >= 0);
    if (p_imue >= 0)   telem.imu_err = (uint32_t)line.substring(p_imue + 10).toInt();
    if (p_nrfhw >= 0)  telem.nrf_hw = (line.substring(p_nrfhw + 9, p_nrfhw + 14).indexOf("true") >= 0);
    if (p_nrflk >= 0)  telem.nrf_link = (line.substring(p_nrflk + 11, p_nrflk + 16).indexOf("true") >= 0);
    if (p_nrfpk >= 0)  telem.nrf_pkts = (uint32_t)line.substring(p_nrfpk + 11).toInt();
    if (p_nrfsq >= 0)  telem.nrf_seq  = (uint16_t)line.substring(p_nrfsq + 10).toInt();
    if (p_nrfag >= 0)  telem.nrf_age  = (uint32_t)line.substring(p_nrfag + 10).toInt();
    if (p_nrfdv >= 0)  telem.nrf_drv  = (int16_t)line.substring(p_nrfdv + 10).toInt();
    if (p_nrfst >= 0)  telem.nrf_str  = (int16_t)line.substring(p_nrfst + 10).toInt();
    if (p_nrfraw >= 0) telem.nrf_st   = (uint8_t)line.substring(p_nrfraw + 9).toInt();
    if (p_spm >= 0)    telem.supermini_ok = (line.substring(p_spm + 15, p_spm + 20).indexOf("true") >= 0);
    if (p_upr >= 0)    telem.upright = (line.substring(p_upr + 10, p_upr + 15).indexOf("true") >= 0);

    telem.last_rx_ms = millis();
    return;
  }
}

// ──────────────────────────────────────────────
// Gửi lệnh PID sang STM32
// ──────────────────────────────────────────────
void sendPidToStm32(float kp, float kd, float ki, float kp_s, float ki_s, float steer, float trim) {
  char cmd[160];
  snprintf(cmd, sizeof(cmd), "SET_PID %.2f %.2f %.6f %.2f %.2f %.2f %.2f\r\n", kp, kd, ki, kp_s, ki_s, steer, trim);
  StmSerial.print(cmd);

  snprintf(cmd, sizeof(cmd),
    "{\"type\":\"set_pid\",\"seq\":1,\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.6f,\"kp_s\":%.2f,\"ki_s\":%.2f,\"steer\":%.2f,\"trim\":%.2f}\r\n",
    kp, kd, ki, kp_s, ki_s, steer, trim);
  StmSerial.print(cmd);

  DBG_PRINTF("[TX to STM32]: SET_PID kp=%.2f, kd=%.2f, ki=%.6f, kp_s=%.2f, ki_s=%.2f, steer=%.2f, trim=%.2f\n",
             kp, kd, ki, kp_s, ki_s, steer, trim);
}

void sendCmdToStm32(const String& act) {
  if (act == "arm") {
    StmSerial.print("ARM\r\n");
    StmSerial.print("{\"type\":\"command\",\"action\":\"arm_request\"}\r\n");
    DBG_PRINTLN("[TX to STM32]: ARM");
  } else if (act == "disarm") {
    StmSerial.print("DISARM\r\n");
    StmSerial.print("{\"type\":\"command\",\"action\":\"disarm\"}\r\n");
    DBG_PRINTLN("[TX to STM32]: DISARM");
  } else if (act == "stop") {
    StmSerial.print("STOP\r\n");
    DBG_PRINTLN("[TX to STM32]: STOP");
  }
}

void sendDriveToStm32(float speed, float steer) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "DRIVE %.2f %.2f\r\n", speed, steer);
  StmSerial.print(cmd);
  DBG_PRINTF("[TX to STM32]: DRIVE speed=%.2f steer=%.2f\n", speed, steer);
}

void handleDrive() {
  float speed = server.hasArg("speed") ? server.arg("speed").toFloat() : 0.0f;
  float steer = server.hasArg("steer") ? server.arg("steer").toFloat() : 0.0f;
  sendDriveToStm32(speed, steer);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK");
}

// ──────────────────────────────────────────────
// Các Endpoint HTTP
// ──────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", PAGE_MAIN);
}

void handleGetData() {
  bool stm_link = (telem.last_rx_ms > 0 && (millis() - telem.last_rx_ms < 2000));
  char json[640];
  snprintf(json, sizeof(json),
    "{\"pitch\":%.2f,\"target\":%.2f,\"rate\":%.2f,"
    "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"state\":%d,\"fault\":%d,"
    "\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.6f,\"kp_s\":%.2f,\"ki_s\":%.2f,\"steer\":%.2f,\"trim\":%.2f,"
    "\"dist_f\":%u,\"dist_l\":%u,\"dist_r\":%u,\"tof_mm\":%u,\"bat\":%.1f,"
    "\"stm_link\":%s,\"mpu_ok\":%s,\"imu_err\":%u,\"nrf_hw\":%s,\"nrf_link\":%s,\"upright\":%s,"
    "\"nrf_pkts\":%u,\"nrf_seq\":%u,\"nrf_age\":%u,\"nrf_drv\":%d,\"nrf_str\":%d,\"nrf_st\":%u}",
    telem.pitch, telem.target, telem.rate,
    telem.rpm_l, telem.rpm_r, telem.state, telem.fault,
    telem.kp, telem.kd, telem.ki, telem.kp_s, telem.ki_s, telem.steer, telem.trim,
    telem.dist_f, telem.dist_l, telem.dist_r, telem.tof_mm, telem.bat,
    stm_link ? "true" : "false",
    telem.mpu_ok ? "true" : "false",
    telem.imu_err,
    telem.nrf_hw ? "true" : "false",
    telem.nrf_link ? "true" : "false",
    telem.upright ? "true" : "false",
    telem.nrf_pkts, telem.nrf_seq, telem.nrf_age, telem.nrf_drv, telem.nrf_str, telem.nrf_st
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleSetPid() {
  float kp    = server.hasArg("kp") ? server.arg("kp").toFloat() : telem.kp;
  float kd    = server.hasArg("kd") ? server.arg("kd").toFloat() : telem.kd;
  float ki    = server.hasArg("ki") ? server.arg("ki").toFloat() : telem.ki;
  float kp_s  = server.hasArg("kp_s") ? server.arg("kp_s").toFloat() : telem.kp_s;
  float ki_s  = server.hasArg("ki_s") ? server.arg("ki_s").toFloat() : telem.ki_s;
  float steer = server.hasArg("steer") ? server.arg("steer").toFloat() : telem.steer;
  float trim  = server.hasArg("trim") ? server.arg("trim").toFloat() : (server.hasArg("target") ? server.arg("target").toFloat() : telem.trim);

  sendPidToStm32(kp, kd, ki, kp_s, ki_s, steer, trim);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK");
}

void handleCmd() {
  if (server.hasArg("action")) {
    String act = server.arg("action");
    sendCmdToStm32(act);
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// ──────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────
void setup() {
  #if ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(115200);
  delay(500);
  #endif

  #ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  #endif

  DBG_PRINTLN("\n==============================================");
  DBG_PRINTLN("  ESP32-S3 Super Mini - Robot Web Dashboard & PID");
  DBG_PRINTLN("==============================================");

  // Khởi tạo UART giao tiếp với STM32 (PA9 TX, PA10 RX)
  StmSerial.begin(STM32_BAUDRATE, SERIAL_8N1, s_current_rx_pin, s_current_tx_pin);
  DBG_PRINTF("UART STM32: RX=GPIO%d, TX=GPIO%d @ %d baud\n",
             s_current_rx_pin, s_current_tx_pin, STM32_BAUDRATE);

  // Cấu hình IP Access Point (192.168.4.1) để điện thoại nhận IP tức thì
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (String(WIFI_STA_SSID) != "YOUR_WIFI_NAME" && String(WIFI_STA_SSID).length() > 0) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP); // Thuần AP: Khởi động tức thì, không quét kênh, cực kỳ ổn định
  }

  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  DBG_PRINTF("\n[WIFI AP] Đã phát sóng SSID: '%s' (Mật khẩu: '%s')\n", WIFI_AP_SSID, WIFI_AP_PASS);
  DBG_PRINTF("[WIFI AP] Địa chỉ IP Dashboard: http://192.168.4.1\n");

  // Khởi động DNS Server cho Captive Portal
  dnsServer.start(53, "*", local_ip);

  // Kết nối WiFi nhà (nếu có cấu hình)
  if (String(WIFI_STA_SSID) != "YOUR_WIFI_NAME" && String(WIFI_STA_SSID).length() > 0) {
    DBG_PRINTF("Đang kết nối WiFi nhà '%s'...\n", WIFI_STA_SSID);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    int timeout = 10;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(500);
      DBG_PRINT(".");
      timeout--;
    }
    DBG_PRINTLN();

    if (WiFi.status() == WL_CONNECTED) {
      DBG_PRINT("Đã kết nối WiFi nhà! Địa chỉ IP: http://");
      DBG_PRINTLN(WiFi.localIP());
    } else {
      DBG_PRINTLN("Không kết nối được WiFi nhà. Dùng WiFi AP riêng của xe: http://192.168.4.1");
    }
  }

  // Khởi tạo mDNS: http://robot.local
  if (MDNS.begin(MDNS_NAME)) {
    DBG_PRINTF("mDNS hoạt động: http://%s.local\n", MDNS_NAME);
  }

  // Cấu hình định tuyến HTTP Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleGetData);
  server.on("/api/set_pid", HTTP_POST, handleSetPid);
  server.on("/api/set_pid", HTTP_GET, handleSetPid);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.on("/api/cmd", HTTP_GET, handleCmd);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/drive", HTTP_GET, handleDrive);

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
  DBG_PRINTLN("\n>>> [SẴN SÀNG] Mở trình duyệt gõ: http://192.168.4.1 <<<");
}

// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  #ifdef PIN_LED
  bool stm_alive = (telem.last_rx_ms > 0 && (millis() - telem.last_rx_ms < 500));
  digitalWrite(PIN_LED, stm_alive ? HIGH : LOW);
  #endif

  static uint32_t s_last_diag_ms = 0;

  // Đọc dữ liệu từ STM32 qua StmSerial
  while (StmSerial.available() > 0) {
    char c = (char)StmSerial.read();
    s_rx_bytes_total++;
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        parseTelemetryFrame(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
      if (rxBuffer.length() > 600) {
        rxBuffer = "";
      }
    }
  }

  uint32_t now = millis();

  // Cơ chế AUTO-SWAP PIN thông minh:
  // Nếu sau 3.5 giây không nhận được byte nào từ STM32, tự động hoán đổi chân RX và TX
  if (s_rx_bytes_total > 50) {
    s_pin_locked = true;
  } else if (!s_pin_locked && (now - s_last_pin_swap_ms >= 3500)) {
    s_last_pin_swap_ms = now;
    if (s_rx_bytes_total == 0) {
      uint8_t tmp = s_current_rx_pin;
      s_current_rx_pin = s_current_tx_pin;
      s_current_tx_pin = tmp;
      StmSerial.end();
      delay(20);
      StmSerial.begin(STM32_BAUDRATE, SERIAL_8N1, s_current_rx_pin, s_current_tx_pin);
      DBG_PRINTF("[UART AUTO-SWAP] Thử hoán đổi chân: RX=GPIO%d, TX=GPIO%d\n", s_current_rx_pin, s_current_tx_pin);
    }
  }

  if (now - s_last_diag_ms >= 1500) {
    s_last_diag_ms = now;
    if (telem.last_rx_ms > 0 && (now - telem.last_rx_ms < 2000)) {
      DBG_PRINTF("[STM32 KẾT NỐI TỐT 🟢] Đã nhận %u bytes | Pitch: %.2f° | Rate: %.2f°/s | State: %d\n",
                 s_rx_bytes_total, telem.pitch, telem.rate, telem.state);
    } else {
      DBG_PRINTF("[CHỜ STM32 ⏳] Chưa nhận được gói tin (Tổng nhận: %u bytes). RX=GPIO%d, TX=GPIO%d\n",
                 s_rx_bytes_total, s_current_rx_pin, s_current_tx_pin);
    }
  }
}
