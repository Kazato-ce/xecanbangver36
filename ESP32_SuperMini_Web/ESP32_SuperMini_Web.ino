/**
 * @file ESP32_SuperMini_Web.ino
 * @brief Firmware nạp bằng Arduino IDE cho ESP32-S3 Super Mini / ESP32 Super Mini:
 *        - Đọc dữ liệu Telemetry từ STM32F405 (PA9 TX, PA10 RX) qua UART
 *        - Phát Web Server trên mạng WiFi AP (192.168.4.1) và WiFi STA mạng nhà
 *        - Dashboard hiển thị góc nghiêng thời gian thực, đồ thị Canvas, vật cản, pin
 *        - Tinh chỉnh thông số PID trực tiếp từ trình duyệt Web và gửi sang STM32
 *
 * Sơ đồ đấu dây:
 *   STM32F405 PA9  (TX)  ----->  ESP32 RXD (Mặc định GPIO 44 trên ESP32-S3, GPIO 16 trên ESP32 thường)
 *   STM32F405 PA10 (RX)  <-----  ESP32 TXD (Mặc định GPIO 43 trên ESP32-S3, GPIO 17 trên ESP32 thường)
 *   STM32F405 GND        <=====> ESP32 GND (BẮT BUỘC NỐI CHUNG MASS GND)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "config.h"
#include "web_index.h"

WebServer server(80);
DNSServer dnsServer;

struct TelemetryData {
  float    pitch = 0.0f;
  float    target = 0.0f;
  float    rate = 0.0f;
  float    rpm_l = 0.0f;
  float    rpm_r = 0.0f;
  int      state = 0;       // 0:BOOT, 1:CALIB, 2:DISARMED, 3:ARMED, 4:FAULT
  int      fault = 0;
  float    kp = 18.0f;
  float    kd = 0.85f;
  float    ki = 0.0f;
  uint16_t dist_f = 999;
  uint16_t dist_l = 999;
  uint16_t dist_r = 999;
  uint16_t tof_mm = 8190;
  float    bat = 14.8f;
  bool     mpu_ok = false;
  uint32_t imu_err = 0;
  bool     nrf_ok = false;
  bool     supermini_ok = false;
  bool     upright = false;
  uint32_t last_rx_ms = 0;
} telem;

String rxBuffer = "";

// ──────────────────────────────────────────────
// Xử lý gói tin Telemetry nhận từ STM32
// Hỗ trợ cả định dạng JSON lẫn định dạng $T,...
// ──────────────────────────────────────────────
void parseTelemetryFrame(const String& line) {
  // 1. Nếu là định dạng JSON từ STM32:
  // {"type":"telemetry","seq":...,"state":3,"pitch":1.25,"target":0.0,"rate":0.1,...}
  if (line.indexOf("\"type\":\"telemetry\"") >= 0 || line.indexOf("\"type\": \"telemetry\"") >= 0) {
    int p_pitch  = line.indexOf("\"pitch\":");
    int p_target = line.indexOf("\"target\":");
    int p_rate   = line.indexOf("\"rate\":");
    int p_state  = line.indexOf("\"state\":");
    int p_rpml   = line.indexOf("\"rpm_l\":");
    int p_rpmr   = line.indexOf("\"rpm_r\":");
    int p_kp     = line.indexOf("\"kp\":");
    int p_kd     = line.indexOf("\"kd\":");
    int p_ki     = line.indexOf("\"ki\":");
    int p_distf  = line.indexOf("\"dist_f\":");
    int p_tof    = line.indexOf("\"tof_mm\":");
    int p_mpu    = line.indexOf("\"mpu_ok\":");
    int p_imue   = line.indexOf("\"imu_err\":");
    int p_nrf    = line.indexOf("\"nrf_ok\":");
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
    if (p_distf >= 0)  telem.dist_f = line.substring(p_distf + 9).toInt();
    if (p_tof >= 0)    telem.tof_mm = line.substring(p_tof + 9).toInt();
    if (p_mpu >= 0)    telem.mpu_ok = (line.substring(p_mpu + 9, p_mpu + 14).indexOf("true") >= 0);
    if (p_imue >= 0)   telem.imu_err = (uint32_t)line.substring(p_imue + 10).toInt();
    if (p_nrf >= 0)    telem.nrf_ok = (line.substring(p_nrf + 9, p_nrf + 14).indexOf("true") >= 0);
    if (p_spm >= 0)    telem.supermini_ok = (line.substring(p_spm + 15, p_spm + 20).indexOf("true") >= 0);
    if (p_upr >= 0)    telem.upright = (line.substring(p_upr + 10, p_upr + 15).indexOf("true") >= 0);

    telem.last_rx_ms = millis();
    return;
  }

  // 2. Nếu là định dạng CSV $T,pitch,rate,target,rpm_l,rpm_r,state,kp,kd,ki
  if (line.startsWith("$T,")) {
    int startIdx = 3;
    int fieldIndex = 0;
    String fields[12];

    for (int i = 3; i < line.length(); i++) {
      if (line[i] == ',' || line[i] == '\n' || line[i] == '\r') {
        fields[fieldIndex++] = line.substring(startIdx, i);
        startIdx = i + 1;
        if (fieldIndex >= 11) break;
      }
    }

    if (fieldIndex >= 7) {
      telem.pitch  = fields[0].toFloat();
      telem.rate   = fields[1].toFloat();
      telem.target = fields[2].toFloat();
      telem.rpm_l  = fields[3].toFloat();
      telem.rpm_r  = fields[4].toFloat();
      telem.state  = fields[5].toInt();
      telem.kp     = fields[6].toFloat();
      if (fieldIndex >= 9) {
        telem.kd   = fields[7].toFloat();
        telem.ki   = fields[8].toFloat();
      }
      telem.last_rx_ms = millis();
    }
  }
}

// ──────────────────────────────────────────────
// Gửi lệnh PID sang STM32
// ──────────────────────────────────────────────
void sendPidToStm32(float kp, float kd, float ki, float target) {
  // Gửi cả định dạng Text lẫn JSON để STM32 nhận ngay lập tức
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "SET_PID %.2f %.2f %.2f\r\n", kp, kd, ki);
  Serial1.print(cmd);

  snprintf(cmd, sizeof(cmd), "{\"type\":\"set_pid\",\"seq\":1,\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,\"target\":%.2f}\r\n",
           kp, kd, ki, target);
  Serial1.print(cmd);

  Serial.printf("[TX to STM32]: SET_PID kp=%.2f, kd=%.2f, ki=%.2f, target=%.2f\n", kp, kd, ki, target);
}

void sendCmdToStm32(const String& act) {
  if (act == "arm") {
    Serial1.print("ARM\r\n");
    Serial1.print("{\"type\":\"command\",\"action\":\"arm_request\"}\r\n");
    Serial.println("[TX to STM32]: ARM");
  } else if (act == "disarm") {
    Serial1.print("DISARM\r\n");
    Serial1.print("{\"type\":\"command\",\"action\":\"disarm\"}\r\n");
    Serial.println("[TX to STM32]: DISARM");
  }
}

// ──────────────────────────────────────────────
// Các Endpoint HTTP
// ──────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", PAGE_MAIN);
}

void handleGetData() {
  bool stm_link = (telem.last_rx_ms > 0 && (millis() - telem.last_rx_ms < 2000));
  char json[420];
  snprintf(json, sizeof(json),
    "{\"pitch\":%.2f,\"target\":%.2f,\"rate\":%.2f,"
    "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"state\":%d,\"fault\":%d,"
    "\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,"
    "\"dist_f\":%u,\"dist_l\":%u,\"dist_r\":%u,\"tof_mm\":%u,\"bat\":%.1f,"
    "\"stm_link\":%s,\"mpu_ok\":%s,\"imu_err\":%u,\"nrf_ok\":%s,\"upright\":%s}",
    telem.pitch, telem.target, telem.rate,
    telem.rpm_l, telem.rpm_r, telem.state, telem.fault,
    telem.kp, telem.kd, telem.ki,
    telem.dist_f, telem.dist_l, telem.dist_r, telem.tof_mm, telem.bat,
    stm_link ? "true" : "false",
    telem.mpu_ok ? "true" : "false",
    telem.imu_err,
    telem.nrf_ok ? "true" : "false",
    telem.upright ? "true" : "false"
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleSetPid() {
  if (server.hasArg("kp") && server.hasArg("kd")) {
    float kp   = server.arg("kp").toFloat();
    float kd   = server.arg("kd").toFloat();
    float ki   = server.hasArg("ki") ? server.arg("ki").toFloat() : 0.0f;
    float trim = server.hasArg("target") ? server.arg("target").toFloat() : 0.0f;

    sendPidToStm32(kp, kd, ki, trim);

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: Missing parameters");
  }
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

void handleDrive() {
  float spd = server.hasArg("speed") ? server.arg("speed").toFloat() : 0.0f;
  float str = server.hasArg("steer") ? server.arg("steer").toFloat() : 0.0f;

  char cmd[80];
  snprintf(cmd, sizeof(cmd), "DRIVE %.2f %.2f\r\n", spd, str);
  Serial1.print(cmd);

  snprintf(cmd, sizeof(cmd), "{\"type\":\"command\",\"action\":\"drive\",\"speed\":%.2f,\"steer\":%.2f}\r\n", spd, str);
  Serial1.print(cmd);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "OK");
}

// ──────────────────────────────────────────────
// SETUP
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  #ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
  // Chớp LED xác nhận khởi động
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
  }
  #endif

  Serial.println("\n==============================================");
  Serial.println("  ESP32-S3 Super Mini - Robot Web Dashboard & PID");
  Serial.println("==============================================");

  // Khởi tạo UART1 nối với STM32 (PA9 TX, PA10 RX)
  Serial1.begin(STM32_BAUDRATE, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  Serial.printf("UART1 khoi tao: RX=GPIO%d, TX=GPIO%d @ %d baud\n",
                STM32_RX_PIN, STM32_TX_PIN, STM32_BAUDRATE);

  // Cấu hình rõ IP Access Point (192.168.4.1) để điện thoại nhận IP ngay lập tức
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
  Serial.printf("\n[WIFI AP] Đã phát sóng SSID: '%s' (Mật khẩu: '%s')\n", WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.printf("[WIFI AP] Địa chỉ IP Dashboard: http://192.168.4.1\n");

  // Khởi động DNS Server cho Captive Portal (mọi tên miền/IP đều trỏ về dashboard)
  dnsServer.start(53, "*", local_ip);

  // Kết nối WiFi nhà (nếu có cấu hình)
  if (String(WIFI_STA_SSID) != "YOUR_WIFI_NAME" && String(WIFI_STA_SSID).length() > 0) {
    Serial.printf("Đang kết nối WiFi nhà '%s'...\n", WIFI_STA_SSID);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    int timeout = 10;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(500);
      Serial.print(".");
      timeout--;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Đã kết nối WiFi nhà! Địa chỉ IP: http://");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Không kết nối được WiFi nhà. Dùng WiFi AP riêng của xe: http://192.168.4.1");
    }
  }

  // Khởi tạo mDNS: http://robot.local
  if (MDNS.begin(MDNS_NAME)) {
    Serial.printf("mDNS hoạt động: http://%s.local\n", MDNS_NAME);
  }

  // Cấu hình định tuyến HTTP Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleGetData);
  server.on("/api/set_pid", HTTP_POST, handleSetPid);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/drive", HTTP_GET, handleDrive);

  // Captive Portal (Tự động mở trang Dashboard khi điện thoại kết nối WiFi)
  server.on("/generate_204", handleRoot);
  server.on("/gen_204", handleRoot);
  server.on("/hotspot-detect.html", handleRoot);
  server.on("/canonical.html", handleRoot);
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();
  Serial.println("\n>>> [SẴN SÀNG] Mở trình duyệt điện thoại/máy tính gõ: http://192.168.4.1 <<<");
}

// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  #ifdef PIN_LED
  // Nếu nhận được dữ liệu từ STM32 (PA9) trong 500ms -> Bật đèn LED sáng liên tục
  bool stm_alive = (telem.last_rx_ms > 0 && (millis() - telem.last_rx_ms < 500));
  digitalWrite(PIN_LED, stm_alive ? HIGH : LOW);
  #endif

  // Đọc dữ liệu từ STM32 qua UART1
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        parseTelemetryFrame(rxBuffer);
        rxBuffer = "";
      }
    } else {
      rxBuffer += c;
      if (rxBuffer.length() > 250) {
        rxBuffer = ""; // Tránh tràn bộ nhớ
      }
    }
  }
}
