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
#include <ESPmDNS.h>
#include "config.h"
#include "web_index.h"

WebServer server(80);

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
  char json[320];
  snprintf(json, sizeof(json),
    "{\"pitch\":%.2f,\"target\":%.2f,\"rate\":%.2f,"
    "\"rpm_l\":%.1f,\"rpm_r\":%.1f,\"state\":%d,\"fault\":%d,"
    "\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,"
    "\"dist_f\":%u,\"dist_l\":%u,\"dist_r\":%u,\"tof_mm\":%u,\"bat\":%.1f}",
    telem.pitch, telem.target, telem.rate,
    telem.rpm_l, telem.rpm_r, telem.state, telem.fault,
    telem.kp, telem.kd, telem.ki,
    telem.dist_f, telem.dist_l, telem.dist_r, telem.tof_mm, telem.bat
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
  Serial.println("\n==============================================");
  Serial.println("  ESP32 Super Mini - Robot Web Dashboard & PID");
  Serial.println("==============================================");

  // Khởi tạo UART1 nối với STM32 (PA9 TX, PA10 RX)
  Serial1.begin(STM32_BAUDRATE, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  Serial.printf("UART1 khoi tao: RX=GPIO%d, TX=GPIO%d @ %d baud\n",
                STM32_RX_PIN, STM32_TX_PIN, STM32_BAUDRATE);

  // Bật chế độ vừa phát WiFi AP vừa kết nối WiFi nhà STA
  WiFi.mode(WIFI_AP_STA);

  // Khởi động Access Point riêng
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.printf("WiFi AP bat: SSID '%s' (Pass: '%s')\n", WIFI_AP_SSID, WIFI_AP_PASS);
  Serial.print("Dia chi IP AP: http://");
  Serial.println(WiFi.softAPIP());

  // Kết nối WiFi nhà (nếu có cấu hình)
  if (String(WIFI_STA_SSID) != "YOUR_WIFI_NAME") {
    Serial.printf("Dang ket noi WiFi nha '%s'...\n", WIFI_STA_SSID);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    int timeout = 16;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(500);
      Serial.print(".");
      timeout--;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Da ket noi WiFi nha! Dia chi IP: http://");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Khong ket noi duoc WiFi nha. Dung WiFi AP rieng cua xe!");
    }
  }

  // Khởi tạo mDNS: http://robot.local
  if (MDNS.begin(MDNS_NAME)) {
    Serial.printf("mDNS hoat dong: http://%s.local\n", MDNS_NAME);
  }

  // Cấu hình định tuyến HTTP Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleGetData);
  server.on("/api/set_pid", HTTP_POST, handleSetPid);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/drive", HTTP_GET, handleDrive);

  server.begin();
  Serial.println("Web Server da san sang! Mo trinh duyet truy cap IP hoac http://robot.local");
}

// ──────────────────────────────────────────────
// LOOP
// ──────────────────────────────────────────────
void loop() {
  server.handleClient();

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
