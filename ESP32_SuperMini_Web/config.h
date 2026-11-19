#ifndef CONFIG_H
#define CONFIG_H

// ──────────────────────────────────────────────
// Cấu hình chân UART nối với STM32F405 (PA9 TX, PA10 RX)
// ──────────────────────────────────────────────
// Sơ đồ nối:
//   STM32 PA9  (TX)  ----->  ESP32 RXD (Mặc định GPIO 44 trên ESP32-S3, hoặc GPIO 16 trên ESP32 thường)
//   STM32 PA10 (RX)  <-----  ESP32 TXD (Mặc định GPIO 43 trên ESP32-S3, hoặc GPIO 17 trên ESP32 thường)
//   STM32 GND        <=====> ESP32 GND (BẮT BUỘC NỐI CHUNG MASS GND)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define STM32_RX_PIN       44    // ESP32-S3 Super Mini RX (nhận từ PA9)
  #define STM32_TX_PIN       43    // ESP32-S3 Super Mini TX (gửi sang PA10)
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
  #define STM32_RX_PIN       20    // ESP32-C3 Super Mini RX (chân RX / GPIO 20)
  #define STM32_TX_PIN       21    // ESP32-C3 Super Mini TX (chân TX / GPIO 21)
#else
  #define STM32_RX_PIN       16    // ESP32 DevKit / Super Mini thường RX
  #define STM32_TX_PIN       17    // ESP32 DevKit / Super Mini thường TX
#endif

#define STM32_BAUDRATE     115200

// ──────────────────────────────────────────────
// Cấu hình Điểm phát WiFi riêng (WiFi Access Point)
// Mặc định truy cập: http://192.168.4.1 hoặc http://robot.local
// ──────────────────────────────────────────────
#define WIFI_AP_SSID       "Robot_CanBang_AP"
#define WIFI_AP_PASS       "12345678"

// ──────────────────────────────────────────────
// Cấu hình WiFi Mạng Nhà (WiFi STA mode - Tùy chọn)
// Điền tên WiFi nhà bạn nếu muốn xe tự kết nối vào mạng nhà
// ──────────────────────────────────────────────
#define WIFI_STA_SSID      "YOUR_WIFI_NAME"
#define WIFI_STA_PASS      "YOUR_WIFI_PASS"

#define MDNS_NAME          "robot"

#endif // CONFIG_H
