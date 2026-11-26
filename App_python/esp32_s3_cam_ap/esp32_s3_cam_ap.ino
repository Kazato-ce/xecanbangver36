#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>



// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// SoftAP credentials
// ===========================
const char *ap_ssid     = "Xe_FPV";
const char *ap_password = "uit2024iot";

WiFiServer streamServer(81);

// ======================================================
// TRAO ĐỔI KHUNG HÌNH ZERO-COPY GIỮA CORE 0 VÀ CORE 1
// ======================================================
camera_fb_t *g_latest_fb = nullptr;
portMUX_TYPE g_fb_mux    = portMUX_INITIALIZER_UNLOCKED;

// Task chạy trên Core 0: Chuyên lấy khung hình mới nhất từ DMA/PSRAM
void cameraCaptureTask(void *pvParameters) {
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(1);
      continue;
    }

    camera_fb_t *old_fb = nullptr;
    portENTER_CRITICAL(&g_fb_mux);
    old_fb = g_latest_fb;   // Lấy khung cũ chưa kịp gửi (nếu có)
    g_latest_fb = fb;       // Cập nhật con trỏ khung mới nhất
    portEXIT_CRITICAL(&g_fb_mux);

    // Trả lại khung cũ ngay cho phần cứng DMA để không làm nghẽn cảm biến
    if (old_fb) {
      esp_camera_fb_return(old_fb);
    }

    vTaskDelay(1);
  }
}

void initLedFlashLocal() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
  ledcWrite(LED_GPIO_NUM, 0);
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240 khớp 1:1 màn hình ST7789
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 23;                  // Giảm từ 25 -> 14 để ảnh nét hơn mà vẫn đạt ~30 FPS
  config.fb_count     = 3;                   // 3 buffer PSRAM cho pipeline 2 nhân

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);
  s->set_gainceiling(s, (gainceiling_t)6);
  s->set_sharpness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
  }
#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  initLedFlashLocal();

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_18_5dBm);
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 2);


  streamServer.begin();
  streamServer.setNoDelay(true);

  // Ghim task chụp camera vào Core 0
  xTaskCreatePinnedToCore(
    cameraCaptureTask,
    "CamCapture",
    4096,
    NULL,
    2,
    NULL,
    0
  );

  Serial.print("Camera AP ready! Stream: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
}

// ======================================================
// CORE 1: CHUYÊN PHÁT LUỒNG MJPEG QUA WI-FI
// ======================================================
void loop() {
  WiFiClient client = streamServer.available();
  if (!client) {
    vTaskDelay(2);
    return;
  }

  Serial.println("[STREAM] Client da ket noi!");
  client.setNoDelay(true);
  client.setTimeout(1); // Thoát nhanh trong 1s nếu thiết bị nhận tắt nguồn

  // Đọc bỏ HTTP Request Header
  while (client.connected() && client.available()) {
    String line = client.readStringUntil('\n');
    if (line == "\r" || line.length() == 0) break;
  }

  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Access-Control-Allow-Origin: *\r\n"
               "Connection: close\r\n\r\n");

  while (client.connected()) {
    camera_fb_t *fb = nullptr;

    portENTER_CRITICAL(&g_fb_mux);
    fb = g_latest_fb;
    g_latest_fb = nullptr;
    portEXIT_CRITICAL(&g_fb_mux);

    if (!fb) {
      vTaskDelay(1);
      continue;
    }

    char partHeader[96];
    int hLen = snprintf(partHeader, sizeof(partHeader),
                        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        fb->len);

    if (client.write((const uint8_t *)partHeader, hLen) != hLen ||
        client.write(fb->buf, fb->len) != fb->len ||
        client.write((const uint8_t *)"\r\n", 2) != 2) {
      esp_camera_fb_return(fb);
      break;
    }

    esp_camera_fb_return(fb);
  }

  client.stop();
  Serial.println("[STREAM] Client da ngat ket noi.");
}