#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>
#include "board_config.h"

// ======================================================
// 1. CẤU HÌNH WIFI SOFTAP & IP TĨNH
// ======================================================
const char *ap_ssid     = "Xe_FPV";
const char *ap_password = "uit2024iot";

IPAddress AP_LOCAL_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

// IP cố định của mạch nhận ESP32 DevKit V1
const char *RX_STATIC_IP_STR = "192.168.4.2";
const uint16_t UDP_PORT      = 8888;

#define MAX_UDP_PAYLOAD 1400
#define PACKET_MAGIC    0xAA55

#pragma pack(push, 1)
struct UdpFrameHeader {
  uint16_t magic;
  uint16_t frameId;
  uint8_t  chunkIdx;
  uint8_t  totalChunks;
  uint16_t chunkLen;
};
#pragma pack(pop)

int udpSock = -1;
struct sockaddr_in destAddr = {};
uint16_t frameCounter = 0;

// ======================================================
// 2. TRAO ĐỔI CON TRỎ ZERO-COPY GIỮA CAMERA & UDP
// ======================================================
camera_fb_t *g_latest_fb   = nullptr;
portMUX_TYPE g_fb_mux      = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_txTaskHandle = nullptr;

void cameraCaptureTask(void *pvParameters) {
  while (true) {
    // Hàm này tự block chờ ngắt DMA xong khung hình, không cần vTaskDelay(1) ở cuối
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(1);
      continue;
    }

    camera_fb_t *old_fb = nullptr;
    portENTER_CRITICAL(&g_fb_mux);
    old_fb = g_latest_fb;
    g_latest_fb = fb;
    portEXIT_CRITICAL(&g_fb_mux);

    if (old_fb) {
      esp_camera_fb_return(old_fb);
    }

    // Đánh thức loop() gửi UDP ngay lập tức (< 5 us)
    if (g_txTaskHandle != nullptr) {
      xTaskNotifyGive(g_txTaskHandle);
    }
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

  // Xóa trắng toàn bộ struct config tránh giá trị rác trên Stack
  camera_config_t config = {};
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

  config.xclk_freq_hz     = 20000000;
  config.frame_size       = FRAMESIZE_QVGA;      // 320x240 khớp màn hình ST7789
  config.pixel_format     = PIXFORMAT_JPEG;
  config.grab_mode        = CAMERA_GRAB_LATEST;
  config.jpeg_quality     = 18;
  config.jpeg_buffer_size = 0;                   // Bắt buộc đặt = 0 trên core 3.3.12

  // Ép dùng RAM nội siêu tốc để tránh nghẽn bus PSRAM với WiFi HT40
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.fb_count    = 3;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    // Dự phòng nếu DRAM không đủ cấp 3 buffer thì lùi về 2 buffer DRAM
    config.fb_count = 2;
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed: 0x%x\n", err);
      return;
    }
    Serial.println("DRAM OK -> Dung 2 buffer trong RAM noi");
  } else {
    Serial.println("DRAM OK -> Dung 3 buffer trong RAM noi");
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);                         // Tắt Night Mode giữ cứng FPS
  s->set_ae_level(s, 2);                     // Đẩy sáng phơi sáng tự động
  s->set_gainceiling(s, GAINCEILING_128X);    // Tăng ISO điện tử
  s->set_brightness(s, 2);
  s->set_contrast(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);

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

  // Cấu hình IP tĩnh cho SoftAP trên ESP32-S3 CAM
  WiFi.mode(WIFI_AP);
  WiFi.persistent(false);
  WiFi.softAPConfig(AP_LOCAL_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(ap_ssid, ap_password, 1, 0, 2);

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Mở socket LwIP thuần để bắn gói tin Zero-Copy
  udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  destAddr.sin_family      = AF_INET;
  destAddr.sin_port        = htons(UDP_PORT);
  destAddr.sin_addr.s_addr = inet_addr(RX_STATIC_IP_STR);

  g_txTaskHandle = xTaskGetCurrentTaskHandle();

  xTaskCreatePinnedToCore(
    cameraCaptureTask,
    "CamCapture",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  Serial.printf("S3 CAM AP IP: %s -> Target DevKit V1: %s:%u (DRAM Zero-Copy)\n",
                WiFi.softAPIP().toString().c_str(),
                RX_STATIC_IP_STR,
                UDP_PORT);
}

void loop() {
  // Chỉ phát UDP khi ESP32 DevKit V1 đã kết nối vào WiFi
  if (WiFi.softAPgetStationNum() == 0) {
    camera_fb_t *stale_fb = nullptr;
    portENTER_CRITICAL(&g_fb_mux);
    stale_fb = g_latest_fb;
    g_latest_fb = nullptr;
    portEXIT_CRITICAL(&g_fb_mux);
    if (stale_fb) {
      esp_camera_fb_return(stale_fb);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }

  // Ngủ chờ tín hiệu từ cameraCaptureTask (tối đa 10ms)
  if (g_latest_fb == nullptr) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }

  camera_fb_t *fb = nullptr;
  portENTER_CRITICAL(&g_fb_mux);
  fb = g_latest_fb;
  g_latest_fb = nullptr;
  portEXIT_CRITICAL(&g_fb_mux);

  if (!fb) {
    return;
  }

  uint16_t fid = frameCounter++;
  uint16_t totalChunks = (fb->len + MAX_UDP_PAYLOAD - 1) / MAX_UDP_PAYLOAD;

  if (totalChunks > 0 && totalChunks <= 250 && udpSock >= 0) {
    for (uint8_t c = 0; c < totalChunks; c++) {
      size_t offset = (size_t)c * MAX_UDP_PAYLOAD;
      uint16_t chunkLen = min((size_t)MAX_UDP_PAYLOAD, fb->len - offset);

      UdpFrameHeader hdr;
      hdr.magic       = PACKET_MAGIC;
      hdr.frameId     = fid;
      hdr.chunkIdx    = c;
      hdr.totalChunks = (uint8_t)totalChunks;
      hdr.chunkLen    = chunkLen;

      // Scatter-Gather I/O: ghép header và con trỏ fb->buf trực tiếp trong DRAM, không memcpy
      struct iovec iov[2];
      iov[0].iov_base = &hdr;
      iov[0].iov_len  = sizeof(UdpFrameHeader);
      iov[1].iov_base = fb->buf + offset;
      iov[1].iov_len  = chunkLen;

      struct msghdr msg = {};
      msg.msg_name    = &destAddr;
      msg.msg_namelen = sizeof(destAddr);
      msg.msg_iov     = iov;
      msg.msg_iovlen  = 2;

      if (sendmsg(udpSock, &msg, 0) < 0) {
        delayMicroseconds(150);
        sendmsg(udpSock, &msg, 0);
      }

      if (c + 1 < totalChunks) {
        delayMicroseconds(100);
      }
    }
  }

  // Trả thẳng buffer DRAM về vòng DMA camera
  esp_camera_fb_return(fb);
}