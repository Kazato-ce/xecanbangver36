#include <WiFi.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <JPEGDEC.h>

// ======================================================
// 1. CẤU HÌNH WIFI & STREAM CỦA ESP32-S3 CAM
// ======================================================
const char* CAM_SSID     = "Xe_FPV";
const char* CAM_PASSWORD = "uit2024iot";

const char* CAM_HOST     = "192.168.4.1";
const uint16_t CAM_PORT  = 81;
const char* CAM_PATH     = "/stream";

// ======================================================
// 2. CẤU HÌNH MÀN HÌNH ST7789 QUA LOVYANGFX (60 MHz + DMA)
// ======================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = VSPI_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 60000000;        // 60 MHz (nếu dây dài bị nhiễu thì hạ về 40000000)
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = 18;
      cfg.pin_mosi    = 23;
      cfg.pin_miso    = 19;
      cfg.pin_dc      = 17;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 5;
      cfg.pin_rst          = 4;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = false;
      cfg.rgb_order        = false;      // Đổi thành true nếu bị tráo Đỏ và Xanh dương
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
JPEGDEC jpeg;

// Nâng lên 26KB/khung (3 x 26KB = 78KB Heap) để không rớt khung khi quay ra vùng nhiều chi tiết
#define MAX_JPG_SIZE 26624
uint8_t* frameBuf[3] = {nullptr, nullptr, nullptr};
size_t   frameLen[3] = {0, 0, 0};
uint8_t  rxChunk[2048];

// 2 vùng đệm DMA ping-pong theo dải ngang (320 x 16 pixel = 10.240 bytes/dải)
#define DMA_STRIP_PIXELS (320 * 16)
uint16_t* dmaStrip[2] = {nullptr, nullptr};
uint8_t   dmaStripIdx = 0;

volatile int readyIdx    = -1;
volatile int decodingIdx = -1;
int writeIdx             = 0;
portMUX_TYPE frameMux    = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t rxFpsCounter   = 0;
volatile uint32_t tftFpsCounter  = 0;
volatile uint32_t dropCounter    = 0;
volatile size_t   lastFrameSize  = 0;
volatile bool isStreamConnected  = false;
bool isWaitingScreenShown        = false;

// ======================================================
// 3. CALLBACK VẼ DẢI NGANG QUA SPI DMA (CHỈ 15 LẦN/KHUNG)
// ======================================================
int JPEGDraw(JPEGDRAW *pDraw) {
  if (pDraw->y >= tft.height()) return 0;

  uint32_t pixels = (uint32_t)pDraw->iWidth * pDraw->iHeight;
  if (pixels <= DMA_STRIP_PIXELS) {
    tft.waitDMA();
    memcpy(dmaStrip[dmaStripIdx], pDraw->pPixels, pixels * sizeof(uint16_t));
    tft.pushImageDMA(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, dmaStrip[dmaStripIdx]);
    dmaStripIdx ^= 1;
  } else {
    tft.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
  }

  return 1;
}

void showStatusMessage(const char* line1, const char* line2) {
  if (isWaitingScreenShown) return;
  isWaitingScreenShown = true;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(16, 100);
  tft.println(line1);

  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(16, 130);
  tft.println(line2);
}

// ======================================================
// 4. TASK CORE 0: CHUYÊN NHẬN LUỒNG MJPEG TỪ WI-FI
// ======================================================
void wifiStreamTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      isStreamConnected = false;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    WiFiClient client;
    client.setNoDelay(true);

    if (!client.connect(CAM_HOST, CAM_PORT, 1500)) {
      isStreamConnected = false;
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    client.print(String("GET ") + CAM_PATH + " HTTP/1.1\r\n" +
                 "Host: " + CAM_HOST + ":" + String(CAM_PORT) + "\r\n" +
                 "Connection: keep-alive\r\n\r\n");

    bool inFrame = false;
    size_t jpgLen = 0;
    uint8_t prevByte = 0;
    uint32_t lastDataMs = millis();

    while (client.connected() || client.available() > 0) {
      int avail = client.available();
      if (avail <= 0) {
        if (millis() - lastDataMs > 2500) break;
        vTaskDelay(1);
        continue;
      }

      int toRead = min(avail, (int)sizeof(rxChunk));
      int n = client.read(rxChunk, toRead);
      if (n <= 0) continue;

      lastDataMs = millis();
      isStreamConnected = true;

      for (int i = 0; i < n; i++) {
        uint8_t b = rxChunk[i];

        if (!inFrame) {
          if (prevByte == 0xFF && b == 0xD8) {
            inFrame = true;
            jpgLen = 0;
            frameBuf[writeIdx][jpgLen++] = 0xFF;
            frameBuf[writeIdx][jpgLen++] = 0xD8;
          }
        } else {
          if (jpgLen < MAX_JPG_SIZE) {
            frameBuf[writeIdx][jpgLen++] = b;

            if (prevByte == 0xFF && b == 0xD9) {
              portENTER_CRITICAL(&frameMux);
              frameLen[writeIdx] = jpgLen;
              readyIdx = writeIdx;
              for (int k = 0; k < 3; k++) {
                if (k != decodingIdx && k != readyIdx) {
                  writeIdx = k;
                  break;
                }
              }
              portEXIT_CRITICAL(&frameMux);

              lastFrameSize = jpgLen;
              rxFpsCounter++;
              inFrame = false;
              jpgLen = 0;
            }
          } else {
            // Đếm số khung vượt trần MAX_JPG_SIZE để theo dõi trên Serial
            dropCounter++;
            inFrame = false;
            jpgLen = 0;
          }
        }
        prevByte = b;
      }
    }

    isStreamConnected = false;
    client.stop();
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ======================================================
// 5. SETUP & LOOP (CORE 1 CHUYÊN GIẢI MÃ JPEGDEC & DMA)
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  for (int i = 0; i < 3; i++) {
    frameBuf[i] = (uint8_t*)malloc(MAX_JPG_SIZE);
    if (!frameBuf[i]) {
      Serial.printf("[LOI] Khong du Heap cho frameBuf[%d]\n", i);
      while (true) delay(1000);
    }
  }
  for (int i = 0; i < 2; i++) {
    dmaStrip[i] = (uint16_t*)heap_caps_malloc(DMA_STRIP_PIXELS * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!dmaStrip[i]) {
      Serial.printf("[LOI] Khong du DMA RAM cho dmaStrip[%d]\n", i);
      while (true) delay(1000);
    }
  }

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true); // Giao việc đảo byte màu cho phần cứng SPI để giảm tải CPU Core 1
  tft.fillScreen(TFT_BLACK);

  showStatusMessage("Dang ket noi WiFi...", CAM_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(CAM_SSID, CAM_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\n[WIFI] Da ket noi! IP: " + WiFi.localIP().toString());
  isWaitingScreenShown = false;

  xTaskCreatePinnedToCore(
    wifiStreamTask,
    "WiFiStream",
    4096,
    NULL,
    2,
    NULL,
    0
  );
}

void loop() {
  int idx = -1;
  size_t len = 0;

  portENTER_CRITICAL(&frameMux);
  if (readyIdx != -1) {
    idx = readyIdx;
    readyIdx = -1;
    decodingIdx = idx;
    len = frameLen[idx];
  }
  portEXIT_CRITICAL(&frameMux);

  if (idx != -1) {
    isWaitingScreenShown = false;

    if (jpeg.openRAM(frameBuf[idx], len, JPEGDraw)) {
      jpeg.setPixelType(RGB565_LITTLE_ENDIAN); // Xuất trực tiếp không đảo byte bằng phần mềm
      tft.startWrite();
      jpeg.decode(0, 0, 0);
      tft.waitDMA();
      tft.endWrite();
      jpeg.close();
      tftFpsCounter++;
    }

    portENTER_CRITICAL(&frameMux);
    decodingIdx = -1;
    portEXIT_CRITICAL(&frameMux);
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      showStatusMessage("Mat WiFi S3 CAM", "Dang ket noi lai...");
    } else if (!isStreamConnected) {
      showStatusMessage("Dang cho S3 CAM...", "Kiem tra nguon xe hoac doi GCS tat");
    }
    vTaskDelay(1);
  }

  static uint32_t lastLogMs = 0;
  uint32_t now = millis();
  if (now - lastLogMs >= 1000) {
    uint32_t rx   = rxFpsCounter;
    uint32_t tx   = tftFpsCounter;
    uint32_t drop = dropCounter;
    rxFpsCounter  = 0;
    tftFpsCounter = 0;
    dropCounter   = 0;
    lastLogMs = now;
    Serial.printf("[FPV] Core0 RX: %u FPS | Core1 TFT: %u FPS | Size: %u B | Drop: %u\n",
                  rx, tx, (unsigned int)lastFrameSize, drop);
  }
}