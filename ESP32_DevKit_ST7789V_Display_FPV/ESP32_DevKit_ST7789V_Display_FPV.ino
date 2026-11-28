#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#define MAX_BUFFERED_PIXELS (320 * 16)
#include <JPEGDEC.h>

// ======================================================
// 1. CẤU HÌNH WIFI & IP TĨNH CHO ESP32 DEVKIT V1
// ======================================================
const char* CAM_SSID     = "Xe_FPV";
const char* CAM_PASSWORD = "uit2024iot";

IPAddress STA_LOCAL_IP(192, 168, 4, 2);   // IP tĩnh của ESP32 DevKit V1
IPAddress STA_GATEWAY(192, 168, 4, 1);    // IP của ESP32-S3 CAM
IPAddress STA_SUBNET(255, 255, 255, 0);

const uint16_t UDP_PORT  = 8888;

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

// ======================================================
// 2. CẤU HÌNH MÀN HÌNH ST7789 QUA LOVYANGFX (80 MHz + DMA)
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
      cfg.freq_write  = 80000000;        // 80 MHz (hạ về 40000000 nếu dây nối dài bị nhiễu)
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
      cfg.rgb_order        = false;      // Đổi thành true nếu bị tráo màu Đỏ và Xanh dương
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
JPEGDEC jpeg;

#define MAX_JPG_SIZE 26624
uint8_t* frameBuf[3] = {nullptr, nullptr, nullptr};
size_t   frameLen[3] = {0, 0, 0};

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

void lockWifiRadioConfig() {
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

// ======================================================
// 3. CALLBACK VẼ QUA SPI DMA GOM DẢI NGANG 320x16 ZERO-COPY
// ======================================================
int JPEGDraw(JPEGDRAW *pDraw) {
  if (pDraw->y >= tft.height() || pDraw->x >= tft.width()) return 0;

  int copyW = min((int)pDraw->iWidth, (int)(tft.width() - pDraw->x));
  int copyH = min((int)pDraw->iHeight, (int)(tft.height() - pDraw->y));

  if (copyH <= 16) {
    uint16_t *dstStrip = dmaStrip[dmaStripIdx];
    for (int row = 0; row < copyH; row++) {
      memcpy(&dstStrip[row * 320 + pDraw->x],
             &pDraw->pPixels[row * pDraw->iWidth],
             copyW * sizeof(uint16_t));
    }

    // Chỉ bắn DMA một lần khi đã ghép chạm mép phải màn hình (320px)
    if (pDraw->x + pDraw->iWidth >= tft.width()) {
      tft.waitDMA();
      tft.pushImageDMA(0, pDraw->y, tft.width(), copyH, dstStrip);
      dmaStripIdx ^= 1;
    }
  } else {
    tft.waitDMA();
    tft.pushImage(pDraw->x, pDraw->y, copyW, copyH, pDraw->pPixels);
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
// 4. TASK CORE 0: LẮNG NGHE CỔNG UDP CỐ ĐỊNH 8888
// ======================================================
void wifiUdpTask(void *pvParameters) {
  WiFiUDP udp;
  static uint8_t rxPacket[sizeof(UdpFrameHeader) + MAX_UDP_PAYLOAD + 16];
  bool udpStarted   = false;
  bool wasConnected = true;

  uint32_t lastRxMs       = 0;
  uint32_t curFrameId     = 0xFFFFFFFF;
  uint8_t  expectedChunk  = 0;
  size_t   receivedBytes  = 0;
  bool     frameValid     = false;
  bool     frameComplete  = true;

  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      isStreamConnected = false;
      wasConnected = false;
      if (udpStarted) {
        udp.stop();
        udpStarted = false;
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // Khóa lại thông số HT40 & tắt Sleep ngay khi vừa tự kết nối lại WiFi
    if (!wasConnected) {
      wasConnected = true;
      lockWifiRadioConfig();
    }

    if (!udpStarted) {
      udp.begin(UDP_PORT); // Mở cố định cổng 8888 trên IP 192.168.4.2
      udpStarted = true;
    }

    uint32_t now = millis();
    int pktSize = udp.parsePacket();
    if (pktSize <= 0) {
      if (isStreamConnected && (now - lastRxMs > 1500)) {
        isStreamConnected = false;
      }
      vTaskDelay(1);
      continue;
    }

    int len = udp.read(rxPacket, sizeof(rxPacket));
    if (len < (int)sizeof(UdpFrameHeader)) continue;

    UdpFrameHeader hdr;
    memcpy(&hdr, rxPacket, sizeof(UdpFrameHeader));

    if (hdr.magic != PACKET_MAGIC ||
        hdr.chunkLen > MAX_UDP_PAYLOAD ||
        len < (int)(sizeof(UdpFrameHeader) + hdr.chunkLen)) {
      continue;
    }

    lastRxMs = now;
    isStreamConnected = true;

    if ((uint32_t)hdr.frameId != curFrameId) {
      if (curFrameId != 0xFFFFFFFF && !frameComplete) {
        dropCounter++;
      }

      curFrameId    = hdr.frameId;
      frameComplete = false;

      if (hdr.chunkIdx == 0) {
        expectedChunk = 0;
        receivedBytes = 0;
        frameValid    = true;
      } else {
        frameValid = false;
        dropCounter++;
      }
    }

    if (frameValid && hdr.chunkIdx == expectedChunk) {
      if (receivedBytes + hdr.chunkLen <= MAX_JPG_SIZE) {
        memcpy(frameBuf[writeIdx] + receivedBytes,
               rxPacket + sizeof(UdpFrameHeader),
               hdr.chunkLen);
        receivedBytes += hdr.chunkLen;
        expectedChunk++;

        if (expectedChunk == hdr.totalChunks) {
          if (receivedBytes >= 4 &&
              frameBuf[writeIdx][0] == 0xFF &&
              frameBuf[writeIdx][1] == 0xD8) {
            portENTER_CRITICAL(&frameMux);
            frameLen[writeIdx] = receivedBytes;
            readyIdx = writeIdx;
            for (int k = 0; k < 3; k++) {
              if (k != decodingIdx && k != readyIdx) {
                writeIdx = k;
                break;
              }
            }
            portEXIT_CRITICAL(&frameMux);

            lastFrameSize = receivedBytes;
            rxFpsCounter++;
          } else {
            dropCounter++;
          }

          frameComplete = true;
          frameValid    = false;
        }
      } else {
        frameValid    = false;
        frameComplete = true;
        dropCounter++;
      }
    } else if (frameValid) {
      frameValid    = false;
      frameComplete = true;
      dropCounter++;
    }
  }
}

// ======================================================
// 5. SETUP & LOOP (CORE 1 GIẢI MÃ JPEGDEC & DMA)
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
  tft.setRotation(3);
  tft.setSwapBytes(false);
  tft.fillScreen(TFT_BLACK);

  showStatusMessage("Dang ket noi WiFi...", CAM_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.config(STA_LOCAL_IP, STA_GATEWAY, STA_SUBNET);
  WiFi.begin(CAM_SSID, CAM_PASSWORD, 1);

  lockWifiRadioConfig();

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  // Khóa cứng lại lần nữa sau khi bắt tay WiFi xong để chống driver tự bật Sleep
  lockWifiRadioConfig();

  Serial.println("\n[WIFI] Static IP: " + WiFi.localIP().toString());
  isWaitingScreenShown = false;

  xTaskCreatePinnedToCore(
    wifiUdpTask,
    "WiFiUDP",
    8192,
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
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
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
      showStatusMessage("Dang cho UDP CAM...", "Kiem tra nguon xe");
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