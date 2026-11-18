# ESP32 DevKit V1 ↔ NRF24L01 Ground Station

Firmware này dành cho board **ESP32 DevKit V1** đóng vai trò là trạm điều khiển mặt đất (Ground Station / Remote Controller).

---

## 1. Sơ đồ đấu dây giữa NRF24L01 và ESP32 DevKit V1

| Chân NRF24L01 | Chân ESP32 DevKit V1 | Ghi chú |
|---|---|---|
| **VCC** | **3.3V** | **LƯU Ý:** Không cấp 5V vào NRF24. Khuyến nghị hàn thêm 1 tụ 10µF–100µF giữa VCC và GND của NRF24 để chống sụt áp khi phát sóng |
| **GND** | **GND** | Nối chung mass |
| **CE** | **GPIO 4** | Chân Chip Enable |
| **CSN** | **GPIO 5** | Chân Chip Select Not (SPI SS) |
| **SCK** | **GPIO 18** | Chân SPI Clock |
| **MOSI** | **GPIO 23** | Chân SPI Master Out Slave In |
| **MISO** | **GPIO 19** | Chân SPI Master In Slave Out |
| **IRQ** | *Không nối* | Để trống |

---

## 2. Hướng dẫn nạp bằng Arduino IDE

1. Mở **Arduino IDE**.
2. Cài đặt thư viện **RF24**:
   - Vào menu `Sketch` $\rightarrow$ `Include Library` $\rightarrow$ `Manage Libraries...` (hoặc phím tắt `Ctrl + Shift + I`).
   - Gõ tìm kiếm `RF24` của tác giả **TMRh20**.
   - Bấm **Install**.
3. Chọn Board:
   - Menu `Tools` $\rightarrow$ `Board` $\rightarrow$ `esp32` $\rightarrow$ **DOIT ESP32 DEVKIT V1** (hoặc ESP32 Dev Module).
   - Chọn đúng cổng `Port` COM của ESP32.
4. Mở file [ESP32_DevKit_NRF24.ino](file:///c:/Users/Thanh/Documents/Thực%20Hành%20Thiết%20Kế%20Hệ%20Thông%20Nhúng/ĐỒ%20ÁN/XECANBANGFINAL/ESP32_DevKit_NRF24/ESP32_DevKit_NRF24.ino).
5. Bấm nút **Upload** (`Ctrl + U`) để biên dịch và nạp vào ESP32.

---

## 3. Cách sử dụng điều khiển xe từ Serial Monitor

Mở **Serial Monitor** trên Arduino IDE ở tốc độ **115200 baud**:
- Gõ `x` hoặc `X` $\rightarrow$ Gửi lệnh **ARM** (bật cân bằng xe khi xe dựng thẳng).
- Gõ `z` hoặc `Z` $\rightarrow$ Gửi lệnh **DISARM** (tắt động cơ).
- Gõ `e` hoặc `E` $\rightarrow$ **DỪNG KHẨN CẤP** (Emergency Stop).
- Gõ `w` / `s` $\rightarrow$ Tăng tốc **Tiến** / **Lùi**.
- Gõ `a` / `d` $\rightarrow$ **Quay trái** / **Quay phải**.
- Gõ phím cách (Space) $\rightarrow$ Dừng tại chỗ (Tốc độ = 0).
