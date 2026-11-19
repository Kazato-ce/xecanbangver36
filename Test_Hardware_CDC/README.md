# BỘ TEST PHẦN CỨNG XE CÂN BẰNG STM32F405 (XEM QUA USB CDC)

Thư mục này là **firmware chẩn đoán độc lập** dành riêng cho STM32F405RGT6 để kiểm tra và xác nhận 100% phần cứng trước khi chạy thuật toán cân bằng:
1. **MPU6050** (I2C1: SCL=PB6, SDA=PB7, INT=PB3)
2. **TMC2208 Stepper Drivers** (EN=PA0, STEP_L=PA1, DIR_L=PA2, STEP_R=PA3, DIR_R=PA4)
3. **NRF24L01 PA+LNA** (SPI2: SCK=PB13, MISO=PB14, MOSI=PB15, CE=PB8, CSN=PB9)
4. **USB CDC Virtual COM Port** (PA11 USB_DM, PA12 USB_DP)

---

## 1. File Nạp Sẵn Có Trong Thư Mục

Firmware đã được biên dịch sẵn, bạn có thể nạp ngay vào STM32 qua ST-Link:
- File nhị phân: `Test_Hardware_CDC.bin`
- File Intel Hex: `Test_Hardware_CDC.hex`

---

## 2. Hướng Dẫn Sử Dụng Nhanh

### Bước 1: Nạp Firmware Vào STM32
- Dùng **STM32CubeProgrammer**, **ST-Link Utility** hoặc lệnh OpenOCD/STM32CubeIDE để nạp file `Test_Hardware_CDC.bin` (hoặc `Test_Hardware_CDC.hex`) vào board STM32F405.

### Bước 2: Cắm Dây USB Từ STM32 Vào Máy Tính
- Cắm cáp Micro-USB / Type-C nối từ cổng USB của board STM32F405 vào máy tính.
- Máy tính sẽ tự động nhận diện thiết bị: **`STM32 Virtual ComPort (COMx)`** (ví dụ COM7, COM8).

### Bước 3: Mở Giao Diện Chẩn Đoán
Bạn có thể dùng một trong các cách sau:
- **Cách 1 (Khuyên dùng):** Chạy công cụ Python có sẵn trong thư mục:
  ```bash
  python tools/test_viewer.py
  ```
- **Cách 2:** Mở bất kỳ phần mềm Serial Monitor nào (Arduino Serial Monitor, PuTTY, Tera Term, VS Code Serial Monitor):
  - Chọn cổng COM của STM32.
  - Tốc độ Baud: `115200` (hoặc bất kỳ vì USB CDC tự thích ứng tốc độ).
  - Bấm phím `?` hoặc `Enter` để hiện Menu chẩn đoán.

---

## 3. Các Lệnh Điều Khiển Trong Console USB CDC

Khi mở màn hình console, bạn sẽ thấy giao diện:

```text
======================================================================
   BỘ CHẨN ĐOÁN PHẦN CỨNG XE CÂN BẰNG STM32F405 (USB CDC)
   Kiểm tra: MPU6050 (I2C1) | TMC2208 (GPIO) | NRF24L01 (SPI2)
======================================================================
 [1] QUÉT TỔNG THỂ PHẦN CỨNG (Scan 3 module trong 1 giây)
 [2] STREAM GÓC NGHIÊNG REAL-TIME (MPU6050 Live Pitch & Gyro)
 [3] TEST ĐỘNG CƠ TRÁI (PA1 STEP, PA2 DIR: Quay 1 vòng tới/lui)
 [4] TEST ĐỘNG CƠ PHẢI (PA3 STEP, PA4 DIR: Quay 1 vòng tới/lui)
 [5] TEST CẢ 2 ĐỘNG CƠ (Cùng quay tới 2 vòng -> cùng quay lui)
 [6] BẬT DRIVER ĐỘNG CƠ (PA0 = LOW -> Động cơ gông cứng giữ vị trí)
 [7] TẮT DRIVER ĐỘNG CƠ (PA0 = HIGH -> Động cơ ngắt lực an toàn)
 [8] BẮT SÓNG NRF24 (Xem trực tiếp gói tin từ ESP32 DevKit TX)
 [9] HIỆU CHUẨN GYRO MPU6050 (Đặt xe đứng yên 3 giây)
 [0] DỪNG TẤT CẢ / QUAY VỀ MENU CHÍNH
======================================================================
Nhập phím [0 - 9] từ bàn phím để chọn:
```

### Chi tiết các chức năng:
- **Bấm `1`:** Quét nhanh toàn bộ 3 linh kiện:
  - MPU6050: đọc mã `WHO_AM_I` (phải là 0x68), đọc góc nghiêng Pitch ban đầu.
  - NRF24L01: đọc thanh ghi STATUS, CONFIG, Kênh 76.
  - TMC2208: kiểm tra trạng thái mức logic chân EN (PA0).
  - Báo kết quả **`[PASS 🟢]`** hoặc **`[FAIL 🔴]`** kèm lý do dây cắm.
- **Bấm `2`:** Bật chế độ xem trực tiếp góc nghiêng theo thời gian thực (mỗi 100ms):
  - Dùng tay nghiêng xe qua lại: xem góc Pitch thay đổi từ $-15^\circ$ đến $+15^\circ$.
  - Bấm phím bất kỳ để dừng stream.
- **Bấm `3` / `4` / `5`:** Test động cơ bước:
  - *Lưu ý: Bánh xe phải được nhấc khỏi mặt đất.*
  - STM32 sẽ tự động bật driver (PA0=LOW), phát đúng 3200 xung quay 1 vòng tới, dừng 300ms rồi quay 1 vòng lui để kiểm tra xung STEP và chiều DIR.
- **Bấm `6`:** Bật driver (PA0 = LOW). Cả 2 động cơ sẽ gông cứng để bạn dùng tay xoay thử bánh xe và cảm nhận mô-men giữ của động cơ.
- **Bấm `7`:** Tắt driver (PA0 = HIGH). Động cơ ngắt điện, xoay trơn tru và không nóng.
- **Bấm `8`:** Bắt sóng NRF24L01:
  - Bật kit ESP32 DevKit TX (chạy sketch `ESP32_DevKit_NRF24.ino`).
  - Màn hình USB CDC sẽ in từng gói tin nhận được từ ESP32:
    `[NRF24 RX] Gói #152 | Lệnh: DRIVE | Drive: 0 | Steer: 0 | CRC: OK 🟢`
  - Đồng thời kích hoạt Auto-ACK để đèn xanh trên kit ESP32 sáng liên tục!
- **Bấm `9`:** Đo 500 mẫu khi xe đứng yên để tính Gyro Bias offset.
- **Bấm `0`:** Dừng khẩn cấp, tắt động cơ và quay về Menu.

---

## 4. Biên Dịch Lại (Nếu Có Chỉnh Sửa Code)

Dự án sử dụng CMake + Ninja + ARM GCC (tương tự firmware chính):
```bash
cmake -S Test_Hardware_CDC --preset Debug
cmake --build Test_Hardware_CDC/build/Debug
```
Output: `Test_Hardware_CDC/build/Debug/Test_Hardware_CDC.bin`
