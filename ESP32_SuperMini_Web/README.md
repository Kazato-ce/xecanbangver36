# ESP32 Super Mini - Web Dashboard & Real-time PID Tuning

Firmware này dành cho board **ESP32 Super Mini** (hoặc ESP32-S3 Super Mini) gắn trực tiếp trên robot để phát Web Server, hiển thị góc nghiêng, đồ thị thời gian thực và cho phép chỉnh thông số PID trực tiếp từ trình duyệt Web điện thoại/máy tính mà không cần cắm dây.

---

## 1. Sơ đồ nối dây giữa STM32F405 và ESP32 Super Mini

| Chân STM32F405 | Chân ESP32-S3 Super Mini | Chân ESP32 Super Mini thường | Ghi chú |
|---|---|---|---|
| **PA9 (USART1_TX)** | **GPIO 44** (RXD) | **GPIO 16** (RX2) | Nhận dữ liệu Telemetry từ STM32 |
| **PA10 (USART1_RX)** | **GPIO 43** (TXD) | **GPIO 17** (TX2) | Gửi lệnh PID / ARM sang STM32 |
| **GND** | **GND** | **GND** | **BẮT BUỘC NỐI CHUNG MASS GND** |
| **5V** (từ mạch hạ áp) | **5V / VBUS** | **5V / VBUS** | Cấp nguồn cho ESP32 |

*(Chân RX/TX có thể đổi lại dễ dàng trong file `config.h` nếu bạn hàn vào chân khác)*

---

## 2. Hướng dẫn nạp bằng Arduino IDE

1. Mở **Arduino IDE**.
2. Chọn Board:
   - Nếu bạn dùng **ESP32-S3 Super Mini**: Vào `Tools` $\rightarrow$ `Board` $\rightarrow$ `esp32` $\rightarrow$ **ESP32S3 Dev Module** (hoặc NGENUITY ESP32-S3). Bật `USB CDC On Boot: Enabled`.
   - Nếu bạn dùng **ESP32 Super Mini (ESP32-C3 hoặc ESP32 thường)**: Chọn đúng board tương ứng (ví dụ ESP32 Dev Module).
3. Chọn đúng cổng `Port` COM.
4. Mở file [ESP32_SuperMini_Web.ino](file:///c:/Users/Thanh/Documents/Thực%20Hành%20Thiết%20Kế%20Hệ%20Thông%20Nhúng/ĐỒ%20ÁN/XECANBANGFINAL/ESP32_SuperMini_Web/ESP32_SuperMini_Web.ino).
5. Bấm nút **Upload** (`Ctrl + U`) để nạp.

---

## 3. Cách kết nối và sử dụng Web Dashboard

1. Cấp nguồn cho robot. ESP32 Super Mini sẽ tự động phát một mạng WiFi:
   - **Tên WiFi (SSID):** `Robot_CanBang_AP`
   - **Mật khẩu:** `12345678`
2. Dùng điện thoại hoặc máy tính kết nối vào WiFi trên.
3. Mở trình duyệt Web (Chrome, Safari, Cốc Cốc, Edge...) và gõ địa chỉ:
   ```text
   http://192.168.4.1
   ```
   *(hoặc gõ `http://robot.local`)*
4. Giao diện Web Dashboard sẽ xuất hiện:
   - **Góc nghiêng Pitch & Tốc độ góc:** Cập nhật liên tục 20 Hz (50 ms).
   - **Đồ thị Canvas:** Vẽ diễn biến góc nghiêng và góc mục tiêu trực tiếp.
   - **Thanh trượt chỉnh PID:** Kéo thanh trượt hoặc nhập số cho $K_p$, $K_d$, $K_i$, Trim góc đặt.
   - Bấm **"Lưu & Cập nhật PID sang STM32"**: Thông số sẽ lập tức được nạp vào bộ điều khiển của STM32 mà xe không cần dừng lại hay khởi động lại!
   - Nút **"BẬT CÂN BẰNG (ARM)"** và **"TẮT ĐỘNG CƠ (DISARM)"** để can thiệp an toàn.
