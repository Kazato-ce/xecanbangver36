# Dự Án Xe Cân Bằng 2 Bánh STM32F405 — xecanbangver36

Phiên bản chuyên biệt: **TỰ ĐỘNG CÂN BẰNG THỰC TẾ KHI ĐẶT XUỐNG ĐẤT (Ground Self-Balancing)**.

---

## 1. Cơ Chế Hoạt Động Khi Đặt Xe Xuống Đất

Khác với các bản thử nghiệm cần phải chờ kết nối sóng không dây hay gửi lệnh ARM thủ công từ máy tính, **xecanbangver36** được thiết kế để bạn chỉ cần cầm xe đặt xuống sàn nhà là xe tự động kích hoạt cân bằng:

1. **Khởi động an toàn (Power On):**
   * Động cơ **hoàn toàn TẮT** (Chân PA0 = HIGH, bánh xe quay tự do).
   * Xe tiến hành đo và khử sai số tĩnh (bias) của Gyro trong khoảng 1.5 giây.
2. **Đặt xe xuống đất (Auto-Arm on Ground):**
   * Người dùng đặt xe xuống sàn và giữ xe dựng thẳng đứng (góc nghiêng trong khoảng $\pm 3.5^\circ$).
   * Khi giữ thẳng ổn định liên tục trong **250 ms (50 chu kỳ)**: Hệ thống **TỰ ĐỘNG BẬT DRIVER (PA0 = LOW)**, reset bộ tích phân PID, và bắt đầu cân bằng tức thì!
   * Bạn chỉ cần buông nhẹ tay ra là xe tự đứng vững trên 2 bánh!
3. **Chống ngã an toàn (Fall Cutoff Failsafe):**
   * Nếu xe bị xô đẩy hoặc nghiêng ngã quá $\pm 32^\circ$: Hệ thống **TỰ ĐỘNG NGẮT DRIVER NGAY LẬP TỨC (PA0 = HIGH)** và dừng xung phát, bánh xe thả lỏng để bảo vệ động cơ bước và driver TMC2208 không bị quá tải.
4. **Tự động phục hồi khi dựng lại (Auto-Recovery):**
   * Bạn chỉ cần nhấc xe lên hoặc dựng thẳng lại trên mặt sàn trong khoảng $\pm 3.5^\circ$.
   * Sau **300 ms** giữ thẳng, xe sẽ **TỰ ĐỘNG CÂN BẰNG TIẾP** mà không cần phải reset nguồn!

---

## 2. File Nạp Firmware Đã Sẵn Sàng

Firmware đã được biên dịch sạch 100% (0 warning, 0 error) bằng toolchain GCC ARM:
* **File ELF:** `build/Debug/xecanbangver36.elf`
* **File HEX:** `build/Debug/xecanbangver36.hex`
* **File BIN:** `build/Debug/xecanbangver36.bin`

Nạp vào MCU STM32F405 qua mạch nạp ST-Link (SWDIO = PA13, SWCLK = PA14) bằng **STM32CubeProgrammer** hoặc **STM32CubeIDE**.

---

## 3. Bảng Chân Phần Cứng Chuẩn (STM32F405RGT6)

| Thiết bị | Tín hiệu | Chân STM32 | Ghi chú cấu hình |
|---|---|---|---|
| **TMC2208 (Chung)** | **EN** | **PA0** | Active-low (HIGH = Tắt, LOW = Bật) |
| **TMC2208 Trái** | **STEP** | **PA1** | Phát xung bước |
| **TMC2208 Trái** | **DIR** | **PA2** | Chiều quay |
| **TMC2208 Phải** | **STEP** | **PA3** | Phát xung bước |
| **TMC2208 Phải** | **DIR** | **PA4** | Chiều quay |
| **MPU6050** | **SCL / SDA** | **PB6 / PB7** | I2C1 (Fast Mode 400kHz, trục nghiêng X) |
| **MPU6050** | **INT** | **PB3** | Ngắt ngoài EXTI3 |
| **USB Virtual COM**| **DM / DP** | **PA11 / PA12** | Cổng USB cắm trực tiếp máy tính để Test & Debug |
| **UART Telemetry** | **TX / RX** | **PA9 / PA10** | 115200 baud (Xem góc nghiêng/PID real-time) |
| **NRF24L01** | **SCK/MISO/MOSI**| **PB13/14/15** | SPI2 (Tùy chọn nhận lệnh tiến/lùi/rẽ) |
| **NRF24L01** | **CE / CSN** | **PB8 / PB9** | Điều khiển module RF |

---

## 4. Thuật Toán Cân Bằng Tiếp Sàn (Cascade PID + Zero Drift)

Được cấu hình trong file: `App/Inc/config.h`
* **Vòng trong (Inner Loop - Angle PD):**
  * `PID_ANGLE_KP = 20.0f`: Phản hồi độ cứng theo góc nghiêng $\theta$.
  * `PID_ANGLE_KD = 0.95f`: Giảm chấn theo tốc độ góc $\dot{\theta}$ (Gyro) để chống rung lắc (chống overshoot).
* **Vòng ngoài (Outer Loop - Speed PI):**
  * `PID_SPEED_KP = 0.40f` & `PID_SPEED_KI = 0.025f`: Tích phân vị trí bánh xe để triệt tiêu hiện tượng xe bị trôi dạt (zero-drift) khi tiếp xúc mặt sàn gạch hoặc thảm.
* **Bộ lọc bù:**
  * `IMU_FILTER_ALPHA = 0.985f`: Lọc 98.5% Gyro + 1.5% Accel giúp loại bỏ hoàn toàn rung động cơ học khi bánh xe chạm mặt đất.

---

## 5. Hướng Dẫn Kiểm Tra & Điều Khiển Trực Tiếp Qua Cổng USB F405

Chỉ cần cắm trực tiếp cáp Micro-USB / Type-C từ bo mạch STM32F405 vào máy tính (nhận cổng `COM` ảo Virtual COM Port), mở **Serial Monitor** (Arduino IDE, PuTTY, RealTerm, Hercules) hoặc công cụ Python GCS:

### Bảng Lệnh Kiểm Tra Phần Cứng:
1. **Kiểm tra cảm biến MPU6050:**
   * Gõ lệnh: `TEST_MPU` (hoặc `CHECK_MPU`)
   * Kết quả: Đọc chip ID (WHO_AM_I), gia tốc 3 trục Ax, Ay, Az, Gyro X, Y, Z, góc Pitch thực tế và số lỗi I2C.
2. **Kiểm tra 2 Driver TMC2208 & Động cơ bước:**
   * Gõ lệnh: `TEST_STEPPER` $\rightarrow$ Tự động bật driver (PA0=LOW), quay 2 bánh xe 35 RPM trong 5 giây để kiểm tra chiều quay và lực kéo.
   * Gõ lệnh: `TEST_MOTOR <rpm_trai> <rpm_phai> [so_giay]` $\rightarrow$ Điều khiển tốc độ quay độc lập tùy ý (VD: `TEST_MOTOR 40 40 5`).
3. **Kiểm tra Module sóng NRF24L01:**
   * Gõ lệnh: `TEST_NRF` (hoặc `CHECK_NRF`)
   * Kết quả: Kiểm tra giao tiếp SPI2 (PB13/14/15), thanh ghi CONFIG, STATUS, kênh tần số RF (2476 MHz).
4. **Kiểm tra Điều khiển Cân bằng & Trạng thái Xe:**
   * Gõ lệnh: `STATUS` $\rightarrow$ In toàn bộ báo cáo: Góc Pitch, Driver TMC2208, NRF, PID, Cảm biến vật cản.
   * Gõ lệnh: `ARM` $\rightarrow$ Bật chế độ cân bằng (khi xe dựng thẳng).
   * Gõ lệnh: `DISARM` $\rightarrow$ Tắt motor lập tức.
   * Gõ lệnh: `PLOT` $\rightarrow$ Chuyển sang định dạng vẽ đồ thị Arduino Serial Plotter.
   * Gõ lệnh: `SET_PID <kp> <kd> <ki>` $\rightarrow$ Cài đặt trực tiếp bộ số PID qua USB.
   * Gõ lệnh: `HELP` $\rightarrow$ Xem lại danh sách tất cả các lệnh.
