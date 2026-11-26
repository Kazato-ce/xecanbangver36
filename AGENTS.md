# AGENTS.md — Firmware xe cân bằng 2 bánh STM32F405RGT6

Tài liệu này dành cho Codex agent. Đọc toàn bộ trước khi sửa bất kỳ file nào.
Nguồn pinout gốc: `docs/Huong_dan_cau_hinh_STM32F405_tren_CubeMX.md`.
Nếu hai tài liệu mâu thuẫn, **dừng lại và hỏi người dùng**, không tự chọn.

---

## 1. Mục tiêu

Viết firmware C (STM32 HAL) cho xe cân bằng 2 bánh chạy động cơ bước:

- MCU: **STM32F405RGT6**, LQFP64.
- 2 driver **TMC2208** (chế độ STEP/DIR) + 2 động cơ NEMA 17.
- **MPU6050** đo góc nghiêng (I2C1).
- **NRF24L01 PA+LNA** nhận lệnh điều khiển (SPI2).
- **VL53L0X** (I2C2) và 3 × **HC-SR05** để đo vật cản (ưu tiên thấp).
- Chuỗi điều khiển: `PC (Python/COM) → ESP32 DevKit V1 → NRF24L01 → NRF24L01 PA+LNA → STM32`.

Ưu tiên theo thứ tự: **(1) an toàn → (2) cân bằng ổn định → (3) nhận lệnh không dây → (4) cảm biến vật cản.**

---

## 2. Quy tắc bắt buộc

### An toàn

1. Khi khởi động, driver phải **tắt**: PA0 (EN, active-low) = HIGH trước khi bất kỳ thứ gì khác chạy.
2. Chỉ bật driver (PA0 = LOW) khi đồng thời thỏa: IMU đọc ổn định, gyro đã hiệu chuẩn, xe đang dựng thẳng trong ngưỡng, có lệnh ARM hợp lệ.
3. Tắt động cơ ngay (EN = HIGH, dừng xung STEP) khi: xe nghiêng quá ngưỡng, mất sóng NRF quá timeout, lỗi đọc IMU liên tiếp, hoặc vào trạng thái FAULT.
4. Mọi ngưỡng (góc ngã, timeout, tốc độ tối đa) để trong `App/Inc/config.h`, không hard-code rải rác.
5. Mọi bước thử động cơ ban đầu phải ghi rõ trong báo cáo: **nhấc bánh khỏi mặt đất**.
6. Không cấp 5V vào chân GPIO STM32. ECHO của HC-SR05 phải qua mạch hạ mức 5V → 3.3V.

### Cấu hình chân

7. **Không tự đổi, thêm, bớt chân** so với bảng ở mục 3. Cần đổi chân thì hỏi người dùng và nêu lý do.
8. Debug chỉ dùng **Serial Wire** (PA13/PA14). **Không** bật JTAG/SWO vì PB3 dùng cho INT của MPU6050.
9. Không sinh lại `.ioc` bằng cách đoán. Nếu file `.ioc` có sẵn, đọc nó và so với bảng chân; liệt kê mọi chỗ lệch trước khi code.
10. Kiểm tra Alternate Function bằng bảng chân trong CubeMX/datasheet, không dựa vào trí nhớ.

### Code

11. Chỉ viết code của mình trong khối `/* USER CODE BEGIN */ ... /* USER CODE END */` của code sinh ra, hoặc trong thư mục `App/`. Không sửa phần sinh tự động ngoài các khối đó.
12. ISR phải ngắn: chỉ đặt cờ/đọc thanh ghi/ghi bộ đệm. Không gọi `HAL_Delay`, không I2C/SPI blocking, không `printf` trong ISR.
13. Vòng điều khiển cân bằng chạy theo timer phần cứng với chu kỳ cố định, **không** phụ thuộc `HAL_Delay` hay thời gian vòng `while(1)`.
14. Không có lệnh chờ blocking dài (>1 ms) trong đường chạy của vòng cân bằng. Đọc cảm biến vật cản phải là máy trạng thái không chặn.
15. Mọi hàm giao tiếp ngoại vi trả về mã lỗi và có timeout. Lỗi phải đi vào failsafe, không bị bỏ qua.
16. Tên hằng/chân dùng alias trong `board_config.h` (ví dụ `STEP_L_Pin`), không dùng số cứng.

---

## 3. Bảng chân (nguồn chuẩn)

| Thiết bị | Tín hiệu | Chân | Cấu hình |
|---|---|---|---|
| TMC2208 (chung) | EN | PA0 | GPIO_Output, mặc định HIGH (tắt) |
| TMC2208 trái | STEP | PA1 | Timer AF hoặc GPIO_Output (xem mục 5) |
| TMC2208 trái | DIR | PA2 | GPIO_Output |
| TMC2208 phải | STEP | PA3 | Timer AF hoặc GPIO_Output (xem mục 5) |
| TMC2208 phải | DIR | PA4 | GPIO_Output |
| MPU6050 | I2C1 SCL / SDA | PB6 / PB7 | I2C1 |
| MPU6050 | INT | PB3 | GPIO_EXTI3, thường Rising |
| NRF24L01 | SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2, Full-Duplex Master |
| NRF24L01 | CE / CSN | PB8 / PB9 | GPIO_Output |
| NRF24L01 | IRQ (tùy chọn) | PB5 | GPIO_EXTI5 |
| VL53L0X | I2C2 SCL / SDA | PB10 / PB11 | I2C2 |
| VL53L0X | XSHUT (tùy chọn) | PC6 | GPIO_Output |
| HC-SR05 trước | TRIG / ECHO | PC0 / PC1 | Output / Input |
| HC-SR05 trái | TRIG / ECHO | PC2 / PC3 | Output / Input |
| HC-SR05 phải | TRIG / ECHO | PC4 / PC5 | Output / Input |
| Debug | SWDIO / SWCLK | PA13 / PA14 | Serial Wire |

Ghi chú về SPI2: nếu CubeMX tự gán SPI2 sang PB10/PC2/PC3, phải Reset_State các chân đó rồi gán tay PB13/14/15. Khi đó PB10 phải còn trống cho I2C2_SCL.

### Điểm cần agent tự phát hiện và báo lại

- **Xung đột EXTI:** mỗi EXTI line chỉ nối được một port. PB3 (INT MPU6050) chiếm EXTI3 nên **không dùng EXTI cho PC3**. PB5 (IRQ NRF, nếu dùng) chiếm EXTI5 nên **không dùng EXTI cho PC5**. ECHO phải đo bằng cách khác (xem mục 7).
- **Không có UART debug** trong bảng chân. Đề xuất USART1 trên PA9/PA10 (chưa dùng trong bảng), nhưng **phải được người dùng xác nhận** trước khi thêm vào `.ioc`. Trước đó, quan sát biến bằng debugger (Live Expressions).
- **MS1/MS2 của TMC2208** không có trong bảng chân, nghĩa là vi bước đặt bằng phần cứng. Phải hỏi người dùng giá trị thực tế.

---

## 4. Cấu hình CubeMX cần có

- SYS → Debug: **Serial Wire**.
- I2C1 (PB6/PB7), I2C2 (PB10/PB11): Standard hoặc Fast mode 400 kHz; xác nhận module đã có pull-up.
- SPI2: Full-Duplex Master, bắt đầu với tốc độ thấp (prescaler lớn), CPOL = Low, CPHA = 1 Edge, MSB first, 8 bit, NSS Software.
- NVIC: bật EXTI3; chỉ bật EXTI5 nếu thật sự nối IRQ của NRF.
- Timer cho vòng điều khiển: dùng timer cơ bản chưa gán chân (ví dụ **TIM6**), tạo ngắt định kỳ.
- Clock: **không đoán HSE**. Hỏi người dùng tần số thạch anh thật. Cho đến khi biết, dùng HSI. Giữ mọi clock bus trong giới hạn datasheet F405.
- Project: CMake (hoặc IDE người dùng đang dùng), bật "Generate peripheral initialization as a pair of .c/.h files" và "Keep User Code when re-generating".

---

## 5. Tạo xung STEP cho 2 động cơ

Hai động cơ cần **tần số xung độc lập**. Theo datasheet STM32F405, PA1 và PA3 đều có Timer AF (TIM2 và TIM5 cùng có CH2 trên PA1 và CH4 trên PA3). Agent phải **xác minh trong CubeMX** rồi mới dùng.

Hướng đề xuất (cần xác minh):

- PA1 → một kênh của **TIM2** (PWM, duty ≈ 50%).
- PA3 → một kênh của **TIM5** (khác timer với PA1).
- Dùng hai timer khác nhau để mỗi bánh đổi tần số độc lập. Hai kênh của cùng một timer **chung ARR**, không đổi tần số riêng được.
- Tốc độ bánh = đổi `ARR` (và `CCR` để giữ duty 50%). Tốc độ 0 = dừng timer, giữ chân STEP ở mức thấp.

Nếu xác minh không khớp, quay lại phương án phát xung bằng ngắt timer + GPIO, và báo người dùng.

Yêu cầu chung:

- `stepper_tmc2208.c` cung cấp giao diện kiểu `stepper_set_speed(motor, steps_per_sec_signed)`, `stepper_enable(bool)`, `stepper_stop()`.
- Đổi DIR **trước** khi phát xung theo chiều mới; dừng xung ngắn quanh lúc đảo chiều.
- Giới hạn tần số tối đa và độ thay đổi tần số mỗi chu kỳ (giới hạn gia tốc) để tránh mất bước.
- Hai hằng `MOTOR_L_DIR_INVERT`, `MOTOR_R_DIR_INVERT` vì hai động cơ lắp đối xứng.
- Công thức quy đổi: `steps_per_rev = 200 × MICROSTEPS` (nếu động cơ 1.8°); `MICROSTEPS` lấy từ người dùng.

---

## 6. Kiến trúc phần mềm

### Thư mục

```
App/
  Inc/  config.h  board_config.h  protocol.h
        app.h  state_machine.h  balance.h  imu_mpu6050.h
        stepper_tmc2208.h  radio_nrf24.h  tof_vl53l0x.h  ultrasonic_hcsr05.h
  Src/  app.c  state_machine.c  balance.c  imu_mpu6050.c
        stepper_tmc2208.c  radio_nrf24.c  tof_vl53l0x.c  ultrasonic_hcsr05.c
docs/
tools/    (Python/ESP32 phía điều khiển, làm sau cùng)
```

File mới trong `App/Src` phải được thêm vào `CMakeLists.txt` (khối user) hoặc cấu hình build tương ứng. Không để CubeMX sinh lại làm mất.

### Máy trạng thái

`BOOT → CALIBRATING → DISARMED → ARMED(BALANCING) → FAULT`

- `BOOT`: EN = HIGH, khởi tạo ngoại vi.
- `CALIBRATING`: xe đứng yên, đo bias gyro; hủy nếu có chuyển động lớn.
- `DISARMED`: động cơ tắt, chờ lệnh ARM và điều kiện dựng thẳng.
- `ARMED`: bật driver, chạy vòng cân bằng.
- `FAULT`: tắt động cơ; chỉ thoát bằng lệnh DISARM/reset rõ ràng.

### Vòng điều khiển (TIM6, tần số cấu hình được, bắt đầu khoảng 200 Hz)

1. Lấy mẫu IMU mới nhất, tính góc nghiêng (bộ lọc bù hoặc Kalman) với `dt` thật của timer.
2. PID/điều khiển cân bằng → tốc độ bánh mong muốn.
3. Cộng thành phần đi thẳng/quay từ lệnh NRF.
4. Giới hạn, rồi gọi `stepper_set_speed()` cho hai bánh.
5. Kiểm tra điều kiện failsafe.

Hệ số PID/đặt cực để trong `config.h` dạng placeholder `/* TODO tune */`. **Không tự bịa hệ số**; người dùng có kết quả mô phỏng riêng. Cấu trúc code phải cho phép đổi hệ số mà không sửa logic.

---

## 7. Yêu cầu theo module

### MPU6050 (`imu_mpu6050`)

- Địa chỉ 0x68 (AD0 thấp) hoặc 0x69 (AD0 cao); kiểm tra `WHO_AM_I`.
- Cấu hình DLPF và dải đo hợp lý; đọc accel + gyro một lần 14 byte.
- INT (PB3) chỉ đặt cờ "có dữ liệu mới"; việc đọc I2C làm ngoài ISR.
- Hiệu chuẩn bias gyro khi khởi động, xe đứng yên.
- Trục nghiêng và dấu phụ thuộc cách gắn module: dùng `IMU_TILT_AXIS`, `IMU_TILT_SIGN` trong config, và hỏi/ghi rõ cách xác nhận bằng thực nghiệm.
- Lỗi đọc liên tiếp quá `IMU_MAX_ERRORS` → FAULT.

### NRF24L01 (`radio_nrf24` và `protocol.h`)

- SPI2 tốc độ thấp trước; kiểm tra bằng đọc/ghi thanh ghi (ví dụ `CONFIG`, `SETUP_AW`) trước khi làm gì khác.
- Địa chỉ, kênh, tốc độ dữ liệu, độ dài payload: **hỏi người dùng**, để trong `config.h`, phải khớp ESP32 phía phát.
- `protocol.h` định nghĩa gói tin dùng chung cho STM32 và ESP32: gồm header/phiên bản, số thứ tự, lệnh (ARM/DISARM), tiến/lùi, quay, checksum hoặc CRC. Dùng `packed struct` với kích thước cố định.
- Mỗi gói hợp lệ làm mới bộ đếm timeout. Hết `RADIO_TIMEOUT_MS` → lệnh điều khiển về 0 và theo chiến lược an toàn (mục 2).
- Xử lý gói sai/trùng/ngoài thứ tự mà không làm treo.
- Nếu không nối IRQ (PB5), dùng polling trạng thái; không bật EXTI5.

### VL53L0X (`tof_vl53l0x`)

- I2C2, đọc không chặn vòng cân bằng, tần số đọc thấp (vài chục Hz).
- XSHUT (PC6) tùy chọn; chỉ dùng nếu đã gán trong `.ioc`.
- Có thể dùng thư viện ST VL53L0X API hoặc driver tối giản; ghi rõ nguồn nếu thêm thư viện ngoài.

### HC-SR05 (`ultrasonic_hcsr05`)

- Kích hoạt **lần lượt từng cảm biến**, không đồng thời.
- Xung TRIG ≥ 10 µs. Timeout echo theo tầm đo tối đa.
- ECHO là GPIO_Input thường; **không EXTI cho PC3/PC5** (xung đột, xem mục 3). Đo bằng máy trạng thái không chặn kết hợp bộ đếm chu kỳ (`DWT->CYCCNT`).
- Độ phân giải có giới hạn vì không dùng Input Capture. Ghi rõ giới hạn này; nếu cần chính xác hơn thì đề xuất đổi ECHO sang chân có Timer AF, không tự đổi.
- Dữ liệu vật cản chỉ giảm tốc/dừng, không được phá vòng cân bằng.

---

## 8. Quy trình làm việc theo giai đoạn

Hoàn thành và báo cáo từng giai đoạn trước khi sang giai đoạn sau.

| GĐ | Nội dung | Điều kiện hoàn thành |
|---|---|---|
| 0 | Đọc tài liệu, đối chiếu `.ioc` với mục 3, liệt kê chỗ lệch và câu hỏi mở | Có danh sách, người dùng xác nhận |
| 1 | Khung project, clock, GPIO, EN = HIGH khi boot | Build sạch, chân đúng bảng |
| 2 | MPU6050 + ước lượng góc | Góc đúng dấu và đúng tỷ lệ khi nghiêng tay |
| 3 | Stepper: STEP/DIR/EN, giới hạn tốc độ | Hai bánh quay độc lập, bánh **nhấc khỏi đất** |
| 4 | Vòng cân bằng + máy trạng thái + failsafe | Cân bằng ngắn, tự tắt khi ngã |
| 5 | NRF24L01 nhận lệnh + timeout | Mất sóng thì dừng đúng thời gian |
| 6 | VL53L0X, HC-SR05 | Không ảnh hưởng chu kỳ vòng điều khiển |
| 7 | ESP32 + Python phía điều khiển (tùy chọn, làm sau cùng) | Gửi/nhận khớp `protocol.h` |

Cuối mỗi giai đoạn, báo cáo ngắn gồm: file đã đổi, kết quả build, phần nào **cần thử trên phần cứng**, và câu hỏi còn mở. Không nói "đã chạy được" cho phần chưa chạy trên phần cứng thật.

---

## 9. Build

Giả định mặc định: CMake + Ninja + `arm-none-eabi-gcc`, nạp bằng ST-Link. Nếu người dùng dùng STM32CubeIDE thì đổi lệnh cho phù hợp.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Quy ước: code phải build không cảnh báo mới (`-Wall -Wextra`). Không commit thư mục build.

---

## 10. Câu hỏi agent phải hỏi người dùng (không tự đoán)

1. Tần số thạch anh HSE của board (hoặc dùng HSI).
2. MS1/MS2 của TMC2208 đang đặt vi bước bao nhiêu; dòng `Vref` đã chỉnh chưa.
3. Đường kính bánh, góc bước động cơ, điện áp pin.
4. Cách gắn MPU6050 (trục nào hướng phía trước/hướng lên).
5. Có đồng ý thêm UART debug (đề xuất USART1 PA9/PA10) không.
6. Có nối IRQ của NRF24L01 vào PB5 không.
7. Kênh, địa chỉ, tốc độ, payload của NRF24L01 (khớp phía ESP32).
8. Có dùng XSHUT của VL53L0X (PC6) không.
9. Hệ số PID/đặt cực từ mô phỏng.
10. Chiều quay đúng của từng bánh khi bánh nhấc lên.

---

## 11. Những điều không được làm

- Không đổi chân hoặc bật JTAG/SWO để "cho tiện".
- Không dùng EXTI cho PC3 hoặc PC5.
- Không chạy động cơ khi chưa qua điều kiện ARM, hoặc bỏ qua failsafe để "test nhanh".
- Không bịa tần số HSE, hệ số PID, địa chỉ NRF, vi bước.
- Không chặn vòng cân bằng bằng đo siêu âm hay `HAL_Delay`.
- Không cấp 5V vào GPIO, không cấp 5V cho NRF24L01.
- Không tuyên bố hoàn thành phần chưa kiểm tra trên phần cứng.
