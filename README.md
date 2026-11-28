# XE CÂN BẰNG 2 BÁNH STM32F405 — XECANBANGFINAL

Dự án firmware và hệ thống điều khiển hoàn chỉnh cho xe 2 bánh tự cân bằng:
- **MCU:** STM32F405RGT6 (LQFP64, 168 MHz SYSCLK, HSE 8 MHz).
- **Động cơ & Driver:** 2 x Động cơ bước NEMA 17 + 2 x Driver TMC2208 (chế độ STEP/DIR, 1/16 microstepping).
- **Cảm biến góc nghiêng:** MPU6050 (I2C1 Fast Mode 400 kHz, trục nghiêng X, ngắt INT PB3).
- **Truyền nhận không dây:** NRF24L01+ PA+LNA (SPI2, Kênh 76 @ 1 Mbps).
- **Trạm điều khiển máy tính:** Python Ground Control Station (`App_python/`) có tích hợp stream video từ ESP32-CAM và điều khiển phím tắt bàn phím.
- **Trạm trung gian NRF24:** ESP32 DevKit V1 (`ESP32_DevKit_NRF24/`).
- **Web Dashboard & PID Tuning:** ESP32 Super Mini (`ESP32_SuperMini_Web/`) giao tiếp UART (PA9/PA10).

---

## Cấu Trúc Thư Mục

```
XECANBANGFINAL/
├── App/                  # Source code ứng dụng cân bằng trên STM32 (HAL)
│   ├── Inc/              # Header files: config.h, board_config.h, protocol.h, balance.h, ...
│   └── Src/              # C files: balance.c, imu_mpu6050.c, stepper_tmc2208.c, ...
├── App_python/           # Ứng dụng Python Ground Control Station & Vision Camera
├── ESP32_DevKit_NRF24/   # Firmware Arduino cho ESP32 DevKit V1 (NRF24L01 Bridge)
├── ESP32_SuperMini_Web/  # Firmware Arduino cho ESP32 Super Mini (Web Dashboard PID)
├── Core/                 # CubeMX generated HAL core
├── Drivers/              # STM32F4xx HAL Driver & CMSIS
├── CMakeLists.txt        # Cấu hình biên dịch CMake
└── README.md
```

---

## Hướng Dẫn Biên Dịch STM32

Yêu cầu: `arm-none-eabi-gcc`, `cmake`, `ninja`.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

File thực thi được tạo tại `build/Debug/XECANBANGFINAL.elf` (kèm `.hex` và `.bin`).

---

## Ứng Dụng Python Ground Control Station

Vào thư mục `App_python/`:
```bash
cd App_python
pip install -r requirements.txt
python app.py
```
Hoặc nhấp đúp file `run_app.bat` trên Windows.
