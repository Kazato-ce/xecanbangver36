# Ứng Dụng Python Ground Control Station & Vision Stream (ESP32-CAM)

Ứng dụng máy trạm điều khiển trên máy tính dành cho dự án **Xe Cân Bằng 2 Bánh STM32F405**.

---

## 1. Tính Năng Chính

1. **Khung Video Stream ESP32-CAM (Hoàn toàn độc lập)**:
   - Nhận video stream trực tiếp qua mạng WiFi/HTTP từ module ESP32-CAM (MJPEG stream hoặc JPEG snapshot).
   - Không phụ thuộc, không nối dây hay can thiệp vào các linh kiện cảm biến bên dưới.
   - Hỗ trợ đổi URL linh hoạt (`http://192.168.4.2/stream`, `http://192.168.1.100:81/stream`, hoặc webcam `0` để test).
   - Nút **Chụp ảnh (Snapshot)** lưu ngay khung hình vào thư mục `snapshots/`.
   - Hiển thị FPS thực tế và độ phân giải hình ảnh.

2. **Giao tiếp không dây qua ESP32 DevKit V1 (NRF24L01 Bridge)**:
   - Kết nối cổng COM USB Serial ở tốc độ **115200 baud**.
   - Tự động nhận diện danh sách cổng COM đang cắm trên Windows.
   - Driver Serial tùy biến: tương thích cả khi máy đã cài `pyserial` lẫn chạy trực tiếp qua Windows API `ctypes` không cần cài thêm thư viện phụ thuộc.

3. **Bảng Giám Sát Cảm Biến Real-Time (Telemetry)**:
   - Trạng thái xe: `BOOT`, `CALIBRATING`, `DISARMED`, `ARMED`, `FAULT`.
   - Góc nghiêng Pitch (°): đo từ trục X của MPU6050, kèm thước đo cân bằng trực quan.
   - Khoảng cách vật cản (cm): từ cảm biến ToF VL53L0X và siêu âm HC-SR05.
   - Điện áp Pin 4S (V): giám sát mức pin an toàn (14.8V định danh).

4. **Bảng Điều Khiển & Phím Tắt Nhanh**:
   - **X** / Nút **ARM XE**: Kích hoạt cân bằng (chỉ bật khi xe dựng thẳng).
   - **Z** / Nút **DISARM**: Ngắt động cơ an toàn.
   - **E** / Nút **ESTOP**: Dừng khẩn cấp ngay lập tức.
   - **W**: Tiến (+150 tốc độ)
   - **S**: Lùi (-150 tốc độ)
   - **A**: Rẽ trái (-200 góc lái)
   - **D**: Rẽ phải (+200 góc lái)
   - **Space** / **R**: Phanh dừng tại chỗ (về 0).
   - Thanh trượt tùy chỉnh bước tốc độ và độ nhạy rẽ.

5. **Terminal / Console**:
   - Hiển thị lịch sử gói tin TX/RX.
   - Ô nhập lệnh trực tiếp (ví dụ: `DRIVE 200 0`, `ARM`, `DISARM`, `ESTOP`).

---

## 2. Hướng Dẫn Khởi Động

### Cách 1: Chạy bằng file `.bat` (Tiện nhất trên Windows)
- Nhấp đúp chuột vào file: `run_app.bat`

### Cách 2: Chạy qua Terminal / Command Prompt
```bash
cd tools/python_ground_station
python app.py
```

---

## 3. Cách Kết Nối Với ESP32-CAM

Nếu bạn nạp code mẫu `CameraWebServer` của Arduino IDE cho ESP32-CAM:
1. **Trường hợp ESP32-CAM phát WiFi (Access Point)**:
   - Máy tính kết nối vào WiFi của ESP32-CAM (ví dụ `ESP32-CAM-AP`).
   - Địa chỉ URL mặc định thường là: `http://192.168.4.2/stream` hoặc `http://192.168.4.1:81/stream`.
2. **Trường hợp ESP32-CAM kết nối cùng mạng WiFi gia đình/router**:
   - Mở Serial Monitor của Arduino khi ESP32-CAM khởi động để xem IP (ví dụ `192.168.1.150`).
   - Nhập vào ô URL: `http://192.168.1.150:81/stream` (port 81 là cổng stream mặc định của thư viện ESP32 CameraWebServer).
3. Bấm nút **▶ MỞ STREAM**.

---

## 4. Quy Tắc An Toàn (Bắt Buộc theo AGENTS.md)

> [!WARNING]
> Trong mọi lần thử nghiệm ban đầu khi bật chế độ ARM hoặc điều khiển động cơ:
> **LUÔN LUÔN NHẤC 2 BÁNH XE KHỎI MẶT ĐẤT** (kê xe trên bệ đỡ) để kiểm tra chiều quay động cơ và phản hồi cân bằng trước khi đặt xe xuống sàn!
