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
   - Nút **ADAS: ON (An toàn)** / **ADAS: OFF (Chế độ Đua)** hoặc phím **F2** bật/tắt tránh vật cản và phanh mép bàn; trạng thái được gửi khi kết nối và mỗi lần đổi. Tắt ADAS sẽ bỏ qua các phản ứng vật cản nhưng không vô hiệu hóa tự DISARM khi xe ngã.
   - **X** / Nút **ARM XE**: Kích hoạt cân bằng (chỉ bật khi xe dựng thẳng).
   - **Z** / Nút **DISARM**: Ngắt động cơ an toàn.
   - **E** / Nút **ESTOP**: Dừng khẩn cấp ngay lập tức.
   - **W/S/A/D** và các nút hướng hoạt động theo kiểu nhấn-giữ; mức lệnh lấy từ các thanh trượt tốc độ và độ nhạy rẽ.
   - Lệnh được gia hạn mỗi 100 ms khi giữ; nhấn nhanh Tiến/Lùi giữ xung tối thiểu 450 ms, còn Xoay trái/phải giữ xung tối thiểu 220 ms.
   - **Space** / **R** hoặc nút **STOP**: dừng tức thì (về 0).
   - Thanh trượt tùy chỉnh mức tốc độ tiến/lùi và độ nhạy rẽ.
   - Tay cầm USB generic: bật **TAY CẦM USB** trong GCS; joystick trái (trục Y) điều khiển tiến/lùi, joystick phải (trục X) điều khiển rẽ. Có deadzone 12%; tốc độ và độ nhạy lấy theo hai thanh trượt.
   - Nút 1 (button index 0) toggle ARM/DISARM theo trạng thái telemetry; chỉ ARM khi xe báo `DISARMED`. Nút 2 STOP, nút 3 ESTOP, nút 4 chưa gán.
   - Lệnh DRIVE được gửi định kỳ khi tay cầm hoạt động. Khi rút tay cầm mà Serial GCS vẫn nối, GCS gửi `DRIVE 0 0`, giữ trạng thái ARM hiện có và cho phép chuyển sang webcam/phím; nếu Serial GCS mất thì không thể gửi lệnh. Chuyển động từ joystick bị khóa khi trạng thái xe chưa là `ARMED`.
   - Bật Webcam lái tay khi đang bật tay cầm sẽ chuyển nguồn điều khiển: dừng lệnh joystick và tắt tay cầm trước khi mở webcam.
   - Trên Windows, webcam thử backend DirectShow trước rồi fallback sang backend OpenCV mặc định; lỗi mở camera/đọc frame được in traceback ở terminal và báo trong GCS.

5. **Terminal / Console**:
   - Hiển thị lịch sử gói tin TX/RX.
   - Ô nhập lệnh trực tiếp (ví dụ: `DRIVE 200 0`, `ARM`, `DISARM`, `ESTOP`).

---

## 2. Hướng Dẫn Khởi Động

Cài các thư viện Python từ thư mục `App_python` trước khi chạy:
```bash
python -m pip install -r requirements.txt
```

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
