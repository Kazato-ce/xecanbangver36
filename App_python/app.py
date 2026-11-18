#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  MÁY TRẠM ĐIỀU KHIỂN & GIÁM SÁT XE CÂN BẰNG 2 BÁNH STM32F405
  (Ground Control Station - GCS)
================================================================================
Tác vụ chính:
 1. Điều khiển xe qua ESP32 DevKit V1 (NRF24L01 Bridge @ 115200 baud).
 2. Hiển thị Telemetry: Góc nghiêng (Pitch), Trạng thái (ARM/DISARM/FAULT),
    Khoảng cách vật cản (ToF/Siêu âm), Điện áp Pin 4S.
 3. Tích hợp khung stream video từ link ESP32-CAM (hoàn toàn độc lập,
    truyền qua HTTP MJPEG/RTSP stream không cần dây nối).
 4. Bàn phím điều khiển nhanh: W, A, S, D, Space, X (ARM), Z (DISARM), E (ESTOP).
================================================================================
"""

import os
import sys
import time
import re
import datetime
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import cv2
import numpy as np
from PIL import Image, ImageTk

# Import driver serial tùy biến (tương thích cả pyserial lẫn Windows ctypes gốc)
try:
    from tools.python_ground_station.serial_driver import SerialClient, list_com_ports
    from tools.python_ground_station.video_worker import VideoStreamWorker
except ImportError:
    # Nếu chạy trực tiếp từ thư mục tools/python_ground_station
    from serial_driver import SerialClient, list_com_ports
    from video_worker import VideoStreamWorker


# ──────────────────────────────────────────────────────────────────────────────
# BẢNG MÀU GIAO DIỆN HIỆN ĐẠI (DARK THEME)
# ──────────────────────────────────────────────────────────────────────────────
THEME = {
    "bg": "#11111b",             # Nền tối sâu
    "panel": "#181825",          # Khung thẻ panel
    "card": "#1e1e2e",           # Thẻ nội dung
    "card_alt": "#252538",       # Thẻ nhấn
    "border": "#313244",         # Viền
    "text": "#cdd6f4",           # Chữ chính sáng
    "subtext": "#a6adc8",        # Chữ phụ
    "primary": "#89b4fa",        # Xanh dương điểm nhấn
    "success": "#a6e3a1",        # Xanh lá thành công
    "warning": "#fab387",        # Cam cảnh báo
    "danger": "#f38ba8",         # Đỏ nguy hiểm
    "accent": "#cba6f7",         # Tím
    "console_bg": "#11111b",     # Nền terminal
}


class GroundStationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Xe Cân Bằng STM32F405 - Ground Control Station & Camera")
        self.root.geometry("1300x820")
        self.root.minsize(1050, 700)
        self.root.configure(bg=THEME["bg"])

        # Trạng thái Serial
        self.serial_client = None
        self.serial_thread = None
        self.serial_running = False

        # Trạng thái Video
        self.video_worker = None
        self.current_frame = None
        self.photo_img = None
        self.fps_display = 0.0

        # Biến trạng thái xe
        self.robot_state_str = "DISCONNECTED"
        self.pitch_deg = 0.0
        self.dist_cm = 0
        self.bat_v = 0.0
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.tx_count = 0
        self.rx_count = 0

        # Biến chẩn đoán phần cứng (Hardware Health Diagnostics)
        self.diag_mpu_ok = False
        self.diag_nrf_ok = False
        self.diag_spm_ok = False
        self.diag_upright = False
        self.diag_imu_err = 0

        # Thiết lập giao diện
        self._setup_styles()
        self._build_header()
        self._build_main_layout()
        self._build_status_bar()

        # Đăng ký phím tắt điều khiển bàn phím
        self._bind_keyboard_controls()

        # Quét cổng COM ban đầu
        self.refresh_com_ports()

        # Bắt đầu vòng lặp cập nhật giao diện định kỳ
        self.root.after(100, self._ui_periodic_update)

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("TLabel", background=THEME["card"], foreground=THEME["text"], font=("Segoe UI", 10))
        style.configure("TFrame", background=THEME["bg"])
        style.configure("Card.TFrame", background=THEME["card"], relief="flat")
        style.configure("TCombobox", fieldbackground=THEME["card_alt"], background=THEME["border"],
                        foreground=THEME["text"], arrowcolor=THEME["primary"])

    def _build_header(self):
        header = tk.Frame(self.root, bg=THEME["panel"], height=55, padx=20, pady=8)
        header.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(
            header,
            text="⚡ STM32F405 TWO-WHEEL SELF-BALANCING ROBOT — GCS & VISION",
            font=("Segoe UI", 13, "bold"),
            fg=THEME["primary"],
            bg=THEME["panel"]
        )
        title_lbl.pack(side=tk.LEFT)

        sub_lbl = tk.Label(
            header,
            text="[200Hz Loop | MPU6050 | TMC2208 1/16 | NRF24L01 | ESP32-CAM]",
            font=("Consolas", 10),
            fg=THEME["subtext"],
            bg=THEME["panel"]
        )
        sub_lbl.pack(side=tk.LEFT, padx=15)

        self.lbl_system_clock = tk.Label(
            header,
            text="",
            font=("Consolas", 10),
            fg=THEME["accent"],
            bg=THEME["panel"]
        )
        self.lbl_system_clock.pack(side=tk.RIGHT)

    def _build_main_layout(self):
        main_pane = tk.Frame(self.root, bg=THEME["bg"], padx=10, pady=10)
        main_pane.pack(fill=tk.BOTH, expand=True)

        # Cột trái: Khung Video Stream ESP32-CAM (chiếm khoảng 55%)
        left_col = tk.Frame(main_pane, bg=THEME["bg"], padx=5, pady=5)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Cột phải: Điều khiển & Telemetry & Console (chiếm khoảng 45%)
        right_col = tk.Frame(main_pane, bg=THEME["bg"], width=520, padx=5, pady=5)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)

        self._build_video_panel(left_col)
        self._build_serial_panel(right_col)
        self._build_telemetry_panel(right_col)
        self._build_controls_panel(right_col)
        self._build_console_panel(right_col)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 1: KHUNG STREAM VIDEO TỪ LINK ESP32-CAM
    # ──────────────────────────────────────────────────────────────────────────
    def _build_video_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📹 ESP32-CAM Video Stream (Độc lập qua HTTP/MJPEG) ",
            font=("Segoe UI", 11, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.BOTH, expand=True)

        # Thanh nhập URL Stream
        url_frame = tk.Frame(card, bg=THEME["card"])
        url_frame.pack(fill=tk.X, pady=(0, 8))

        tk.Label(url_frame, text="URL Stream:", fg=THEME["text"], bg=THEME["card"], font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.cb_stream_url = ttk.Combobox(
            url_frame,
            values=[
                "http://192.168.4.2/stream",           # ESP32-CAM phát WiFi Access Point
                "http://192.168.4.2:81/stream",
                "http://192.168.1.100:81/stream",       # ESP32-CAM kết nối WiFi nhà
                "http://192.168.1.50/stream",
                "http://192.168.4.2/capture",          # Chế độ chụp ảnh snapshot
                "0"                                    # Webcam laptop/PC (thử nghiệm)
            ],
            width=36,
            font=("Consolas", 9)
        )
        self.cb_stream_url.set("http://192.168.4.2/stream")
        self.cb_stream_url.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        self.btn_toggle_stream = tk.Button(
            url_frame,
            text="▶ MỞ STREAM",
            font=("Segoe UI", 9, "bold"),
            bg=THEME["primary"],
            fg="#11111b",
            activebackground=THEME["accent"],
            relief=tk.FLAT,
            padx=12,
            pady=3,
            command=self.toggle_video_stream
        )
        self.btn_toggle_stream.pack(side=tk.LEFT, padx=4)

        self.btn_snapshot = tk.Button(
            url_frame,
            text="📷 Chụp Ảnh",
            font=("Segoe UI", 9),
            bg=THEME["card_alt"],
            fg=THEME["text"],
            relief=tk.FLAT,
            padx=8,
            pady=3,
            command=self.save_snapshot
        )
        self.btn_snapshot.pack(side=tk.LEFT, padx=4)

        # Khung hiển thị Video Canvas
        self.video_canvas = tk.Canvas(
            card,
            bg="#0b0b12",
            highlightthickness=1,
            highlightbackground=THEME["border"]
        )
        self.video_canvas.pack(fill=tk.BOTH, expand=True)
        self.video_canvas.bind("<Configure>", self._on_canvas_resize)

        # Vẽ hình nền chờ ban đầu
        self._draw_video_placeholder("Chưa mở stream video ESP32-CAM\nNhập URL stream (ví dụ http://192.168.4.2/stream) và bấm 'MỞ STREAM'")

        # Thanh trạng thái video dưới đáy
        info_bar = tk.Frame(card, bg=THEME["card_alt"], height=26, padx=8, pady=3)
        info_bar.pack(fill=tk.X, pady=(6, 0))

        self.lbl_stream_status = tk.Label(
            info_bar,
            text="● STREAM TẮT",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card_alt"]
        )
        self.lbl_stream_status.pack(side=tk.LEFT)

        self.lbl_fps = tk.Label(
            info_bar,
            text="FPS: 0.0 | Độ phân giải: --x--",
            font=("Consolas", 9),
            fg=THEME["subtext"],
            bg=THEME["card_alt"]
        )
        self.lbl_fps.pack(side=tk.RIGHT)

    def _draw_video_placeholder(self, message):
        self.video_canvas.delete("all")
        w = max(self.video_canvas.winfo_width(), 400)
        h = max(self.video_canvas.winfo_height(), 300)
        self.video_canvas.create_text(
            w // 2, h // 2,
            text=f"📷\n\n{message}",
            fill=THEME["subtext"],
            font=("Segoe UI", 11),
            justify=tk.CENTER
        )

    def _on_canvas_resize(self, event):
        if self.video_worker is None or not self.video_worker.running:
            self._draw_video_placeholder("Chưa mở stream video ESP32-CAM\nNhập URL stream và bấm 'MỞ STREAM'")

    def toggle_video_stream(self):
        if self.video_worker and self.video_worker.running:
            self.stop_video_stream()
        else:
            self.start_video_stream()

    def start_video_stream(self):
        url = self.cb_stream_url.get().strip()
        if not url:
            messagebox.showwarning("Cảnh báo", "Vui lòng nhập đường dẫn URL Stream của ESP32-CAM!")
            return

        self.btn_toggle_stream.config(text="⏹ DỪNG STREAM", bg=THEME["danger"])
        self.lbl_stream_status.config(text="● ĐANG KẾT NỐI...", fg=THEME["warning"])

        self.video_worker = VideoStreamWorker(
            source_url=url,
            on_frame=self._on_video_frame,
            on_status=self._on_video_status
        )
        self.video_worker.start()

    def stop_video_stream(self):
        if self.video_worker:
            self.video_worker.stop()
            self.video_worker = None
        self.btn_toggle_stream.config(text="▶ MỞ STREAM", bg=THEME["primary"])
        self.lbl_stream_status.config(text="● STREAM ĐÃ DỪNG", fg=THEME["subtext"])
        self.lbl_fps.config(text="FPS: 0.0 | Độ phân giải: --x--")
        self._draw_video_placeholder("Stream đã dừng.\nBấm 'MỞ STREAM' để kết nối lại.")

    def _on_video_frame(self, frame_bgr, fps):
        self.current_frame = frame_bgr
        self.fps_display = fps

    def _on_video_status(self, msg, is_ok):
        def _update():
            if is_ok:
                self.lbl_stream_status.config(text="● LIVE STREAM", fg=THEME["success"])
            else:
                self.lbl_stream_status.config(text=f"● {msg}", fg=THEME["danger"] if "Lỗi" in msg else THEME["warning"])
        self.root.after(0, _update)

    def save_snapshot(self):
        if self.current_frame is None:
            messagebox.showinfo("Thông báo", "Chưa có khung hình video nào để lưu!")
            return

        snap_dir = os.path.join(os.path.dirname(__file__), "snapshots")
        os.makedirs(snap_dir, exist_ok=True)
        fname = datetime.datetime.now().strftime("snapshot_%Y%m%d_%H%M%S.png")
        path = os.path.join(snap_dir, fname)

        cv2.imwrite(path, self.current_frame)
        self.log_console(f"[Vision] Đã lưu ảnh chụp: {fname}", "INFO")
        messagebox.showinfo("Thành công", f"Đã lưu ảnh snapshot vào:\n{path}")

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 2: KẾT NỐI ESP32 DEVKIT V1 (NRF24 BRIDGE)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_serial_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📡 Cổng Kết Nối ESP32 DevKit V1 (NRF24L01 Bridge) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=6
        )
        card.pack(fill=tk.X, pady=(0, 8))

        row = tk.Frame(card, bg=THEME["card"])
        row.pack(fill=tk.X)

        tk.Label(row, text="Cổng COM:", fg=THEME["text"], bg=THEME["card"]).pack(side=tk.LEFT)

        self.cb_com_ports = ttk.Combobox(row, width=12, state="readonly")
        self.cb_com_ports.pack(side=tk.LEFT, padx=6)

        btn_refresh = tk.Button(
            row, text="🔄 Quét", bg=THEME["card_alt"], fg=THEME["text"],
            relief=tk.FLAT, padx=6, command=self.refresh_com_ports
        )
        btn_refresh.pack(side=tk.LEFT, padx=2)

        tk.Label(row, text="Baud: 115200", fg=THEME["subtext"], bg=THEME["card"]).pack(side=tk.LEFT, padx=8)

        btn_ping = tk.Button(
            row, text="⚡ Ping Sóng", bg=THEME["card_alt"], fg=THEME["text"],
            relief=tk.FLAT, padx=8, command=self.ping_robot
        )
        btn_ping.pack(side=tk.LEFT, padx=4)

        self.btn_connect = tk.Button(
            row,
            text="KẾT NỐI",
            font=("Segoe UI", 9, "bold"),
            bg=THEME["success"],
            fg="#11111b",
            relief=tk.FLAT,
            padx=14,
            pady=2,
            command=self.toggle_serial
        )
        self.btn_connect.pack(side=tk.RIGHT)

        status_row = tk.Frame(card, bg=THEME["card"])
        status_row.pack(fill=tk.X, pady=(4, 0))

        self.lbl_serial_badge = tk.Label(
            status_row,
            text="● Chưa kết nối Serial",
            font=("Segoe UI", 9),
            fg=THEME["subtext"],
            bg=THEME["card"]
        )
        self.lbl_serial_badge.pack(side=tk.LEFT)

        self.lbl_rf_link = tk.Label(
            status_row,
            text="● Sóng NRF: Chờ kết nối COM...",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card"]
        )
        self.lbl_rf_link.pack(side=tk.RIGHT)

    def ping_robot(self):
        if not self.serial_running or not self.serial_client:
            self.log_console("[Ping] Chưa kết nối cổng COM của ESP32!", "WARN")
            return
        self.send_command("PING")
        self.log_console("[Ping] Đã gửi lệnh PING kiểm tra sóng sang ESP32 DevKit...", "INFO")

    def refresh_com_ports(self):
        ports = list_com_ports()
        self.cb_com_ports["values"] = ports
        if ports:
            self.cb_com_ports.current(0)
        self.log_console(f"[Serial] Tìm thấy các cổng: {ports}", "INFO")

    def toggle_serial(self):
        if self.serial_running:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.cb_com_ports.get()
        if not port:
            messagebox.showwarning("Cảnh báo", "Vui lòng chọn cổng COM của ESP32 DevKit V1!")
            return

        try:
            self.serial_client = SerialClient(port=port, baudrate=115200, timeout=0.1)
            self.serial_client.open()
            self.serial_running = True

            self.btn_connect.config(text="NGẮT KẾT NỐI", bg=THEME["danger"], fg="#ffffff")
            self.lbl_serial_badge.config(text=f"● Đã kết nối: {port} @ 115200", fg=THEME["success"])
            self.log_console(f"[Serial] Đã mở cổng {port} thành công!", "SUCCESS")

            # Bật luồng đọc dữ liệu nền
            self.serial_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self.serial_thread.start()

        except Exception as e:
            self.serial_running = False
            self.lbl_serial_badge.config(text=f"● Lỗi mở cổng {port}", fg=THEME["danger"])
            self.log_console(f"[Serial Lỗi] {e}", "ERROR")
            messagebox.showerror("Lỗi kết nối", f"Không thể mở cổng {port}:\n{e}")

    def disconnect_serial(self):
        self.serial_running = False
        if self.serial_client:
            self.serial_client.close()
            self.serial_client = None

        self.btn_connect.config(text="KẾT NỐI", bg=THEME["success"], fg="#11111b")
        self.lbl_serial_badge.config(text="● Đã ngắt kết nối", fg=THEME["subtext"])
        self.log_console("[Serial] Đã đóng cổng kết nối.", "WARN")

    def _serial_reader_loop(self):
        buffer = ""
        while self.serial_running and self.serial_client and self.serial_client.is_open:
            try:
                data = self.serial_client.read(128)
                if data:
                    text = data.decode("utf-8", errors="ignore")
                    buffer += text
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.rx_count += 1
                            self.root.after(0, self._process_rx_line, line)
                else:
                    time.sleep(0.02)
            except Exception as e:
                time.sleep(0.1)

    def _process_rx_line(self, line):
        self.log_console(f"RX: {line}", "RX")

        # Cập nhật trạng thái kết nối từ thông báo của ESP32 DevKit
        if "KẾT NỐI TỐT" in line or "Sóng NRF OK" in line:
            self.diag_nrf_ok = True
            if hasattr(self, 'lbl_rf_link'):
                self.lbl_rf_link.config(text="● SÓNG NRF: ĐÃ BẮT SÓNG XE (ACK OK)", fg=THEME["success"])
            if self.robot_state_str in ("DISCONNECTED", "WAITING...", "NO_RADIO"):
                self.robot_state_str = "CONNECTED"
            self._update_telemetry_widgets()
        elif "CHỜ TÍN HIỆU XE" in line or "Chưa nhận được ACK" in line:
            self.diag_nrf_ok = False
            if hasattr(self, 'lbl_rf_link'):
                self.lbl_rf_link.config(text="● SÓNG NRF: ĐANG TÌM XE (KIỂM TRA NGUỒN STM32)", fg=THEME["warning"])
            if self.robot_state_str == "CONNECTED":
                self.robot_state_str = "WAITING..."
            self._update_telemetry_widgets()
        elif "ACK: ARM" in line:
            self.robot_state_str = "ARMED"
            self._update_telemetry_widgets()
        elif "ACK: DISARM" in line:
            self.robot_state_str = "DISARMED"
            self._update_telemetry_widgets()
        elif "ACK: ESTOP" in line:
            self.robot_state_str = "FAULT"
            self._update_telemetry_widgets()
        elif "PONG:" in line:
            if "RADIO_OK=1" in line:
                if hasattr(self, 'lbl_rf_link'):
                    self.lbl_rf_link.config(text="● NRF24 ESP32: SẴN SÀNG", fg=THEME["primary"])
            else:
                if hasattr(self, 'lbl_rf_link'):
                    self.lbl_rf_link.config(text="● NRF24: LỖI PHẦN CỨNG", fg=THEME["danger"])

        # Phân tích dòng Telemetry từ STM32 gửi qua NRF24:
        # Ví dụ: "[STM32 Telemetry] State=3, Pitch=1.25 deg, Dist=45 cm, Bat=15.80 V, MPU=1, NRF=1, SPM=1, Upright=1, MotorEN=1, ImuErr=0"
        if "Telemetry" in line or "Pitch=" in line:
            self.diag_nrf_ok = True
            m_state = re.search(r"State=(\d+)", line)
            m_pitch = re.search(r"Pitch=([\+\-\d\.]+)", line)
            m_dist  = re.search(r"Dist=(\d+)", line)
            m_bat   = re.search(r"Bat=([\d\.]+)", line)
            m_mpu   = re.search(r"MPU=(\d+)", line)
            m_nrf   = re.search(r"NRF=(\d+)", line)
            m_spm   = re.search(r"SPM=(\d+)", line)
            m_upr   = re.search(r"Upright=(\d+)", line)
            m_err   = re.search(r"ImuErr=(\d+)", line)

            if m_state:
                st_val = int(m_state.group(1))
                state_map = {0: "BOOT", 1: "CALIBRATING", 2: "DISARMED", 3: "ARMED", 4: "FAULT"}
                self.robot_state_str = state_map.get(st_val, f"STATE_{st_val}")

            if m_pitch:
                try:
                    self.pitch_deg = float(m_pitch.group(1))
                except ValueError:
                    pass

            if m_dist:
                try:
                    self.dist_cm = int(m_dist.group(1))
                except ValueError:
                    pass

            if m_bat:
                try:
                    self.bat_v = float(m_bat.group(1))
                except ValueError:
                    pass

            if m_mpu: self.diag_mpu_ok = (int(m_mpu.group(1)) == 1)
            if m_nrf: self.diag_nrf_ok = (int(m_nrf.group(1)) == 1)
            if m_spm: self.diag_spm_ok = (int(m_spm.group(1)) == 1)
            if m_upr: self.diag_upright = (int(m_upr.group(1)) == 1)
            if m_err: self.diag_imu_err = int(m_err.group(1))

            self._update_telemetry_widgets()

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 3: TELEMETRY & CẢM BIẾN XE CÂN BẰNG
    # ──────────────────────────────────────────────────────────────────────────
    def _build_telemetry_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📊 Telemetry & Cảm Biến Real-Time ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.X, pady=(0, 8))

        grid = tk.Frame(card, bg=THEME["card"])
        grid.pack(fill=tk.X)

        # Ô 1: Trạng thái xe
        f_st = tk.Frame(grid, bg=THEME["card_alt"], padx=8, pady=6, bd=1, relief=tk.SOLID)
        f_st.grid(row=0, column=0, sticky="nsew", padx=3, pady=3)
        tk.Label(f_st, text="TRẠNG THÁI", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_telem_state = tk.Label(f_st, text="DISCONNECTED", font=("Segoe UI", 11, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"])
        self.lbl_telem_state.pack()

        # Ô 2: Góc nghiêng Pitch (MPU6050 Trục X)
        f_pitch = tk.Frame(grid, bg=THEME["card_alt"], padx=8, pady=6, bd=1, relief=tk.SOLID)
        f_pitch.grid(row=0, column=1, sticky="nsew", padx=3, pady=3)
        tk.Label(f_pitch, text="GÓC PITCH (°)", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_telem_pitch = tk.Label(f_pitch, text="0.00°", font=("Consolas", 12, "bold"), fg=THEME["success"], bg=THEME["card_alt"])
        self.lbl_telem_pitch.pack()

        # Ô 3: Khoảng cách vật cản (ToF / Siêu âm)
        f_dist = tk.Frame(grid, bg=THEME["card_alt"], padx=8, pady=6, bd=1, relief=tk.SOLID)
        f_dist.grid(row=0, column=2, sticky="nsew", padx=3, pady=3)
        tk.Label(f_dist, text="VẬT CẢN TRƯỚC", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_telem_dist = tk.Label(f_dist, text="-- cm", font=("Consolas", 12, "bold"), fg=THEME["primary"], bg=THEME["card_alt"])
        self.lbl_telem_dist.pack()

        # Ô 4: Điện áp Pin 4S
        f_bat = tk.Frame(grid, bg=THEME["card_alt"], padx=8, pady=6, bd=1, relief=tk.SOLID)
        f_bat.grid(row=0, column=3, sticky="nsew", padx=3, pady=3)
        tk.Label(f_bat, text="PIN 4S (14.8V)", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_telem_bat = tk.Label(f_bat, text="-- V", font=("Consolas", 12, "bold"), fg=THEME["warning"], bg=THEME["card_alt"])
        self.lbl_telem_bat.pack()

        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(2, weight=1)
        grid.columnconfigure(3, weight=1)

        # Hàng chẩn đoán phần cứng chi tiết (Hardware Health Diagnostics)
        diag_frame = tk.Frame(card, bg=THEME["card"], pady=4)
        diag_frame.pack(fill=tk.X, pady=(4, 0))

        # Ô D1: MPU6050
        f_mpu = tk.Frame(diag_frame, bg=THEME["card_alt"], padx=6, pady=4, bd=1, relief=tk.SOLID)
        f_mpu.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_mpu, text="🩺 MPU6050", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_diag_mpu = tk.Label(f_mpu, text="⚪ CHỜ KẾT NỐI", font=("Segoe UI", 9, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"])
        self.lbl_diag_mpu.pack()

        # Ô D2: Sóng NRF24
        f_nrf = tk.Frame(diag_frame, bg=THEME["card_alt"], padx=6, pady=4, bd=1, relief=tk.SOLID)
        f_nrf.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_nrf, text="📡 SÓNG NRF24", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_diag_nrf = tk.Label(f_nrf, text="⚪ CHƯA CÓ SÓNG", font=("Segoe UI", 9, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"])
        self.lbl_diag_nrf.pack()

        # Ô D3: ESP32 Super Mini UART
        f_spm = tk.Frame(diag_frame, bg=THEME["card_alt"], padx=6, pady=4, bd=1, relief=tk.SOLID)
        f_spm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_spm, text="⚡ SUPER MINI UART", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_diag_spm = tk.Label(f_spm, text="⚪ CHƯA NỐI", font=("Segoe UI", 9, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"])
        self.lbl_diag_spm.pack()

        # Ô D4: Tư thế sẵn sàng ARM
        f_upr = tk.Frame(diag_frame, bg=THEME["card_alt"], padx=6, pady=4, bd=1, relief=tk.SOLID)
        f_upr.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        tk.Label(f_upr, text="📐 TƯ THẾ XE", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack()
        self.lbl_diag_upright = tk.Label(f_upr, text="⚪ CHỜ TÍN HIỆU", font=("Segoe UI", 9, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"])
        self.lbl_diag_upright.pack()

        # Thanh hiển thị độ nghiêng trực quan (Level Bar)
        tilt_frame = tk.Frame(card, bg=THEME["card"], pady=4)
        tilt_frame.pack(fill=tk.X)
        tk.Label(tilt_frame, text="Thanh Cân Bằng [-15°..+15°]:", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card"]).pack(side=tk.LEFT)
        self.canvas_tilt = tk.Canvas(tilt_frame, height=14, bg="#11111b", highlightthickness=0)
        self.canvas_tilt.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)

        # Banner chỉ dẫn tư thế sẵn sàng ARM
        self.lbl_upright_guide = tk.Label(
            card,
            text="👉 HÃY DỰNG XE THẲNG ĐỨNG (GÓC ±3.5°) RỒI BẤM 'ARM XE (X)' ĐỂ BẬT CÂN BẰNG",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["warning"],
            bg=THEME["card"],
            pady=4
        )
        self.lbl_upright_guide.pack(fill=tk.X)

    def _update_telemetry_widgets(self):
        # Màu theo trạng thái
        st_colors = {
            "ARMED": THEME["success"],
            "DISARMED": THEME["warning"],
            "CALIBRATING": THEME["accent"],
            "BOOT": THEME["primary"],
            "FAULT": THEME["danger"]
        }
        col = st_colors.get(self.robot_state_str, THEME["subtext"])
        self.lbl_telem_state.config(text=self.robot_state_str, fg=col)

        # Cập nhật Pitch
        self.lbl_telem_pitch.config(
            text=f"{self.pitch_deg:+.2f}°",
            fg=THEME["success"] if abs(self.pitch_deg) <= 3.0 else (THEME["warning"] if abs(self.pitch_deg) <= 12.0 else THEME["danger"])
        )

        # Cập nhật Khoảng cách
        self.lbl_telem_dist.config(
            text=f"{self.dist_cm} cm",
            fg=THEME["danger"] if self.dist_cm < 20 and self.dist_cm > 0 else THEME["primary"]
        )

        # Cập nhật Pin
        bat_str = f"{self.bat_v:.2f} V" if self.bat_v > 0 else "-- V"
        self.lbl_telem_bat.config(text=bat_str)

        # Cập nhật banner chỉ dẫn tư thế sẵn sàng ARM
        if hasattr(self, 'lbl_upright_guide'):
            if self.robot_state_str == "ARMED":
                self.lbl_upright_guide.config(
                    text="⚡ XE ĐANG CÂN BẰNG TỰ ĐỘNG (ARMED) - DÙNG W/A/S/D ĐỂ LÁI XE",
                    fg=THEME["success"]
                )
            elif abs(self.pitch_deg) <= 3.5:
                self.lbl_upright_guide.config(
                    text="✔ XE ĐANG DỰNG THẲNG ĐÚNG CHUẨN (±3.5°) - BẤM 'ARM XE (X)' ĐỂ BẮT ĐẦU CÂN BẰNG!",
                    fg=THEME["success"]
                )
            else:
                self.lbl_upright_guide.config(
                    text=f"⚠ XE ĐANG NGHIÊNG ({self.pitch_deg:+.1f}°) - CẦN DỰNG THẲNG TRONG VÙNG ±3.5° ĐỂ ARM",
                    fg=THEME["warning"]
                )

        # Cập nhật các ô chẩn đoán phần cứng (Hardware Diagnostics)
        if hasattr(self, 'lbl_diag_mpu'):
            if self.diag_mpu_ok:
                self.lbl_diag_mpu.config(text=f"🟢 OK ({self.diag_imu_err} lỗi)", fg=THEME["success"])
            elif self.robot_state_str not in ("DISCONNECTED", "WAITING..."):
                self.lbl_diag_mpu.config(text=f"🔴 LỖI I2C ({self.diag_imu_err})", fg=THEME["danger"])
            else:
                self.lbl_diag_mpu.config(text="⚪ CHỜ XE", fg=THEME["subtext"])

        if hasattr(self, 'lbl_diag_nrf'):
            if self.diag_nrf_ok:
                self.lbl_diag_nrf.config(text="🟢 ĐÃ KẾT NỐI (ACK OK)", fg=THEME["success"])
            elif self.serial_running:
                self.lbl_diag_nrf.config(text="🔴 MẤT SÓNG XE", fg=THEME["danger"])
            else:
                self.lbl_diag_nrf.config(text="⚪ CHƯA MỞ COM", fg=THEME["subtext"])

        if hasattr(self, 'lbl_diag_spm'):
            if self.diag_spm_ok:
                self.lbl_diag_spm.config(text="🟢 ĐÃ NỐI UART1", fg=THEME["success"])
            else:
                self.lbl_diag_spm.config(text="⚪ CHƯA NỐI", fg=THEME["subtext"])

        if hasattr(self, 'lbl_diag_upright'):
            if self.diag_upright:
                self.lbl_diag_upright.config(text="🟢 SẴN SÀNG ARM (±3.5°)", fg=THEME["success"])
            else:
                self.lbl_diag_upright.config(text=f"🟡 NGHIÊNG ({self.pitch_deg:+.1f}°)", fg=THEME["warning"])

        # Vẽ thanh Tilt visualizer
        self._draw_tilt_bar()

    def _draw_tilt_bar(self):
        self.canvas_tilt.delete("all")
        w = max(self.canvas_tilt.winfo_width(), 100)
        h = max(self.canvas_tilt.winfo_height(), 14)
        cx = w // 2

        # Vạch trung tâm (cân bằng 0 độ)
        self.canvas_tilt.create_line(cx, 0, cx, h, fill="#ffffff", width=2)

        # Giới hạn an toàn ±15 độ
        clamped_pitch = max(-15.0, min(15.0, self.pitch_deg))
        indicator_x = cx + int((clamped_pitch / 15.0) * (w / 2 - 4))

        color = THEME["success"] if abs(self.pitch_deg) < 4.0 else (THEME["warning"] if abs(self.pitch_deg) < 12.0 else THEME["danger"])
        self.canvas_tilt.create_rectangle(min(cx, indicator_x), 2, max(cx, indicator_x), h - 2, fill=color, outline="")

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 4: ĐIỀU KHIỂN XE (NÚT BẤM & TỐC ĐỘ)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_controls_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🎮 Bảng Lệnh Điều Khiển Xe (Phím tắt: W/A/S/D/Space/X/Z/E) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.X, pady=(0, 8))

        # Hàng 1: Nút trạng thái chính (ARM, DISARM, ESTOP)
        act_row = tk.Frame(card, bg=THEME["card"])
        act_row.pack(fill=tk.X, pady=(0, 8))

        btn_arm = tk.Button(
            act_row,
            text="🟢 ARM XE (X)",
            font=("Segoe UI", 10, "bold"),
            bg="#2e7d32",
            fg="#ffffff",
            activebackground=THEME["success"],
            relief=tk.FLAT,
            padx=12,
            pady=6,
            command=self.cmd_arm
        )
        btn_arm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        btn_disarm = tk.Button(
            act_row,
            text="🟡 DISARM (Z)",
            font=("Segoe UI", 10, "bold"),
            bg="#f57f17",
            fg="#11111b",
            activebackground=THEME["warning"],
            relief=tk.FLAT,
            padx=12,
            pady=6,
            command=self.cmd_disarm
        )
        btn_disarm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        btn_estop = tk.Button(
            act_row,
            text="🛑 DỪNG KHẨN (E)",
            font=("Segoe UI", 10, "bold"),
            bg="#c62828",
            fg="#ffffff",
            activebackground=THEME["danger"],
            relief=tk.FLAT,
            padx=12,
            pady=6,
            command=self.cmd_estop
        )
        btn_estop.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=3)

        # Hàng 2: Bàn phím điều hướng D-Pad và thanh trượt
        dpad_frame = tk.Frame(card, bg=THEME["card"])
        dpad_frame.pack(fill=tk.X)

        # D-Pad bên trái
        pad = tk.Frame(dpad_frame, bg=THEME["card"])
        pad.pack(side=tk.LEFT, padx=10)

        btn_fwd = tk.Button(pad, text="▲ Tiến (W)", width=10, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd)
        btn_fwd.grid(row=0, column=1, pady=2)

        btn_left = tk.Button(pad, text="◄ Trái (A)", width=10, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, command=self.cmd_left)
        btn_left.grid(row=1, column=0, padx=2)

        btn_stop = tk.Button(pad, text="■ STOP (Space)", width=12, bg="#37474f", fg="#ffffff", relief=tk.FLAT, command=self.cmd_stop)
        btn_stop.grid(row=1, column=1, padx=2)

        btn_right = tk.Button(pad, text="► Phải (D)", width=10, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, command=self.cmd_right)
        btn_right.grid(row=1, column=2, padx=2)

        btn_bwd = tk.Button(pad, text="▼ Lùi (S)", width=10, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd)
        btn_bwd.grid(row=2, column=1, pady=2)

        # Thanh điều chỉnh bước tốc độ
        sliders = tk.Frame(dpad_frame, bg=THEME["card"])
        sliders.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10)

        tk.Label(sliders, text="Tốc độ đặt (Drive Step):", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_speed_step = tk.Scale(
            sliders, from_=50, to=400, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_speed_step.set(150)
        self.slider_speed_step.pack(fill=tk.X, pady=(0, 4))

        tk.Label(sliders, text="Độ nhạy rẽ (Steer Step):", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_steer_step = tk.Scale(
            sliders, from_=50, to=500, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_steer_step.set(200)
        self.slider_steer_step.pack(fill=tk.X)

    def _bind_keyboard_controls(self):
        self.root.bind("<Key-w>", lambda e: self.cmd_fwd())
        self.root.bind("<Key-W>", lambda e: self.cmd_fwd())
        self.root.bind("<Key-s>", lambda e: self.cmd_bwd())
        self.root.bind("<Key-S>", lambda e: self.cmd_bwd())
        self.root.bind("<Key-a>", lambda e: self.cmd_left())
        self.root.bind("<Key-A>", lambda e: self.cmd_left())
        self.root.bind("<Key-d>", lambda e: self.cmd_right())
        self.root.bind("<Key-D>", lambda e: self.cmd_right())
        self.root.bind("<space>", lambda e: self.cmd_stop())
        self.root.bind("<Key-r>", lambda e: self.cmd_stop())
        self.root.bind("<Key-R>", lambda e: self.cmd_stop())
        self.root.bind("<Key-x>", lambda e: self.cmd_arm())
        self.root.bind("<Key-X>", lambda e: self.cmd_arm())
        self.root.bind("<Key-z>", lambda e: self.cmd_disarm())
        self.root.bind("<Key-Z>", lambda e: self.cmd_disarm())
        self.root.bind("<Key-e>", lambda e: self.cmd_estop())
        self.root.bind("<Key-E>", lambda e: self.cmd_estop())

    def send_serial_cmd(self, cmd_text):
        if not self.serial_client or not self.serial_client.is_open:
            self.log_console(f"[Lỗi gửi] Chưa kết nối ESP32 DevKit V1!", "WARN")
            return
        try:
            self.serial_client.write_line(cmd_text)
            self.tx_count += 1
            self.log_console(f"TX: {cmd_text}", "TX")
        except Exception as e:
            self.log_console(f"[Lỗi gửi Serial] {e}", "ERROR")

    def cmd_arm(self):
        self.send_serial_cmd("ARM")

    def cmd_disarm(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd("DISARM")

    def cmd_estop(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd("ESTOP")

    def cmd_fwd(self):
        step = int(self.slider_speed_step.get())
        self.drive_cmd = min(1000, self.drive_cmd + step)
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def cmd_bwd(self):
        step = int(self.slider_speed_step.get())
        self.drive_cmd = max(-1000, self.drive_cmd - step)
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def cmd_left(self):
        step = int(self.slider_steer_step.get())
        self.steer_cmd = max(-1000, self.steer_cmd - step)
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def cmd_right(self):
        step = int(self.slider_steer_step.get())
        self.steer_cmd = min(1000, self.steer_cmd + step)
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def cmd_stop(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd(" ")

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 5: CONSOLE LOG / TERMINAL
    # ──────────────────────────────────────────────────────────────────────────
    def _build_console_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🖥 Console & Lệnh Trực Tiếp ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=8,
            pady=6
        )
        card.pack(fill=tk.BOTH, expand=True)

        self.txt_console = tk.Text(
            card,
            bg=THEME["console_bg"],
            fg=THEME["text"],
            font=("Consolas", 9),
            height=8,
            wrap=tk.WORD,
            bd=0
        )
        self.txt_console.pack(fill=tk.BOTH, expand=True)

        # Định nghĩa các màu chữ tag trong console
        self.txt_console.tag_config("TX", foreground=THEME["primary"])
        self.txt_console.tag_config("RX", foreground=THEME["success"])
        self.txt_console.tag_config("WARN", foreground=THEME["warning"])
        self.txt_console.tag_config("ERROR", foreground=THEME["danger"])
        self.txt_console.tag_config("INFO", foreground=THEME["subtext"])
        self.txt_console.tag_config("SUCCESS", foreground=THEME["success"])

        # Hàng nhập lệnh tay
        input_frame = tk.Frame(card, bg=THEME["card"], pady=4)
        input_frame.pack(fill=tk.X)

        self.entry_cmd = tk.Entry(
            input_frame,
            bg=THEME["card_alt"],
            fg=THEME["text"],
            font=("Consolas", 10),
            insertbackground="#ffffff",
            relief=tk.FLAT
        )
        self.entry_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.entry_cmd.bind("<Return>", lambda e: self._send_manual_command())

        btn_send = tk.Button(
            input_frame,
            text="GỬI",
            font=("Segoe UI", 8, "bold"),
            bg=THEME["primary"],
            fg="#11111b",
            relief=tk.FLAT,
            padx=10,
            command=self._send_manual_command
        )
        btn_send.pack(side=tk.LEFT, padx=2)

        btn_clear = tk.Button(
            input_frame,
            text="XÓA",
            font=("Segoe UI", 8),
            bg=THEME["card_alt"],
            fg=THEME["subtext"],
            relief=tk.FLAT,
            padx=8,
            command=lambda: self.txt_console.delete("1.0", tk.END)
        )
        btn_clear.pack(side=tk.LEFT, padx=2)

    def _send_manual_command(self):
        cmd = self.entry_cmd.get().strip()
        if cmd:
            self.send_serial_cmd(cmd)
            self.entry_cmd.delete(0, tk.END)

    def log_console(self, text, tag="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] {text}\n"
        self.txt_console.insert(tk.END, line, tag)
        self.txt_console.see(tk.END)

    def _build_status_bar(self):
        status_bar = tk.Frame(self.root, bg=THEME["panel"], height=24, padx=12, pady=2)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_footer_info = tk.Label(
            status_bar,
            text="Hệ thống sẵn sàng | Tuân thủ quy tắc AGENTS.md: Nhấc bánh xe khỏi mặt đất khi thử nghiệm!",
            font=("Segoe UI", 9),
            fg=THEME["subtext"],
            bg=THEME["panel"]
        )
        self.lbl_footer_info.pack(side=tk.LEFT)

        self.lbl_packets_stat = tk.Label(
            status_bar,
            text="TX: 0 | RX: 0",
            font=("Consolas", 9),
            fg=THEME["accent"],
            bg=THEME["panel"]
        )
        self.lbl_packets_stat.pack(side=tk.RIGHT)

    # ──────────────────────────────────────────────────────────────────────────
    # VÒNG LẶP CẬP NHẬT GIAO DIỆN CHÍNH ( định kỳ ~30ms )
    # ──────────────────────────────────────────────────────────────────────────
    def _ui_periodic_update(self):
        # 1. Cập nhật đồng hồ hệ thống
        now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.lbl_system_clock.config(text=now_str)

        # 2. Cập nhật thống kê gói tin
        self.lbl_packets_stat.config(text=f"TX: {self.tx_count} | RX: {self.rx_count}")

        # 3. Cập nhật khung hình Video (nếu có frame mới)
        if self.current_frame is not None:
            try:
                h_can = self.video_canvas.winfo_height()
                w_can = self.video_canvas.winfo_width()

                if w_can > 50 and h_can > 50:
                    orig_h, orig_w = self.current_frame.shape[:2]
                    scale = min(w_can / orig_w, h_can / orig_h)
                    new_w = max(1, int(orig_w * scale))
                    new_h = max(1, int(orig_h * scale))

                    resized = cv2.resize(self.current_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                    rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                    pil_img = Image.fromarray(rgb_frame)
                    self.photo_img = ImageTk.PhotoImage(image=pil_img)

                    self.video_canvas.delete("all")
                    pos_x = (w_can - new_w) // 2
                    pos_y = (h_can - new_h) // 2
                    self.video_canvas.create_image(pos_x, pos_y, anchor=tk.NW, image=self.photo_img)

                    self.lbl_fps.config(text=f"FPS: {self.fps_display:.1f} | Kích thước: {orig_w}x{orig_h}")
            except Exception:
                pass

        # Lặp lại sau 33ms (~30 FPS cho UI update)
        self.root.after(33, self._ui_periodic_update)

    def on_closing(self):
        if messagebox.askokcancel("Thoát", "Bạn có chắc muốn đóng Ground Control Station?"):
            self.stop_video_stream()
            self.disconnect_serial()
            self.root.destroy()


def main():
    root = tk.Tk()
    app = GroundStationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
