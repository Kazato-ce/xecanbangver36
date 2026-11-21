#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  MÁY TRẠM ĐIỀU KHIỂN & GIÁM SÁT XE CÂN BẰNG 2 BÁNH STM32F405 (GCS)
  TÍCH HỢP ESP32-CAM VIDEO STREAM & ĐIỀU KHIỂN CỬ CHỈ BÀN TAY MEDIAPIPE
================================================================================
Tác vụ chính:
 1. Video Stream từ ESP32-CAM / ESP32-S3 CAM qua HTTP MJPEG/RTSP (Góc nhìn thứ nhất FPV).
 2. Điều khiển xe bằng CỬ CHỈ BÀN TAY (MediaPipe Hands qua Webcam máy tính):
    - Đưa ngón tay lên/xuống -> Tiến/Lùi.
    - Nghiêng ngón tay trái/phải -> Bẻ lái Trái/Phải.
    - Rút tay về hoặc ra ngoài vùng -> Tự động dừng phanh (STOP).
 3. Kết nối Serial COM siêu tốc (115200 baud) qua ESP32 DevKit V1 NRF24 Dongle.
 4. Bảng điều khiển 8 HƯỚNG DI CHUYỂN: Tiến, Lùi, Trái, Phải, Tiến-Trái, Tiến-Phải,
    Lùi-Trái, Lùi-Phải (Phím tắt: W/A/S/D/Q/E/Z/C/Space/X/V).
 5. Giám sát Telemetry thời gian thực: Góc Pitch, Tốc độ góc Gyro, Pin 4S, Vật cản.
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

# Import driver serial, video worker và hand gesture controller
try:
    from serial_driver import SerialClient, list_com_ports, get_port_details, find_best_esp32_port
    from video_worker import VideoStreamWorker
    from hand_controller import HandGestureWorker
except ImportError:
    try:
        from tools.python_ground_station.serial_driver import SerialClient, list_com_ports, get_port_details, find_best_esp32_port
        from tools.python_ground_station.video_worker import VideoStreamWorker
        from tools.python_ground_station.hand_controller import HandGestureWorker
    except ImportError:
        SerialClient = None
        VideoStreamWorker = None
        HandGestureWorker = None

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
        self.root.title("Xe Cân Bằng STM32F405 — Ground Station & MediaPipe Hand Control")
        self.root.geometry("1340x860")
        self.root.minsize(1120, 720)
        self.root.configure(bg=THEME["bg"])

        # Trạng thái Serial COM
        self.serial_client = None
        self.serial_thread = None
        self.serial_running = False
        self.last_connected_port = None
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        self.auto_reconnecting = False
        self._last_reconnect_attempt_ms = 0

        # Trạng thái Video Stream FPV ESP32-CAM
        self.video_worker = None
        self.current_frame = None
        self.photo_img = None
        self.fps_display = 0.0

        # Trạng thái Điều khiển cử chỉ bàn tay (MediaPipe Hand Gesture)
        self.hand_worker = None
        self.hand_mode_enabled = False
        self.hand_frame = None
        self.hand_photo_img = None
        self._hand_was_active = False

        # Trạng thái Xử lý Thị giác máy tính (AI Vision Hooks)
        self.ai_vision_enabled = tk.BooleanVar(value=False)
        self.ai_mode = tk.StringVar(value="Color / Object Tracking")
        self.ai_auto_steer_enabled = tk.BooleanVar(value=False)

        # Biến trạng thái xe
        self.robot_state_str = "DISCONNECTED"
        self.pitch_deg = 0.0
        self.gyro_rate = 0.0
        self.dist_cm = 0
        self.bat_v = 0.0
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.tx_count = 0
        self.rx_count = 0

        # Biến chẩn đoán phần cứng
        self.diag_mpu_ok = False
        self.diag_nrf_ok = False
        self.nrf_link_ok = False

        # Khởi tạo giao diện
        self._build_header()
        self._build_main_layout()
        self._build_status_bar()

        # Đăng ký phím tắt bàn phím
        self._bind_keyboard_events()

        # Quét cổng COM ban đầu
        self.refresh_ports()
        self.root.after(500, self._auto_connect_dongle)

        # Bắt đầu vòng lặp cập nhật giao diện (33 ms ~ 30 FPS)
        self.root.after(33, self._ui_periodic_update)

        # Đăng ký đóng cửa sổ an toàn
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # ──────────────────────────────────────────────────────────────────────────
    # HEADER: TIÊU ĐỀ & ĐỒNG HỒ HỆ THỐNG
    # ──────────────────────────────────────────────────────────────────────────
    def _build_header(self):
        header = tk.Frame(self.root, bg=THEME["panel"], height=46, padx=16, pady=6)
        header.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(
            header,
            text="⚡ ROBOT TỰ CÂN BẰNG STM32F405 — GROUND STATION & MEDIAPIPE GESTURE CONTROL",
            font=("Segoe UI", 11, "bold"),
            fg=THEME["primary"],
            bg=THEME["panel"]
        )
        title_lbl.pack(side=tk.LEFT)

        self.lbl_dongle_status = tk.Label(
            header,
            text="● DONGLE: CHƯA KẾT NỐI",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["danger"],
            bg=THEME["panel"],
            padx=12
        )
        self.lbl_dongle_status.pack(side=tk.LEFT)

        self.lbl_system_clock = tk.Label(
            header,
            text="",
            font=("Consolas", 10),
            fg=THEME["accent"],
            bg=THEME["panel"]
        )
        self.lbl_system_clock.pack(side=tk.RIGHT)

    # ──────────────────────────────────────────────────────────────────────────
    # BỐ CỤC CHÍNH (CỘT TRÁI: VIDEO FPV & AI VISION | CỘT PHẢI: LÁI TAY & 8 HƯỚNG)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_main_layout(self):
        main_pane = tk.Frame(self.root, bg=THEME["bg"], padx=8, pady=6)
        main_pane.pack(fill=tk.BOTH, expand=True)

        # Cột trái: Khung Video Stream FPV ESP32-CAM (~54% chiều rộng)
        left_col = tk.Frame(main_pane, bg=THEME["bg"], padx=4, pady=4)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Cột phải: COM Port, Telemetry, Điều khiển 8 hướng & Lái tay MediaPipe (~46% chiều rộng)
        right_col = tk.Frame(main_pane, bg=THEME["bg"], width=560, padx=4, pady=4)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)

        self._build_video_panel(left_col)
        self._build_serial_panel(right_col)
        self._build_telemetry_panel(right_col)
        self._build_controls_8way_panel(right_col)
        self._build_console_panel(right_col)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 1: KHUNG STREAM VIDEO FPV ESP32-CAM & THỊ GIÁC MÁY TÍNH
    # ──────────────────────────────────────────────────────────────────────────
    def _build_video_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📹 ESP32-S3 CAM Video Stream (FPV Góc Nhìn Thứ Nhất) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.BOTH, expand=True)

        # Hàng 1: URL Stream & Nút kết nối
        url_frame = tk.Frame(card, bg=THEME["card"])
        url_frame.pack(fill=tk.X, pady=(0, 6))

        tk.Label(url_frame, text="URL Stream:", fg=THEME["text"], bg=THEME["card"], font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.cb_stream_url = ttk.Combobox(
            url_frame,
            values=[
                "http://192.168.4.2/stream",           # ESP32-CAM / ESP32-S3 phát Access Point
                "http://192.168.4.2:81/stream",
                "http://192.168.1.100:81/stream",       # ESP32-CAM nối WiFi nhà
                "http://192.168.1.50/stream",
                "http://192.168.4.2/capture",          # Chế độ chụp ảnh snapshot
                "0"                                    # Webcam laptop/PC (thử nghiệm)
            ],
            width=32,
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

        btn_snap = tk.Button(
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
        btn_snap.pack(side=tk.LEFT, padx=2)

        # Hàng 2: Thanh công cụ Nhận diện AI / Thị giác máy tính
        ai_frame = tk.Frame(card, bg=THEME["card_alt"], padx=8, pady=4, relief=tk.GROOVE, bd=1)
        ai_frame.pack(fill=tk.X, pady=(0, 6))

        chk_ai = tk.Checkbutton(
            ai_frame,
            text="⚡ BẬT NHẬN DIỆN AI",
            variable=self.ai_vision_enabled,
            fg=THEME["success"],
            bg=THEME["card_alt"],
            selectcolor="#11111b",
            activebackground=THEME["card_alt"],
            activeforeground=THEME["success"],
            font=("Segoe UI", 8, "bold")
        )
        chk_ai.pack(side=tk.LEFT, padx=(0, 6))

        tk.Label(ai_frame, text="Chế độ:", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(side=tk.LEFT, padx=(0, 4))

        cb_ai_mode = ttk.Combobox(
            ai_frame,
            textvariable=self.ai_mode,
            values=[
                "Color / Object Tracking",
                "Line Following (Bám đường)",
                "Edge / Canny Detection",
                "Raw (Không xử lý)"
            ],
            width=22,
            font=("Segoe UI", 8)
        )
        cb_ai_mode.pack(side=tk.LEFT, padx=(0, 6))

        chk_auto = tk.Checkbutton(
            ai_frame,
            text="Tự động lái theo mục tiêu",
            variable=self.ai_auto_steer_enabled,
            fg=THEME["accent"],
            bg=THEME["card_alt"],
            selectcolor="#11111b",
            activebackground=THEME["card_alt"],
            activeforeground=THEME["accent"],
            font=("Segoe UI", 8)
        )
        chk_auto.pack(side=tk.LEFT)

        # Khung Video Canvas hiển thị khung hình FPV
        self.video_canvas = tk.Canvas(
            card,
            bg="#0b0b12",
            highlightthickness=1,
            highlightbackground=THEME["border"]
        )
        self.video_canvas.pack(fill=tk.BOTH, expand=True)
        self.video_canvas.bind("<Configure>", self._on_canvas_resize)

        self._draw_video_placeholder("Chưa mở stream video ESP32-CAM\nNhập URL stream và bấm 'MỞ STREAM'")

        info_bar = tk.Frame(card, bg=THEME["card_alt"], height=24, padx=8, pady=2)
        info_bar.pack(fill=tk.X, pady=(4, 0))

        self.lbl_stream_status = tk.Label(
            info_bar,
            text="● STREAM TẮT",
            font=("Segoe UI", 8, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card_alt"]
        )
        self.lbl_stream_status.pack(side=tk.LEFT)

        self.lbl_fps = tk.Label(
            info_bar,
            text="FPS: 0.0 | Phân giải: --x--",
            font=("Consolas", 8),
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
        if VideoStreamWorker is None:
            messagebox.showerror("Lỗi", "Không tìm thấy module video_worker.py!")
            return

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
        self._log_console(f"[VISION] Đang kết nối video stream: {url}", "info")

    def stop_video_stream(self):
        if self.video_worker:
            self.video_worker.stop()
            self.video_worker = None
        self.btn_toggle_stream.config(text="▶ MỞ STREAM", bg=THEME["primary"])
        self.lbl_stream_status.config(text="● STREAM ĐÃ DỪNG", fg=THEME["subtext"])
        self.lbl_fps.config(text="FPS: 0.0 | Phân giải: --x--")
        self._draw_video_placeholder("Stream đã dừng.\nBấm 'MỞ STREAM' để kết nối lại.")
        self._log_console("[VISION] Đã dừng video stream.", "info")

    def _on_video_frame(self, frame_bgr, fps):
        self.current_frame = frame_bgr
        self.fps_display = fps

    def _on_video_status(self, msg, is_ok):
        def _update():
            if is_ok:
                self.lbl_stream_status.config(text="● LIVE STREAM (ESP32-CAM OK)", fg=THEME["success"])
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
        self._log_console(f"[VISION] Đã lưu ảnh snapshot: {fname}", "info")
        messagebox.showinfo("Thành công", f"Đã lưu ảnh snapshot vào:\n{path}")

    def _process_ai_vision(self, frame):
        """Pipeline nhận diện thị giác máy tính"""
        if not self.ai_vision_enabled.get():
            return frame

        mode = self.ai_mode.get()
        h, w = frame.shape[:2]

        if mode == "Color / Object Tracking":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_color = np.array([0, 120, 70])
            upper_color = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_color, upper_color)

            lower_color2 = np.array([170, 120, 70])
            upper_color2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_color2, upper_color2)

            mask = mask1 | mask2
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                ((cx, cy), radius) = cv2.minEnclosingCircle(c)
                if radius > 15:
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)

                    center_x = w // 2
                    err_x = cx - center_x
                    cv2.line(frame, (center_x, int(cy)), (int(cx), int(cy)), (255, 255, 0), 1)

                    text = f"Target: ({int(cx)}, {int(cy)}) | Err_X: {int(err_x)}"
                    cv2.putText(frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    if self.ai_auto_steer_enabled.get() and self.serial_running and not self.hand_mode_enabled:
                        steer_val = int(np.clip(err_x * 1.5, -400, 400))
                        drive_val = 150 if abs(err_x) < 80 else 0
                        self.send_serial_command(f"DRIVE {drive_val} {steer_val}")

        elif mode == "Line Following (Bám đường)":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
            roi = thresh[int(h * 0.6):, :]
            M = cv2.moments(roi)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + int(h * 0.6)
                cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                cv2.line(frame, (w // 2, cy), (cx, cy), (0, 255, 255), 2)

        elif mode == "Edge / Canny Detection":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        return frame

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 2: KẾT NỐI SERIAL COM USB (ESP32 DevKit NRF24 Dongle)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_serial_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🔌 Cổng USB Serial COM (ESP32 DevKit NRF24 Dongle) ",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=6
        )
        card.pack(fill=tk.X, pady=(0, 6))

        row1 = tk.Frame(card, bg=THEME["card"])
        row1.pack(fill=tk.X, pady=2)

        tk.Label(row1, text="Cổng COM:", fg=THEME["text"], bg=THEME["card"], font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(0, 4))

        self.cb_ports = ttk.Combobox(row1, width=14, font=("Consolas", 8))
        self.cb_ports.pack(side=tk.LEFT, padx=(0, 4))

        btn_refresh = tk.Button(
            row1, text="🔄 Quét", font=("Segoe UI", 8),
            bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, padx=6, pady=1,
            command=self.refresh_ports
        )
        btn_refresh.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_connect = tk.Button(
            row1, text="⚡ KẾT NỐI", font=("Segoe UI", 8, "bold"),
            bg=THEME["primary"], fg="#11111b", activebackground=THEME["accent"],
            relief=tk.FLAT, padx=10, pady=2,
            command=self.toggle_serial_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 6))

        btn_ping = tk.Button(
            row1, text="PING", font=("Segoe UI", 8),
            bg=THEME["card_alt"], fg=THEME["subtext"], relief=tk.FLAT, padx=6, pady=1,
            command=lambda: self.send_serial_command("PING")
        )
        btn_ping.pack(side=tk.LEFT)

        row2 = tk.Frame(card, bg=THEME["card"])
        row2.pack(fill=tk.X, pady=(4, 0))

        chk_reconnect = tk.Checkbutton(
            row2,
            text="Tự động phát hiện & kết nối lại khi cắm USB",
            variable=self.auto_reconnect_var,
            fg=THEME["subtext"],
            bg=THEME["card"],
            selectcolor=THEME["card_alt"],
            activebackground=THEME["card"],
            activeforeground=THEME["text"],
            font=("Segoe UI", 8)
        )
        chk_reconnect.pack(side=tk.LEFT)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 3: GIÁM SÁT TELEMETRY XE CÂN BẰNG
    # ──────────────────────────────────────────────────────────────────────────
    def _build_telemetry_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📊 Giám Sát Trạng Thái & Telemetry ",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=6
        )
        card.pack(fill=tk.X, pady=(0, 6))

        row_state = tk.Frame(card, bg=THEME["card"])
        row_state.pack(fill=tk.X, pady=(0, 6))

        self.lbl_robot_state = tk.Label(
            row_state,
            text="DISARMED",
            font=("Segoe UI", 12, "bold"),
            fg="#11111b",
            bg=THEME["warning"],
            padx=10,
            pady=2,
            relief=tk.FLAT
        )
        self.lbl_robot_state.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        self.lbl_nrf_link = tk.Label(
            row_state,
            text="SÓNG NRF: CHỜ XE...",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["warning"],
            bg=THEME["card_alt"],
            padx=8,
            pady=2
        )
        self.lbl_nrf_link.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        tilt_card = tk.Frame(card, bg=THEME["card_alt"], padx=8, pady=6, relief=tk.GROOVE, bd=1)
        tilt_card.pack(fill=tk.X, pady=(0, 6))

        top_tilt = tk.Frame(tilt_card, bg=THEME["card_alt"])
        top_tilt.pack(fill=tk.X)

        tk.Label(top_tilt, text="GÓC PITCH:", font=("Segoe UI", 8, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(side=tk.LEFT)
        self.lbl_pitch_val = tk.Label(top_tilt, text="0.00°", font=("Segoe UI", 12, "bold"), fg=THEME["success"], bg=THEME["card_alt"])
        self.lbl_pitch_val.pack(side=tk.RIGHT)

        self.canvas_tilt = tk.Canvas(tilt_card, height=14, bg="#11111b", highlightthickness=0)
        self.canvas_tilt.pack(fill=tk.X, pady=(4, 0))
        self.canvas_tilt.bind("<Configure>", lambda e: self._draw_tilt_gauge())

        grid_frame = tk.Frame(card, bg=THEME["card"])
        grid_frame.pack(fill=tk.X, pady=(0, 4))

        b1 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=6, pady=4)
        b1.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(b1, text="Tốc độ Gyro", font=("Segoe UI", 7), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_gyro_val = tk.Label(b1, text="0.0 °/s", font=("Segoe UI", 10, "bold"), fg=THEME["text"], bg=THEME["card_alt"])
        self.lbl_gyro_val.pack(anchor="w")

        b2 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=6, pady=4)
        b2.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(b2, text="Pin 4S", font=("Segoe UI", 7), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_bat_val = tk.Label(b2, text="--.- V", font=("Segoe UI", 10, "bold"), fg=THEME["primary"], bg=THEME["card_alt"])
        self.lbl_bat_val.pack(anchor="w")

        b3 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=6, pady=4)
        b3.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(b3, text="Vật cản", font=("Segoe UI", 7), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_dist_val = tk.Label(b3, text="-- cm", font=("Segoe UI", 10, "bold"), fg=THEME["text"], bg=THEME["card_alt"])
        self.lbl_dist_val.pack(anchor="w")

        b4 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=6, pady=4)
        b4.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(b4, text="Drive / Steer", font=("Segoe UI", 7), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_cmd_val = tk.Label(b4, text="0 / 0", font=("Segoe UI", 10, "bold"), fg=THEME["accent"], bg=THEME["card_alt"])
        self.lbl_cmd_val.pack(anchor="w")

        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 4: BẢNG ĐIỀU KHIỂN 8 HƯỚNG & LÁI BẰNG CỬ CHỈ TAY MEDIAPIPE
    # ──────────────────────────────────────────────────────────────────────────
    def _build_controls_8way_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🎮 Điều Khiển 8 Hướng & Cử Chỉ Bàn Tay (MediaPipe Hands) ",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=6
        )
        card.pack(fill=tk.X, pady=(0, 6))

        # Hàng 1: Nút trạng thái chính + Nút bật Lái Cử Chỉ Tay MediaPipe
        act_row = tk.Frame(card, bg=THEME["card"])
        act_row.pack(fill=tk.X, pady=(0, 6))

        btn_arm = tk.Button(
            act_row, text="🟢 ARM (X)", font=("Segoe UI", 8, "bold"),
            bg="#2e7d32", fg="#ffffff", activebackground=THEME["success"],
            relief=tk.FLAT, padx=6, pady=4, command=self.cmd_arm
        )
        btn_arm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        btn_disarm = tk.Button(
            act_row, text="🟡 DISARM (Z/V)", font=("Segoe UI", 8, "bold"),
            bg="#f57f17", fg="#11111b", activebackground=THEME["warning"],
            relief=tk.FLAT, padx=6, pady=4, command=self.cmd_disarm
        )
        btn_disarm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        btn_estop = tk.Button(
            act_row, text="🛑 ESTOP (Esc)", font=("Segoe UI", 8, "bold"),
            bg="#c62828", fg="#ffffff", activebackground=THEME["danger"],
            relief=tk.FLAT, padx=6, pady=4, command=self.cmd_estop
        )
        btn_estop.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_hand_mode = tk.Button(
            act_row, text="✋ LÁI TAY: TẮT", font=("Segoe UI", 8, "bold"),
            bg=THEME["card_alt"], fg=THEME["text"], activebackground=THEME["accent"],
            relief=tk.FLAT, padx=6, pady=4, command=self.toggle_hand_mode
        )
        self.btn_hand_mode.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Hàng 2: D-Pad 8 hướng + Thanh trượt + Khung Mini Preview Cử Chỉ Tay
        ctrl_split = tk.Frame(card, bg=THEME["card"])
        ctrl_split.pack(fill=tk.X, pady=2)

        # 3x3 D-Pad
        dpad = tk.Frame(ctrl_split, bg=THEME["card_alt"], padx=6, pady=6, relief=tk.GROOVE, bd=1)
        dpad.pack(side=tk.LEFT, padx=(0, 6))

        btn_nw = tk.Button(dpad, text="↖ Q", width=5, font=("Segoe UI", 8, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd_left)
        btn_nw.grid(row=0, column=0, padx=2, pady=2)

        btn_n = tk.Button(dpad, text="▲ W", width=7, font=("Segoe UI", 8, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd)
        btn_n.grid(row=0, column=1, padx=2, pady=2)

        btn_ne = tk.Button(dpad, text="↗ E", width=5, font=("Segoe UI", 8, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd_right)
        btn_ne.grid(row=0, column=2, padx=2, pady=2)

        btn_w = tk.Button(dpad, text="◄ A", width=5, font=("Segoe UI", 8, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_left)
        btn_w.grid(row=1, column=0, padx=2, pady=2)

        btn_stop = tk.Button(dpad, text="■ STOP", width=7, font=("Segoe UI", 8, "bold"), bg="#c62828", fg="#ffffff", relief=tk.FLAT, command=self.cmd_stop)
        btn_stop.grid(row=1, column=1, padx=2, pady=2)

        btn_e = tk.Button(dpad, text="► D", width=5, font=("Segoe UI", 8, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_right)
        btn_e.grid(row=1, column=2, padx=2, pady=2)

        btn_sw = tk.Button(dpad, text="↙ Z", width=5, font=("Segoe UI", 8, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd_left)
        btn_sw.grid(row=2, column=0, padx=2, pady=2)

        btn_s = tk.Button(dpad, text="▼ S", width=7, font=("Segoe UI", 8, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd)
        btn_s.grid(row=2, column=1, padx=2, pady=2)

        btn_se = tk.Button(dpad, text="↘ C", width=5, font=("Segoe UI", 8, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd_right)
        btn_se.grid(row=2, column=2, padx=2, pady=2)

        # Thanh trượt
        sliders = tk.Frame(ctrl_split, bg=THEME["card"])
        sliders.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4)

        tk.Label(sliders, text="Tốc độ Tiến/Lùi:", font=("Segoe UI", 7, "bold"), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_speed_step = tk.Scale(
            sliders, from_=100, to=600, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_speed_step.set(200)
        self.slider_speed_step.pack(fill=tk.X, pady=(0, 2))

        tk.Label(sliders, text="Độ nhạy Rẽ/Quay:", font=("Segoe UI", 7, "bold"), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_steer_step = tk.Scale(
            sliders, from_=100, to=500, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_steer_step.set(250)
        self.slider_steer_step.pack(fill=tk.X, pady=(0, 2))

        self.lbl_active_direction = tk.Label(
            sliders, text="HƯỚNG: TĨNH (STOP)", font=("Segoe UI", 8, "bold"),
            fg=THEME["primary"], bg=THEME["card_alt"], pady=2
        )
        self.lbl_active_direction.pack(fill=tk.X)

        # Khung Mini Preview Cử Chỉ Tay (MediaPipe Webcam)
        hand_box = tk.Frame(ctrl_split, bg=THEME["card_alt"], bd=1, relief=tk.SOLID)
        hand_box.pack(side=tk.RIGHT, padx=(4, 0))

        self.hand_canvas = tk.Canvas(hand_box, width=148, height=110, bg="#0b0b12", highlightthickness=0)
        self.hand_canvas.pack()
        self._draw_hand_placeholder("Webcam Lái Tay\n(Đang tắt)")

    def _draw_hand_placeholder(self, text):
        self.hand_canvas.delete("all")
        self.hand_canvas.create_text(
            74, 55, text=text, fill=THEME["subtext"], font=("Segoe UI", 8), justify=tk.CENTER
        )

    # ──────────────────────────────────────────────────────────────────────────
    # QUẢN LÝ CHẾ ĐỘ ĐIỀU KHIỂN CỬ CHỈ BÀN TAY (MEDIAPIPE HANDS)
    # ──────────────────────────────────────────────────────────────────────────
    def toggle_hand_mode(self):
        if self.hand_mode_enabled:
            self.stop_hand_mode()
        else:
            self.start_hand_mode()

    def start_hand_mode(self):
        if HandGestureWorker is None:
            messagebox.showerror("Lỗi", "Không tìm thấy module hand_controller.py hoặc mediapipe!")
            return

        self.hand_mode_enabled = True
        self._hand_was_active = False
        self.btn_hand_mode.config(text="✋ LÁI TAY: BẬT", bg=THEME["accent"], fg="#11111b")
        self._draw_hand_placeholder("Đang mở Webcam...")

        self.hand_worker = HandGestureWorker(
            cam_index=0,
            get_limits_fn=lambda: (int(self.slider_speed_step.get()), int(self.slider_steer_step.get())),
            on_control=self._on_hand_control,
            on_frame=self._on_hand_frame,
            on_status=self._on_hand_status
        )
        self.hand_worker.start()
        self._log_console("[GESTURE] Bắt đầu chế độ lái bằng cử chỉ tay MediaPipe.", "info")

    def stop_hand_mode(self):
        self.hand_mode_enabled = False
        if self.hand_worker:
            self.hand_worker.stop()
            self.hand_worker = None
        self.hand_frame = None
        self.btn_hand_mode.config(text="✋ LÁI TAY: TẮT", bg=THEME["card_alt"], fg=THEME["text"])
        self._draw_hand_placeholder("Webcam Lái Tay\n(Đang tắt)")

        if self._hand_was_active or self.drive_cmd != 0 or self.steer_cmd != 0:
            self._hand_was_active = False
            self.cmd_stop()
        self._log_console("[GESTURE] Đã tắt chế độ lái bằng cử chỉ tay.", "warn")

    def _on_hand_frame(self, frame_bgr):
        self.hand_frame = frame_bgr

    def _on_hand_status(self, msg, is_ok):
        def _update():
            if is_ok:
                self._log_console(f"[GESTURE] {msg}", "info")
            else:
                self._log_console(f"[GESTURE LỖI] {msg}", "err")
                self.stop_hand_mode()
        self.root.after(0, _update)

    def _on_hand_control(self, drive_val, steer_val, hand_found):
        if not self.hand_mode_enabled:
            return
        if hand_found:
            self._hand_was_active = True
            self.drive_cmd = drive_val
            self.steer_cmd = steer_val
            self.send_serial_command(f"DRIVE {drive_val} {steer_val}")
            self.lbl_active_direction.config(text=f"✋ LÁI: D={drive_val} S={steer_val}")
        elif self._hand_was_active:
            self._hand_was_active = False
            self.cmd_stop()

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 5: TERMINAL CONSOLE & GỬI LỆNH TRỰC TIẾP
    # ──────────────────────────────────────────────────────────────────────────
    def _build_console_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 💻 Terminal Console ",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=8,
            pady=6
        )
        card.pack(fill=tk.BOTH, expand=True)

        log_frame = tk.Frame(card, bg=THEME["console_bg"])
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.txt_console = tk.Text(
            log_frame,
            height=6,
            bg=THEME["console_bg"],
            fg="#a6e3a1",
            insertbackground="#ffffff",
            font=("Consolas", 8),
            yscrollcommand=scrollbar.set,
            bd=0,
            relief=tk.FLAT
        )
        self.txt_console.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.txt_console.yview)

        self.txt_console.tag_config("tx", foreground="#89b4fa")
        self.txt_console.tag_config("rx", foreground="#a6e3a1")
        self.txt_console.tag_config("err", foreground="#f38ba8")
        self.txt_console.tag_config("warn", foreground="#fab387")
        self.txt_console.tag_config("info", foreground="#cdd6f4")

        in_frame = tk.Frame(card, bg=THEME["card"])
        in_frame.pack(fill=tk.X)

        self.ent_cmd = tk.Entry(in_frame, font=("Consolas", 8), bg=THEME["card_alt"], fg=THEME["text"], insertbackground=THEME["text"], bd=1, relief=tk.SOLID)
        self.ent_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4), ipady=2)
        self.ent_cmd.bind("<Return>", lambda e: self._send_manual_cmd())

        btn_send = tk.Button(
            in_frame, text="GỬI", font=("Segoe UI", 8, "bold"),
            bg=THEME["primary"], fg="#11111b", relief=tk.FLAT, padx=8,
            command=self._send_manual_cmd
        )
        btn_send.pack(side=tk.LEFT, padx=(0, 2))

        btn_clear = tk.Button(
            in_frame, text="XÓA", font=("Segoe UI", 8),
            bg=THEME["card_alt"], fg=THEME["subtext"], relief=tk.FLAT, padx=6,
            command=self.clear_console
        )
        btn_clear.pack(side=tk.LEFT)

    # ──────────────────────────────────────────────────────────────────────────
    # STATUS BAR (THANH TRẠNG THÁI CUỐI TRANG)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_status_bar(self):
        sb = tk.Frame(self.root, bg=THEME["panel"], height=22, padx=10)
        sb.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_sb_port = tk.Label(sb, text="Port: CHƯA KẾT NỐI", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["panel"])
        self.lbl_sb_port.pack(side=tk.LEFT, padx=(0, 15))

        self.lbl_sb_txrx = tk.Label(sb, text="TX: 0 | RX: 0", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["panel"])
        self.lbl_sb_txrx.pack(side=tk.LEFT, padx=(0, 15))

        self.lbl_sb_mode = tk.Label(sb, text="MediaPipe Gesture & ESP32-S3 FPV Stream Ready", font=("Segoe UI", 8), fg=THEME["accent"], bg=THEME["panel"])
        self.lbl_sb_mode.pack(side=tk.RIGHT)

    # ──────────────────────────────────────────────────────────────────────────
    # XỬ LÝ PHÍM TẮT BÀN PHÍM
    # ──────────────────────────────────────────────────────────────────────────
    def _bind_keyboard_events(self):
        self.root.bind("<Key>", self._on_key_press)
        self.root.bind("<space>", lambda e: self.cmd_stop())

    def _on_key_press(self, event):
        if event.widget == self.ent_cmd or event.widget == self.cb_stream_url:
            return

        k = event.keysym.lower()
        if k == 'w':
            self.cmd_fwd()
        elif k == 's':
            self.cmd_bwd()
        elif k == 'a':
            self.cmd_left()
        elif k == 'd':
            self.cmd_right()
        elif k == 'q':
            self.cmd_fwd_left()
        elif k == 'e':
            self.cmd_fwd_right()
        elif k == 'z':
            self.cmd_bwd_left()
        elif k == 'c':
            self.cmd_bwd_right()
        elif k == 'x':
            self.cmd_arm()
        elif k == 'v':
            self.cmd_disarm()
        elif k == 'escape':
            self.cmd_estop()

    # ──────────────────────────────────────────────────────────────────────────
    # CÁC LỆNH ĐIỀU KHIỂN XE (8 HƯỚNG & HÀNH ĐỘNG)
    # ──────────────────────────────────────────────────────────────────────────
    def cmd_arm(self):
        self.send_serial_command("ARM")
        self.lbl_active_direction.config(text="LỆNH: 🟢 ARM XE")

    def cmd_disarm(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_command("DISARM")
        self.lbl_active_direction.config(text="LỆNH: 🟡 DISARM")

    def cmd_estop(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_command("ESTOP")
        self.lbl_active_direction.config(text="LỆNH: 🛑 DỪNG KHẨN")

    def cmd_stop(self):
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_command(" ")
        self.lbl_active_direction.config(text="HƯỚNG: TĨNH (STOP)")

    def cmd_fwd(self):
        step = self.slider_speed_step.get()
        self.drive_cmd = step
        self.steer_cmd = 0
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ▲ TIẾN THẲNG")

    def cmd_bwd(self):
        step = self.slider_speed_step.get()
        self.drive_cmd = -step
        self.steer_cmd = 0
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ▼ LÙI THẲNG")

    def cmd_left(self):
        steer = self.slider_steer_step.get()
        self.drive_cmd = 0
        self.steer_cmd = -steer
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ◄ QUAY TRÁI")

    def cmd_right(self):
        steer = self.slider_steer_step.get()
        self.drive_cmd = 0
        self.steer_cmd = steer
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ► QUAY PHẢI")

    def cmd_fwd_left(self):
        step = self.slider_speed_step.get()
        steer = self.slider_steer_step.get()
        self.drive_cmd = int(step * 0.85)
        self.steer_cmd = -int(steer * 0.85)
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ↖ TIẾN - CHÉO TRÁI")

    def cmd_fwd_right(self):
        step = self.slider_speed_step.get()
        steer = self.slider_steer_step.get()
        self.drive_cmd = int(step * 0.85)
        self.steer_cmd = int(steer * 0.85)
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ↗ TIẾN - CHÉO PHẢI")

    def cmd_bwd_left(self):
        step = self.slider_speed_step.get()
        steer = self.slider_steer_step.get()
        self.drive_cmd = -int(step * 0.85)
        self.steer_cmd = -int(steer * 0.85)
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ↙ LÙI - CHÉO TRÁI")

    def cmd_bwd_right(self):
        step = self.slider_speed_step.get()
        steer = self.slider_steer_step.get()
        self.drive_cmd = -int(step * 0.85)
        self.steer_cmd = int(steer * 0.85)
        self.send_serial_command(f"DRIVE {self.drive_cmd} {self.steer_cmd}")
        self.lbl_active_direction.config(text="HƯỚNG: ↘ LÙI - CHÉO PHẢI")

    # ──────────────────────────────────────────────────────────────────────────
    # GIAO TIẾP SERIAL VÀ XỬ LÝ TELEMETRY
    # ──────────────────────────────────────────────────────────────────────────
    def refresh_ports(self):
        if SerialClient is None:
            return
        ports = list_com_ports()
        self.cb_ports['values'] = ports
        if ports and not self.cb_ports.get():
            best = find_best_esp32_port()
            self.cb_ports.set(best if best else ports[0])

    def _auto_connect_dongle(self):
        if not self.serial_running and self.auto_reconnect_var.get() and SerialClient:
            best = find_best_esp32_port()
            if best:
                self.cb_ports.set(best)
                self.toggle_serial_connection()

    def toggle_serial_connection(self):
        if self.serial_running:
            self._disconnect_serial()
        else:
            port = self.cb_ports.get().strip()
            if not port:
                messagebox.showwarning("Cảnh báo", "Vui lòng chọn một cổng COM hợp lệ!")
                return
            self._connect_serial(port)

    def _connect_serial(self, port):
        if SerialClient is None:
            messagebox.showerror("Lỗi", "Không tìm thấy driver serial (serial_driver.py)!")
            return

        try:
            self.serial_client = SerialClient(port=port, baudrate=115200, timeout=0.1)
            self.serial_client.open()
            self.serial_running = True
            self.last_connected_port = port

            self.btn_connect.config(text="❌ NGẮT KẾT NỐI", bg=THEME["danger"])
            self.lbl_dongle_status.config(text=f"● DONGLE: {port} (115200)", fg=THEME["success"])
            self.lbl_sb_port.config(text=f"Port: {port} (Connected)")

            self._log_console(f"[HỆ THỐNG] Đã mở cổng {port} @ 115200 baud.", "info")

            self.serial_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
            self.serial_thread.start()

            self.root.after(200, lambda: self.send_serial_command("PING"))

        except Exception as e:
            self.serial_running = False
            self.serial_client = None
            self._log_console(f"[LỖI] Không thể mở cổng {port}: {e}", "err")

    def _disconnect_serial(self):
        self.serial_running = False
        if self.serial_client:
            try:
                self.serial_client.close()
            except Exception:
                pass
            self.serial_client = None

        self.btn_connect.config(text="⚡ KẾT NỐI", bg=THEME["primary"])
        self.lbl_dongle_status.config(text="● DONGLE: ĐÃ NGẮT", fg=THEME["danger"])
        self.lbl_sb_port.config(text="Port: Disconnected")
        self.lbl_nrf_link.config(text="SÓNG NRF: ĐÃ NGẮT", fg=THEME["danger"])
        self._log_console("[HỆ THỐNG] Đã đóng kết nối Serial.", "info")

    def send_serial_command(self, cmd):
        if not self.serial_running or not self.serial_client:
            return
        try:
            full_cmd = cmd.strip() + "\n"
            self.serial_client.write(full_cmd.encode('utf-8', errors='ignore'))
            self.tx_count += 1
            self._log_console(f"TX >> {cmd.strip()}", "tx")
        except Exception as e:
            self._log_console(f"[LỖI TX] {e}", "err")

    def _send_manual_cmd(self):
        text = self.ent_cmd.get().strip()
        if text:
            self.send_serial_command(text)
            self.ent_cmd.delete(0, tk.END)

    def _serial_reader_thread(self):
        while self.serial_running and self.serial_client:
            try:
                line = self.serial_client.readline()
                if line:
                    line_str = line.decode('utf-8', errors='ignore').strip()
                    if line_str:
                        self.rx_count += 1
                        self.root.after(0, self._process_rx_line, line_str)
                else:
                    time.sleep(0.005)
            except Exception as e:
                if self.serial_running:
                    self.root.after(0, self._log_console, f"[LỖI RX] {e}", "err")
                break

        if self.serial_running:
            self.root.after(0, self._disconnect_serial)

    def _process_rx_line(self, line):
        tag = "rx"
        if "LỖI" in line or "ERR" in line:
            tag = "err"
        elif "CHỜ" in line or "WARN" in line:
            tag = "warn"
        elif "SÓNG TỐT" in line or "KẾT NỐI TỐT" in line:
            tag = "rx"
        self._log_console(line, tag)

        if "SÓNG TỐT" in line or "KẾT NỐI TỐT" in line:
            self.nrf_link_ok = True
            self.lbl_nrf_link.config(text="SÓNG NRF: 🟢 ĐÃ KẾT NỐI (AUTO-ACK)", fg=THEME["success"])
        elif "CHỜ TÍN HIỆU XE" in line or "Đang phát sóng tìm xe" in line:
            self.nrf_link_ok = False
            self.lbl_nrf_link.config(text="SÓNG NRF: 🟡 CHỜ XE BẬT NGUỒN", fg=THEME["warning"])
        elif "LỖI NRF24" in line or "LỖI PHẦN CỨNG" in line:
            self.nrf_link_ok = False
            self.lbl_nrf_link.config(text="SÓNG NRF: 🔴 LỖI PHẦN CỨNG", fg=THEME["danger"])

        if "pitch" in line.lower() or "vbat" in line.lower():
            m_pitch = re.search(r'"?pitch"?\s*:\s*([-+]?\d*\.?\d+)', line, re.I)
            if m_pitch:
                self.pitch_deg = float(m_pitch.group(1))

            m_rate = re.search(r'"?rate"?\s*:\s*([-+]?\d*\.?\d+)', line, re.I)
            if m_rate:
                self.gyro_rate = float(m_rate.group(1))

            m_vbat = re.search(r'"?vbat"?\s*:\s*([-+]?\d*\.?\d+)', line, re.I)
            if m_vbat:
                self.bat_v = float(m_vbat.group(1))

            m_state = re.search(r'"?state"?\s*:\s*"([A-Za-z0-9_]+)"', line, re.I)
            if m_state:
                self.robot_state_str = m_state.group(1).upper()

            m_dist = re.search(r'"?dist"?\s*:\s*(\d+)', line, re.I)
            if m_dist:
                self.dist_cm = int(m_dist.group(1))

    # ──────────────────────────────────────────────────────────────────────────
    # VÒNG LẶP CẬP NHẬT GIAO DIỆN & RENDER VIDEO (30 FPS)
    # ──────────────────────────────────────────────────────────────────────────
    def _ui_periodic_update(self):
        # 1. Cập nhật đồng hồ hệ thống
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        self.lbl_system_clock.config(text=now_str)

        # 2. Cập nhật Telemetry
        self.lbl_pitch_val.config(text=f"{self.pitch_deg:+.2f}°")
        self.lbl_gyro_val.config(text=f"{self.gyro_rate:+.1f} °/s")
        if self.bat_v > 0.1:
            self.lbl_bat_val.config(text=f"{self.bat_v:.2f} V")
        if self.dist_cm > 0:
            self.lbl_dist_val.config(text=f"{self.dist_cm} cm")

        self.lbl_cmd_val.config(text=f"{self.drive_cmd} / {self.steer_cmd}")

        if self.robot_state_str == "ARMED":
            self.lbl_robot_state.config(text="ARMED (CÂN BẰNG)", bg=THEME["success"], fg="#11111b")
        elif self.robot_state_str == "FAULT":
            self.lbl_robot_state.config(text="FAULT (NGÃ XE)", bg=THEME["danger"], fg="#ffffff")
        else:
            self.lbl_robot_state.config(text=self.robot_state_str, bg=THEME["warning"], fg="#11111b")

        self._draw_tilt_gauge()
        self.lbl_sb_txrx.config(text=f"TX: {self.tx_count} | RX: {self.rx_count}")

        # 3. Render khung hình Video FPV ESP32-CAM và Xử lý AI Vision
        if self.current_frame is not None:
            try:
                frame_to_show = self.current_frame.copy()
                frame_to_show = self._process_ai_vision(frame_to_show)

                h_can = self.video_canvas.winfo_height()
                w_can = self.video_canvas.winfo_width()

                if w_can > 50 and h_can > 50:
                    orig_h, orig_w = frame_to_show.shape[:2]
                    scale = min(w_can / orig_w, h_can / orig_h)
                    new_w = max(1, int(orig_w * scale))
                    new_h = max(1, int(orig_h * scale))

                    resized = cv2.resize(frame_to_show, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
                    rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                    pil_img = Image.fromarray(rgb_frame)
                    self.photo_img = ImageTk.PhotoImage(image=pil_img)

                    self.video_canvas.delete("all")
                    pos_x = (w_can - new_w) // 2
                    pos_y = (h_can - new_h) // 2
                    self.video_canvas.create_image(pos_x, pos_y, anchor=tk.NW, image=self.photo_img)

                    self.lbl_fps.config(text=f"FPS: {self.fps_display:.1f} | Phân giải: {orig_w}x{orig_h}")
            except Exception:
                pass

        # 4. Render khung hình Mini Preview Nhận diện Cử Chỉ Bàn Tay (MediaPipe)
        if self.hand_mode_enabled and self.hand_frame is not None:
            try:
                resized_hand = cv2.resize(self.hand_frame, (148, 110), interpolation=cv2.INTER_LINEAR)
                rgb_hand = cv2.cvtColor(resized_hand, cv2.COLOR_BGR2RGB)
                pil_hand = Image.fromarray(rgb_hand)
                self.hand_photo_img = ImageTk.PhotoImage(image=pil_hand)

                self.hand_canvas.delete("all")
                self.hand_canvas.create_image(0, 0, anchor=tk.NW, image=self.hand_photo_img)
            except Exception:
                pass

        # Lặp lại sau 33 ms (~30 FPS)
        self.root.after(33, self._ui_periodic_update)

    def _draw_tilt_gauge(self):
        w = self.canvas_tilt.winfo_width()
        h = self.canvas_tilt.winfo_height()
        if w < 10:
            return

        self.canvas_tilt.delete("all")
        cx = w // 2

        self.canvas_tilt.create_line(cx, 0, cx, h, fill="#6c7086", width=2)
        clamped_pitch = max(-15.0, min(15.0, self.pitch_deg))
        indicator_x = cx + int((clamped_pitch / 15.0) * (w / 2 - 4))

        color = THEME["success"] if abs(self.pitch_deg) < 4.0 else (THEME["warning"] if abs(self.pitch_deg) < 12.0 else THEME["danger"])
        self.canvas_tilt.create_rectangle(min(cx, indicator_x), 2, max(cx, indicator_x), h - 2, fill=color, outline="")

    def _log_console(self, msg, tag="info"):
        self.txt_console.insert(tk.END, msg + "\n", tag)
        self.txt_console.see(tk.END)

    def clear_console(self):
        self.txt_console.delete("1.0", tk.END)

    def on_closing(self):
        if messagebox.askokcancel("Thoát", "Bạn có chắc muốn đóng Ground Control Station?"):
            self.stop_hand_mode()
            self.stop_video_stream()
            self._disconnect_serial()
            self.root.destroy()


def main():
    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
