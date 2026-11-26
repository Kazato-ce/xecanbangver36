#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  MÁY TRẠM ĐIỀU KHIỂN & GIÁM SÁT XE CÂN BẰNG 2 BÁNH STM32F405
  (Ground Control Station - Dual Camera: ESP32-S3 FPV + Laptop Webcam Gesture)
================================================================================
"""

import os
import time
import re
import datetime
import threading
import traceback
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
from PIL import Image, ImageTk
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
try:
    import pygame
except ImportError:
    pygame = None

try:
    from tools.python_ground_station.serial_driver import (
        SerialClient, list_com_ports, get_port_details, find_best_esp32_port
    )
    from tools.python_ground_station.video_worker import VideoStreamWorker
    from tools.python_ground_station.hand_controller import HandGestureWorker
except ImportError:
    from serial_driver import (
        SerialClient, list_com_ports, get_port_details, find_best_esp32_port
    )
    from video_worker import VideoStreamWorker
    from hand_controller import HandGestureWorker


THEME = {
    "bg": "#11111b",
    "panel": "#181825",
    "card": "#1e1e2e",
    "card_alt": "#252538",
    "border": "#313244",
    "text": "#cdd6f4",
    "subtext": "#a6adc8",
    "primary": "#89b4fa",
    "success": "#a6e3a1",
    "warning": "#fab387",
    "danger": "#f38ba8",
    "accent": "#cba6f7",
    "console_bg": "#11111b",
}

MIN_PULSE_DRIVE_MS = 450
MIN_PULSE_STEER_MS = 220
DRIVE_REPEAT_MS = 100
GAMEPAD_POLL_MS = 16
GAMEPAD_RECONNECT_MS = 500
GAMEPAD_DEADZONE = 0.12


class GroundStationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Xe Cân Bằng STM32F405 - GCS (ESP32-S3 FPV + Webcam Gesture)")
        self.root.geometry("1340x830")
        self.root.minsize(1100, 720)
        self.root.configure(bg=THEME["bg"])

        # Trạng thái Serial
        self.serial_client = None
        self.serial_thread = None
        self.serial_running = False
        self.last_connected_port = None
        self.adas_enabled = False
        self.adas_enabled = True
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        self.auto_reconnecting = False
        self._last_reconnect_attempt_ms = 0
        self._last_drive_tx_ms = 0
        self._active_drive_inputs = {}
        self._pressed_keys = set()
        self._stop_input_active = False
        self._drive_axis_active = {"drive": False, "steer": False}
        self._drive_axis_started_at = {"drive": None, "steer": None}
        self._drive_axis_deadline = {"drive": None, "steer": None}
        self._drive_axis_hold_value = {"drive": 0, "steer": 0}
        self._drive_repeat_after_id = None
        self.gamepad_enabled = False
        self.gamepad = None
        self.gamepad_connected = False
        self.gamepad_buttons = [False, False, False, False]
        self.gamepad_ignore_button_edges = True
        self.gamepad_last_scan_ms = 0.0
        self.gamepad_last_tx_ms = 0.0
        self.gamepad_last_drive = 0
        self.gamepad_last_steer = 0
        self.gamepad_drive_baseline = 0.0
        self.gamepad_steer_baseline = 0.0
        self.gamepad_baseline_ready = False
        self.gamepad_last_drive = None
        self.gamepad_last_steer = None
        self._gamepad_poll_after_id = None
        self._hand_preview_error_reported = False
        self._control_limits_lock = threading.Lock()
        self._control_limits = (150, 200)

        # Trạng thái Video FPV (ESP32-S3 CAM)
        self.video_worker = None
        self.current_frame = None
        self.photo_img = None
        self.fps_display = 0.0

        # Trạng thái Webcam lái tay (MediaPipe)
        self.hand_worker = None
        self.hand_mode_enabled = False
        self.hand_frame = None
        self.hand_photo_img = None
        self._hand_was_active = False

        # Biến trạng thái xe
        self.robot_state_str = "CHƯA CÓ TRẠNG THÁI"
        self.pitch_deg = 0.0
        self.dist_cm = 0
        self.bat_v = 0.0
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.tx_count = 0
        self.rx_count = 0

        # Chẩn đoán phần cứng
        self.diag_mpu_ok = False
        self.diag_nrf_ok = False
        self.diag_spm_ok = False
        self.diag_upright = False
        self.diag_imu_err = 0

        self._setup_styles()
        self._build_header()
        self._build_main_layout()
        self._build_status_bar()
        self._bind_keyboard_controls()

        self.refresh_com_ports()
        self.root.after(100, self._ui_periodic_update)

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TLabel", background=THEME["card"], foreground=THEME["text"], font=("Segoe UI", 10))
        style.configure("TFrame", background=THEME["bg"])
        style.configure("Card.TFrame", background=THEME["card"], relief="flat")
        style.configure(
            "TCombobox",
            fieldbackground=THEME["card_alt"],
            background=THEME["border"],
            foreground=THEME["text"],
            arrowcolor=THEME["primary"]
        )

    def _build_header(self):
        header = tk.Frame(self.root, bg=THEME["panel"], height=55, padx=20, pady=8)
        header.pack(fill=tk.X, side=tk.TOP)

        tk.Label(
            header,
            text="⚡ STM32F405 SELF-BALANCING ROBOT — FPV & GESTURE GCS",
            font=("Segoe UI", 13, "bold"),
            fg=THEME["primary"],
            bg=THEME["panel"]
        ).pack(side=tk.LEFT)

        tk.Label(
            header,
            text="[ESP32-S3 FPV | Webcam MediaPipe | NRF24L01]",
            font=("Consolas", 10),
            fg=THEME["subtext"],
            bg=THEME["panel"]
        ).pack(side=tk.LEFT, padx=15)

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

        # Cột trái: Khung Video FPV từ ESP32-S3 CAM
        left_col = tk.Frame(main_pane, bg=THEME["bg"], padx=5, pady=5)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Cột phải: Kết nối + trạng thái xe + Điều khiển + Console
        right_col = tk.Frame(main_pane, bg=THEME["bg"], width=550, padx=5, pady=5)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)

        self._build_video_panel(left_col)
        self._build_serial_panel(right_col)
        self._build_controls_panel(right_col)
        self._build_console_panel(right_col)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 1: KHUNG FPV VIDEO TỪ ESP32-S3 CAM
    # ──────────────────────────────────────────────────────────────────────────
    def _build_video_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📹 ESP32-S3 CAM — Góc Nhìn Thứ Nhất (FPV Stream) ",
            font=("Segoe UI", 11, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.BOTH, expand=True)

        url_frame = tk.Frame(card, bg=THEME["card"])
        url_frame.pack(fill=tk.X, pady=(0, 8))

        tk.Label(url_frame, text="URL FPV:", fg=THEME["text"], bg=THEME["card"], font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.cb_stream_url = ttk.Combobox(
            url_frame,
            values=[
                "http://192.168.4.1:81/stream",
                "http://172.20.10.8:81/stream",
                "http://192.168.4.2:81/stream",
                "http://192.168.1.100:81/stream",
                "http://172.20.10.8/capture",
            ],
            width=34,
            font=("Consolas", 9)
        )
        self.cb_stream_url.set("http://192.168.4.1:81/stream")
        self.cb_stream_url.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        self.btn_toggle_stream = tk.Button(
            url_frame,
            text="▶ MỞ FPV",
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

        tk.Button(
            url_frame,
            text="📷 Chụp Ảnh",
            font=("Segoe UI", 9),
            bg=THEME["card_alt"],
            fg=THEME["text"],
            relief=tk.FLAT,
            padx=8,
            pady=3,
            command=self.save_snapshot
        ).pack(side=tk.LEFT, padx=4)

        self.video_canvas = tk.Canvas(
            card,
            bg="#0b0b12",
            highlightthickness=1,
            highlightbackground=THEME["border"]
        )
        self.video_canvas.pack(fill=tk.BOTH, expand=True)
        self.video_canvas.bind("<Configure>", self._on_canvas_resize)

        self._draw_video_placeholder("Chưa mở FPV từ ESP32-S3 CAM\nNhập IP:81/stream và bấm 'MỞ FPV'")

        info_bar = tk.Frame(card, bg=THEME["card_alt"], height=26, padx=8, pady=3)
        info_bar.pack(fill=tk.X, pady=(6, 0))

        self.lbl_stream_status = tk.Label(
            info_bar,
            text="● FPV TẮT",
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
            text=f"📹\n\n{message}",
            fill=THEME["subtext"],
            font=("Segoe UI", 11),
            justify=tk.CENTER
        )

    def _on_canvas_resize(self, event):
        if self.video_worker is None or not self.video_worker.running:
            self._draw_video_placeholder("Chưa mở FPV từ ESP32-S3 CAM\nNhập IP:81/stream và bấm 'MỞ FPV'")

    def toggle_video_stream(self):
        if self.video_worker and self.video_worker.running:
            self.stop_video_stream()
        else:
            self.start_video_stream()

    def start_video_stream(self):
        url = self.cb_stream_url.get().strip()
        if not url:
            messagebox.showwarning("Cảnh báo", "Vui lòng nhập URL Stream của ESP32-S3 CAM!")
            return

        self.btn_toggle_stream.config(text="⏹ DỪNG FPV", bg=THEME["danger"])
        self.lbl_stream_status.config(text="● ĐANG KẾT NỐI FPV...", fg=THEME["warning"])

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
        self.current_frame = None
        self.btn_toggle_stream.config(text="▶ MỞ FPV", bg=THEME["primary"])
        self.lbl_stream_status.config(text="● FPV ĐÃ DỪNG", fg=THEME["subtext"])
        self.lbl_fps.config(text="FPS: 0.0 | Độ phân giải: --x--")
        self._draw_video_placeholder("FPV đã dừng.\nBấm 'MỞ FPV' để kết nối lại.")

    def _on_video_frame(self, frame_bgr, fps):
        self.current_frame = frame_bgr
        self.fps_display = fps

    def _on_video_status(self, msg, is_ok):
        def _update():
            if is_ok:
                self.lbl_stream_status.config(text="● FPV LIVE", fg=THEME["success"])
            else:
                self.lbl_stream_status.config(text=f"● {msg}", fg=THEME["danger"] if "Lỗi" in msg else THEME["warning"])
        self.root.after(0, _update)

    def save_snapshot(self):
        if self.current_frame is None:
            messagebox.showinfo("Thông báo", "Chưa có khung hình FPV nào để lưu!")
            return

        snap_dir = os.path.join(os.path.dirname(__file__), "snapshots")
        os.makedirs(snap_dir, exist_ok=True)
        fname = datetime.datetime.now().strftime("fpv_%Y%m%d_%H%M%S.png")
        path = os.path.join(snap_dir, fname)

        cv2.imwrite(path, self.current_frame)
        self.log_console(f"[FPV] Đã lưu ảnh chụp: {fname}", "INFO")
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

        tk.Label(row, text="COM:", font=("Segoe UI", 9, "bold"), fg=THEME["text"], bg=THEME["card"]).pack(side=tk.LEFT)

        self.cb_com_ports = ttk.Combobox(row, width=22, state="readonly")
        self.cb_com_ports.pack(side=tk.LEFT, padx=6)

        tk.Button(
            row, text="🔄 Quét", bg=THEME["card_alt"], fg=THEME["text"],
            relief=tk.FLAT, padx=6, command=self.refresh_com_ports
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            row, text="⚡ Tự Nối", bg=THEME["primary"], fg="#11111b",
            font=("Segoe UI", 8, "bold"), relief=tk.FLAT, padx=6, command=self.auto_connect_esp32
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            row, text="📡 Ping", bg=THEME["card_alt"], fg=THEME["text"],
            relief=tk.FLAT, padx=6, command=self.ping_robot
        ).pack(side=tk.LEFT, padx=2)

        self.btn_connect = tk.Button(
            row,
            text="KẾT NỐI",
            font=("Segoe UI", 9, "bold"),
            bg=THEME["success"],
            fg="#11111b",
            relief=tk.FLAT,
            padx=12,
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

        tk.Checkbutton(
            status_row,
            text="Tự nối lại",
            variable=self.auto_reconnect_var,
            font=("Segoe UI", 8),
            fg=THEME["subtext"],
            bg=THEME["card"],
            selectcolor=THEME["card_alt"],
            activebackground=THEME["card"],
            activeforeground=THEME["text"]
        ).pack(side=tk.LEFT, padx=10)

        self.lbl_rf_link = tk.Label(
            status_row,
            text="● Sóng NRF: Chờ kết nối COM...",
            font=("Segoe UI", 9, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card"]
        )
        self.lbl_rf_link.pack(side=tk.RIGHT)

        vehicle_row = tk.Frame(card, bg=THEME["card_alt"], padx=8, pady=5)
        vehicle_row.pack(fill=tk.X, pady=(6, 0))

        tk.Label(
            vehicle_row,
            text="TRẠNG THÁI XE",
            font=("Segoe UI", 8, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card_alt"]
        ).pack(side=tk.LEFT)

        self.lbl_vehicle_state = tk.Label(
            vehicle_row,
            text=self.robot_state_str,
            font=("Segoe UI", 10, "bold"),
            fg=THEME["subtext"],
            bg=THEME["card_alt"]
        )
        self.lbl_vehicle_state.pack(side=tk.RIGHT)

    def ping_robot(self):
        if not self.serial_running or not self.serial_client:
            self.log_console("[Ping] Chưa kết nối cổng COM của ESP32!", "WARN")
            return
        self.send_serial_cmd("PING")
        self.log_console("[Ping] Đã gửi lệnh PING kiểm tra sóng sang ESP32 DevKit...", "INFO")

    def refresh_com_ports(self):
        details = get_port_details()
        labels = [d["label"] for d in details]
        self.cb_com_ports["values"] = labels

        best_idx = 0
        for i, d in enumerate(details):
            if d.get("is_esp"):
                best_idx = i
                break

        if labels:
            self.cb_com_ports.current(best_idx)
        self.log_console(f"[Serial] Tìm thấy {len(details)} cổng COM. Tự chọn: {labels[best_idx] if labels else 'Không có'}", "INFO")

    def auto_connect_esp32(self):
        self.refresh_com_ports()
        port = find_best_esp32_port()
        if not port:
            messagebox.showinfo("Tìm cổng COM", "Chưa tìm thấy cổng COM nào cắm vào máy tính!")
            return
        self.log_console(f"[Auto-Connect] Đang tự động kết nối cổng: {port}...", "INFO")
        self.connect_serial(target_port=port)


    def ping_robot(self):
        if not self.serial_running or not self.serial_client:
            self.log_console("[Ping] Chưa kết nối cổng COM của ESP32!", "WARN")
            return
        self.send_serial_cmd("PING")
        self.log_console("[Ping] Đã gửi lệnh PING kiểm tra sóng sang ESP32 DevKit...", "INFO")

    def refresh_com_ports(self):
        details = get_port_details()
        labels = [d["label"] for d in details]
        self.cb_com_ports["values"] = labels

        best_idx = 0
        for i, d in enumerate(details):
            if d.get("is_esp"):
                best_idx = i
                break

        if labels:
            self.cb_com_ports.current(best_idx)
        self.log_console(f"[Serial] Tìm thấy {len(details)} cổng COM. Tự chọn: {labels[best_idx] if labels else 'Không có'}", "INFO")

    def auto_connect_esp32(self):
        self.refresh_com_ports()
        port = find_best_esp32_port()
        if not port:
            messagebox.showinfo("Tìm cổng COM", "Chưa tìm thấy cổng COM nào cắm vào máy tính!")
            return
        self.log_console(f"[Auto-Connect] Đang tự động kết nối cổng: {port}...", "INFO")
        self.connect_serial(target_port=port)

    def toggle_serial(self):
        if self.serial_running:
            self.disconnect_serial(is_unexpected=False)
        else:
            self.connect_serial()

    def connect_serial(self, target_port=None):
        if target_port is None:
            selected = self.cb_com_ports.get().strip()
            if not selected:
                messagebox.showwarning("Cảnh báo", "Vui lòng chọn cổng COM của ESP32 DevKit V1!")
                return False
            target_port = selected.split()[0].strip()

        try:
            self.serial_client = SerialClient(port=target_port, baudrate=115200, timeout=0.1)
            self.serial_client.open()
            self.serial_running = True
            self.last_connected_port = target_port

            self.btn_connect.config(text="NGẮT KẾT NỐI", bg=THEME["danger"], fg="#ffffff")
            self.lbl_serial_badge.config(text=f"● Đã kết nối: {target_port} @ 115200", fg=THEME["success"])
            self.log_console(f"[Serial] Đã mở cổng {target_port} thành công!", "SUCCESS")

            self.serial_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
            self.serial_thread.start()
            self._set_adas_state(False)
            self._send_adas_state(log_state=True)
            self.cmd_arm()
            self.robot_state_str = "ARMED"
            self._update_vehicle_state_widget()
            self.log_console("[ARM] Đã gửi lệnh ARM mặc định sau khi kết nối.", "SUCCESS")
            self._send_adas_state(log_state=True)

            self.root.after(300, self.ping_robot)
            return True

        except Exception as e:
            self.serial_running = False
            self.lbl_serial_badge.config(text=f"● Lỗi mở cổng {target_port}", fg=THEME["danger"])
            self.log_console(f"[Serial Lỗi] {e}", "ERROR")
            if not getattr(self, 'auto_reconnecting', False):
                messagebox.showerror("Lỗi kết nối", f"Không thể mở cổng {target_port}:\n{e}")
            return False

    def disconnect_serial(self, is_unexpected=False):
        self.serial_running = False
        if self.serial_client:
            self.serial_client.close()
            self.serial_client = None

        self.btn_connect.config(text="KẾT NỐI", bg=THEME["success"], fg="#11111b")
        if is_unexpected:
            self.lbl_serial_badge.config(text="● Mất kết nối USB (Đang chờ cắm lại...)", fg=THEME["warning"])
            self.log_console("[Serial] Cổng USB bị ngắt đột ngột!", "WARN")
        else:
            self.lbl_serial_badge.config(text="● Đã ngắt kết nối", fg=THEME["subtext"])
            self.log_console("[Serial] Đã đóng cổng kết nối.", "WARN")

    def _serial_reader_loop(self):
        buffer = ""
        while self.serial_running and self.serial_client and self.serial_client.is_open:
            try:
                data = self.serial_client.read(256)
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
                    time.sleep(0.005)
            except Exception:
                if self.serial_running:
                    self.root.after(0, lambda: self.disconnect_serial(is_unexpected=True))
                break

    def _process_rx_line(self, line):
        self.log_console(f"RX: {line}", "RX")

        if "TX 1-CHIỀU" in line or "1-WAY-TX" in line:
            self.diag_nrf_ok = True
            self.lbl_rf_link.config(text="● SÓNG NRF: PHÁT 1-CHIỀU", fg=THEME["success"])
        elif "KẾT NỐI TỐT" in line or "Sóng NRF OK" in line:
            self.diag_nrf_ok = True
            self.lbl_rf_link.config(text="● SÓNG NRF: ĐÃ BẮT SÓNG XE (ACK OK)", fg=THEME["success"])
        elif "CHỜ TÍN HIỆU XE" in line or "Chưa nhận được ACK" in line:
            self.diag_nrf_ok = False
            self.lbl_rf_link.config(text="● SÓNG NRF: ĐANG TÌM XE", fg=THEME["warning"])
        elif "ACK: ARM" in line:
            self.robot_state_str = "ARMED"
            self._update_vehicle_state_widget()
        elif "ACK: DISARM" in line:
            self.robot_state_str = "DISARMED"
            self._update_vehicle_state_widget()
        elif "ACK: ESTOP" in line:
            self.robot_state_str = "FAULT"
            self._update_vehicle_state_widget()
        elif "ACK: ADAS ON" in line:
            self._set_adas_state(True)
        elif "ACK: ADAS OFF" in line:
            self._set_adas_state(False)
        elif "PONG:" in line:
            if "RADIO_OK=1" in line:
                self.lbl_rf_link.config(text="● NRF24 ESP32: SẴN SÀNG", fg=THEME["primary"])
            else:
                self.lbl_rf_link.config(text="● NRF24: LỖI PHẦN CỨNG", fg=THEME["danger"])

        adas_match = re.search(r'"adas"\s*:\s*(true|false)', line, re.IGNORECASE)
        if adas_match:
            self._set_adas_state(adas_match.group(1).lower() == "true")

        if "Telemetry" in line or "Pitch=" in line:
            self.diag_nrf_ok = True
            m_state = re.search(r"State=(\d+)", line)
            m_pitch = re.search(r"Pitch=([\+\-\d\.]+)", line)
            m_dist = re.search(r"Dist=(\d+)", line)
            m_bat = re.search(r"Bat=([\d\.]+)", line)
            m_mpu = re.search(r"MPU=(\d+)", line)
            m_nrf = re.search(r"NRF=(\d+)", line)
            m_spm = re.search(r"SPM=(\d+)", line)
            m_upr = re.search(r"Upright=(\d+)", line)
            m_err = re.search(r"ImuErr=(\d+)", line)

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

            if m_mpu:
                self.diag_mpu_ok = (int(m_mpu.group(1)) == 1)
            if m_nrf:
                self.diag_nrf_ok = (int(m_nrf.group(1)) == 1)
            if m_spm:
                self.diag_spm_ok = (int(m_spm.group(1)) == 1)
            if m_upr:
                self.diag_upright = (int(m_upr.group(1)) == 1)
            if m_err:
                self.diag_imu_err = int(m_err.group(1))

            self._update_vehicle_state_widget()

    def _update_vehicle_state_widget(self):
        st_colors = {
            "ARMED": THEME["success"],
            "DISARMED": THEME["warning"],
            "CALIBRATING": THEME["accent"],
            "BOOT": THEME["primary"],
            "FAULT": THEME["danger"],
        }
        col = st_colors.get(self.robot_state_str, THEME["subtext"])
        self.lbl_vehicle_state.config(text=self.robot_state_str, fg=col)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 4: ĐIỀU KHIỂN XE (NÚT BẤM + WEBCAM LÁI TAY)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_controls_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🎮 Điều Khiển Xe (Phím: W/A/S/D/Space/X/Z/E/F2 hoặc Webcam Lái Tay) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=6
        )
        card.pack(fill=tk.X, pady=(0, 8))

        # Hàng 1: ARM, DISARM, ESTOP, LÁI TAY
        act_row = tk.Frame(card, bg=THEME["card"])
        act_row.pack(fill=tk.X, pady=(0, 6))

        tk.Button(
            act_row, text="🟢 ARM (X)", font=("Segoe UI", 9, "bold"),
            bg="#2e7d32", fg="#ffffff", activebackground=THEME["success"],
            relief=tk.FLAT, padx=8, pady=5, command=self.cmd_arm
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            act_row, text="🟡 DISARM (Z)", font=("Segoe UI", 9, "bold"),
            bg="#f57f17", fg="#11111b", activebackground=THEME["warning"],
            relief=tk.FLAT, padx=8, pady=5, command=self.cmd_disarm
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        tk.Button(
            act_row, text="🛑 ESTOP (E)", font=("Segoe UI", 9, "bold"),
            bg="#c62828", fg="#ffffff", activebackground=THEME["danger"],
            relief=tk.FLAT, padx=8, pady=5, command=self.cmd_estop
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_hand_mode = tk.Button(
            act_row, text="✋ LÁI TAY: TẮT", font=("Segoe UI", 9, "bold"),
            bg=THEME["card_alt"], fg=THEME["text"],
            relief=tk.FLAT, padx=8, pady=5, command=self.toggle_hand_mode
        )
        self.btn_hand_mode.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.btn_adas = tk.Button(
            act_row,
            font=("Segoe UI", 9, "bold"),
            relief=tk.FLAT,
            padx=8,
            pady=5,
            command=self.toggle_adas
        )
        self.btn_adas.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        self._set_adas_state(self.adas_enabled)

        gamepad_row = tk.Frame(card, bg=THEME["card"])
        gamepad_row.pack(fill=tk.X, pady=(0, 6))
        self.btn_gamepad = tk.Button(
            gamepad_row,
            text="🎮 TAY CẦM USB: TẮT",
            font=("Segoe UI", 9, "bold"),
            bg=THEME["card_alt"],
            fg=THEME["text"],
            relief=tk.FLAT,
            padx=8,
            pady=4,
            command=self.toggle_gamepad
        )
        self.btn_gamepad.pack(side=tk.LEFT, padx=(0, 8))
        self.lbl_gamepad_status = tk.Label(
            gamepad_row,
            text="Joystick trái: tiến/lùi | Joystick phải: rẽ | Nút 1 ARM/DISARM, 2 STOP, 3 ESTOP",
            font=("Segoe UI", 8),
            fg=THEME["subtext"],
            bg=THEME["card"]
        )
        self.lbl_gamepad_status.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Hàng 2: D-Pad + Thanh trượt + Khung xem trước Webcam lái tay
        dpad_frame = tk.Frame(card, bg=THEME["card"])
        dpad_frame.pack(fill=tk.X)

        pad = tk.Frame(dpad_frame, bg=THEME["card"])
        pad.pack(side=tk.LEFT, padx=(0, 6))

        btn_fwd = tk.Button(pad, text="▲ Tiến (W)", width=9, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT)
        btn_fwd.grid(row=0, column=1, pady=2)
        self._bind_momentary_button(btn_fwd, "button:fwd", "forward")

        btn_left = tk.Button(pad, text="◄ Trái (A)", width=9, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT)
        btn_left.grid(row=1, column=0, padx=2)
        self._bind_momentary_button(btn_left, "button:left", "left")

        btn_stop = tk.Button(pad, text="■ STOP", width=10, bg="#37474f", fg="#ffffff", relief=tk.FLAT)
        btn_stop.grid(row=1, column=1, padx=2)
        btn_stop.bind("<ButtonPress-1>", lambda _event: self._press_stop_input())
        btn_stop.bind("<ButtonRelease-1>", lambda _event: self._release_stop_input())

        btn_right = tk.Button(pad, text="► Phải (D)", width=9, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT)
        btn_right.grid(row=1, column=2, padx=2)
        self._bind_momentary_button(btn_right, "button:right", "right")

        btn_bwd = tk.Button(pad, text="▼ Lùi (S)", width=9, bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT)
        btn_bwd.grid(row=2, column=1, pady=2)
        self._bind_momentary_button(btn_bwd, "button:bwd", "backward")

        sliders = tk.Frame(dpad_frame, bg=THEME["card"])
        sliders.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)

        tk.Label(sliders, text="Tốc độ đặt:", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_speed_step = tk.Scale(
            sliders, from_=50, to=400, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b",
            command=self._update_speed_limit
        )
        self.slider_speed_step.set(150)
        self.slider_speed_step.pack(fill=tk.X, pady=(0, 2))

        tk.Label(sliders, text="Độ nhạy rẽ:", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_steer_step = tk.Scale(
            sliders, from_=50, to=500, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b",
            command=self._update_steer_limit
        )
        self.slider_steer_step.set(200)
        self.slider_steer_step.pack(fill=tk.X)

        # Khung mini preview Webcam nhận diện bàn tay
        hand_box = tk.Frame(dpad_frame, bg=THEME["card_alt"], bd=1, relief=tk.SOLID)
        hand_box.pack(side=tk.RIGHT, padx=(4, 0))

        self.hand_canvas = tk.Canvas(hand_box, width=148, height=110, bg="#0b0b12", highlightthickness=0)
        self.hand_canvas.pack()
        self._draw_hand_placeholder("Webcam Lái Tay\n(Đang tắt)")

    def _draw_hand_placeholder(self, text):
        self.hand_canvas.delete("all")
        self.hand_canvas.create_text(
            74, 55, text=text, fill=THEME["subtext"], font=("Segoe UI", 8), justify=tk.CENTER
        )

    def _update_speed_limit(self, value):
        with self._control_limits_lock:
            self._control_limits = (int(float(value)), self._control_limits[1])

    def _update_steer_limit(self, value):
        with self._control_limits_lock:
            self._control_limits = (self._control_limits[0], int(float(value)))

    def _get_control_limits(self):
        with self._control_limits_lock:
            return self._control_limits

    def toggle_hand_mode(self):
        if self.hand_mode_enabled:
            self.stop_hand_mode()
        else:
            self.start_hand_mode()

    def toggle_gamepad(self):
        if self.gamepad_enabled:
            was_connected = self.gamepad_connected
            self.gamepad_enabled = False
            if self._gamepad_poll_after_id is not None:
                self.root.after_cancel(self._gamepad_poll_after_id)
                self._gamepad_poll_after_id = None
            self._close_gamepad(shutdown=True)
            if was_connected:
                self.cmd_stop()
            self.btn_gamepad.config(text="🎮 TAY CẦM USB: TẮT", bg=THEME["card_alt"], fg=THEME["text"])
            self.lbl_gamepad_status.config(text="Tay cầm đã tắt; xe dừng lệnh di chuyển và giữ trạng thái ARM.", fg=THEME["subtext"])
            return

        if pygame is None:
            self.lbl_gamepad_status.config(text="Thiếu pygame; cài dependencies trong requirements.txt.", fg=THEME["danger"])
            self.log_console("[Gamepad Lỗi] Không import được pygame. Cài App_python/requirements.txt.", "ERROR")
            return

        if self.hand_mode_enabled:
            self.stop_hand_mode()

        self._cancel_drive_timers()
        self._active_drive_inputs.clear()
        self._reset_drive_axis_state()
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd("DRIVE 0 0")

        try:
            # The dummy display backend keeps SDL event pumping separate from OpenCV's camera backend.
            pygame.display.init()
            pygame.joystick.init()
            self.gamepad_enabled = True
            self.gamepad_last_scan_ms = 0.0
            self.btn_gamepad.config(text="🎮 TAY CẦM USB: BẬT", bg=THEME["accent"], fg="#11111b")
            self.lbl_gamepad_status.config(text="Đang tìm tay cầm USB...", fg=THEME["warning"])
            self._schedule_gamepad_poll()
        except pygame.error as exc:
            self.gamepad_enabled = False
            self._close_gamepad(shutdown=True)
            self.lbl_gamepad_status.config(text=f"Lỗi khởi tạo tay cầm: {exc}", fg=THEME["danger"])
            self.log_console(f"[Gamepad Lỗi] Không khởi tạo được pygame joystick: {exc}", "ERROR")

    def _close_gamepad(self, shutdown=False):
        if self.gamepad is not None:
            try:
                self.gamepad.quit()
            except pygame.error:
                pass
            self.gamepad = None
        if shutdown and pygame is not None:
            try:
                pygame.joystick.quit()
                pygame.display.quit()
            except pygame.error:
                pass
        self.gamepad_connected = False
        self.gamepad_buttons = [False, False, False, False]
        self.gamepad_ignore_button_edges = True
        self.gamepad_last_drive = 0
        self.gamepad_last_steer = 0
        self.gamepad_drive_baseline = 0.0
        self.gamepad_steer_baseline = 0.0
        self.gamepad_baseline_ready = False
        self.gamepad_last_drive = None
        self.gamepad_last_steer = None

    def _handle_gamepad_disconnect(self):
        self.gamepad_enabled = False
        self._close_gamepad(shutdown=True)
        if self._gamepad_poll_after_id is not None:
            self.root.after_cancel(self._gamepad_poll_after_id)
            self._gamepad_poll_after_id = None
        self.btn_gamepad.config(text="🎮 TAY CẦM USB: TẮT", bg=THEME["card_alt"], fg=THEME["text"])
        self.cmd_stop()
        if self.serial_running:
            status = "Mất tay cầm; GCS còn kết nối. Xe đã dừng lệnh joystick, có thể chuyển sang webcam/phím."
            self.lbl_gamepad_status.config(text=status, fg=THEME["warning"])
            self.log_console("[Gamepad] Mất tay cầm; đã gửi STOP, giữ trạng thái ARM để chuyển sang điều khiển GCS khác.", "WARN")
        else:
            status = "Mất tay cầm và mất Serial GCS; không thể gửi lệnh chuyển tiếp."
            self.lbl_gamepad_status.config(text=status, fg=THEME["danger"])
            self.log_console("[Gamepad] Mất tay cầm khi Serial GCS không kết nối; không gửi được lệnh STOP.", "ERROR")

    def _schedule_gamepad_poll(self):
        if self.gamepad_enabled and self._gamepad_poll_after_id is None:
            self._gamepad_poll_after_id = self.root.after(GAMEPAD_POLL_MS, self._gamepad_poll_tick)

    def _gamepad_poll_tick(self):
        self._gamepad_poll_after_id = None
        if not self.gamepad_enabled:
            return
        self._poll_gamepad()
        self._schedule_gamepad_poll()

    def _poll_gamepad(self):
        if not self.gamepad_enabled or pygame is None:
            return

        try:
            pygame.event.pump()
            if self.gamepad is None:
                now_ms = time.monotonic() * 1000.0
                if (now_ms - self.gamepad_last_scan_ms) < GAMEPAD_RECONNECT_MS:
                    return
                self.gamepad_last_scan_ms = now_ms
                if pygame.joystick.get_count() == 0:
                    self.lbl_gamepad_status.config(text="Chưa tìm thấy tay cầm USB; đang tìm lại...", fg=THEME["warning"])
                    return

                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()
                self.gamepad_connected = True
                self.gamepad_buttons = [False, False, False, False]
                self.gamepad_ignore_button_edges = True
                self.gamepad_last_drive = 0
                self.gamepad_last_steer = 0
                self.gamepad_last_tx_ms = 0.0
                axis_count = self.gamepad.get_numaxes()
                baseline_drive = -self.gamepad.get_axis(1) if axis_count > 1 else 0.0
                baseline_steer = self.gamepad.get_axis(2) if axis_count > 2 else 0.0
                self.gamepad_drive_baseline = baseline_drive
                self.gamepad_steer_baseline = baseline_steer
                self.gamepad_baseline_ready = True
                self.gamepad_last_drive = None
                self.gamepad_last_steer = None
                self.gamepad_last_tx_ms = 0.0
                name = self.gamepad.get_name()
                self.lbl_gamepad_status.config(text=f"Đã kết nối: {name} | Trục Y trái: tiến/lùi, trục X phải: rẽ", fg=THEME["success"])
                self.log_console(f"[Gamepad] Đã kết nối tay cầm USB: {name}", "SUCCESS")

            if pygame.joystick.get_count() == 0 or not self.gamepad.get_init():
                self._handle_gamepad_disconnect()
                return

            button_count = self.gamepad.get_numbuttons()
            buttons = [
                (index < button_count and bool(self.gamepad.get_button(index)))
                for index in range(4)
            ]
            if self.gamepad_ignore_button_edges:
                pressed = [False, False, False, False]
                self.gamepad_ignore_button_edges = False
            else:
                pressed = [buttons[i] and not self.gamepad_buttons[i] for i in range(4)]
            self.gamepad_buttons = buttons

            if pressed[0]:
                if self.robot_state_str == "ARMED":
                    self.cmd_disarm()
                elif self.robot_state_str == "DISARMED":
                    self.cmd_arm()
                else:
                    self.log_console(f"[Gamepad] ARM/DISARM không khả dụng khi trạng thái là {self.robot_state_str}.", "WARN")
            if pressed[1]:
                self.cmd_stop()
            if pressed[2]:
                self.cmd_estop()

            if buttons[1] or self._stop_input_active:
                drive_cmd = 0
                steer_cmd = 0
            else:
                axis_count = self.gamepad.get_numaxes()
                raw_drive_axis = -self.gamepad.get_axis(1) if axis_count > 1 else 0.0
                raw_steer_axis = self.gamepad.get_axis(2) if axis_count > 2 else 0.0

                if self.gamepad_baseline_ready:
                    drive_delta = raw_drive_axis - self.gamepad_drive_baseline
                    steer_delta = raw_steer_axis - self.gamepad_steer_baseline
                else:
                    drive_delta = raw_drive_axis
                    steer_delta = raw_steer_axis

                drive_delta = max(-1.0, min(1.0, drive_delta))
                steer_delta = max(-1.0, min(1.0, steer_delta))
                drive_axis = drive_delta if abs(drive_delta) >= GAMEPAD_DEADZONE else 0.0
                steer_axis = steer_delta if abs(steer_delta) >= GAMEPAD_DEADZONE else 0.0
                drive_axis = -self.gamepad.get_axis(1) if axis_count > 1 else 0.0
                steer_axis = self.gamepad.get_axis(2) if axis_count > 2 else 0.0
                if abs(drive_axis) < GAMEPAD_DEADZONE:
                    drive_axis = 0.0
                if abs(steer_axis) < GAMEPAD_DEADZONE:
                    steer_axis = 0.0
                drive_cmd = int(round(drive_axis * int(self.slider_speed_step.get())))
                steer_cmd = int(round(steer_axis * int(self.slider_steer_step.get())))

            if self.robot_state_str != "ARMED":
                drive_cmd = 0
                steer_cmd = 0

            now_ms = time.monotonic() * 1000.0
            changed = (drive_cmd != self.gamepad_last_drive or
                       steer_cmd != self.gamepad_last_steer)
            gamepad_active = (drive_cmd != 0 or steer_cmd != 0)
            should_send = changed
            if gamepad_active and (now_ms - self.gamepad_last_tx_ms) >= DRIVE_REPEAT_MS:
                should_send = True

            if should_send:
                self.drive_cmd = drive_cmd
                self.steer_cmd = steer_cmd
                if self.serial_running and (gamepad_active or changed):
            if changed or (now_ms - self.gamepad_last_tx_ms) >= DRIVE_REPEAT_MS:
                self.drive_cmd = drive_cmd
                self.steer_cmd = steer_cmd
                if self.serial_running:
                    self.send_serial_cmd(f"DRIVE {drive_cmd} {steer_cmd}")
                self.gamepad_last_drive = drive_cmd
                self.gamepad_last_steer = steer_cmd
                self.gamepad_last_tx_ms = now_ms
        except pygame.error as exc:
            if self.gamepad_connected:
                self.log_console(f"[Gamepad Lỗi] Tay cầm không còn khả dụng: {exc}", "WARN")
                self._handle_gamepad_disconnect()
            else:
                self.gamepad_enabled = False
                self._close_gamepad(shutdown=True)
                self.lbl_gamepad_status.config(text=f"Lỗi đọc tay cầm: {exc}", fg=THEME["danger"])
                self.log_console(f"[Gamepad Lỗi] Không đọc được joystick: {exc}", "ERROR")

    def start_hand_mode(self):
        if self.gamepad_enabled:
            was_connected = self.gamepad_connected
            self.gamepad_enabled = False
            if self._gamepad_poll_after_id is not None:
                self.root.after_cancel(self._gamepad_poll_after_id)
                self._gamepad_poll_after_id = None
            self._close_gamepad(shutdown=True)
            self._cancel_drive_timers()
            self._active_drive_inputs.clear()
            self._reset_drive_axis_state()
            self.drive_cmd = 0
            self.steer_cmd = 0
            if self.serial_running:
                self.send_serial_cmd("DRIVE 0 0")
            self.btn_gamepad.config(text="🎮 TAY CẦM USB: TẮT", bg=THEME["card_alt"], fg=THEME["text"])
            status = "Đã chuyển sang webcam; tay cầm đã ngắt điều khiển."
            self.lbl_gamepad_status.config(text=status, fg=THEME["subtext"])
            self.log_console(
                "[Gesture] Chuyển nguồn điều khiển từ tay cầm sang webcam"
                + ("." if was_connected else " (tay cầm chưa kết nối)."),
                "INFO"
            )
        self.hand_mode_enabled = True
        self._hand_was_active = False
        self.btn_hand_mode.config(text="✋ LÁI TAY: BẬT", bg=THEME["accent"], fg="#11111b")
        self._draw_hand_placeholder("Đang mở Webcam...")

        self.hand_worker = HandGestureWorker(
            cam_index=0,
            get_limits_fn=self._get_control_limits,
            on_control=self._on_hand_control,
            on_frame=self._on_hand_frame,
            on_status=self._on_hand_status
        )
        self._hand_preview_error_reported = False
        self.hand_worker.start()

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
        self.log_console("[Gesture] Đã TẮT chế độ lái bằng tay.", "WARN")

    def _on_hand_frame(self, frame_bgr):
        self.hand_frame = frame_bgr

    def _on_hand_status(self, msg, is_ok):
        def _update():
            if is_ok:
                self.log_console(f"[Gesture] {msg}", "SUCCESS")
            else:
                self.log_console(f"[Gesture Lỗi] {msg}", "ERROR")
                self.stop_hand_mode()
        self.root.after(0, _update)

    def _on_hand_control(self, drive_val, steer_val, hand_found):
        if not self.hand_mode_enabled:
            return
        if self._stop_input_active:
            return
        if hand_found:
            self._hand_was_active = True
            self.drive_cmd = drive_val
            self.steer_cmd = steer_val
            self.root.after(0, self._dispatch_drive)
        elif self._hand_was_active:
            self._hand_was_active = False
            self.root.after(0, self.cmd_stop)

    # ──────────────────────────────────────────────────────────────────────────
    # BÀN PHÍM AN TOÀN (KHÔNG KÍCH HOẠT NHẦM KHI GÕ VĂN BẢN)
    # ──────────────────────────────────────────────────────────────────────────
    def _bind_momentary_button(self, button, token, direction):
        button.bind(
            "<ButtonPress-1>",
            lambda _event, input_token=token, movement=direction:
                self._start_drive_input(input_token, movement)
        )
        button.bind(
            "<ButtonRelease-1>",
            lambda _event, input_token=token:
                self._release_drive_input(input_token)
        )

    def _bind_keyboard_controls(self):
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)

    def _on_key_press(self, event):
        if isinstance(event.widget, (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text)):
            return

        key = event.keysym.lower()
        if key in self._pressed_keys:
            return
        self._pressed_keys.add(key)

        movement = {
            "w": "forward",
            "s": "backward",
            "a": "left",
            "d": "right",
        }.get(key)
        if movement:
            self._start_drive_input(f"key:{key}", movement)
        elif key in ("space", "r"):
            self._press_stop_input()
        elif key == "x":
            self.cmd_arm()
        elif key == "z":
            self.cmd_disarm()
        elif key == "e":
            self.cmd_estop()
        elif key == "f2":
            self.toggle_adas()

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self._pressed_keys.discard(key)
        if key in ("w", "s", "a", "d"):
            self._release_drive_input(f"key:{key}")
        elif key in ("space", "r"):
            self._release_stop_input()

    def send_serial_cmd(self, cmd_text):
        if not self.serial_client or not self.serial_client.is_open:
            self.log_console("[Lỗi gửi] Chưa kết nối ESP32 DevKit V1!", "WARN")
            return
        try:
            self.serial_client.write_line(cmd_text)
            self.tx_count += 1
            self.log_console(f"TX: {cmd_text}", "TX")
        except Exception as e:
            self.log_console(f"[Lỗi gửi Serial] {e}", "ERROR")
            self.disconnect_serial(is_unexpected=True)

    def cmd_arm(self):
        self.send_serial_cmd("ARM")

    def cmd_disarm(self):
        self._cancel_drive_timers()
        self._active_drive_inputs.clear()
        self._reset_drive_axis_state()
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd("DISARM")

    def cmd_estop(self):
        self._cancel_drive_timers()
        self._active_drive_inputs.clear()
        self._reset_drive_axis_state()
        self.drive_cmd = 0
        self.steer_cmd = 0
        self.send_serial_cmd("ESTOP")

    def _set_adas_state(self, enabled):
        self.adas_enabled = bool(enabled)
        if not hasattr(self, "btn_adas"):
            return
        if self.adas_enabled:
            self.btn_adas.config(
                text="ADAS: ON (An toàn)",
                bg="#2e7d32",
                fg="#ffffff",
                activebackground=THEME["success"]
            )
        else:
            self.btn_adas.config(
                text="ADAS: OFF (Chế độ Đua)",
                bg="#c65d0e",
                fg="#ffffff",
                activebackground=THEME["warning"]
            )

    def _send_adas_state(self, log_state=False):
        self.send_serial_cmd(f"ADAS:{1 if self.adas_enabled else 0}")
        if log_state:
            self._log_adas_state()

    def _log_adas_state(self):
        if self.adas_enabled:
            self.log_console("[ADAS] Đã BẬT tránh vật cản", "SUCCESS")
        else:
            self.log_console("[ADAS] Đã TẮT tránh vật cản - Chế độ Đua", "WARN")

    def toggle_adas(self):
        self._set_adas_state(not self.adas_enabled)
        self._send_adas_state(log_state=True)

    def _cancel_drive_timers(self):
        if self._drive_repeat_after_id is not None:
            self.root.after_cancel(self._drive_repeat_after_id)
            self._drive_repeat_after_id = None

    def _reset_drive_axis_state(self):
        for axis in ("drive", "steer"):
            self._drive_axis_active[axis] = False
            self._drive_axis_started_at[axis] = None
            self._drive_axis_deadline[axis] = None
            self._drive_axis_hold_value[axis] = 0

    def _press_stop_input(self):
        self._stop_input_active = True
        self.cmd_stop()

    def _release_stop_input(self):
        self._stop_input_active = False

    def _set_drive_from_active_inputs(self):
        directions = set(self._active_drive_inputs.values())
        speed = int(self.slider_speed_step.get())
        steer_step = int(self.slider_steer_step.get())
        desired = {
            "drive": (int("forward" in directions) - int("backward" in directions)) * speed,
            "steer": (int("right" in directions) - int("left" in directions)) * steer_step,
        }
        requested = {
            "drive": "forward" in directions or "backward" in directions,
            "steer": "right" in directions or "left" in directions,
        }
        minimum_ms = {"drive": MIN_PULSE_DRIVE_MS, "steer": MIN_PULSE_STEER_MS}
        now = time.monotonic()

        for axis, attr in (("drive", "drive_cmd"), ("steer", "steer_cmd")):
            current_value = getattr(self, attr)
            desired_value = desired[axis]
            deadline = self._drive_axis_deadline[axis]

            if requested[axis]:
                held_value = self._drive_axis_hold_value[axis]
                if desired_value == 0:
                    self._drive_axis_started_at[axis] = None
                    self._drive_axis_deadline[axis] = None
                    self._drive_axis_hold_value[axis] = 0
                elif not self._drive_axis_active[axis]:
                    if deadline is None or (held_value != 0 and held_value * desired_value < 0):
                        self._drive_axis_started_at[axis] = now
                    self._drive_axis_deadline[axis] = None
                    self._drive_axis_hold_value[axis] = 0
                elif self._drive_axis_started_at[axis] is None or current_value * desired_value < 0:
                    self._drive_axis_started_at[axis] = now

                setattr(self, attr, desired_value)
                self._drive_axis_active[axis] = True
                continue

            if self._drive_axis_active[axis]:
                self._drive_axis_active[axis] = False
                started_at = self._drive_axis_started_at[axis]
                if current_value != 0 and started_at is not None:
                    deadline = started_at + minimum_ms[axis] / 1000.0
                    if now < deadline:
                        self._drive_axis_deadline[axis] = deadline
                        self._drive_axis_hold_value[axis] = current_value
                        setattr(self, attr, current_value)
                        continue

            if deadline is not None and now < deadline:
                setattr(self, attr, self._drive_axis_hold_value[axis])
            else:
                self._drive_axis_started_at[axis] = None
                self._drive_axis_deadline[axis] = None
                self._drive_axis_hold_value[axis] = 0
                setattr(self, attr, 0)

    def _send_drive_command(self):
        self._set_drive_from_active_inputs()
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def _start_drive_input(self, token, direction):
        if self.gamepad_enabled or self._stop_input_active or token in self._active_drive_inputs:
            return

        self._active_drive_inputs[token] = direction
        self._send_drive_command()
        self._schedule_drive_repeat()

    def _repeat_drive_command(self):
        self._drive_repeat_after_id = None
        if not self._active_drive_inputs and not any(self._drive_axis_deadline.values()):
            return
        self._send_drive_command()
        self._schedule_drive_repeat()

    def _schedule_drive_repeat(self):
        if self._drive_repeat_after_id is not None:
            self.root.after_cancel(self._drive_repeat_after_id)
            self._drive_repeat_after_id = None

        deadlines = [deadline for deadline in self._drive_axis_deadline.values() if deadline is not None]
        if not self._active_drive_inputs and not deadlines:
            return

        delay_ms = DRIVE_REPEAT_MS
        if not self._active_drive_inputs:
            remaining_ms = max(1, int((min(deadlines) - time.monotonic()) * 1000.0 + 0.999))
            delay_ms = min(delay_ms, remaining_ms)
        self._drive_repeat_after_id = self.root.after(delay_ms, self._repeat_drive_command)


    def _send_drive_command(self):
        self._set_drive_from_active_inputs()
        self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def _start_drive_input(self, token, direction):
        if self.gamepad_enabled or self._stop_input_active or token in self._active_drive_inputs:
            return

        self._active_drive_inputs[token] = direction
        self._send_drive_command()
        self._schedule_drive_repeat()

    def _repeat_drive_command(self):
        self._drive_repeat_after_id = None
        if not self._active_drive_inputs and not any(self._drive_axis_deadline.values()):
            return
        self._send_drive_command()
        self._schedule_drive_repeat()

    def _schedule_drive_repeat(self):
        if self._drive_repeat_after_id is not None:
            self.root.after_cancel(self._drive_repeat_after_id)
            self._drive_repeat_after_id = None

        deadlines = [deadline for deadline in self._drive_axis_deadline.values() if deadline is not None]
        if not self._active_drive_inputs and not deadlines:
            return

        delay_ms = DRIVE_REPEAT_MS
        if not self._active_drive_inputs:
            remaining_ms = max(1, int((min(deadlines) - time.monotonic()) * 1000.0 + 0.999))
            delay_ms = min(delay_ms, remaining_ms)
        self._drive_repeat_after_id = self.root.after(delay_ms, self._repeat_drive_command)

    def _release_drive_input(self, token):
        if token not in self._active_drive_inputs:
            return
        del self._active_drive_inputs[token]
        self._send_drive_command()
        self._schedule_drive_repeat()

    def _dispatch_drive(self):
        now_ms = time.time() * 1000
        if (now_ms - self._last_drive_tx_ms) >= 30:
            self._last_drive_tx_ms = now_ms
            self.send_serial_cmd(f"DRIVE {self.drive_cmd} {self.steer_cmd}")

    def cmd_stop(self):
        self._cancel_drive_timers()
        self._active_drive_inputs.clear()
        self._reset_drive_axis_state()
        self.drive_cmd = 0
        self.steer_cmd = 0
        self._last_drive_tx_ms = 0
        self.send_serial_cmd("DRIVE 0 0")

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
            height=7,
            wrap=tk.WORD,
            bd=0,
            state=tk.DISABLED
        )
        self.txt_console.pack(fill=tk.BOTH, expand=True)

        self.txt_console.tag_config("TX", foreground=THEME["primary"])
        self.txt_console.tag_config("RX", foreground=THEME["success"])
        self.txt_console.tag_config("WARN", foreground=THEME["warning"])
        self.txt_console.tag_config("ERROR", foreground=THEME["danger"])
        self.txt_console.tag_config("INFO", foreground=THEME["subtext"])
        self.txt_console.tag_config("SUCCESS", foreground=THEME["success"])

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

        tk.Button(
            input_frame, text="GỬI", font=("Segoe UI", 8, "bold"),
            bg=THEME["primary"], fg="#11111b", relief=tk.FLAT, padx=10,
            command=self._send_manual_command
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            input_frame, text="XÓA", font=("Segoe UI", 8),
            bg=THEME["card_alt"], fg=THEME["subtext"], relief=tk.FLAT, padx=8,
            command=self._clear_console
        ).pack(side=tk.LEFT, padx=2)

    def _clear_console(self):
        self.txt_console.config(state=tk.NORMAL)
        self.txt_console.delete("1.0", tk.END)
        self.txt_console.config(state=tk.DISABLED)

    def _send_manual_command(self):
        cmd = self.entry_cmd.get().strip()
        if cmd:
            self.send_serial_cmd(cmd)
            self.entry_cmd.delete(0, tk.END)

    def log_console(self, text, tag="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{ts}] {text}\n"
        self.txt_console.config(state=tk.NORMAL)
        self.txt_console.insert(tk.END, line, tag)
        self.txt_console.see(tk.END)
        self.txt_console.config(state=tk.DISABLED)

    def _build_status_bar(self):
        status_bar = tk.Frame(self.root, bg=THEME["panel"], height=24, padx=12, pady=2)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

        tk.Label(
            status_bar,
            text="Hệ thống sẵn sàng | Lưu ý an toàn: Nhấc 2 bánh xe khỏi mặt đất khi thử nghiệm lần đầu!",
            font=("Segoe UI", 9),
            fg=THEME["subtext"],
            bg=THEME["panel"]
        ).pack(side=tk.LEFT)

        self.lbl_packets_stat = tk.Label(
            status_bar,
            text="TX: 0 | RX: 0",
            font=("Consolas", 9),
            fg=THEME["accent"],
            bg=THEME["panel"]
        )
        self.lbl_packets_stat.pack(side=tk.RIGHT)

    # ──────────────────────────────────────────────────────────────────────────
    # VÒNG LẶP CẬP NHẬT GIAO DIỆN CHÍNH (~30 FPS)
    # ──────────────────────────────────────────────────────────────────────────
    def _ui_periodic_update(self):
        now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.lbl_system_clock.config(text=now_str)
        self.lbl_packets_stat.config(text=f"TX: {self.tx_count} | RX: {self.rx_count}")
        # Tự động kết nối lại cổng COM nếu bị rút cáp và cắm lại
        if not self.serial_running and self.auto_reconnect_var.get() and self.last_connected_port:
            now_ms = time.time() * 1000
            if (now_ms - self._last_reconnect_attempt_ms) >= 2000:
                self._last_reconnect_attempt_ms = now_ms
                if self.last_connected_port in list_com_ports():
                    self.auto_reconnecting = True
                    self.connect_serial(self.last_connected_port)
                    self.auto_reconnecting = False

        # Cập nhật khung hình FPV chính (ESP32-S3 CAM)
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
                    self.photo_img = ImageTk.PhotoImage(image=Image.fromarray(rgb_frame))

                    self.video_canvas.delete("all")
                    self.video_canvas.create_image(
                        (w_can - new_w) // 2,
                        (h_can - new_h) // 2,
                        anchor=tk.NW,
                        image=self.photo_img
                    )
                    self.lbl_fps.config(text=f"FPS: {self.fps_display:.1f} | Kích thước: {orig_w}x{orig_h}")
            except Exception:
                pass

        # Cập nhật khung hình mini Webcam lái tay (nếu đang bật)
        if self.hand_mode_enabled and self.hand_frame is not None:
            try:
                resized_hand = cv2.resize(self.hand_frame, (148, 110), interpolation=cv2.INTER_LINEAR)
                rgb_hand = cv2.cvtColor(resized_hand, cv2.COLOR_BGR2RGB)
                self.hand_photo_img = ImageTk.PhotoImage(image=Image.fromarray(rgb_hand))
                self.hand_canvas.delete("all")
                self.hand_canvas.create_image(0, 0, anchor=tk.NW, image=self.hand_photo_img)
                self._hand_preview_error_reported = False
            except Exception:
                if not self._hand_preview_error_reported:
                    traceback.print_exc()
                    self.log_console("[Webcam Lỗi] Không hiển thị được khung hình webcam.", "ERROR")
                    self._hand_preview_error_reported = True

        self.root.after(16, self._ui_periodic_update)

    def on_closing(self):
        if messagebox.askokcancel("Thoát", "Bạn có chắc muốn đóng Ground Control Station?"):
            if self.gamepad_connected:
                self.cmd_stop()
                self.cmd_disarm()
            self.gamepad_enabled = False
            if self._gamepad_poll_after_id is not None:
                self.root.after_cancel(self._gamepad_poll_after_id)
                self._gamepad_poll_after_id = None
            self._close_gamepad(shutdown=True)
            self.stop_hand_mode()
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