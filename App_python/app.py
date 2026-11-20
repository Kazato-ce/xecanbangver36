#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  MÁY TRẠM ĐIỀU KHIỂN & GIÁM SÁT XE CÂN BẰNG 2 BÁNH STM32F405 (GCS)
================================================================================
Tác vụ chính:
 1. Kết nối Serial COM siêu tốc (115200 baud) qua ESP32 DevKit V1 NRF24 Dongle.
 2. Điều khiển xe 8 HƯỚNG DI CHUYỂN: Tiến, Lùi, Trái, Phải, Tiến-Trái, Tiến-Phải,
    Lùi-Trái, Lùi-Phải (Hỗ trợ bàn phím W/A/S/D/Q/E/Z/C/Space/X).
 3. Giám sát Telemetry thời gian thực: Góc Pitch, Tốc độ góc Gyro, Pin 4S, Vật cản.
 4. Bảng điều khiển tối giản, loại bỏ hoàn toàn các khối video nặng nề, khởi động
    ngay tức thì không cần cài OpenCV/Pillow.
================================================================================
"""

import os
import sys
import time
import re
import datetime
import threading
import tkinter as tk
from tkinter import ttk, messagebox

# Import driver serial tùy biến (tương thích cả pyserial lẫn Windows ctypes gốc)
try:
    from serial_driver import SerialClient, list_com_ports, get_port_details, find_best_esp32_port
except ImportError:
    try:
        from tools.python_ground_station.serial_driver import SerialClient, list_com_ports, get_port_details, find_best_esp32_port
    except ImportError:
        SerialClient = None

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
        self.root.title("Xe Cân Bằng STM32F405 - Ground Control Station (8-Way Radio)")
        self.root.geometry("1180x760")
        self.root.minsize(980, 640)
        self.root.configure(bg=THEME["bg"])

        # Trạng thái Serial
        self.serial_client = None
        self.serial_thread = None
        self.serial_running = False
        self.last_connected_port = None
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        self.auto_reconnecting = False
        self._last_reconnect_attempt_ms = 0

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
        self.diag_upright = False
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

        # Bắt đầu vòng lặp cập nhật giao diện (40 ms = 25 Hz)
        self.root.after(40, self._ui_periodic_update)

    # ──────────────────────────────────────────────────────────────────────────
    # HEADER: TIÊU ĐỀ & ĐỒNG HỒ HỆ THỐNG
    # ──────────────────────────────────────────────────────────────────────────
    def _build_header(self):
        header = tk.Frame(self.root, bg=THEME["panel"], height=46, padx=16, pady=6)
        header.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(
            header,
            text="⚡ GROUND CONTROL STATION — ROBOT TỰ CÂN BẰNG 2 BÁNH STM32F405",
            font=("Segoe UI", 12, "bold"),
            fg=THEME["primary"],
            bg=THEME["panel"]
        )
        title_lbl.pack(side=tk.LEFT)

        self.lbl_dongle_status = tk.Label(
            header,
            text="● DONGLE: CHƯA KẾT NỐI",
            font=("Segoe UI", 10, "bold"),
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
    # BỐ CỤC CHÍNH: 2 CỘT (TRÁI: TELEMETRY & COM | PHẢI: ĐIỀU KHIỂN 8 HƯỚNG & CONSOLE)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_main_layout(self):
        main_pane = tk.Frame(self.root, bg=THEME["bg"], padx=10, pady=8)
        main_pane.pack(fill=tk.BOTH, expand=True)

        # Cột trái: Kết nối Serial COM + Giám sát Telemetry
        left_col = tk.Frame(main_pane, bg=THEME["bg"], width=520, padx=5, pady=5)
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Cột phải: Bảng điều khiển 8 hướng + Terminal Console
        right_col = tk.Frame(main_pane, bg=THEME["bg"], width=620, padx=5, pady=5)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self._build_serial_panel(left_col)
        self._build_telemetry_panel(left_col)
        self._build_controls_8way_panel(right_col)
        self._build_console_panel(right_col)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 1: KẾT NỐI SERIAL COM USB
    # ──────────────────────────────────────────────────────────────────────────
    def _build_serial_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🔌 Cổng USB Serial COM (ESP32 DevKit NRF24 Dongle) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.X, pady=(0, 8))

        row1 = tk.Frame(card, bg=THEME["card"])
        row1.pack(fill=tk.X, pady=2)

        tk.Label(row1, text="Cổng COM:", fg=THEME["text"], bg=THEME["card"], font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 6))

        self.cb_ports = ttk.Combobox(row1, width=16, font=("Consolas", 9))
        self.cb_ports.pack(side=tk.LEFT, padx=(0, 6))

        btn_refresh = tk.Button(
            row1, text="🔄 Quét", font=("Segoe UI", 8, "bold"),
            bg=THEME["card_alt"], fg=THEME["text"], relief=tk.FLAT, padx=8, pady=2,
            command=self.refresh_ports
        )
        btn_refresh.pack(side=tk.LEFT, padx=(0, 8))

        self.btn_connect = tk.Button(
            row1, text="⚡ KẾT NỐI", font=("Segoe UI", 9, "bold"),
            bg=THEME["primary"], fg="#11111b", activebackground=THEME["accent"],
            relief=tk.FLAT, padx=12, pady=3,
            command=self.toggle_serial_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=(0, 8))

        btn_ping = tk.Button(
            row1, text="PING", font=("Segoe UI", 8, "bold"),
            bg=THEME["card_alt"], fg=THEME["subtext"], relief=tk.FLAT, padx=8, pady=2,
            command=lambda: self.send_serial_command("PING")
        )
        btn_ping.pack(side=tk.LEFT)

        # Hàng 2: Tự động kết nối lại
        row2 = tk.Frame(card, bg=THEME["card"])
        row2.pack(fill=tk.X, pady=(6, 0))

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
    # PANEL 2: GIÁM SÁT TELEMETRY XE CÂN BẰNG
    # ──────────────────────────────────────────────────────────────────────────
    def _build_telemetry_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 📊 Giám Sát Trạng Thái & Telemetry (Real-time) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.BOTH, expand=True)

        # Hàng 1: Trạng thái xe lớn (ARM / DISARM / FAULT)
        row_state = tk.Frame(card, bg=THEME["card"])
        row_state.pack(fill=tk.X, pady=(0, 8))

        self.lbl_robot_state = tk.Label(
            row_state,
            text="DISARMED",
            font=("Segoe UI", 14, "bold"),
            fg="#11111b",
            bg=THEME["warning"],
            padx=14,
            pady=4,
            relief=tk.FLAT
        )
        self.lbl_robot_state.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))

        self.lbl_nrf_link = tk.Label(
            row_state,
            text="SÓNG NRF: CHỜ XE...",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["warning"],
            bg=THEME["card_alt"],
            padx=10,
            pady=4
        )
        self.lbl_nrf_link.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Hàng 2: Đo góc nghiêng Pitch & Thanh thủy chuẩn trực quan
        tilt_card = tk.Frame(card, bg=THEME["card_alt"], padx=10, pady=8, relief=tk.GROOVE, bd=1)
        tilt_card.pack(fill=tk.X, pady=(0, 8))

        top_tilt = tk.Frame(tilt_card, bg=THEME["card_alt"])
        top_tilt.pack(fill=tk.X)

        tk.Label(top_tilt, text="GÓC NGHIÊNG (PITCH):", font=("Segoe UI", 9, "bold"), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(side=tk.LEFT)
        self.lbl_pitch_val = tk.Label(top_tilt, text="0.00°", font=("Segoe UI", 14, "bold"), fg=THEME["success"], bg=THEME["card_alt"])
        self.lbl_pitch_val.pack(side=tk.RIGHT)

        # Thanh thủy chuẩn (Spirit Level Bar)
        self.canvas_tilt = tk.Canvas(tilt_card, height=18, bg="#11111b", highlightthickness=0)
        self.canvas_tilt.pack(fill=tk.X, pady=(6, 2))
        self.canvas_tilt.bind("<Configure>", lambda e: self._draw_tilt_gauge())

        # Hàng 3: Các chỉ số chi tiết (Tốc độ góc, Pin, Vật cản, Tốc độ lệnh)
        grid_frame = tk.Frame(card, bg=THEME["card"])
        grid_frame.pack(fill=tk.X, pady=(0, 8))

        # Ô 1: Tốc độ góc Gyro
        b1 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=8, pady=6)
        b1.grid(row=0, column=0, sticky="nsew", padx=3, pady=3)
        tk.Label(b1, text="Tốc độ Gyro Rate", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_gyro_val = tk.Label(b1, text="0.0 °/s", font=("Segoe UI", 11, "bold"), fg=THEME["text"], bg=THEME["card_alt"])
        self.lbl_gyro_val.pack(anchor="w")

        # Ô 2: Điện áp Pin 4S
        b2 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=8, pady=6)
        b2.grid(row=0, column=1, sticky="nsew", padx=3, pady=3)
        tk.Label(b2, text="Điện Áp Pin 4S", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_bat_val = tk.Label(b2, text="--.- V", font=("Segoe UI", 11, "bold"), fg=THEME["primary"], bg=THEME["card_alt"])
        self.lbl_bat_val.pack(anchor="w")

        # Ô 3: Vật cản phía trước
        b3 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=8, pady=6)
        b3.grid(row=1, column=0, sticky="nsew", padx=3, pady=3)
        tk.Label(b3, text="Khoảng cách vật cản", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_dist_val = tk.Label(b3, text="-- cm", font=("Segoe UI", 11, "bold"), fg=THEME["text"], bg=THEME["card_alt"])
        self.lbl_dist_val.pack(anchor="w")

        # Ô 4: Hướng & Lệnh lái đang gửi
        b4 = tk.Frame(grid_frame, bg=THEME["card_alt"], padx=8, pady=6)
        b4.grid(row=1, column=1, sticky="nsew", padx=3, pady=3)
        tk.Label(b4, text="Lệnh Drive / Steer", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["card_alt"]).pack(anchor="w")
        self.lbl_cmd_val = tk.Label(b4, text="0 / 0 (STOP)", font=("Segoe UI", 11, "bold"), fg=THEME["accent"], bg=THEME["card_alt"])
        self.lbl_cmd_val.pack(anchor="w")

        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)

        # Hàng 4: Chẩn đoán phần cứng (Hardware Diagnostic Flags)
        diag_box = tk.LabelFrame(card, text=" Chẩn đoán thiết bị phần cứng ", font=("Segoe UI", 8, "bold"), fg=THEME["subtext"], bg=THEME["card"], padx=6, pady=4)
        diag_box.pack(fill=tk.X, pady=(4, 0))

        self.lbl_diag_mpu = tk.Label(diag_box, text="[ MPU6050: ? ]", font=("Consolas", 8), fg=THEME["subtext"], bg=THEME["card"])
        self.lbl_diag_mpu.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_nrf = tk.Label(diag_box, text="[ NRF24: ? ]", font=("Consolas", 8), fg=THEME["subtext"], bg=THEME["card"])
        self.lbl_diag_nrf.pack(side=tk.LEFT, padx=4)

        self.lbl_diag_pos = tk.Label(diag_box, text="[ DỰNG THẲNG: ? ]", font=("Consolas", 8), fg=THEME["subtext"], bg=THEME["card"])
        self.lbl_diag_pos.pack(side=tk.LEFT, padx=4)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 3: BẢNG ĐIỀU KHIỂN 8 HƯỚNG DI CHUYỂN (8-WAY D-PAD & SLIDERS)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_controls_8way_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 🎮 Bảng Lệnh Điều Khiển 8 Hướng (Joystick / D-Pad) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.X, pady=(0, 8))

        # Hàng 1: Nút hành động chính (ARM, DISARM, ESTOP)
        act_row = tk.Frame(card, bg=THEME["card"])
        act_row.pack(fill=tk.X, pady=(0, 8))

        btn_arm = tk.Button(
            act_row, text="🟢 ARM XE (X)", font=("Segoe UI", 9, "bold"),
            bg="#2e7d32", fg="#ffffff", activebackground=THEME["success"],
            relief=tk.FLAT, padx=8, pady=6, command=self.cmd_arm
        )
        btn_arm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        btn_disarm = tk.Button(
            act_row, text="🟡 DISARM (Z/V)", font=("Segoe UI", 9, "bold"),
            bg="#f57f17", fg="#11111b", activebackground=THEME["warning"],
            relief=tk.FLAT, padx=8, pady=6, command=self.cmd_disarm
        )
        btn_disarm.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        btn_estop = tk.Button(
            act_row, text="🛑 DỪNG KHẨN (E)", font=("Segoe UI", 9, "bold"),
            bg="#c62828", fg="#ffffff", activebackground=THEME["danger"],
            relief=tk.FLAT, padx=8, pady=6, command=self.cmd_estop
        )
        btn_estop.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # Hàng 2: Cụm 8 hướng di chuyển + Thanh trượt bước tốc độ
        ctrl_split = tk.Frame(card, bg=THEME["card"])
        ctrl_split.pack(fill=tk.X, pady=4)

        # Cụm 3x3 D-Pad 8 hướng bên trái
        dpad = tk.Frame(ctrl_split, bg=THEME["card_alt"], padx=8, pady=8, relief=tk.GROOVE, bd=1)
        dpad.pack(side=tk.LEFT, padx=(0, 10))

        # Hàng 0: ↖ Tiến-Trái, ▲ Tiến, ↗ Tiến-Phải
        btn_nw = tk.Button(dpad, text="↖ Q", width=6, font=("Segoe UI", 9, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd_left)
        btn_nw.grid(row=0, column=0, padx=2, pady=2)

        btn_n = tk.Button(dpad, text="▲ W", width=8, font=("Segoe UI", 9, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd)
        btn_n.grid(row=0, column=1, padx=2, pady=2)

        btn_ne = tk.Button(dpad, text="↗ E", width=6, font=("Segoe UI", 9, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_fwd_right)
        btn_ne.grid(row=0, column=2, padx=2, pady=2)

        # Hàng 1: ◄ Trái, ■ STOP, ► Phải
        btn_w = tk.Button(dpad, text="◄ A", width=6, font=("Segoe UI", 9, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_left)
        btn_w.grid(row=1, column=0, padx=2, pady=2)

        btn_stop = tk.Button(dpad, text="■ DỪNG", width=8, font=("Segoe UI", 9, "bold"), bg="#c62828", fg="#ffffff", relief=tk.FLAT, command=self.cmd_stop)
        btn_stop.grid(row=1, column=1, padx=2, pady=2)

        btn_e = tk.Button(dpad, text="► D", width=6, font=("Segoe UI", 9, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_right)
        btn_e.grid(row=1, column=2, padx=2, pady=2)

        # Hàng 2: ↙ Lùi-Trái, ▼ Lùi, ↘ Lùi-Phải
        btn_sw = tk.Button(dpad, text="↙ Z", width=6, font=("Segoe UI", 9, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd_left)
        btn_sw.grid(row=2, column=0, padx=2, pady=2)

        btn_s = tk.Button(dpad, text="▼ S", width=8, font=("Segoe UI", 9, "bold"), bg="#45475a", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd)
        btn_s.grid(row=2, column=1, padx=2, pady=2)

        btn_se = tk.Button(dpad, text="↘ C", width=6, font=("Segoe UI", 9, "bold"), bg="#313244", fg=THEME["text"], relief=tk.FLAT, command=self.cmd_bwd_right)
        btn_se.grid(row=2, column=2, padx=2, pady=2)

        # Cột điều chỉnh bước tốc độ bên phải
        sliders = tk.Frame(ctrl_split, bg=THEME["card"])
        sliders.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        tk.Label(sliders, text="Tốc độ Tiến/Lùi (Drive Step):", font=("Segoe UI", 8, "bold"), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_speed_step = tk.Scale(
            sliders, from_=100, to=600, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_speed_step.set(200)
        self.slider_speed_step.pack(fill=tk.X, pady=(0, 6))

        tk.Label(sliders, text="Độ nhạy Rẽ/Quay (Steer Step):", font=("Segoe UI", 8, "bold"), fg=THEME["subtext"], bg=THEME["card"]).pack(anchor="w")
        self.slider_steer_step = tk.Scale(
            sliders, from_=100, to=500, orient=tk.HORIZONTAL,
            bg=THEME["card_alt"], fg=THEME["text"], highlightthickness=0, troughcolor="#11111b"
        )
        self.slider_steer_step.set(250)
        self.slider_steer_step.pack(fill=tk.X, pady=(0, 6))

        # Nhãn hiển thị hướng di chuyển hiện tại
        self.lbl_active_direction = tk.Label(
            sliders,
            text="HƯỚNG: TĨNH (STOP)",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card_alt"],
            pady=4
        )
        self.lbl_active_direction.pack(fill=tk.X)

    # ──────────────────────────────────────────────────────────────────────────
    # PANEL 4: TERMINAL CONSOLE & GỬI LỆNH TRỰC TIẾP
    # ──────────────────────────────────────────────────────────────────────────
    def _build_console_panel(self, parent):
        card = tk.LabelFrame(
            parent,
            text=" 💻 Terminal Console (Giao Tiếp ESP32 & NRF24) ",
            font=("Segoe UI", 10, "bold"),
            fg=THEME["primary"],
            bg=THEME["card"],
            bd=1,
            relief=tk.SOLID,
            padx=10,
            pady=8
        )
        card.pack(fill=tk.BOTH, expand=True)

        # Khung văn bản hiển thị log
        log_frame = tk.Frame(card, bg=THEME["console_bg"])
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))

        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.txt_console = tk.Text(
            log_frame,
            height=10,
            bg=THEME["console_bg"],
            fg="#a6e3a1",
            insertbackground="#ffffff",
            font=("Consolas", 9),
            yscrollcommand=scrollbar.set,
            bd=0,
            relief=tk.FLAT
        )
        self.txt_console.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.txt_console.yview)

        # Cấu hình màu sắc thẻ text
        self.txt_console.tag_config("tx", foreground="#89b4fa")
        self.txt_console.tag_config("rx", foreground="#a6e3a1")
        self.txt_console.tag_config("err", foreground="#f38ba8")
        self.txt_console.tag_config("warn", foreground="#fab387")
        self.txt_console.tag_config("info", foreground="#cdd6f4")

        # Hàng nhập lệnh thủ công
        in_frame = tk.Frame(card, bg=THEME["card"])
        in_frame.pack(fill=tk.X)

        self.ent_cmd = tk.Entry(in_frame, font=("Consolas", 9), bg=THEME["card_alt"], fg=THEME["text"], insertbackground=THEME["text"], bd=1, relief=tk.SOLID)
        self.ent_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6), ipady=3)
        self.ent_cmd.bind("<Return>", lambda e: self._send_manual_cmd())

        btn_send = tk.Button(
            in_frame, text="GỬI", font=("Segoe UI", 8, "bold"),
            bg=THEME["primary"], fg="#11111b", relief=tk.FLAT, padx=12,
            command=self._send_manual_cmd
        )
        btn_send.pack(side=tk.LEFT, padx=(0, 4))

        btn_clear = tk.Button(
            in_frame, text="XÓA", font=("Segoe UI", 8),
            bg=THEME["card_alt"], fg=THEME["subtext"], relief=tk.FLAT, padx=8,
            command=self.clear_console
        )
        btn_clear.pack(side=tk.LEFT)

    # ──────────────────────────────────────────────────────────────────────────
    # STATUS BAR (THANH TRẠNG THÁI CUỐI TRANG)
    # ──────────────────────────────────────────────────────────────────────────
    def _build_status_bar(self):
        sb = tk.Frame(self.root, bg=THEME["panel"], height=24, padx=10)
        sb.pack(fill=tk.X, side=tk.BOTTOM)

        self.lbl_sb_port = tk.Label(sb, text="Port: CHƯA KẾT NỐI", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["panel"])
        self.lbl_sb_port.pack(side=tk.LEFT, padx=(0, 15))

        self.lbl_sb_txrx = tk.Label(sb, text="TX: 0 | RX: 0", font=("Segoe UI", 8), fg=THEME["subtext"], bg=THEME["panel"])
        self.lbl_sb_txrx.pack(side=tk.LEFT, padx=(0, 15))

        self.lbl_sb_mode = tk.Label(sb, text="Chế độ: 8-Way NRF Wireless Bridge", font=("Segoe UI", 8), fg=THEME["accent"], bg=THEME["panel"])
        self.lbl_sb_mode.pack(side=tk.RIGHT)

    # ──────────────────────────────────────────────────────────────────────────
    # XỬ LÝ PHÍM TẮT BÀN PHÍM
    # ──────────────────────────────────────────────────────────────────────────
    def _bind_keyboard_events(self):
        self.root.bind("<Key>", self._on_key_press)
        self.root.bind("<space>", lambda e: self.cmd_stop())

    def _on_key_press(self, event):
        # Tránh can thiệp khi đang gõ phím vào ô nhập text
        if event.widget == self.ent_cmd:
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
        self.lbl_active_direction.config(text="LỆNH: 🛑 DỪNG KHẨN CẤP")

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

            # Tạo luồng nhận dữ liệu
            self.serial_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
            self.serial_thread.start()

            # Gửi lệnh PING kiểm tra
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
        # In vào console
        tag = "rx"
        if "LỖI" in line or "ERR" in line:
            tag = "err"
        elif "CHỜ" in line or "WARN" in line:
            tag = "warn"
        elif "SÓNG TỐT" in line or "KẾT NỐI TỐT" in line:
            tag = "rx"
        self._log_console(line, tag)

        # Phân tích cú pháp trạng thái sóng NRF
        if "SÓNG TỐT" in line or "KẾT NỐI TỐT" in line:
            self.nrf_link_ok = True
            self.lbl_nrf_link.config(text="SÓNG NRF: 🟢 ĐÃ KẾT NỐI (AUTO-ACK)", fg=THEME["success"])
        elif "CHỜ TÍN HIỆU XE" in line or "Đang phát sóng tìm xe" in line:
            self.nrf_link_ok = False
            self.lbl_nrf_link.config(text="SÓNG NRF: 🟡 CHỜ XE BẬT NGUỒN", fg=THEME["warning"])
        elif "LỖI NRF24" in line or "LỖI PHẦN CỨNG" in line:
            self.nrf_link_ok = False
            self.lbl_nrf_link.config(text="SÓNG NRF: 🔴 LỖI PHẦN CỨNG", fg=THEME["danger"])

        # Phân tích chuỗi Telemetry nếu có (Ví dụ JSON hoặc ACK định dạng)
        # Format chuẩn: "pitch": 1.25, "rate": -0.8, "target": 0.0, "state": "ARMED", "vbat": 15.2
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
    # VÒNG LẶP CẬP NHẬT GIAO DIỆN ĐỊNH KỲ (25 Hz)
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

        # Đổi màu trạng thái ARM / DISARM
        if self.robot_state_str == "ARMED":
            self.lbl_robot_state.config(text="ARMED (ĐANG CÂN BẰNG)", bg=THEME["success"], fg="#11111b")
        elif self.robot_state_str == "FAULT":
            self.lbl_robot_state.config(text="FAULT (LỖI/NGÃ XE)", bg=THEME["danger"], fg="#ffffff")
        else:
            self.lbl_robot_state.config(text=self.robot_state_str, bg=THEME["warning"], fg="#11111b")

        # Cập nhật thanh thủy chuẩn góc nghiêng
        self._draw_tilt_gauge()

        # 3. Cập nhật Status Bar
        self.lbl_sb_txrx.config(text=f"TX: {self.tx_count} | RX: {self.rx_count}")

        # Lặp lại sau 40 ms
        self.root.after(40, self._ui_periodic_update)

    def _draw_tilt_gauge(self):
        w = self.canvas_tilt.winfo_width()
        h = self.canvas_tilt.winfo_height()
        if w < 10:
            return

        self.canvas_tilt.delete("all")
        cx = w // 2

        # Vạch tâm cân bằng
        self.canvas_tilt.create_line(cx, 0, cx, h, fill="#6c7086", width=2)

        # Vị trí con trỏ góc nghiêng (-15° .. +15°)
        clamped_pitch = max(-15.0, min(15.0, self.pitch_deg))
        indicator_x = cx + int((clamped_pitch / 15.0) * (w / 2 - 4))

        color = THEME["success"] if abs(self.pitch_deg) < 4.0 else (THEME["warning"] if abs(self.pitch_deg) < 12.0 else THEME["danger"])
        self.canvas_tilt.create_rectangle(min(cx, indicator_x), 2, max(cx, indicator_x), h - 2, fill=color, outline="")

    def _log_console(self, msg, tag="info"):
        self.txt_console.insert(tk.END, msg + "\n", tag)
        self.txt_console.see(tk.END)

    def clear_console(self):
        self.txt_console.delete("1.0", tk.END)


def main():
    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
