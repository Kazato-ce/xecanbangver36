#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Công Cụ Giám Sát & Điều Khiển Bộ Chẩn Đoán STM32F405 Qua Cổng USB CDC
Tự động tìm cổng STM32 Virtual COM Port và mở giao diện console trực tiếp.
"""

import sys
import time
import threading

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

def find_stm32_port():
    if not HAS_PYSERIAL:
        return None
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        if "stm" in desc or "virtual" in desc or "stmicroelectronics" in mfg:
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    print("=" * 70)
    print("   CÔNG CỤ GIÁM SÁT BỘ TEST PHẦN CỨNG STM32F405 (USB CDC)")
    print("   Kiểm tra MPU6050 (I2C1) - TMC2208 (GPIO) - NRF24L01 (SPI2)")
    print("=" * 70)

    if not HAS_PYSERIAL:
        print("[!] Chưa cài pyserial. Bạn có thể cài bằng lệnh: pip install pyserial")
        print("    Hoặc mở Arduino Serial Monitor / PuTTY / Tera Term với cổng COM của STM32.")
        input("\nBấm Enter để thoát...")
        return

    port = find_stm32_port()
    if not port:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        print(f"[*] Các cổng COM đang có trên máy: {ports}")
        port = input("Nhập tên cổng COM của STM32 (ví dụ COM7): ").strip().upper()

    print(f"[*] Đang kết nối với cổng: {port} ...")
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=0.1)
        print(f"[OK] Đã kết nối thành công với {port}!\n")
    except Exception as e:
        print(f"[LỖI] Không thể mở cổng {port}: {e}")
        input("\nBấm Enter để thoát...")
        return

    running = True

    def rx_thread_fn():
        while running:
            try:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    text = data.decode("utf-8", errors="replace")
                    sys.stdout.write(text)
                    sys.stdout.flush()
            except Exception:
                break

    t = threading.Thread(target=rx_thread_fn, daemon=True)
    t.start()

    # Gửi lệnh '?' để yêu cầu STM32 in lại menu
    time.sleep(0.3)
    ser.write(b"?\r\n")

    try:
        while running:
            # Nhập phím từ bàn phím và gửi ngay sang STM32
            user_input = sys.stdin.readline()
            if not user_input:
                break
            ser.write(user_input.encode("utf-8"))
    except KeyboardInterrupt:
        print("\n[*] Đang đóng kết nối...")
    finally:
        running = False
        ser.close()
        print("[*] Đã ngắt kết nối.")

if __name__ == "__main__":
    main()
