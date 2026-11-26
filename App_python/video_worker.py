"""
Video Stream Grabber for ESP32-S3 CAM FPV and HTTP / MJPEG streams.
"""

import threading
import time
import os
import traceback
import urllib.request

import cv2
import numpy as np


class VideoStreamWorker(threading.Thread):
    def __init__(self, source_url, on_frame, on_status):
        super().__init__(daemon=True)
        self.source_url = source_url.strip()
        self.on_frame = on_frame
        self.on_status = on_status
        self.running = False
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()
        self.running = False

    def run(self):
        self.running = True
        self.on_status(f"Đang kết nối FPV: {self.source_url} ...", False)
        try:
            if self.source_url.isdigit():
                self._run_webcam_capture(int(self.source_url))
            elif self._is_snapshot_url():
                self._run_snapshot_loop()
            else:
                self._run_mjpeg_stream()
        finally:
            self.running = False

    def _is_snapshot_url(self):
        url = self.source_url.lower()
        return url.endswith((".jpg", ".jpeg", ".png")) or "capture" in url

    @staticmethod
    def _open_camera(camera_index):
        if os.name == "nt":
            cap = None
            try:
                cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
                if cap.isOpened():
                    return cap
            except Exception:
                traceback.print_exc()
            if cap is not None:
                cap.release()

        return cv2.VideoCapture(camera_index)

    def _run_webcam_capture(self, camera_index):
        while not self._stop_event.is_set():
            cap = self._open_camera(camera_index)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                cap.release()
                self.on_status("Chưa mở được webcam... thử lại sau 1 giây", False)
                self._stop_event.wait(1.0)
                continue

            self.on_status("Đã kết nối webcam!", True)
            frame_count = 0
            start_time = time.monotonic()
            current_fps = 0.0
            try:
                while not self._stop_event.is_set():
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        break

                    frame_count += 1
                    elapsed = time.monotonic() - start_time
                    if elapsed >= 1.0:
                        current_fps = frame_count / elapsed
                        frame_count = 0
                        start_time = time.monotonic()
                    self.on_frame(frame, current_fps)
            finally:
                cap.release()

            if not self._stop_event.is_set():
                self.on_status("Mất webcam... đang tự nối lại", False)
                self._stop_event.wait(1.0)

    def _run_mjpeg_stream(self):
        frame_count = 0
        start_time = time.monotonic()
        current_fps = 0.0

        while not self._stop_event.is_set():
            buf = bytearray()
            try:
                req = urllib.request.Request(
                    self.source_url,
                    headers={"User-Agent": "Mozilla/5.0"},
                )
                with urllib.request.urlopen(req, timeout=2.0) as response:
                    self.on_status("Đã kết nối FPV stream!", True)
                    while not self._stop_event.is_set():
                        chunk = response.read(4096)
                        if not chunk:
                            break
                        buf.extend(chunk)

                        if len(buf) > 300_000:
                            buf.clear()
                            continue

                        end_idx = buf.rfind(b"\xff\xd9")
                        if end_idx < 0:
                            continue

                        start_idx = buf.rfind(b"\xff\xd8", 0, end_idx)
                        if start_idx < 0:
                            del buf[:end_idx + 2]
                            continue

                        jpeg = bytes(buf[start_idx:end_idx + 2])
                        del buf[:end_idx + 2]
                        encoded = np.frombuffer(jpeg, dtype=np.uint8)
                        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue

                        frame_count += 1
                        elapsed = time.monotonic() - start_time
                        if elapsed >= 1.0:
                            current_fps = frame_count / elapsed
                            frame_count = 0
                            start_time = time.monotonic()
                        self.on_frame(frame, current_fps)

                if not self._stop_event.is_set():
                    self.on_status("Mất tín hiệu FPV... đang tự nối lại", False)
                    self._stop_event.wait(1.0)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.on_status(f"Lỗi stream FPV: {exc} ... thử lại sau 1 giây", False)
                    self._stop_event.wait(1.0)

        self.on_status("Đã dừng stream FPV.", False)

    def _run_snapshot_loop(self):
        self.on_status("Đã kết nối chế độ Snapshot HTTP!", True)
        frame_count = 0
        start_time = time.monotonic()
        current_fps = 0.0

        while not self._stop_event.is_set():
            started_at = time.monotonic()
            try:
                req = urllib.request.Request(
                    self.source_url,
                    headers={"User-Agent": "Mozilla/5.0"},
                )
                with urllib.request.urlopen(req, timeout=2.0) as response:
                    img_bytes = response.read()
                img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    frame_count += 1
                    elapsed = time.monotonic() - start_time
                    if elapsed >= 1.0:
                        current_fps = frame_count / elapsed
                        frame_count = 0
                        start_time = time.monotonic()
                    self.on_frame(frame, current_fps)
            except Exception as exc:
                self.on_status(f"Lỗi snapshot: {exc}", False)

            self._stop_event.wait(max(0.02, 0.06 - (time.monotonic() - started_at)))

        self.on_status("Đã dừng snapshot.", False)
