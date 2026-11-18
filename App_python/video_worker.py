"""
Video Stream Grabber for ESP32-CAM and other HTTP / MJPEG / RTSP streams.
Decoupled and standalone: Takes any URL, grabs frames via OpenCV or HTTP, and passes to GUI.
"""

import time
import threading
import urllib.request
import cv2
import numpy as np


class VideoStreamWorker(threading.Thread):
    def __init__(self, source_url, on_frame, on_status):
        super().__init__(daemon=True)
        self.source_url = source_url.strip()
        self.on_frame = on_frame          # callback(cv2_bgr_frame, fps)
        self.on_status = on_status        # callback(status_str, is_connected)
        self.running = False
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()
        self.running = False

    def run(self):
        self.running = True
        self.on_status(f"Đang kết nối: {self.source_url} ...", False)

        # Kiểm tra nếu là webcam local (e.g. '0', '1')
        if self.source_url.isdigit():
            self._run_opencv_capture(int(self.source_url))
            return

        # Kiểm tra nếu là URL ảnh tĩnh (snapshot) hay stream MJPEG
        is_snapshot = self.source_url.lower().endswith(('.jpg', '.jpeg', '.png')) or 'capture' in self.source_url.lower()

        if is_snapshot:
            self._run_snapshot_loop()
        else:
            self._run_opencv_capture(self.source_url)

    def _run_opencv_capture(self, source):
        cap = None
        try:
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                # Thử thêm flag timeout hoặc ffmpeg
                self.on_status(f"Không thể mở stream từ: {source}", False)
                return

            self.on_status("Đã kết nối stream thành công!", True)
            frame_count = 0
            start_time = time.time()
            current_fps = 0.0

            while not self._stop_event.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.on_status("Mất tín hiệu video... đang thử lại", False)
                    time.sleep(0.5)
                    continue

                frame_count += 1
                elapsed = time.time() - start_time
                if elapsed >= 1.0:
                    current_fps = frame_count / elapsed
                    frame_count = 0
                    start_time = time.time()

                try:
                    self.on_frame(frame, current_fps)
                except Exception:
                    pass

                # Điều chỉnh tốc độ khung hình nhẹ nhàng để giảm tải CPU
                time.sleep(0.01)

        except Exception as e:
            self.on_status(f"Lỗi stream: {e}", False)
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            self.on_status("Đã dừng stream video.", False)

    def _run_snapshot_loop(self):
        """Dùng cho ESP32-CAM ở chế độ chụp ảnh snapshot định kỳ (URL /capture)"""
        self.on_status("Đã kết nối chế độ Snapshot HTTP!", True)
        frame_count = 0
        start_time = time.time()
        current_fps = 0.0

        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                req = urllib.request.Request(
                    self.source_url,
                    headers={'User-Agent': 'Mozilla/5.0'}
                )
                with urllib.request.urlopen(req, timeout=2.0) as resp:
                    img_bytes = resp.read()
                img_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    frame_count += 1
                    elapsed = time.time() - start_time
                    if elapsed >= 1.0:
                        current_fps = frame_count / elapsed
                        frame_count = 0
                        start_time = time.time()

                    self.on_frame(frame, current_fps)
            except Exception:
                time.sleep(0.2)

            sleep_time = max(0.02, 0.06 - (time.time() - t0))
            time.sleep(sleep_time)

        self.on_status("Đã dừng snapshot.", False)
