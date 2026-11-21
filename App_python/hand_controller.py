"""
Hand Gesture Controller using Laptop Webcam + MediaPipe Hands.
Runs independently from the ESP32-S3 CAM FPV stream.
"""

import time
import threading
import cv2

try:
    import mediapipe as mp
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False


class HandGestureWorker(threading.Thread):
    def __init__(self, cam_index, get_limits_fn, on_control, on_frame, on_status):
        super().__init__(daemon=True)
        self.cam_index = cam_index
        self.get_limits_fn = get_limits_fn  # returns (max_drive, max_steer)
        self.on_control = on_control        # callback(drive_cmd, steer_cmd, hand_found)
        self.on_frame = on_frame            # callback(annotated_bgr_frame)
        self.on_status = on_status          # callback(msg_str, is_ok)
        self._stop_event = threading.Event()
        self.running = False

    def stop(self):
        self._stop_event.set()
        self.running = False

    def run(self):
        if not HAS_MEDIAPIPE:
            self.on_status("Thiếu thư viện mediapipe (chạy: pip install mediapipe)", False)
            return

        cap = cv2.VideoCapture(self.cam_index)
        if not cap.isOpened():
            self.on_status(f"Không mở được Webcam ({self.cam_index})!", False)
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        mp_hands = mp.solutions.hands
        mp_draw = mp.solutions.drawing_utils
        detector = mp_hands.Hands(
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )

        self.running = True
        self.on_status("Webcam lái tay đang hoạt động!", True)

        try:
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    time.sleep(0.02)
                    continue

                frame = cv2.flip(frame, 1)
                h, w, _ = frame.shape
                cx, cy = w // 2, h // 2

                # Vẽ hồng tâm và khung vùng chết (deadzone)
                dz_x = int(w * 0.04)
                dz_y = int(h * 0.05)
                cv2.rectangle(frame, (cx - dz_x, cy - dz_y), (cx + dz_x, cy + dz_y), (120, 120, 120), 1)
                cv2.line(frame, (cx - 14, cy), (cx + 14, cy), (180, 180, 180), 1)
                cv2.line(frame, (cx, cy - 14), (cx, cy + 14), (180, 180, 180), 1)

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                results = detector.process(rgb)

                drive_cmd = 0
                steer_cmd = 0
                hand_found = False

                if results.multi_hand_landmarks:
                    hand_found = True
                    hand_lm = results.multi_hand_landmarks[0]
                    mp_draw.draw_landmarks(frame, hand_lm, mp_hands.HAND_CONNECTIONS)

                    ix = max(0.0, min(1.0, hand_lm.landmark[8].x))
                    iy = max(0.0, min(1.0, hand_lm.landmark[8].y))

                    dx = ix - 0.5   # Trái (-0.5) -> Phải (+0.5)
                    dy = 0.5 - iy   # Xuống (-0.5) -> Lên (+0.5)

                    max_drive, max_steer = self.get_limits_fn()

                    if abs(dx) >= 0.04:
                        steer_cmd = int((dx / 0.46) * max_steer)
                        steer_cmd = max(-max_steer, min(max_steer, steer_cmd))

                    if abs(dy) >= 0.05:
                        drive_cmd = int((dy / 0.45) * max_drive)
                        drive_cmd = max(-max_drive, min(max_drive, drive_cmd))

                    px, py = int(ix * w), int(iy * h)
                    cv2.line(frame, (cx, cy), (px, py), (0, 255, 255), 2)
                    cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)

                label = f"D:{drive_cmd} S:{steer_cmd}" if hand_found else "NO HAND"
                cv2.putText(frame, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

                self.on_control(drive_cmd, steer_cmd, hand_found)
                self.on_frame(frame)
                time.sleep(0.015)

        finally:
            detector.close()
            cap.release()
            self.running = False