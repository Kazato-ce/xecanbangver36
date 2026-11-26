"""
Hand Gesture Controller using Laptop Webcam + MediaPipe Hands.
- OK sign: Arm / Recalibrate center origin
- Hand Up / Down: Drive Forward / Backward
- 2 Fingers (Index + Middle) Left / Right: Steer Left / Right
- Fist / No Hand: Brake & Auto-Lock
"""

import math
import os
import shutil
import time
import threading
import traceback
from importlib import import_module
from pathlib import Path
from urllib.request import urlopen

import cv2

mp = None
mp_solutions = None
MpBaseOptions = None
HandLandmarker = None
HandLandmarkerOptions = None
RunningMode = None
HAS_TASKS_MEDIAPIPE = False

try:
    import mediapipe as mp
    mp_solutions = getattr(mp, "solutions", None)
    if mp_solutions is None:
        try:
            mp_solutions = import_module("mediapipe.python.solutions")
        except ImportError:
            mp_solutions = None
    try:
        from mediapipe.tasks.python import BaseOptions as MpBaseOptions
        from mediapipe.tasks.python.vision import (
            HandLandmarker,
            HandLandmarkerOptions,
            RunningMode,
        )
        HAS_TASKS_MEDIAPIPE = True
    except ImportError:
        pass
except ImportError:
    pass

HAS_MEDIAPIPE = (
    (mp_solutions is not None and hasattr(mp_solutions, "hands"))
    or HAS_TASKS_MEDIAPIPE
)

_HAND_LANDMARKER_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
_HAND_LANDMARKER_MODEL_PATH = (
    Path(__file__).resolve().parent / "models" / "hand_landmarker.task"
)
_HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)

_HAND_CAMERA_LOCK = threading.Lock()


def _ensure_hand_landmarker_model():
    model_path = _HAND_LANDMARKER_MODEL_PATH
    if model_path.is_file() and model_path.stat().st_size > 0:
        return model_path

    model_path.parent.mkdir(parents=True, exist_ok=True)
    download_path = model_path.with_suffix(".task.download")
    try:
        with urlopen(_HAND_LANDMARKER_MODEL_URL, timeout=60) as response:
            with download_path.open("wb") as model_file:
                shutil.copyfileobj(response, model_file)
        if download_path.stat().st_size == 0:
            raise RuntimeError("The downloaded hand landmark model is empty")
        download_path.replace(model_path)
    except Exception as exc:
        try:
            download_path.unlink()
        except FileNotFoundError:
            pass
        raise RuntimeError(
            f"Could not download MediaPipe hand model from {_HAND_LANDMARKER_MODEL_URL}"
        ) from exc
    return model_path


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

        # Trạng thái điều khiển & tâm động
        self.armed = False
        self.origin_x = 0.5
        self.origin_y = 0.5

        # Bộ đếm chống nhiễu cử chỉ
        self._ok_frames = 0
        self._fist_frames = 0
        self._lost_frames = 0

        # Bộ lọc mượt tín hiệu (EMA)
        self._smooth_drive = 0.0
        self._smooth_steer = 0.0

    def stop(self):
        self._stop_event.set()
        self.running = False

    @staticmethod
    def _px_dist(lm1, lm2, w, h):
        """Tính khoảng cách pixel thực tế để không bị méo theo tỉ lệ khung hình 4:3."""
        return math.hypot((lm1.x - lm2.x) * w, (lm1.y - lm2.y) * h)

    def _is_finger_extended(self, lm, tip_idx, pip_idx, w, h):
        """Kiểm tra ngón tay duỗi dựa trên tỉ lệ khoảng cách tới cổ tay (không phụ thuộc góc nghiêng)."""
        wrist = lm[0]
        d_tip = self._px_dist(wrist, lm[tip_idx], w, h)
        d_pip = self._px_dist(wrist, lm[pip_idx], w, h)
        return d_tip > (d_pip * 1.15)

    def _detect_gestures(self, lm, w, h):
        """Nhận diện 3 nhóm cử chỉ: OK, 2 ngón lái, Nắm đấm phanh."""
        palm_size = max(self._px_dist(lm[0], lm[9], w, h), 1.0)
        thumb_index_dist = self._px_dist(lm[4], lm[8], w, h)

        idx_ext = self._is_finger_extended(lm, 8, 6, w, h)
        mid_ext = self._is_finger_extended(lm, 12, 10, w, h)
        ring_ext = self._is_finger_extended(lm, 16, 14, w, h)
        pinky_ext = self._is_finger_extended(lm, 20, 18, w, h)

        # 1. Cử chỉ OK: Đầu ngón cái chạm đầu ngón trỏ + 3 ngón còn lại duỗi
        is_ok = (
            thumb_index_dist < (0.38 * palm_size)
            and mid_ext
            and ring_ext
            and pinky_ext
        )

        # 2. Cử chỉ 2 ngón: Ngón trỏ + giữa duỗi, ngón áp út + út gập, không chạm ngón cái
        is_two_fingers = (
            idx_ext
            and mid_ext
            and (not ring_ext)
            and (not pinky_ext)
            and thumb_index_dist >= (0.42 * palm_size)
        )

        # 3. Cử chỉ nắm đấm: Cả 4 ngón chính đều gập
        is_fist = (not idx_ext) and (not mid_ext) and (not ring_ext) and (not pinky_ext)

        return is_ok, is_two_fingers, is_fist

    def run(self):
        with _HAND_CAMERA_LOCK:
            if not self._stop_event.is_set():
                self._run_camera_session()

    def _open_camera(self):
        if os.name == "nt":
            cap = None
            try:
                cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW)
                if cap.isOpened():
                    return cap
            except Exception:
                traceback.print_exc()

            if cap is not None:
                cap.release()
            print("Webcam: DirectShow unavailable; trying the default OpenCV backend.")

        return cv2.VideoCapture(self.cam_index)

    def _create_hand_detector(self):
        if mp_solutions is not None and hasattr(mp_solutions, "hands"):
            mp_hands = mp_solutions.hands
            detector = mp_hands.Hands(
                max_num_hands=1,
                model_complexity=0,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.7,
            )
            return (
                "solutions",
                detector,
                getattr(mp_solutions, "drawing_utils", None),
                getattr(mp_hands, "HAND_CONNECTIONS", _HAND_CONNECTIONS),
            )

        if HAS_TASKS_MEDIAPIPE:
            model_path = _ensure_hand_landmarker_model()
            options = HandLandmarkerOptions(
                base_options=MpBaseOptions(model_asset_path=str(model_path)),
                running_mode=RunningMode.VIDEO,
                num_hands=1,
                min_hand_detection_confidence=0.7,
                min_hand_presence_confidence=0.7,
                min_tracking_confidence=0.7,
            )
            detector = HandLandmarker.create_from_options(options)
            return "tasks", detector, None, _HAND_CONNECTIONS

        raise RuntimeError("Installed MediaPipe has neither solutions nor Tasks HandLandmarker")

    @staticmethod
    def _draw_hand_landmarks(frame, landmarks, connections):
        height, width, _ = frame.shape
        for start_idx, end_idx in connections:
            start = (int(landmarks[start_idx].x * width), int(landmarks[start_idx].y * height))
            end = (int(landmarks[end_idx].x * width), int(landmarks[end_idx].y * height))
            cv2.line(frame, start, end, (0, 255, 0), 2)

        for landmark in landmarks:
            point = (int(landmark.x * width), int(landmark.y * height))
            cv2.circle(frame, point, 3, (0, 0, 255), -1)

    def _run_camera_session(self):
        if not HAS_MEDIAPIPE:
            self.on_status("Thiếu thư viện mediapipe (chạy: pip install mediapipe)", False)
            return

        cap = None
        detector = None
        try:
            cap = self._open_camera()
            if not cap.isOpened():
                raise RuntimeError(f"Không mở được webcam ({self.cam_index}) bằng DirectShow hoặc backend mặc định")

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            detector_backend, detector, mp_draw, hand_connections = (
                self._create_hand_detector()
            )
        except Exception as exc:
            traceback.print_exc()
            self.on_status(f"Lỗi khởi tạo webcam/MediaPipe: {exc}", False)
            if detector is not None:
                try:
                    detector.close()
                except Exception:
                    traceback.print_exc()
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    traceback.print_exc()
            return

        self.running = True

        # Ngưỡng vùng chết (deadzone) & độ nhạy
        DZ_Y = 0.06       # Vùng đứng yên theo chiều dọc (6% chiều cao khung hình)
        DZ_X = 0.05       # Vùng đi thẳng theo chiều ngang (5% chiều rộng khung hình)
        RANGE_Y = 0.32    # Đưa tay lên/xuống 32% khung hình là đạt tối đa ga
        RANGE_X = 0.30    # Đưa 2 ngón qua trái/phải 30% khung hình là hết lái
        EMA_ALPHA = 0.35  # Độ mượt (0.1 = rất mượt/trễ nhẹ, 0.5 = nhạy/ít lọc)
        read_failures = 0
        last_timestamp_ms = 0

        try:
            self.on_status("Sẵn sàng! Giơ ký hiệu OK để bắt đầu lái.", True)
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    read_failures += 1
                    if read_failures >= 25:
                        raise RuntimeError("Webcam không trả frame hợp lệ sau 25 lần đọc liên tiếp")
                    time.sleep(0.02)
                    continue
                read_failures = 0

                frame = cv2.flip(frame, 1)
                h, w, _ = frame.shape

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                if detector_backend == "solutions":
                    results = detector.process(rgb)
                    hand_landmarks = results.multi_hand_landmarks
                else:
                    timestamp_ms = int(time.monotonic() * 1000)
                    if timestamp_ms <= last_timestamp_ms:
                        timestamp_ms = last_timestamp_ms + 1
                    last_timestamp_ms = timestamp_ms
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                    results = detector.detect_for_video(mp_image, timestamp_ms)
                    hand_landmarks = results.hand_landmarks

                raw_drive = 0.0
                raw_steer = 0.0
                hand_found = False
                mode_text = "LOCKED (GIO OK)"
                hud_color = (0, 165, 255)  # Cam: đang khóa

                if hand_landmarks:
                    hand_found = True
                    self._lost_frames = 0
                    hand_lm = hand_landmarks[0]
                    lm = hand_lm.landmark if detector_backend == "solutions" else hand_lm
                    if mp_draw is not None and detector_backend == "solutions":
                        mp_draw.draw_landmarks(frame, hand_lm, hand_connections)
                    else:
                        self._draw_hand_landmarks(frame, lm, hand_connections)

                    # Tâm lòng bàn tay (gốc ngón giữa - landmark 9) rất ổn định khi gập/duỗi ngón
                    palm_x = max(0.0, min(1.0, lm[9].x))
                    palm_y = max(0.0, min(1.0, lm[9].y))

                    is_ok, is_two_fingers, is_fist = self._detect_gestures(lm, w, h)
                    max_drive, max_steer = self.get_limits_fn()

                    # --- XỬ LÝ CỬ CHỈ OK (MỞ KHÓA / ĐẶT LẠI TÂM) ---
                    if is_ok:
                        self._ok_frames += 1
                        self._fist_frames = 0
                        progress = min(100, int((self._ok_frames / 8) * 100))
                        mode_text = f"OK... {progress}%"
                        hud_color = (255, 255, 0)

                        if self._ok_frames == 8:
                            self.armed = True
                            self.origin_x = palm_x
                            self.origin_y = palm_y
                            self._smooth_drive = 0.0
                            self._smooth_steer = 0.0
                            self.on_status("Đã nhận OK - Đang điều khiển xe!", True)
                    else:
                        self._ok_frames = 0

                    # --- XỬ LÝ KHI ĐÃ MỞ KHÓA (ARMED) ---
                    if self.armed:
                        hud_color = (0, 255, 0)  # Xanh lá: đang lái

                        if is_fist:
                            # Nắm đấm: Phanh ngay lập tức, giữ 15 frame (~0.5s) thì khóa lại
                            self._fist_frames += 1
                            raw_drive = 0.0
                            raw_steer = 0.0
                            self._smooth_drive = 0.0
                            self._smooth_steer = 0.0
                            mode_text = "BRAKE (NAM TAY)"
                            hud_color = (0, 0, 255)

                            if self._fist_frames >= 15:
                                self.armed = False
                                self._fist_frames = 0
                                self.on_status("Đã khóa điều khiển (Giơ OK để mở lại).", True)
                        else:
                            self._fist_frames = 0

                            # 1. Tính Tiến / Lùi theo độ cao bàn tay so với tâm origin_y
                            dy = self.origin_y - palm_y  # Lên cao (+), xuống thấp (-)
                            if abs(dy) > DZ_Y:
                                norm_y = (abs(dy) - DZ_Y) / (RANGE_Y - DZ_Y)
                                norm_y = max(0.0, min(1.0, norm_y))
                                raw_drive = math.copysign(norm_y * max_drive, dy)

                            # 2. Tính Rẽ Trái / Phải CHỈ KHI giơ 2 ngón (trỏ + giữa)
                            if is_two_fingers:
                                two_x = (lm[8].x + lm[12].x) * 0.5
                                dx = two_x - self.origin_x  # Qua phải (+), qua trái (-)
                                if abs(dx) > DZ_X:
                                    norm_x = (abs(dx) - DZ_X) / (RANGE_X - DZ_X)
                                    norm_x = max(0.0, min(1.0, norm_x))
                                    raw_steer = math.copysign(norm_x * max_steer, dx)

                                # Vẽ điểm điều hướng 2 ngón
                                sx, sy = int(two_x * w), int(((lm[8].y + lm[12].y) * 0.5) * h)
                                cv2.circle(frame, (sx, sy), 6, (255, 0, 255), -1)
                                mode_text = "LAI 2 NGON"
                            else:
                                raw_steer = 0.0
                                if not is_ok:
                                    mode_text = "TIEN / LUI"

                        # Vẽ tâm gốc (origin) và vùng chết lên màn hình để dễ căn tay
                        ox, oy = int(self.origin_x * w), int(self.origin_y * h)
                        dz_px_x, dz_px_y = int(DZ_X * w), int(DZ_Y * h)
                        cv2.rectangle(
                            frame,
                            (ox - dz_px_x, oy - dz_px_y),
                            (ox + dz_px_x, oy + dz_px_y),
                            (180, 180, 180),
                            1,
                        )
                        px, py = int(palm_x * w), int(palm_y * h)
                        cv2.line(frame, (ox, oy), (px, py), (0, 255, 255), 2)
                        cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)

                else:
                    # Mất dấu tay: Dừng xe ngay; mất quá 45 frame (~1.5s) thì tự khóa lại
                    self._ok_frames = 0
                    self._fist_frames = 0
                    self._smooth_drive = 0.0
                    self._smooth_steer = 0.0

                    if self.armed:
                        self._lost_frames += 1
                        mode_text = "MAT TAY - DUNG"
                        hud_color = (0, 0, 255)
                        if self._lost_frames >= 45:
                            self.armed = False
                            self._lost_frames = 0
                            self.on_status("Mất tín hiệu tay - Đã tự động khóa.", True)

                # Làm mượt tín hiệu đầu ra
                if self.armed and hand_found:
                    self._smooth_drive = EMA_ALPHA * raw_drive + (1.0 - EMA_ALPHA) * self._smooth_drive
                    self._smooth_steer = EMA_ALPHA * raw_steer + (1.0 - EMA_ALPHA) * self._smooth_steer
                    drive_cmd = int(round(self._smooth_drive))
                    steer_cmd = int(round(self._smooth_steer))
                else:
                    drive_cmd = 0
                    steer_cmd = 0

                # Hiển thị thông số lên khung hình
                cv2.putText(frame, mode_text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, hud_color, 2)
                cv2.putText(
                    frame,
                    f"D:{drive_cmd} S:{steer_cmd}",
                    (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )

                self.on_control(drive_cmd, steer_cmd, hand_found and self.armed)
                self.on_frame(frame)
                time.sleep(0.015)

        except Exception as exc:
            traceback.print_exc()
            self.on_status(f"Lỗi khi đọc webcam/nhận diện tay: {exc}", False)
        finally:
            try:
                detector.close()
            except Exception:
                traceback.print_exc()
            try:
                cap.release()
            except Exception:
                traceback.print_exc()
            self.running = False