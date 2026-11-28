import socket
import struct
import time
import cv2
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 8888
PACKET_MAGIC = 0xAA55
HEADER_FMT = "<HHBBH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.05)

cur_frame_id = None
chunks = {}
total_chunks_expected = 0

pkt_counter = 0
fps_counter = 0
drop_counter = 0
display_fps = 0
display_drop = 0
last_stat_time = time.time()

window_name = "ESP32-S3 CAM FPV (UDP)"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, 640, 480)

# Tạo màn hình chờ để cửa sổ OpenCV luôn hiển thị ngay lập tức
wait_screen = np.zeros((240, 320, 3), dtype=np.uint8)
cv2.putText(wait_screen, "Cho UDP 192.168.4.2:8888...", (15, 120),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
has_frame = False

print(f"[INFO] Dang lang nghe UDP tai cong {UDP_PORT}... Nhan 'q' de thoat.")

try:
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            pkt_counter += 1
        except socket.timeout:
            if not has_frame:
                cv2.imshow(window_name, wait_screen)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            now = time.time()
            if now - last_stat_time >= 1.0:
                print(f"[PC RX] Pkts/s: {pkt_counter} | FPS: {fps_counter} | Drop: {drop_counter}")
                pkt_counter = fps_counter = drop_counter = 0
                last_stat_time = now
            continue

        if len(data) < HEADER_SIZE:
            continue

        magic, frame_id, chunk_idx, total_chunks, chunk_len = struct.unpack(
            HEADER_FMT, data[:HEADER_SIZE]
        )

        if magic != PACKET_MAGIC or len(data) < HEADER_SIZE + chunk_len:
            continue

        if frame_id != cur_frame_id:
            if cur_frame_id is not None and len(chunks) != total_chunks_expected:
                drop_counter += 1
            cur_frame_id = frame_id
            chunks = {}
            total_chunks_expected = total_chunks

        chunks[chunk_idx] = data[HEADER_SIZE : HEADER_SIZE + chunk_len]

        if len(chunks) == total_chunks_expected:
            jpg_bytes = b"".join(chunks[i] for i in range(total_chunks_expected))
            img_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                has_frame = True
                fps_counter += 1
                cv2.putText(frame, f"FPS: {display_fps} | Drop: {display_drop}", (8, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord("s"):
                    filename = f"fpv_{int(time.time())}.jpg"
                    cv2.imwrite(filename, frame)
                    print(f"[SAVED] Da luu anh: {filename}")
            else:
                drop_counter += 1

            cur_frame_id = None
            chunks = {}

        now = time.time()
        if now - last_stat_time >= 1.0:
            display_fps = fps_counter
            display_drop = drop_counter
            print(f"[PC RX] Pkts/s: {pkt_counter} | FPS: {display_fps} | Drop: {display_drop}")
            pkt_counter = fps_counter = drop_counter = 0
            last_stat_time = now

finally:
    sock.close()
    cv2.destroyAllWindows()