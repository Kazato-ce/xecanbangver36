#ifndef WEB_REMOTE_H
#define WEB_REMOTE_H

#include <pgmspace.h>

const char PAGE_REMOTE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Bộ Điều Khiển Không Dây Xe Cân Bằng</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #1e293b;
      --border: #334155;
      --text: #f8fafc;
      --muted: #94a3b8;
      --accent: #38bdf8;
      --green: #22c55e;
      --red: #ef4444;
      --orange: #f97316;
      --yellow: #eab308;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; -webkit-user-select: none; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg);
      color: var(--text);
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 12px;
      touch-action: manipulation;
    }
    .container { width: 100%; max-width: 480px; display: flex; flex-direction: column; gap: 12px; }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 14px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }
    h1 { font-size: 1.15rem; color: var(--accent); text-align: center; }
    .status-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.9rem; }
    .badge {
      padding: 5px 12px; border-radius: 12px; font-weight: 700; font-size: 0.8rem;
    }
    .badge-ok { background: var(--green); color: #000; }
    .badge-wait { background: var(--yellow); color: #000; }
    .badge-err { background: var(--red); color: #fff; }

    .btn-row { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; }
    button {
      border: none; border-radius: 10px; padding: 14px 8px; font-weight: 700;
      font-size: 0.95rem; cursor: pointer; color: #fff; transition: opacity 0.1s, transform 0.1s;
    }
    button:active { transform: scale(0.96); opacity: 0.85; }
    .btn-arm { background: var(--green); color: #000; }
    .btn-disarm { background: var(--orange); }
    .btn-estop { background: var(--red); }

    /* D-PAD DIỀU KHIỂN */
    .dpad-container {
      display: grid;
      grid-template-columns: 80px 80px 80px;
      grid-template-rows: 75px 75px 75px;
      gap: 10px;
      justify-content: center;
      margin: 10px 0;
    }
    .btn-dir {
      background: #334155;
      font-size: 1.5rem;
      border: 2px solid #475569;
      border-radius: 16px;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .btn-dir:active, .btn-dir.active {
      background: var(--accent);
      color: #000;
      border-color: #7dd3fc;
    }
    .btn-stop {
      background: #64748b;
      font-size: 1rem;
      border-radius: 16px;
    }
    .btn-stop:active { background: var(--red); }

    .slider-row {
      display: flex; flex-direction: column; gap: 6px; font-size: 0.85rem; color: var(--muted);
    }
    input[type=range] { width: 100%; accent-color: var(--accent); height: 8px; }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <h1>🎮 Bộ Điều Khiển Không Dây NRF24</h1>
      <div class="status-row">
        <span>Sóng NRF24 (ESP32):</span>
        <span id="rf-status" class="badge badge-wait">ĐANG KIỂM TRA</span>
      </div>
      <div class="status-row">
        <span>Xe STM32 Phản Hồi:</span>
        <span id="link-status" class="badge badge-wait">ĐANG TÌM XE</span>
      </div>
      <div class="status-row">
        <span>Lệnh / Tốc độ:</span>
        <span id="cmd-status" style="font-weight:700; color:var(--accent);">DISARM | 0</span>
      </div>
    </div>

    <div class="card">
      <div class="btn-row">
        <button class="btn-arm" onclick="sendCmd('arm')">⚡ ARM</button>
        <button class="btn-disarm" onclick="sendCmd('disarm')">⏸ DISARM</button>
        <button class="btn-estop" onclick="sendCmd('estop')">🚨 DỪNG</button>
      </div>
    </div>

    <div class="card">
      <div style="text-align:center; font-size:0.9rem; color:var(--muted);">Bàn phím: W (Tiến) | S (Lùi) | A (Trái) | D (Phải) | Space (Dừng)</div>
      <div class="dpad-container">
        <div></div>
        <button class="btn-dir" id="btn-up" onpointerdown="startDrive(350, 0)" onpointerup="stopDrive()">▲</button>
        <div></div>

        <button class="btn-dir" id="btn-left" onpointerdown="startDrive(0, -350)" onpointerup="stopDrive()">◄</button>
        <button class="btn-stop" onclick="stopDrive()">■ STOP</button>
        <button class="btn-dir" id="btn-right" onpointerdown="startDrive(0, 350)" onpointerup="stopDrive()">►</button>

        <div></div>
        <button class="btn-dir" id="btn-down" onpointerdown="startDrive(-350, 0)" onpointerup="stopDrive()">▼</button>
        <div></div>
      </div>

      <div class="slider-row">
        <div style="display:flex; justify-content:space-between;">
          <span>Giới hạn tốc độ:</span>
          <span id="speed-val" style="color:var(--text); font-weight:700;">350</span>
        </div>
        <input type="range" id="speed-slider" min="150" max="800" step="50" value="350" oninput="updateSpeedVal(this.value)">
      </div>
    </div>
  </div>

  <script>
    let curDrive = 0, curSteer = 0;
    let driveInterval = null;
    let maxSpeed = 350;

    function updateSpeedVal(val) {
      maxSpeed = parseInt(val);
      document.getElementById('speed-val').innerText = maxSpeed;
    }

    function sendCmd(action) {
      fetch('/api/cmd?action=' + action, { method: 'POST' });
    }

    function sendDrive(spd, str) {
      fetch('/api/drive?speed=' + spd + '&steer=' + str, { method: 'POST' });
    }

    function startDrive(spdFactor, strFactor) {
      let spd = spdFactor !== 0 ? (spdFactor > 0 ? maxSpeed : -maxSpeed) : 0;
      let str = strFactor !== 0 ? (strFactor > 0 ? maxSpeed : -maxSpeed) : 0;
      curDrive = spd;
      curSteer = str;
      sendDrive(curDrive, curSteer);
      if (!driveInterval) {
        driveInterval = setInterval(() => sendDrive(curDrive, curSteer), 80);
      }
    }

    function stopDrive() {
      if (driveInterval) {
        clearInterval(driveInterval);
        driveInterval = null;
      }
      curDrive = 0;
      curSteer = 0;
      sendDrive(0, 0);
    }

    // Bàn phím máy tính
    window.addEventListener('keydown', (e) => {
      if (e.repeat) return;
      if (e.key === 'w' || e.key === 'W' || e.key === 'ArrowUp') startDrive(maxSpeed, 0);
      else if (e.key === 's' || e.key === 'S' || e.key === 'ArrowDown') startDrive(-maxSpeed, 0);
      else if (e.key === 'a' || e.key === 'A' || e.key === 'ArrowLeft') startDrive(0, -maxSpeed);
      else if (e.key === 'd' || e.key === 'D' || e.key === 'ArrowRight') startDrive(0, maxSpeed);
      else if (e.key === ' ' || e.key === 'Escape') stopDrive();
      else if (e.key === 'x' || e.key === 'X') sendCmd('arm');
      else if (e.key === 'z' || e.key === 'Z') sendCmd('disarm');
    });

    window.addEventListener('keyup', (e) => {
      if (['w','W','s','S','a','A','d','D','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) {
        stopDrive();
      }
    });

    // Cập nhật trạng thái định kỳ 400ms
    setInterval(() => {
      fetch('/api/status')
        .then(res => res.json())
        .then(data => {
          const rfBadge = document.getElementById('rf-status');
          if (data.rf_ok) {
            rfBadge.className = 'badge badge-ok';
            rfBadge.innerText = 'SẴN SÀNG';
          } else {
            rfBadge.className = 'badge badge-err';
            rfBadge.innerText = 'LỖI PHẦN CỨNG';
          }

          const linkBadge = document.getElementById('link-status');
          if (data.link_ok) {
            linkBadge.className = 'badge badge-ok';
            linkBadge.innerText = 'ĐÃ KẾT NỐI (ĐÈN XANH)';
          } else {
            linkBadge.className = 'badge badge-wait';
            linkBadge.innerText = 'ĐANG TÌM XE...';
          }

          document.getElementById('cmd-status').innerText = `${data.cmd} | Tốc độ: ${data.drive} | Rẽ: ${data.steer}`;
        })
        .catch(() => {});
    }, 400);
  </script>
</body>
</html>
)rawliteral";

#endif // WEB_REMOTE_H
