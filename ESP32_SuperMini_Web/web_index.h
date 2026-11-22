#ifndef WEB_INDEX_H
#define WEB_INDEX_H

const char PAGE_MAIN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Robot Cân Bằng 2 Bánh - Điều Khiển & Giám Sát Sóng NRF</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #1e293b;
      --border: #334155;
      --accent: #38bdf8;
      --accent-hover: #0284c7;
      --text: #f8fafc;
      --muted: #94a3b8;
      --green: #22c55e;
      --yellow: #eab308;
      --red: #ef4444;
      --purple: #a855f7;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg);
      color: var(--text);
      padding: 12px;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }
    .container { width: 100%; max-width: 960px; display: flex; flex-direction: column; gap: 12px; }
    header {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px;
      padding: 12px 18px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px;
    }
    h1 { font-size: 1.2rem; font-weight: 700; color: var(--accent); }
    .badge {
      padding: 5px 12px; border-radius: 20px; font-size: 0.8rem; font-weight: 700;
      text-transform: uppercase; letter-spacing: 0.5px; display: inline-flex; align-items: center; gap: 4px;
    }
    .badge-boot { background: #475569; color: #fff; }
    .badge-calib { background: var(--yellow); color: #000; }
    .badge-disarmed { background: #f97316; color: #fff; }
    .badge-armed { background: var(--green); color: #fff; animation: pulse 1.5s infinite; }
    .badge-fault { background: var(--red); color: #fff; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.65; } }

    .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 10px; }
    .card {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px;
    }
    .card-title { font-size: 0.85rem; text-transform: uppercase; color: var(--muted); margin-bottom: 6px; font-weight: 600; display: flex; justify-content: space-between; align-items: center; }
    .val-large { font-size: 1.85rem; font-weight: 700; }
    .val-unit { font-size: 0.9rem; color: var(--muted); font-weight: 400; margin-left: 4px; }

    /* Canvas chart */
    .chart-container { position: relative; width: 100%; height: 180px; background: #0b1120; border-radius: 8px; overflow: hidden; margin-top: 8px; }
    canvas { width: 100%; height: 100%; display: block; }

    /* Buttons */
    .btn-group { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 8px; }
    button {
      flex: 1; min-width: 120px; padding: 12px 14px; border: none; border-radius: 8px;
      font-weight: 700; font-size: 0.95rem; cursor: pointer; transition: 0.15s;
    }
    .btn-arm { background: var(--green); color: #fff; }
    .btn-arm:hover, .btn-arm:active { background: #16a34a; }
    .btn-disarm { background: var(--red); color: #fff; }
    .btn-disarm:hover, .btn-disarm:active { background: #dc2626; }
    .btn-secondary { background: #334155; color: #fff; }
    .btn-secondary:hover { background: #475569; }

    /* D-Pad Controller */
    .dpad-container {
      display: flex; flex-direction: column; align-items: center; gap: 10px; margin-top: 10px;
    }
    .dpad-grid {
      display: grid;
      grid-template-columns: repeat(3, 85px);
      grid-template-rows: repeat(3, 75px);
      gap: 8px;
      justify-content: center;
      touch-action: none;
      user-select: none;
      -webkit-user-select: none;
    }
    .dpad-btn {
      background: #0b1120;
      border: 2px solid var(--border);
      border-radius: 12px;
      color: #fff;
      font-size: 1.35rem;
      font-weight: 700;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      cursor: pointer;
      transition: all 0.1s ease;
      touch-action: none;
      padding: 0;
      min-width: auto;
    }
    .dpad-btn:active, .dpad-btn.active {
      background: var(--accent);
      color: #0f172a;
      transform: scale(0.94);
      border-color: #fff;
      box-shadow: 0 0 15px rgba(56, 189, 248, 0.6);
    }
    .dpad-btn-stop {
      background: #7f1d1d;
      border-color: #ef4444;
      color: #fca5a5;
    }
    .dpad-btn-stop:active, .dpad-btn-stop.active {
      background: #dc2626;
      color: #fff;
    }
    .dpad-sub {
      font-size: 0.65rem;
      font-weight: 600;
      margin-top: 2px;
      text-transform: uppercase;
    }
    .speed-selector {
      display: flex; gap: 6px; justify-content: center; align-items: center; width: 100%; max-width: 420px; margin-top: 4px;
    }
    .speed-btn {
      flex: 1; padding: 6px 8px; font-size: 0.8rem; background: #0b1120; border: 1px solid var(--border); border-radius: 6px; color: var(--muted); cursor: pointer; min-width: auto;
    }
    .speed-btn.active {
      background: var(--accent); color: #0f172a; font-weight: 700; border-color: var(--accent);
    }

    /* Console Log */
    .log-box {
      height: 150px;
      overflow-y: auto;
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
      font-size: 0.75rem;
      color: #93c5fd;
      line-height: 1.45;
      background: #020617;
      border-radius: 6px;
      padding: 8px 10px;
      border: 1px solid #1e293b;
    }
    .log-box div { margin-bottom: 2px; word-break: break-all; }

    .toast {
      position: fixed; bottom: 20px; right: 20px; background: var(--card); border: 1px solid var(--accent);
      color: #fff; padding: 12px 20px; border-radius: 8px; font-size: 0.9rem; display: none;
      box-shadow: 0 4px 12px rgba(0,0,0,0.5); z-index: 999;
    }
  </style>
</head>
<body>
  <div class="container">
    <!-- Header -->
    <header>
      <div>
        <h1>STM32F405 ↔ ESP32 Web Dashboard</h1>
        <div style="font-size: 0.8rem; color: var(--muted);">Điều Khiển Xe Cân Bằng Trực Tiếp & Giám Sát Sóng NRF24</div>
      </div>
      <div style="display: flex; gap: 8px; align-items: center;">
        <span id="badgeState" class="badge badge-boot">BOOT</span>
        <span id="badgeBat" style="font-size: 0.85rem; color: var(--green); font-weight: 700;">14.8V (4S)</span>
      </div>
    </header>

    <!-- Thanh Chẩn Đoán Trạng Thái Nhanh -->
    <div style="background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 10px 16px; display: grid; grid-template-columns: repeat(auto-fit, minmax(190px, 1fr)); gap: 10px; align-items: center;">
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">STM32 UART:</span>
        <span id="badgeStmLink" class="badge" style="background:var(--red);">MẤT TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">MPU6050:</span>
        <span id="badgeMpu" class="badge" style="background:var(--muted);">CHỜ TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">SÓNG NRF24:</span>
        <span id="badgeNrf" class="badge" style="background:var(--muted);">CHỜ TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">TƯ THẾ XE:</span>
        <span id="badgeUpright" class="badge" style="background:var(--yellow); color:#000;">NGHIÊNG</span>
      </div>
    </div>

    <!-- 🎮 BẢNG ĐIỀU KHIỂN CHẠY XE TRỰC TIẾP BẰNG ESP32 MINI -->
    <div class="card" style="border-left: 4px solid var(--green);">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:8px;">
        <div class="card-title" style="color:var(--green); font-size:0.95rem; font-weight:700; margin:0;">
          🎮 ĐIỀU KHIỂN CHẠY XE TRỰC TIẾP BẰNG ESP32 MINI (TOUCH PAD / PHÍM PC)
        </div>
        <span id="driveStatusText" style="font-size:0.85rem; color:var(--accent); font-weight:700;">Sẵn sàng lái</span>
      </div>
      <div style="font-size:0.8rem; color:var(--muted); margin: 6px 0 10px 0;">
        Chạm giữ nút để xe chạy theo 8 hướng mong muốn, thả tay ra xe sẽ <b>TỰ ĐỘNG DỪNG (STOP)</b> an toàn. Hỗ trợ cả phím W/A/S/D, Mũi tên và Space trên máy tính!
      </div>

      <!-- Nút ARM / DISARM -->
      <div class="btn-group">
        <button class="btn-arm" onclick="sendCmd('arm')">⚡ BẬT CÂN BẰNG (ARM)</button>
        <button class="btn-disarm" onclick="sendCmd('disarm')">🛑 TẮT ĐỘNG CƠ (DISARM / CẮT KHẨN CẤP)</button>
      </div>

      <!-- Bàn D-Pad điều khiển 8 hướng -->
      <div class="dpad-container">
        <!-- Bộ chọn tốc độ -->
        <div class="speed-selector">
          <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">Tốc độ:</span>
          <button class="speed-btn" onclick="setDriveSpeed(0.20, this)">20% Chậm</button>
          <button class="speed-btn active" onclick="setDriveSpeed(0.40, this)">40% Vừa</button>
          <button class="speed-btn" onclick="setDriveSpeed(0.70, this)">70% Nhanh</button>
          <button class="speed-btn" onclick="setDriveSpeed(1.00, this)">100% Max</button>
        </div>

        <div class="dpad-grid">
          <!-- Hàng 1: Tiến-Trái, Tiến, Tiến-Phải -->
          <button class="dpad-btn" id="btnFwdLeft" onmousedown="startDrive('fwd_left')" onmouseup="stopDrive()" ontouchstart="startDrive('fwd_left', event)" ontouchend="stopDrive(event)">
            ↖<span class="dpad-sub">Chéo Trái</span>
          </button>
          <button class="dpad-btn" id="btnFwd" onmousedown="startDrive('fwd')" onmouseup="stopDrive()" ontouchstart="startDrive('fwd', event)" ontouchend="stopDrive(event)">
            ▲<span class="dpad-sub">TIẾN</span>
          </button>
          <button class="dpad-btn" id="btnFwdRight" onmousedown="startDrive('fwd_right')" onmouseup="stopDrive()" ontouchstart="startDrive('fwd_right', event)" ontouchend="stopDrive(event)">
            ↗<span class="dpad-sub">Chéo Phải</span>
          </button>

          <!-- Hàng 2: Trái, Dừng, Phải -->
          <button class="dpad-btn" id="btnLeft" onmousedown="startDrive('left')" onmouseup="stopDrive()" ontouchstart="startDrive('left', event)" ontouchend="stopDrive(event)">
            ◄<span class="dpad-sub">TRÁI</span>
          </button>
          <button class="dpad-btn dpad-btn-stop" id="btnStop" onclick="forceStop()">
            ⏹<span class="dpad-sub">DỪNG</span>
          </button>
          <button class="dpad-btn" id="btnRight" onmousedown="startDrive('right')" onmouseup="stopDrive()" ontouchstart="startDrive('right', event)" ontouchend="stopDrive(event)">
            ►<span class="dpad-sub">PHẢI</span>
          </button>

          <!-- Hàng 3: Lùi-Trái, Lùi, Lùi-Phải -->
          <button class="dpad-btn" id="btnBwdLeft" onmousedown="startDrive('bwd_left')" onmouseup="stopDrive()" ontouchstart="startDrive('bwd_left', event)" ontouchend="stopDrive(event)">
            ↙<span class="dpad-sub">Lùi Trái</span>
          </button>
          <button class="dpad-btn" id="btnBwd" onmousedown="startDrive('bwd')" onmouseup="stopDrive()" ontouchstart="startDrive('bwd', event)" ontouchend="stopDrive(event)">
            ▼<span class="dpad-sub">LÙI</span>
          </button>
          <button class="dpad-btn" id="btnBwdRight" onmousedown="startDrive('bwd_right')" onmouseup="stopDrive()" ontouchstart="startDrive('bwd_right', event)" ontouchend="stopDrive(event)">
            ↘<span class="dpad-sub">Lùi Phải</span>
          </button>
        </div>
      </div>
    </div>

    <!-- 📡 BẢNG CHẨN ĐOÁN & NHẬT KÝ GÓI TIN NHẬN TỪ ESP DEVKIT 1 QUA NRF24 -->
    <div class="card" style="border-left: 4px solid var(--accent);">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:8px;">
        <div class="card-title" style="color:var(--accent); font-size:0.95rem; font-weight:700; margin:0;">
          📡 GIÁM SÁT & NHẬT KÝ GÓI TIN TỪ ESP DEVKIT 1 QUA NRF24L01
        </div>
        <span id="nrfLinkText" style="font-size:0.85rem; font-weight:700; color:var(--yellow);">CHỜ TÍN HIỆU...</span>
      </div>
      <div style="font-size: 0.8rem; color: var(--muted); margin: 6px 0 10px 0;">
        Theo dõi trạng thái chip SPI2, đếm số gói tin nhận được và hiển thị từng gói dữ liệu joystick phát từ ESP32 DevKit 1.
      </div>

      <div class="grid-4">
        <!-- Ô 1: Phần cứng SPI -->
        <div style="background:#0b1120; padding:10px 12px; border-radius:8px; border:1px solid #1e293b;">
          <div style="font-size:0.75rem; color:var(--muted); font-weight:600;">PHẦN CỨNG SPI2</div>
          <div id="diagNrfHw" style="font-size:1.05rem; font-weight:700; margin-top:4px; color:var(--red);">CHƯA KẾT NỐI</div>
          <div id="diagNrfRaw" style="font-size:0.75rem; color:var(--muted); margin-top:2px;">Mã STATUS: -- | Kênh: 76</div>
        </div>

        <!-- Ô 2: Trạng thái Sóng -->
        <div style="background:#0b1120; padding:10px 12px; border-radius:8px; border:1px solid #1e293b;">
          <div style="font-size:0.75rem; color:var(--muted); font-weight:600;">KẾT NỐI TAY CẦM</div>
          <div id="diagNrfLink" style="font-size:1.05rem; font-weight:700; margin-top:4px; color:var(--yellow);">CHỜ PHÁT SÓNG</div>
          <div id="diagNrfAge" style="font-size:0.75rem; color:var(--muted); margin-top:2px;">Gói cuối: -- ms trước</div>
        </div>

        <!-- Ô 3: Tổng số gói nhận -->
        <div style="background:#0b1120; padding:10px 12px; border-radius:8px; border:1px solid #1e293b;">
          <div style="font-size:0.75rem; color:var(--muted); font-weight:600;">GÓI TIN ĐÃ NHẬN</div>
          <div id="diagNrfPkts" style="font-size:1.35rem; font-weight:700; margin-top:2px; color:var(--green);">0</div>
          <div id="diagNrfSeq" style="font-size:0.75rem; color:var(--muted);">Gói số: #0</div>
        </div>

        <!-- Ô 4: Lệnh hiện tại từ NRF -->
        <div style="background:#0b1120; padding:10px 12px; border-radius:8px; border:1px solid #1e293b;">
          <div style="font-size:0.75rem; color:var(--muted); font-weight:600;">LỆNH TỪ TAY CẦM</div>
          <div id="diagNrfCmdText" style="font-size:0.95rem; font-weight:700; margin-top:4px; color:var(--text);">ĐỨNG YÊN (STOP)</div>
          <div id="diagNrfCmdVal" style="font-size:0.75rem; color:var(--muted); margin-top:2px;">Drive: 0 | Steer: 0</div>
        </div>
      </div>

      <!-- Cửa sổ Nhật Ký Gói Tin Thời Gian Thực từ ESP32 DevKit 1 -->
      <div style="margin-top:12px; background:#020617; border:1px solid #1e293b; border-radius:8px; padding:10px 12px;">
        <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px; flex-wrap:wrap; gap:6px;">
          <span style="font-size:0.8rem; font-weight:700; color:#38bdf8; display:flex; align-items:center; gap:6px;">
            📜 NHẬT KÝ GÓI TIN NHẬN TỪ ESP DEVKIT 1 (NRF24) THỜI GIAN THỰC
          </span>
          <div style="display:flex; gap:6px;">
            <button style="padding:4px 8px; font-size:0.7rem; background:#1e293b; color:#94a3b8; border-radius:4px; min-width:auto;" onclick="toggleNrfAutoScroll()">Cuộn: <span id="txtAutoScroll">BẬT</span></button>
            <button style="padding:4px 8px; font-size:0.7rem; background:#1e293b; color:#ef4444; border-radius:4px; min-width:auto;" onclick="clearNrfLog()">Xóa Log</button>
          </div>
        </div>
        <div id="nrfLogConsole" class="log-box">
          <div style="color:#64748b;">[Đang chờ luồng gói tin NRF từ ESP32 DevKit 1...]</div>
        </div>
      </div>
    </div>

    <!-- Hàng thẻ thông số đo góc & tốc độ thời gian thực -->
    <div class="grid-4">
      <div class="card">
        <div class="card-title">Góc Nghiêng Pitch</div>
        <div class="val-large" id="valPitch" style="color:var(--accent);">0.00<span class="val-unit">°</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Điểm đặt mục tiêu: <span id="valTarget">0.00</span>°</div>
      </div>
      <div class="card">
        <div class="card-title">Tốc Độ Góc Gyro</div>
        <div class="val-large" id="valRate" style="color:var(--yellow);">0.0<span class="val-unit">°/s</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Trục cân bằng X (Lọc bù 42Hz)</div>
      </div>
      <div class="card">
        <div class="card-title">Tốc Độ Bánh Xe (RPM)</div>
        <div class="val-large"><span id="valRpmL">0</span> / <span id="valRpmR">0</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Trái / Phải (TMC2208 vi bước 1/16)</div>
      </div>
      <div class="card">
        <div class="card-title">Cảm Biến Vật Cản</div>
        <div class="val-large" id="valDistFront">--<span class="val-unit">cm</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">ToF VL53L0X: <span id="valTof">--</span> mm</div>
      </div>
    </div>

    <!-- Đồ thị trực quan thời gian thực (Pitch & Mục tiêu) -->
    <div class="card">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <div class="card-title">Sóng Góc Nghiêng Thời Gian Thực (Pitch & Mục Tiêu)</div>
        <div style="font-size: 0.75rem; color: var(--accent);">Tần số 20 Hz (50ms) | Xanh lá: Pitch, Xanh lam: Target</div>
      </div>
      <div class="chart-container">
        <canvas id="chartCanvas"></canvas>
      </div>
    </div>
  </div>

  <div id="toast" class="toast">Thông báo</div>

  <script>
    const stateNames = ["BOOT", "CALIBRATING", "DISARMED", "ARMED (BALANCING)", "FAULT"];
    const stateBadges = ["badge-boot", "badge-calib", "badge-disarmed", "badge-armed", "badge-fault"];

    function showToast(msg) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.style.display = 'block';
      setTimeout(() => { t.style.display = 'none'; }, 2200);
    }

    // ──────────────────────────────────────────────
    // ĐIỀU KHIỂN CHẠY XE QUA ESP32 MINI
    // ──────────────────────────────────────────────
    let currentSpeedScale = 0.40;
    let driveIntervalTimer = null;
    let currentDir = null;
    let currentVehicleState = 0;

    function setDriveSpeed(spd, btn) {
      currentSpeedScale = spd;
      document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
      if (btn) btn.classList.add('active');
      showToast(`Đã chọn tốc độ: ${Math.round(spd * 100)}%`);
    }

    function getDriveVector(dir) {
      const spd = currentSpeedScale;
      switch (dir) {
        case 'fwd':       return { speed: spd, steer: 0.0, name: '▲ TIẾN THẲNG' };
        case 'bwd':       return { speed: -spd, steer: 0.0, name: '▼ LÙI THẲNG' };
        case 'left':      return { speed: 0.0, steer: -spd * 1.2, name: '◄ QUAY TRÁI' };
        case 'right':     return { speed: 0.0, steer: spd * 1.2, name: '► QUAY PHẢI' };
        case 'fwd_left':  return { speed: spd * 0.8, steer: -spd * 0.7, name: '↖ TIẾN - CHÉO TRÁI' };
        case 'fwd_right': return { speed: spd * 0.8, steer: spd * 0.7, name: '↗ TIẾN - CHÉO PHẢI' };
        case 'bwd_left':  return { speed: -spd * 0.8, steer: -spd * 0.7, name: '↙ LÙI - CHÉO TRÁI' };
        case 'bwd_right': return { speed: -spd * 0.8, steer: spd * 0.7, name: '↘ LÙI - CHÉO PHẢI' };
        default:          return { speed: 0.0, steer: 0.0, name: 'DỪNG (STOP)' };
      }
    }

    function sendDrivePacket(speed, steer) {
      fetch(`/api/drive?speed=${speed.toFixed(2)}&steer=${steer.toFixed(2)}`, { method: 'POST' })
        .catch(() => {});
    }

    function startDrive(dir, e) {
      if (e) { e.preventDefault(); }
      if (currentVehicleState !== 3) {
        showToast("⚠️ Xe chưa ARM! Bấm '⚡ BẬT CÂN BẰNG (ARM)' trước khi lái!");
      }
      currentDir = dir;
      const v = getDriveVector(dir);
      document.getElementById('driveStatusText').innerText = 'Đang chạy: ' + v.name;
      document.getElementById('driveStatusText').style.color = 'var(--accent)';

      highlightButton(dir, true);
      sendDrivePacket(v.speed, v.steer);

      if (driveIntervalTimer) clearInterval(driveIntervalTimer);
      driveIntervalTimer = setInterval(() => {
        if (currentDir) {
          const curV = getDriveVector(currentDir);
          sendDrivePacket(curV.speed, curV.steer);
        }
      }, 100);
    }

    function stopDrive(e) {
      if (e) { e.preventDefault(); }
      if (driveIntervalTimer) {
        clearInterval(driveIntervalTimer);
        driveIntervalTimer = null;
      }
      if (currentDir) {
        highlightButton(currentDir, false);
      }
      currentDir = null;
      document.getElementById('driveStatusText').innerText = 'Xe đã DỪNG (STOP)';
      document.getElementById('driveStatusText').style.color = 'var(--muted)';
      sendDrivePacket(0.0, 0.0);
    }

    function forceStop() {
      stopDrive();
      sendCmd('stop');
      showToast("🛑 Lệnh DỪNG KHẨN CẤP!");
    }

    function highlightButton(dir, active) {
      const idMap = {
        'fwd': 'btnFwd', 'bwd': 'btnBwd', 'left': 'btnLeft', 'right': 'btnRight',
        'fwd_left': 'btnFwdLeft', 'fwd_right': 'btnFwdRight', 'bwd_left': 'btnBwdLeft', 'bwd_right': 'btnBwdRight'
      };
      const btn = document.getElementById(idMap[dir]);
      if (btn) {
        if (active) btn.classList.add('active');
        else btn.classList.remove('active');
      }
    }

    // Điều khiển bằng Bàn phím máy tính (W, A, S, D, Mũi tên, Space)
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT') return;
      if (e.repeat) return;
      const key = e.key.toLowerCase();
      if (key === 'w' || key === 'arrowup') startDrive('fwd');
      else if (key === 's' || key === 'arrowdown') startDrive('bwd');
      else if (key === 'a' || key === 'arrowleft') startDrive('left');
      else if (key === 'd' || key === 'arrowright') startDrive('right');
      else if (key === ' ' || key === 'escape') forceStop();
    });

    window.addEventListener('keyup', (e) => {
      if (e.target.tagName === 'INPUT') return;
      const key = e.key.toLowerCase();
      if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        stopDrive();
      }
    });

    function sendCmd(act) {
      fetch(`/api/cmd?action=${act}`, { method: 'POST' })
        .then(r => r.text())
        .then(res => { showToast("Đã gửi lệnh: " + act.toUpperCase()); })
        .catch(err => { showToast("Lỗi gửi lệnh: " + err); });
    }

    // ──────────────────────────────────────────────
    // NHẬT KÝ GÓI TIN NRF24 THỜI GIAN THỰC
    // ──────────────────────────────────────────────
    let lastLoggedSeq = -1;
    let nrfAutoScroll = true;
    const maxLogEntries = 80;

    function clearNrfLog() {
      document.getElementById('nrfLogConsole').innerHTML = '<div style="color:#64748b;">[Đã xóa nhật ký...]</div>';
    }

    function toggleNrfAutoScroll() {
      nrfAutoScroll = !nrfAutoScroll;
      document.getElementById('txtAutoScroll').innerText = nrfAutoScroll ? "BẬT" : "TẮT";
    }

    function appendNrfLog(seq, cmdText, drv, str, st, age) {
      const consoleEl = document.getElementById('nrfLogConsole');
      const now = new Date();
      const timeStr = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
      const row = document.createElement('div');
      row.innerHTML = `<span style="color:#64748b;">[${timeStr}]</span> <span style="color:#22c55e; font-weight:700;">PKT #${seq}</span> | <span style="color:#38bdf8;">${cmdText}</span> (Drv: ${drv}, Str: ${str}) | <span style="color:#94a3b8;">ST: 0x${st.toString(16).toUpperCase().padStart(2,'0')}</span> | <span style="color:#eab308;">${age}ms</span>`;
      consoleEl.appendChild(row);

      while (consoleEl.children.length > maxLogEntries) {
        consoleEl.removeChild(consoleEl.firstChild);
      }
      if (nrfAutoScroll) {
        consoleEl.scrollTop = consoleEl.scrollHeight;
      }
    }

    // ──────────────────────────────────────────────
    // ĐỒ THỊ CANVAS THỜI GIAN THỰC
    // ──────────────────────────────────────────────
    const canvas = document.getElementById('chartCanvas');
    const ctx = canvas.getContext('2d');
    const maxPoints = 120;
    const historyPitch = new Array(maxPoints).fill(0);
    const historyTarget = new Array(maxPoints).fill(0);

    function drawChart() {
      const container = canvas.parentElement;
      const targetW = (container && container.clientWidth > 50) ? container.clientWidth : 600;
      const targetH = (container && container.clientHeight > 50) ? container.clientHeight : 180;

      if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
      }

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const w = canvas.width;
      const h = canvas.height;
      const midY = h / 2;

      // Lưới tọa độ
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let y = 0; y < h; y += 30) {
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
      }
      ctx.stroke();

      // Đường 0 độ ở giữa
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, midY);
      ctx.lineTo(w, midY);
      ctx.stroke();

      const scale = h / 40.0; // thang +-20 độ

      // Vẽ Target Pitch (màu xanh dương)
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const y = midY - (historyTarget[i] * scale);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Vẽ Pitch thực tế (màu xanh lá)
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const y = midY - (historyPitch[i] * scale);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // Polling Telemetry data mỗi 80 ms
    setInterval(() => {
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          if (!d) return;

          const pitch = (typeof d.pitch === 'number' && Number.isFinite(d.pitch)) ? d.pitch : 0.0;
          const target = (typeof d.target === 'number' && Number.isFinite(d.target)) ? d.target : 0.0;
          const rate = (typeof d.rate === 'number' && Number.isFinite(d.rate)) ? d.rate : 0.0;
          const rpmL = (typeof d.rpm_l === 'number') ? d.rpm_l : 0.0;
          const rpmR = (typeof d.rpm_r === 'number') ? d.rpm_r : 0.0;
          const distF = (typeof d.dist_f === 'number') ? d.dist_f : 999;
          const tof = (typeof d.tof_mm === 'number') ? d.tof_mm : 8190;

          // Cập nhật thẻ đo góc
          const elPitch = document.getElementById('valPitch');
          elPitch.innerHTML = pitch.toFixed(2) + '<span class="val-unit">°</span>';
          if (Math.abs(pitch) < 3.0) elPitch.style.color = 'var(--green)';
          else if (Math.abs(pitch) < 15.0) elPitch.style.color = 'var(--yellow)';
          else elPitch.style.color = 'var(--red)';

          document.getElementById('valTarget').innerText = target.toFixed(2);
          document.getElementById('valRate').innerHTML = rate.toFixed(1) + '<span class="val-unit">°/s</span>';
          document.getElementById('valRpmL').innerText = Math.round(rpmL);
          document.getElementById('valRpmR').innerText = Math.round(rpmR);
          document.getElementById('valDistFront').innerHTML = (distF < 400 ? distF : '--') + '<span class="val-unit">cm</span>';
          document.getElementById('valTof').innerText = (tof < 8000 ? tof : '--');

          const b = document.getElementById('badgeState');
          const st = d.state || 0;
          currentVehicleState = st;
          b.innerText = stateNames[st] || "UNKNOWN";
          b.className = "badge " + (stateBadges[st] || "badge-boot");

          // Trạng thái UART STM32
          const bLink = document.getElementById('badgeStmLink');
          if (d.stm_link) {
            bLink.innerText = "ĐÃ KẾT NỐI (UART OK)";
            bLink.style.background = "var(--green)";
            bLink.style.color = "#fff";
          } else {
            bLink.innerText = "MẤT TÍN HIỆU STM32";
            bLink.style.background = "var(--red)";
            bLink.style.color = "#fff";
          }

          // Trạng thái MPU6050
          const bMpu = document.getElementById('badgeMpu');
          if (d.stm_link && d.mpu_ok) {
            bMpu.innerText = "BÌNH THƯỜNG (0 LỖI)";
            bMpu.style.background = "var(--green)";
            bMpu.style.color = "#fff";
          } else if (d.stm_link) {
            bMpu.innerText = `LỖI I2C (${d.imu_err})`;
            bMpu.style.background = "var(--red)";
            bMpu.style.color = "#fff";
          } else {
            bMpu.innerText = "--";
            bMpu.style.background = "var(--muted)";
          }

          // Trạng thái Sóng NRF24 nhanh
          const bNrf = document.getElementById('badgeNrf');
          if (d.stm_link && d.nrf_link) {
            bNrf.innerText = "TAY CẦM ĐÃ NỐI 🟢";
            bNrf.style.background = "var(--green)";
            bNrf.style.color = "#fff";
          } else if (d.stm_link && d.nrf_hw) {
            bNrf.innerText = "CHỜ TAY CẦM (NRF OK) 🟡";
            bNrf.style.background = "var(--yellow)";
            bNrf.style.color = "#000";
          } else if (d.stm_link) {
            bNrf.innerText = "LỖI NRF XE (SPI2)";
            bNrf.style.background = "var(--red)";
            bNrf.style.color = "#fff";
          } else {
            bNrf.innerText = "--";
            bNrf.style.background = "var(--muted)";
          }

          const bUpr = document.getElementById('badgeUpright');
          if (d.upright) {
            bUpr.innerText = "SẴN SÀNG ARM (±8.0°)";
            bUpr.style.background = "var(--green)";
            bUpr.style.color = "#fff";
          } else {
            bUpr.innerText = "ĐANG NGHIÊNG";
            bUpr.style.background = "var(--yellow)";
            bUpr.style.color = "#000";
          }

          // ──────────────────────────────────────────────
          // CẬP NHẬT BẢNG GIÁM SÁT NRF24 CHI TIẾT
          // ──────────────────────────────────────────────
          const nrfPkts = (typeof d.nrf_pkts === 'number') ? d.nrf_pkts : 0;
          const nrfSeq  = (typeof d.nrf_seq === 'number') ? d.nrf_seq : 0;
          const nrfAge  = (typeof d.nrf_age === 'number') ? d.nrf_age : 999999;
          const nrfDrv  = (typeof d.nrf_drv === 'number') ? d.nrf_drv : 0;
          const nrfStr  = (typeof d.nrf_str === 'number') ? d.nrf_str : 0;
          const nrfSt   = (typeof d.nrf_st === 'number') ? d.nrf_st : 0;

          document.getElementById('diagNrfPkts').innerText = nrfPkts.toLocaleString();
          document.getElementById('diagNrfSeq').innerText = 'Gói số: #' + nrfSeq;
          document.getElementById('diagNrfRaw').innerText = `Mã STATUS: 0x${nrfSt.toString(16).toUpperCase().padStart(2, '0')} | Kênh: 76`;

          const elDiagHw = document.getElementById('diagNrfHw');
          if (d.nrf_hw) {
            elDiagHw.innerText = "🟢 SẴN SÀNG (SPI OK)";
            elDiagHw.style.color = "var(--green)";
          } else {
            elDiagHw.innerText = "🔴 LỖI SPI (Mất kết nối)";
            elDiagHw.style.color = "var(--red)";
          }

          const elDiagLink = document.getElementById('diagNrfLink');
          const elDiagAge  = document.getElementById('diagNrfAge');
          const elNrfText  = document.getElementById('nrfLinkText');

          if (d.nrf_link) {
            elDiagLink.innerText = "🟢 ĐÃ KẾT NỐI (NHẬN TỐT)";
            elDiagLink.style.color = "var(--green)";
            elDiagAge.innerText = `Gói cuối: ${nrfAge} ms trước`;
            elNrfText.innerText = "🟢 TAY CẦM ĐANG KẾT NỐI";
            elNrfText.style.color = "var(--green)";
          } else if (d.nrf_hw) {
            elDiagLink.innerText = "🟡 CHỜ TAY CẦM PHÁT SÓNG";
            elDiagLink.style.color = "var(--yellow)";
            elDiagAge.innerText = (nrfAge < 60000) ? `Mất sóng ${Math.round(nrfAge/1000)}s` : "Chưa nhận gói nào";
            elNrfText.innerText = "🟡 CHỜ TAY CẦM PHÁT SÓNG";
            elNrfText.style.color = "var(--yellow)";
          } else {
            elDiagLink.innerText = "🔴 KHÔNG CÓ SÓNG";
            elDiagLink.style.color = "var(--red)";
            elDiagAge.innerText = "Kiểm tra module RF";
            elNrfText.innerText = "🔴 LỖI NRF XE";
            elNrfText.style.color = "var(--red)";
          }

          // Hiển thị trực quan lệnh nhận từ NRF
          const elCmdText = document.getElementById('diagNrfCmdText');
          const elCmdVal  = document.getElementById('diagNrfCmdVal');
          elCmdVal.innerText = `Drive: ${nrfDrv} | Steer: ${nrfStr}`;
          let cmdName = "ĐỨNG YÊN (STOP)";

          if (Math.abs(nrfDrv) < 100 && Math.abs(nrfStr) < 100) {
            cmdName = "ĐỨNG YÊN (STOP)";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--muted)";
          } else if (nrfDrv > 150 && Math.abs(nrfStr) < 150) {
            cmdName = "▲ TIẾN THẲNG (+" + nrfDrv + ")";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--accent)";
          } else if (nrfDrv < -150 && Math.abs(nrfStr) < 150) {
            cmdName = "▼ LÙI THẲNG (" + nrfDrv + ")";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--yellow)";
          } else if (Math.abs(nrfDrv) < 150 && nrfStr < -150) {
            cmdName = "◄ QUAY TRÁI (" + nrfStr + ")";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--purple)";
          } else if (Math.abs(nrfDrv) < 150 && nrfStr > 150) {
            cmdName = "► QUAY PHẢI (+" + nrfStr + ")";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--purple)";
          } else if (nrfDrv > 100 && nrfStr < -100) {
            cmdName = "↖ TIẾN - CHÉO TRÁI";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--accent)";
          } else if (nrfDrv > 100 && nrfStr > 100) {
            cmdName = "↗ TIẾN - CHÉO PHẢI";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--accent)";
          } else if (nrfDrv < -100 && nrfStr < -100) {
            cmdName = "↙ LÙI - CHÉO TRÁI";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--yellow)";
          } else if (nrfDrv < -100 && nrfStr > 100) {
            cmdName = "↘ LÙI - CHÉO PHẢI";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--yellow)";
          } else {
            cmdName = "DI CHUYỂN";
            elCmdText.innerText = cmdName;
            elCmdText.style.color = "var(--text)";
          }

          // Ghi vào Nhật ký Stream Log nếu có gói tin mới từ ESP32 DevKit 1
          if (nrfSeq !== lastLoggedSeq && nrfPkts > 0) {
            lastLoggedSeq = nrfSeq;
            appendNrfLog(nrfSeq, cmdName, nrfDrv, nrfStr, nrfSt, nrfAge);
          }

          // Cập nhật chart
          historyPitch.shift();
          historyPitch.push(pitch);
          historyTarget.shift();
          historyTarget.push(target);
          drawChart();
        })
        .catch(() => {});
    }, 80);
  </script>
</body>
</html>
)rawliteral";

#endif /* WEB_INDEX_H */
