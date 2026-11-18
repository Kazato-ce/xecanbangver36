#ifndef WEB_INDEX_H
#define WEB_INDEX_H

const char PAGE_MAIN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot Tự Cân Bằng - Dashboard & Realtime PID</title>
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
      --orange: #f97316;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg);
      color: var(--text);
      padding: 14px;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }
    .container { width: 100%; max-width: 920px; display: flex; flex-direction: column; gap: 14px; }
    header {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px;
      padding: 14px 20px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px;
    }
    h1 { font-size: 1.2rem; font-weight: 700; color: var(--accent); }
    .badge {
      padding: 6px 14px; border-radius: 20px; font-size: 0.85rem; font-weight: 700;
      text-transform: uppercase; letter-spacing: 0.5px;
    }
    .badge-boot { background: #475569; color: #fff; }
    .badge-calib { background: var(--yellow); color: #000; }
    .badge-disarmed { background: var(--orange); color: #fff; }
    .badge-armed { background: var(--green); color: #fff; animation: pulse 1.5s infinite; }
    .badge-fault { background: var(--red); color: #fff; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.6; } }

    .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; }
    .card {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px;
    }
    .card-title { font-size: 0.8rem; text-transform: uppercase; color: var(--muted); margin-bottom: 6px; font-weight: 600; }
    .val-large { font-size: 1.8rem; font-weight: 700; }
    .val-unit { font-size: 0.9rem; color: var(--muted); font-weight: 400; margin-left: 4px; }

    /* Canvas chart */
    .chart-container { position: relative; width: 100%; height: 200px; background: #0b1120; border-radius: 8px; overflow: hidden; margin-top: 8px; }
    canvas { width: 100%; height: 100%; display: block; }

    /* Controls & PID */
    .btn-group { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 8px; }
    button {
      flex: 1; min-width: 120px; padding: 12px 16px; border: none; border-radius: 8px;
      font-weight: 700; font-size: 0.95rem; cursor: pointer; transition: 0.2s;
    }
    .btn-arm { background: var(--green); color: #fff; }
    .btn-arm:hover { background: #16a34a; }
    .btn-disarm { background: var(--red); color: #fff; }
    .btn-disarm:hover { background: #dc2626; }
    .btn-submit { background: var(--accent); color: #0f172a; width: 100%; margin-top: 10px; }
    .btn-submit:hover { background: var(--accent-hover); }

    /* D-Pad Virtual Controller */
    .dpad-container { display: flex; flex-direction: column; align-items: center; gap: 8px; margin: 12px 0; }
    .dpad-row { display: flex; gap: 10px; justify-content: center; }
    .btn-dpad {
      width: 80px; height: 56px; border-radius: 8px; font-weight: 700; font-size: 1.05rem;
      background: #334155; color: #fff; display: flex; flex-direction: column; align-items: center; justify-content: center;
      user-select: none; -webkit-user-select: none; cursor: pointer; transition: 0.15s;
    }
    .btn-dpad:active { background: var(--accent); color: #0f172a; transform: scale(0.96); }
    .btn-dpad-stop { background: #475569; }
    .btn-dpad-stop:active { background: var(--red); color: #fff; }

    .form-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 12px; margin-top: 10px; }
    .field { display: flex; flex-direction: column; gap: 4px; }
    label { font-size: 0.85rem; color: var(--muted); font-weight: 600; display: flex; justify-content: space-between; }
    input[type=number] {
      background: #0b1120; border: 1px solid var(--border); border-radius: 6px;
      color: #fff; padding: 8px 10px; font-size: 1rem; font-family: monospace;
    }
    input[type=range] { width: 100%; accent-color: var(--accent); cursor: pointer; margin-top: 2px; }

    .toast {
      position: fixed; bottom: 20px; right: 20px; background: var(--card); border: 1px solid var(--accent);
      color: #fff; padding: 12px 20px; border-radius: 8px; font-size: 0.9rem; display: none;
      box-shadow: 0 4px 12px rgba(0,0,0,0.4); z-index: 999;
    }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <div>
        <h1>STM32F405 ↔ ESP32 Web Dashboard</h1>
        <div style="font-size: 0.8rem; color: var(--muted);">Robot Xe Tự Cân Bằng 2 Bánh (TMC2208 + MPU6050)</div>
      </div>
      <div style="display: flex; gap: 8px; align-items: center;">
        <span id="badgeState" class="badge badge-boot">BOOT</span>
        <span id="badgeBat" style="font-size: 0.85rem; color: var(--green); font-weight: 700;">14.8V (4S)</span>
      </div>
    </header>

    <!-- Thanh Chẩn Đoán Phần Cứng Real-Time -->
    <div style="background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 10px 16px; display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 10px; align-items: center;">
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">STM32 UART:</span>
        <span id="badgeStmLink" class="badge" style="background:var(--red); padding:3px 8px; font-size:0.75rem;">MẤT TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">MPU6050:</span>
        <span id="badgeMpu" class="badge" style="background:var(--muted); padding:3px 8px; font-size:0.75rem;">CHỜ TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">SÓNG NRF24:</span>
        <span id="badgeNrf" class="badge" style="background:var(--muted); padding:3px 8px; font-size:0.75rem;">CHỜ TÍN HIỆU</span>
      </div>
      <div style="display:flex; align-items:center; gap:8px;">
        <span style="font-size:0.8rem; color:var(--muted); font-weight:600;">TƯ THẾ XE:</span>
        <span id="badgeUpright" class="badge" style="background:var(--yellow); color:#000; padding:3px 8px; font-size:0.75rem;">NGHIÊNG</span>
      </div>
    </div>

    <!-- Hàng thông số chính -->
    <div class="grid-4">
      <div class="card">
        <div class="card-title">Góc Nghiêng Pitch</div>
        <div class="val-large" id="valPitch">0.00<span class="val-unit">°</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Góc đặt: <span id="valTarget">0.00</span>°</div>
      </div>
      <div class="card">
        <div class="card-title">Tốc Độ Góc Gyro</div>
        <div class="val-large" id="valRate">0.0<span class="val-unit">°/s</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Trục cân bằng: X</div>
      </div>
      <div class="card">
        <div class="card-title">Tốc Độ Bánh Xe (RPM)</div>
        <div class="val-large"><span id="valRpmL">0</span> / <span id="valRpmR">0</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">Trái / Phải (1/16 vi bước)</div>
      </div>
      <div class="card">
        <div class="card-title">Cảm Biến Vật Cản</div>
        <div class="val-large" id="valDistFront">--<span class="val-unit">cm</span></div>
        <div style="font-size: 0.75rem; color: var(--muted); margin-top: 4px;">ToF: <span id="valTof">--</span> mm</div>
      </div>
    </div>

    <!-- Đồ thị trực quan thời gian thực -->
    <div class="card">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <div class="card-title">Đồ Thị Thời Gian Thực (Pitch & Mục Tiêu)</div>
        <div style="font-size: 0.75rem; color: var(--accent);">20 Hz (50ms)</div>
      </div>
      <div class="chart-container">
        <canvas id="chartCanvas"></canvas>
      </div>
    </div>

    <!-- Nút điều khiển nhanh -->
    <div class="card">
      <div class="card-title">Điều Khiển Trạng Thái Robot</div>
      <div class="btn-group">
        <button class="btn-arm" onclick="sendCmd('arm')">⚡ BẬT CÂN BẰNG (ARM - X)</button>
        <button class="btn-disarm" onclick="sendCmd('disarm')">🛑 TẮT ĐỘNG CƠ (DISARM - Z)</button>
      </div>
    </div>

    <!-- Bàn Điều Khiển Xe Di Chuyển (D-Pad & Phím W/A/S/D/Space) -->
    <div class="card">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <div class="card-title">🎮 Bàn Điều Khiển Di Chuyển (Phím W/A/S/D/Space)</div>
        <div style="font-size: 0.8rem; font-weight: 700; color: var(--accent);" id="lblDriveStatus">DỪNG (0, 0)</div>
      </div>

      <div class="dpad-container">
        <div class="dpad-row">
          <button class="btn-dpad" id="btnFwd" onmousedown="startDrive('fwd')" onmouseup="stopDrive()" ontouchstart="startDrive('fwd')" ontouchend="stopDrive()">▲<span style="font-size:0.65rem">TIẾN (W)</span></button>
        </div>
        <div class="dpad-row">
          <button class="btn-dpad" id="btnLeft" onmousedown="startDrive('left')" onmouseup="stopDrive()" ontouchstart="startDrive('left')" ontouchend="stopDrive()">◀<span style="font-size:0.65rem">TRÁI (A)</span></button>
          <button class="btn-dpad btn-dpad-stop" onclick="stopDrive()">■<span style="font-size:0.65rem">DỪNG</span></button>
          <button class="btn-dpad" id="btnRight" onmousedown="startDrive('right')" onmouseup="stopDrive()" ontouchstart="startDrive('right')" ontouchend="stopDrive()">▶<span style="font-size:0.65rem">PHẢI (D)</span></button>
        </div>
        <div class="dpad-row">
          <button class="btn-dpad" id="btnBack" onmousedown="startDrive('back')" onmouseup="stopDrive()" ontouchstart="startDrive('back')" ontouchend="stopDrive()">▼<span style="font-size:0.65rem">LÙI (S)</span></button>
        </div>
      </div>

      <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin-top: 10px;">
        <div class="field">
          <label>Tốc độ đặt (Drive): <span id="lblDriveSpd">150</span></label>
          <input type="range" id="rngDriveSpd" min="50" max="400" step="10" value="150" oninput="document.getElementById('lblDriveSpd').innerText=this.value">
        </div>
        <div class="field">
          <label>Độ nhạy rẽ (Steer): <span id="lblSteerSpd">200</span></label>
          <input type="range" id="rngSteerSpd" min="50" max="400" step="10" value="200" oninput="document.getElementById('lblSteerSpd').innerText=this.value">
        </div>
      </div>
    </div>

    <!-- Form Chỉnh Thông Số PID Thời Gian Thực -->
    <div class="card">
      <div class="card-title">Tinh Chỉnh Thông Số PID Thời Gian Thực</div>
      <div style="font-size: 0.8rem; color: var(--muted);">Thay đổi giá trị bên dưới và bấm "Lưu & Cập Nhật", thông số sẽ truyền thẳng về STM32 mà không cần dừng xe!</div>

      <div class="form-grid">
        <!-- Kp Góc -->
        <div class="field">
          <label>Kp Góc (Độ cứng vững) <span id="lblKp">18.0</span></label>
          <input type="number" id="inpKp" step="0.5" min="0" max="100" value="18.0">
          <input type="range" id="rngKp" min="0" max="60" step="0.5" value="18.0" oninput="syncVal('Kp', this.value)">
        </div>

        <!-- Kd Góc -->
        <div class="field">
          <label>Kd Góc (Giảm chấn góc) <span id="lblKd">0.85</span></label>
          <input type="number" id="inpKd" step="0.05" min="0" max="10" value="0.85">
          <input type="range" id="rngKd" min="0" max="5" step="0.05" value="0.85" oninput="syncVal('Kd', this.value)">
        </div>

        <!-- Ki Góc -->
        <div class="field">
          <label>Ki Góc (Tích phân) <span id="lblKi">0.00</span></label>
          <input type="number" id="inpKi" step="0.05" min="0" max="10" value="0.0">
          <input type="range" id="rngKi" min="0" max="5" step="0.05" value="0.0" oninput="syncVal('Ki', this.value)">
        </div>

        <!-- Trim / Điểm cân bằng cơ học -->
        <div class="field">
          <label>Trim Góc Đặt (Độ nghiêng tự nhiên) <span id="lblTrim">0.0</span>°</label>
          <input type="number" id="inpTrim" step="0.1" min="-10" max="10" value="0.0">
          <input type="range" id="rngTrim" min="-5" max="5" step="0.1" value="0.0" oninput="syncVal('Trim', this.value)">
        </div>
      </div>

      <button class="btn-submit" onclick="submitPid()">💾 LƯU & CẬP NHẬT PID SANG STM32</button>
    </div>
  </div>

  <div id="toast" class="toast">Thông báo</div>

  <script>
    function showToast(msg) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      t.style.display = 'block';
      setTimeout(() => { t.style.display = 'none'; }, 2500);
    }

    function syncVal(name, val) {
      document.getElementById('inp' + name).value = val;
      document.getElementById('lbl' + name).innerText = val;
    }

    document.querySelectorAll('input[type=number]').forEach(inp => {
      inp.addEventListener('input', e => {
        const id = e.target.id.replace('inp', '');
        document.getElementById('rng' + id).value = e.target.value;
        document.getElementById('lbl' + id).innerText = e.target.value;
      });
    });

    function submitPid() {
      const kp = document.getElementById('inpKp').value;
      const kd = document.getElementById('inpKd').value;
      const ki = document.getElementById('inpKi').value;
      const trim = document.getElementById('inpTrim').value;

      fetch(`/api/set_pid?kp=${kp}&kd=${kd}&ki=${ki}&target=${trim}`, { method: 'POST' })
        .then(r => r.text())
        .then(res => { showToast("Đã gửi thông số PID sang STM32!"); })
        .catch(err => { showToast("Lỗi gửi PID: " + err); });
    }

    function sendCmd(act) {
      fetch(`/api/cmd?action=${act}`, { method: 'POST' })
        .then(r => r.text())
        .then(res => { showToast("Đã gửi lệnh: " + act.toUpperCase()); })
        .catch(err => { showToast("Lỗi gửi lệnh: " + err); });
    }

    /* Logic điều khiển di chuyển xe qua Web & Bàn phím */
    let driveInterval = null;

    function sendDriveCmd(spd, str) {
      const lbl = document.getElementById('lblDriveStatus');
      if (lbl) {
        lbl.innerText = (spd === 0 && str === 0) ? "DỪNG (0, 0)" : `CHẠY (${spd}, ${str})`;
      }
      fetch(`/api/drive?speed=${spd}&steer=${str}`, { method: 'POST' }).catch(()=>{});
    }

    function startDrive(dir) {
      const spdVal = parseInt(document.getElementById('rngDriveSpd').value);
      const strVal = parseInt(document.getElementById('rngSteerSpd').value);
      let s = 0, st = 0;
      if (dir === 'fwd') s = spdVal;
      else if (dir === 'back') s = -spdVal;
      else if (dir === 'left') st = -strVal;
      else if (dir === 'right') st = strVal;

      sendDriveCmd(s, st);
      if (driveInterval) clearInterval(driveInterval);
      driveInterval = setInterval(() => { sendDriveCmd(s, st); }, 120);
    }

    function stopDrive() {
      if (driveInterval) { clearInterval(driveInterval); driveInterval = null; }
      sendDriveCmd(0, 0);
    }

    /* Bắt sự kiện bàn phím máy tính W / A / S / D / Space / X / Z */
    const keysPressed = {};
    window.addEventListener('keydown', (e) => {
      const k = e.key.toLowerCase();
      if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' ', 'x', 'z'].includes(k)) {
        if (!keysPressed[k]) {
          keysPressed[k] = true;
          if (k === 'w' || k === 'arrowup') startDrive('fwd');
          else if (k === 's' || k === 'arrowdown') startDrive('back');
          else if (k === 'a' || k === 'arrowleft') startDrive('left');
          else if (k === 'd' || k === 'arrowright') startDrive('right');
          else if (k === ' ') stopDrive();
          else if (k === 'x') sendCmd('arm');
          else if (k === 'z') sendCmd('disarm');
        }
      }
    });

    window.addEventListener('keyup', (e) => {
      const k = e.key.toLowerCase();
      if (keysPressed[k]) {
        keysPressed[k] = false;
        if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) {
          stopDrive();
        }
      }
    });

    // Chart Canvas setup
    const canvas = document.getElementById('chartCanvas');
    const ctx = canvas.getContext('2d');
    const maxPoints = 120;
    const historyPitch = new Array(maxPoints).fill(0);
    const historyTarget = new Array(maxPoints).fill(0);

    function resizeCanvas() {
      canvas.width = canvas.parentElement.clientWidth;
      canvas.height = canvas.parentElement.clientHeight;
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    function drawChart() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const w = canvas.width;
      const h = canvas.height;
      const midY = h / 2;

      // Draw zero line
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, midY);
      ctx.lineTo(w, midY);
      ctx.stroke();

      // Helper to map angle [-30..+30] to Y [h..0]
      const scale = (midY - 10) / 25.0; // 25 degrees full scale

      // Draw Target line (Blue)
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const y = midY - (historyTarget[i] * scale);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Draw Pitch line (Green)
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const y = midY - (historyPitch[i] * scale);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    const stateNames = ["BOOT", "CALIB", "DISARMED", "ARMED", "FAULT"];
    const stateBadges = ["badge-boot", "badge-calib", "badge-disarmed", "badge-armed", "badge-fault"];

    // Polling Telemetry data every 80 ms
    setInterval(() => {
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('valPitch').innerHTML = d.pitch.toFixed(2) + '<span class="val-unit">°</span>';
          document.getElementById('valTarget').innerText = d.target.toFixed(2);
          document.getElementById('valRate').innerHTML = d.rate.toFixed(1) + '<span class="val-unit">°/s</span>';
          document.getElementById('valRpmL').innerText = Math.round(d.rpm_l);
          document.getElementById('valRpmR').innerText = Math.round(d.rpm_r);
          document.getElementById('valDistFront').innerHTML = (d.dist_f < 400 ? d.dist_f : '--') + '<span class="val-unit">cm</span>';
          document.getElementById('valTof').innerText = (d.tof_mm < 8000 ? d.tof_mm : '--');

          const b = document.getElementById('badgeState');
          const st = d.state || 0;
          b.innerText = stateNames[st] || "UNKNOWN";
          b.className = "badge " + (stateBadges[st] || "badge-boot");

          // Cập nhật các trạng thái chẩn đoán phần cứng
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

          const bNrf = document.getElementById('badgeNrf');
          if (d.stm_link && d.nrf_ok) {
            bNrf.innerText = "SẴN SÀNG (SPI2 OK)";
            bNrf.style.background = "var(--green)";
            bNrf.style.color = "#fff";
          } else if (d.stm_link) {
            bNrf.innerText = "LỖI NRF XE";
            bNrf.style.background = "var(--red)";
            bNrf.style.color = "#fff";
          } else {
            bNrf.innerText = "--";
            bNrf.style.background = "var(--muted)";
          }

          const bUpr = document.getElementById('badgeUpright');
          if (d.upright) {
            bUpr.innerText = "SẴN SÀNG ARM (±3.5°)";
            bUpr.style.background = "var(--green)";
            bUpr.style.color = "#fff";
          } else {
            bUpr.innerText = "ĐANG NGHIÊNG";
            bUpr.style.background = "var(--yellow)";
            bUpr.style.color = "#000";
          }

          // Update chart
          historyPitch.shift();
          historyPitch.push(d.pitch);
          historyTarget.shift();
          historyTarget.push(d.target);
          drawChart();
        })
        .catch(() => {});
    }, 80);
  </script>
</body>
</html>
)rawliteral";

#endif /* WEB_INDEX_H */
