#ifndef WEB_INDEX_H
#define WEB_INDEX_H

const char PAGE_MAIN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot Cân Bằng 2 Bánh - PID Station & Monitor</title>
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
    .container { width: 100%; max-width: 960px; display: flex; flex-direction: column; gap: 14px; }
    header {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px;
      padding: 14px 20px; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px;
    }
    h1 { font-size: 1.25rem; font-weight: 700; color: var(--accent); }
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

    .grid-4 { display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; }
    .grid-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 12px; }
    .card {
      background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 14px;
    }
    .card-title { font-size: 0.8rem; text-transform: uppercase; color: var(--muted); margin-bottom: 6px; font-weight: 600; display: flex; justify-content: space-between; }
    .val-large { font-size: 1.85rem; font-weight: 700; }
    .val-unit { font-size: 0.9rem; color: var(--muted); font-weight: 400; margin-left: 4px; }

    /* Canvas chart */
    .chart-container { position: relative; width: 100%; height: 210px; background: #0b1120; border-radius: 8px; overflow: hidden; margin-top: 8px; }
    canvas { width: 100%; height: 100%; display: block; }

    /* Buttons */
    .btn-group { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 8px; }
    button {
      flex: 1; min-width: 130px; padding: 12px 16px; border: none; border-radius: 8px;
      font-weight: 700; font-size: 0.95rem; cursor: pointer; transition: 0.2s;
    }
    .btn-arm { background: var(--green); color: #fff; }
    .btn-arm:hover { background: #16a34a; }
    .btn-disarm { background: var(--red); color: #fff; }
    .btn-disarm:hover { background: #dc2626; }
    .btn-submit { background: var(--accent); color: #0f172a; }
    .btn-submit:hover { background: var(--accent-hover); }
    .btn-secondary { background: #334155; color: #fff; }
    .btn-secondary:hover { background: #475569; }

    .form-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; margin-top: 10px; }
    .field { display: flex; flex-direction: column; gap: 4px; background: #0b1120; padding: 10px; border-radius: 8px; border: 1px solid #1e293b; }
    label { font-size: 0.85rem; color: var(--muted); font-weight: 600; display: flex; justify-content: space-between; }
    .field-row { display: flex; gap: 8px; align-items: center; margin-top: 4px; }
    input[type=number] {
      width: 90px; background: #1e293b; border: 1px solid var(--border); border-radius: 6px;
      color: #fff; padding: 6px 8px; font-size: 1rem; font-family: monospace; text-align: right;
    }
    input[type=range] { flex: 1; accent-color: var(--accent); cursor: pointer; }

    .toast {
      position: fixed; bottom: 20px; right: 20px; background: var(--card); border: 1px solid var(--accent);
      color: #fff; padding: 12px 20px; border-radius: 8px; font-size: 0.9rem; display: none;
      box-shadow: 0 4px 12px rgba(0,0,0,0.4); z-index: 999;
    }
  </style>
</head>
<body>
  <div class="container">
    <!-- Header -->
    <header>
      <div>
        <h1>STM32F405 ↔ ESP32 Web Dashboard</h1>
        <div style="font-size: 0.8rem; color: var(--muted);">Trạm Đo Góc Nghiêng & Tinh Chỉnh Thông Số PID Thời Gian Thực</div>
      </div>
      <div style="display: flex; gap: 8px; align-items: center;">
        <span id="badgeState" class="badge badge-boot">BOOT</span>
        <span id="badgeBat" style="font-size: 0.85rem; color: var(--green); font-weight: 700;">14.8V (4S)</span>
      </div>
    </header>

    <!-- Thanh Chẩn Đoán Phần Cứng Real-Time -->
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

    <!-- Bảng thông số PID HIỆN TẠI đang nạp trên STM32 -->
    <div class="card" style="border-left: 4px solid var(--purple);">
      <div class="card-title" style="color:var(--purple);">⚡ THÔNG SỐ PID ĐANG CHẠY TRỰC TIẾP TRÊN XE</div>
      <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 10px; margin-top: 8px;">
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Kp Góc</div>
          <div style="font-size:1.15rem; font-weight:700; color:#38bdf8;" id="liveKp">18.00</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Kd Góc</div>
          <div style="font-size:1.15rem; font-weight:700; color:#38bdf8;" id="liveKd">1.36</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Ki Góc</div>
          <div style="font-size:1.15rem; font-weight:700; color:#38bdf8;" id="liveKi">0.00</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Kp Vận Tốc</div>
          <div style="font-size:1.15rem; font-weight:700; color:#22c55e;" id="liveKpS">0.35</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Ki Vận Tốc</div>
          <div style="font-size:1.15rem; font-weight:700; color:#22c55e;" id="liveKiS">0.02</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Kp Lái</div>
          <div style="font-size:1.15rem; font-weight:700; color:#eab308;" id="liveSteer">1.20</div>
        </div>
        <div style="background:#0b1120; padding:8px 10px; border-radius:6px; text-align:center;">
          <div style="font-size:0.75rem; color:var(--muted);">Điểm 0 Trim</div>
          <div style="font-size:1.15rem; font-weight:700; color:#f97316;" id="liveTrim">0.45°</div>
        </div>
      </div>
    </div>

    <!-- Nút điều khiển trạng thái an toàn -->
    <div class="card">
      <div class="card-title">Điều Khiển Trạng Thái Xe</div>
      <div class="btn-group">
        <button class="btn-arm" onclick="sendCmd('arm')">⚡ BẬT CÂN BẰNG (ARM)</button>
        <button class="btn-disarm" onclick="sendCmd('disarm')">🛑 TẮT ĐỘNG CƠ (DISARM / CẮT KHẨN CẤP)</button>
      </div>
    </div>

    <!-- Form Chỉnh Thông Số PID Thời Gian Thực -->
    <div class="card">
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <div class="card-title">🎛️ BẢNG ĐIỀU KHIỂN & TINH CHỈNH PID THỜI GIAN THỰC</div>
        <div style="display:flex; gap:6px;">
          <button class="btn-secondary" style="padding:4px 8px; font-size:0.75rem; min-width:auto;" onclick="applyPreset('soft')">Êm ái</button>
          <button class="btn-secondary" style="padding:4px 8px; font-size:0.75rem; min-width:auto;" onclick="applyPreset('stiff')">Cứng cáp</button>
        </div>
      </div>
      <div style="font-size: 0.8rem; color: var(--muted); margin-bottom: 8px;">Thay đổi thông số và bấm <b>"Lưu & Cập Nhật"</b>, giá trị sẽ truyền thẳng về STM32 cập nhật ngay lập tức mà không cần dừng xe hay nạp lại code!</div>

      <div class="form-grid">
        <!-- Kp Góc -->
        <div class="field">
          <label>Kp Góc (Độ cứng vững thân xe) <span id="lblKp" style="color:var(--accent);">30</span></label>
          <div class="field-row">
            <input type="range" id="rngKp" min="0" max="60" step="0.5" value="30" oninput="syncVal('Kp', this.value)">
            <input type="number" id="inpKp" step="0.5" min="0" max="100" value="30">
          </div>
        </div>

        <!-- Kd Góc -->
        <div class="field">
          <label>Kd Góc (Giảm chấn góc - Triệt tiêu rung) <span id="lblKd" style="color:var(--accent);">1.9</span></label>
          <div class="field-row">
            <input type="range" id="rngKd" min="0" max="5.0" step="0.05" value="1.9" oninput="syncVal('Kd', this.value)">
            <input type="number" id="inpKd" step="0.05" min="0" max="10" value="1.9">
          </div>
        </div>

        <!-- Ki Góc -->
        <div class="field">
          <label>Ki Góc (Tích phân góc) <span id="lblKi" style="color:var(--accent);">0.000001</span></label>
          <div class="field-row">
            <input type="range" id="rngKi" min="0" max="1.0" step="0.000001" value="0.000001" oninput="syncVal('Ki', this.value)">
            <input type="number" id="inpKi" step="0.000001" min="0" max="10" value="0.000001">
          </div>
        </div>

        <!-- Kp Vận Tốc -->
        <div class="field">
          <label>Kp Vận Tốc (Độ nhạy vòng ngoài) <span id="lblKpS" style="color:var(--green);">0.35</span></label>
          <div class="field-row">
            <input type="range" id="rngKpS" min="0" max="2.0" step="0.05" value="0.35" oninput="syncVal('KpS', this.value)">
            <input type="number" id="inpKpS" step="0.05" min="0" max="5.0" value="0.35">
          </div>
        </div>

        <!-- Ki Vận Tốc -->
        <div class="field">
          <label>Ki Vận Tốc (Khử trôi vị trí) <span id="lblKiS" style="color:var(--green);">0.55</span></label>
          <div class="field-row">
            <input type="range" id="rngKiS" min="0" max="2.0" step="0.01" value="0.55" oninput="syncVal('KiS', this.value)">
            <input type="number" id="inpKiS" step="0.01" min="0" max="5.0" value="0.55">
          </div>
        </div>

        <!-- Kp Lái (Steer) -->
        <div class="field">
          <label>Kp Lái (Độ nhạy rẽ khi bẻ lái) <span id="lblSteer" style="color:var(--yellow);">1.2</span></label>
          <div class="field-row">
            <input type="range" id="rngSteer" min="0" max="3.0" step="0.1" value="1.2" oninput="syncVal('Steer', this.value)">
            <input type="number" id="inpSteer" step="0.1" min="0" max="5.0" value="1.2">
          </div>
        </div>

        <!-- Trim Điểm Cân Bằng Cơ Học -->
        <div class="field" style="grid-column: 1 / -1;">
          <label>Trim Góc Đặt (Điểm 0 trọng tâm cân bằng tự nhiên) <span id="lblTrim" style="color:#f97316;">-0.45</span></label>
          <div class="field-row">
            <input type="range" id="rngTrim" min="-5.0" max="5.0" step="0.05" value="-0.45" oninput="syncVal('Trim', this.value)">
            <input type="number" id="inpTrim" step="0.05" min="-10" max="10" value="-0.45">
          </div>
        </div>
      </div>

      <div class="btn-group" style="margin-top:12px;">
        <button class="btn-submit" onclick="submitPid()">💾 LƯU & CẬP NHẬT PID SANG STM32</button>
        <button class="btn-secondary" onclick="loadFromVehicle()">🔄 NẠP LẠI PID TỪ XE</button>
      </div>
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
      const r = document.getElementById('rng' + name);
      if (r) r.value = val;
      document.getElementById('inp' + name).value = val;
      document.getElementById('lbl' + name).innerText = val;
    }

    document.querySelectorAll('input[type=number]').forEach(inp => {
      inp.addEventListener('input', e => {
        const id = e.target.id.replace('inp', '');
        const r = document.getElementById('rng' + id);
        if (r) r.value = e.target.value;
        const l = document.getElementById('lbl' + id);
        if (l) l.innerText = e.target.value;
      });
    });

    function applyPreset(type) {
      if (type === 'soft') {
        syncVal('Kp', 18.0);
        syncVal('Kd', 1.36);
        syncVal('Ki', 0.0);
        syncVal('KpS', 0.35);
        syncVal('KiS', 0.02);
        syncVal('Steer', 1.20);
        syncVal('Trim', 0.45);
        showToast("Đã chọn preset: ÊM ÁI (Khuyên dùng)");
      } else if (type === 'stiff') {
        syncVal('Kp', 30.0);
        syncVal('Kd', 1.90);
        syncVal('Ki', 0.000001);
        syncVal('KpS', 0.35);
        syncVal('KiS', 0.55);
        syncVal('Steer', 1.20);
        syncVal('Trim', -0.45);
        showToast("Đã chọn preset: CỨNG CÁP (Mặc định chuẩn)");
      }
    }

    let latestTelemetry = null;

    function loadFromVehicle() {
      if (latestTelemetry) {
        if (typeof latestTelemetry.kp === 'number') syncVal('Kp', latestTelemetry.kp);
        if (typeof latestTelemetry.kd === 'number') syncVal('Kd', latestTelemetry.kd);
        if (typeof latestTelemetry.ki === 'number') syncVal('Ki', latestTelemetry.ki);
        if (typeof latestTelemetry.kp_s === 'number') syncVal('KpS', latestTelemetry.kp_s);
        if (typeof latestTelemetry.ki_s === 'number') syncVal('KiS', latestTelemetry.ki_s);
        if (typeof latestTelemetry.steer === 'number') syncVal('Steer', latestTelemetry.steer);
        if (typeof latestTelemetry.trim === 'number') syncVal('Trim', latestTelemetry.trim);
        showToast("Đã nạp thông số hiện tại từ xe vào bảng chỉnh!");
      }
    }

    function submitPid() {
      const kp = document.getElementById('inpKp').value;
      const kd = document.getElementById('inpKd').value;
      const ki = document.getElementById('inpKi').value;
      const kp_s = document.getElementById('inpKpS').value;
      const ki_s = document.getElementById('inpKiS').value;
      const steer = document.getElementById('inpSteer').value;
      const trim = document.getElementById('inpTrim').value;

      fetch(`/api/set_pid?kp=${kp}&kd=${kd}&ki=${ki}&kp_s=${kp_s}&ki_s=${ki_s}&steer=${steer}&trim=${trim}`, { method: 'POST' })
        .then(r => r.text())
        .then(res => { showToast("Đã gửi toàn bộ thông số PID sang STM32!"); })
        .catch(err => { showToast("Lỗi gửi PID: " + err); });
    }

    function sendCmd(act) {
      fetch(`/api/cmd?action=${act}`, { method: 'POST' })
        .then(r => r.text())
        .then(res => { showToast("Đã gửi lệnh: " + act.toUpperCase()); })
        .catch(err => { showToast("Lỗi gửi lệnh: " + err); });
    }

    // Chart Canvas setup
    const canvas = document.getElementById('chartCanvas');
    const ctx = canvas.getContext('2d');
    const maxPoints = 120;
    const historyPitch = new Array(maxPoints).fill(0);
    const historyTarget = new Array(maxPoints).fill(0);

    function drawChart() {
      const container = canvas.parentElement;
      const targetW = (container && container.clientWidth > 50) ? container.clientWidth : 600;
      const targetH = (container && container.clientHeight > 50) ? container.clientHeight : 210;

      if (canvas.width !== targetW || canvas.height !== targetH) {
        canvas.width = targetW;
        canvas.height = targetH;
      }

      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const w = canvas.width;
      const h = canvas.height;
      const midY = h / 2;

      // Draw grid lines
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      const scale = (midY - 10) / 20.0; // ±20 degrees full scale

      [-10, 10].forEach(deg => {
        const y = midY - (deg * scale);
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      });

      // Zero line
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, midY);
      ctx.lineTo(w, midY);
      ctx.stroke();

      // Draw Target line (Blue)
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const val = Number.isFinite(historyTarget[i]) ? historyTarget[i] : 0;
        const y = midY - (val * scale);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Draw Pitch line (Green)
      ctx.strokeStyle = '#22c55e';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      for (let i = 0; i < maxPoints; i++) {
        const x = (i / (maxPoints - 1)) * w;
        const val = Number.isFinite(historyPitch[i]) ? historyPitch[i] : 0;
        const y = midY - (val * scale);
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
          if (!d) return;
          latestTelemetry = d;

          const pitch = (typeof d.pitch === 'number' && Number.isFinite(d.pitch)) ? d.pitch : 0.0;
          const target = (typeof d.target === 'number' && Number.isFinite(d.target)) ? d.target : 0.0;
          const rate = (typeof d.rate === 'number' && Number.isFinite(d.rate)) ? d.rate : 0.0;
          const rpmL = (typeof d.rpm_l === 'number' && Number.isFinite(d.rpm_l)) ? d.rpm_l : 0.0;
          const rpmR = (typeof d.rpm_r === 'number' && Number.isFinite(d.rpm_r)) ? d.rpm_r : 0.0;
          const distF = (typeof d.dist_f === 'number') ? d.dist_f : 999;
          const tof = (typeof d.tof_mm === 'number') ? d.tof_mm : 8190;

          // Cập nhật thẻ đo góc với màu sắc cảnh báo trực quan
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

          // Cập nhật thông số PID hiện tại từ STM32
          if (typeof d.kp === 'number') document.getElementById('liveKp').innerText = d.kp.toFixed(2);
          if (typeof d.kd === 'number') document.getElementById('liveKd').innerText = d.kd.toFixed(2);
          if (typeof d.ki === 'number') {
            document.getElementById('liveKi').innerText = (d.ki > 0 && d.ki < 0.001) ? d.ki.toFixed(6) : d.ki.toFixed(2);
          }
          if (typeof d.kp_s === 'number') document.getElementById('liveKpS').innerText = d.kp_s.toFixed(2);
          if (typeof d.ki_s === 'number') document.getElementById('liveKiS').innerText = d.ki_s.toFixed(2);
          if (typeof d.steer === 'number') document.getElementById('liveSteer').innerText = d.steer.toFixed(2);
          if (typeof d.trim === 'number') document.getElementById('liveTrim').innerText = d.trim.toFixed(2) + '°';

          const b = document.getElementById('badgeState');
          const st = d.state || 0;
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

          // Trạng thái Sóng NRF24: phân biệt rõ giữa "chờ tay cầm" và "lỗi phần cứng"
          const bNrf = document.getElementById('badgeNrf');
          if (d.stm_link && d.nrf_link) {
            bNrf.innerText = "TAY CẦM ĐÃ NỐI 🟢";
            bNrf.style.background = "var(--green)";
            bNrf.style.color = "#fff";
          } else if (d.stm_link && d.nrf_hw) {
            bNrf.innerText = "CHỜ TAY CẦM (NRF SẴN SÀNG) 🟡";
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

          // Update chart
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
