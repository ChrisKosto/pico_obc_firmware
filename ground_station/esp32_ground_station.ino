#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

const char *ssid = "CUBESAT-GROUND-STATION";
const char *password = "12345678";

WebServer server(80);
WiFiUDP udp;

const unsigned int UDP_PORT_RX = 5000;
const unsigned int UDP_PORT_TX = 5001;
IPAddress pico_ip(192, 168, 4, 2);

#pragma pack(push, 1)
typedef struct {
  uint16_t packet_id;
  uint16_t packet_seq_ctrl;
  uint16_t packet_data_len;
} ccsds_primary_header_t;

typedef struct {
  ccsds_primary_header_t header;
  uint32_t uptime_ms;
  int16_t  mpu_temp_centi_c;
  int16_t  mcu_temp_centi_c;
  int16_t  roll_centi_deg;
  int16_t  pitch_centi_deg;
  int16_t  yaw_centi_deg;
  uint8_t  flight_mode;
  uint8_t  boot_cause;
  uint8_t  fdir_alarm;
  uint16_t crc16;
} ccsds_telemetry_packet_t;
#pragma pack(pop)

float sat_mpu_temp = 0.0;
float sat_mcu_temp = 0.0;
float sat_roll = 0.0;
float sat_pitch = 0.0;
float sat_yaw = 0.0;
uint32_t sat_uptime = 0;
String sat_mode = "DISCONNECTED";
String sat_boot_cause = "UNKNOWN";
String sat_fdir_msg = "NOMINAL";
uint16_t last_pkt_id = 0;
uint32_t last_packet_seq = 0;
uint16_t last_data_len = 0;
bool crc_valid = false;
String last_raw_hex = "--";
uint32_t ack_n = 0;                // number of telecommand reports received
String last_ack = "--";            // last telecommand report, human readable

unsigned long last_rx_ms = 0;      // millis() of the last packet from the OBC
unsigned long ping_sent_time = 0;
int last_ping_rtt = -1;

uint16_t calc_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = crc << 1;
    }
  }
  return crc;
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CUBESAT GROUND SEGMENT CONTROL</title>
  <style>
    :root {
      --bg: #090d16;
      --panel: #111827;
      --panel-head: #1a2234;
      --border: #26334d;
      --cyan: #00f0ff;
      --blue: #3b82f6;
      --orange: #f59e0b;
      --green: #10b981;
      --red: #ef4444;
      --text: #e2e8f0;
      --muted: #64748b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: "Consolas", "Courier New", monospace; }
    body { background: var(--bg); color: var(--text); padding: 12px; }

    .header { display: flex; justify-content: space-between; align-items: center; background: var(--panel); border: 1px solid var(--border); padding: 10px 16px; border-radius: 4px; margin-bottom: 12px; }
    .header-title { font-size: 1.05rem; font-weight: bold; letter-spacing: 1px; color: var(--cyan); display: flex; align-items: center; gap: 8px; }
    .header-meta { display: flex; gap: 12px; align-items: center; }
    .tag { font-size: 0.75rem; padding: 3px 8px; border-radius: 3px; border: 1px solid var(--border); background: #0c1220; }
    .tag-green { border-color: var(--green); color: var(--green); }
    .tag-red { border-color: var(--red); color: var(--red); }

    .row { display: grid; grid-template-columns: 1fr 1fr 1.1fr; gap: 12px; margin-bottom: 12px; }
    .row-charts { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 12px; }

    .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 4px; overflow: hidden; display: flex; flex-direction: column; }
    .panel-hdr { background: var(--panel-head); border-bottom: 1px solid var(--border); padding: 6px 12px; font-size: 0.75rem; font-weight: bold; color: var(--muted); letter-spacing: 0.5px; text-transform: uppercase; display: flex; justify-content: space-between; }
    .panel-body { padding: 12px; flex: 1; }

    .t-row { display: flex; justify-content: space-between; align-items: center; padding: 5px 0; border-bottom: 1px dashed #1e293b; font-size: 0.8rem; }
    .t-key { color: var(--muted); }
    .t-val { font-weight: bold; color: var(--cyan); }

    .cmd-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button.btn { background: #1e293b; color: var(--text); border: 1px solid var(--border); padding: 9px; cursor: pointer; border-radius: 3px; font-size: 0.75rem; font-weight: bold; text-align: center; }
    button.btn:hover { border-color: var(--cyan); color: var(--cyan); }
    button.btn-danger { border-color: rgba(239,68,68,0.5); color: #fca5a5; }
    button.btn-danger:hover { background: var(--red); color: #fff; border-color: var(--red); }

    .chart-box { width: 100%; height: 160px; background: #070a12; border: 1px solid var(--border); border-radius: 3px; position: relative; }
    svg { width: 100%; height: 100%; display: block; }
    .legend { display: flex; gap: 14px; font-size: 0.7rem; margin-top: 6px; justify-content: flex-end; }
    .leg-item { display: flex; align-items: center; gap: 4px; }
    .dot { width: 8px; height: 8px; border-radius: 2px; }

    .hex-box { background: #070a12; border: 1px solid var(--border); padding: 8px; font-size: 0.7rem; color: #38bdf8; word-break: break-all; border-radius: 3px; margin-top: 8px; }
    .log-box { background: #070a12; border: 1px solid var(--border); padding: 8px; font-size: 0.72rem; color: #a7f3d0; height: 110px; overflow-y: auto; border-radius: 3px; display: flex; flex-direction: column-reverse; }
    .log-line { margin-bottom: 2px; }
  </style>
</head>
<body>

  <div class="header">
    <div class="header-title">
      <span>🛰️ SCOS-2000-INSPIRED GROUND SEGMENT</span>
      <span class="tag" style="color:var(--muted)">MISSION: CUBESAT-1U</span>
    </div>
    <div class="header-meta">
      <span class="tag" id="utc-clock">UTC 00:00:00</span>
      <span class="tag tag-red" id="link-badge">AIR LINK: WAITING</span>
      <span class="tag" id="rtt-badge">RTT: -- ms</span>
    </div>
  </div>

  <div class="row">

    <div class="panel">
      <div class="panel-hdr">
        <span>ATTITUDE DETERMINATION (MPU6050)</span>
        <span id="rate-badge">0.0 Hz</span>
      </div>
      <div class="panel-body">
        <canvas id="cubeCanvas" width="300" height="190" style="width:100%; height:190px; background:#070a12; border:1px solid var(--border); border-radius:3px;"></canvas>
        <div style="display:flex; justify-content:space-around; margin-top:10px; font-size:0.8rem;">
          <div>R: <span class="t-val" id="r-val">0.0°</span></div>
          <div>P: <span class="t-val" id="p-val">0.0°</span></div>
          <div>Y: <span class="t-val" id="y-val">0.0°</span></div>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-hdr">
        <span>HOUSEKEEPING TELEMETRY (HK-TM)</span>
        <span id="crc-tag" class="tag tag-green">CRC: --</span>
      </div>
      <div class="panel-body">
        <div class="t-row"><span class="t-key">FSM State:</span><span class="t-val" id="val-mode">--</span></div>
        <div class="t-row"><span class="t-key">FDIR Status:</span><span class="t-val" id="val-fdir">NOMINAL</span></div>
        <div class="t-row"><span class="t-key">Boot Source:</span><span class="t-val" id="val-boot">--</span></div>
        <div class="t-row"><span class="t-key">MCU Core Temp:</span><span class="t-val" id="val-pico-temp">--.- °C</span></div>
        <div class="t-row"><span class="t-key">IMU Sensor Temp:</span><span class="t-val" id="val-mpu-temp">--.- °C</span></div>
        <div class="t-row"><span class="t-key">Packet Seq Ctrl:</span><span class="t-val" id="val-seq">#0000</span></div>
        <div class="t-row"><span class="t-key">OBC Uptime:</span><span class="t-val" id="val-uptime">00:00:00</span></div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-hdr">
        <span>TELECOMMAND & CCSDS PACKET INSPECTOR</span>
      </div>
      <div class="panel-body">
        <div class="cmd-grid">
          <button class="btn" onclick="sendTC('10','00')">TC: PUS17 PING</button>
          <button class="btn" onclick="sendTC('20','00')">TC: SET NOMINAL</button>
          <button class="btn" onclick="sendTC('20','01')">TC: SAFE MODE</button>
          <button class="btn btn-danger" onclick="confirmReboot()">TC: FORCE WDT RST</button>
        </div>

        <div style="font-size:0.7rem; color:var(--muted); margin-top:12px; margin-bottom:4px;">CCSDS HEADER PARSER</div>
        <div style="display:grid; grid-template-columns:1fr 1fr; gap:6px; font-size:0.72rem; background:#0c1220; padding:6px; border:1px solid var(--border); border-radius:3px;">
          <div>APID: <span style="color:var(--cyan)" id="ccsds-apid">--</span></div>
          <div>LEN: <span style="color:var(--cyan)" id="ccsds-len">-- bytes</span></div>
          <div>SEQ: <span style="color:var(--cyan)" id="ccsds-seq">--</span></div>
          <div>CRC16: <span style="color:var(--green)" id="ccsds-crc">--</span></div>
        </div>

        <div class="hex-box" id="hex-view">Waiting for frames...</div>
      </div>
    </div>

  </div>

  <div class="row-charts">
    <div class="panel">
      <div class="panel-hdr">
        <span>ATTITUDE TELEMETRY HISTORY [-180° .. +180°]</span>
        <div class="legend">
          <div class="leg-item"><div class="dot" style="background:var(--cyan)"></div>Roll</div>
          <div class="leg-item"><div class="dot" style="background:var(--blue)"></div>Pitch</div>
          <div class="leg-item"><div class="dot" style="background:var(--orange)"></div>Yaw</div>
        </div>
      </div>
      <div class="panel-body">
        <div class="chart-box">
          <svg id="svgAtt" viewBox="0 0 500 160" preserveAspectRatio="none"></svg>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-hdr">
        <span>THERMAL SUBSYSTEM PROFILE [+20°C .. +80°C]</span>
        <div class="legend">
          <div class="leg-item"><div class="dot" style="background:var(--red)"></div>MCU Core</div>
          <div class="leg-item"><div class="dot" style="background:var(--orange)"></div>IMU Sensor</div>
        </div>
      </div>
      <div class="panel-body">
        <div class="chart-box">
          <svg id="svgTemp" viewBox="0 0 500 160" preserveAspectRatio="none"></svg>
        </div>
      </div>
    </div>
  </div>

  <div class="panel">
    <div class="panel-hdr">
      <span>FLIGHT LOG & EVENT RECORDER</span>
      <button class="btn" style="padding:2px 8px; font-size:0.68rem;" onclick="exportCSV()">EXPORT LOGS (.CSV)</button>
    </div>
    <div class="panel-body">
      <div class="log-box" id="event-log">
        <div class="log-line">[SYSTEM] MOC Online. Awaiting Space Packets...</div>
      </div>
    </div>
  </div>

  <script>
    setInterval(() => {
      document.getElementById('utc-clock').innerText = 'UTC ' + new Date().toISOString().substring(11, 19);
    }, 1000);

    // ---------- 3D wireframe ----------
    const canvas = document.getElementById('cubeCanvas');
    const ctx = canvas.getContext('2d');
    let roll = 0, pitch = 0, yaw = 0;

    const vertices = [
      [-1,-1.5,-1], [1,-1.5,-1], [1,1.5,-1], [-1,1.5,-1],
      [-1,-1.5, 1], [1,-1.5, 1], [1,1.5, 1], [-1,1.5, 1],
      [-2.5,-1, 0], [-1,-1, 0], [-1,1, 0], [-2.5,1, 0],
      [1,-1, 0], [2.5,-1, 0], [2.5,1, 0], [1,1, 0]
    ];
    const edges = [
      [0,1],[1,2],[2,3],[3,0], [4,5],[5,6],[6,7],[7,4],
      [0,4],[1,5],[2,6],[3,7],
      [8,9],[9,10],[10,11],[11,8],
      [12,13],[13,14],[14,15],[15,12]
    ];

    function draw3D() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const cx = canvas.width / 2;
      const cy = canvas.height / 2;
      const scale = 32;

      const rR = roll * Math.PI / 180;
      const rP = pitch * Math.PI / 180;
      const rY = yaw * Math.PI / 180;

      const proj = vertices.map(v => {
        let x = v[0], y = v[1], z = v[2];
        let x1 = x * Math.cos(rY) + z * Math.sin(rY);
        let z1 = -x * Math.sin(rY) + z * Math.cos(rY);
        let y2 = y * Math.cos(rP) - z1 * Math.sin(rP);
        let x3 = x1 * Math.cos(rR) - y2 * Math.sin(rR);
        let y3 = x1 * Math.sin(rR) + y2 * Math.cos(rR);
        return [cx + x3 * scale, cy - y3 * scale];
      });

      ctx.lineWidth = 1.5;
      edges.forEach(([i, j], idx) => {
        ctx.beginPath();
        ctx.moveTo(proj[i][0], proj[i][1]);
        ctx.lineTo(proj[j][0], proj[j][1]);
        ctx.strokeStyle = idx >= 12 ? '#38bdf8' : '#00f0ff';
        ctx.stroke();
      });
    }

    // ---------- SVG charts ----------
    const maxPts = 50;
    const attData = [];
    const tempData = [];

    function renderSvgChart(svgId, lines, minVal, maxVal, ticks) {
      const svg = document.getElementById(svgId);
      const w = 500, h = 160;
      const padLeft = 45, padRight = 10, padTop = 15, padBtm = 20;
      const plotW = w - padLeft - padRight;
      const plotH = h - padTop - padBtm;

      let html = '';

      ticks.forEach(t => {
        const norm = (t - minVal) / (maxVal - minVal);
        const y = padTop + plotH - (norm * plotH);
        html += `<line x1="${padLeft}" y1="${y}" x2="${w - padRight}" y2="${y}" stroke="#1f293d" stroke-dasharray="3,3" />`;
        html += `<text x="5" y="${y + 3}" fill="#64748b" font-size="10" font-family="monospace">${t > 0 ? '+' : ''}${t}</text>`;
      });

      html += `<line x1="${padLeft}" y1="${padTop}" x2="${padLeft}" y2="${padTop + plotH}" stroke="#334155" />`;
      html += `<line x1="${padLeft}" y1="${padTop + plotH}" x2="${w - padRight}" y2="${padTop + plotH}" stroke="#334155" />`;

      lines.forEach(line => {
        if (line.data.length < 2) return;
        const step = plotW / (maxPts - 1);
        let dStr = '';
        line.data.forEach((val, idx) => {
          const norm = (val - minVal) / (maxVal - minVal);
          const clamped = Math.max(0, Math.min(1, norm));
          const x = padLeft + (idx * step);
          const y = padTop + plotH - (clamped * plotH);
          dStr += (idx === 0 ? `M ${x} ${y}` : ` L ${x} ${y}`);
        });
        html += `<path d="${dStr}" fill="none" stroke="${line.color}" stroke-width="2" />`;
      });

      svg.innerHTML = html;
    }

    // ---------- Logs & CSV ----------
    const CSV_MAX_ROWS = 5000;
    const csvRows = [["Timestamp", "Seq", "Mode", "BootCause", "FDIR", "PicoTemp_C", "MpuTemp_C", "Roll_deg", "Pitch_deg", "Yaw_deg", "CRC"]];
    let prevMode = "", prevFdir = "";

    function addLog(msg, type = "info") {
      const logBox = document.getElementById('event-log');
      const time = new Date().toISOString().substring(11, 19);
      const div = document.createElement('div');
      div.className = 'log-line';
      if (type === "warn") div.style.color = "var(--orange)";
      if (type === "err") div.style.color = "var(--red)";
      div.innerText = `[${time}] ${msg}`;
      logBox.prepend(div);
      while (logBox.children.length > 200) logBox.removeChild(logBox.lastChild);
    }

    function formatUptime(ms) {
      let s = Math.floor(ms / 1000);
      let m = Math.floor(s / 60); s %= 60;
      let h = Math.floor(m / 60); m %= 60;
      return `${String(h).padStart(2,'0')}:${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
    }

    // ---------- Link supervision ----------
    const LOS_TIMEOUT_MS = 3000;   // > 500 ms SAFE-mode telemetry period
    let losFlag = false;
    let lastAckN = 0;
    let gsOfflineFlag = false;
    let lastUptime = -1;
    let newPkts = 0;

    function setLink(ok, text) {
      const lb = document.getElementById('link-badge');
      lb.innerText = text;
      lb.className = 'tag ' + (ok ? 'tag-green' : 'tag-red');
    }

    // Measured downlink rate (new frames per second)
    setInterval(() => {
      document.getElementById('rate-badge').innerText = (newPkts / 2).toFixed(1) + ' Hz';
      newPkts = 0;
    }, 2000);

    // ---------- Telemetry polling ----------
    setInterval(() => {
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          if (gsOfflineFlag) {
            gsOfflineFlag = false;
            addLog("Ground station connection restored.");
          }

          // Telecommand acceptance reports from the OBC
          if (d.ack_n !== lastAckN) {
            if (lastAckN !== 0 || d.ack_n === 1) {
              addLog('TC REPORT: ' + d.ack, d.ack.indexOf('REJECTED') >= 0 ? 'warn' : 'info');
            }
            lastAckN = d.ack_n;
          }

          // Loss-of-signal detection
          const lost = (d.age < 0 || d.age > LOS_TIMEOUT_MS);
          if (lost) {
            setLink(false, d.age < 0 ? 'AIR LINK: WAITING' : 'AIR LINK: LOST');
            document.getElementById('val-mode').innerText = 'NO SIGNAL';
            if (!losFlag) {
              losFlag = true;
              if (d.age >= 0) addLog('LOSS OF SIGNAL - no frames from OBC', 'err');
            }
            return;
          }
          if (losFlag) {
            losFlag = false;
            addLog('SIGNAL ACQUIRED - frames from OBC resumed');
          }
          setLink(true, 'AIR LINK: OK');

          document.getElementById('val-mode').innerText = d.mode;
          document.getElementById('val-boot').innerText = d.boot;
          document.getElementById('val-pico-temp').innerText = d.pico_temp.toFixed(1) + ' °C';
          document.getElementById('val-mpu-temp').innerText = d.mpu_temp.toFixed(1) + ' °C';
          document.getElementById('val-seq').innerText = '#' + d.seq;
          document.getElementById('val-uptime').innerText = formatUptime(d.uptime);

          const fdirEl = document.getElementById('val-fdir');
          fdirEl.innerText = d.fdir;
          fdirEl.style.color = (d.fdir === "NOMINAL") ? "var(--cyan)" : "var(--red)";

          const crcTag = document.getElementById('crc-tag');
          crcTag.innerText = d.crc ? "CRC: VALID" : "CRC: ERROR";
          crcTag.className = 'tag ' + (d.crc ? "tag-green" : "tag-red");

          // CCSDS parser (real values from the packet header)
          const apid = d.pktid & 0x07FF;
          document.getElementById('ccsds-apid').innerText =
            '0x' + apid.toString(16).toUpperCase().padStart(3, '0') +
            ' (PKT 0x' + d.pktid.toString(16).toUpperCase().padStart(4, '0') + ')';
          document.getElementById('ccsds-seq').innerText = '#' + d.seq;
          document.getElementById('ccsds-len').innerText = (d.len + 1) + ' bytes';   // CCSDS: field = length - 1
          const ccrc = document.getElementById('ccsds-crc');
          ccrc.innerText = d.crc ? "VALID" : "ERROR";
          ccrc.style.color = d.crc ? "var(--green)" : "var(--red)";
          if (d.raw) document.getElementById('hex-view').innerText = d.raw;

          if (d.rtt >= 0) {
            document.getElementById('rtt-badge').innerText = `RTT: ${d.rtt} ms`;
            document.getElementById('rtt-badge').className = 'tag tag-green';
          }

          if (prevMode !== "" && prevMode !== d.mode) {
            addLog(`STATE MACHINE: Mode changed to ${d.mode}`, d.mode === "SAFE_MODE" ? "warn" : "info");
          }
          prevMode = d.mode;

          if (prevFdir !== "" && prevFdir !== d.fdir && d.fdir !== "NOMINAL") {
            addLog(`FDIR EXCEPTION: ${d.fdir}! Transferred to SAFE_MODE`, "err");
          }
          prevFdir = d.fdir;

          roll = d.roll;
          pitch = d.pitch;
          yaw = d.yaw;
          document.getElementById('r-val').innerText = roll.toFixed(1) + '°';
          document.getElementById('p-val').innerText = pitch.toFixed(1) + '°';
          document.getElementById('y-val').innerText = yaw.toFixed(1) + '°';
          draw3D();

          // Only store NEW, valid frames (the poll is faster than the downlink in SAFE mode)
          if (!d.crc || d.uptime === lastUptime) return;
          lastUptime = d.uptime;
          newPkts++;

          if (attData.length >= maxPts) attData.shift();
          attData.push({ r: roll, p: pitch, y: yaw });

          if (tempData.length >= maxPts) tempData.shift();
          tempData.push({ pico: d.pico_temp, mpu: d.mpu_temp });

          renderSvgChart('svgAtt', [
            { data: attData.map(a => a.r), color: '#00f0ff' },
            { data: attData.map(a => a.p), color: '#3b82f6' },
            { data: attData.map(a => a.y), color: '#f59e0b' }
          ], -180, 180, [-180, -90, 0, 90, 180]);

          renderSvgChart('svgTemp', [
            { data: tempData.map(t => t.pico), color: '#ef4444' },
            { data: tempData.map(t => t.mpu), color: '#f59e0b' }
          ], 20, 80, [20, 40, 60, 80]);

          csvRows.push([new Date().toISOString(), d.seq, d.mode, d.boot, d.fdir, d.pico_temp, d.mpu_temp, roll, pitch, yaw, "VALID"]);
          if (csvRows.length > CSV_MAX_ROWS + 1) csvRows.splice(1, 1);   // keep header, drop oldest
        })
        .catch(() => {
          setLink(false, 'GROUND STN: OFFLINE');
          if (!gsOfflineFlag) {
            gsOfflineFlag = true;
            addLog('Cannot reach ground station web server', 'err');
          }
        });
    }, 120);

    function sendTC(id, val) {
      addLog(`TC UPLINK: $TC:${id},${val}`);
      fetch(`/api/tc?id=${id}&val=${val}`);
    }

    function confirmReboot() {
      if (confirm("Confirm command: FORCE HARDWARE WATCHDOG RESET?")) {
        addLog("TC UPLINK: $TC:99,01 (Hardware WDT Reset Triggered)", "warn");
        fetch('/api/tc?id=99&val=01');
      }
    }

    function exportCSV() {
      let csv = "data:text/csv;charset=utf-8," + csvRows.map(e => e.join(",")).join("\n");
      let link = document.createElement("a");
      link.setAttribute("href", encodeURI(csv));
      link.setAttribute("download", `cubesat_telemetry_${new Date().toISOString().substring(0,19)}.csv`);
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      addLog("Exported telemetry session to CSV.");
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleData() {
  long age = last_rx_ms ? (long)(millis() - last_rx_ms) : -1L;

  String json = "{";
  json += "\"mpu_temp\":" + String(sat_mpu_temp, 2) + ",";
  json += "\"pico_temp\":" + String(sat_mcu_temp, 2) + ",";
  json += "\"roll\":" + String(sat_roll, 2) + ",";
  json += "\"pitch\":" + String(sat_pitch, 2) + ",";
  json += "\"yaw\":" + String(sat_yaw, 2) + ",";
  json += "\"uptime\":" + String(sat_uptime) + ",";
  json += "\"mode\":\"" + sat_mode + "\",";
  json += "\"boot\":\"" + sat_boot_cause + "\",";
  json += "\"fdir\":\"" + sat_fdir_msg + "\",";
  json += "\"seq\":" + String(last_packet_seq) + ",";
  json += "\"pktid\":" + String(last_pkt_id) + ",";
  json += "\"len\":" + String(last_data_len) + ",";
  json += "\"crc\":" + String(crc_valid ? "true" : "false") + ",";
  json += "\"rtt\":" + String(last_ping_rtt) + ",";
  json += "\"age\":" + String(age) + ",";
  json += "\"ack_n\":" + String(ack_n) + ",";
  json += "\"ack\":\"" + last_ack + "\",";
  json += "\"raw\":\"" + last_raw_hex + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleTC() {
  if (server.hasArg("id") && server.hasArg("val")) {
    String id = server.arg("id");
    String val = server.arg("val");
    String tc_packet = "$TC:" + id + "," + val + "\n";

    if (id == "10" && val == "00") {
      ping_sent_time = millis();
      last_ping_rtt = -1;
    }

    udp.beginPacket(pico_ip, UDP_PORT_TX);
    udp.write((const uint8_t*)tc_packet.c_str(), tc_packet.length());
    udp.endPacket();
    Serial.printf("[TC] sent $TC:%s,%s to %s:%u\n", id.c_str(), val.c_str(), pico_ip.toString().c_str(), UDP_PORT_TX);

    server.send(200, "text/plain", "TC_DISPATCHED");
  } else {
    server.send(400, "text/plain", "BAD_TC_REQ");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);

  udp.begin(UDP_PORT_RX);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/tc", HTTP_GET, handleTC);

  server.begin();
}

void loop() {
  server.handleClient();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 254);
    if (len > 0) {
      packetBuffer[len] = 0;
      String line = String(packetBuffer);
      line.trim();

      if (line.indexOf("$TM:PUS17,ALIVE") >= 0) {
        last_rx_ms = millis();
        if (ping_sent_time > 0) {
          last_ping_rtt = millis() - ping_sent_time;
          ping_sent_time = 0;
        }
      }
      else if (line.startsWith("$TM:ACK,")) {
        last_rx_ms = millis();
        Serial.println("[RX] " + line);
        String body = line.substring(8);              // e.g. "20,01,0"
        int c1 = body.indexOf(',');
        int c2 = body.lastIndexOf(',');
        if (c1 > 0 && c2 > c1) {
          int st = body.substring(c2 + 1).toInt();
          const char *txt = (st == 0) ? "ACCEPTED" :
                            (st == 1) ? "REJECTED (unknown)" :
                            (st == 2) ? "REJECTED (malformed)" : "REJECTED (queue full)";
          last_ack = "TC " + body.substring(0, c2) + " " + txt;
          ack_n++;
        }
      }
      else if (line.startsWith("$CCSDS:")) {
        last_rx_ms = millis();
        String hex = line.substring(7);
        last_raw_hex = hex;
        size_t byteLen = hex.length() / 2;

        if (byteLen == sizeof(ccsds_telemetry_packet_t)) {
          uint8_t buffer[sizeof(ccsds_telemetry_packet_t)];
          for (size_t i = 0; i < byteLen; i++) {
            String byteString = hex.substring(i * 2, i * 2 + 2);
            buffer[i] = (uint8_t) strtol(byteString.c_str(), NULL, 16);
          }

          ccsds_telemetry_packet_t pkt;
          memcpy(&pkt, buffer, sizeof(ccsds_telemetry_packet_t));

          uint16_t expected_crc = calc_crc16((uint8_t*)&pkt, sizeof(ccsds_telemetry_packet_t) - 2);
          if (expected_crc == pkt.crc16 &&
              pkt.header.packet_id == 0x0801u &&
              (pkt.header.packet_seq_ctrl & 0xC000u) == 0xC000u &&
              pkt.header.packet_data_len == sizeof(ccsds_telemetry_packet_t) - 7u &&
              pkt.flight_mode <= 3u && pkt.fdir_alarm <= 3u && pkt.boot_cause <= 5u) {
            /* Learn command endpoint only from a structurally valid frame. */
            pico_ip = udp.remoteIP();
            crc_valid = true;
            last_pkt_id = pkt.header.packet_id;
            last_data_len = pkt.header.packet_data_len;
            sat_uptime = pkt.uptime_ms;
            sat_mpu_temp = (float)pkt.mpu_temp_centi_c / 100.0f;
            sat_mcu_temp = (float)pkt.mcu_temp_centi_c / 100.0f;
            sat_roll = (float)pkt.roll_centi_deg / 100.0f;
            sat_pitch = (float)pkt.pitch_centi_deg / 100.0f;
            sat_yaw = (float)pkt.yaw_centi_deg / 100.0f;

            switch (pkt.flight_mode) {
              case 0: sat_mode = "BOOT"; break;
              case 1: sat_mode = "DETUMBLE"; break;
              case 2: sat_mode = "NOMINAL"; break;
              case 3: sat_mode = "SAFE_MODE"; break;
              default: sat_mode = "UNKNOWN"; break;
            }

            switch (pkt.boot_cause) {
              case 0: sat_boot_cause = "POWER_ON"; break;
              case 1: sat_boot_cause = "WATCHDOG_TIMEOUT"; break;
              case 2: sat_boot_cause = "COMMANDED"; break;
              case 3: sat_boot_cause = "STACK_OVERFLOW"; break;
              case 4: sat_boot_cause = "HEAP_EXHAUSTED"; break;
              case 5: sat_boot_cause = "FATAL_INIT"; break;
              default: sat_boot_cause = "UNKNOWN"; break;
            }

            switch (pkt.fdir_alarm) {
              case 0: sat_fdir_msg = "NOMINAL"; break;
              case 1: sat_fdir_msg = "OVERHEAT (>65C)"; break;
              case 2: sat_fdir_msg = "UNCONTROLLED TUMBLE"; break;
              case 3: sat_fdir_msg = "IMU FAULT"; break;
              default: sat_fdir_msg = "UNKNOWN"; break;
            }

            last_packet_seq = pkt.header.packet_seq_ctrl & 0x3FFF;
          } else {
            crc_valid = false;
          }
        } else {
          // Wrong frame size: flag it instead of silently keeping the previous "VALID"
          crc_valid = false;
        }
      }
    }
  }
}