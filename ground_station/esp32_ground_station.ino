#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

const char *ssid = "CUBESAT-GROUND-STATION";
const char *password = "12345678";

WebServer server(80);
WiFiUDP udp;

const unsigned int UDP_PORT_RX = 5000; // Ακούει τηλεμετρία από Pico
const unsigned int UDP_PORT_TX = 5001; // Στέλνει εντολές στο Pico
IPAddress pico_ip(192, 168, 4, 2);     // Η IP που παίρνει συνήθως το Pico

#pragma pack(push, 1)
typedef struct {
  uint16_t packet_id;        
  uint16_t packet_seq_ctrl;  
  uint16_t packet_data_len;  
} ccsds_primary_header_t;

typedef struct {
  ccsds_primary_header_t header; 
  uint32_t uptime_ms;            
  int16_t  core_temp_centi_c;    
  uint8_t  flight_mode;          
  uint16_t crc16;                
} ccsds_telemetry_packet_t;
#pragma pack(pop)

// Telemetry State
float sat_temp = 0.0;
String sat_mode = "DISCONNECTED";
uint32_t last_packet_seq = 0;
bool crc_valid = false;
String last_raw_hex = "--";

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
  <title>AEROSPACE GROUND SEGMENT | MOC</title>
  <style>
    :root {
      --bg: #0b0f19;
      --card: #111827;
      --border: #1f2937;
      --accent: #00e5ff;
      --text: #e5e7eb;
      --dim: #9ca3af;
      --green: #10b981;
      --danger: #ef4444;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: monospace; }
    body { background-color: var(--bg); color: var(--text); padding: 15px; }
    .top-bar { display: flex; justify-content: space-between; align-items: center; border-bottom: 2px solid var(--border); padding-bottom: 10px; margin-bottom: 15px; }
    .title { font-size: 1.1rem; font-weight: bold; color: var(--accent); }
    .badge { padding: 4px 8px; border-radius: 4px; background: #1e293b; border: 1px solid var(--border); font-size: 0.8rem; }
    .badge.online { color: var(--green); border-color: var(--green); }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 15px; }
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 6px; padding: 15px; }
    .card-title { font-size: 0.8rem; color: var(--dim); border-bottom: 1px solid var(--border); padding-bottom: 5px; margin-bottom: 10px; text-transform: uppercase; }
    .data-row { display: flex; justify-content: space-between; padding: 6px 0; border-bottom: 1px dashed rgba(255,255,255,0.05); font-size: 0.85rem; }
    .data-val { color: var(--accent); font-weight: bold; }
    #cubeCanvas { width: 100%; height: 230px; background: #05070d; border-radius: 4px; border: 1px solid var(--border); display: block; }
    .btn-group { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 10px; }
    button.cmd-btn { background: #1f2937; color: var(--text); border: 1px solid var(--border); padding: 10px; cursor: pointer; border-radius: 4px; font-weight: bold; font-size: 0.8rem; }
    button.cmd-btn:hover { border-color: var(--accent); color: var(--accent); }
    button.danger:hover { border-color: var(--danger); color: var(--danger); }
    .terminal { background: #05070d; border: 1px solid var(--border); color: #38bdf8; padding: 10px; font-size: 0.75rem; height: 60px; overflow-y: auto; border-radius: 4px; margin-top: 10px; word-break: break-all; }
  </style>
</head>
<body>
  <div class="top-bar">
    <div class="title">🛰️ MISSION CONTROL CENTER (AIR LINK)</div>
    <div>
      <span class="badge" id="utc-clock">UTC 00:00:00</span>
      <span class="badge online">RF: ACTIVE</span>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="card-title">Attitude Determination (ADCS 3D)</div>
      <canvas id="cubeCanvas" width="300" height="230"></canvas>
      <div class="data-row" style="margin-top: 10px;">
        <span>Roll: <span class="data-val" id="val-roll">0.0°</span></span>
        <span>Pitch: <span class="data-val" id="val-pitch">0.0°</span></span>
        <span>Yaw: <span class="data-val" id="val-yaw">0.0°</span></span>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Telemetry & Health Status</div>
      <div class="data-row"><span>Target Asset</span><span class="data-val">0x01 (CubeSat)</span></div>
      <div class="data-row"><span>Mode</span><span class="data-val" id="val-mode">--</span></div>
      <div class="data-row"><span>Core Temp</span><span class="data-val" id="val-temp">--.- °C</span></div>
      <div class="data-row"><span>Frame Counter</span><span class="data-val" id="val-seq">#0000</span></div>
      <div class="data-row"><span>CRC16 Check</span><span class="data-val" id="val-crc">--</span></div>
    </div>

    <div class="card">
      <div class="card-title">Telecommand Uplink</div>
      <div class="btn-group">
        <button class="cmd-btn" onclick="sendTC('10', '00')">PING OBC</button>
        <button class="cmd-btn" onclick="sendTC('20', '00')">SET NOMINAL</button>
        <button class="cmd-btn danger" onclick="sendTC('20', '01')">SET SAFE MODE</button>
        <button class="cmd-btn" onclick="resetAngles()">RESET ATTITUDE</button>
      </div>

      <div class="card-title" style="margin-top: 15px;">CCSDS Downlink Hex Inspector</div>
      <div class="terminal" id="hex-view">Waiting for RF frames...</div>
    </div>
  </div>

  <script>
    setInterval(() => {
      document.getElementById('utc-clock').innerText = 'UTC ' + new Date().toISOString().substring(11, 19);
    }, 1000);

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
      const scale = 36;

      const radRoll = roll * Math.PI / 180;
      const radPitch = pitch * Math.PI / 180;
      const radYaw = yaw * Math.PI / 180;

      const projected = vertices.map(v => {
        let x = v[0], y = v[1], z = v[2];
        let x1 = x * Math.cos(radYaw) + z * Math.sin(radYaw);
        let z1 = -x * Math.sin(radYaw) + z * Math.cos(radYaw);
        let y2 = y * Math.cos(radPitch) - z1 * Math.sin(radPitch);
        let z2 = y * Math.sin(radPitch) + z1 * Math.cos(radPitch);
        let x3 = x1 * Math.cos(radRoll) - y2 * Math.sin(radRoll);
        let y3 = x1 * Math.sin(radRoll) + y2 * Math.cos(radRoll);
        return [cx + x3 * scale, cy - y3 * scale];
      });

      ctx.lineWidth = 1.5;
      edges.forEach(([i, j], idx) => {
        ctx.beginPath();
        ctx.moveTo(projected[i][0], projected[i][1]);
        ctx.lineTo(projected[j][0], projected[j][1]);
        ctx.strokeStyle = idx >= 12 ? '#38bdf8' : '#00e5ff';
        ctx.stroke();
      });
    }

    let sim = 0;
    setInterval(() => {
      fetch('/api/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('val-mode').innerText = d.mode;
          document.getElementById('val-temp').innerText = d.temp.toFixed(2) + ' °C';
          document.getElementById('val-seq').innerText = '#' + d.seq;

          const crcEl = document.getElementById('val-crc');
          crcEl.innerText = d.crc ? "VALID" : "CRC ERROR";
          crcEl.style.color = d.crc ? "var(--green)" : "var(--danger)";

          if (d.raw) {
            document.getElementById('hex-view').innerText = d.raw;
          }

          sim += 0.04;
          roll = Math.sin(sim) * 20;
          pitch = Math.cos(sim) * 15;
          yaw = (sim * 10) % 360;

          document.getElementById('val-roll').innerText = roll.toFixed(1) + '°';
          document.getElementById('val-pitch').innerText = pitch.toFixed(1) + '°';
          document.getElementById('val-yaw').innerText = yaw.toFixed(1) + '°';
          draw3D();
        }).catch(() => {});
    }, 250);

    function sendTC(id, val) {
      fetch('/api/tc?id=' + id + '&val=' + val);
    }

    function resetAngles() {
      roll = 0; pitch = 0; yaw = 0;
      draw3D();
    }
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { 
  server.send(200, "text/html", index_html); 
}

void handleData() {
  String json = "{";
  json += "\"temp\":" + String(sat_temp, 2) + ",";
  json += "\"mode\":\"" + sat_mode + "\",";
  json += "\"seq\":" + String(last_packet_seq) + ",";
  json += "\"crc\":" + String(crc_valid ? "true" : "false") + ",";
  json += "\"raw\":\"" + last_raw_hex + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleTC() {
  if (server.hasArg("id") && server.hasArg("val")) {
    String id = server.arg("id");
    String val = server.arg("val");
    String tc_packet = "$TC:" + id + "," + val + "\n";
    
    // Αποστολή εντολής ασύρματα μέσω UDP στο Pico
    udp.beginPacket(pico_ip, UDP_PORT_TX);
    udp.write((const uint8_t*)tc_packet.c_str(), tc_packet.length());
    udp.endPacket();

    server.send(200, "text/plain", "TC_DISPATCHED");
  } else {
    server.send(400, "text/plain", "BAD_TC_REQ");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);

  // Έναρξη UDP server για λήψη τηλεμετρίας
  udp.begin(UDP_PORT_RX);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/tc", HTTP_GET, handleTC);

  server.begin();
}

void loop() {
  server.handleClient();

  // Έλεγχος για ασύρματα UDP πακέτα από το Pico
  int packetSize = udp.parsePacket();
  if (packetSize) {
    pico_ip = udp.remoteIP(); // Αποθηκεύει αυτόματα την IP του Pico
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 254);
    if (len > 0) {
      packetBuffer[len] = 0;
      String line = String(packetBuffer);
      line.trim();

      if (line.startsWith("$CCSDS:")) {
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
          if (expected_crc == pkt.crc16) {
            crc_valid = true;
            sat_temp = (float)pkt.core_temp_centi_c / 100.0f;
            sat_mode = (pkt.flight_mode == 0) ? "NORMAL" : "SAFE_MODE";
            last_packet_seq = pkt.header.packet_seq_ctrl & 0x3FFF;
          } else {
            crc_valid = false;
          }
        }
      }
    }
  }
}