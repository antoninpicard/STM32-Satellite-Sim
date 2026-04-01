/*
 * ESP32 Satellite Simulator — WiFi Bridge
 * 
 * Receives telemetry from STM32 via UART (Serial2)
 * Hosts a web dashboard with WebSocket for real-time data
 * Sends commands back to STM32 (mode change, alarm thresholds, data dump)
 *
 * Wiring:
 *   ESP32 GPIO16 (RX2) <--- STM32 PA9  (USART1_TX)
 *   ESP32 GPIO17 (TX2) ---> STM32 PA10 (USART1_RX)
 *   GND <---> GND
 *
 * Libraries needed (install via Arduino Library Manager):
 *   - ESPAsyncWebServer
 *   - AsyncTCP
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

/* ---- WiFi credentials ---- */
const char* ssid     = "SATELLITE_SIM";
const char* password = "sat12345";

/* ---- UART to STM32 ---- */
#define STM32_BAUD 115200

/* ---- Web server + WebSocket ---- */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/* ---- Latest telemetry line from STM32 ---- */
String lastTelemetry = "";

/* ---- Command protocol to STM32 ---- */
/* 
 * Commands sent over UART to STM32:
 *   CMD:MODE:NOMINAL\n
 *   CMD:MODE:SAFE\n
 *   CMD:MODE:SCIENCE\n
 *   CMD:MODE:ERROR_LOW\n
 *   CMD:MODE:ERROR_HIGH\n
 *   CMD:ALARM:TILT:0.8\n
 *   CMD:ALARM:TEMP:50.0\n
 *   CMD:ALARM:PRES_LOW:950.0\n
 *   CMD:ALARM:PRES_HIGH:1050.0\n
 *   CMD:ALARM:ALT_HIGH:2000.0\n
 *   CMD:ALARM:ALT_LOW:-100.0\n
 *   CMD:DUMP\n
 */

/* ---- HTML Dashboard ---- */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Satellite Simulator</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: 'Courier New', monospace; background: #0a0a0a; color: #00ff41; padding: 10px; }
  h1 { text-align: center; font-size: 1.4em; margin-bottom: 10px; color: #00ff41; 
       text-shadow: 0 0 10px #00ff41; }
  .status-bar { display: flex; justify-content: space-between; padding: 5px 10px; 
                background: #111; border: 1px solid #00ff41; margin-bottom: 10px; font-size: 0.8em; }
  .panel { background: #111; border: 1px solid #333; padding: 10px; margin-bottom: 10px; border-radius: 4px; }
  .panel h2 { font-size: 1em; color: #0af; margin-bottom: 8px; border-bottom: 1px solid #333; 
              padding-bottom: 4px; }
  .data-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 5px; }
  .data-item { font-size: 0.85em; }
  .data-item span { color: #fff; }
  .mode-btns { display: flex; gap: 5px; flex-wrap: wrap; }
  .mode-btns button { flex: 1; padding: 8px; border: 1px solid #333; background: #1a1a1a; 
                      color: #ccc; cursor: pointer; font-family: inherit; font-size: 0.8em; 
                      border-radius: 3px; min-width: 80px; }
  .mode-btns button:hover { background: #333; }
  .mode-btns button.active { background: #00ff41; color: #000; border-color: #00ff41; }
  .mode-btns button.error-low { border-color: #ff0; }
  .mode-btns button.error-low.active { background: #ff0; color: #000; }
  .mode-btns button.error-high { border-color: #f00; }
  .mode-btns button.error-high.active { background: #f00; color: #fff; }
  .threshold-row { display: flex; align-items: center; gap: 8px; margin: 4px 0; font-size: 0.8em; }
  .threshold-row label { min-width: 90px; }
  .threshold-row input { width: 70px; background: #1a1a1a; border: 1px solid #333; color: #fff; 
                          padding: 3px 5px; font-family: inherit; }
  .threshold-row button { padding: 3px 8px; background: #0af; border: none; color: #000; 
                           cursor: pointer; font-family: inherit; border-radius: 2px; }
  .btn-dump { width: 100%; padding: 10px; background: #0af; border: none; color: #000; 
              cursor: pointer; font-family: inherit; font-size: 0.9em; border-radius: 3px; 
              margin-top: 5px; }
  .alarm-flag { display: inline-block; padding: 2px 6px; border-radius: 2px; font-size: 0.8em; margin: 2px; }
  .alarm-ok { background: #1a3a1a; color: #0f0; }
  .alarm-on { background: #3a1a1a; color: #f00; animation: blink 0.5s infinite; }
  @keyframes blink { 50% { opacity: 0.3; } }
  .log { background: #050505; border: 1px solid #333; padding: 5px; height: 100px; 
         overflow-y: auto; font-size: 0.7em; color: #666; margin-top: 5px; }
  #wsStatus { color: #f00; }
  #wsStatus.connected { color: #0f0; }
</style>
</head>
<body>
<h1>SATELLITE SIMULATOR</h1>
<div class="status-bar">
  <span>WS: <span id="wsStatus">DISCONNECTED</span></span>
  <span>MODE: <span id="currentMode">--</span></span>
  <span>UPTIME: <span id="uptime">0s</span></span>
</div>

<div class="panel">
  <h2>IMU DATA</h2>
  <div class="data-grid">
    <div class="data-item">AX: <span id="ax">--</span></div>
    <div class="data-item">AY: <span id="ay">--</span></div>
    <div class="data-item">AZ: <span id="az">--</span></div>
    <div class="data-item">GX: <span id="gx">--</span></div>
    <div class="data-item">GY: <span id="gy">--</span></div>
    <div class="data-item">GZ: <span id="gz">--</span></div>
  </div>
  <div class="data-item" style="margin-top:4px">T_IMU: <span id="timu">--</span> C</div>
</div>

<div class="panel">
  <h2>BMP280 DATA</h2>
  <div class="data-grid">
    <div class="data-item">TEMP: <span id="tbmp">--</span> C</div>
    <div class="data-item">PRES: <span id="pres">--</span> hPa</div>
    <div class="data-item">ALT: <span id="alt">--</span> m</div>
  </div>
</div>

<div class="panel">
  <h2>ALARMS</h2>
  <span id="a_tilt" class="alarm-flag alarm-ok">TILT:OK</span>
  <span id="a_temp" class="alarm-flag alarm-ok">TEMP:OK</span>
  <span id="a_pres" class="alarm-flag alarm-ok">PRES:OK</span>
  <span id="a_alt" class="alarm-flag alarm-ok">ALT:OK</span>
</div>

<div class="panel">
  <h2>MODE CONTROL</h2>
  <div class="mode-btns">
    <button id="btnNominal" onclick="sendMode('NOMINAL')">NOMINAL</button>
    <button id="btnSafe" onclick="sendMode('SAFE')">SAFE</button>
    <button id="btnScience" onclick="sendMode('SCIENCE')">SCIENCE</button>
    <button id="btnErrorLow" class="error-low" onclick="sendMode('ERROR_LOW')">ERROR LOW</button>
    <button id="btnErrorHigh" class="error-high" onclick="sendMode('ERROR_HIGH')">ERROR HIGH</button>
  </div>
</div>

<div class="panel">
  <h2>ALARM THRESHOLDS</h2>
  <div class="threshold-row">
    <label>Tilt (g):</label>
    <input type="number" id="thTilt" value="0.8" step="0.1">
    <button onclick="sendThreshold('TILT', 'thTilt')">SET</button>
  </div>
  <div class="threshold-row">
    <label>Temp (C):</label>
    <input type="number" id="thTemp" value="50" step="1">
    <button onclick="sendThreshold('TEMP', 'thTemp')">SET</button>
  </div>
  <div class="threshold-row">
    <label>Pres Low:</label>
    <input type="number" id="thPresLow" value="950" step="5">
    <button onclick="sendThreshold('PRES_LOW', 'thPresLow')">SET</button>
  </div>
  <div class="threshold-row">
    <label>Pres High:</label>
    <input type="number" id="thPresHigh" value="1050" step="5">
    <button onclick="sendThreshold('PRES_HIGH', 'thPresHigh')">SET</button>
  </div>
  <div class="threshold-row">
    <label>Alt High (m):</label>
    <input type="number" id="thAltHigh" value="2000" step="100">
    <button onclick="sendThreshold('ALT_HIGH', 'thAltHigh')">SET</button>
  </div>
  <div class="threshold-row">
    <label>Alt Low (m):</label>
    <input type="number" id="thAltLow" value="-100" step="10">
    <button onclick="sendThreshold('ALT_LOW', 'thAltLow')">SET</button>
  </div>
</div>

<div class="panel">
  <button class="btn-dump" onclick="sendCmd('CMD:DUMP')">REQUEST DATA DUMP</button>
  <div class="log" id="log"></div>
</div>

<script>
let ws;
let uptime = 0;
let currentMode = 'NOMINAL';

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen = () => {
    document.getElementById('wsStatus').textContent = 'CONNECTED';
    document.getElementById('wsStatus').className = 'connected';
  };
  ws.onclose = () => {
    document.getElementById('wsStatus').textContent = 'DISCONNECTED';
    document.getElementById('wsStatus').className = '';
    setTimeout(connect, 2000);
  };
  ws.onmessage = (e) => {
    let d = e.data.trim();
    if (d.startsWith('MODE:')) {
      currentMode = d.substring(5);
      document.getElementById('currentMode').textContent = currentMode;
      updateModeButtons();
      return;
    }
    if (d.startsWith('LOG:')) {
      appendLog(d.substring(4));
      return;
    }
    /* Parse CSV telemetry: AX,AY,AZ,GX,GY,GZ,T_IMU,T_BMP,P,ALT,JX,JY,ALARMS */
    let v = d.split(',');
    if (v.length >= 13) {
      document.getElementById('ax').textContent = v[0];
      document.getElementById('ay').textContent = v[1];
      document.getElementById('az').textContent = v[2];
      document.getElementById('gx').textContent = v[3];
      document.getElementById('gy').textContent = v[4];
      document.getElementById('gz').textContent = v[5];
      document.getElementById('timu').textContent = v[6];
      document.getElementById('tbmp').textContent = v[7];
      document.getElementById('pres').textContent = v[8];
      document.getElementById('alt').textContent = v[9];
      let alarms = parseInt(v[12]);
      setAlarm('a_tilt', 'TILT', alarms & 1);
      setAlarm('a_temp', 'TEMP', alarms & 2);
      setAlarm('a_pres', 'PRES', alarms & 4);
      setAlarm('a_alt', 'ALT', alarms & 8);
    }
  };
}

function setAlarm(id, name, on) {
  let el = document.getElementById(id);
  el.textContent = name + ':' + (on ? '!!' : 'OK');
  el.className = 'alarm-flag ' + (on ? 'alarm-on' : 'alarm-ok');
}

function sendCmd(cmd) {
  if (ws && ws.readyState === 1) ws.send(cmd);
  appendLog('> ' + cmd);
}

function sendMode(mode) {
  sendCmd('CMD:MODE:' + mode);
}

function sendThreshold(param, inputId) {
  let val = document.getElementById(inputId).value;
  sendCmd('CMD:ALARM:' + param + ':' + val);
}

function updateModeButtons() {
  ['Nominal','Safe','Science','ErrorLow','ErrorHigh'].forEach(m => {
    document.getElementById('btn'+m).classList.remove('active');
  });
  let map = {'NOMINAL':'Nominal','SAFE':'Safe','SCIENCE':'Science',
             'ERROR_LOW':'ErrorLow','ERROR_HIGH':'ErrorHigh'};
  if (map[currentMode]) document.getElementById('btn'+map[currentMode]).classList.add('active');
}

function appendLog(msg) {
  let log = document.getElementById('log');
  log.innerHTML += msg + '<br>';
  log.scrollTop = log.scrollHeight;
}

setInterval(() => {
  uptime++;
  document.getElementById('uptime').textContent = uptime + 's';
}, 1000);

connect();
</script>
</body>
</html>
)rawliteral";

/* ---- WebSocket event handler ---- */
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.println("[WS] Client connected");
    /* Send current mode */
    client->text("MODE:NOMINAL");
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.println("[WS] Client disconnected");
  }
  else if (type == WS_EVT_DATA)
  {
    /* Forward command to STM32 */
    String cmd = "";
    for (size_t i = 0; i < len; i++) cmd += (char)data[i];
    cmd.trim();
    Serial.println("[WS] CMD: " + cmd);

    /* Send to STM32 via UART */
    Serial2.println(cmd);

    /* Echo back to all WS clients */
    ws.textAll("LOG:" + cmd + " -> sent");
  }
}

void setup()
{
  Serial.begin(115200);   /* Debug USB */
  Serial2.begin(STM32_BAUD, SERIAL_8N1, 16, 17);  /* UART to STM32: RX=16, TX=17 */

  /* Start WiFi as Access Point */
  WiFi.softAP(ssid, password);
  Serial.println("[WiFi] AP started");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.softAPIP());

  /* WebSocket */
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  /* HTTP */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
  Serial.println("[HTTP] Server started");
}

void loop()
{
  /* Read telemetry from STM32 and forward to WebSocket clients */
  if (Serial2.available())
  {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      lastTelemetry = line;
      ws.textAll(line);
    }
  }

  /* Cleanup disconnected clients */
  ws.cleanupClients();

  delay(1);
}
