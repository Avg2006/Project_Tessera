
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ---------- WiFi ----------
const char* ssid = "abc";
const char* password = "12344321";

// ---------- Servers ----------
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ---------- PID Variables ----------
float Kp_roll_rate = 0, Ki_roll_rate = 0, Kd_roll_rate = 0;
float Kp_roll_angle = 0, Ki_roll_angle = 0, Kd_roll_angle = 0;
float Kp_pitch_rate = 0, Ki_pitch_rate = 0, Kd_pitch_rate = 0;
float Kp_pitch_angle = 0, Ki_pitch_angle = 0, Kd_pitch_angle = 0;

// ---------- Telemetry Variables ----------
float roll_pid_rate, rollRate;
float pitch_pid_rate, pitchRate;
float rollInput, roll;
float pitchInput, pitch;
int duty1, duty2, duty3, duty4;
float actual_rate_loop_time = 0.0;
float actual_angle_loop_time = 0.0;

// ---------- HTML Page ----------
String htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Drone Telemetry Dashboard</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial; text-align:center; margin:20px; background:#f2f2f2; }
    .container { display: grid; grid-template-rows: 1fr auto 1fr; gap: 15px; max-width: 1400px; margin: 0 auto; }
    .graph-row { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    .graph-section { background: white; padding: 10px; border-radius: 10px; box-shadow: 0 0 5px gray; }
    .graph-title { font-size: 12px; margin-top: 5px; text-align: center; line-height: 1.2; }
    canvas { width: 100%; height: 250px; }
    .controls { background: white; padding: 15px; border-radius: 10px; box-shadow: 0 0 5px gray; display: grid; grid-template-rows: repeat(5, auto); gap: 10px; }
    .pid-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; align-items: end; }
    .pid-group-left { text-align: left; }
    .pid-group-right { text-align: right; }
    .pid-group h5 { margin: 0 0 5px 0; }
    input { width: 80px; margin: 2px; padding: 3px; }
    .send-btn { justify-self: center; padding: 5px 10px; font-size: 16px; }
    .freq-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; justify-items: center; }
    .freq-group { text-align: left; }
    .motor-row { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; justify-items: center; }
    .motor-group { text-align: left; }
    .motor-group label { display: block; margin-bottom: 5px; }
    span { background: #e0e0e0; padding: 3px 6px; border-radius: 4px; }
  </style>
</head>
<body>
<h2>Drone Telemetry Dashboard</h2>

<div class="container">
  <div class="graph-row">
    <div class="graph-section">
      <canvas id="rollRate"></canvas>
      <div class="graph-title">
        desired roll rate <span id="curr_roll_rate_des">0</span> actual roll rate <span id="curr_roll_rate_act">0</span>
      </div>
    </div>
    <div class="graph-section">
      <canvas id="pitchRate"></canvas>
      <div class="graph-title">
        desired pitch rate <span id="curr_pitch_rate_des">0</span> actual pitch rate <span id="curr_pitch_rate_act">0</span>
      </div>
    </div>
  </div>

  <div class="controls">
    <div class="pid-row">
      <div class="pid-group pid-group-left">
        <h5>Roll Rate</h5>
        Kp <input id="Kp_roll_rate" value="0"><br>
        Ki <input id="Ki_roll_rate" value="0"><br>
        Kd <input id="Kd_roll_rate" value="0">
      </div>
      <div class="pid-group pid-group-right">
        <h5>Pitch Rate</h5>
        Kp <input id="Kp_pitch_rate" value="0"><br>
        Ki <input id="Ki_pitch_rate" value="0"><br>
        Kd <input id="Kd_pitch_rate" value="0">
      </div>
    </div>

    <button class="send-btn" onclick="sendPID()">Send</button>

    <div class="pid-row">
      <div class="pid-group pid-group-left">
        <h5>Roll Angle</h5>
        Kp <input id="Kp_roll_angle" value="0"><br>
        Ki <input id="Ki_roll_angle" value="0"><br>
        Kd <input id="Kd_roll_angle" value="0">
      </div>
      <div class="pid-group pid-group-right">
        <h5>Pitch Angle</h5>
        Kp <input id="Kp_pitch_angle" value="0"><br>
        Ki <input id="Ki_pitch_angle" value="0"><br>
        Kd <input id="Kd_pitch_angle" value="0">
      </div>
    </div>

    <div class="freq-row">
      <div class="freq-group">
        <label>Real rate frequency <span id="actual_rate_loop_time">0</span></label>
      </div>
      <div class="freq-group">
        <label>Real angle frequency <span id="actual_angle_loop_time">0</span></label>
      </div>
    </div>

    <div class="motor-row">
      <div class="motor-group">
        <label>M1 thrust <span id="M1_pwm">0</span></label>
        <label>M2 thrust <span id="M2_pwm">0</span></label>
      </div>
      <div class="motor-group">
        <label>M3 thrust <span id="M3_pwm">0</span></label>
        <label>M4 thrust <span id="M4_pwm">0</span></label>
      </div>
    </div>
  </div>

  <div class="graph-row">
    <div class="graph-section">
      <canvas id="rollTheta"></canvas>
      <div class="graph-title">
        desired roll angle <span id="curr_roll_theta_des">0</span> actual roll angle <span id="curr_roll_theta_act">0</span>
      </div>
    </div>
    <div class="graph-section">
      <canvas id="pitchTheta"></canvas>
      <div class="graph-title">
        desired pitch angle <span id="curr_pitch_theta_des">0</span> actual pitch angle <span id="curr_pitch_theta_act">0</span>
      </div>
    </div>
  </div>
</div>

<script>
let ws = new WebSocket("ws://" + location.hostname + ":81/");
let charts = {};
let maxPoints = 100;

function makeChart(id, label1, label2, ylabel) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: [], datasets: [
      {label: label1, data: [], borderColor: 'blue', fill: false},
      {label: label2, data: [], borderColor: 'red', fill: false}
    ]},
    options: {
      animation: false,
      elements: {
        point: {
          radius: 0
        }
      },
      scales: {x:{title:{display:true,text:'Time(s)'}}, y:{title:{display:true,text: ylabel}}}
    }
  });
}

window.onload = () => {
  charts.rollRate = makeChart("rollRate","desired roll rate","actual roll rate","(degree/sec)");
  charts.pitchRate = makeChart("pitchRate","desired pitch rate","actual pitch rate","(degree/sec)");
  charts.rollTheta = makeChart("rollTheta","desired roll angle","actual roll angle","(degree)");
  charts.pitchTheta = makeChart("pitchTheta","desired pitch angle","actual pitch angle","(degree)");
};

ws.onmessage = (e) => {
  let d = JSON.parse(e.data);
  let t = d.time.toFixed(2);

  function upd(chart, y1, y2, desId, actId) {
    chart.data.labels.push(t);
    chart.data.datasets[0].data.push(y1);
    chart.data.datasets[1].data.push(y2);
    if (chart.data.labels.length > maxPoints) {
      chart.data.labels.shift();
      chart.data.datasets[0].data.shift();
      chart.data.datasets[1].data.shift();
    }
    chart.update();
    // Update current values
    document.getElementById(desId).innerText = y1.toFixed(3);
    document.getElementById(actId).innerText = y2.toFixed(3);
  }

  upd(charts.rollRate, d.roll_pid_rate, d.rollRate, 'curr_roll_rate_des', 'curr_roll_rate_act');
  upd(charts.pitchRate, d.pitch_pid_rate, d.pitchRate, 'curr_pitch_rate_des', 'curr_pitch_rate_act');
  upd(charts.rollTheta, d.rollInput, d.roll, 'curr_roll_theta_des', 'curr_roll_theta_act');
  upd(charts.pitchTheta, d.pitchInput, d.pitch, 'curr_pitch_theta_des', 'curr_pitch_theta_act');

  document.getElementById('M1_pwm').innerText = d.duty1;
  document.getElementById('M2_pwm').innerText = d.duty2;
  document.getElementById('M3_pwm').innerText = d.duty3;
  document.getElementById('M4_pwm').innerText = d.duty4;
  document.getElementById('actual_rate_loop_time').innerText = d.actual_rate_loop_time.toFixed(3);
  document.getElementById('actual_angle_loop_time').innerText = d.actual_angle_loop_time.toFixed(3);
};

function sendPID() {
  let payload = {
    Kp_roll_rate: parseFloat(document.getElementById('Kp_roll_rate').value),
    Ki_roll_rate: parseFloat(document.getElementById('Ki_roll_rate').value),
    Kd_roll_rate: parseFloat(document.getElementById('Kd_roll_rate').value),
    Kp_roll_angle: parseFloat(document.getElementById('Kp_roll_angle').value),
    Ki_roll_angle: parseFloat(document.getElementById('Ki_roll_angle').value),
    Kd_roll_angle: parseFloat(document.getElementById('Kd_roll_angle').value),
    Kp_pitch_rate: parseFloat(document.getElementById('Kp_pitch_rate').value),
    Ki_pitch_rate: parseFloat(document.getElementById('Ki_pitch_rate').value),
    Kd_pitch_rate: parseFloat(document.getElementById('Kd_pitch_rate').value),
    Kp_pitch_angle: parseFloat(document.getElementById('Kp_pitch_angle').value),
    Ki_pitch_angle: parseFloat(document.getElementById('Ki_pitch_angle').value),
    Kd_pitch_angle: parseFloat(document.getElementById('Kd_pitch_angle').value)
  };
  ws.send(JSON.stringify(payload));
}
</script>
</body></html>
)rawliteral";

// ---------- Handlers ----------
void handleRoot() { server.send(200, "text/html", htmlPage); }

void handleWebSocketMessage(uint8_t num, uint8_t *payload, size_t length) {
  String msg = String((char*)payload).substring(0, length);
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, msg);
  if (!err) {
    Kp_roll_rate = doc["Kp_roll_rate"];
    Ki_roll_rate = doc["Ki_roll_rate"];
    Kd_roll_rate = doc["Kd_roll_rate"];
    Kp_roll_angle = doc["Kp_roll_angle"];
    Ki_roll_angle = doc["Ki_roll_angle"];
    Kd_roll_angle = doc["Kd_roll_angle"];
    Kp_pitch_rate = doc["Kp_pitch_rate"];
    Ki_pitch_rate = doc["Ki_pitch_rate"];
    Kd_pitch_rate = doc["Kd_pitch_rate"];
    Kp_pitch_angle = doc["Kp_pitch_angle"];
    Ki_pitch_angle = doc["Ki_pitch_angle"];
    Kd_pitch_angle = doc["Kd_pitch_angle"];
    Serial.println("PID values updated from webpage!");
  }
}

void sendTelemetry() {
  static unsigned long last = 0;
  if (millis() - last >= 100) {
    last = millis();
    float t = millis()/1000.0;
    // Simulated telemetry values
    roll_pid_rate = 0.3 * sin(t);
    rollRate = 0.3 * sin(t + 0.1);
    pitch_pid_rate = 0.25 * cos(t);
    pitchRate = 0.25 * cos(t + 0.1);
    rollInput = 10 * sin(t/2);
    roll = 10 * sin(t/2 + 0.05);
    pitchInput = 8 * cos(t/2);
    pitch = 8 * cos(t/2 + 0.05);
    duty1 = 1200 + 200*sin(t);
    duty2 = 1200 + 200*cos(t);
    duty3 = 1200 + 200*sin(t + 1);
    duty4 = 1200 + 200*cos(t + 1);
    actual_rate_loop_time = 0.006 + 0.001 * sin(t);  // Simulated around 0.006
    actual_angle_loop_time = 0.008 + 0.001 * cos(t); // Simulated around 0.008

    DynamicJsonDocument doc(1024);
    doc["time"] = t;
    doc["roll_pid_rate"] = roll_pid_rate;
    doc["rollRate"] = rollRate;
    doc["pitch_pid_rate"] = pitch_pid_rate;
    doc["pitchRate"] = pitchRate;
    doc["rollInput"] = rollInput;
    doc["roll"] = roll;
    doc["pitchInput"] = pitchInput;
    doc["pitch"] = pitch;
    doc["duty1"] = duty1;
    doc["duty2"] = duty2;
    doc["duty3"] = duty3;
    doc["duty4"] = duty4;
    doc["actual_rate_loop_time"] = actual_rate_loop_time;
    doc["actual_angle_loop_time"] = actual_angle_loop_time;

    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
  }
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) handleWebSocketMessage(num, payload, length);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nConnected to WiFi!");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();
  sendTelemetry();
}
