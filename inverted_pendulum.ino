#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>

// --- PINAGEM HARDWARE ---
const int PIN_STEP = 25;
const int PIN_DIR = 26;
const int PIN_ENABLE = 27;

const int PIN_MS1 = 14;
const int PIN_MS2 = 12;
const int PIN_MS3 = 13;

const int PIN_ENC_A = 32;
const int PIN_ENC_B = 33;

// --- CONFIGURAÇÃO DA REDE (MODO AP) ---
const char* ssid = "Pendulo_ESP32";
const char* password = "12345678";

WebServer server(80);

// --- CONSTANTES DO SISTEMA ---
const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEP_DIV = 32;
const long ENCODER_COUNTS_PER_REV = 2400L; 

const float MOTOR_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEP_DIV) / 360.0f;
const float SETPOINT_DEG = 180.0f;
const float GUARD_HALF_WIDTH = 80.0f;
const float MAX_STEPPER_ABS_DEG = 360.0f;
const float MAX_CONTROL_SPEED_DEG_S = 2500.0f;
const float INTEGRAL_CLAMP = 2000.0f;

const float MAX_CENTERING_SPEED_DEG_S = 120.0f;
const float CENTERING_FILTER_ALPHA = 0.15f;

const unsigned long CONTROL_PERIOD_US = 2000UL; // 500 Hz (dt = 0.002s)

float DERIVATIVE_FILTER_ALPHA = 0.82f;

float Kp = 0.009f;
float Ki = 3.05f;
float Kd = 0.002f;
float Kx = 0.00f;
float Bias = 0.00f;

volatile long encoderCount = 0; 
long encoderOffset = 0;
int encoderDir = 1;
int motorDir = 1;

bool calibrated = false;
bool controlEnabled = false;

unsigned long totalControlSamples = 0;

float integral = 0.0f;
float previousAngle = 0.0f;
float filteredVelocity = 0.0f;
float currentError = 0.0f;
float currentRefDeg = SETPOINT_DEG;
float centeringSpeedDegPerSecond = 0.0f;

unsigned long lastControlUs = 0;
String serialBuffer;

AccelStepper motor(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile float target_motor_speed = 0.0f;
volatile bool flag_reset_motor_pos = false;

struct LogPoint {
  float time_s;
  float angle_deg;
  float error_deg;
  float omega_deg_s;
  float speed_ps;
};

const int LOG_SIZE = 2500;
LogPoint logBuffer[LOG_SIZE];
volatile int logIndex = 0;
volatile bool isLoggingRAM = false;
volatile bool logFinished = false;
unsigned long logStartUs = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Inverted Pendulum — High-Precision Telemetry</title>
  <style>
    :root {
      --bg: #0d1017; --panel: #141922; --border: rgba(255,255,255,.1);
      --text: #e8ecf2; --muted: #7d8798; --cyan: #35c4dd; --red: #e0655f;
      --green: #4cc07a; --amber: #d9a94a; --accent: #4a7fc4;
      --mono: "JetBrains Mono", Consolas, monospace;
    }
    * { box-sizing: border-box; }
    body { margin: 0; padding: 16px; font-family: system-ui, sans-serif; color: var(--text); background: var(--bg); }
    .dashboard { width: min(1200px, 98vw); margin: 0 auto; }
    .topbar { display: flex; justify-content: space-between; border-bottom: 1px solid var(--border); padding-bottom: 8px; margin-bottom: 14px; }
    .topbar h1 { font-size: 16px; margin: 0; text-transform: uppercase; }
    .kpis { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 14px; }
    .kpi { background: var(--panel); border: 1px solid var(--border); padding: 10px; border-radius: 4px; }
    .kpi-label { font-size: 10px; color: var(--muted); text-transform: uppercase; }
    .kpi-value { font-family: var(--mono); font-size: 20px; font-weight: bold; margin-top: 4px; }
    .control-panel { background: var(--panel); border: 1px solid var(--border); padding: 14px; border-radius: 4px; margin-bottom: 14px; display: flex; flex-direction: column; gap: 10px; }
    .control-row { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
    .control-group { display: flex; align-items: center; gap: 6px; }
    .control-group label { font-size: 12px; color: var(--muted); font-family: var(--mono); }
    .control-group input { background: #0a0d12; border: 1px solid var(--border); color: var(--text); padding: 5px; border-radius: 2px; width: 75px; font-family: var(--mono); }
    .btn { color: #fff; border: 1px solid rgba(255,255,255,.15); padding: 7px 14px; border-radius: 3px; font-weight: 600; cursor: pointer; font-size: 12px; font-family: var(--mono); background: #232a36; }
    .btn:hover { filter: brightness(1.2); }
    .btn-start { border-left: 3px solid #059669; }
    .btn-stop { border-left: 3px solid #dc2626; }
    .btn-ram { border-left: 3px solid #f59e0b; background: #3c2408; }
    .btn-dl { border-left: 3px solid #3b82f6; background: #11284d; }
    .status-msg { font-size: 11px; color: var(--green); font-family: var(--mono); }
    .log-box { background: #000; border: 1px solid var(--border); padding: 10px; font-family: var(--mono); font-size: 11px; color: var(--amber); border-radius: 4px; margin-top: 10px; }
  </style>
</head>
<body>
  <div class="dashboard">
    <div class="topbar">
      <h1>Rotary Inverted Pendulum — Telemetry System</h1>
      <div>REF: <span id="refVal">180°</span></div>
    </div>

    <div class="kpis">
      <div class="kpi"><div class="kpi-label">Angle (&theta;)</div><div class="kpi-value" id="angleVal" style="color:var(--cyan);">---</div></div>
      <div class="kpi"><div class="kpi-label">Error (e)</div><div class="kpi-value" id="errorVal" style="color:var(--red);">---</div></div>
      <div class="kpi"><div class="kpi-label">Angular Vel. (&omega;)</div><div class="kpi-value" id="omegaVal" style="color:var(--amber);">---</div></div>
      <div class="kpi"><div class="kpi-label">Motor Speed</div><div class="kpi-value" id="speedVal" style="color:var(--green);">---</div></div>
    </div>

    <div class="control-panel">
      <div class="control-row">
        <button class="btn" onclick="sendCmd('0')">Zero Down (0)</button>
        <button class="btn" onclick="sendCmd('1')">Set Up 180 (1)</button>
        <button class="btn btn-start" onclick="sendCmd('S')">START (S)</button>
        <button class="btn btn-stop" onclick="sendCmd('P')">STOP (P)</button>
        <button class="btn btn-ram" onclick="startRAMLog()">Log 5s to RAM (500Hz)</button>
        <button class="btn btn-dl" onclick="downloadCSV()">Download Raw CSV</button>
      </div>

      <div class="control-row">
        <div class="control-group"><label>Kp:</label><input type="number" id="inputKp" step="0.001"></div>
        <div class="control-group"><label>Ki:</label><input type="number" id="inputKi" step="0.01"></div>
        <div class="control-group"><label>Kd:</label><input type="number" id="inputKd" step="0.0001"></div>
        <div class="control-group"><label>Kx:</label><input type="number" id="inputKx" step="0.01"></div>
        <div class="control-group"><label>&alpha;:</label><input type="number" id="inputAlpha" step="0.01"></div>
        <button class="btn" onclick="updateParams()">Apply PID</button>
        <span id="statusMsg" class="status-msg"></span>
      </div>
    </div>

    <div class="log-box" id="logStatus">
      Datalogger Status: Waiting for RAM recording command...
    </div>
  </div>

  <script>
    let initialSyncDone = false;

    async function fetchData() {
      try {
        const res = await fetch("/data");
        const data = await res.json();

        if (!initialSyncDone) {
          document.getElementById("inputKp").value = data.kp;
          document.getElementById("inputKi").value = data.ki;
          document.getElementById("inputKd").value = data.kd;
          document.getElementById("inputKx").value = data.kx;
          document.getElementById("inputAlpha").value = data.alpha;
          initialSyncDone = true;
        }

        document.getElementById("angleVal").innerText = data.angle.toFixed(2) + "°";
        document.getElementById("errorVal").innerText = data.error.toFixed(2) + "°";
        document.getElementById("omegaVal").innerText = data.omega.toFixed(1) + "°/s";
        document.getElementById("speedVal").innerText = data.speed.toFixed(0) + " p/s";
        document.getElementById("refVal").innerText = data.ref.toFixed(2) + "°";

        if (data.ramLogging) {
          document.getElementById("logStatus").innerText = `[RECORDING] Samples: ${data.ramIndex} / 2500 (${(data.ramIndex/25).toFixed(0)}%)`;
        } else if (data.ramFinished) {
          document.getElementById("logStatus").innerText = "[COMPLETED] 2500 samples recorded in RAM at 500Hz! Ready to download.";
        }
      } catch (e) {}
    }
    setInterval(fetchData, 150);

    async function startRAMLog() {
      await fetch("/start_ram_log");
      document.getElementById("logStatus").innerText = "Starting high-speed data acquisition...";
    }

    function downloadCSV() {
      window.location.href = "/download_csv";
    }

    async function updateParams() {
      const kp = document.getElementById("inputKp").value;
      const ki = document.getElementById("inputKi").value;
      const kd = document.getElementById("inputKd").value;
      const kx = document.getElementById("inputKx").value;
      const alpha = document.getElementById("inputAlpha").value;
      await fetch(`/set_params?kp=${kp}&ki=${ki}&kd=${kd}&kx=${kx}&alpha=${alpha}`);
    }

    async function sendCmd(cmd) {
      await fetch(`/cmd?c=${cmd}`);
    }
  </script>
</body>
</html>
)rawliteral";

long roundLong(float value) {
  return (value >= 0.0f) ? (long)(value + 0.5f) : (long)(value - 0.5f);
}

long degToSteps(float deg) { return roundLong(deg * MOTOR_STEPS_PER_DEG); }
float stepsToDeg(long steps) { return (float)steps / MOTOR_STEPS_PER_DEG; }

float wrap360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

float shortestAngleError(float setpoint, float position) {
  float error = setpoint - position;
  while (error > 180.0f) error -= 360.0f;
  while (error <= -180.0f) error += 360.0f;
  return error;
}

long getEncoderCount() {
  long value;
  portENTER_CRITICAL(&timerMux);
  value = encoderCount;
  portEXIT_CRITICAL(&timerMux);
  return value;
}

float getEncoderAngle() {
  long raw = getEncoderCount();
  long relative = raw - encoderOffset;
  float angle = ((float)relative * 360.0f) / (float)ENCODER_COUNTS_PER_REV;
  angle *= (float)encoderDir;
  return wrap360(angle);
}

float getMotorAngle() {
  return stepsToDeg(motor.currentPosition()) * (float)motorDir;
}

float computeCenteringSpeed(float motorAngleDeg) {
  float targetSpeed = -Kx * motorAngleDeg;
  if (targetSpeed > MAX_CENTERING_SPEED_DEG_S) targetSpeed = MAX_CENTERING_SPEED_DEG_S;
  if (targetSpeed < -MAX_CENTERING_SPEED_DEG_S) targetSpeed = -MAX_CENTERING_SPEED_DEG_S;

  centeringSpeedDegPerSecond = (CENTERING_FILTER_ALPHA * targetSpeed) +
                              ((1.0f - CENTERING_FILTER_ALPHA) * centeringSpeedDegPerSecond);
  return centeringSpeedDegPerSecond;
}

void handleData() {
  String json;
  json.reserve(300);
  json = "{";
  json += "\"angle\":" + String(getEncoderAngle(), 2) + ",";
  json += "\"error\":" + String(currentError, 2) + ",";
  json += "\"omega\":" + String(filteredVelocity, 2) + ",";
  json += "\"speed\":" + String(target_motor_speed, 2) + ",";
  json += "\"ref\":" + String(currentRefDeg, 2) + ",";
  json += "\"kp\":" + String(Kp, 5) + ",";
  json += "\"ki\":" + String(Ki, 5) + ",";
  json += "\"kd\":" + String(Kd, 5) + ",";
  json += "\"kx\":" + String(Kx, 5) + ",";
  json += "\"alpha\":" + String(DERIVATIVE_FILTER_ALPHA, 5) + ",";
  json += "\"ramLogging\":" + String(isLoggingRAM ? "true" : "false") + ",";
  json += "\"ramFinished\":" + String(logFinished ? "true" : "false") + ",";
  json += "\"ramIndex\":" + String(logIndex);
  json += "}";
  server.send(200, "application/json", json);
}

void handleStartRAMLog() {
  logIndex = 0;
  logFinished = false;
  logStartUs = micros();
  isLoggingRAM = true;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleDownloadCSV() {
  if (logIndex == 0) {
    server.send(400, "text/plain", "Nenhum dado gravado na RAM ainda.");
    return;
  }
  
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("Time_s,Angle_deg,Error_deg,Omega_deg_s,Speed_ps\n");

  String chunk = "";
  chunk.reserve(1024);
  
  for (int i = 0; i < logIndex; i++) {
    chunk += String(logBuffer[i].time_s, 4) + "," +
             String(logBuffer[i].angle_deg, 3) + "," +
             String(logBuffer[i].error_deg, 3) + "," +
             String(logBuffer[i].omega_deg_s, 2) + "," +
             String(logBuffer[i].speed_ps, 1) + "\n";
             
    if (chunk.length() > 800) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length() > 0) {
    server.sendContent(chunk);
  }
  server.sendContent(""); // Finaliza o chunk
}

void handleSetParams() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("kx")) Kx = server.arg("kx").toFloat();
  if (server.hasArg("alpha")) {
    float newAlpha = server.arg("alpha").toFloat();
    if (newAlpha >= 0.0f && newAlpha < 1.0f) DERIVATIVE_FILTER_ALPHA = newAlpha;
  }
  integral = 0.0f;
  centeringSpeedDegPerSecond = 0.0f;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void processCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd == "0") { setZeroDown(); return; }
  if (cmd == "1") { setUp180(); return; }
  if (cmd == "S") { startControl(); return; }
  if (cmd == "P") { stopControl(); return; }
}

void handleCommandWeb() {
  if (server.hasArg("c")) processCommand(server.arg("c"));
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void webServerTask(void * pvParameters) {
  server.on("/", []() { server.send_P(200, "text/html", index_html); });
  server.on("/data", handleData);
  server.on("/start_ram_log", handleStartRAMLog);
  server.on("/download_csv", handleDownloadCSV);
  server.on("/set_params", handleSetParams);
  server.on("/cmd", handleCommandWeb);
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// --- INTERRUPÇÕES DE ENCODER ---
void IRAM_ATTR encoderA_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B)) encoderCount++;
  else encoderCount--;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR encoderB_ISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B)) encoderCount++;
  else encoderCount--;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void enableMotor() { digitalWrite(PIN_ENABLE, LOW); }
void disableMotor() { digitalWrite(PIN_ENABLE, HIGH); }

void resetControllerState(float angle) {
  integral = 0.0f;
  previousAngle = angle;
  filteredVelocity = 0.0f;
  centeringSpeedDegPerSecond = 0.0f;
  lastControlUs = micros();
}

void stopControl() {
  controlEnabled = false;
  target_motor_speed = 0.0f;
  resetControllerState(getEncoderAngle());
  disableMotor();
}

void startControl() {
  if (!calibrated) return;
  float angle = getEncoderAngle();
  float error = shortestAngleError(SETPOINT_DEG, angle);
  if (fabsf(error) > GUARD_HALF_WIDTH) return;
  
  target_motor_speed = 0.0f;
  flag_reset_motor_pos = true;
  totalControlSamples = 0;
  currentRefDeg = SETPOINT_DEG;

  resetControllerState(angle);
  enableMotor();
  controlEnabled = true;
}

void setZeroDown() {
  stopControl();
  encoderOffset = getEncoderCount();
  flag_reset_motor_pos = true;
  currentRefDeg = SETPOINT_DEG;
  calibrated = true;
}

void setUp180() {
  stopControl();
  encoderOffset = getEncoderCount() - (long)(1200L * encoderDir);
  flag_reset_motor_pos = true;
  currentRefDeg = SETPOINT_DEG;
  calibrated = true;
}

void controlLoop() {
  unsigned long now = micros();
  if ((unsigned long)(now - lastControlUs) < CONTROL_PERIOD_US) return;

  float dt = (float)(now - lastControlUs) / 1000000.0f;
  lastControlUs = now;
  if (dt <= 0.0f) return;

  totalControlSamples++;

  float angle = getEncoderAngle();
  float motorAngle = getMotorAngle();

  float guardError = shortestAngleError(SETPOINT_DEG, angle);
  if (fabsf(guardError) > GUARD_HALF_WIDTH) { stopControl(); return; }

  currentRefDeg = SETPOINT_DEG;
  currentError = shortestAngleError(currentRefDeg, angle);

  float deltaAngle = shortestAngleError(angle, previousAngle);
  float measuredVelocity = deltaAngle / dt;
  filteredVelocity = (DERIVATIVE_FILTER_ALPHA * filteredVelocity) +
                     ((1.0f - DERIVATIVE_FILTER_ALPHA) * measuredVelocity);
  previousAngle = angle;

  float proportional = Kp * currentError;
  float derivative = -Kd * filteredVelocity;
  float integralTerm = Ki * integral;

  float pidOutputDeg = proportional + integralTerm + derivative + Bias;
  float pidOutputLimit = MAX_CONTROL_SPEED_DEG_S * dt;
  if (pidOutputLimit < 0.001f) pidOutputLimit = 0.001f;

  if (pidOutputDeg > pidOutputLimit) pidOutputDeg = pidOutputLimit;
  if (pidOutputDeg < -pidOutputLimit) pidOutputDeg = -pidOutputLimit;

  bool saturatedHigh = (pidOutputDeg >= pidOutputLimit);
  bool saturatedLow = (pidOutputDeg <= -pidOutputLimit);

  bool integralAllowed = (!saturatedHigh && !saturatedLow) ||
                          (saturatedHigh && currentError < 0.0f) ||
                          (saturatedLow && currentError > 0.0f);

  if (integralAllowed) {
    integral += currentError * dt;
    if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
    if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;
  }

  pidOutputDeg = (Kp * currentError) + (Ki * integral) - (Kd * filteredVelocity) + Bias;
  float pidSpeedDegPerSecond = pidOutputDeg / dt;

  if (pidSpeedDegPerSecond > MAX_CONTROL_SPEED_DEG_S) pidSpeedDegPerSecond = MAX_CONTROL_SPEED_DEG_S;
  if (pidSpeedDegPerSecond < -MAX_CONTROL_SPEED_DEG_S) pidSpeedDegPerSecond = -MAX_CONTROL_SPEED_DEG_S;

  float centeringSpeed = computeCenteringSpeed(motorAngle);
  float speedDegPerSecond = pidSpeedDegPerSecond + centeringSpeed;

  if (speedDegPerSecond > MAX_CONTROL_SPEED_DEG_S) speedDegPerSecond = MAX_CONTROL_SPEED_DEG_S;
  if (speedDegPerSecond < -MAX_CONTROL_SPEED_DEG_S) speedDegPerSecond = -MAX_CONTROL_SPEED_DEG_S;

  speedDegPerSecond *= (float)motorDir;

  if (MAX_STEPPER_ABS_DEG > 0.0f && fabsf(motorAngle) >= MAX_STEPPER_ABS_DEG) {
    stopControl();
    return;
  }

  target_motor_speed = speedDegPerSecond * MOTOR_STEPS_PER_DEG;

  if (isLoggingRAM && logIndex < LOG_SIZE) {
    logBuffer[logIndex].time_s = (float)(now - logStartUs) / 1000000.0f;
    logBuffer[logIndex].angle_deg = angle;
    logBuffer[logIndex].error_deg = currentError;
    logBuffer[logIndex].omega_deg_s = filteredVelocity;
    logBuffer[logIndex].speed_ps = target_motor_speed;
    logIndex++;
    if (logIndex >= LOG_SIZE) {
      isLoggingRAM = false;
      logFinished = true;
    }
  }
}

void controlTask(void * pvParameters) {
  for (;;) {
    if (controlEnabled) controlLoop();
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);
  disableLoopWDT();

  WiFi.softAP(ssid, password); 

  xTaskCreatePinnedToCore(webServerTask, "WebServer", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "ControlLoop", 8192, NULL, 2, NULL, 0);

  pinMode(PIN_MS1, OUTPUT); pinMode(PIN_MS2, OUTPUT); pinMode(PIN_MS3, OUTPUT);
  digitalWrite(PIN_MS1, HIGH); digitalWrite(PIN_MS2, HIGH); digitalWrite(PIN_MS3, HIGH);

  pinMode(PIN_ENABLE, OUTPUT); disableMotor();
  pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderB_ISR, CHANGE);

  motor.setMaxSpeed(30000);
  motor.setMinPulseWidth(2);
  motor.setCurrentPosition(0);
  motor.setSpeed(0);

  lastControlUs = micros();
}

void loop() {
  if (flag_reset_motor_pos) {
    motor.setCurrentPosition(0);
    flag_reset_motor_pos = false;
  }
  
  static float last_speed = 0.0f;
  if (last_speed != target_motor_speed) {
    last_speed = target_motor_speed;
    motor.setSpeed(last_speed);
  }

  motor.runSpeed();
}