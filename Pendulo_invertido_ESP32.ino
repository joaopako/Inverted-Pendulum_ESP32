#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>

const int PIN_STEP = 25;
const int PIN_DIR = 26;
const int PIN_ENABLE = 27;

const char* ssid = "Eventos2023";
const char* password = "cmpf2024@ufersa";

WebServer server(80);

const int PIN_MS1 = 14;
const int PIN_MS2 = 12;
const int PIN_MS3 = 13;

const int PIN_ENC_A = 32;
const int PIN_ENC_B = 33;

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

const unsigned long CONTROL_PERIOD_US = 2000UL;
const unsigned long STATUS_PERIOD_US = 100000UL;

float DERIVATIVE_FILTER_ALPHA = 0.82f;

float Kp = 0.009f;
float Ki = 3.00f;
float Kd = 0.002f;
float Kx = 0.05f;
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
unsigned long lastStatusUs = 0;

String serialBuffer;

AccelStepper motor(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile float target_motor_speed = 0.0f;
volatile bool flag_reset_motor_pos = false;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Pêndulo Invertido - Dashboard</title>
  <style>
    :root {
      --bg: #0d1017;
      --panel: #141922;
      --panel2: #171d27;
      --border: rgba(255,255,255,.07);
      --border2: rgba(255,255,255,.12);
      --text: #e8ecf2;
      --muted: #7d8798;
      --faint: #4a5261;
      --cyan: #35c4dd;
      --red: #e0655f;
      --green: #4cc07a;
      --amber: #d9a94a;
      --accent: #4a7fc4;
      --mono: "JetBrains Mono", "SFMono-Regular", Consolas, "Liberation Mono", monospace;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0; padding: 18px 16px 28px;
      font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
      color: var(--text); background: var(--bg);
      background-image:
        linear-gradient(rgba(255,255,255,.015) 1px, transparent 1px),
        linear-gradient(90deg, rgba(255,255,255,.015) 1px, transparent 1px);
      background-size: 28px 28px;
    }
    .dashboard { width: min(1400px, 98vw); margin: 0 auto; }

    .topbar {
      display: flex; align-items: baseline; justify-content: space-between;
      border-bottom: 1px solid var(--border2);
      padding-bottom: 8px; margin-bottom: 14px;
    }
    .topbar h1 { font-size: 15px; margin: 0; font-weight: 700; letter-spacing: .06em; text-transform: uppercase; }
    .topbar h1 .dot {
      display: inline-block; width: 8px; height: 8px; border-radius: 50%;
      background: var(--green); margin-right: 8px;
      box-shadow: 0 0 6px var(--green);
      animation: blink 2.4s infinite;
    }
    @keyframes blink { 0%,100%{opacity:1} 50%{opacity:.35} }
    .topbar .meta { font-family: var(--mono); font-size: 10px; color: var(--faint); letter-spacing: .05em; }

    .kpis { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-bottom: 14px; }
    .kpi {
      background: var(--panel); border: 1px solid var(--border);
      padding: 10px 14px 11px; border-radius: 3px;
      border-top: 2px solid var(--kcolor, var(--border2));
      position: relative;
    }
    .kpi::after {
      content: ""; position: absolute; left: 0; top: 0; bottom: 0; width: 3px;
      background: var(--kcolor, transparent); opacity: .55;
    }
    .kpi-label { font-size: 10px; color: var(--muted); text-transform: uppercase; font-weight: 600; letter-spacing: .1em; }
    .kpi-value {
      font-family: var(--mono); font-size: 21px; font-weight: 600; margin-top: 5px;
      letter-spacing: -.02em; font-variant-numeric: tabular-nums;
    }

    .control-panel {
      background: var(--panel); border: 1px solid var(--border);
      padding: 14px; margin-bottom: 14px; display: flex; flex-direction: column; gap: 13px;
    }
    .control-row { display: flex; flex-wrap: wrap; align-items: center; gap: 14px; }
    .control-group { display: flex; align-items: center; gap: 7px; }
    .control-group label { font-size: 12px; font-weight: 600; color: var(--muted); font-family: var(--mono); }
    .control-group input {
      background: #0a0d12; border: 1px solid var(--border2); color: var(--text);
      padding: 6px 9px; border-radius: 2px; width: 82px; font-size: 12px; font-weight: 600;
      font-family: var(--mono); font-variant-numeric: tabular-nums;
    }
    .control-group input:focus { outline: none; border-color: var(--accent); }
    .btn {
      color: #e8ecf2; border: 1px solid rgba(255,255,255,.12);
      padding: 7px 16px; border-radius: 2px; font-weight: 600; cursor: pointer;
      transition: filter .15s, background .2s; font-size: 12px; font-family: var(--mono);
      background: #232a36; letter-spacing: .03em;
    }
    .btn:hover { filter: brightness(1.25); }
    .btn:active { transform: translateY(1px); }
    .btn-zero   { border-left: 3px solid #6b7280; }
    .btn-setup  { border-left: 3px solid #d97706; }
    .btn-start  { border-left: 3px solid #059669; }
    .btn-stop   { border-left: 3px solid #dc2626; }
    .btn-invert { border-left: 3px solid #8b5cf6; }
    .btn-invert.active { background: #5b21b6; border-left-color: #a78bfa; }
    .btn-apply  { border-left: 3px solid var(--accent); background: #1d2c42; }
    .status-msg { font-size: 11px; color: var(--green); margin-left: 8px; opacity: 0; transition: opacity 0.3s; font-family: var(--mono); }

    .sample-info { font-size: 12px; color: var(--muted); margin-bottom: 12px; text-align: right; font-family: var(--mono); }

    .charts-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 14px; }
    .panel {
      background: var(--panel); border: 1px solid var(--border); border-radius: 3px;
      padding: 10px 12px 8px;
    }
    .panel-header { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 6px; }
    .panel-title { font-size: 11px; font-weight: 700; text-transform: uppercase; letter-spacing: .08em; color: var(--muted); }
    .panel-title::before { content: "▮ "; font-size: 9px; color: var(--pcolor, var(--muted)); }
    .legend-now {
      font-family: var(--mono); font-size: 11px; font-weight: 600; color: var(--pcolor, var(--text));
      font-variant-numeric: tabular-nums;
    }

    .chart-wrap { display: flex; }
    .y-axis {
      width: 52px; flex-shrink: 0; height: 190px; position: relative;
      font-size: 9px; color: var(--faint); font-family: var(--mono);
    }
    .y-axis span { position: absolute; right: 8px; transform: translateY(-50%); font-variant-numeric: tabular-nums; }
    .plot-area { position: relative; flex: 1; height: 190px; }
    svg { width: 100%; height: 100%; display: block; background: #0a0d13; border: 1px solid var(--border); border-radius: 2px; cursor: crosshair; }

    .x-axis {
      display: flex; justify-content: space-between; margin: 3px 0 0 52px;
      font-size: 9px; color: var(--faint); font-family: var(--mono);
    }

    path.smooth-line {
      fill: none; stroke-width: 1.6; vector-effect: non-scaling-stroke;
      stroke-linejoin: round; stroke-linecap: round;
    }
    path.area-fill { stroke: none; opacity: .14; }

    .tooltip {
      position: absolute; pointer-events: none; opacity: 0;
      background: #1a2130; border: 1px solid var(--border2); border-radius: 2px;
      padding: 3px 8px; font-family: var(--mono); font-size: 10px; white-space: nowrap;
      transform: translate(-50%, -130%); transition: opacity .1s; z-index: 5;
      font-variant-numeric: tabular-nums;
    }

    .footer {
      text-align: center; font-family: var(--mono); font-size: 9px; color: var(--faint);
      letter-spacing: .12em; padding-top: 6px; border-top: 1px solid var(--border);
    }

    @media (max-width: 900px) {
      .charts-grid { grid-template-columns: 1fr; }
      .kpis { grid-template-columns: repeat(2, 1fr); }
    }
  </style>
</head>
<body>
  <div class="dashboard">

    <div class="topbar">
      <h1><span class="dot"></span>Pêndulo Invertido</h1>
      <div class="meta">REF: <span id="refVal">180°</span></div>
    </div>

    <div class="kpis">
      <div class="kpi" style="--kcolor: var(--cyan);">
        <div class="kpi-label">Ângulo</div>
        <div class="kpi-value" id="angleVal" style="color: var(--cyan);">---</div>
      </div>
      <div class="kpi" style="--kcolor: var(--red);">
        <div class="kpi-label">Erro</div>
        <div class="kpi-value" id="errorVal" style="color: var(--red);">---</div>
      </div>
      <div class="kpi" style="--kcolor: var(--amber);">
        <div class="kpi-label">Vel. Haste (&omega;)</div>
        <div class="kpi-value" id="omegaVal" style="color: var(--amber);">---</div>
      </div>
      <div class="kpi" style="--kcolor: var(--green);">
        <div class="kpi-label">Velocidade Motor</div>
        <div class="kpi-value" id="speedVal" style="color: var(--green);">---</div>
      </div>
    </div>

    <div class="control-panel">
      <div class="control-row">
        <button class="btn btn-zero" onclick="sendCmd('0')">Zero Down (0)</button>
        <button class="btn btn-setup" onclick="sendCmd('1')">Set Up 180 (1)</button>
        <button class="btn btn-start" onclick="sendCmd('S')">START (S)</button>
        <button class="btn btn-stop" onclick="sendCmd('P')">STOP (P)</button>
        <button class="btn btn-invert" id="btnEncDir" onclick="toggleEncDir()">ENC: DIR 1</button>
        <button class="btn btn-invert" id="btnMotDir" onclick="toggleMotDir()">MOT: DIR 1</button>
      </div>
      <div class="control-row">
        <div class="control-group">
          <label>Kp:</label>
          <input type="number" id="inputKp" step="0.001">
        </div>
        <div class="control-group">
          <label>Ki:</label>
          <input type="number" id="inputKi" step="0.01">
        </div>
        <div class="control-group">
          <label>Kd:</label>
          <input type="number" id="inputKd" step="0.0001">
        </div>
        <div class="control-group">
          <label>Kx:</label>
          <input type="number" id="inputKx" step="0.01" min="0">
        </div>
        <div class="control-group">
          <label>&alpha;:</label>
          <input type="number" id="inputAlpha" step="0.01" min="0" max="0.99">
        </div>
        <button class="btn btn-apply" onclick="updateParams()">Aplicar PID</button>
        <span id="statusMsg" class="status-msg">ok ✓</span>
      </div>
    </div>

    <div class="sample-info">
      amostras pós-START: <strong id="sampleCount" style="color: var(--cyan);">0</strong>
    </div>

    <div class="charts-grid">

      <div class="panel" style="--pcolor: var(--cyan);">
        <div class="panel-header">
          <div class="panel-title">Ângulo da Haste</div>
          <span class="legend-now" id="nowAngle">---</span>
        </div>
        <div class="chart-wrap">
          <div class="y-axis" id="yAxisAngle"></div>
          <div class="plot-area">
            <svg id="svgAngle" viewBox="0 0 500 190" preserveAspectRatio="none">
              <defs>
                <linearGradient id="gradAngle" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stop-color="var(--cyan)" stop-opacity=".35"/>
                  <stop offset="100%" stop-color="var(--cyan)" stop-opacity="0"/>
                </linearGradient>
              </defs>
              <g id="gridAngle"></g>
              <path id="fillAngle" class="area-fill" fill="url(#gradAngle)"/>
              <path id="pathAngle" class="smooth-line" stroke="var(--cyan)"/>
              <circle id="endAngle" r="3" fill="var(--cyan)" stroke="#0a0d13" stroke-width="1.5" style="display:none;"/>
              <circle id="minAngle" r="2.5" fill="none" stroke="var(--cyan)" stroke-width="1.2" style="display:none;"/>
              <circle id="maxAngle" r="2.5" fill="none" stroke="var(--cyan)" stroke-width="1.2" style="display:none;"/>
            </svg>
            <div class="tooltip" id="tipAngle"></div>
          </div>
        </div>
        <div class="x-axis"><span>-12 s</span><span>-9 s</span><span>-6 s</span><span>-3 s</span><span>agora</span></div>
      </div>

      <div class="panel" style="--pcolor: var(--red);">
        <div class="panel-header">
          <div class="panel-title">Erro de Posição</div>
          <span class="legend-now" id="nowError">---</span>
        </div>
        <div class="chart-wrap">
          <div class="y-axis" id="yAxisError"></div>
          <div class="plot-area">
            <svg id="svgError" viewBox="0 0 500 190" preserveAspectRatio="none">
              <defs>
                <linearGradient id="gradError" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stop-color="var(--red)" stop-opacity=".35"/>
                  <stop offset="100%" stop-color="var(--red)" stop-opacity="0"/>
                </linearGradient>
              </defs>
              <g id="gridError"></g>
              <path id="fillError" class="area-fill" fill="url(#gradError)"/>
              <path id="pathError" class="smooth-line" stroke="var(--red)"/>
              <circle id="endError" r="3" fill="var(--red)" stroke="#0a0d13" stroke-width="1.5" style="display:none;"/>
              <circle id="minError" r="2.5" fill="none" stroke="var(--red)" stroke-width="1.2" style="display:none;"/>
              <circle id="maxError" r="2.5" fill="none" stroke="var(--red)" stroke-width="1.2" style="display:none;"/>
            </svg>
            <div class="tooltip" id="tipError"></div>
          </div>
        </div>
        <div class="x-axis"><span>-12 s</span><span>-9 s</span><span>-6 s</span><span>-3 s</span><span>agora</span></div>
      </div>

      <div class="panel" style="--pcolor: var(--amber);">
        <div class="panel-header">
          <div class="panel-title">Velocidade Angular da Haste</div>
          <span class="legend-now" id="nowOmega">---</span>
        </div>
        <div class="chart-wrap">
          <div class="y-axis" id="yAxisOmega"></div>
          <div class="plot-area">
            <svg id="svgOmega" viewBox="0 0 500 190" preserveAspectRatio="none">
              <defs>
                <linearGradient id="gradOmega" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stop-color="var(--amber)" stop-opacity=".35"/>
                  <stop offset="100%" stop-color="var(--amber)" stop-opacity="0"/>
                </linearGradient>
              </defs>
              <g id="gridOmega"></g>
              <path id="fillOmega" class="area-fill" fill="url(#gradOmega)"/>
              <path id="pathOmega" class="smooth-line" stroke="var(--amber)"/>
              <circle id="endOmega" r="3" fill="var(--amber)" stroke="#0a0d13" stroke-width="1.5" style="display:none;"/>
              <circle id="minOmega" r="2.5" fill="none" stroke="var(--amber)" stroke-width="1.2" style="display:none;"/>
              <circle id="maxOmega" r="2.5" fill="none" stroke="var(--amber)" stroke-width="1.2" style="display:none;"/>
            </svg>
            <div class="tooltip" id="tipOmega"></div>
          </div>
        </div>
        <div class="x-axis"><span>-12 s</span><span>-9 s</span><span>-6 s</span><span>-3 s</span><span>agora</span></div>
      </div>

      <div class="panel" style="--pcolor: var(--green);">
        <div class="panel-header">
          <div class="panel-title">Velocidade do Motor</div>
          <span class="legend-now" id="nowSpeed">---</span>
        </div>
        <div class="chart-wrap">
          <div class="y-axis" id="yAxisSpeed"></div>
          <div class="plot-area">
            <svg id="svgSpeed" viewBox="0 0 500 190" preserveAspectRatio="none">
              <defs>
                <linearGradient id="gradSpeed" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stop-color="var(--green)" stop-opacity=".35"/>
                  <stop offset="100%" stop-color="var(--green)" stop-opacity="0"/>
                </linearGradient>
              </defs>
              <g id="gridSpeed"></g>
              <path id="fillSpeed" class="area-fill" fill="url(#gradSpeed)"/>
              <path id="pathSpeed" class="smooth-line" stroke="var(--green)"/>
              <circle id="endSpeed" r="3" fill="var(--green)" stroke="#0a0d13" stroke-width="1.5" style="display:none;"/>
              <circle id="minSpeed" r="2.5" fill="none" stroke="var(--green)" stroke-width="1.2" style="display:none;"/>
              <circle id="maxSpeed" r="2.5" fill="none" stroke="var(--green)" stroke-width="1.2" style="display:none;"/>
            </svg>
            <div class="tooltip" id="tipSpeed"></div>
          </div>
        </div>
        <div class="x-axis"><span>-12 s</span><span>-9 s</span><span>-6 s</span><span>-3 s</span><span>agora</span></div>
      </div>

    </div>

  </div>

  <script>
    const maxData = 120;
    const history = { angle: [], error: [], omega: [], speed: [] };
    let currentData = { angle: 180, error: 0, omega: 0, speed: 0, samples: 0, encDir: 1, motDir: 1 };
    let initialSyncDone = false;

    const charts = [
      { key: "angle", unit: "°",  dec: 2, minSpan: 2.0,   path: "pathAngle", fill: "fillAngle",
        grid: "gridAngle", yAxis: "yAxisAngle", svg: "svgAngle", tip: "tipAngle", now: "nowAngle",
        end: "endAngle", min: "minAngle", max: "maxAngle" },
      { key: "error", unit: "°",  dec: 2, minSpan: 0.5,   path: "pathError", fill: "fillError",
        grid: "gridError", yAxis: "yAxisError", svg: "svgError", tip: "tipError", now: "nowError",
        end: "endError", min: "minError", max: "maxError" },
      { key: "omega", unit: "°/s", dec: 1, minSpan: 10.0, path: "pathOmega", fill: "fillOmega",
        grid: "gridOmega", yAxis: "yAxisOmega", svg: "svgOmega", tip: "tipOmega", now: "nowOmega",
        end: "endOmega", min: "minOmega", max: "maxOmega" },
      { key: "speed", unit: "p/s", dec: 0, minSpan: 200.0, path: "pathSpeed", fill: "fillSpeed",
        grid: "gridSpeed", yAxis: "yAxisSpeed", svg: "svgSpeed", tip: "tipSpeed", now: "nowSpeed",
        end: "endSpeed", min: "minSpeed", max: "maxSpeed" },
    ];
    const lastBounds = {};

    function getBounds(arr, minSpan) {
      if (!arr.length) return { min: 0, max: 1, mid: 0.5 };
      let min = Math.min(...arr);
      let max = Math.max(...arr);
      if (max - min < minSpan) {
        const mid = (max + min) / 2;
        min = mid - minSpan / 2;
        max = mid + minSpan / 2;
      }
      const pad = (max - min) * 0.12;
      min -= pad;
      max += pad;
      return { min, max, mid: (max + min) / 2 };
    }

    function yToPx(val, b) { return 190 - ((val - b.min) / (b.max - b.min)) * 190; }
    function xToPx(idx) { return idx * (500 / (maxData - 1)); }

    function generateSmoothPath(dataArr, b) {
      if (dataArr.length < 2) return "";
      const points = dataArr.map((val, idx) => ({ x: xToPx(idx), y: yToPx(val, b) }));
      let d = `M ${points[0].x.toFixed(1)} ${points[0].y.toFixed(1)}`;
      for (let i = 0; i < points.length - 1; i++) {
        const p0 = points[i === 0 ? i : i - 1];
        const p1 = points[i];
        const p2 = points[i + 1];
        const p3 = points[i + 2 < points.length ? i + 2 : i + 1];
        const cp1x = p1.x + (p2.x - p0.x) / 6;
        const cp1y = p1.y + (p2.y - p0.y) / 6;
        const cp2x = p2.x - (p3.x - p1.x) / 6;
        const cp2y = p2.y - (p3.y - p1.y) / 6;
        d += ` C ${cp1x.toFixed(1)} ${cp1y.toFixed(1)}, ${cp2x.toFixed(1)} ${cp2y.toFixed(1)}, ${p2.x.toFixed(1)} ${p2.y.toFixed(1)}`;
      }
      return d;
    }

    function drawGrid(el, b) {
      let s = "";
      for (let i = 0; i <= 4; i++) {
        const y = (190 / 4) * i;
        const strong = (i === 2);
        s += `<line x1="0" y1="${y}" x2="500" y2="${y}" stroke="rgba(255,255,255,${strong ? .13 : .05})" ${strong ? 'stroke-dasharray="5 4"' : ''}/>`;
      }
      for (let i = 1; i < 4; i++) {
        const x = (500 / 4) * i;
        s += `<line x1="${x}" y1="0" x2="${x}" y2="190" stroke="rgba(255,255,255,.035)"/>`;
      }
      if (b.min < 0 && b.max > 0) {
        const y0 = yToPx(0, b);
        s += `<line x1="0" y1="${y0}" x2="500" y2="${y0}" stroke="rgba(255,255,255,.22)" stroke-dasharray="2 3"/>`;
      }
      el.innerHTML = s;
    }

    function renderGraphs() {
      charts.forEach(c => {
        const data = history[c.key];
        if (!data.length) return;
        const b = getBounds(data, c.minSpan);
        lastBounds[c.key] = b;

        const yEl = document.getElementById(c.yAxis);
        let yHtml = "";
        for (let i = 0; i <= 4; i++) {
          const v = b.max - ((b.max - b.min) / 4) * i;
          const y = (190 / 4) * i;
          yHtml += `<span style="top:${y}px">${v.toFixed(c.dec)}</span>`;
        }
        yEl.innerHTML = yHtml;

        drawGrid(document.getElementById(c.grid), b);

        const line = generateSmoothPath(data, b);
        document.getElementById(c.path).setAttribute("d", line);
        document.getElementById(c.fill).setAttribute("d", line + " L 500 190 L 0 190 Z");

        const n = data.length;
        const lastX = xToPx(n - 1), lastY = yToPx(data[n - 1], b);
        const endEl = document.getElementById(c.end);
        endEl.setAttribute("cx", lastX); endEl.setAttribute("cy", lastY);
        endEl.style.display = n >= 2 ? "" : "none";

        let minV = Infinity, maxV = -Infinity, minI = 0, maxI = 0;
        data.forEach((v, i) => { if (v < minV) { minV = v; minI = i; } if (v > maxV) { maxV = v; maxI = i; } });
        const minEl = document.getElementById(c.min);
        minEl.setAttribute("cx", xToPx(minI)); minEl.setAttribute("cy", yToPx(minV, b));
        minEl.style.display = (n >= 8 && (maxV - minV) > c.minSpan * 0.5) ? "" : "none";
        const maxEl = document.getElementById(c.max);
        maxEl.setAttribute("cx", xToPx(maxI)); maxEl.setAttribute("cy", yToPx(maxV, b));
        maxEl.style.display = minEl.style.display;
      });
    }

    charts.forEach(c => {
      const svg = document.getElementById(c.svg);
      const tip = document.getElementById(c.tip);
      svg.addEventListener("mousemove", (e) => {
        const data = history[c.key];
        const rect = svg.getBoundingClientRect();
        if (!data.length || !lastBounds[c.key]) return;
        const fx = (e.clientX - rect.left) / rect.width;
        const idx = Math.round(fx * (maxData - 1));
        const clamped = Math.max(0, Math.min(data.length - 1, idx));
        const val = data[clamped];
        const b = lastBounds[c.key];
        tip.innerText = `${val.toFixed(c.dec)} ${c.unit} · há ${((data.length - 1 - clamped) * 0.1).toFixed(1)}s`;
        tip.style.left = (xToPx(clamped) / 500 * 100) + "%";
        tip.style.top = (yToPx(val, b) / 190 * 100) + "%";
        tip.style.opacity = "1";
      });
      svg.addEventListener("mouseleave", () => tip.style.opacity = "0");
    });

    function animLoop() {
      history.angle.push(currentData.angle);
      history.error.push(currentData.error);
      history.omega.push(currentData.omega);
      history.speed.push(currentData.speed);
      if (history.angle.length > maxData) {
        history.angle.shift();
        history.error.shift();
        history.omega.shift();
        history.speed.shift();
      }
      renderGraphs();
    }
    setInterval(animLoop, 100);

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

        currentData = data;

        document.getElementById("angleVal").innerText = data.angle.toFixed(2) + "°";
        document.getElementById("errorVal").innerText = data.error.toFixed(2) + "°";
        document.getElementById("omegaVal").innerText = data.omega.toFixed(1) + "°/s";
        document.getElementById("speedVal").innerText = data.speed.toFixed(0) + " p/s";
        document.getElementById("sampleCount").innerText = data.samples;

        document.getElementById("nowAngle").innerText = data.angle.toFixed(2) + " °";
        document.getElementById("nowError").innerText = data.error.toFixed(2) + " °";
        document.getElementById("nowOmega").innerText = data.omega.toFixed(1) + " °/s";
        document.getElementById("nowSpeed").innerText = data.speed.toFixed(0) + " p/s";
        document.getElementById("refVal").innerText = data.ref.toFixed(2) + "°";

        const btnEnc = document.getElementById("btnEncDir");
        if (data.encDir === 1) {
          btnEnc.innerText = "ENC: DIR 1";
          btnEnc.classList.remove("active");
        } else {
          btnEnc.innerText = "ENC: DIR -1";
          btnEnc.classList.add("active");
        }

        const btnMot = document.getElementById("btnMotDir");
        if (data.motDir === 1) {
          btnMot.innerText = "MOT: DIR 1";
          btnMot.classList.remove("active");
        } else {
          btnMot.innerText = "MOT: DIR -1";
          btnMot.classList.add("active");
        }
      } catch (e) {}
    }
    setInterval(fetchData, 100);

    async function updateParams() {
      const kp = document.getElementById("inputKp").value;
      const ki = document.getElementById("inputKi").value;
      const kd = document.getElementById("inputKd").value;
      const kx = document.getElementById("inputKx").value;
      const alpha = document.getElementById("inputAlpha").value;
      try {
        await fetch(`/set_params?kp=${kp}&ki=${ki}&kd=${kd}&kx=${kx}&alpha=${alpha}`);
        showMsg();
      } catch (e) {}
    }

    async function sendCmd(cmd) {
      try {
        await fetch(`/cmd?c=${cmd}`);
        showMsg();
      } catch (e) {}
    }

    async function toggleEncDir() {
      const newVal = currentData.encDir === 1 ? -1 : 1;
      await sendCmd(`ENC_DIR ${newVal}`);
    }

    async function toggleMotDir() {
      const newVal = currentData.motDir === 1 ? -1 : 1;
      await sendCmd(`MOTOR_DIR ${newVal}`);
    }

    function showMsg() {
      const msg = document.getElementById("statusMsg");
      msg.style.opacity = "1";
      setTimeout(() => msg.style.opacity = "0", 2000);
    }
  </script>
</body>
</html>
)rawliteral";

long roundLong(float value) {
  if (value >= 0.0f) return (long)(value + 0.5f);
  return (long)(value - 0.5f);
}

long degToSteps(float deg) {
  return roundLong(deg * MOTOR_STEPS_PER_DEG);
}

float stepsToDeg(long steps) {
  return (float)steps / MOTOR_STEPS_PER_DEG;
}

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

  if (targetSpeed > MAX_CENTERING_SPEED_DEG_S)
    targetSpeed = MAX_CENTERING_SPEED_DEG_S;
  if (targetSpeed < -MAX_CENTERING_SPEED_DEG_S)
    targetSpeed = -MAX_CENTERING_SPEED_DEG_S;

  centeringSpeedDegPerSecond =
      (CENTERING_FILTER_ALPHA * targetSpeed) +
      ((1.0f - CENTERING_FILTER_ALPHA) * centeringSpeedDegPerSecond);

  return centeringSpeedDegPerSecond;
}

void handleData() {
  String json;
  json.reserve(350);
  json = "{";
  json += "\"angle\":" + String(getEncoderAngle(), 2) + ",";
  json += "\"error\":" + String(currentError, 2) + ",";
  json += "\"omega\":" + String(filteredVelocity, 2) + ",";
  json += "\"speed\":" + String(target_motor_speed, 2) + ",";
  json += "\"samples\":" + String(totalControlSamples) + ",";
  json += "\"ref\":" + String(currentRefDeg, 2) + ",";
  json += "\"kp\":" + String(Kp, 5) + ",";
  json += "\"ki\":" + String(Ki, 5) + ",";
  json += "\"kd\":" + String(Kd, 5) + ",";
  json += "\"kx\":" + String(Kx, 5) + ",";
  json += "\"alpha\":" + String(DERIVATIVE_FILTER_ALPHA, 5) + ",";
  json += "\"encDir\":" + String(encoderDir) + ",";
  json += "\"motDir\":" + String(motorDir);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSetParams() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("kx")) Kx = server.arg("kx").toFloat();
  if (server.hasArg("alpha")) {
    float newAlpha = server.arg("alpha").toFloat();
    if (newAlpha >= 0.0f && newAlpha < 1.0f) {
      DERIVATIVE_FILTER_ALPHA = newAlpha;
    }
  }
  integral = 0.0f;
  centeringSpeedDegPerSecond = 0.0f;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void processPIDCommand(String cmd) {
  char buffer[80];
  cmd.toCharArray(buffer, sizeof(buffer));
  char* token = strtok(buffer, " ");
  token = strtok(NULL, " "); if (!token) return; float newKp = atof(token);
  token = strtok(NULL, " "); if (!token) return; float newKi = atof(token);
  token = strtok(NULL, " "); if (!token) return; float newKd = atof(token);
  token = strtok(NULL, " "); float newBias = token ? atof(token) : 0.0f;
  Kp = newKp; Ki = newKi; Kd = newKd; Bias = newBias; integral = 0.0f;
}

void processCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd.length() == 0) return;
  if (cmd == "0") { setZeroDown(); return; }
  if (cmd == "1") { setUp180(); return; }
  if (cmd == "S") { startControl(); return; }
  if (cmd == "P") { stopControl(); return; }
  if (cmd.startsWith("PID ")) { processPIDCommand(cmd); return; }

  if (cmd.startsWith("KP ")) { Kp = cmd.substring(3).toFloat(); integral = 0.0f; return; }
  if (cmd.startsWith("KI ")) { Ki = cmd.substring(3).toFloat(); integral = 0.0f; return; }
  if (cmd.startsWith("KD ")) { Kd = cmd.substring(3).toFloat(); return; }
  if (cmd.startsWith("KX ")) { Kx = cmd.substring(3).toFloat(); return; }
  if (cmd.startsWith("BIAS ")) { Bias = cmd.substring(5).toFloat(); return; }
  if (cmd.startsWith("ALPHA ")) { 
    float newAlpha = cmd.substring(6).toFloat();
    if (newAlpha >= 0.0f && newAlpha < 1.0f) {
      DERIVATIVE_FILTER_ALPHA = newAlpha;
    }
    return;
  }

  if (cmd.startsWith("ENC_DIR ")) {
    int value = cmd.substring(8).toInt();
    if (value == 1 || value == -1) {
      encoderDir = value;
    }
    return;
  }

  if (cmd.startsWith("MOTOR_DIR ")) {
    int value = cmd.substring(10).toInt();
    if (value == 1 || value == -1) {
      motorDir = value;
    }
    return;
  }
}

void handleCommandWeb() {
  if (server.hasArg("c")) {
    String c = server.arg("c");
    c.toUpperCase();
    if (c == "0") setZeroDown();
    else if (c == "1") setUp180();
    else if (c == "S") startControl();
    else if (c == "P") stopControl();
    else if (c.startsWith("ENC_DIR ") || c.startsWith("MOTOR_DIR ")) processCommand(c);
  }
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void webServerTask(void * pvParameters) {
  server.on("/", []() {
    server.send_P(200, "text/html", index_html);
  });
  server.on("/data", handleData);
  server.on("/set_params", handleSetParams);
  server.on("/cmd", handleCommandWeb);
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

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
  Serial.println("STOP");
}

void startControl() {
  if (!calibrated) { return; }
  float angle = getEncoderAngle();
  float error = shortestAngleError(SETPOINT_DEG, angle);
  if (fabsf(error) > GUARD_HALF_WIDTH) { return; }
  
  target_motor_speed = 0.0f;
  flag_reset_motor_pos = true;

  totalControlSamples = 0;
  currentRefDeg = SETPOINT_DEG;

  resetControllerState(angle);
  enableMotor();
  controlEnabled = true;
  Serial.println("START");
}

void setZeroDown() {
  stopControl();
  encoderOffset = getEncoderCount();
  flag_reset_motor_pos = true;
  currentRefDeg = SETPOINT_DEG;
  calibrated = true;
  Serial.println("ZERO_DOWN");
}

void setUp180() {
  stopControl();
  encoderOffset = getEncoderCount() - (long)(1200L * encoderDir);
  flag_reset_motor_pos = true;
  currentRefDeg = SETPOINT_DEG;
  calibrated = true;
  Serial.println("SET_UP 180");
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) { processCommand(serialBuffer); serialBuffer = ""; }
    } else { if (serialBuffer.length() < 79) serialBuffer += c; }
  }
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

  bool integralAllowed =
      (!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && currentError < 0.0f) ||
      (saturatedLow && currentError > 0.0f);

  if (integralAllowed) {
    integral += currentError * dt;

    if (integral > INTEGRAL_CLAMP) integral = INTEGRAL_CLAMP;
    if (integral < -INTEGRAL_CLAMP) integral = -INTEGRAL_CLAMP;
  }

  pidOutputDeg = (Kp * currentError) +
                 (Ki * integral) -
                 (Kd * filteredVelocity) +
                 Bias;

  float pidSpeedDegPerSecond = pidOutputDeg / dt;

  if (pidSpeedDegPerSecond > MAX_CONTROL_SPEED_DEG_S)
    pidSpeedDegPerSecond = MAX_CONTROL_SPEED_DEG_S;
  if (pidSpeedDegPerSecond < -MAX_CONTROL_SPEED_DEG_S)
    pidSpeedDegPerSecond = -MAX_CONTROL_SPEED_DEG_S;

  float centeringSpeed = computeCenteringSpeed(motorAngle);

  float speedDegPerSecond = pidSpeedDegPerSecond + centeringSpeed;

  if (speedDegPerSecond > MAX_CONTROL_SPEED_DEG_S)
    speedDegPerSecond = MAX_CONTROL_SPEED_DEG_S;
  if (speedDegPerSecond < -MAX_CONTROL_SPEED_DEG_S)
    speedDegPerSecond = -MAX_CONTROL_SPEED_DEG_S;

  speedDegPerSecond *= (float)motorDir;

  float motorAbsAngle = fabsf(motorAngle);

  if (MAX_STEPPER_ABS_DEG > 0.0f &&
      motorAbsAngle >= MAX_STEPPER_ABS_DEG) {
    stopControl();
    return;
  }

  target_motor_speed = speedDegPerSecond * MOTOR_STEPS_PER_DEG;
}

void controlTask(void * pvParameters) {
  for (;;) {
    readSerial();
    if (controlEnabled) {
      controlLoop();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);

  disableLoopWDT();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

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
