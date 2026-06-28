/*
 * Ball and Beam - Controlador PID Integrado (ESP32) - v3 + WiFi
 * Integra:
 *  - Control PID discreto en tarea FreeRTOS (Core 1) con derivada Tustin Bilineal.
 *  - Zona Muerta Total (BANDA_OK) para eliminar el temblor del servo en el centro.
 *  - Servo via ESP32Servo calibrado.
 *  - Sensor Sharp GP2Y0A21 con tabla de calibración + filtro (ALPHA = 0.50).
 *  - Referencia seleccionable por: (a) Potenciómetro analógico en pin 35
 *                                  (b) WiFi (Modo AP + Web Server, slider/botones)
 *                                  (c) Comando Serial USB "ref=<m>"
 *  - Tarea de red WiFi en Core 0 (Servidor Web con gráfica en tiempo real).
 *
 * RED WiFi (modo AP):
 *   SSID:     BallAndBeam
 *   Password: 12345678
 *   IP:       http://192.168.4.1
 *
 *  Cambios v3.1:
 *   - Rango de referencia ampliado a ±15 cm (pote, slider y comandos).
 *   - Slider web con paso 0.5 cm y mas botones de preset.
 *   - Eje Y de la grafica y barra visual escalados a ±16 cm / ±15 cm.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// ====================== CONFIGURACION DE HARDWARE ======================
#define SERVO_PIN     13
#define SENSOR_PIN    34
#define POT_PIN       35

const int US_PULSO_MIN = 500;
const int US_PULSO_MAX = 2500;
const int US_CENTRO    = 1420;
const int US_MIN_SEG   = 950;
const int US_MAX_SEG   = 1900;

// ====================== SENSOR SHARP (Calibrado v3) =======================
#define N_MUESTRAS   20
#define ALPHA        0.50f  // Filtro adaptado para Tustin (Menos lag)

const int   N_PTS = 19;
const float dist_cm[N_PTS] = { 3,     5,     7,     9,     11,    13,    15,    17,    19,    21,
                               23,    25,    27,    29,    31,    33,    35,    37,    39 };
const float adc_val[N_PTS] = { 2760, 2350, 1940, 1680, 1470, 1360, 1190, 1080, 970,  895,
                               820,  760,  710,  660,  620,  600,  570,  540,  516 };

const float CENTRO_CM      = 21.5f;
const float RADIO_BOLA_CM = 2.0f;

float distFiltrada = 0.0f;
bool  primeraLectura = true;

// ====================== POTENCIOMETRO ==================================
// >>> CAMBIO v3.1: rango ampliado a ±15 cm <<<
const float REF_MIN_M     = -0.15f;
const float REF_MAX_M     =  0.15f;
const float ALPHA_POT     = 0.10f;
const float REF_DEADBAND  = 0.005f;
float refFiltradaADC = 2048.0f;

// ====================== FUENTE DE REFERENCIA ===========================
enum RefSource { REF_POT, REF_REMOTE };
volatile RefSource ref_source = REF_POT;

// ====================== MAPPING SERVO ==================================
const float DEG_A_US = 134.78f;

// ====================== VARIABLES DEL PID (v3) ==============================
float Kp = -0.907f;
float Ki = -0.0f;
float Kd = -3.743f;
float Ts = 0.05f;

float integral = 0.0f;
float e_prev   = 0.0f;
float referencia_m = 0.0f;

// Filtro derivativo Tustin Bilineal: D(s) = s * N / (s + N)
float N_DERIV = 8.0f;         // Polo del filtro derivativo (Rango recomendado: 5-20)
float derivFiltrada = 0.0f;   // Estado interno del filtro
float Kaw = 1.0f / 0.05f;     // Ganancia Anti-windup

// ====================== ZONA MUERTA TOTAL (v3) ==========================
float BANDA_OK = 0.010f;      // Metros tolerables (0.010 = 1 cm) para apagar PID

// ====================== COMPENSACION DE FRICCION (Coulomb) =============
float U_COULOMB = 0.0f;
float E_BANDA   = 0.005f;
float V_PARADA  = 0.020f;
float E_SUAVE   = 0.010f;
float velFiltrada = 0.0f;

// ====================== CALIBRACION DE COULOMB =========================
volatile bool  g_modo_calibracion = false;
volatile int   g_signo_rampa      = 1;
const float    TASA_RAMPA_DEG_S   = 0.5f;
const float    ANGULO_MAX_RAMPA   = 3.5f;
float          g_angulo_rampa_deg = 0.0f;
float          g_t_rampa          = 0.0f;
bool           g_rampa_recien_iniciada = true;

// ====================== TELEMETRIA =====================================
volatile float g_dist_cm = 0.0f;
volatile float g_pos_m   = 0.0f;
volatile float g_error_m = 0.0f;
volatile float g_u_rad   = 0.0f;
volatile int   g_pulso_us = US_CENTRO;
volatile float g_vel     = 0.0f;
volatile float g_coulomb = 0.0f;
volatile float g_ref_m   = 0.0f;

// ====================== OBJETOS / RED WiFi =============================
Servo servo;
WebServer server(80);

const char* AP_SSID = "BallAndBeam";
const char* AP_PASS = "12345678";

String serialBuffer = "";

// ====================== PROTOTIPOS =====================================
void  vControlTask(void *pvParameters);
void  vNetworkTask(void *pvParameters);
void  procesarComando(String cmd, Stream* respuesta);
void  setupWebServer();
void  handleRoot();
void  handleData();
void  handleSet();

// ====================== HELPERS ========================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float leerADC() {
  long suma = 0;
  for (int i = 0; i < N_MUESTRAS; i++) {
    suma += analogRead(SENSOR_PIN);
    delayMicroseconds(200);
  }
  return suma / (float)N_MUESTRAS;
}

float adcADistancia(float adc) {
  if (adc >= adc_val[0])        return dist_cm[0];
  if (adc <= adc_val[N_PTS-1])  return dist_cm[N_PTS-1];
  for (int i = 0; i < N_PTS - 1; i++) {
    if (adc <= adc_val[i] && adc >= adc_val[i+1]) {
      float t = (adc - adc_val[i]) / (adc_val[i+1] - adc_val[i]);
      return dist_cm[i] + t * (dist_cm[i+1] - dist_cm[i]);
    }
  }
  return -1.0f;
}

// ====================== INTERFAZ HTML EMBEBIDA =========================
const char PAGE_INDEX[] = R"HTML(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Ball &amp; Beam v3</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 12px;
    font-family: -apple-system, system-ui, sans-serif;
    background: #0e1116; color: #e6edf3;
  }
  h1 { font-size: 20px; margin: 0 0 12px; }
  .card {
    background: #161b22; border: 1px solid #30363d;
    border-radius: 12px; padding: 12px; margin-bottom: 12px;
  }
  .row { display: flex; justify-content: space-between; margin: 6px 0; font-size: 14px; }
  .row b { color: #58a6ff; }
  input[type=range] { width: 100%; height: 40px; -webkit-appearance: none; background: transparent; }
  input[type=range]::-webkit-slider-runnable-track { height: 8px; background: #30363d; border-radius: 4px; }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 28px; height: 28px; border-radius: 50%;
    background: #58a6ff; margin-top: -10px; border: 2px solid #0e1116;
  }
  .btns { display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-top: 8px; }
  button {
    padding: 10px 0; border-radius: 8px; border: 1px solid #30363d;
    background: #21262d; color: #e6edf3; font-size: 14px; font-weight: 600;
  }
  button:active { background: #30363d; }
  button.home { background: #1f6feb; border-color: #1f6feb; }
  .src {
    display: inline-block; padding: 2px 8px; border-radius: 10px;
    background: #1f6feb33; color: #58a6ff; font-size: 12px; font-weight: 600;
  }
  .bar { position: relative; height: 22px; background: #30363d; border-radius: 11px; margin-top: 8px; overflow: visible; }
  .bar .center { position: absolute; left: 50%; top: 0; width: 2px; height: 100%; background: #6e7681; }
  .bar .ball { position: absolute; top: 3px; width: 16px; height: 16px; border-radius: 50%;
               background: #f78166; margin-left: -8px; transition: left 0.1s; }
  .bar .ref  { position: absolute; top: -3px; width: 2px; height: 28px; background: #3fb950; }
  #chart { width: 100%; height: 220px; display: block; background: #0d1117; border-radius: 8px; }
  .legend { display: flex; gap: 14px; font-size: 12px; margin-top: 6px; color: #8b949e; }
  .legend .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 5px; vertical-align: middle; }
  .params { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 12px; font-size: 14px; }
  .params .k { color: #8b949e; }
  .params .v { color: #58a6ff; font-weight: 600; text-align: right; font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<h1>Ball &amp; Beam v3 <span id="srcTag" class="src">POT</span></h1>

<div class="card">
  <div class="row"><span>Posicion</span><b><span id="pos">--</span> cm</b></div>
  <div class="row"><span>Referencia</span><b><span id="ref">--</span> cm</b></div>
  <div class="row"><span>Error</span><b><span id="err">--</span> cm</b></div>
  <div class="bar">
    <div class="center"></div>
    <div class="ref"  id="refMark"></div>
    <div class="ball" id="ballMark"></div>
  </div>
</div>

<div class="card">
  <canvas id="chart"></canvas>
  <div class="legend">
    <span><span class="dot" style="background:#3fb950"></span>Referencia</span>
    <span><span class="dot" style="background:#f78166"></span>Posicion</span>
    <span style="margin-left:auto">ventana: <span id="winSec">20</span>s</span>
  </div>
</div>

<div class="card">
  <div class="row"><span>Slider de referencia</span><b><span id="sliderVal">0.0</span> cm</b></div>
  <input id="slider" type="range" min="-15" max="15" step="0.5" value="0">
  <div class="btns">
    <button onclick="setRef(-12)">-12</button>
    <button onclick="setRef(-8)">-8</button>
    <button onclick="setRef(-4)">-4</button>
    <button onclick="setRef(-2)">-2</button>
    <button onclick="setRef(0)">0</button>
  </div>
  <div class="btns" style="margin-top:6px">
    <button onclick="setRef(2)">+2</button>
    <button onclick="setRef(4)">+4</button>
    <button onclick="setRef(8)">+8</button>
    <button onclick="setRef(12)">+12</button>
    <button class="home" onclick="setRef(0)">&#8962;</button>
  </div>
</div>

<div class="card">
  <div class="row" style="margin-bottom:10px"><b>Parametros PID</b></div>
  <div class="params">
    <div class="k">Kp</div>        <div class="v" id="pKp">--</div>
    <div class="k">Ki</div>        <div class="v" id="pKi">--</div>
    <div class="k">Kd</div>        <div class="v" id="pKd">--</div>
    <div class="k">U_Coulomb</div> <div class="v" id="pUcb">--</div>
  </div>
</div>

<div class="card">
  <div class="row"><span>Pulso servo</span><b><span id="us">--</span> us</b></div>
  <div class="row"><span>u (rad)</span><b><span id="u">--</span></b></div>
  <div class="row"><span>Vel</span><b><span id="vel">--</span> m/s</b></div>
  <div class="btns" style="grid-template-columns: 1fr 1fr;">
    <button onclick="cmd('pot=on')">Usar Pote</button>
    <button onclick="cmd('i')">Reset Integral</button>
  </div>
</div>

<script>
const slider = document.getElementById('slider');
const sliderVal = document.getElementById('sliderVal');
let sliderTimer = null;

slider.addEventListener('input', () => {
  sliderVal.textContent = (+slider.value).toFixed(1);
  clearTimeout(sliderTimer);
  sliderTimer = setTimeout(() => setRef(+slider.value), 50);
});

function setRef(cm) {
  slider.value = cm;
  sliderVal.textContent = (+cm).toFixed(1);
  fetch('/set?ref=' + (cm/100).toFixed(4));
}
function cmd(c) { fetch('/set?cmd=' + encodeURIComponent(c)); }
function clamp(v, a, b) { return Math.max(a, Math.min(b, v)); }

const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');
const WINDOW_MS = 20000;
const Y_MIN = -16, Y_MAX = 16;
const history = [];
let dpr = window.devicePixelRatio || 1;

function resizeCanvas() {
  dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  canvas.width  = w * dpr;
  canvas.height = h * dpr;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function drawChart() {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  const padL = 32*dpr, padR = 8*dpr, padT = 8*dpr, padB = 18*dpr;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  ctx.fillStyle = '#0d1117';
  ctx.fillRect(0, 0, W, H);

  ctx.strokeStyle = '#21262d';
  ctx.lineWidth = 1*dpr;
  ctx.fillStyle = '#6e7681';
  ctx.font = (10*dpr) + 'px sans-serif';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let cm = Y_MIN; cm <= Y_MAX; cm += 4) {
    const y = padT + plotH * (1 - (cm - Y_MIN) / (Y_MAX - Y_MIN));
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + plotW, y); ctx.stroke();
    ctx.fillText(cm.toString(), padL - 4*dpr, y);
  }

  const yZero = padT + plotH * (1 - (0 - Y_MIN) / (Y_MAX - Y_MIN));
  ctx.strokeStyle = '#30363d';
  ctx.beginPath(); ctx.moveTo(padL, yZero); ctx.lineTo(padL + plotW, yZero); ctx.stroke();

  if (history.length < 2) return;
  const tNow = history[history.length - 1].t;
  const tMin = tNow - WINDOW_MS;

  function xOf(t) { return padL + plotW * (t - tMin) / WINDOW_MS; }
  function yOf(cm) { return padT + plotH * (1 - (clamp(cm, Y_MIN, Y_MAX) - Y_MIN) / (Y_MAX - Y_MIN)); }

  function plotSeries(key, color) {
    ctx.strokeStyle = color; ctx.lineWidth = 2*dpr; ctx.beginPath();
    let started = false;
    for (const p of history) {
      if (p.t < tMin) continue;
      const x = xOf(p.t), y = yOf(p[key]);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else          { ctx.lineTo(x, y); }
    }
    ctx.stroke();
  }
  plotSeries('ref', '#3fb950');
  plotSeries('pos', '#f78166');
}

async function poll() {
  try {
    const r = await fetch('/data');
    const d = await r.json();
    const posCm = d.pos * 100, refCm = d.ref * 100, errCm = d.err * 100;

    document.getElementById('pos').textContent = posCm.toFixed(2);
    document.getElementById('ref').textContent = refCm.toFixed(2);
    document.getElementById('err').textContent = errCm.toFixed(2);
    document.getElementById('us').textContent  = d.us;
    document.getElementById('u').textContent   = d.u.toFixed(3);
    document.getElementById('vel').textContent = d.vel.toFixed(3);
    document.getElementById('srcTag').textContent = d.src;
    document.getElementById('pKp').textContent  = d.kp.toFixed(4);
    document.getElementById('pKi').textContent  = d.ki.toFixed(4);
    document.getElementById('pKd').textContent  = d.kd.toFixed(4);
    document.getElementById('pUcb').textContent = d.ucb.toFixed(4) + ' rad';

    // ±15 cm -> 0..100% en la barra visual
    const ballPct = clamp(50 + posCm * (50/15), 0, 100);
    const refPct  = clamp(50 + refCm * (50/15), 0, 100);
    document.getElementById('ballMark').style.left = ballPct + '%';
    document.getElementById('refMark').style.left  = refPct  + '%';

    const now = performance.now();
    history.push({ t: now, ref: refCm, pos: posCm });
    while (history.length && history[0].t < (now - WINDOW_MS - 1000)) history.shift();
    drawChart();
  } catch(e) {}
}
setInterval(poll, 100);
poll();
</script>
</body>
</html>
)HTML";

// ====================== WEB SERVER HANDLERS ============================
void handleRoot() { server.send(200, "text/html", PAGE_INDEX); }

void handleData() {
  char buf[320];
  const char* src = (ref_source == REF_POT) ? "POT" : "REMOTE";
  snprintf(buf, sizeof(buf),
    "{\"pos\":%.4f,\"ref\":%.4f,\"err\":%.4f,\"u\":%.4f,\"vel\":%.4f,\"us\":%d,\"src\":\"%s\","
    "\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"ucb\":%.4f}",
    g_pos_m, g_ref_m, g_error_m, g_u_rad, g_vel, g_pulso_us, src,
    Kp, Ki, Kd, U_COULOMB);
  server.send(200, "application/json", buf);
}

void handleSet() {
  if (server.hasArg("ref")) {
    float v = server.arg("ref").toFloat();
    if (v < REF_MIN_M) v = REF_MIN_M;
    if (v > REF_MAX_M) v = REF_MAX_M;
    referencia_m = v;
    ref_source = REF_REMOTE;
    server.send(200, "text/plain", "OK");
    return;
  }
  if (server.hasArg("cmd")) {
    String c = server.arg("cmd");
    procesarComando(c, &Serial);
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "missing ref or cmd");
}

void setupWebServer() {
  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.on("/set",  handleSet);
  server.onNotFound([](){ server.send(404, "text/plain", "not found"); });
  server.begin();
}

// ====================== TAREA DE RED (Core 0) ==========================
void vNetworkTask(void *pvParameters) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("# WiFi AP listo. SSID: ");
  Serial.print(AP_SSID);
  Serial.print("  IP: http://");
  Serial.println(ip);

  setupWebServer();

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ====================== SETUP ==========================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, US_PULSO_MIN, US_PULSO_MAX);
  servo.writeMicroseconds(US_CENTRO);
  delay(1000);

  // Creación de Tareas compartidas en núcleos separados para evitar jitter temporal
  xTaskCreatePinnedToCore(vControlTask, "Control_PID", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vNetworkTask, "Network", 8192, NULL, 1, NULL, 0);

  Serial.println();
  Serial.println("# === Ball and Beam v3.1 - Modo Hibrido (Serial/WiFi/Pote) ===");
  Serial.println("#   Rango de referencia: -15 cm  ..  +15 cm");
  Serial.println("#   c          : rampa de calibracion Coulomb (+)");
  Serial.println("#   n          : rampa de calibracion Coulomb (-)");
  Serial.println("#   p          : volver a modo PID");
  Serial.println("#   kp=<val>   : fijar Kp");
  Serial.println("#   kd=<val>   : fijar Kd");
  Serial.println("#   ki=<val>   : fijar Ki (resetea integral)");
  Serial.println("#   nd=<val>   : polo filtro derivativo Tustin (ej: nd=8)");
  Serial.println("#   bk=<val>   : zona muerta total en metros (ej: bk=0.010)");
  Serial.println("#   cb=<val>   : fijar U_COULOMB en radianes");
  Serial.println("#   pot=on/off : usar / no usar potenciometro");
  Serial.println("#   ref=<m>    : fijar referencia manual (modo REMOTE)");
  Serial.println("#   i          : reset integral + derivativo");
  Serial.println("#   ?          : imprimir parametros actuales");
  Serial.println("# --------------------------------------------");
  Serial.println("# WiFi AP: SSID=BallAndBeam  pass=12345678  ->  http://192.168.4.1");
  Serial.println("# ============================================");
}

// ====================== LOOP (Telemetria Serial) =======================
void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        procesarComando(serialBuffer, &Serial);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) serialBuffer = "";
    }
  }

  if (g_modo_calibracion) {
    if (g_rampa_recien_iniciada) {
      Serial.println("TRIAL");
      g_rampa_recien_iniciada = false;
    }
    Serial.print(g_t_rampa, 4);  Serial.print(",");
    Serial.println(g_angulo_rampa_deg, 4);
  } else {
    const char* src = (ref_source == REF_POT) ? "POT" : "REM";
    Serial.print("[");  Serial.print(src); Serial.print("] ");
    Serial.print("Pos: "); Serial.print(g_pos_m * 100.0f, 2);
    Serial.print(" cm\tRef: "); Serial.print(g_ref_m * 100.0f, 2);
    Serial.print(" cm\tErr: "); Serial.print(g_error_m * 100.0f, 2);
    Serial.print(" cm\tu: ");   Serial.print(g_u_rad, 3);
    Serial.print(" rad\tVel: "); Serial.print(g_vel, 3);
    Serial.print(" m/s\tPulso: "); Serial.println(g_pulso_us);
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

// ====================== TAREA DE CONTROL (Core 1) ======================
void vControlTask(void *pvParameters) {
  const TickType_t xFrecuenciaMuestreo = pdMS_TO_TICKS((uint32_t)(Ts * 1000.0f));
  TickType_t xUltimoDespertar = xTaskGetTickCount();

  for (;;) {
    float adc  = leerADC();
    float dist = adcADistancia(adc);

    if (dist >= 0.0f) {
      if (primeraLectura) { distFiltrada = dist; primeraLectura = false; }
      else                { distFiltrada = ALPHA * dist + (1.0f - ALPHA) * distFiltrada; }
    }

    float dist_centro_cm = distFiltrada + RADIO_BOLA_CM;
    float posicion_m = (dist_centro_cm - CENTRO_CM) / 100.0f;

    if (g_modo_calibracion) {
      g_angulo_rampa_deg += g_signo_rampa * TASA_RAMPA_DEG_S * Ts;
      g_t_rampa += Ts;

      if (fabsf(g_angulo_rampa_deg) > ANGULO_MAX_RAMPA) {
        g_modo_calibracion = false;
        Serial.println("# ABORTADO: ANGULO_MAX_RAMPA alcanzado");
      }

      float pulso_us = (float)US_CENTRO - DEG_A_US * g_angulo_rampa_deg;
      float pulso_sat = pulso_us;
      if (pulso_sat > US_MAX_SEG) pulso_sat = US_MAX_SEG;
      if (pulso_sat < US_MIN_SEG) pulso_sat = US_MIN_SEG;

      servo.writeMicroseconds((int)pulso_sat);

      g_dist_cm  = distFiltrada;
      g_pos_m    = posicion_m;
      g_u_rad    = g_angulo_rampa_deg * (1.0f / 57.2958f);
      g_pulso_us = (int)pulso_sat;
      g_ref_m    = 0.0f;

      integral = 0.0f;
      e_prev   = referencia_m - posicion_m;
      velFiltrada = 0.0f;
      derivFiltrada = 0.0f;

    } else {
      // Leer Potenciómetro si está activo como fuente
      if (ref_source == REF_POT) {
        float adc_pot = (float)analogRead(POT_PIN);
        refFiltradaADC = ALPHA_POT * adc_pot + (1.0f - ALPHA_POT) * refFiltradaADC;
        float ref = map_float(refFiltradaADC, 0.0f, 4095.0f, REF_MIN_M, REF_MAX_M);
        if (fabsf(ref) < REF_DEADBAND) ref = 0.0f;
        referencia_m = ref;
      }

      float e_actual = referencia_m - posicion_m;

      // Discretización Derivativa por Filtro Tustin Bilineal
      float a_df = (N_DERIV * Ts - 2.0f) / (N_DERIV * Ts + 2.0f);
      float b_df = (2.0f * N_DERIV) / (N_DERIV * Ts + 2.0f);
      derivFiltrada = a_df * derivFiltrada + b_df * (e_actual - e_prev);

      float u_d = Kd * derivFiltrada;

      if (fabsf(e_actual) > E_BANDA) {
        integral += e_actual * Ts;
      }

      float u_pid = (Kp * e_actual) + (Ki * integral) + u_d;

      float vel_inst = -derivFiltrada;
      velFiltrada = 0.3f * vel_inst + 0.7f * velFiltrada;

      float u_coulomb = 0.0f;
      if (fabsf(e_actual) > E_BANDA && fabsf(velFiltrada) < V_PARADA) {
        float dir = tanhf((Kp * e_actual) / (fabsf(Kp) * E_SUAVE));
        u_coulomb = U_COULOMB * dir;
      }

      float u_actual = u_pid + u_coulomb;

      // >>> CAMBIO v3: ZONA MUERTA TOTAL <<<
      if (fabsf(e_actual) < BANDA_OK) {
        u_actual = 0.0f;
        integral = 0.0f; // Evita acumulación estática en zona muerta
      }

      // Mapeo Dinámico: Acción u (rad) -> Ángulo (deg) -> Pulso (us)
      float angulo_deg = u_actual * 57.2958f;
      float pulso_us   = (float)US_CENTRO - DEG_A_US * angulo_deg;

      float pulso_sat = pulso_us;
      if (pulso_sat > US_MAX_SEG) pulso_sat = US_MAX_SEG;
      if (pulso_sat < US_MIN_SEG) pulso_sat = US_MIN_SEG;

      // Anti-windup condicional si está saturado
      if (pulso_sat != pulso_us && Ki != 0.0f) {
        float u_sat = -(((pulso_sat - (float)US_CENTRO) / DEG_A_US)) / 57.2958f;
        integral += (Kaw * (u_sat - u_actual) * Ts) / Ki;
      }

      servo.writeMicroseconds((int)pulso_sat);

      // Copias de variables para hilos asíncronos de Telemetría (Serial/Web)
      g_dist_cm  = distFiltrada;
      g_pos_m    = posicion_m;
      g_error_m  = e_actual;
      g_u_rad    = u_actual;
      g_pulso_us = (int)pulso_sat;
      g_vel      = velFiltrada;
      g_coulomb  = u_coulomb;
      g_ref_m    = referencia_m;

      e_prev = e_actual;

      g_angulo_rampa_deg = 0.0f;
      g_t_rampa = 0.0f;
    }

    vTaskDelayUntil(&xUltimoDespertar, xFrecuenciaMuestreo);
  }
}

// ====================== PARSER DE COMANDOS =============================
void procesarComando(String cmd, Stream* resp) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "c") {
    g_signo_rampa = 1; g_angulo_rampa_deg = 0.0f; g_t_rampa = 0.0f;
    g_rampa_recien_iniciada = true; g_modo_calibracion = true;
    resp->println("# Rampa Coulomb (+)"); return;
  }
  if (cmd == "n") {
    g_signo_rampa = -1; g_angulo_rampa_deg = 0.0f; g_t_rampa = 0.0f;
    g_rampa_recien_iniciada = true; g_modo_calibracion = true;
    resp->println("# Rampa Coulomb (-)"); return;
  }
  if (cmd == "p") { g_modo_calibracion = false; resp->println("# Modo PID"); return; }

  if (cmd == "i" || cmd == "reset") {
    integral = 0.0f; derivFiltrada = 0.0f;
    resp->println("# Integral y derivada reseteadas"); return;
  }
  if (cmd == "pot=on" || cmd == "pot on") { ref_source = REF_POT; resp->println("# Fuente: POTENCIOMETRO"); return; }
  if (cmd == "pot=off" || cmd == "pot off") { ref_source = REF_REMOTE; resp->println("# Fuente: REMOTE"); return; }

  if (cmd == "?" || cmd == "s" || cmd == "status") {
    resp->println("# --- Parametros actuales ---");
    resp->print("#   Kp        = "); resp->println(Kp, 4);
    resp->print("#   Kd        = "); resp->println(Kd, 4);
    resp->print("#   Ki        = "); resp->println(Ki, 4);
    resp->print("#   N_DERIV   = "); resp->println(N_DERIV, 4);
    resp->print("#   BANDA_OK  = "); resp->print(BANDA_OK, 4); resp->println(" m");
    resp->print("#   U_COULOMB = "); resp->print(U_COULOMB, 4); resp->println(" rad");
    resp->print("#   ref_m     = "); resp->println(referencia_m, 4);
    resp->print("#   rango ref : ["); resp->print(REF_MIN_M, 3);
    resp->print(", "); resp->print(REF_MAX_M, 3); resp->println("] m");
    resp->print("#   fuente    : "); resp->println(ref_source == REF_POT ? "POT" : "REMOTE");
    return;
  }

  int sep = cmd.indexOf('=');
  if (sep < 0) sep = cmd.indexOf(' ');
  if (sep > 0) {
    String key = cmd.substring(0, sep);
    String val = cmd.substring(sep + 1);
    key.trim(); val.trim();
    float v = val.toFloat();

    if (key == "kp") { Kp = v; resp->print("# Kp = "); resp->println(Kp, 4); return; }
    if (key == "kd") { Kd = v; resp->print("# Kd = "); resp->println(Kd, 4); return; }
    if (key == "ki") { Ki = v; integral = 0.0f; resp->print("# Ki = "); resp->println(Ki, 4); return; }
    if (key == "nd") { N_DERIV = v; resp->print("# N_DERIV = "); resp->println(N_DERIV, 4); return; }
    if (key == "bk") { BANDA_OK = v; resp->print("# BANDA_OK = "); resp->print(BANDA_OK, 4); resp->println(" m"); return; }
    if (key == "cb" || key == "coulomb") { U_COULOMB = v; resp->print("# U_COULOMB = "); resp->println(U_COULOMB, 4); return; }
    if (key == "ref") {
      if (v < REF_MIN_M) v = REF_MIN_M;
      if (v > REF_MAX_M) v = REF_MAX_M;
      referencia_m = v; ref_source = REF_REMOTE;
      resp->print("# ref = "); resp->print(referencia_m, 4); resp->println(" m (REMOTE)");
      return;
    }
  }
  resp->print("# Comando desconocido: '"); resp->print(cmd); resp->println("'");
}
