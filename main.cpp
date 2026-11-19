// ============================================================
//  Apple — ESP8266 + TM1638 + AHT20 + BMP280 + MQ2 + MAX30102
//  + график в EEPROM (4 параметра, 180 точек)
//  + CSV лог в LittleFS (новый файл в 21:40, ротация 30 дней)
//  + экран 5 = APPLE + номер версии
//  + счётчик OTA-обновлений (apple N) в EEPROM
//  + давление BMP280 в мм рт.ст. + калибровка
//  + расширенная консоль (gpio, scan, sys, sound, fun)
//  + /exec и /api/exec без токена, /admin/exec — с токеном
// ============================================================

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
#include <TM1638plus.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include "passwords.h"

// ============================================================
//  НАСТРОЙКИ
// ============================================================
const char* WIFI_SSID  = CFG_WIFI_SSID;
const char* WIFI_PASS  = CFG_WIFI_PASS;
const char* ADMIN_USER = CFG_ADMIN_USER;
const char* ADMIN_PASS = CFG_ADMIN_PASS;

// ============================================================
//  ПИНЫ
// ============================================================
#define PIN_SDA     4
#define PIN_SCL     5
#define PIN_TM_STB  D5
#define PIN_TM_CLK  D6
#define PIN_TM_DIO  D7
#define MQ2_ANALOG  A0
#define MQ2_DIGITAL D0
#define LED_PIN     2
#define BUZZER_PIN  15

// ============================================================
//  ОБЪЕКТЫ
// ============================================================
ESP8266WebServer server(80);
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
MAX30105 particleSensor;
TM1638plus tm(PIN_TM_STB, PIN_TM_CLK, PIN_TM_DIO, true);

// ============================================================
//  СЕГМЕНТНЫЕ КОДЫ
// ============================================================
#define SEG_0 0x3F
#define SEG_1 0x06
#define SEG_2 0x5B
#define SEG_3 0x4F
#define SEG_4 0x66
#define SEG_5 0x6D
#define SEG_6 0x7D
#define SEG_7 0x07
#define SEG_8 0x7F
#define SEG_9 0x6F
#define SEG_C 0x61
#define SEG_n 0x54
#define SEG_O 0x3F
#define SEG_K 0x76
#define SEG_A 0x77
#define SEG_P 0x73
#define SEG_L 0x38
#define SEG_E 0x79
#define SEG_BLANK 0x00
#define SEG_DOT 0x80

// ============================================================
//  EEPROM layout
// ============================================================
#define EEPROM_SIZE       2048
#define EE_ADDR_MQ2THR    0
#define EE_ADDR_BRIGHT    2
#define EE_ADDR_TOKEN     4
#define EE_ADDR_MAGIC     63
#define EE_MAGIC          0x4F

#define EE_ADDR_CSV_DL    200
#define EE_ADDR_CSV_DIRTY 204
#define EE_ADDR_CSV_ACK   208
#define EE_ADDR_FWVER     212
#define EE_ADDR_PRESS_CAL 216   // int16_t, десятые доли мм рт.ст.

#define GRAPH_POINTS      180
#define EE_ADDR_GRAPH     256
#define EE_GRAPH_MAGIC    0x47
#define EE_GRAPH_HEADER   4
#define EE_GRAPH_DATA     (EE_ADDR_GRAPH + EE_GRAPH_HEADER)
#define EE_GRAPH_STRIDE   (GRAPH_POINTS * 2)

// ============================================================
//  ДАННЫЕ
// ============================================================
float gTemp = NAN;
int   gGasAnalog = 0;
bool  gGasDigital = false;
int   gGasPpm = 0;
float gHomeTemp = NAN, gHomeHum = NAN;
float gBmpPress = NAN;        // уже в мм рт.ст. (после калибровки)
float gBmpTemp  = NAN;
int   gMq2Threshold = 1000;
int16_t gPressCal = 0;        // смещение в десятых долях мм рт.ст.

float gOutTemp  = NAN;
float gOutPress = NAN;
unsigned long gLastWttr = 0;
const unsigned long WTTR_INTERVAL = 10UL * 60UL * 1000UL;

uint32_t irBuffer[100], redBuffer[100];
int32_t  bufIdx = 0;
int32_t  gHeartRate = 0, gSpO2 = 0;
int8_t   gValidHR = 0, gValidSpO2 = 0;
bool     gFinger = false;
uint32_t gWave[60];
int      gWaveIdx = 0;

const uint32_t FINGER_ON_THRESHOLD  = 30000;
const uint32_t FINGER_OFF_THRESHOLD = 15000;

uint32_t gIrBaseline = 0;
bool     gBaselineReady = false;
uint64_t gCalSum = 0;
int      gCalCount = 0;

int  currentScreen = 0;
bool ledEnabled = true;
int  brightnessLevel = 0;
const int brightnessMap[3] = {1, 3, 7};
const int brightnessPct[3] = {25, 50, 100};

bool          gDotBlinkState = true;
unsigned long gLastDotBlink  = 0;
const unsigned long DOT_BLINK_INTERVAL = 500;

int   cacheGasPpm = -1;
bool  cacheGasDigital = false;
int   cacheHomeTemp = -9999, cacheHomeHum = -9999;
int   cacheBmpPress = -1, cacheOutPress = -1;
int   cacheHour = -1, cacheMin = -1;
int   cacheOutTemp = -9999;
int   cacheScreen = -1;
int   cacheNyDay = -1, cacheNyHour = -1, cacheNyMin = -1, cacheNySec = -1;

uint32_t gUptime = 0;
unsigned long gLastSecond = 0;
unsigned long gLastSensor = 0;
bool gWifiAnnounced = false;
String gAuthToken = "";

// === ГРАФИК ===
struct GraphData {
  int16_t temp[GRAPH_POINTS];
  int16_t hum[GRAPH_POINTS];
  int16_t press[GRAPH_POINTS];
  int16_t gas[GRAPH_POINTS];
  uint16_t head;
  uint16_t count;
};
GraphData gGraph;
unsigned long gLastGraphSample = 0;
unsigned long gLastGraphSave   = 0;
const unsigned long GRAPH_INTERVAL      = 60UL * 1000UL;
const unsigned long GRAPH_SAVE_INTERVAL = 5UL * 60UL * 1000UL;

// === CSV ===
uint32_t gLastDownloadTime = 0;
bool     gCsvDirty = false;
uint32_t gCsvAckTime = 0;
String   gLogDate = "";
unsigned long gLastCleanup = 0;

// === ВЕРСИЯ ПРОШИВКИ ===
uint16_t gFwVersion = 1;
bool     gFwJustUpdated = false;

// ============================================================
//  ПРЕДВАРИТЕЛЬНЫЕ ОБЪЯВЛЕНИЯ
// ============================================================
void sensorsReadAhtBmp();
void drawScreen(int scr);
void drawDotOnly();
void drawCountdownDotOnly();
void max30102ResetBaseline();
void csvAppendSample();
void csvCleanupOld();
String currentLogDate();
void eepromSaveCsvState();
void eepromLoadCsvState();
void eepromSaveFwVersion();
void eepromLoadFwVersion();
void eepromSavePressCal();
void eepromLoadPressCal();
void graphLoadFromEEPROM();
void graphSaveFullToEEPROM();
void graphSample();

// ============================================================
//  HTML — ГЛАВНАЯ
// ============================================================
const char DASH_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Apple Sensor</title>
<style>
:root{--bg:#0a0f1c;--card:rgba(26,29,36,.8);--acc:#2a9d8f;--txt:#e8eaed;--dim:#8a94a6;--red:#e63946;--warn:#f4a261}
*{box-sizing:border-box}
body{margin:0;background:radial-gradient(1200px 600px at 20% -10%,#12203a 0%,var(--bg) 60%);color:var(--txt);font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;padding:16px}
h1{font-size:1.3rem;color:var(--acc);margin:0 0 14px;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
h1 .dot{width:10px;height:10px;border-radius:50%;background:var(--acc);box-shadow:0 0 12px var(--acc);animation:pulse 1.8s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin-bottom:14px}
.card{background:var(--card);border:1px solid #2a2f3a;border-radius:16px;padding:14px;backdrop-filter:blur(10px);position:relative;overflow:hidden}
.card .lbl{font-size:.72rem;color:var(--dim);text-transform:uppercase;letter-spacing:.08em}
.card .val{font-size:2rem;font-weight:700;margin-top:4px;line-height:1.1}
.card .unit{font-size:.9rem;color:var(--dim);font-weight:400}
.card .sub{font-size:.72rem;color:var(--dim);margin-top:6px}
.acc{color:var(--acc)}.red{color:var(--red)}.warn{color:var(--warn)}
#heart-card{grid-column:1/-1;display:grid;grid-template-columns:130px 1fr 200px;gap:16px;align-items:center;
  background:linear-gradient(135deg,rgba(230,57,70,.12),var(--card));border-color:rgba(230,57,70,.35)}
#heart-svg{width:120px;height:120px;filter:drop-shadow(0 0 14px rgba(230,57,70,.55));transition:transform .1s}
#heart-svg.beat{animation:beat .9s ease-in-out infinite}
@keyframes beat{0%,100%{transform:scale(1)}25%{transform:scale(1.12)}40%{transform:scale(.97)}55%{transform:scale(1.06)}}
#heart-info{display:flex;gap:26px;flex-wrap:wrap}
#heart-info .big{font-size:2.4rem;font-weight:800;line-height:1}
#heart-info .big.bpm{color:var(--red)}
#heart-info .big.spo2{color:var(--acc)}
#heart-info small{display:block;color:var(--dim);font-size:.72rem;letter-spacing:.08em;text-transform:uppercase}
#heart-status{margin-top:8px;font-size:.8rem;color:var(--dim)}
#heart-status .live{color:var(--red);font-weight:600;animation:pulse 1.2s infinite}
#wave{width:200px;height:80px;background:rgba(0,0,0,.3);border-radius:8px}
.actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:6px}
.btn,a.btn{display:inline-block;padding:10px 16px;border-radius:10px;background:var(--acc);color:#fff;text-decoration:none;font-size:.85rem;border:none;cursor:pointer}
.btn.ghost{background:transparent;border:1px solid #2a2f3a;color:var(--dim)}
footer{margin-top:18px;font-size:.72rem;color:var(--dim);text-align:center}
#chart{width:100%;height:260px;background:rgba(0,0,0,.25);border-radius:10px;margin-top:8px;display:block}
#legend{display:flex;gap:18px;flex-wrap:wrap;margin-top:10px;font-size:.78rem;color:var(--dim)}
.lg{display:inline-block;width:12px;height:3px;border-radius:2px;vertical-align:middle;margin-right:6px}
</style></head><body>
<h1><span class="dot"></span>🍎 Apple <span id="fwver" style="font-size:.9rem;color:var(--dim);font-weight:400">v…</span> <span style="font-size:.8rem;color:var(--dim);font-weight:400">ESP8266 · TM1638 · AHT20/BMP280 · MQ2 · MAX30102</span></h1>
<div class="grid">
  <div class="card" id="heart-card">
    <svg id="heart-svg" viewBox="0 0 100 100"><path d="M50 88C20 66 6 52 6 34c0-14 11-24 24-24 9 0 16 5 20 12 4-7 11-12 20-12 13 0 24 10 24 24 0 18-14 32-44 54z" fill="#e63946" stroke="#ff6b7a" stroke-width="2"/></svg>
    <div>
      <div id="heart-info">
        <div><div class="big bpm" id="hbpm">--</div><small>Пульс · BPM</small></div>
        <div><div class="big spo2" id="hspo2">--</div><small>SpO₂ · %</small></div>
      </div>
      <div id="heart-status">Ожидание данных с MAX30102…</div>
    </div>
    <canvas id="wave" width="200" height="80"></canvas>
  </div>
  <div class="card"><div class="lbl">Температура (AHT20)</div><div class="val acc" id="t-temp">--</div><div class="unit">°C</div><div class="sub" id="t-hum">Влаж. --%</div></div>
  <div class="card"><div class="lbl">Давление (BMP280)</div><div class="val acc" id="t-press">--</div><div class="sub" id="t-btmp">-- °C</div></div>
  <div class="card"><div class="lbl">Газ / дым (MQ2)</div><div class="val warn" id="t-mq2">--</div><div class="unit">ppm (расчёт)</div><div class="sub" id="t-mq2raw">ADC: --</div></div>
  <div class="card"><div class="lbl">TM1638</div><div class="val" id="t-tm">--</div><div class="unit">кнопки</div><div class="sub" id="t-tmled">LED: --</div></div>
  <div class="card"><div class="lbl">Улица (wttr.in)</div><div class="val" id="t-out">--</div><div class="unit">°C</div><div class="sub" id="t-outp">-- мм рт.ст.</div></div>
  <div class="card"><div class="lbl">Uptime</div><div class="val" id="t-up">--</div><div class="unit">ч</div><div class="sub" id="t-heap">Heap: -- KB</div></div>
</div>

<div class="card" style="margin-bottom:14px">
  <div class="lbl">История · 3 часа (шаг 1 мин)</div>
  <canvas id="chart"></canvas>
  <div id="legend">
    <span><span class="lg" style="background:#e63946"></span>Температура AHT20 · °C</span>
    <span><span class="lg" style="background:#2a9d8f"></span>Влажность · %</span>
    <span><span class="lg" style="background:#f4a261"></span>Давление BMP280 · мм рт.ст.</span>
    <span><span class="lg" style="background:#e9c46a"></span>Газ MQ2 · ppm</span>
  </div>
</div>

<div class="actions">
  <a class="btn" href="/admin">⚙ Админка</a>
  <a class="btn ghost" href="/admin/console">>_ Консоль</a>
  <button class="btn ghost" onclick="fetch('/api/refresh')">⟳ Обновить</button>
</div>
<footer>🍎 Apple <span id="fwver2">v…</span> · ESP8266 · <span id="ip">--</span></footer>
<script>
const $=id=>document.getElementById(id);
const cv=$('wave'),ctx=cv.getContext('2d');
let lastBpm=0;
function fmt(n,d){return (n===null||n===undefined||isNaN(n))?'--':Number(n).toFixed(d||0);}
function setHeart(bpm,spo2,finger){
  const hb=$('hbpm'),hs=$('hspo2'),svg=$('heart-svg'),st=$('heart-status');
  if(finger&&bpm>20&&bpm<220){
    hb.textContent=Math.round(bpm);
    if(Math.abs(bpm-lastBpm)>3){lastBpm=bpm;svg.style.animationDuration=(60/bpm).toFixed(2)+'s';}
    svg.classList.add('beat');
    st.innerHTML='<span class="live">● LIVE</span> палец обнаружен';
  }else{
    hb.textContent='--';svg.classList.remove('beat');
    st.textContent=finger?'Считаю…':'Нет пальца на датчике';
  }
  hs.textContent=(finger&&spo2>70&&spo2<=100)?Math.round(spo2):'--';
}
function drawWave(arr){
  ctx.clearRect(0,0,200,80);
  if(!arr||!arr.length)return;
  let mn=Math.min(...arr),mx=Math.max(...arr);
  if(mx-mn<100)mx=mn+100;
  ctx.beginPath();ctx.strokeStyle='#e63946';ctx.lineWidth=2;
  for(let i=0;i<arr.length;i++){
    const x=i/(arr.length-1)*200, y=80-((arr[i]-mn)/(mx-mn))*70-5;
    i?ctx.lineTo(x,y):ctx.moveTo(x,y);
  }
  ctx.stroke();
}
async function tick(){
  try{
    const d=await(await fetch('/api/data',{cache:'no-store'})).json();
    $('t-temp').textContent=fmt(d.aht.t,1);
    $('t-hum').textContent='Влаж. '+fmt(d.aht.h,0)+'%';
    $('t-press').textContent=fmt(d.bmp.p,0);
    $('t-btmp').textContent=fmt(d.bmp.t,1)+' °C';
    $('t-out').textContent=fmt(d.outt,1);
    $('t-outp').textContent=fmt(d.outp,0)+' мм рт.ст.';
    $('t-mq2').textContent=fmt(d.mq2.ppm,0);
    $('t-mq2raw').textContent='ADC: '+(d.mq2.raw??'--');
    $('t-tm').textContent=d.tm1638.keys;
    $('t-tmled').textContent='LED: 0x'+d.tm1638.leds.toString(16);
    $('t-up').textContent=(d.sys.uptime/3600).toFixed(1);
    $('t-heap').textContent='Heap: '+Math.round(d.sys.heap/1024)+' KB';
    $('ip').textContent=d.sys.ip||'--';
    if(d.sys.fwver){
      $('fwver').textContent='v'+d.sys.fwver;
      $('fwver2').textContent='v'+d.sys.fwver;
    }
    setHeart(d.heart.bpm,d.heart.spo2,d.heart.finger);
    drawWave(d.heart.wave);
  }catch(e){}
}
tick();setInterval(tick,1000);

const chart=$('chart'),cctx=chart.getContext('2d');
let graphData=null;
const CHART_COLORS=['#e63946','#2a9d8f','#f4a261','#e9c46a'];
function fmtVal(s,v){ return (s===0)?(v/10).toFixed(1):String(v); }
function drawChart(g){
  const W=chart.width,H=chart.height,PAD=10;
  cctx.clearRect(0,0,W,H);
  if(!g||!g.n){ cctx.fillStyle='#8a94a6'; cctx.font='14px sans-serif'; cctx.textAlign='center';
    cctx.fillText('Данные появятся через минуту…',W/2,H/2); cctx.textAlign='left'; return; }
  cctx.strokeStyle='rgba(255,255,255,.06)';cctx.lineWidth=1;
  for(let y=0;y<5;y++){const yy=PAD+(H-2*PAD)*y/4;
    cctx.beginPath();cctx.moveTo(PAD,yy);cctx.lineTo(W-PAD,yy);cctx.stroke();}
  const n=g.n;
  for(let s=0;s<4;s++){
    const arr=g.series[s]; let mn=Infinity,mx=-Infinity;
    for(const v of arr){if(v<mn)mn=v;if(v>mx)mx=v;}
    if(mx-mn<1)mx=mn+1;
    cctx.beginPath();cctx.strokeStyle=CHART_COLORS[s];cctx.lineWidth=2;cctx.globalAlpha=.9;
    for(let i=0;i<n;i++){
      const x=PAD+(W-2*PAD)*(n===1?0.5:i/(n-1));
      const y=H-PAD-((arr[i]-mn)/(mx-mn))*(H-2*PAD);
      i?cctx.lineTo(x,y):cctx.moveTo(x,y);
    }
    cctx.stroke();cctx.globalAlpha=1;
    cctx.fillStyle=CHART_COLORS[s];cctx.font='11px monospace';
    cctx.fillText('max '+fmtVal(s,mx), W-PAD-70, PAD+12+s*14);
  }
}
async function tickGraph(){
  try{
    const g=await(await fetch('/api/graph',{cache:'no-store'})).json();
    graphData=g;
    const W=chart.clientWidth||800;
    if(chart.width!==W){ chart.width=W; }
    chart.height=260;
    drawChart(g);
  }catch(e){}
}
tickGraph();setInterval(tickGraph,30000);
window.addEventListener('resize',()=>graphData&&drawChart(graphData));
</script>
</body></html>
)HTML";

// ============================================================
//  HTML — ЛОГИН
// ============================================================
const char LOGIN_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Apple · Вход</title>
<style>body{background:#0a0f1c;color:#e8eaed;font-family:sans-serif;display:flex;justify-content:center;align-items:center;height:100vh;margin:0}
.card{background:rgba(26,29,36,.8);padding:28px;border-radius:16px;max-width:320px;width:100%;backdrop-filter:blur(10px)}
h1{color:#2a9d8f;text-align:center}input{width:100%;padding:12px;margin:8px 0;border-radius:8px;border:1px solid #2a2f3a;background:#0f1115;color:#fff;box-sizing:border-box}
button{width:100%;padding:12px;margin-top:16px;background:#2a9d8f;color:#fff;border:none;border-radius:8px;cursor:pointer}.err{color:#e63946;text-align:center}</style>
</head><body><div class="card"><h1>🍎 Apple</h1><form method="POST" action="/login">
<input name="user" placeholder="Логин" required><input name="pass" type="password" placeholder="Пароль" required>
<button type="submit">Войти</button></form><p class="err" id="e" style="display:none">Неверный логин или пароль</p></div>
<script>if(location.search.includes('err'))document.getElementById('e').style.display='block';</script></body></html>
)HTML";

// ============================================================
//  HTML — АДМИНКА
// ============================================================
const char ADMIN_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Apple · Админка</title>
<style>body{margin:0;background:#0a0f1c;color:#e8eaed;font-family:'Segoe UI',system-ui,sans-serif;padding:16px}
h1{color:#2a9d8f;font-size:1.2rem}.card{background:rgba(26,29,36,.8);border:1px solid #2a2f3a;border-radius:16px;padding:18px;margin-bottom:14px;backdrop-filter:blur(10px)}
label{display:block;font-size:.8rem;color:#8a94a6;margin:10px 0 4px}input{width:100%;padding:10px;border-radius:8px;border:1px solid #2a2f3a;background:#0f1115;color:#fff;box-sizing:border-box}
button{padding:10px 16px;margin-top:14px;background:#2a9d8f;color:#fff;border:none;border-radius:8px;cursor:pointer}
.btn{display:inline-block;padding:10px 16px;border-radius:8px;background:transparent;border:1px solid #2a2f3a;color:#8a94a6;text-decoration:none;font-size:.85rem;margin-right:8px}
.btn.primary{background:#2a9d8f;color:#fff;border:none;padding:14px 24px;font-size:.95rem;margin-top:10px}
#fwbig{font-size:2rem;font-weight:800;color:#2a9d8f}
</style></head><body><h1>🍎 Apple · Админка</h1>

<div class="card">
  <h3>🍏 Версия прошивки</h3>
  <div id="fwbig">—</div>
  <p style="font-size:.75rem;color:#8a94a6;margin-top:6px">Инкрементится при каждом OTA-обновлении</p>
  <label>Установить вручную</label>
  <input id="fwinput" type="number" min="1" max="9999" value="1">
  <button onclick="setFw()">Установить</button>
</div>

<div class="card">
  <h3>📊 CSV лог (сутки с 20:11)</h3>
  <p style="font-size:.8rem;color:#8a94a6" id="csv-status">Загрузка…</p>
  <a class="btn primary" id="csv-dl" href="#" download style="display:inline-block">⬇ Скачать текущий CSV</a>
  <div id="csv-list" style="margin-top:14px;font-size:.8rem;color:#8a94a6"></div>
</div>

<div class="card"><h3>Настройки</h3>
<label>Порог MQ2 (ppm)</label><input id="mq2thr" type="number" value="1000">
<label>Яркость TM1638 (0=25%, 1=50%, 2=100%)</label><input id="tmbr" type="number" min="0" max="2" value="0">
<button onclick="save()">Сохранить</button></div>

<div class="card"><h3>Действия</h3>
<a class="btn" id="lnk-dash" href="/">← Дашборд</a>
<a class="btn" id="lnk-console" href="#">>_ Консоль</a>
<a class="btn" id="lnk-logout" href="#">Выйти</a>
<button onclick="if(confirm('Reboot?'))fetch(withT('/api/reboot'))">⟳ Reboot</button></div>

<script>
const TOKEN=new URLSearchParams(location.search).get('t')||'';
function withT(url){return TOKEN?url+(url.includes('?')?'&':'?')+'t='+encodeURIComponent(TOKEN):url;}
document.getElementById('lnk-console').href=withT('/admin/console');
document.getElementById('lnk-logout').href=withT('/logout');
document.getElementById('lnk-dash').href=withT('/');
async function save(){
  await fetch(withT('/api/save')+'&mq2thr='+document.getElementById('mq2thr').value+'&tmbr='+document.getElementById('tmbr').value);
  alert('Сохранено');
}
async function loadFw(){
  try{
    const d=await(await fetch(withT('/api/data'),{cache:'no-store'})).json();
    if(d.sys.fwver){
      document.getElementById('fwbig').textContent='apple '+d.sys.fwver;
      document.getElementById('fwinput').value=d.sys.fwver;
    }
  }catch(e){}
}
async function setFw(){
  const n=parseInt(document.getElementById('fwinput').value)||1;
  await fetch(withT('/api/fwver?n='+n));
  await loadFw();
  alert('Версия установлена: apple '+n);
}
async function loadLogs(){
  const st=document.getElementById('csv-status'), dl=document.getElementById('csv-dl'), list=document.getElementById('csv-list');
  try{
    const r=await fetch(withT('/api/logs'));
    if(!r.ok){ st.textContent='Нет доступа ('+r.status+')'; return; }
    const arr=await r.json();
    if(!arr.length){ st.textContent='Файлов пока нет (нужна синхронизация NTP)'; return; }
    arr.sort((a,b)=> b.name.localeCompare(a.name));
    const cur=arr[0];
    const name=cur.name.replace('/logs/','');
    st.textContent='Текущий: '+name+' · '+(cur.size/1024).toFixed(1)+' KB · всего '+arr.length+' файл(ов)';
    dl.href=withT('/api/log?f='+encodeURIComponent(name));
    dl.setAttribute('download', name);
    if(arr.length>1){
      list.innerHTML='<div style="margin-top:8px">Архив:</div>'+arr.slice(1,11).map(f=>{
        const n=f.name.replace('/logs/','');
        return `<div style="padding:4px 0"><a class="btn" style="padding:4px 10px;font-size:.75rem" href="${withT('/api/log?f='+encodeURIComponent(n))}" download>⬇ ${n} (${(f.size/1024).toFixed(1)} KB)</a></div>`;
      }).join('');
    }
  }catch(e){ st.textContent='Ошибка: '+e.message; }
}
loadFw();loadLogs();
</script></body></html>
)HTML";

// ============================================================
//  HTML — КОНСОЛЬ
// ============================================================
const char CONSOLE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Apple · Консоль</title>
<style>body{background:#0a0a0a;color:#00ff00;font-family:'Courier New',monospace;padding:20px;margin:0;height:100vh;box-sizing:border-box;display:flex;flex-direction:column}
#bar{display:flex;justify-content:space-between;margin-bottom:10px;padding-bottom:10px;border-bottom:1px solid #004400;font-size:.85rem}
#bar a{color:#888;text-decoration:none}#output{flex:1;overflow-y:auto;white-space:pre-wrap;font-size:14px;line-height:1.4}
#input-line{display:flex;margin-top:10px}#prompt{color:#00ff00;font-weight:bold;margin-right:8px}
#input{background:transparent;border:none;color:#00ff00;font-family:inherit;font-size:14px;flex:1;outline:none}
.err{color:#ff4444}.ok{color:#44ff44}#hint{font-size:.7rem;color:#446644;margin-left:auto}</style>
</head><body><div id="bar"><span>🍎 Apple <span id="ver">v…</span> Console</span><span><span id="hint">help · clear · ↑/↓</span> <a id="lnk-admin" href="/admin" style="margin-left:12px">← админка</a></span></div>
<div id="output"></div><div id="input-line"><span id="prompt">apple@esp&gt;</span><input id="input" autofocus autocomplete="off"></div>
<script>
const output=document.getElementById('output'),input=document.getElementById('input');
const history=[];let historyIdx=-1;
const TOKEN=new URLSearchParams(location.search).get('t')||'';
function withT(url){return TOKEN?url+(url.includes('?')?'&':'?')+'t='+encodeURIComponent(TOKEN):url;}
document.getElementById('lnk-admin').href=withT('/admin');
function append(t,c){if(t==='__CLEAR__'){output.innerHTML='';return;}const s=document.createElement('span');if(c)s.className=c;s.textContent=t+'\n';output.appendChild(s);output.scrollTop=output.scrollHeight;}
async function exec(cmd){if(!cmd.trim())return;
  if(cmd.trim()==='clear'){append('__CLEAR__');return;}
  append('apple@esp> '+cmd);history.push(cmd);historyIdx=history.length;
  try{const r=await fetch(withT('/admin/exec')+'&cmd='+encodeURIComponent(cmd));
    if(r.status===401){append('Доступ запрещён','err');return;}
    const txt=await r.text();
    append(txt, txt.startsWith('Неизвестная')?'err':'');
  }catch(e){append('Ошибка: '+e.message,'err');}}
input.addEventListener('keydown',e=>{if(e.key==='Enter'){const c=input.value;input.value='';exec(c);}
  else if(e.key==='ArrowUp'){e.preventDefault();if(historyIdx>0){historyIdx--;input.value=history[historyIdx];}}
  else if(e.key==='ArrowDown'){e.preventDefault();if(historyIdx<history.length-1){historyIdx++;input.value=history[historyIdx];}else{historyIdx=history.length;input.value='';}}});
fetch(withT('/api/data')).then(r=>r.json()).then(d=>{if(d.sys.fwver)document.getElementById('ver').textContent='v'+d.sys.fwver;}).catch(()=>{});
append('🍎 Apple Console — введите help');input.focus();
</script></body></html>
)HTML";

// ============================================================
//  EEPROM helpers
// ============================================================
void eepromSaveToken() {
  for (int i = 0; i < 16; i++) {
    EEPROM.write(EE_ADDR_TOKEN + i, i < (int)gAuthToken.length() ? gAuthToken[i] : 0);
  }
  EEPROM.write(EE_ADDR_TOKEN + 16, 0);
  EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.commit();
}

void eepromLoadToken() {
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC) { gAuthToken = ""; return; }
  char buf[17]; buf[16] = 0;
  for (int i = 0; i < 16; i++) buf[i] = EEPROM.read(EE_ADDR_TOKEN + i);
  buf[16] = 0;
  bool ok = true;
  for (int i = 0; i < 16; i++) {
    char c = buf[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) { ok = false; break; }
  }
  gAuthToken = ok ? String(buf) : String("");
}

void eepromSaveSettings() {
  EEPROM.put(EE_ADDR_MQ2THR, gMq2Threshold);
  EEPROM.write(EE_ADDR_BRIGHT, (uint8_t)brightnessLevel);
  EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.commit();
}

void eepromLoadSettings() {
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC) {
    gMq2Threshold = 1000;
    brightnessLevel = 0;
    return;
  }
  EEPROM.get(EE_ADDR_MQ2THR, gMq2Threshold);
  uint8_t br = EEPROM.read(EE_ADDR_BRIGHT);
  brightnessLevel = (br <= 2) ? br : 0;
}

void eepromSavePressCal() {
  EEPROM.put(EE_ADDR_PRESS_CAL, gPressCal);
  EEPROM.commit();
}

void eepromLoadPressCal() {
  int16_t v = 0;
  EEPROM.get(EE_ADDR_PRESS_CAL, v);
  if (v < -2000 || v > 2000) v = 0;
  gPressCal = v;
  Serial.printf("Press cal: %d.%d mmHg\n", gPressCal / 10, abs(gPressCal % 10));
}

void eepromSaveCsvState() {
  EEPROM.put(EE_ADDR_CSV_DL, gLastDownloadTime);
  EEPROM.put(EE_ADDR_CSV_ACK, gCsvAckTime);
  EEPROM.write(EE_ADDR_CSV_DIRTY, gCsvDirty ? 1 : 0);
  EEPROM.commit();
}

void eepromLoadCsvState() {
  EEPROM.get(EE_ADDR_CSV_DL, gLastDownloadTime);
  EEPROM.get(EE_ADDR_CSV_ACK, gCsvAckTime);
  gCsvDirty = EEPROM.read(EE_ADDR_CSV_DIRTY) == 1;
  if (gLastDownloadTime > 2000000000UL) gLastDownloadTime = 0;
  if (gCsvAckTime > 2000000000UL) gCsvAckTime = 0;
}

void eepromSaveFwVersion() {
  EEPROM.put(EE_ADDR_FWVER, gFwVersion);
  EEPROM.commit();
}

void eepromLoadFwVersion() {
  uint16_t v = 0;
  EEPROM.get(EE_ADDR_FWVER, v);
  if (v == 0xFFFF || v == 0 || v > 9999) {
    gFwVersion = 1;
    eepromSaveFwVersion();
  } else {
    gFwVersion = v;
  }
  Serial.printf("Firmware version: apple %u\n", gFwVersion);
}

// ============================================================
//  ГРАФИК — EEPROM
// ============================================================
void graphLoadFromEEPROM() {
  if (EEPROM.read(EE_ADDR_GRAPH) != EE_GRAPH_MAGIC) {
    memset(&gGraph, 0, sizeof(gGraph));
    Serial.println("Graph: empty (fresh)");
    return;
  }
  gGraph.head  = EEPROM.read(EE_ADDR_GRAPH + 1) | (EEPROM.read(EE_ADDR_GRAPH + 2) << 8);
  gGraph.count = EEPROM.read(EE_ADDR_GRAPH + 3) | (EEPROM.read(EE_ADDR_GRAPH + 4) << 8);
  if (gGraph.head  >= GRAPH_POINTS) gGraph.head  = 0;
  if (gGraph.count >  GRAPH_POINTS) gGraph.count = 0;
  int addr = EE_GRAPH_DATA;
  EEPROM.get(addr, gGraph.temp);  addr += EE_GRAPH_STRIDE;
  EEPROM.get(addr, gGraph.hum);   addr += EE_GRAPH_STRIDE;
  EEPROM.get(addr, gGraph.press); addr += EE_GRAPH_STRIDE;
  EEPROM.get(addr, gGraph.gas);
  Serial.printf("Graph loaded: %u points\n", gGraph.count);
}

void graphSaveHeaderToEEPROM() {
  EEPROM.write(EE_ADDR_GRAPH, EE_GRAPH_MAGIC);
  EEPROM.write(EE_ADDR_GRAPH + 1, gGraph.head & 0xFF);
  EEPROM.write(EE_ADDR_GRAPH + 2, (gGraph.head >> 8) & 0xFF);
  EEPROM.write(EE_ADDR_GRAPH + 3, gGraph.count & 0xFF);
  EEPROM.write(EE_ADDR_GRAPH + 4, (gGraph.count >> 8) & 0xFF);
}

void graphSaveFullToEEPROM() {
  graphSaveHeaderToEEPROM();
  int addr = EE_GRAPH_DATA;
  EEPROM.put(addr, gGraph.temp);  addr += EE_GRAPH_STRIDE;
  EEPROM.put(addr, gGraph.hum);   addr += EE_GRAPH_STRIDE;
  EEPROM.put(addr, gGraph.press); addr += EE_GRAPH_STRIDE;
  EEPROM.put(addr, gGraph.gas);
  EEPROM.commit();
  Serial.println("Graph saved to EEPROM");
}

void graphSample() {
  int i = gGraph.head;
  float t = isnan(gHomeTemp) ? 0 : gHomeTemp;
  float h = isnan(gHomeHum)  ? 0 : gHomeHum;
  float p = isnan(gBmpPress) ? 0 : gBmpPress;
  gGraph.temp[i]  = (int16_t)roundf(t * 10.0f);
  gGraph.hum[i]   = (int16_t)roundf(h);
  gGraph.press[i] = (int16_t)roundf(p);
  gGraph.gas[i]   = (int16_t)constrain(gGasPpm, 0, 2000);
  gGraph.head = (gGraph.head + 1) % GRAPH_POINTS;
  if (gGraph.count < GRAPH_POINTS) gGraph.count++;
}

// ============================================================
//  CSV — LittleFS
// ============================================================
String currentLogDate() {
  time_t now = time(nullptr);
  if (now < 1700000000) return "";
  now -= 20 * 3600 + 11 * 60;
  struct tm ti;
  localtime_r(&now, &ti);
  char buf[11];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
           ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday);
  return String(buf);
}

void csvAppendSample() {
  String date = currentLogDate();
  if (date.length() == 0) return;

  String path = "/logs/" + date + ".csv";
  bool isNew = !LittleFS.exists(path);
  File f = LittleFS.open(path, "a");
  if (!f) { Serial.println("CSV open fail"); return; }
  if (isNew) f.println("дата;давление_мм_рт_ст;темп_°C;влаж_%;газ_ppm");

  time_t now = time(nullptr);
  struct tm ti;
  localtime_r(&now, &ti);
  char ts[20];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d",
           ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
           ti.tm_hour, ti.tm_min, ti.tm_sec);

  char line[128];
  snprintf(line, sizeof(line), "%s;%.0f;%.1f;%.0f;%d",
           ts,
           isnan(gBmpPress) ? 0.0 : gBmpPress,
           isnan(gHomeTemp) ? 0.0 : gHomeTemp,
           isnan(gHomeHum)  ? 0.0 : gHomeHum,
           gGasPpm);
  f.println(line);
  f.close();

  if (date != gLogDate) {
    gLogDate = date;
    gCsvDirty = true;
    gLastDownloadTime = 0;
    gCsvAckTime = 0;
    eepromSaveCsvState();
    Serial.printf("CSV new period: %s\n", date.c_str());
  }
}

void csvCleanupOld() {
  String today = currentLogDate();
  if (today.length() == 0) return;
  time_t now = time(nullptr);
  Dir dir = LittleFS.openDir("/logs");
  while (dir.next()) {
    String name = dir.fileName();
    if (!name.endsWith(".csv")) continue;
    if (name.indexOf(today) >= 0) continue;
    int slash = name.lastIndexOf('/');
    String d = name.substring(slash + 1, slash + 11);
    struct tm ti = {};
    if (sscanf(d.c_str(), "%d-%d-%d", &ti.tm_year, &ti.tm_mon, &ti.tm_mday) != 3) continue;
    ti.tm_year -= 1900; ti.tm_mon -= 1;
    time_t ft = mktime(&ti);
    if (ft > 0 && (now - ft) > 30L * 86400L) {
      LittleFS.remove(name);
      Serial.printf("CSV removed old: %s\n", name.c_str());
    }
  }
}

// ============================================================
//  АВТОРИЗАЦИЯ
// ============================================================
bool isAuthorized() {
  if (gAuthToken.length() == 0) return false;
  if (server.hasHeader("Cookie")) {
    String c = server.header("Cookie");
    if (c.indexOf("auth=" + gAuthToken) >= 0) return true;
  }
  if (server.hasArg("t") && server.arg("t") == gAuthToken) return true;
  return false;
}

void ensureAuthCookie() {
  if (gAuthToken.length() == 0) return;
  if (!server.hasArg("t")) return;
  if (server.arg("t") != gAuthToken) return;
  server.sendHeader("Set-Cookie", "auth=" + gAuthToken + "; Path=/; Max-Age=86400");
}

void handleLoginForm() { server.send_P(200, "text/html", LOGIN_HTML); }

void handleLogin() {
  if (server.arg("user") == ADMIN_USER && server.arg("pass") == ADMIN_PASS) {
    char buf[32]; snprintf(buf, sizeof(buf), "%08X%08X", (unsigned)random(0x7FFFFFFF), (unsigned)random(0x7FFFFFFF));
    gAuthToken = String(buf);
    eepromSaveToken();
    server.sendHeader("Set-Cookie", "auth=" + gAuthToken + "; Path=/; Max-Age=86400");
    server.sendHeader("Location", "/admin?t=" + gAuthToken);
    server.send(302, "text/plain", "OK");
  } else {
    server.sendHeader("Location", "/login?err=1");
    server.send(302, "text/plain", "Fail");
  }
}

void handleLogout() {
  gAuthToken = "";
  eepromSaveToken();
  server.sendHeader("Set-Cookie", "auth=; Path=/; Max-Age=0");
  server.sendHeader("Location", "/login");
  server.send(302, "text/plain", "OK");
}

// ============================================================
//  ЧТЕНИЕ AHT/BMP  —  здесь ЕДИНСТВЕННЫЙ пересчёт Па → мм рт.ст.
// ============================================================
void sensorsReadAhtBmp() {
  sensors_event_t hum, temp;
  if (aht.getEvent(&hum, &temp)) {
    gHomeTemp = temp.temperature;
    gHomeHum  = hum.relative_humidity;
  }
  float rawPa = bmp.readPressure();
  float mmhg  = rawPa / 133.322f;
  mmhg += (float)gPressCal / 10.0f;
  gBmpPress = mmhg;
  gBmpTemp  = bmp.readTemperature();
}

// ============================================================
//  УЛИЧНАЯ ТЕМПЕРАТУРА  (wttr.in отдаёт гПа — 1.3332 верно)
// ============================================================
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClient client;
  HTTPClient http;
  String url = "http://wttr.in/Zelenogradsk?format=%t+%25P";
  if (!http.begin(client, url)) return;
  http.setTimeout(4000);
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    body.trim();
    int sp = body.indexOf(' ');
    String tPart = (sp > 0) ? body.substring(0, sp) : body;
    String pPart = (sp > 0) ? body.substring(sp + 1) : "";

    int deg = tPart.indexOf("°");
    String numT = (deg > 0) ? tPart.substring(0, deg) : tPart;
    numT.trim(); numT.replace("+", "");
    float t = numT.toFloat();
    if (!isnan(t) && t > -60.0f && t < 60.0f) {
      gOutTemp = t;
      Serial.print("OutTemp: "); Serial.println(gOutTemp);
    }

    int hp = pPart.indexOf("hPa");
    if (hp < 0) hp = pPart.indexOf("hpa");
    if (hp > 0) {
      String numP = pPart.substring(0, hp);
      numP.trim();
      float hpa = numP.toFloat();
      if (!isnan(hpa) && hpa > 800.0f && hpa < 1100.0f) {
        gOutPress = hpa / 1.3332f;
        Serial.print("OutPress: "); Serial.println(gOutPress);
      }
    }
  }
  http.end();
}

// ============================================================
//  ХЕНДЛЕРЫ
// ============================================================
void handleRoot() {
  ensureAuthCookie();
  server.send_P(200, "text/html", DASH_HTML);
}
void handleAdmin() {
  if (!isAuthorized()) { handleLoginForm(); return; }
  ensureAuthCookie();
  server.send_P(200, "text/html", ADMIN_HTML);
}
void handleConsole() {
  if (!isAuthorized()) { handleLoginForm(); return; }
  ensureAuthCookie();
  server.send_P(200, "text/html", CONSOLE_HTML);
}

void handleData() {
  String j; j.reserve(1800);
  j += "{\"aht\":{\"t\":" + String(gHomeTemp, 2) + ",\"h\":" + String(gHomeHum, 2) + "},";
  j += "\"bmp\":{\"t\":" + String(gBmpTemp, 2) + ",\"p\":" + String(gBmpPress, 1) + "},";
  j += "\"outt\":" + String(gOutTemp, 1) + ",";
  j += "\"outp\":" + String(gOutPress, 1) + ",";
  j += "\"mq2\":{\"raw\":" + String(gGasAnalog) + ",\"ppm\":" + String(gGasPpm) +
       ",\"thr\":" + String(gMq2Threshold) + "},";
  j += "\"heart\":{\"bpm\":" + String(gHeartRate) + ",\"spo2\":" + String(gSpO2) +
       ",\"finger\":" + String(gFinger ? "true" : "false") + ",\"wave\":[";
  for (int i = 0; i < 60; i++) { j += String(gWave[(gWaveIdx + i) % 60]); if (i < 59) j += ","; }
  j += "]},";
  j += "\"tm1638\":{\"keys\":" + String(tm.readButtons()) + ",\"leds\":" + String(ledEnabled ? 0xFF : 0) + "},";
  j += "\"sys\":{\"uptime\":" + String(gUptime) + ",\"heap\":" + String(ESP.getFreeHeap()) +
       ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"fwver\":" + String(gFwVersion) + "}";
  j += "}";
  server.send(200, "application/json", j);
}

void handleGraph() {
  String j; j.reserve(3600);
  j += "{\"n\":" + String(gGraph.count) + ",\"series\":[";
  const int16_t* series[4] = {gGraph.temp, gGraph.hum, gGraph.press, gGraph.gas};
  for (int s = 0; s < 4; s++) {
    j += "[";
    for (int k = 0; k < gGraph.count; k++) {
      int idx = (gGraph.count < GRAPH_POINTS)
                  ? k
                  : (gGraph.head + k) % GRAPH_POINTS;
      j += String(series[s][idx]);
      if (k < gGraph.count - 1) j += ",";
    }
    j += "]";
    if (s < 3) j += ",";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleRefresh() {
  sensorsReadAhtBmp();
  fetchWeather();
  server.send(200, "text/plain", "OK");
}

void handleReboot() {
  eepromSaveCsvState();
  graphSaveFullToEEPROM();
  server.send(200, "text/plain", "OK");
  delay(300);
  ESP.restart();
}

void handleSave() {
  if (server.hasArg("mq2thr")) {
    gMq2Threshold = constrain(server.arg("mq2thr").toInt(), 0, 10000);
  }
  if (server.hasArg("tmbr")) {
    int lvl = constrain(server.arg("tmbr").toInt(), 0, 2);
    brightnessLevel = lvl;
    tm.brightness(brightnessMap[brightnessLevel]);
  }
  eepromSaveSettings();
  server.send(200, "text/plain", "OK");
}

void handleSetFwVer() {
  if (!isAuthorized()) { server.send(401, "text/plain", "Unauthorized"); return; }
  if (!server.hasArg("n")) { server.send(400, "text/plain", "no n"); return; }
  int n = server.arg("n").toInt();
  if (n < 1 || n > 9999) { server.send(400, "text/plain", "bad n"); return; }
  gFwVersion = (uint16_t)n;
  eepromSaveFwVersion();
  if (currentScreen == 5) drawScreen(5);
  server.send(200, "text/plain", "OK");
}

void handleListLogs() {
  if (!isAuthorized()) { server.send(401, "text/plain", "Unauthorized"); return; }
  String j = "[";
  Dir dir = LittleFS.openDir("/logs");
  bool first = true;
  while (dir.next()) {
    if (!first) j += ",";
    first = false;
    j += "{\"name\":\"" + dir.fileName() + "\",\"size\":" + String(dir.fileSize()) + "}";
  }
  j += "]";
  server.send(200, "application/json", j);
}

void handleDownloadLog() {
  if (!isAuthorized()) { server.send(401, "text/plain", "Unauthorized"); return; }
  if (!server.hasArg("f")) { server.send(400, "text/plain", "no file"); return; }
  String name = server.arg("f");
  if (name.indexOf("..") >= 0 || name.indexOf('/') >= 0) {
    server.send(400, "text/plain", "bad name"); return;
  }
  String path = "/logs/" + name;
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "not found"); return; }
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "open fail"); return; }
  server.streamFile(f, "text/csv");
  f.close();

  gLastDownloadTime = time(nullptr);
  gCsvDirty = false;
  gCsvAckTime = 0;
  eepromSaveCsvState();
}

// ============================================================
//  ТЕРМИНАЛ  —  /admin/exec требует токен, /exec и /api/exec — нет
// ============================================================
void handleTerminalExec() {
  String uri = server.uri();
  bool needAuth = (uri == "/admin/exec");
  if (needAuth && !isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!server.hasArg("cmd")) { server.send(400, "text/plain", "no cmd"); return; }
  String cmd = server.arg("cmd"); cmd.trim();
  String out = "";

  if (cmd == "help") {
    out += "Apple Console — команды:\n";
    out += "  -- система --\n";
    out += "  status           - статус системы\n";
    out += "  version          - версия (apple N)\n";
    out += "  version set N    - установить версию N\n";
    out += "  uptime           - аптайм\n";
    out += "  heap             - RAM + фрагментация\n";
    out += "  free             - RAM (total/used/free)\n";
    out += "  df               - Flash (size/used/free)\n";
    out += "  flash            - Flash / скетч / свободно\n";
    out += "  cpu              - частота CPU\n";
    out += "  mac              - MAC-адрес\n";
    out += "  uname / uname -a - версия ядра/SDK/chip\n";
    out += "  ifconfig         - сеть (IP/mask/gw/dns/mac)\n";
    out += "  top              - сводка CPU/RAM/WiFi/Flash\n";
    out += "  dmesg            - лог загрузки\n";
    out += "  ls               - виртуальная ФС\n";
    out += "  ps               - список задач\n";
    out += "  -- Wi-Fi --\n";
    out += "  wifi             - SSID/IP/RSSI/MAC/канал\n";
    out += "  scan             - сканировать Wi-Fi\n";
    out += "  scan best        - лучшая сеть\n";
    out += "  scan json        - JSON-вывод сетей\n";
    out += "  -- GPIO --\n";
    out += "  gpio P V         - пин = 0|1\n";
    out += "  gpio read P      - прочитать пин\n";
    out += "  gpio mode P in|out - режим пина\n";
    out += "  gpio all         - все GPIO\n";
    out += "  pin P 0|1        - быстрый доступ (0=GND,1=VCC)\n";
    out += "  pin gN gnd|vcc   - замыкание на GND/VCC\n";
    out += "  adc              - A0\n";
    out += "  -- I2C --\n";
    out += "  i2c scan         - сканировать I2C\n";
    out += "  aht read         - AHT20\n";
    out += "  bmp read         - BMP280 (мм рт.ст.)\n";
    out += "  -- погода --\n";
    out += "  out read         - улица (T/P)\n";
    out += "  weather          - уличная T/P\n";
    out += "  weather update   - принудительно обновить\n";
    out += "  press            - оба давления\n";
    out += "  cal              - показать калибровку\n";
    out += "  cal N            - смещение ±200 мм рт.ст.\n";
    out += "  cal 0            - сбросить калибровку\n";
    out += "  -- датчики --\n";
    out += "  mq2              - MQ2\n";
    out += "  max read         - MAX30102\n";
    out += "  reset            - сброс baseline MAX\n";
    out += "  -- TM1638 --\n";
    out += "  tm1638 read      - кнопки\n";
    out += "  tm1638 write N V - LED N = V\n";
    out += "  screen N         - экран N (0,1,2,3,4,5)\n";
    out += "  bright N         - яркость 0|1|2\n";
    out += "  -- звук --\n";
    out += "  tone F MS        - писк (частота, мс)\n";
    out += "  beep             - короткий писк\n";
    out += "  melody           - мелодия\n";
    out += "  led 0|1          - встроенный LED\n";
    out += "  -- приколы --\n";
    out += "  dice             - кубик\n";
    out += "  coin             - орёл/решка\n";
    out += "  -- CSV/график --\n";
    out += "  logs             - список CSV\n";
    out += "  csv sample       - записать строку CSV\n";
    out += "  graph            - состояние графика\n";
    out += "  graph clear      - очистить график\n";
    out += "  -- прочее --\n";
    out += "  reboot           - перезагрузка\n";
    out += "  clear            - очистить консоль\n";
  }
  else if (cmd == "version") {
    out += "🍎 apple " + String(gFwVersion) + "\n";
  }
  else if (cmd.startsWith("version set ")) {
    int n;
    if (sscanf(cmd.c_str(), "version set %d", &n) == 1 && n >= 1 && n <= 9999) {
      gFwVersion = (uint16_t)n;
      eepromSaveFwVersion();
      if (currentScreen == 5) drawScreen(5);
      out += "Версия установлена: apple " + String(gFwVersion) + "\n";
    } else out += "Использование: version set <1-9999>\n";
  }
  else if (cmd == "status") {
    out += "===== STATUS =====\n";
    out += "IP: " + WiFi.localIP().toString() + "\n";
    out += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    out += "Uptime: " + String(gUptime) + " s\n";
    out += "Heap: " + String(ESP.getFreeHeap()) + " B\n";
    out += "Version: apple " + String(gFwVersion) + "\n";
    out += "Screen: " + String(currentScreen) + "\n";
    out += "Bright: " + String(brightnessPct[brightnessLevel]) + "%\n";
    out += "Press: " + String(gBmpPress, 1) + " mmHg (cal " +
           String(gPressCal / 10.0f, 1) + ")\n";
    out += "MQ2: raw=" + String(gGasAnalog) + " ppm=" + String(gGasPpm) +
           " thr=" + String(gMq2Threshold) + "\n";
    out += "CSV: date=" + gLogDate + " dirty=" + String(gCsvDirty ? "yes" : "no") + "\n";
    out += "Graph: " + String(gGraph.count) + "/" + String(GRAPH_POINTS) + " points\n";
    out += "==================\n";
  }
  else if (cmd == "uptime") {
    uint32_t s = gUptime;
    uint32_t d = s / 86400; s %= 86400;
    uint32_t h = s / 3600;  s %= 3600;
    uint32_t m = s / 60;    s %= 60;
    out += "Uptime: " + String(d) + "d " + String(h) + "h " + String(m) + "m " + String(s) + "s\n";
  }
  else if (cmd == "heap") {
    out += "Свободно RAM: " + String(ESP.getFreeHeap()) + " байт\n";
    out += "Фрагментация: " + String(ESP.getHeapFragmentation()) + "%\n";
  }
  else if (cmd == "free") {
    int freeHeap = ESP.getFreeHeap();
    int totalHeap = 81920;
    out += "              total        used        free\n";
    out += "Mem:       " + String(totalHeap) + "       " + String(totalHeap - freeHeap) + "       " + String(freeHeap) + "\n";
    out += "Frag:      " + String(ESP.getHeapFragmentation()) + " %\n";
  }
  else if (cmd == "df") {
    out += "Filesystem      Size    Used    Free\n";
    out += "Flash           " + String(ESP.getFlashChipSize() / 1024) + "K   " +
           String(ESP.getSketchSize() / 1024) + "K   " +
           String(ESP.getFreeSketchSpace() / 1024) + "K\n";
  }
  else if (cmd == "flash") {
    out += "Flash: " + String(ESP.getFlashChipSize() / 1024) + " КБ\n";
    out += "Sketch: " + String(ESP.getSketchSize() / 1024) + " КБ\n";
    out += "Свободно: " + String(ESP.getFreeSketchSpace() / 1024) + " КБ\n";
  }
  else if (cmd == "cpu") { out += "CPU: " + String(ESP.getCpuFreqMHz()) + " МГц\n"; }
  else if (cmd == "mac") { out += "MAC: " + WiFi.macAddress() + "\n"; }
  else if (cmd == "uname" || cmd.startsWith("uname ")) {
    out += "ESP8266 Apple (Arduino core " + String(ESP.getCoreVersion()) + ")\n";
    out += "CPU: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    out += "SDK: " + String(ESP.getSdkVersion()) + "\n";
    out += "Chip ID: " + String(ESP.getChipId(), HEX) + "\n";
    out += "Flash: " + String(ESP.getFlashChipSize() / 1024) + " KB\n";
  }
  else if (cmd == "ifconfig") {
    out += "wlan0: flags=UP\n";
    out += "  inet " + WiFi.localIP().toString() + "\n";
    out += "  mask " + WiFi.subnetMask().toString() + "\n";
    out += "  gw   " + WiFi.gatewayIP().toString() + "\n";
    out += "  dns  " + WiFi.dnsIP().toString() + "\n";
    out += "  mac  " + WiFi.macAddress() + "\n";
    out += "  ssid " + WiFi.SSID() + "\n";
    out += "  rssi " + String(WiFi.RSSI()) + " dBm\n";
  }
  else if (cmd == "top") {
    out += "top - " + String(millis() / 1000) + "s uptime\n";
    out += "CPU: " + String(ESP.getCpuFreqMHz()) + " MHz\n";
    out += "RAM: " + String(ESP.getFreeHeap() / 1024) + "K free / " +
           String(81920 / 1024) + "K total\n";
    out += "WiFi: " + String(WiFi.RSSI()) + " dBm\n";
    out += "Flash: " + String(ESP.getFlashChipSize() / 1024) + "K\n";
    out += "Tasks: init wifi web ota weather tm1638 max30102\n";
  }
  else if (cmd == "dmesg") {
    out += "[0.000] Apple boot\n";
    out += "[0.012] AHT20 " + String(isnan(gHomeTemp) ? "not found" : "OK") + "\n";
    out += "[0.024] BMP280 " + String(isnan(gBmpPress) ? "not found" : "OK") + "\n";
    out += "[0.036] MAX30102 " + String(gBaselineReady ? "OK" : "calibrating") + "\n";
    out += "[0.048] TM1638 OK\n";
    out += "[0.060] WiFi " + String(WiFi.status() == WL_CONNECTED ? "connected" : "not connected") + "\n";
    out += "[0.072] Web server started\n";
    out += "[0.084] OTA ready\n";
  }
  else if (cmd == "ls") {
    out += "Виртуальная ФС ESP8266:\n";
    out += "  /            (root)\n";
    out += "  /api         (виртуальный)\n";
    out += "  /api/data\n";
    out += "  /api/graph\n";
    out += "  /api/logs\n";
    out += "  /admin\n";
    out += "  /admin/console\n";
    out += "  /logs        (CSV)\n";
  }
  else if (cmd == "ps") {
    out += " PID  NAME             STATE\n";
    out += "   1  init             run\n";
    out += "   2  wifi             " + String(WiFi.status() == WL_CONNECTED ? "run" : "down") + "\n";
    out += "   3  web-server       run\n";
    out += "   4  ota              run\n";
    out += "   5  weather-task     run\n";
    out += "   6  tm1638-task      run\n";
    out += "   7  max30102-task    run\n";
    out += "   8  csv-task         run\n";
  }
  else if (cmd == "wifi") {
    out += "SSID: " + WiFi.SSID() + "\n";
    out += "IP: " + WiFi.localIP().toString() + "\n";
    out += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
    out += "MAC: " + WiFi.macAddress() + "\n";
    out += "Канал: " + String(WiFi.channel()) + "\n";
  }
  else if (cmd == "scan") {
    out += "Сканирование Wi-Fi...\n";
    int n = WiFi.scanNetworks();
    if (n == 0) out += "Сетей не найдено\n";
    else {
      for (int i = 0; i < n; i++) {
        out += String(i + 1) + ". " + WiFi.SSID(i) +
               " (" + String(WiFi.RSSI(i)) + " dBm, ch " + String(WiFi.channel(i)) + ")\n";
      }
    }
    WiFi.scanDelete();
  }
  else if (cmd == "scan best") {
    out += "Поиск лучшей сети...\n";
    int n = WiFi.scanNetworks();
    if (n == 0) out += "Сетей не найдено\n";
    else {
      int best = 0;
      for (int i = 1; i < n; i++) if (WiFi.RSSI(i) > WiFi.RSSI(best)) best = i;
      out += "Лучшая: " + WiFi.SSID(best) + "\n";
      out += "RSSI: " + String(WiFi.RSSI(best)) + " dBm\n";
      out += "Канал: " + String(WiFi.channel(best)) + "\n";
    }
    WiFi.scanDelete();
  }
  else if (cmd == "scan json") {
    out += "{\n  \"networks\": [\n";
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      out += "    {\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) +
             ",\"channel\":" + String(WiFi.channel(i)) + "}";
      if (i < n - 1) out += ",";
      out += "\n";
    }
    out += "  ]\n}\n";
    WiFi.scanDelete();
  }
  else if (cmd == "i2c scan") {
    Wire.begin(PIN_SDA, PIN_SCL);
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) out += "0x" + String(a, HEX) + "\n";
    }
    if (out.length() == 0) out = "Ничего не найдено\n";
  }
  else if (cmd == "aht read") {
    sensorsReadAhtBmp();
    out += "AHT20: T=" + String(gHomeTemp, 2) + " C  H=" + String(gHomeHum, 2) + " %\n";
  }
  else if (cmd == "bmp read") {
    sensorsReadAhtBmp();
    out += "BMP280: T=" + String(gBmpTemp, 2) + " C  P=" + String(gBmpPress, 1) + " mmHg\n";
  }
  else if (cmd == "out read") {
    fetchWeather();
    out += "OutTemp: " + String(gOutTemp, 1) + " C\n";
    out += "OutPress: " + String(gOutPress, 1) + " mmHg\n";
  }
  else if (cmd == "weather") {
    out += "T=" + (isnan(gOutTemp) ? String("--") : String(gOutTemp, 1)) + " C\n";
    out += "P=" + (isnan(gOutPress) ? String("--") : String(gOutPress, 1)) + " mmHg\n";
  }
  else if (cmd == "weather update") {
    fetchWeather();
    out += "OK\n";
    out += "T=" + (isnan(gOutTemp) ? String("--") : String(gOutTemp, 1)) + " C\n";
    out += "P=" + (isnan(gOutPress) ? String("--") : String(gOutPress, 1)) + " mmHg\n";
  }
  else if (cmd == "press") {
    out += "BMP280 (дом):  " + String(gBmpPress, 1) + " mmHg\n";
    out += "wttr.in (улица): " + String(gOutPress, 1) + " mmHg\n";
  }
  else if (cmd == "cal") {
    out += "Калибровка давления: " + String(gPressCal / 10.0f, 1) + " мм рт.ст.\n";
    out += "Текущее: " + String(gBmpPress, 1) + " мм рт.ст.\n";
    out += "Использование: cal <N> — смещение в мм рт.ст. (например, cal 2 или cal -1.5)\n";
  }
  else if (cmd.startsWith("cal ")) {
    float n = cmd.substring(4).toFloat();
    if (n < -200.0f || n > 200.0f) {
      out += "Слишком большое смещение (допустимо ±200 мм рт.ст.)\n";
    } else {
      gPressCal = (int16_t)roundf(n * 10.0f);
      eepromSavePressCal();
      sensorsReadAhtBmp();
      out += "Калибровка установлена: " + String(gPressCal / 10.0f, 1) + " мм рт.ст.\n";
      out += "Новое давление: " + String(gBmpPress, 1) + " мм рт.ст.\n";
    }
  }
  else if (cmd == "mq2") {
    out += "MQ2: raw=" + String(gGasAnalog) + "  ppm=" + String(gGasPpm) +
           "  порог=" + String(gMq2Threshold) + "\n";
  }
  else if (cmd == "max read") {
    out += "MAX30102: BPM=" + String(gHeartRate) + "  SpO2=" + String(gSpO2) +
           "  finger=" + String(gFinger ? "yes" : "no") + "\n";
    out += "IR baseline=" + String(gIrBaseline) + " (ready=" + String(gBaselineReady ? "yes" : "no") + ")\n";
  }
  else if (cmd == "reset") {
    max30102ResetBaseline();
    out += "MAX30102 baseline сброшен\n";
  }
  else if (cmd == "tm1638 read") {
    out += "Buttons: 0x" + String(tm.readButtons(), HEX) + "\n";
  }
  else if (cmd.startsWith("tm1638 write ")) {
    int n, v;
    if (sscanf(cmd.c_str(), "tm1638 write %d %d", &n, &v) == 2 && n >= 0 && n < 8) {
      tm.setLED(n, v ? 1 : 0);
      out += "OK\n";
    } else out += "Использование: tm1638 write <0-7> <0|1>\n";
  }
  else if (cmd.startsWith("screen ")) {
    int n;
    if (sscanf(cmd.c_str(), "screen %d", &n) == 1 &&
        (n == 0 || n == 1 || n == 2 || n == 3 || n == 4 || n == 5)) {
      currentScreen = n; drawScreen(n); cacheScreen = n;
      out += "Экран " + String(n) + "\n";
    } else out += "Использование: screen <0|1|2|3|4|5>\n";
  }
  else if (cmd.startsWith("bright ")) {
    int n;
    if (sscanf(cmd.c_str(), "bright %d", &n) == 1 && n >= 0 && n <= 2) {
      brightnessLevel = n;
      tm.brightness(brightnessMap[brightnessLevel]);
      out += "Яркость: " + String(brightnessPct[brightnessLevel]) + "%\n";
    } else out += "Использование: bright <0|1|2>\n";
  }
  else if (cmd == "gpio all") {
    out += "Состояние всех GPIO:\n";
    for (int pin = 0; pin <= 16; pin++) {
      if (pin == 6 || pin == 7 || pin == 8 || pin == 9 || pin == 10 || pin == 11) continue;
      out += "  GPIO" + String(pin) + " = " + String(digitalRead(pin)) + "\n";
    }
  }
  else if (cmd.startsWith("gpio mode ")) {
    int pin; char mode[8];
    if (sscanf(cmd.c_str(), "gpio mode %d %7s", &pin, mode) == 2) {
      if (pin >= 0 && pin <= 16) {
        pinMode(pin, (strcmp(mode, "in") == 0) ? INPUT : OUTPUT);
        out += "GPIO" + String(pin) + " режим: " + String(mode) + "\n";
      } else out += "Ошибка: пин 0-16\n";
    } else out += "Использование: gpio mode <пин> <in|out>\n";
  }
  else if (cmd.startsWith("gpio read ")) {
    int pin;
    if (sscanf(cmd.c_str(), "gpio read %d", &pin) == 1) {
      if (pin >= 0 && pin <= 16) out += "GPIO" + String(pin) + " = " + String(digitalRead(pin)) + "\n";
      else out += "Ошибка: пин 0-16\n";
    } else out += "Использование: gpio read <пин>\n";
  }
  else if (cmd.startsWith("gpio ")) {
    int pin, val;
    if (sscanf(cmd.c_str(), "gpio %d %d", &pin, &val) == 2) {
      if (pin >= 0 && pin <= 16) {
        pinMode(pin, OUTPUT); digitalWrite(pin, val);
        out += "GPIO" + String(pin) + " = " + String(val) + "\n";
      } else out += "Ошибка: пин 0-16\n";
    } else out += "Использование: gpio <пин> <0|1>\n";
  }
  else if (cmd.startsWith("pin g") && (cmd.endsWith(" gnd") || cmd.endsWith(" vcc"))) {
    int gpos = cmd.indexOf('g');
    int spaceAfter = cmd.indexOf(' ', gpos);
    if (spaceAfter > gpos + 1) {
      int pin = cmd.substring(gpos + 1, spaceAfter).toInt();
      bool toGnd = cmd.endsWith(" gnd");
      if (pin >= 0 && pin <= 16 && !(pin >= 6 && pin <= 11)) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, toGnd ? LOW : HIGH);
        out += "GPIO" + String(pin) + " -> " + (toGnd ? "GND (LOW)" : "VCC (HIGH)") + "\n";
      } else out += "Ошибка: пин 0-16 (кроме 6-11)\n";
    } else out += "Использование: pin g<N> <gnd|vcc>\n";
  }
  else if (cmd.startsWith("pin ")) {
    int pin, val;
    if (sscanf(cmd.c_str(), "pin %d %d", &pin, &val) == 2) {
      if (pin >= 0 && pin <= 16) {
        pinMode(pin, OUTPUT); digitalWrite(pin, val ? LOW : HIGH);
        out += "Пин " + String(pin) + " = " + (val ? "GND (LOW)" : "VCC (HIGH)") + "\n";
      } else out += "Ошибка: пин 0-16\n";
    } else out += "Использование: pin <пин> <0|1>\n";
  }
  else if (cmd == "adc") { out += "A0=" + String(analogRead(MQ2_ANALOG)) + "\n"; }
  else if (cmd.startsWith("tone ")) {
    int freq, ms;
    if (sscanf(cmd.c_str(), "tone %d %d", &freq, &ms) == 2) {
      tone(BUZZER_PIN, freq, ms);
      out += "Tone: " + String(freq) + " Гц, " + String(ms) + " мс\n";
    } else out += "Использование: tone <частота> <мс>\n";
  }
  else if (cmd == "melody") {
    int notes[] = {262, 294, 330, 349, 392, 440, 494, 523};
    for (int i = 0; i < 8; i++) { tone(BUZZER_PIN, notes[i], 150); delay(200); }
    noTone(BUZZER_PIN);
    out += "Мелодия сыграна\n";
  }
  else if (cmd == "beep") { tone(BUZZER_PIN, 800, 100); out += "Бип!\n"; }
  else if (cmd.startsWith("led ")) {
    int val = cmd.substring(4).toInt();
    ledEnabled = val ? true : false;
    digitalWrite(LED_PIN, val ? LOW : HIGH);
    out += "LED: " + String(val) + "\n";
  }
  else if (cmd == "dice") { out += "Выпало: " + String(random(1, 7)) + "\n"; }
  else if (cmd == "coin") { out += String(random(0, 2) == 0 ? "Орел" : "Решка") + "\n"; }
  else if (cmd == "logs") {
    Dir dir = LittleFS.openDir("/logs");
    while (dir.next()) {
      out += dir.fileName() + " (" + String(dir.fileSize() / 1024.0, 1) + " KB)\n";
    }
    if (out.length() == 0) out = "Пусто\n";
  }
  else if (cmd == "csv sample") {
    csvAppendSample();
    out += "CSV sample добавлен\n";
  }
  else if (cmd == "graph") {
    out += "Points: " + String(gGraph.count) + "/" + String(GRAPH_POINTS) + "\n";
    out += "Head: " + String(gGraph.head) + "\n";
  }
  else if (cmd == "graph clear") {
    memset(&gGraph, 0, sizeof(gGraph));
    graphSaveFullToEEPROM();
    out += "График очищен\n";
  }
  else if (cmd == "reboot") { server.send(200, "text/plain", "Rebooting..."); delay(500); ESP.restart(); return; }
  else { out = "Неизвестная команда: " + cmd + "\n"; }

  server.send(200, "text/plain", out);
}

// ============================================================
//  MAX30102
// ============================================================
void max30102ResetBaseline() {
  gIrBaseline = 0; gBaselineReady = false;
  gCalSum = 0; gCalCount = 0;
  gFinger = false; gHeartRate = 0; gSpO2 = 0;
  gValidHR = 0; gValidSpO2 = 0; bufIdx = 0;
}

void max30102Poll() {
  if (!particleSensor.available()) return;
  uint32_t ir  = particleSensor.getFIFOIR();
  uint32_t red = particleSensor.getFIFORed();
  particleSensor.nextSample();

  bool fingerNow = gFinger;
  if (!gFinger) { if (ir > FINGER_ON_THRESHOLD)  fingerNow = true; }
  else          { if (ir < FINGER_OFF_THRESHOLD) fingerNow = false; }

  if (fingerNow != gFinger) {
    gFinger = fingerNow;
    if (!gFinger) { gHeartRate = 0; gSpO2 = 0; gValidHR = 0; gValidSpO2 = 0; bufIdx = 0; }
    else { bufIdx = 0; }
  }
  if (!gFinger) {
    gCalSum += ir; gCalCount++;
    if (gCalCount >= 100) {
      gIrBaseline = gCalSum / gCalCount; gBaselineReady = true;
      gCalSum = 0; gCalCount = 0;
    }
  }
  irBuffer[bufIdx]  = ir;
  redBuffer[bufIdx] = red;
  gWave[gWaveIdx]   = ir;
  gWaveIdx = (gWaveIdx + 1) % 60;
  bufIdx++;
  if (bufIdx > 100) bufIdx = 100;
  if (!gFinger) return;

  static int fastCount = 0;
  if (++fastCount >= 10 && bufIdx >= 20) {
    fastCount = 0;
    int len = (bufIdx < 20) ? bufIdx : 20;
    int start = bufIdx - len;
    if (start < 0) start = 0;
    uint32_t irMin = 0xFFFFFFFF, irMax = 0, redMin = 0xFFFFFFFF, redMax = 0;
    uint64_t irSum = 0, redSum = 0;
    for (int i = start; i < bufIdx; i++) {
      uint32_t vIr = irBuffer[i], vRed = redBuffer[i];
      irSum += vIr; redSum += vRed;
      if (vIr < irMin) irMin = vIr;
      if (vIr > irMax) irMax = vIr;
      if (vRed < redMin) redMin = vRed;
      if (vRed > redMax) redMax = vRed;
    }
    float dcIr  = (float)irSum / len;
    float dcRed = (float)redSum / len;
    float acIr  = (float)(irMax - irMin);
    float acRed = (float)(redMax - redMin);
    if (dcIr > 0 && dcRed > 0 && acIr > 0) {
      float ratio = (acRed / dcRed) / (acIr / dcIr);
      int spo2 = (int)(104.0f - 17.0f * ratio);
      if (spo2 > 100) spo2 = 100;
      if (spo2 < 70)  spo2 = 70;
      gSpO2 = spo2; gValidSpO2 = 1;
    }
  }
  if (bufIdx >= 100) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer,
      &gSpO2, &gValidSpO2, &gHeartRate, &gValidHR);
    bufIdx = 0;
    if (!gValidHR)  gHeartRate = 0;
  }
}

// ============================================================
//  TM1638 — draw
// ============================================================
uint8_t digitSeg(char c, bool dot = false) {
  uint8_t s = SEG_BLANK;
  switch (c) {
    case '0': s = SEG_0; break; case '1': s = SEG_1; break;
    case '2': s = SEG_2; break; case '3': s = SEG_3; break;
    case '4': s = SEG_4; break; case '5': s = SEG_5; break;
    case '6': s = SEG_6; break; case '7': s = SEG_7; break;
    case '8': s = SEG_8; break; case '9': s = SEG_9; break;
    case 'C': s = SEG_C; break; case 'n': s = SEG_n; break;
    case 'A': s = SEG_A; break; case 'P': s = SEG_P; break;
    case 'L': s = SEG_L; break; case 'E': s = SEG_E; break;
    default: s = SEG_BLANK;
  }
  return dot ? (s | SEG_DOT) : s;
}

bool getNewYearRemain(int &days, int &hours, int &mins, int &secs) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;
  struct tm ti;
  localtime_r(&now, &ti);
  struct tm ny = ti;
  ny.tm_mon  = 0; ny.tm_mday = 1;
  ny.tm_hour = 0; ny.tm_min  = 0; ny.tm_sec = 0;
  ny.tm_isdst = -1;
  ny.tm_year = ti.tm_year + 1;
  time_t nyTime = mktime(&ny);
  if (nyTime <= now) { ny.tm_year++; nyTime = mktime(&ny); }
  long diff = (long)(nyTime - now);
  if (diff < 0) diff = 0;
  days  = diff / 86400; diff %= 86400;
  hours = diff / 3600;  diff %= 3600;
  mins  = diff / 60;    secs  = diff % 60;
  return true;
}

void drawScreen(int scr) {
  for (int i = 0; i < 8; i++) tm.display7Seg(i, SEG_BLANK);

  if (scr == 0) {
    struct tm ti;
    int h = 0, m = 0;
    if (getLocalTime(&ti)) { h = ti.tm_hour; m = ti.tm_min; }
    cacheHour = h; cacheMin = m;
    char hh[3], mm[3];
    snprintf(hh, sizeof(hh), "%02d", h);
    snprintf(mm, sizeof(mm), "%02d", m);
    tm.display7Seg(0, digitSeg(hh[0]));
    tm.display7Seg(1, digitSeg(hh[1], gDotBlinkState));
    tm.display7Seg(2, digitSeg(mm[0]));
    tm.display7Seg(3, digitSeg(mm[1]));
    int t = isnan(gHomeTemp) ? 0 : (int)round(gHomeTemp);
    if (t > 99) t = 99;
    if (t < -9) t = -9;
    cacheOutTemp = t;
    char tt[4]; snprintf(tt, sizeof(tt), "%d", t);
    int len = strlen(tt);
    for (int i = 0; i < len; i++) tm.display7Seg(7 - len + i, digitSeg(tt[i]));
    tm.display7Seg(7, SEG_C);
  }
  else if (scr == 1) {
    cacheGasPpm = gGasPpm;
    cacheGasDigital = gGasDigital;
    char gg[5]; snprintf(gg, sizeof(gg), "%d", gGasPpm);
    int len = strlen(gg);
    for (int i = 0; i < len && i < 4; i++) tm.display7Seg(i, digitSeg(gg[i]));
    tm.display7Seg(5, SEG_O);
    tm.display7Seg(6, gGasDigital ? SEG_n : SEG_K);
  }
  else if (scr == 2) {
    int t = isnan(gHomeTemp) ? 0 : (int)round(gHomeTemp);
    if (t > 99) t = 99;
    if (t < -9) t = -9;
    cacheHomeTemp = t;
    char tt[4]; snprintf(tt, sizeof(tt), "%d", t);
    int len = strlen(tt);
    for (int i = 0; i < len; i++) tm.display7Seg(i, digitSeg(tt[i]));
    tm.display7Seg(len, SEG_C);
    int h = isnan(gHomeHum) ? 0 : (int)round(gHomeHum);
    if (h > 99) h = 99;
    cacheHomeHum = h;
    char hh[4]; snprintf(hh, sizeof(hh), "%d", h);
    int hlen = strlen(hh);
    for (int i = 0; i < hlen; i++) tm.display7Seg(7 - hlen + i, digitSeg(hh[i]));
    tm.display7Seg(7, SEG_n);
  }
  else if (scr == 3) {
    int p = isnan(gBmpPress) ? 0 : (int)round(gBmpPress);
    if (p > 9999) p = 9999;
    if (p < 0)    p = 0;
    cacheBmpPress = p;
    char pp[6]; snprintf(pp, sizeof(pp), "%d", p);
    int len = strlen(pp);
    for (int i = 0; i < len && i < 7; i++) tm.display7Seg(i, digitSeg(pp[i]));
    tm.display7Seg(7, SEG_P);
  }
  else if (scr == 4) {
    int d, h, m, s;
    if (getNewYearRemain(d, h, m, s)) {
      cacheNyDay = d; cacheNyHour = h; cacheNyMin = m; cacheNySec = s;
      char dd[3], hh[3], mm[3], ss[3];
      snprintf(dd, sizeof(dd), "%02d", d > 99 ? 99 : d);
      snprintf(hh, sizeof(hh), "%02d", h);
      snprintf(mm, sizeof(mm), "%02d", m);
      snprintf(ss, sizeof(ss), "%02d", s);
      tm.display7Seg(0, digitSeg(dd[0]));
      tm.display7Seg(1, digitSeg(dd[1], true));
      tm.display7Seg(2, digitSeg(hh[0]));
      tm.display7Seg(3, digitSeg(hh[1], true));
      tm.display7Seg(4, digitSeg(mm[0]));
      tm.display7Seg(5, digitSeg(mm[1], gDotBlinkState));
      tm.display7Seg(6, digitSeg(ss[0]));
      tm.display7Seg(7, digitSeg(ss[1]));
    }
  }
  else if (scr == 5) {
    tm.display7Seg(1, SEG_A);
    tm.display7Seg(2, SEG_P);
    tm.display7Seg(3, SEG_P);
    tm.display7Seg(4, SEG_L);
    tm.display7Seg(5, SEG_E);
    uint16_t v = gFwVersion;
    if (v < 10) {
      tm.display7Seg(7, digitSeg('0' + v));
    } else if (v < 100) {
      tm.display7Seg(6, digitSeg('0' + (v / 10)));
      tm.display7Seg(7, digitSeg('0' + (v % 10)));
    } else {
      tm.display7Seg(6, digitSeg('0' + ((v / 10) % 10)));
      tm.display7Seg(7, digitSeg('0' + (v % 10)));
    }
  }

  cacheScreen = scr;
}

void drawDotOnly() {
  if (currentScreen != 0) return;
  struct tm ti; int h = 0;
  if (getLocalTime(&ti)) { h = ti.tm_hour; }
  char hh[3]; snprintf(hh, sizeof(hh), "%02d", h);
  tm.display7Seg(1, digitSeg(hh[1], gDotBlinkState));
}

void drawCountdownDotOnly() {
  if (currentScreen != 4) return;
  int d, h, m, s;
  if (getNewYearRemain(d, h, m, s)) {
    char mm[3]; snprintf(mm, sizeof(mm), "%02d", m);
    tm.display7Seg(5, digitSeg(mm[1], gDotBlinkState));
  }
}

void drawScreenIfChanged() {
  bool need = false;
  if (currentScreen == 0) {
    struct tm ti;
    int h = 0, m = 0;
    if (getLocalTime(&ti)) { h = ti.tm_hour; m = ti.tm_min; }
    int t = isnan(gHomeTemp) ? 0 : (int)round(gHomeTemp);
    if (h != cacheHour || m != cacheMin || t != cacheOutTemp) need = true;
  }
  else if (currentScreen == 1) {
    if (gGasPpm != cacheGasPpm || gGasDigital != cacheGasDigital) need = true;
  }
  else if (currentScreen == 2) {
    int t = isnan(gHomeTemp) ? 0 : (int)round(gHomeTemp);
    int h = isnan(gHomeHum) ? 0 : (int)round(gHomeHum);
    if (t != cacheHomeTemp || h != cacheHomeHum) need = true;
  }
  else if (currentScreen == 3) {
    int p = isnan(gBmpPress) ? 0 : (int)round(gBmpPress);
    if (p != cacheBmpPress) need = true;
  }
  else if (currentScreen == 4) {
    int d, h, m, s;
    if (getNewYearRemain(d, h, m, s)) {
      if (d != cacheNyDay || h != cacheNyHour || m != cacheNyMin || s != cacheNySec) need = true;
    }
  }
  if (need || cacheScreen != currentScreen) drawScreen(currentScreen);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Apple Sensor boot ===");

  EEPROM.begin(EEPROM_SIZE);
  eepromLoadSettings();
  eepromLoadToken();
  eepromLoadCsvState();
  eepromLoadFwVersion();
  eepromLoadPressCal();
  graphLoadFromEEPROM();

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format();
    LittleFS.begin();
  }
  if (!LittleFS.exists("/logs")) LittleFS.mkdir("/logs");

  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MQ2_DIGITAL, INPUT);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!aht.begin()) Serial.println("AHT20 not found");
  if (!bmp.begin(0x76)) { if (!bmp.begin(0x77)) Serial.println("BMP280 not found"); }
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found");
  } else {
    particleSensor.setup(0x7F, 4, 2, 100, 411, 4096);
  }

  tm.displayBegin();
  tm.brightness(brightnessMap[brightnessLevel]);

  for (int i = 0; i < 8; i++) tm.display7Seg(i, SEG_BLANK);
  tm.display7Seg(1, SEG_A);
  tm.display7Seg(2, SEG_P);
  tm.display7Seg(3, SEG_P);
  tm.display7Seg(4, SEG_L);
  tm.display7Seg(5, SEG_E);
  char vbuf[6]; snprintf(vbuf, sizeof(vbuf), "%u", gFwVersion);
  int vlen = strlen(vbuf);
  for (int i = 0; i < vlen && i < 2; i++) {
    tm.display7Seg(7 - (vlen - 1 - i), digitSeg(vbuf[i]));
  }
  delay(2000);
  for (int i = 0; i < 8; i++) tm.display7Seg(i, SEG_BLANK);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  randomSeed(ESP.getCycleCount() ^ micros());

  ArduinoOTA.setHostname("Apple-Sensor");
  ArduinoOTA.setPassword("101026");
  ArduinoOTA.onStart([]() {
    tm.displayText("OTA     ");
    Serial.println("OTA start");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onEnd([]() {
    gFwVersion++;
    eepromSaveFwVersion();
    Serial.printf("OTA done → apple %u\n", gFwVersion);
    tm.displayText("OK      ");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error[%u]\n", error);
    tm.displayText("OTA ERR ");
  });
  ArduinoOTA.begin();

  server.on("/",                handleRoot);
  server.on("/login",  HTTP_GET,  handleLoginForm);
  server.on("/login",  HTTP_POST, handleLogin);
  server.on("/logout",          handleLogout);
  server.on("/admin",           handleAdmin);
  server.on("/admin/console",   handleConsole);
  server.on("/admin/exec",      handleTerminalExec);
  server.on("/exec",            handleTerminalExec);   // без токена
  server.on("/api/exec",        handleTerminalExec);   // без токена
  server.on("/api/data",        handleData);
  server.on("/api/graph",       handleGraph);
  server.on("/api/logs",        handleListLogs);
  server.on("/api/log",         handleDownloadLog);
  server.on("/api/refresh",     handleRefresh);
  server.on("/api/reboot",      handleReboot);
  server.on("/api/save",        handleSave);
  server.on("/api/fwver",       handleSetFwVer);

  server.begin();
  Serial.println("HTTP started");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  if (!gWifiAnnounced) {
    if (WiFi.status() == WL_CONNECTED) {
      gWifiAnnounced = true;
      Serial.println("WiFi: " + WiFi.localIP().toString());
      currentScreen = 0;
      drawScreen(currentScreen);
      fetchWeather();
      gLastWttr = millis();
      graphSample();
      gLastGraphSample = millis();
    }
  }

  if (gWifiAnnounced) {
    if (millis() - gLastSecond >= 1000) {
      gLastSecond = millis();
      gUptime++;
    }

    if (millis() - gLastWttr >= WTTR_INTERVAL) {
      gLastWttr = millis();
      fetchWeather();
    }

    uint8_t buttons = tm.readButtons();
    if (buttons) {
      for (int i = 0; i < 8; i++) {
        if (buttons & (1 << i)) {
          if (i == 0)      currentScreen = 0;
          else if (i == 1) currentScreen = 1;
          else if (i == 2) currentScreen = 2;
          else if (i == 3) currentScreen = 3;
          else if (i == 4) currentScreen = 5;
          else if (i == 6) {
            if (gCsvDirty) {
              gCsvDirty = false;
              gCsvAckTime = 0;
              eepromSaveCsvState();
              Serial.println("CSV alarm cleared manually");
            }
          }
          else if (i == 7) {
            brightnessLevel = (brightnessLevel + 1) % 3;
            tm.brightness(brightnessMap[brightnessLevel]);
          }
          delay(200);
        }
      }
    }

    if (millis() - gLastDotBlink >= DOT_BLINK_INTERVAL) {
      gLastDotBlink = millis();
      gDotBlinkState = !gDotBlinkState;
      if (currentScreen == 0) drawDotOnly();
      else if (currentScreen == 4) drawCountdownDotOnly();
    }

    drawScreenIfChanged();

    if (millis() - gLastGraphSample >= GRAPH_INTERVAL) {
      gLastGraphSample = millis();
      graphSample();
      csvAppendSample();
      if (millis() - gLastGraphSave >= GRAPH_SAVE_INTERVAL) {
        gLastGraphSave = millis();
        graphSaveFullToEEPROM();
      }
    }

    if (millis() - gLastCleanup >= 86400000UL) {
      gLastCleanup = millis();
      csvCleanupOld();
    }
  }

  if (millis() - gLastSensor >= 2000) {
    gLastSensor = millis();
    gGasAnalog  = analogRead(MQ2_ANALOG);
    gGasDigital = digitalRead(MQ2_DIGITAL);
    gGasPpm     = map(gGasAnalog, 0, 1024, 0, 2000);
    sensorsReadAhtBmp();
    gTemp = gHomeTemp;
  }

  max30102Poll();

  static uint8_t lastLedMask = 0xFF;
  uint8_t ledMask = (1 << currentScreen);
  if (ledMask != lastLedMask) {
    lastLedMask = ledMask;
    for (int i = 0; i < 8; i++) tm.setLED(i, (ledMask >> i) & 1);
  }

  delay(20);
}