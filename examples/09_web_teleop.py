"""网页遥控 CyberDog 2：浏览器看狗的视角，键盘控制行走，雷达防撞报警。连接成功后狗会自动站立。

运行：
    python examples/09_web_teleop.py          # 按提示在终端输入 IP、账号、密码
    python examples/09_web_teleop.py --mock   # 没有真机时用模拟房间测试（A/D 横移不改变雷达画面）
    默认显示机身姿态，狗上收不到 odom_out 时程序报错退出；加上 --no-attitude 可以关闭。

雷达、超声波和底部 TOF 只报警，不会干预运动。雷达和超声波只能看到狗的前方，报警时在视频上拉横幅并鸣响；TOF 只拉横幅。
雷达、超声波和 TOF 没开时库会自动打开，打开后仍收不到数据时程序直接报错退出。

按键（浏览器页面需要处于焦点）：
    W / S    前进 / 后退
    A / D    左移 / 右移
    ← / →    左转 / 右转
    Shift    按住时奔跑（配合上面的按键，切换为快速步态和奔跑速度）
    空格     按住时四脚离地跳跃（hop 步态，可配合方向键）
    Esc      急停，急停后程序退出


松开所有按键狗就停下。网页卡死或断网时，死人开关会在 0.5 秒内让狗停下。
控制接口没有登录验证，默认只监听本机；--bind 0.0.0.0 会让局域网内所有人都能控制这只狗。

按 Ctrl+C 停止。
"""
import argparse
import getpass
import importlib
import json
import math
import os
import sys
import threading
import time
from http.server import ThreadingHTTPServer

from numba.cuda.printimpl import int_print_impl

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from cyberdog2 import CyberDog  # noqa: E402

live = importlib.import_module("08_http_live")  # 文件名以数字开头，不能用 import 语句

CONTROL_PERIOD = 0.1  # 控制线程的周期（秒）
DEADMAN = 0.5         # set_velocity 的死人开关（秒）
STALE_AFTER = 0.3     # 这么久没收到网页的按键状态，就当作没有按键（秒）
SCAN_POLL = 0.03      # 检查新雷达帧的周期（秒）
SCAN_MAX_AGE = 0.5    # 雷达帧超过这个年龄就不算新鲜（秒）
SCAN_TIMEOUT = 3.0    # 这么久等不到新鲜的雷达帧、超声或 TOF 读数就抛 SensorTimeout（秒）
ULTRASONIC_MAX_AGE = 1.0  # 超声读数超过这个年龄就不算新鲜（秒）
TOF_MAX_AGE = 1.0         # TOF 读数超过这个年龄就不算新鲜（秒）
ATTITUDE_MAX_AGE = 1.0    # 姿态读数超过这个年龄就不算新鲜（秒）

RUN_KEY = "ShiftLeft"  # 网页把左右 Shift 都按这个上报
HOP_KEY = "Space"

# 浏览器 KeyboardEvent.code -> 对 (vx, vy, wz) 的方向
KEY_AXES = {
    RUN_KEY: (0, 0, 0),  # 本身不产生运动，只切换到奔跑
    HOP_KEY: (0, 0, 0),         # 同上，切换到四脚离地跳跃
    "KeyW": (1, 0, 0),
    "KeyS": (-1, 0, 0),
    "KeyA": (0, 1, 0),
    "KeyD": (0, -1, 0),
    "ArrowLeft": (0, 0, 1),
    "ArrowRight": (0, 0, -1),
}

PAGE = """<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CyberDog 遥控</title>
<style>
  [hidden] { display: none !important; }
  body { margin: 0; background: #111; color: #ddd; font: 15px system-ui, sans-serif; text-align: center; overflow: hidden; }
  #main { display: grid; grid-template-columns: minmax(230px, 1fr) minmax(0, 2.2fr) minmax(230px, 1fr); gap: 8px; padding: 8px; height: 100vh; box-sizing: border-box; }
  .col { display: flex; flex-direction: column; gap: 8px; min-width: 0; min-height: 0; }
  #videoBox { position: relative; flex: 1 1 0; min-height: 0; background: #000; overflow: hidden; }
  /* 画面贴着顶部，这样横幅总是压在视频上 */
  img { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain; object-position: top; }
  #banners { position: absolute; top: 0; left: 0; right: 0; }
  .banner { padding: 8px; font-size: 22px; font-weight: 700; color: #fff; background: rgba(198, 40, 40, 0.88); font-variant-numeric: tabular-nums; }
  .banner.yellow { color: #111; background: rgba(255, 179, 0, 0.9); }
  .banner.red { animation: blink 0.5s steps(2) infinite; }
  .panel { display: flex; flex-direction: column; min-height: 0; background: #1a1a1a; border: 1px solid #333; border-radius: 8px; padding: 6px 8px; }
  .panel.grow { flex: 1 1 0; }
  #batteryText { font-weight: 600; font-variant-numeric: tabular-nums; }
  #batteryTrack { height: 6px; margin-top: 4px; border-radius: 3px; background: #333; overflow: hidden; }
  #batteryLevel { height: 100%; width: 0; background: #66bb6a; }
  #battery.yellow #batteryLevel { background: #ffca28; }
  #battery.red #batteryLevel { background: #ef5350; }
  #battery.red #batteryText { color: #ef5350; }
  .panel h2 { margin: 0 0 4px; font-size: 14px; font-weight: 600; color: #aaa; }
  canvas { display: block; width: 100%; flex: 1 1 0; min-height: 0; }
  .attitude { display: flex; justify-content: center; align-items: center; gap: 14px; }
  #horizon { flex: none; width: 96px; height: 96px; }
  #attitudeText { white-space: pre-line; text-align: left; font-size: 15px; font-weight: 600; font-variant-numeric: tabular-nums; }
  @media (max-width: 820px) {
    body { overflow: auto; }
    #main { grid-template-columns: minmax(0, 1fr); height: auto; }
    #videoBox { flex: none; aspect-ratio: 4 / 3; }
    .panel.grow { flex: none; }
    canvas { flex: none; aspect-ratio: 2 / 1.08; }
    #horizon { aspect-ratio: auto; }
  }
  .tools { display: flex; justify-content: center; align-items: center; gap: 6px; margin-top: 4px; flex-wrap: wrap; }
  .tool { padding: 4px 10px; color: #ddd; background: #2a2a2a; border: 1px solid #555; border-radius: 5px; cursor: pointer; font: inherit; font-size: 13px; }
  .tool.on { background: #1565c0; border-color: #42a5f5; color: #fff; }
  #nearest { font-size: 18px; font-weight: 700; font-variant-numeric: tabular-nums; }
  #nearest.red { color: #ef5350; }
  #nearest.yellow { color: #ffca28; }
  #nearest.green { color: #66bb6a; }
  #ultrasonic { font-size: 13px; font-weight: 600; font-variant-numeric: tabular-nums; }
  #ultrasonic.alarm { color: #ef5350; }
  @keyframes blink { 50% { opacity: 0.55; } }
  #keys { display: flex; justify-content: center; align-items: stretch; gap: 6px; flex-wrap: wrap; }
  .key { min-width: 44px; padding: 5px 6px; font-size: 14px; border: 1px solid #555; border-radius: 6px; }
  .key.on { background: #2e7d32; border-color: #4caf50; color: #fff; }
  #estop { padding: 5px 18px; font-size: 15px; color: #fff; background: #c62828; border: 0; border-radius: 6px; cursor: pointer; }
  #estop:disabled { background: #555; cursor: default; }
  #status { font-size: 13px; }
  #status.bad { color: #ef5350; }
</style>
</head>
<body>
<div id="main">
<div class="col">
<section class="panel" id="battery">
  <div id="batteryText">电池 --</div>
  <div id="batteryTrack"><div id="batteryLevel"></div></div>
</section>
<section class="panel grow">
  <h2 title="头下方、腹部下方各左右一个传感器，每块 8×8">TOF（越红越近）</h2>
  <canvas id="tof"></canvas>
</section>
<section class="panel" id="attitudePanel" hidden>
  <h2 title="横滚右侧下沉为正，俯仰低头为正，航向左转为正">姿态</h2>
  <div class="attitude">
    <canvas id="horizon"></canvas>
    <div id="attitudeText"></div>
  </div>
</section>
</div>
<div class="col">
<div id="videoBox">
  <img id="video" src="/stream.mjpg" alt="狗的视角">
  <div id="banners">
    <div class="banner" id="frontBanner" hidden></div>
    <div class="banner red" id="tofBanner" hidden></div>
  </div>
</div>
<div id="keys">
  <div class="key" data-code="KeyW">W 前进</div>
  <div class="key" data-code="KeyS">S 后退</div>
  <div class="key" data-code="KeyA">A 左移</div>
  <div class="key" data-code="KeyD">D 右移</div>
  <div class="key" data-code="ArrowLeft">← 左转</div>
  <div class="key" data-code="ArrowRight">→ 右转</div>
  <div class="key" data-code="ShiftLeft">Shift 奔跑</div>
  <div class="key" data-code="Space">空格 跳跃</div>
  <button id="estop">急停（Esc）</button>
</div>
<div id="status">已连接，松开按键狗会停下</div>
</div>
<div class="col">
<section class="panel grow">
  <h2>防撞雷达（前方 180°）</h2>
  <canvas id="fan"></canvas>
  <div class="tools">
    <span id="nearest">--</span>
    <span id="ultrasonic">超声波 --</span>
    <button class="tool" id="mute">静音</button>
  </div>
</section>
<section class="panel grow">
  <h2>2D 雷达</h2>
  <canvas id="map"></canvas>
  <div class="tools">
    <button class="tool mode on" data-mode="points">点云</button>
    <button class="tool mode" data-mode="lines">连线</button>
    <span>&nbsp;</span>
    <button class="tool range" data-range="2">2 m</button>
    <button class="tool range on" data-range="5">5 m</button>
    <button class="tool range" data-range="10">10 m</button>
  </div>
</section>
</div>
</div>
<script>
const CODES = new Set(Array.from(document.querySelectorAll(".key"), el => el.dataset.code));
const held = new Set();
const statusEl = document.getElementById("status");
const estopEl = document.getElementById("estop");
let seq = Date.now();  // 请求可能乱序到达，服务端只接受序号更大的
let dead = false;

function fail(text) {
  dead = true;
  held.clear();
  render();
  statusEl.textContent = text;
  statusEl.className = "bad";
  estopEl.disabled = true;
}

function post(path, body) {
  return fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) })
    .then(r => { if (!r.ok) throw new Error("HTTP " + r.status); });
}

function render() {
  document.querySelectorAll(".key").forEach(el => el.classList.toggle("on", held.has(el.dataset.code)));
}

function send() {
  if (dead) return;
  post("/control", { seq: ++seq, keys: Array.from(held) }).catch(e => fail("与程序的连接断开：" + e.message));
}

function releaseAll() {
  if (held.size === 0) return;
  held.clear();
  render();
  send();
}

function estop() {
  if (dead) return;
  post("/estop", {}).catch(() => {});
  fail("已急停，程序退出。重新运行脚本才能继续控制");
}

function keyCode(e) {
  return e.code === "ShiftRight" ? "ShiftLeft" : e.code;
}

window.addEventListener("keydown", e => {
  if (e.code === "Escape") {
    e.preventDefault();
    if (!e.repeat) estop();
    return;
  }
  const code = keyCode(e);
  if (!CODES.has(code)) return;
  e.preventDefault();
  if (e.repeat || dead) return;
  held.add(code);
  render();
  send();
});

window.addEventListener("keyup", e => {
  if (held.delete(keyCode(e))) {
    render();
    send();
  }
});

window.addEventListener("blur", releaseAll);
document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAll(); });
estopEl.addEventListener("click", estop);
document.getElementById("video").addEventListener("error", () => { if (!dead) fail("视频流断开，请看终端里的报错"); });
setInterval(() => { if (held.size > 0) send(); }, 100);

// ---------- 雷达。角度以狗的前方为 0，左转为正 ----------
const RED_RANGE = 0.3, YELLOW_RANGE = 0.6, GREEN_RANGE = 1.2;  // 报警距离（m）
const SECTORS = 9;
const COLOR_RED = "#ef5350", COLOR_YELLOW = "#ffca28", COLOR_GREEN = "#66bb6a", COLOR_DIM = "#2c2c2c";
const COLOR_GRID = "#3a3a3a", COLOR_LABEL = "#888", COLOR_POINT = "#4dd0e1";
const LINE_BREAK = 0.3;  // 相邻两点距离差超过这个值就不连线（m）
const GRID_STEP = { 2: 0.5, 5: 1, 10: 2 };
const fanEl = document.getElementById("fan");
const mapEl = document.getElementById("map");
const nearestEl = document.getElementById("nearest");
const muteEl = document.getElementById("mute");
const ultrasonicEl = document.getElementById("ultrasonic");
const tofEl = document.getElementById("tof");
// [字段名, 标签, 左 0 / 右 1, 头下方 0 / 腹部下方 1]。四个 TOF 都装在机身下方，*_rear 实际在腹部下方
const TOF_SENSORS = [["left_head", "头下左", 0, 0], ["right_head", "头下右", 1, 0], ["left_rear", "腹下左", 0, 1], ["right_rear", "腹下右", 1, 1]];
const TOF_FAR = 2.0;  // 颜色刻度的最远距离（m）
let scanData = null;
let nearest = Infinity;
let ultrasonicAlarm = false;
let mapMode = "points";
let mapRange = 5;
let muted = false;
let audio = null, gainNode = null;

// 把一帧雷达转成 [{i, a, r}]，只保留有回波的点（服务端已把无回波统一成 0）
function echoes(scan) {
  const out = [];
  scan.ranges.forEach((r, i) => { if (r > 0) out.push({ i: i, a: scan.angle_min + i * scan.angle_increment, r: r }); });
  return out;
}

function prepare(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) {
    canvas.width = Math.round(w * ratio);
    canvas.height = Math.round(h * ratio);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, w, h);
  return { ctx: ctx, w: w, h: h };
}

function zoneColor(d) {
  return d < RED_RANGE ? COLOR_RED : d < YELLOW_RANGE ? COLOR_YELLOW : d < GREEN_RANGE ? COLOR_GREEN : null;
}

function canvasAngle(a) { return -Math.PI / 2 - a; }

function drawFan(points) {
  const { ctx, w, h } = prepare(fanEl);
  const cx = w / 2, cy = h - 6, radius = Math.min(w / 2, h) - 10;
  const bands = [[COLOR_RED, 0.22, 0.46], [COLOR_YELLOW, 0.49, 0.72], [COLOR_GREEN, 0.75, 1.0]];
  const mins = new Array(SECTORS).fill(Infinity);
  const width = Math.PI / SECTORS;
  points.forEach(p => {
    const s = Math.floor((p.a + Math.PI / 2) / width);
    if (s >= 0 && s < SECTORS) mins[s] = Math.min(mins[s], p.r);
  });
  const gap = 0.02;
  mins.forEach((d, s) => {
    const a0 = -Math.PI / 2 + s * width + gap, a1 = -Math.PI / 2 + (s + 1) * width - gap;
    const lit = zoneColor(d);
    bands.forEach(([color, r0, r1]) => {
      ctx.beginPath();
      ctx.arc(cx, cy, radius * r1, canvasAngle(a0), canvasAngle(a1), true);
      ctx.arc(cx, cy, radius * r0, canvasAngle(a1), canvasAngle(a0), false);
      ctx.closePath();
      ctx.fillStyle = lit === color ? color : COLOR_DIM;
      ctx.fill();
    });
  });
  drawDog(ctx, cx, cy);
}

function drawDog(ctx, cx, cy) {
  ctx.beginPath();
  ctx.moveTo(cx, cy - 16);
  ctx.lineTo(cx - 9, cy);
  ctx.lineTo(cx + 9, cy);
  ctx.closePath();
  ctx.fillStyle = "#eee";
  ctx.fill();
}

function drawMap(points) {
  const { ctx, w, h } = prepare(mapEl);
  const cx = w / 2, cy = h - 6, scale = (Math.min(w / 2, h) - 10) / mapRange;
  ctx.strokeStyle = COLOR_GRID;
  ctx.fillStyle = COLOR_LABEL;
  ctx.font = "11px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.lineWidth = 1;
  const step = GRID_STEP[mapRange];
  for (let r = step; r <= mapRange + 1e-6; r += step) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * scale, Math.PI, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(r + " m", cx, cy - r * scale + 12);
  }
  for (let deg = -90; deg <= 90; deg += 30) {
    const a = deg * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx - Math.sin(a) * mapRange * scale, cy - Math.cos(a) * mapRange * scale);
    ctx.stroke();
  }
  const sx = p => cx - Math.sin(p.a) * p.r * scale;
  const sy = p => cy - Math.cos(p.a) * p.r * scale;
  if (mapMode === "points") {
    ctx.fillStyle = COLOR_POINT;
    points.forEach(p => ctx.fillRect(sx(p) - 1, sy(p) - 1, 2, 2));
  } else {
    ctx.strokeStyle = COLOR_POINT;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    points.forEach((p, n) => {
      const prev = points[n - 1];
      const joined = prev && p.i === prev.i + 1 && Math.abs(p.r - prev.r) <= LINE_BREAK;
      if (joined) ctx.lineTo(sx(p), sy(p)); else ctx.moveTo(sx(p), sy(p));
    });
    ctx.stroke();
  }
  drawDog(ctx, cx, cy);
}

// 红 → 黄 → 绿，并且越远越暗，让近处的物体从画面里跳出来
function tofHue(d) {
  return Math.round(130 * Math.min(d / TOF_FAR, 1));
}

function tofColor(d) {
  if (d <= 0) return "#333";
  const t = Math.min(d / TOF_FAR, 1);
  return "hsl(" + tofHue(d) + ",85%," + Math.round(55 - 28 * t * t) + "%)";
}

function drawTofDog(ctx, cx, cy, height) {
  const ry = height * 0.3, rx = Math.min(20, ry * 0.5), headR = rx * 0.7, headY = cy - ry - headR * 0.5;
  ctx.fillStyle = "#2c2c2c";
  ctx.strokeStyle = "#888";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(cx, cy + ry);
  ctx.lineTo(cx, cy + ry + headR);
  ctx.stroke();
  ctx.beginPath();
  ctx.ellipse(cx, cy, rx, ry, 0, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(cx, headY, headR, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#bbb";
  ctx.font = "12px system-ui, sans-serif";
  ctx.textBaseline = "middle";
  ctx.fillText("头", cx, headY);
  ctx.fillText("腹", cx, cy + ry * 0.45);
  ctx.textBaseline = "alphabetic";
}

function drawTofLegend(ctx, w, h) {
  const width = Math.min(200, w * 0.4), x = (w - width) / 2, y = h - 15;
  ctx.font = "12px system-ui, sans-serif";
  ctx.textAlign = "right";
  ctx.fillStyle = COLOR_LABEL;
  ctx.fillText("近", x - 6, y + 10);
  for (let i = 0; i < width; i += 2) {
    ctx.fillStyle = tofColor(Math.max(0.001, i / width * TOF_FAR));
    ctx.fillRect(x + i, y, 2, 12);
  }
  ctx.textAlign = "left";
  ctx.fillStyle = COLOR_LABEL;
  ctx.fillText(TOF_FAR + " m 以上", x + width + 6, y + 10);
  ctx.textAlign = "center";
}

function drawTof(tof) {
  const { ctx, w, h } = prepare(tofEl);
  const legend = 24, rowGap = 10, bodyHalf = 24, labelH = 20;
  const size = Math.min((h - legend - rowGap) / 2 - labelH, (w - 2 * bodyHalf - 8) / 2);
  const cell = size / 8, cx = w / 2, rowH = size + labelH, top = (h - legend - 2 * rowH - rowGap) / 2;
  drawTofDog(ctx, cx, top + rowH + rowGap / 2, 2 * rowH + rowGap);
  ctx.textAlign = "center";
  TOF_SENSORS.forEach(([key, label, side, end]) => {
    const x = side === 0 ? cx - bodyHalf - size : cx + bodyHalf;
    const y = top + end * (rowH + rowGap) + labelH, matrix = tof[key];  // matrix 为 null 表示传感器没有数据
    const values = matrix === null ? [] : matrix.flat().filter(v => v > 0);
    const closest = values.length ? Math.min(...values) : null;
    for (let row = 0; row < 8; row++) {
      for (let col = 0; col < 8; col++) {
        ctx.fillStyle = tofColor(matrix === null ? 0 : matrix[row][col]);
        ctx.fillRect(x + col * cell, y + row * cell, cell - 1, cell - 1);
      }
    }
    ctx.font = "600 14px system-ui, sans-serif";
    ctx.fillStyle = closest === null ? COLOR_LABEL : "hsl(" + tofHue(closest) + ",85%,65%)";
    ctx.fillText(label + " " + (matrix === null ? "无数据" : closest === null ? "无读数" : closest.toFixed(2) + " m"), x + size / 2, y - 6);
  });
  drawTofLegend(ctx, w, h);
}

const frontBannerEl = document.getElementById("frontBanner");
const tofBannerEl = document.getElementById("tofBanner");

// 视频上的报警横幅：前方看雷达和超声波，底部看 TOF。绿区不拉横幅，否则正常走路时会一直挡着画面
function showBanners() {
  const u = scanData.ultrasonic, front = u.alarm ? Math.min(nearest, u.range) : nearest;
  const level = u.alarm || front < RED_RANGE ? "red" : front < YELLOW_RANGE ? "yellow" : "";
  frontBannerEl.hidden = level === "";
  frontBannerEl.className = "banner " + level;
  frontBannerEl.textContent = "前方障碍 " + front.toFixed(2) + " m";
  const t = scanData.tof_alarm;  // null 表示没有过近的读数
  tofBannerEl.hidden = t === null;
  if (t !== null) tofBannerEl.textContent = "底部过近 " + TOF_SENSORS.find(s => s[0] === t.sensor)[1] + " " + t.range.toFixed(2) + " m";
}

function redraw() {
  if (scanData === null) return;
  const points = echoes(scanData);
  nearest = points.reduce((m, p) => Math.min(m, p.r), Infinity);
  nearestEl.textContent = nearest === Infinity ? "无回波" : "最近 " + nearest.toFixed(2) + " m";
  nearestEl.className = nearest < RED_RANGE ? "red" : nearest < YELLOW_RANGE ? "yellow" : nearest < GREEN_RANGE ? "green" : "";
  showBanners();
  // 窗口缩放的瞬间画布可能还没有尺寸，这一帧不画，下一帧会补上；报警数值已在上面更新
  if (fanEl.clientWidth === 0 || fanEl.clientHeight === 0 || tofEl.clientWidth === 0 || tofEl.clientHeight === 0) return;
  drawFan(points);
  drawMap(points);
  drawTof(scanData.tof);
}

function showUltrasonic(reading) {
  ultrasonicAlarm = reading.alarm;
  const distance = reading.range === null ? "无回波" : reading.range.toFixed(2) + " m";
  ultrasonicEl.textContent = (reading.alarm ? "超声波：距离过近 " : "超声波 ") + distance;
  ultrasonicEl.className = reading.alarm ? "alarm" : "";
}

const batteryEl = document.getElementById("battery");
const batteryTextEl = document.getElementById("batteryText");
const batteryLevelEl = document.getElementById("batteryLevel");

function showBattery(b) {
  batteryTextEl.textContent = "电池 " + b.soc + "%  " + b.voltage.toFixed(2) + " V" + (b.charging ? "  充电中" : "");
  batteryLevelEl.style.width = b.soc + "%";
  batteryEl.className = "panel " + (b.soc < 20 ? "red" : b.soc < 50 ? "yellow" : "");
}

const attitudePanel = document.getElementById("attitudePanel");
const horizonEl = document.getElementById("horizon");
const attitudeTextEl = document.getElementById("attitudeText");

// 姿态仪，a 的单位是度。狗右倾时地平线逆时针转，低头时地平线上移
function drawHorizon(a) {
  const { ctx, w, h } = prepare(horizonEl);
  const r = Math.min(w, h) / 2 - 2, cx = w / 2, cy = h / 2;
  ctx.save();
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  ctx.clip();
  ctx.translate(cx, cy);
  ctx.rotate(-a.roll * Math.PI / 180);
  ctx.translate(0, -a.pitch / 90 * r);
  ctx.fillStyle = "#1e88e5";
  ctx.fillRect(-3 * r, -3 * r, 6 * r, 3 * r);
  ctx.fillStyle = "#6d4c41";
  ctx.fillRect(-3 * r, 0, 6 * r, 3 * r);
  ctx.fillStyle = "#fff";
  ctx.fillRect(-3 * r, -1, 6 * r, 2);
  ctx.restore();
  ctx.strokeStyle = "#ffca28";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx - r * 0.5, cy);
  ctx.lineTo(cx - r * 0.15, cy);
  ctx.moveTo(cx + r * 0.15, cy);
  ctx.lineTo(cx + r * 0.5, cy);
  ctx.stroke();
  ctx.strokeStyle = "#555";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, 2 * Math.PI);
  ctx.stroke();
}

function showAttitude(a) {
  if (a === null) return;  // 加了 --no-attitude，面板保持隐藏
  attitudePanel.hidden = false;
  const degrees = v => (v >= 0 ? "+" : "") + v.toFixed(1) + "°";
  attitudeTextEl.textContent = "横滚 " + degrees(a.roll) + "\\n俯仰 " + degrees(a.pitch) + "\\n航向 " + degrees(a.yaw);
  drawHorizon(a);
}

const scanEvents = new EventSource("/scan.events");
scanEvents.onmessage = e => {
  scanData = JSON.parse(e.data);
  showUltrasonic(scanData.ultrasonic);
  showAttitude(scanData.attitude);
  showBattery(scanData.battery);
  redraw();
};
scanEvents.onerror = () => {
  scanEvents.close();
  if (!dead) fail("雷达数据断开，请看终端里的报错");
};

// 浏览器要求先有一次用户操作才能发声，所以在第一次按键或点击时创建
function ensureAudio() {
  if (audio === null) {
    audio = new (window.AudioContext || window.webkitAudioContext)();
    gainNode = audio.createGain();
    gainNode.gain.value = 0;
    const osc = audio.createOscillator();
    osc.frequency.value = 1800;
    osc.connect(gainNode).connect(audio.destination);
    osc.start();
  }
  if (audio.state === "suspended") audio.resume();
}

function beepOn(now) {
  if (muted || dead) return false;
  if (nearest < RED_RANGE || ultrasonicAlarm) return true;  // 雷达红区或超声过近时长鸣
  const period = nearest < YELLOW_RANGE ? 250 : nearest < GREEN_RANGE ? 700 : 0;
  return period > 0 && now % period < 90;
}

setInterval(() => {
  if (gainNode !== null) gainNode.gain.setTargetAtTime(beepOn(performance.now()) ? 0.12 : 0, audio.currentTime, 0.004);
}, 20);

window.addEventListener("keydown", ensureAudio);
window.addEventListener("pointerdown", ensureAudio);
window.addEventListener("resize", redraw);
// 工具按钮不抢焦点，否则点过之后键盘操作可能触发按钮
document.querySelectorAll(".tool").forEach(el => el.addEventListener("mousedown", e => e.preventDefault()));
muteEl.addEventListener("click", () => {
  muted = !muted;
  muteEl.classList.toggle("on", muted);
});
function choose(selector, el) {
  document.querySelectorAll(selector).forEach(b => b.classList.toggle("on", b === el));
  redraw();
}
document.querySelectorAll(".mode").forEach(el => el.addEventListener("click", () => {
  mapMode = el.dataset.mode;
  choose(".mode", el);
}));
document.querySelectorAll(".range").forEach(el => el.addEventListener("click", () => {
  mapRange = Number(el.dataset.range);
  choose(".range", el);
}));
</script>
</body>
</html>
"""


class Pad(object):
    """网页的按键状态。HTTP 线程写入，控制线程读取。"""

    def __init__(self, walk_speeds, run_speeds, hop_speeds):
        self._walk_speeds = walk_speeds    # (vx, vy, wz) 各轴的速度大小
        self._run_speeds = run_speeds      # 按住 Shift 时使用
        self._hop_speeds = hop_speeds      # 按住空格时使用，优先于 Shift
        self._lock = threading.Lock()
        self._seq = -1
        self._keys = frozenset()
        self._stamp = 0.0
        self.estop = threading.Event()

    def update(self, seq: int, keys) -> None:
        unknown = set(keys) - set(KEY_AXES)
        if unknown:
            raise ValueError("未知按键: %s" % sorted(unknown))
        with self._lock:
            if seq > self._seq:
                self._seq = seq
                self._keys = frozenset(keys)
                self._stamp = time.monotonic()

    def command(self):
        """当前按键对应的 ((vx, vy, wz), 步态)。网页太久没有更新时速度全 0。"""
        with self._lock:
            keys = self._keys if time.monotonic() - self._stamp <= STALE_AFTER else frozenset()
        hopping = HOP_KEY in keys
        running = RUN_KEY in keys and not hopping
        speeds = self._hop_speeds if hopping else self._run_speeds if running else self._walk_speeds
        velocity = tuple(speed * sum(KEY_AXES[k][axis] for k in keys) for axis, speed in enumerate(speeds))
        return velocity, "hop" if hopping else "fast" if running else "medium"


class Worker(threading.Thread):
    """后台线程。出错时记录异常并结束，由主线程重新抛出。"""

    def __init__(self, name):
        super().__init__(name=name, daemon=True)
        self._halt = threading.Event()
        self.error = None

    def run(self):
        try:
            self._loop()
        except BaseException as exc:
            self.error = exc

    def halt(self):
        self._halt.set()
        self.join()


class Controller(Worker):
    """唯一向狗下发运动指令的线程。"""

    def __init__(self, dog, pad):
        super().__init__("control")
        self._dog = dog
        self._pad = pad
        self.estopped = False

    def _loop(self):
        moving = False
        while not self._halt.is_set():
            if self._pad.estop.wait(CONTROL_PERIOD):
                self._dog.motion.estop()
                self.estopped = True
                return
            velocity, gait = self._pad.command()
            if any(velocity) or gait == "hop":  # 跳跃时速度可以为 0，也就是原地跳
                self._dog.motion.set_velocity(*velocity, gait=gait, timeout=DEADMAN)
                moving = True
            elif moving:
                self._dog.motion.stop()
                moving = False


def encode_ultrasonic(message, near):
    """把 Range 消息转成 {"range", "alarm"}。0、负数、inf、NaN 表示无回波，range 为 None。"""
    distance = message["range"]
    if distance > 0 and math.isfinite(distance):
        return {"range": round(distance, 3), "alarm": distance < near}
    return {"range": None, "alarm": False}


def encode_tof(matrices, head, rear):
    """把 dog.sensors.tof() 的结果转成可以发 JSON 的形式。无效读数为 0，data_available 为假的传感器为 None。"""
    sources = {"left_head": head, "right_head": head, "left_rear": rear, "right_rear": rear}
    out = {}
    for key, matrix in matrices.items():
        if sources[key][key]["data_available"]:
            out[key] = [[round(v, 3) if v > 0 and math.isfinite(v) else 0 for v in row] for row in matrix]
        else:
            out[key] = None
    return out


def encode_attitude(attitude):
    """把 dog.sensors.attitude() 的弧度转成网页显示用的度。"""
    return {axis: round(math.degrees(value), 1) for axis, value in attitude.items()}


def tof_alarm(tof, near):
    """在 encode_tof 的结果里找小于 near 的最近读数，返回 {"sensor", "range"}，没有时返回 None。0 表示无读数。"""
    close = [(v, key) for key, matrix in tof.items() if matrix is not None for row in matrix for v in row if 0 < v < near]
    if not close:
        return None
    distance, sensor = min(close)
    return {"sensor": sensor, "range": distance}


def encode_scan(scan, mount_yaw, ultrasonic, tof, attitude, tof_near, battery):
    """把 LaserScan、超声、TOF 和姿态转成发给网页的 JSON。超出量程、inf、NaN 和真机的 0.0 都表示无回波，统一成 0。

    attitude 为 None 表示没有开启姿态显示。
    """
    low, high = scan["range_min"], scan["range_max"]
    ranges = [round(r, 3) if low <= r <= high and r > 0 else 0 for r in scan["ranges"]]
    return json.dumps({"angle_min": scan["angle_min"] + mount_yaw, "angle_increment": scan["angle_increment"],
                       "ranges": ranges, "ultrasonic": ultrasonic, "tof": tof, "tof_alarm": tof_alarm(tof, tof_near),
                       "attitude": attitude, "battery": battery},
                      separators=(",", ":")).encode("utf-8")


class ScanFeed(Worker):
    """每有一帧新雷达，就连同最新的超声、TOF 和姿态读数放进 scans。等不到新鲜数据时抛 SensorTimeout。"""

    def __init__(self, dog, scans, mount_yaw, ultrasonic_near, tof_near, show_attitude):
        super().__init__("scan")
        self._dog = dog
        self._scans = scans
        self._mount_yaw = mount_yaw
        self._ultrasonic_near = ultrasonic_near
        self._tof_near = tof_near
        self._show_attitude = show_attitude

    def _loop(self):
        last = None
        while not self._halt.wait(SCAN_POLL):
            scan = self._dog.sensors.scan.get(timeout=SCAN_TIMEOUT, max_age=SCAN_MAX_AGE)
            if scan is not last:
                last = scan
                message = self._dog.sensors.ultrasonic.get(timeout=SCAN_TIMEOUT, max_age=ULTRASONIC_MAX_AGE)
                ultrasonic = encode_ultrasonic(message, self._ultrasonic_near)
                sensors = self._dog.sensors
                head = sensors.head_tof.get(timeout=SCAN_TIMEOUT, max_age=TOF_MAX_AGE)  # tof() 本身不检查新鲜度
                rear = sensors.rear_tof.get(timeout=SCAN_TIMEOUT, max_age=TOF_MAX_AGE)
                tof = encode_tof(sensors.tof(), head, rear)
                attitude = None
                if self._show_attitude:
                    attitude = encode_attitude(sensors.attitude(timeout=SCAN_TIMEOUT, max_age=ATTITUDE_MAX_AGE))
                battery = sensors.battery
                power = {"soc": battery.soc, "voltage": round(battery.voltage, 2), "charging": battery.charging}
                self._scans.put(encode_scan(scan, self._mount_yaw, ultrasonic, tof, attitude, self._tof_near, power))


def make_handler(latest, pad, scans):
    class Handler(live.make_handler(latest)):
        def do_GET(self):
            if self.path == "/scan.events":
                self._send_scan_events()
                return
            if self.path != "/":
                super().do_GET()  # /stream.mjpg 和 404 由 08 示例的处理器负责
                return
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_scan_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = 0
            try:
                while True:
                    seq, data = scans.wait_next(seq)
                    self.wfile.write(b"data: " + data + b"\n\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # 网页关掉了，这是正常情况

        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            # 要求 JSON 类型：浏览器跨站发这种请求要先预检，别的网站就没法遥控这只狗
            if self.headers["Content-Type"] != "application/json":
                self.send_error(415)
                return
            if self.path == "/control":
                msg = json.loads(body.decode("utf-8"))
                pad.update(msg["seq"], msg["keys"])
            elif self.path == "/estop":
                pad.estop.set()
            else:
                self.send_error(404)
                return
            self.send_response(204)
            self.end_headers()

        def log_request(self, code="-", size="-"):
            if self.command != "POST":  # 按住按键时每秒 10 个 POST，不打印
                super().log_request(code, size)

    return Handler


def run(dog, pad, latest, scans, args):
    """站立、开始拉流、雷达和控制，直到急停或出错。退出时 dog.close() 会停止运动。"""
    with dog:
        print("站立中 ...")
        dog.motion.stand()
        controller = Controller(dog, pad)
        feed = ScanFeed(dog, scans, math.radians(args.lidar_yaw), args.ultrasonic_near, args.tof_near, args.attitude)
        workers = (controller, feed)
        with dog.sensors.camera.stream(width=args.width, height=args.height, fps=args.fps,
                                       quality=args.quality) as stream:
            for worker in workers:
                worker.start()
            shown = "127.0.0.1" if args.bind == "0.0.0.0" else args.bind
            print("已就绪，浏览器打开 http://%s:%d/   按 Ctrl+C 停止" % (shown, args.port))
            try:
                for frame in stream:
                    latest.put(live.to_jpeg(frame))
                    if not all(worker.is_alive() for worker in workers):
                        break
            finally:
                for worker in workers:
                    worker.halt()
        for worker in workers:
            if worker.error is not None:
                raise worker.error
        if controller.estopped:
            print("已急停，程序退出")


def sim_dog():
    from roommapper.sim import SimBackend

    class SimSensors(SimBackend):
        """模拟房间本身没有超声波、TOF 和 IMU：超声和头下方 TOF 用雷达射线代替，腹部下方 TOF 用固定值，IMU 只有航向。"""

        def emit(self, topic, data):
            super().emit(topic, data)
            if topic != "scan":
                return
            ranges = data["ranges"]
            middle = len(ranges) // 2
            echoes = [r for r in ranges[middle - 20:middle + 21] if r > 0]
            super().emit("ultrasonic_payload", {"range": min(echoes) if echoes else math.inf,
                                                "min_range": 0.03, "max_range": 4.0})

            def head_side(side, position):
                cells = [ranges[middle + side * (4 + col * 8)] for row in range(8) for col in range(8)]
                return {"data_available": True, "tof_position": position, "data": cells}

            super().emit("head_tof_payload", {"left_head": head_side(1, 0), "right_head": head_side(-1, 1)})
            super().emit("rear_tof_payload", self.latched["rear_tof_payload"])
            yaw = self.room.yaw
            q = {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}
            super().emit("odom_out", {"pose": {"pose": {"orientation": q}},
                                      "twist": {"twist": {"angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}})

    return CyberDog(SimSensors(speed=1.0))


def open_dog(args):
    if args.mock:
        print("使用模拟房间（mock）")
        return sim_dog()
    ip = input("IP: ").strip()
    if not ip:
        raise ValueError("IP 不能为空")
    user = input("账号 [mi]: ").strip() or "mi"
    password = input("密码: ")
    print("连接 %s@%s ..." % (user, ip))
    return CyberDog.connect(ip, user=user, password=password)


def main():
    p = argparse.ArgumentParser(description="CyberDog 2 网页遥控")
    p.add_argument("--mock", action="store_true", help="使用模拟狗")
    p.add_argument("--bind", default="127.0.0.1", help="HTTP 监听地址，局域网使用时为 0.0.0.0（无登录验证）")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--vx", type=float, default=0.3, help="W/S 速度 m/s")
    p.add_argument("--vy", type=float, default=0.2, help="A/D 速度 m/s")
    p.add_argument("--wz", type=float, default=0.6, help="←/→ 角速度 rad/s")
    p.add_argument("--run-vx", type=float, default=1.0, help="Shift+W/S 速度 m/s")
    p.add_argument("--run-vy", type=float, default=0.4, help="Shift+A/D 速度 m/s")
    p.add_argument("--run-wz", type=float, default=1.2, help="Shift+←/→ 角速度 rad/s")
    p.add_argument("--hop-vx", type=float, default=0.2, help="跳跃时 W/S 速度 m/s")
    p.add_argument("--hop-vy", type=float, default=0.1, help="跳跃时 A/D 速度 m/s")
    p.add_argument("--hop-wz", type=float, default=0.4, help="跳跃时 ←/→ 角速度 rad/s")
    p.add_argument("--lidar-yaw", type=float, default=0.0, help="激光雷达相对狗身前方的安装角，单位度")
    p.add_argument("--ultrasonic-near", type=float, default=0.3, help="超声波小于这个距离时报警，单位 m")
    p.add_argument("--tof-near", type=float, default=0.05, help="底部 TOF 小于这个距离时拉横幅，单位 m；应小于站立时的离地读数")
    p.add_argument("--attitude", dest="attitude", action="store_true", default=True,
                   help="显示机身姿态，默认开启（数据来自 odom_out，收不到时程序报错退出）")
    p.add_argument("--no-attitude", dest="attitude", action="store_false", help="不显示机身姿态")
    p.add_argument("--fps", type=float, default=15)
    p.add_argument("--quality", type=int, default=70)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args()
    if args.tof_near <= 0:
        raise ValueError("--tof-near 必须为正数，收到 %r" % args.tof_near)
    if args.ultrasonic_near <= 0:
        raise ValueError("--ultrasonic-near 必须为正数，收到 %r" % args.ultrasonic_near)

    latest = live.LatestJpeg()
    scans = live.LatestJpeg()  # 这个类只管分发最新的一份 bytes，雷达的 JSON 也用它
    pad = Pad((args.vx, args.vy, args.wz), (args.run_vx, args.run_vy, args.run_wz),
              (args.hop_vx, args.hop_vy, args.hop_wz))
    server = ThreadingHTTPServer((args.bind, args.port), make_handler(latest, pad, scans))
    threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
    try:
        run(open_dog(args), pad, latest, scans, args)
    except KeyboardInterrupt:
        print("已停止")
    except BaseException as exc:
        latest.fail(exc)  # 让正在观看的连接也结束
        scans.fail(exc)
        raise             # let it crash：出错直接带着完整异常退出
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
