// app.js — 主程序: 3D 场景 / SSE 实时状态 / UI 控制
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RobotModel, DEG } from './robot.js';
import { solveIK, relPoseToWorld, worldToRelPose } from './ik.js';
import { Planner, buildTrajectory, cartesianPath } from './planner.js';

const AXIS = 6;
const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

// ---- 关节标定: 可视化角 = sign*(pos_deg) - offset, 再 wrap 到 [-180,180] ----
const calib = {
  sign: [1, 1, 1, 1, 1, 1],
  offsetDeg: [0, 0, 0, 0, 0, 0],
};

function wrap180(deg) {
  let d = ((deg + 180) % 360 + 360) % 360 - 180;
  return d;
}
function jointAngleRad(j, cdeg) {
  const deg = wrap180(calib.sign[j] * (cdeg / 100) - calib.offsetDeg[j]);
  return deg * DEG;
}
function anglesFromAxes(axes) {
  const out = new Array(AXIS).fill(0);
  for (const a of axes) {
    const j = a.axis - 1;
    if (j >= 0 && j < AXIS) out[j] = jointAngleRad(j, a.pos_cdeg);
  }
  return out;
}

// ======================================================================
// three.js 场景
// ======================================================================
const holder = document.getElementById('canvasHolder');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e13);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
camera.up.set(0, 0, 1); // Z 轴朝上 (ROS/URDF 约定)
camera.position.set(1.2, -1.2, 0.9);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
holder.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dir1 = new THREE.DirectionalLight(0xffffff, 1.6);
dir1.position.set(2, -2, 3);
scene.add(dir1);
const dir2 = new THREE.DirectionalLight(0x88aaff, 0.6);
dir2.position.set(-2, 2, 1);
scene.add(dir2);

// 网格 (XY 平面, Z 上)
const grid = new THREE.GridHelper(2, 20, 0x33405a, 0x1d2533);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
// 世界坐标轴
const axesHelper = new THREE.AxesHelper(0.25);
scene.add(axesHelper);

// 轨迹与末端标记容器
const trajGroup = new THREE.Group();
scene.add(trajGroup);
let rawLine = null, smoothLine = null;

const eeMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.012, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0x00c2a8, emissive: 0x004038 })
);
scene.add(eeMarker);

let planLine = null;          // 规划轨迹预览线
const wpGroup = new THREE.Group();  // 规划航点标记
scene.add(wpGroup);

let robot = null;
let lastAxes = [];            // 最近一帧反馈
let vizLock = false;          // 为 true 时 SSE 不覆盖 3D 关节(用于试行/纯预览)
const planner = new Planner();
let selWp = -1;               // 选中的航点
let simRun = false;           // 试行(仅 3D 模拟)进行中

function resize() {
  const w = holder.clientWidth, h = holder.clientHeight;
  if (w === 0 || h === 0) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  if (robot) {
    const p = robot.getEndEffectorPosition();
    eeMarker.position.copy(p);
    document.getElementById('eePos').textContent =
      `${p.x.toFixed(3)}, ${p.y.toFixed(3)}, ${p.z.toFixed(3)}`;
    updateEEPose();
  }
  renderer.render(scene, camera);
}

function fitView() {
  if (!robot) return;
  const box = robot.boundingBox();
  if (box.isEmpty()) return;
  const center = new THREE.Vector3(), size = new THREE.Vector3();
  box.getCenter(center); box.getSize(size);
  const maxDim = Math.max(size.x, size.y, size.z) || 1;
  controls.target.copy(center);
  camera.position.set(center.x + maxDim * 1.1, center.y - maxDim * 1.3, center.z + maxDim * 0.9);
  camera.near = maxDim / 100; camera.far = maxDim * 100;
  camera.updateProjectionMatrix();
  controls.update();
  grid.position.set(center.x, center.y, box.min.z);
}

// ======================================================================
// 轨迹绘制
// ======================================================================
function buildLine(posArray, color, width) {
  if (!robot || !posArray || posArray.length < 2) return null;
  const pts = [];
  for (const cdegRow of posArray) {
    const ang = cdegRow.map((c, j) => jointAngleRad(j, c));
    const p = robot.fkEndEffector(ang);
    pts.push(p.x, p.y, p.z);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  const mat = new THREE.LineBasicMaterial({ color });
  return new THREE.Line(geo, mat);
}

function drawRaw(posArray) {
  if (rawLine) { trajGroup.remove(rawLine); rawLine.geometry.dispose(); rawLine = null; }
  rawLine = buildLine(posArray, 0xf0a020, 1);
  if (rawLine) { rawLine.visible = document.getElementById('chkRaw').checked; trajGroup.add(rawLine); }
}
function drawSmooth(posArray) {
  if (smoothLine) { trajGroup.remove(smoothLine); smoothLine.geometry.dispose(); smoothLine = null; }
  smoothLine = buildLine(posArray, 0x00e0ff, 2);
  if (smoothLine) { smoothLine.visible = document.getElementById('chkTraj').checked; trajGroup.add(smoothLine); }
}
function clearTrajLines() {
  for (const l of [rawLine, smoothLine]) {
    if (l) { trajGroup.remove(l); l.geometry.dispose(); }
  }
  rawLine = smoothLine = null;
}

// ======================================================================
// API
// ======================================================================
async function api(path, body) {
  try {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
    });
    return await res.json();
  } catch (e) {
    flash('请求失败: ' + e.message);
    return { ok: false, error: e.message };
  }
}

function flash(msg) {
  const el = document.getElementById('lastMsg');
  el.textContent = msg;
  clearTimeout(flash._t);
  flash._t = setTimeout(() => { el.textContent = ''; }, 4000);
}

// ======================================================================
// SSE 实时状态
// ======================================================================
let userJogging = false;
function startSSE() {
  const es = new EventSource('/events');
  es.onmessage = (ev) => {
    let s;
    try { s = JSON.parse(ev.data); } catch { return; }
    applyState(s);
  };
  es.onerror = () => { /* 浏览器自动重连 */ };
}

function applyState(s) {
  // 连接状态
  const dot = document.getElementById('connDot');
  dot.className = 'dot ' + (s.connected ? 'on' : 'off');
  document.getElementById('connText').textContent = s.status || (s.connected ? '已连接' : '未连接');

  // 急停 / 运动
  const est = document.getElementById('estopState');
  est.textContent = s.estop ? '是' : '否';
  est.className = 'tag ' + (s.estop ? 'alarm' : 'good');
  const mv = document.getElementById('movingState');
  mv.textContent = s.arm_moving ? '是' : '否';
  mv.className = 'tag ' + (s.arm_moving ? '' : 'good');

  // 示教按钮
  const tb = document.getElementById('btnTeach');
  tb.classList.toggle('active', !!s.teach);
  tb.textContent = s.teach ? '退出示教模式' : '进入示教模式';

  // 记录
  document.getElementById('recCount').textContent = s.record_count;
  const rb = document.getElementById('recBadge');
  rb.classList.toggle('active', !!s.recording);
  rb.textContent = s.recording ? '● 记录中' : '● 未记录';
  document.getElementById('btnRecord').classList.toggle('active', !!s.recording);
  document.getElementById('btnRecord').textContent = s.recording ? '停止记录' : '开始记录';

  // 复现进度
  document.getElementById('replayBar').style.width = (s.replay_progress * 100).toFixed(1) + '%';

  // 关节表 + 机械臂位姿
  lastAxes = s.axes;
  updateJointTable(s.axes);
  if (robot && !vizLock) robot.setJointAngles(anglesFromAxes(s.axes));
  if (!userJogging) syncJogSliders(s.axes);
  document.getElementById('planBar').style.width = (s.replay_progress * 100).toFixed(1) + '%';

  if (s.log) renderDeviceLog(s.log);

  if (s.last_err) flash(s.last_err);
}

function renderDeviceLog(lines) {
  const el = document.getElementById('deviceLog');
  const text = lines.join('\n');
  if (el.textContent === text) return;
  el.textContent = text;
  if (document.getElementById('chkAutoscroll').checked) el.scrollTop = el.scrollHeight;
}

// ======================================================================
// 关节表 / 点动面板
// ======================================================================
function buildJointTable() {
  const t = document.getElementById('jointTable');
  t.innerHTML = '';
  for (let i = 0; i < AXIS; i++) {
    const row = document.createElement('div');
    row.className = 'jrow';
    row.innerHTML = `
      <div class="jhead">
        <span class="jname">${JOINT_NAMES[i]} (轴${i + 1})</span>
        <span class="jonline" id="on${i}">离线</span>
      </div>
      <div class="jgrid">
        <span>位置 <b id="pos${i}">-</b>°</span>
        <span>速度 <b id="vel${i}">-</b>°/s</span>
        <span>力矩 <b id="trq${i}">-</b> N·m</span>
        <span>电流 <b id="cur${i}">-</b> A</span>
        <span>温度 <b id="tmp${i}">-</b> ℃</span>
        <span>运动 <b id="mov${i}">-</b></span>
      </div>`;
    t.appendChild(row);
  }
}
function updateJointTable(axes) {
  for (const a of axes) {
    const i = a.axis - 1;
    if (i < 0 || i >= AXIS) continue;
    const on = document.getElementById('on' + i);
    if (!on) continue;
    on.textContent = a.online ? '在线' : '离线';
    on.className = 'jonline ' + (a.online ? 'on' : '');
    document.getElementById('pos' + i).textContent = (a.pos_cdeg / 100).toFixed(2);
    document.getElementById('vel' + i).textContent = a.vel_dps;
    document.getElementById('trq' + i).textContent = a.torque_nm.toFixed(2);
    document.getElementById('cur' + i).textContent = a.current_a.toFixed(2);
    document.getElementById('tmp' + i).textContent = a.temp_c;
    document.getElementById('mov' + i).textContent = a.moving ? '是' : '否';
  }
}

let jogTimers = {};
function buildJogPanel() {
  const p = document.getElementById('jogPanel');
  p.innerHTML = '';
  for (let i = 0; i < AXIS; i++) {
    const row = document.createElement('div');
    row.className = 'jog';
    row.innerHTML = `
      <span class="jlabel">J${i + 1}</span>
      <input type="range" id="jog${i}" min="0" max="359.99" step="0.5" value="0" />
      <span class="jdeg" id="jogv${i}">0.0°</span>`;
    p.appendChild(row);
    const slider = row.querySelector('input');
    slider.addEventListener('input', () => {
      userJogging = true;
      document.getElementById('jogv' + i).textContent = parseFloat(slider.value).toFixed(1) + '°';
      clearTimeout(jogTimers[i]);
      jogTimers[i] = setTimeout(() => {
        const angle_cdeg = Math.round(parseFloat(slider.value) * 100) % 36000;
        api('/api/set_axis', { axis: i + 1, angle_cdeg, speed: 120 });
      }, 120);
    });
    slider.addEventListener('change', () => { setTimeout(() => { userJogging = false; }, 600); });
  }
}
function syncJogSliders(axes) {
  for (const a of axes) {
    const i = a.axis - 1;
    const sl = document.getElementById('jog' + i);
    if (!sl) continue;
    const deg = (a.pos_cdeg / 100);
    sl.value = deg;
    document.getElementById('jogv' + i).textContent = deg.toFixed(1) + '°';
  }
}

// ======================================================================
// 串口
// ======================================================================
async function refreshPorts() {
  const r = await api('/api/ports', {});
  const sel = document.getElementById('portSelect');
  const prev = sel.value;
  sel.innerHTML = '';
  if (!r.ports || r.ports.length === 0) {
    const o = document.createElement('option');
    o.textContent = '(无可用串口)'; o.value = '';
    sel.appendChild(o);
    return;
  }
  for (const p of r.ports) {
    const o = document.createElement('option');
    o.value = p.device;
    o.textContent = `${p.device} — ${p.desc}`;
    sel.appendChild(o);
  }
  if (prev) sel.value = prev;
}

// ======================================================================
// UI 绑定
// ======================================================================
function bindUI() {
  document.getElementById('btnRefresh').onclick = refreshPorts;
  document.getElementById('btnConnect').onclick = async () => {
    const port = document.getElementById('portSelect').value;
    const baud = parseInt(document.getElementById('baudSelect').value, 10);
    if (!port) { flash('请选择串口'); return; }
    const r = await api('/api/connect', { port, baud });
    if (!r.ok) flash(r.error || '连接失败'); else flash('已连接 ' + port);
  };
  document.getElementById('btnDisconnect').onclick = () => api('/api/disconnect', {});

  document.getElementById('btnEnable').onclick = async () => { await api('/api/enable', {}); flash('已使能 (清除急停)'); };
  document.getElementById('btnDisable').onclick = async () => { await api('/api/disable', {}); flash('已失能 (急停)'); };
  document.getElementById('btnClear').onclick = () => api('/api/clear', {});
  document.getElementById('btnEstop').onclick = async () => { await api('/api/estop', {}); flash('⚠ 急停已触发'); };
  document.getElementById('btnHome').onclick = async () => {
    const r = await api('/api/home', {});
    if (r.ok) flash(`回零点中 (六轴 → 0, ${r.speed_dps || ''}°/s)`); else flash(r.error || '回零点失败');
  };

  document.getElementById('btnTeach').onclick = async () => {
    const active = document.getElementById('btnTeach').classList.contains('active');
    const r = await api('/api/teach', { on: !active });
    if (!r.ok) flash('示教指令失败');
  };

  document.getElementById('btnRecord').onclick = async () => {
    const recording = document.getElementById('btnRecord').classList.contains('active');
    if (!recording) {
      clearTrajLines();
      await api('/api/record/start', {});
      flash('开始记录手掰轨迹');
    } else {
      const r = await api('/api/record/stop', {});
      if (r.ok && r.raw) {
        drawRaw(r.raw.pos);
        flash(`记录完成, 共 ${r.raw.pos.length} 点。可点击"平滑处理"`);
      }
    }
  };

  document.getElementById('btnSmooth').onclick = async () => {
    const median_win = parseInt(document.getElementById('medianWin').value, 10);
    const smooth_win = parseInt(document.getElementById('smoothWin').value, 10);
    const r = await api('/api/smooth', { median_win, smooth_win, resample_dt: 0.05 });
    if (!r.ok) { flash(r.error || '平滑失败'); return; }
    if (r.raw) drawRaw(r.raw.pos);
    if (r.smoothed) drawSmooth(r.smoothed.pos);
    flash(`平滑完成: ${r.smoothed.pos.length} 点`);
  };

  document.getElementById('btnReplay').onclick = async () => {
    const speed_factor = parseFloat(document.getElementById('speedFactor').value);
    const r = await api('/api/replay/start', { speed_factor });
    if (!r.ok) flash(r.error || '复现失败'); else flash('开始按轨迹运动');
  };
  document.getElementById('btnReplayStop').onclick = () => api('/api/replay/stop', {});
  document.getElementById('btnClearTraj').onclick = async () => {
    await api('/api/record/clear', {});
    clearTrajLines();
    flash('已清除轨迹');
  };

  // 滑块数值显示
  const bindRange = (id, valId, fmt) => {
    const el = document.getElementById(id);
    const v = document.getElementById(valId);
    const upd = () => { v.textContent = fmt ? fmt(el.value) : el.value; };
    el.addEventListener('input', upd); upd();
  };
  bindRange('medianWin', 'medianWinVal');
  bindRange('smoothWin', 'smoothWinVal');
  bindRange('speedFactor', 'speedVal', (x) => parseFloat(x).toFixed(1));

  // 视图
  document.getElementById('btnFit').onclick = fitView;
  document.getElementById('btnSetZero').onclick = () => {
    // 用当前反馈位置作为可视化零点
    for (let i = 0; i < AXIS; i++) {
      const sl = document.getElementById('jog' + i);
      const deg = parseFloat(sl.value) || 0;
      calib.offsetDeg[i] = calib.sign[i] * deg;
    }
    flash('已将当前姿态校准为可视化零点');
  };
  document.getElementById('chkTraj').onchange = (e) => { if (smoothLine) smoothLine.visible = e.target.checked; };
  document.getElementById('chkRaw').onchange = (e) => { if (rawLine) rawLine.visible = e.target.checked; };
  document.getElementById('chkGrid').onchange = (e) => { grid.visible = e.target.checked; axesHelper.visible = e.target.checked; };
}

function bindCartesian() {
  const pm = () => (parseFloat(document.getElementById('posStep').value) || 5) * 0.001;  // mm→m
  const rm = () => (parseFloat(document.getElementById('rotStep').value) || 3) * DEG;     // deg→rad
  const J = (id, ax, fn) => { const b = document.getElementById(id); if (b) b.onclick = () => cartesianJog(ax, fn()); };
  J('btnJogXp', 'x', () => pm()); J('btnJogXn', 'x', () => -pm());
  J('btnJogYp', 'y', () => pm()); J('btnJogYn', 'y', () => -pm());
  J('btnJogZp', 'z', () => pm()); J('btnJogZn', 'z', () => -pm());
  J('btnJogRp', 'roll', () => rm()); J('btnJogRn', 'roll', () => -rm());
  J('btnJogPp', 'pitch', () => rm()); J('btnJogPn', 'pitch', () => -rm());
  J('btnJogWp', 'yaw', () => rm()); J('btnJogWn', 'yaw', () => -rm());
  document.getElementById('btnGoOrigin').onclick = goOriginPose;
}

function bindPlanner() {
  const after = () => { renderPlanner(); drawPlanned(); persistPlan(); };
  document.getElementById('btnTjAdd').onclick = () => {
    if (!robot) { flash('模型未就绪'); return; }
    planner.add(captureWaypoint()); selWp = planner.waypoints.length - 1; after(); flash('已新增航点 (当前位姿)');
  };
  document.getElementById('btnTjModify').onclick = () => {
    if (selWp < 0) { flash('先选中一行'); return; }
    const wp = planner.waypoints[selWp]; const cap = captureWaypoint();
    cap.type = wp.type; cap.speed = wp.speed; cap.smooth = wp.smooth; cap.delay = wp.delay; cap.name = wp.name;
    planner.modify(selWp, cap); after(); flash('已修正为当前位姿');
  };
  document.getElementById('btnTjInsert').onclick = () => {
    if (!robot) return;
    planner.insert(selWp < 0 ? planner.waypoints.length : selWp, captureWaypoint()); after();
  };
  document.getElementById('btnTjUp').onclick = () => { if (selWp <= 0) return; planner.move(selWp, -1); selWp--; after(); };
  document.getElementById('btnTjDown').onclick = () => { if (selWp < 0 || selWp >= planner.waypoints.length - 1) return; planner.move(selWp, 1); selWp++; after(); };
  document.getElementById('btnTjDel').onclick = () => { if (selWp < 0) return; planner.remove(selWp); selWp = -1; after(); };
  document.getElementById('btnTjUndo').onclick = () => { planner.undo(); if (selWp >= planner.waypoints.length) selWp = -1; after(); };
  document.getElementById('btnTjRedo').onclick = () => { planner.redo(); if (selWp >= planner.waypoints.length) selWp = -1; after(); };
  document.getElementById('btnTjSave').onclick = () => {
    const blob = new Blob([planner.toJSON()], { type: 'application/json' });
    const url = URL.createObjectURL(blob); const a = document.createElement('a');
    a.href = url; a.download = 'trajectory.json'; a.click(); URL.revokeObjectURL(url);
    flash('轨迹已保存为 trajectory.json');
  };
  document.getElementById('btnTjLoad').onclick = () => document.getElementById('tjFile').click();
  document.getElementById('tjFile').onchange = (e) => {
    const f = e.target.files[0]; if (!f) return;
    const r = new FileReader();
    r.onload = () => { try { planner.fromJSON(r.result); selWp = -1; after(); flash('已载入轨迹'); } catch (err) { flash('载入失败: ' + err.message); } };
    r.readAsText(f); e.target.value = '';
  };
  document.getElementById('btnTjTryLine').onclick = simulate;
  document.getElementById('btnTjTryCurve').onclick = simulate;
  document.getElementById('btnTjAuto').onclick = async () => {
    if (!robot) return;
    const traj = buildRunTraj();
    if (!traj) { flash('至少需要 1 个航点'); return; }
    drawRunPath(traj.ee); vizLock = false;
    // approach:false → 后端不再做关节空间"回起点", 直接走轨迹里已含的 当前→第0点 笛卡尔直线
    // 规划元数据: 随运行一起下发, 后端在"运行结束"时把 规划+各电机实际关节角 写进同一文件
    const plan = {
      ts: new Date().toISOString(),
      waypoints: planner.waypoints,
      t: traj.t, jointsRad: traj.jointsRad, ee: traj.ee, pos: traj.pos,
    };
    const r = await api('/api/run_traj', { t: traj.t, pos: traj.pos, speed_factor: 1.0, approach: false, plan });
    if (!r.ok) {
      flash(r.error || '运行失败');
      // 没能运行(如未连接硬件): 退而只导出规划数据
      const dump = await api('/api/dump_plan', plan);
      if (dump && dump.ok) planMsg(`仅导出规划(未运行): ${dump.file}`);
      return;
    }
    flash('自动运行中'); planMsg(`运行: ${traj.pos.length} 点 · 结束后自动导出 规划+实际`);
  };
  document.getElementById('btnTjStop').onclick = () => { stopSim(); api('/api/replay/stop', {}); flash('已停止'); };
  document.getElementById('chkPlan').onchange = (e) => { if (planLine) planLine.visible = e.target.checked; wpGroup.visible = e.target.checked; };
}

// ======================================================================
// 末端位姿 / 笛卡尔遥控 (IK)
// ======================================================================
// URDF 关节角(rad) → 电机命令 cdeg (jointAngleRad 的逆)
function toMotorCdeg(j, urdfRad) {
  const deg = urdfRad * 180 / Math.PI;
  let cdeg = calib.sign[j] * (deg + calib.offsetDeg[j]) * 100;
  return ((Math.round(cdeg) % 36000) + 36000) % 36000;
}
// 由最近反馈得到当前各关节 URDF 角(rad), 作 IK 种子
function currentUrdfAngles() {
  const out = [0, 0, 0, 0, 0, 0];
  for (const a of lastAxes) {
    const j = a.axis - 1;
    if (j >= 0 && j < AXIS) out[j] = jointAngleRad(j, a.pos_cdeg);
  }
  return out;
}
function updateEEPose() {
  if (!robot || !robot.homeMatrix) return;
  const cur = robot.getEEMatrix();
  const rel = worldToRelPose(robot.homeMatrix, cur);
  // x/y/z 显示为 base_link 世界系下相对 home 原点的位移 (与平移遥控同系)
  const home = new THREE.Vector3().setFromMatrixPosition(robot.homeMatrix);
  const now = new THREE.Vector3().setFromMatrixPosition(cur);
  const d = now.sub(home);
  const g = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };
  g('eeX', (d.x * 1000).toFixed(1)); g('eeY', (d.y * 1000).toFixed(1)); g('eeZ', (d.z * 1000).toFixed(1));
  g('eeR', (rel.roll * 180 / Math.PI).toFixed(1));
  g('eeP', (rel.pitch * 180 / Math.PI).toFixed(1));
  g('eeYaw', (rel.yaw * 180 / Math.PI).toFixed(1));
}
// 笛卡尔遥控: axis ∈ {x,y,z,roll,pitch,yaw}, delta (m 或 rad)
// 平移 x/y/z: 沿 base_link 世界系坐标轴移动末端, 姿态保持不变
// 旋转 roll/pitch/yaw: 仍在 home(末端原点) 参考系下做姿态增量
function cartesianJog(axis, delta) {
  if (!robot || !robot.homeMatrix) { flash('模型未就绪'); return; }
  const seed = currentUrdfAngles();
  const cur = robot.eeMatrixFor(seed);
  let target;
  if (axis === 'x' || axis === 'y' || axis === 'z') {
    target = cur.clone();
    const p = new THREE.Vector3().setFromMatrixPosition(target);
    p[axis] += delta;                       // base_link 世界系平移
    target.setPosition(p);
  } else {
    const rel = worldToRelPose(robot.homeMatrix, cur);
    rel[axis] += delta;
    target = relPoseToWorld(robot.homeMatrix, rel);
  }
  const res = solveIK(robot, target, seed);
  if (!res.ok) { flash('该方向不可达 / 接近奇异, 已忽略'); robot.setJointAngles(seed); return; }
  applyIkResult(res.angles);
}
function goOriginPose() {
  if (!robot || !robot.homeMatrix) return;
  const seed = currentUrdfAngles();
  const res = solveIK(robot, robot.homeMatrix.clone(), seed);
  if (!res.ok) { flash('无法回到末端原点'); return; }
  applyIkResult(res.angles);
}
function applyIkResult(angles) {
  const sendHw = document.getElementById('chkSendHw').checked;
  robot.setJointAngles(angles);          // 立即预览
  if (sendHw) {
    vizLock = false;                     // 真机会动, 让反馈驱动显示
    const angles_cdeg = angles.map((a, j) => toMotorCdeg(j, a));
    api('/api/set_all', { angles_cdeg, speed: 60 });
  } else {
    vizLock = true;                      // 不发真机, 锁住以保留预览
  }
}

// ======================================================================
// 轨迹规划
// ======================================================================
function captureWaypoint() {
  const seed = currentUrdfAngles();
  const pose = worldToRelPose(robot.homeMatrix, robot.eeMatrixFor(seed));
  return {
    name: '', type: 'line',
    speed: parseInt(document.getElementById('defSpeed').value) || 100,
    smooth: parseInt(document.getElementById('defSmooth').value) || 9,
    delay: parseInt(document.getElementById('defDelay').value) || 0,
    joints: seed.slice(), pose,
  };
}
function persistPlan() { try { localStorage.setItem('arm_plan', planner.toJSON()); } catch (e) {} }
function renderPlanner() {
  const t = document.getElementById('trajTable');
  t.innerHTML = '';
  const TYPES = [['line', '直线'], ['curve', '曲线']];
  planner.waypoints.forEach((wp, idx) => {
    const row = document.createElement('div');
    row.className = 'tj-row' + (idx === selWp ? ' sel' : '');
    const opts = TYPES.map(([v, l]) => `<option value="${v}">${l}</option>`).join('');
    row.innerHTML = `
      <div class="tj-main">
        <span class="tj-idx">${idx}</span>
        <input class="tj-name" placeholder="Name" />
        <select class="tj-type">${opts}</select>
        <input class="tj-speed" type="number" min="1" max="1000" />
        <input class="tj-smooth" type="number" min="0" max="20" />
        <input class="tj-delay" type="number" min="0" step="50" />
        <button class="btn small tj-sw">切换</button>
      </div>
      <div class="tj-pose">${poseStr(wp.pose)}</div>`;
    row.querySelector('.tj-name').value = wp.name || '';
    row.querySelector('.tj-type').value = wp.type;
    row.querySelector('.tj-speed').value = wp.speed;
    row.querySelector('.tj-smooth').value = wp.smooth;
    row.querySelector('.tj-delay').value = wp.delay;
    row.addEventListener('click', (e) => {
      if (['INPUT', 'SELECT', 'BUTTON'].includes(e.target.tagName)) return;
      selWp = idx; renderPlanner();
    });
    row.querySelector('.tj-name').onchange = (e) => { wp.name = e.target.value; persistPlan(); };
    row.querySelector('.tj-type').onchange = (e) => { wp.type = e.target.value; renderPlanner(); drawPlanned(); persistPlan(); };
    row.querySelector('.tj-speed').onchange = (e) => { wp.speed = parseInt(e.target.value) || 100; persistPlan(); };
    row.querySelector('.tj-smooth').onchange = (e) => { wp.smooth = parseInt(e.target.value) || 0; drawPlanned(); persistPlan(); };
    row.querySelector('.tj-delay').onchange = (e) => { wp.delay = parseInt(e.target.value) || 0; persistPlan(); };
    row.querySelector('.tj-sw').onclick = () => {
      wp.type = wp.type === 'line' ? 'curve' : 'line';
      renderPlanner(); drawPlanned(); persistPlan();
    };
    t.appendChild(row);
  });
  drawWaypoints();
}
function drawPlanned() {
  if (planLine) { trajGroup.remove(planLine); planLine.geometry.dispose(); planLine = null; }
  if (!robot || !robot.homeMatrix) return;
  let pts;
  try { pts = cartesianPath(planner, robot.homeMatrix); } catch (e) { return; }
  if (!pts || pts.length < 2) return;
  const flat = [];
  for (const p of pts) flat.push(p[0], p[1], p[2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
  planLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffd23f }));
  planLine.visible = document.getElementById('chkPlan').checked;
  trajGroup.add(planLine);
}
// 画"真正会运行的完整轨迹"末端路径 (含 当前点→第0点→…→第N点, 经 IK 串起来)
function drawRunPath(eeArr) {
  if (planLine) { trajGroup.remove(planLine); planLine.geometry.dispose(); planLine = null; }
  if (!eeArr || eeArr.length < 2) return;
  const flat = [];
  for (const p of eeArr) flat.push(p[0], p[1], p[2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
  planLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffd23f }));
  planLine.visible = document.getElementById('chkPlan').checked;
  trajGroup.add(planLine);
}
function planMsg(m) { const e = document.getElementById('planMsg'); if (e) e.textContent = m; }

// 相对位姿 → 文本 (mm / °)
function poseStr(p) {
  const d = 180 / Math.PI;
  return `X ${(p.x * 1000).toFixed(1)}  Y ${(p.y * 1000).toFixed(1)}  Z ${(p.z * 1000).toFixed(1)} mm` +
         `  ·  R ${(p.roll * d).toFixed(1)}  P ${(p.pitch * d).toFixed(1)}  Yaw ${(p.yaw * d).toFixed(1)}°`;
}
function _makeLabel(text) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 64;
  const ctx = cv.getContext('2d');
  ctx.fillStyle = 'rgba(0,0,0,0.55)'; ctx.beginPath(); ctx.arc(32, 32, 30, 0, 7); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.font = 'bold 40px sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(text, 32, 35);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(cv), depthTest: false }));
  sp.scale.set(0.035, 0.035, 0.035);
  return sp;
}
function _clearGroup(g) {
  while (g.children.length) {
    const c = g.children.pop();
    if (c.geometry) c.geometry.dispose();
    if (c.material) { if (c.material.map) c.material.map.dispose(); c.material.dispose(); }
  }
}
// 在 3D 中标出每个航点 (按类型着色, 选中放大高亮, 带编号)
function drawWaypoints() {
  _clearGroup(wpGroup);
  if (!robot || !robot.homeMatrix) return;
  const COL = { line: 0x4f8cff, curve: 0x2ecc71 };
  planner.waypoints.forEach((wp, idx) => {
    const p = new THREE.Vector3().setFromMatrixPosition(relPoseToWorld(robot.homeMatrix, wp.pose));
    const sel = idx === selWp;
    const sph = new THREE.Mesh(
      new THREE.SphereGeometry(sel ? 0.016 : 0.010, 16, 16),
      new THREE.MeshStandardMaterial({ color: COL[wp.type] || 0xffffff, emissive: sel ? 0x554400 : 0x000000 })
    );
    sph.position.copy(p); wpGroup.add(sph);
    const lab = _makeLabel(String(idx));
    lab.position.copy(p).add(new THREE.Vector3(0, 0, 0.028)); wpGroup.add(lab);
  });
  wpGroup.visible = document.getElementById('chkPlan').checked;
}

// 把"当前位姿"作为起点 prepend, 使 当前点→第0点→…→第N点 全程走笛卡尔直线/曲线
function buildRunTraj() {
  if (!robot || !robot.homeMatrix || planner.waypoints.length < 1) return null;
  const startWp = captureWaypoint();
  return buildTrajectory({ waypoints: [startWp, ...planner.waypoints] }, robot, robot.homeMatrix, toMotorCdeg);
}

function simulate() {
  const traj = buildRunTraj();
  if (!traj) { flash('至少需要 1 个航点'); return; }
  drawRunPath(traj.ee);   // 展示真正会跑的完整串联轨迹
  stopSim();
  simRun = true; vizLock = true;
  const t = traj.t, jr = traj.jointsRad, dur = t[t.length - 1] || 0.1;
  const t0 = performance.now();
  function step() {
    if (!simRun) return;
    const el = (performance.now() - t0) / 1000;
    if (el >= dur) {
      robot.setJointAngles(jr[jr.length - 1]);
      document.getElementById('planBar').style.width = '100%';
      simRun = false; vizLock = false; flash('试行结束');
      return;
    }
    let i = 0; while (i < t.length - 1 && t[i + 1] < el) i++;
    robot.setJointAngles(jr[i]);
    document.getElementById('planBar').style.width = (el / dur * 100).toFixed(1) + '%';
    requestAnimationFrame(step);
  }
  planMsg(`试行: ${jr.length} 点, 时长 ${dur.toFixed(1)}s`);
  step();
}
function stopSim() { simRun = false; vizLock = false; }

// ======================================================================
// 启动
// ======================================================================
async function main() {
  buildJointTable();
  buildJogPanel();
  bindUI();
  bindCartesian();
  bindPlanner();
  resize();
  animate();
  startSSE();
  await refreshPorts();

  try {
    // 模型路径由后端决定 (见 server.py 的 --assets / ARM_TEACH_ASSETS)。
    let urdfUrl = '/assets/urdf/v2.5.urdf';
    try {
      const cfg = await (await fetch('/api/config')).json();
      if (cfg && cfg.urdf) urdfUrl = cfg.urdf;
    } catch (e) { /* 旧版后端无 /api/config, 回退到默认路径 */ }
    robot = await new RobotModel().load(urdfUrl);
    scene.add(robot.group);
    document.getElementById('loadingOverlay').classList.add('hidden');
    fitView();
    try { const sv = localStorage.getItem('arm_plan'); if (sv) planner.fromJSON(sv); } catch (e) {}
    renderPlanner();
    drawPlanned();
    flash('URDF 模型已加载');
  } catch (e) {
    document.getElementById('loadingOverlay').textContent = 'URDF 加载失败: ' + e.message;
    console.error(e);
  }
}
main();
