// planner.js — 轨迹规划: 直线/曲线航点序列 → 关节轨迹 (经 IK)
// 速度规划已升级为 jerk-limited 闭式七段 S 曲线 (替换原"梯形+事后低通"方案):
//   · 加速度连续、jerk 有界 → 无顿挫、过点柔和;
//   · 起步柔和、收尾干脆(非对称 S 曲线: 减速用更大的 ACC_DN/JERK_DN), 不拖尾巴;
//   · 终点速度由闭式公式精确归零, 不靠事后整形, 无残余慢尾巴;
//   · 分段(停顿)处速度连续衔接, 非停顿边界不再各自归零硬拼;
//   · 拐角/反转(推拉运镜回头点)由 junction-deviation 限速主动减速到 ~0 再加速,
//     不再依赖 Menger 曲率(其对 180° 共线 cusp 数学上失效 → 满速冲过 → 顿挫)。
import * as THREE from 'three';
import { solveIK, relPoseToWorld } from './ik.js';

// 航点类型 (每个航点描述"如何从上一点到达本点")
export const WP_LINE = 'line';     // 直线: 从上一点走直线到本点
export const WP_CURVE = 'curve';   // 曲线: 作为样条途经点, 与相邻曲线点平滑相连

// 归一化类型, 兼容旧档 (旧的 curve_mid / curve_end 一律视为 curve)
export function normType(t) { return t === WP_LINE ? WP_LINE : WP_CURVE; }

export class Planner {
  constructor() {
    this.waypoints = [];
    this._undo = [];
    this._redo = [];
  }
  _snap() {
    this._undo.push(JSON.stringify(this.waypoints));
    if (this._undo.length > 100) this._undo.shift();
    this._redo = [];
  }
  add(wp) { this._snap(); this.waypoints.push(wp); }
  insert(idx, wp) { this._snap(); this.waypoints.splice(Math.max(0, idx), 0, wp); }
  modify(idx, wp) { if (idx < 0 || idx >= this.waypoints.length) return; this._snap(); this.waypoints[idx] = wp; }
  remove(idx) { if (idx < 0 || idx >= this.waypoints.length) return; this._snap(); this.waypoints.splice(idx, 1); }
  move(idx, dir) {
    const j = idx + dir;
    if (idx < 0 || j < 0 || idx >= this.waypoints.length || j >= this.waypoints.length) return;
    this._snap();
    const t = this.waypoints[idx]; this.waypoints[idx] = this.waypoints[j]; this.waypoints[j] = t;
  }
  undo() { if (!this._undo.length) return false; this._redo.push(JSON.stringify(this.waypoints)); this.waypoints = JSON.parse(this._undo.pop()); return true; }
  redo() { if (!this._redo.length) return false; this._undo.push(JSON.stringify(this.waypoints)); this.waypoints = JSON.parse(this._redo.pop()); return true; }
  clear() { this._snap(); this.waypoints = []; }
  toJSON() { return JSON.stringify({ version: 1, waypoints: this.waypoints }, null, 2); }
  fromJSON(s) {
    const o = JSON.parse(s);
    this.waypoints = (o.waypoints || []).map((w) => ({ ...w, type: normType(w.type) }));
    this._undo = []; this._redo = [];
  }
}

function poseQuat(p) { return new THREE.Quaternion().setFromEuler(new THREE.Euler(p.roll, p.pitch, p.yaw, 'XYZ')); }
function poseVec(p) { return new THREE.Vector3(p.x, p.y, p.z); }
function quatToPose(v, q) {
  const e = new THREE.Euler().setFromQuaternion(q, 'XYZ');
  return { x: v.x, y: v.y, z: v.z, roll: e.x, pitch: e.y, yaw: e.z };
}
function lerpPose(a, b, t) {
  const v = poseVec(a).lerp(poseVec(b), t);
  const q = poseQuat(a).slerp(poseQuat(b), t);
  return quatToPose(v, q);
}
// Catmull-Rom 位置插值
function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  const out = new THREE.Vector3();
  out.addScaledVector(p1, 2);
  out.addScaledVector(p2.clone().sub(p0), t);
  out.addScaledVector(p0.clone().multiplyScalar(2).addScaledVector(p1, -5).addScaledVector(p2, 4).addScaledVector(p3, -1), t2);
  out.addScaledVector(p1.clone().multiplyScalar(3).addScaledVector(p0, -1).addScaledVector(p2, -3).addScaledVector(p3, 1), t3);
  return out.multiplyScalar(0.5);
}

// 末端线速度 mm/s → m/s, 最低 2mm/s
const vOf = (sp) => Math.max(0.002, (sp || 50) / 1000);

// 直线段: 从 from(航点) 到 to(航点), 速度/顺滑/延迟取自 to 航点。
// 返回稠密 [{pose, vlin, dwell}], 含终点不含起点 (起点已在序列中)。
function sampleLine(from, to) {
  const a = from.pose, b = to.pose;
  const dp = poseVec(b).distanceTo(poseVec(a));
  const dq = poseQuat(a).angleTo(poseQuat(b));
  const n = Math.min(300, Math.max(4, Math.ceil(Math.max(dp / 0.004, dq / 0.04) * (1 + (to.smooth || 0) / 10))));
  const vlin = vOf(to.speed);
  const out = [];
  for (let k = 1; k <= n; k++) {
    out.push({ pose: lerpPose(a, b, k / n), vlin, dwell: k === n ? (to.delay || 0) / 1000 : 0 });
  }
  return out;
}

// 曲线段: Catmull-Rom 样条经过 anchor 与 run 中每个曲线航点。
// 每个子段的速度/顺滑/延迟取自其目标航点(run[i])。
function sampleCurve(anchor, run) {
  const ctrl = [anchor, ...run];
  const pts = ctrl.map((w) => poseVec(w.pose));
  const quats = ctrl.map((w) => poseQuat(w.pose));
  const out = [];
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = pts[i === 0 ? 0 : i - 1];
    const p1 = pts[i];
    const p2 = pts[i + 1];
    const p3 = pts[i + 2 < pts.length ? i + 2 : pts.length - 1];
    const target = run[i];                 // ctrl[i+1] 对应的航点
    const per = Math.max(6, 14 + (target.smooth || 0) * 2);
    const vlin = vOf(target.speed);
    for (let k = 1; k <= per; k++) {
      const t = k / per;
      const v = catmull(p0, p1, p2, p3, t);
      const q = quats[i].clone().slerp(quats[i + 1], t);
      out.push({ pose: quatToPose(v, q), vlin, dwell: k === per ? (target.delay || 0) / 1000 : 0 });
    }
  }
  return out;
}

// 把航点序列切成段: 连续的曲线点合并为一条样条; 直线点各成一段。
function segmentize(wps) {
  const segs = [];
  let i = 1, anchor = wps[0];
  while (i < wps.length) {
    if (normType(wps[i].type) === WP_CURVE) {
      const run = [];
      while (i < wps.length && normType(wps[i].type) === WP_CURVE) { run.push(wps[i]); i++; }
      segs.push({ kind: 'curve', anchor, run });
      anchor = run[run.length - 1];
    } else {
      segs.push({ kind: 'line', from: anchor, to: wps[i] });
      anchor = wps[i]; i++;
    }
  }
  return segs;
}

// 展开为稠密相对位姿采样: [{pose, vlin(m/s), dwell(s)}], 含起点
function flatten(wps) {
  const out = [{ pose: wps[0].pose, vlin: 0.05, dwell: 0 }];
  for (const seg of segmentize(wps)) {
    const samples = seg.kind === 'line' ? sampleLine(seg.from, seg.to)
                                        : sampleCurve(seg.anchor, seg.run);
    for (const s of samples) out.push(s);
  }
  return out;
}

// 回旋曲线(clothoid)单段: 曲率沿弧长线性变化 κ=κ0+dκ·s。数值积分(中点法)生成 2D 点。
// 返回 [{x,y,theta,kappa}], 起点 p0、起始切向 theta0。
function clothoidSeg(p0, theta0, kappa0, dkappa, L, n) {
  const out = [];
  const ds = L / n;
  let x = p0[0], y = p0[1], th = theta0, k = kappa0;
  out.push({ x, y, theta: th, kappa: k });
  for (let i = 1; i <= n; i++) {
    const kMid = k + 0.5 * dkappa * ds;
    const thMid = th + 0.5 * k * ds + 0.125 * dkappa * ds * ds;
    x += Math.cos(thMid) * ds;
    y += Math.sin(thMid) * ds;
    th += k * ds + 0.5 * dkappa * ds * ds;
    k += dkappa * ds;
    out.push({ x, y, theta: th, kappa: k });
  }
  return out;
}

// 对称折返过渡: clothoid(曲率0→1/R) + 圆弧(恒1/R) + clothoid(1/R→0) → 曲率 G2 连续。
// 起点(0,0)起始切向 +x。turnAngle 折返≈π, R 圆弧半径, ratio clothoid 占转角比例。
function clothoidTurn2D(turnAngle, R, ratio, nPer) {
  ratio = Math.max(0.01, Math.min(1, ratio));
  const phiCl = ratio * turnAngle / 2;
  const phiArc = turnAngle - 2 * phiCl;
  const Lcl = 2 * R * phiCl;
  const dk = Lcl > 1e-9 ? (1 / R) / Lcl : 0;
  const nCl = Math.max(4, Math.round(nPer * ratio));
  const nArc = Math.max(2, Math.round(nPer * (1 - ratio)));
  let pts = clothoidSeg([0, 0], 0, 0, dk, Lcl, nCl);
  let last = pts[pts.length - 1];
  if (phiArc > 1e-6) {
    const seg = clothoidSeg([last.x, last.y], last.theta, 1 / R, 0, R * phiArc, nArc);
    pts = pts.concat(seg.slice(1)); last = pts[pts.length - 1];
  }
  const seg = clothoidSeg([last.x, last.y], last.theta, 1 / R, -dk, Lcl, nCl);
  pts = pts.concat(seg.slice(1));
  return pts;
}

// 圆弧折返: 把 flat 序列里末端方向反转的尖点(cusp)替换成一小段圆弧 → 末端走 U 型弯,
// 折返点保持最低切向速度连续转向, 不再"趴 0"。在末端位置(x,y,z)上圆弧化, 姿态线性过渡。
function roundCusps(flat, radius, cusCos) {
  if (!radius || radius <= 0 || flat.length < 5) return flat;
  const pos = flat.map((s) => [s.pose.x, s.pose.y, s.pose.z]);
  const n = pos.length;
  const sArr = [0];
  for (let i = 1; i < n; i++) sArr[i] = sArr[i - 1] + Math.hypot(pos[i][0] - pos[i-1][0], pos[i][1] - pos[i-1][1], pos[i][2] - pos[i-1][2]);
  const Ltot = sArr[n - 1];
  if (Ltot < 2 * radius) return flat;
  const k = Math.max(2, Math.round(n * radius / Ltot));
  const cusps = [];
  for (let i = k; i < n - k; i++) {
    const d1 = [pos[i][0]-pos[i-k][0], pos[i][1]-pos[i-k][1], pos[i][2]-pos[i-k][2]];
    const d2 = [pos[i+k][0]-pos[i][0], pos[i+k][1]-pos[i][1], pos[i+k][2]-pos[i][2]];
    const n1 = Math.hypot(d1[0],d1[1],d1[2]), n2 = Math.hypot(d2[0],d2[1],d2[2]);
    if (n1 < 1e-9 || n2 < 1e-9) continue;
    const cos = (d1[0]*d2[0]+d1[1]*d2[1]+d1[2]*d2[2])/(n1*n2);
    if (cos < cusCos) cusps.push(i);
  }
  if (!cusps.length) return flat;
  const centers = [];
  let g = [cusps[0]];
  for (let i = 1; i < cusps.length; i++) {
    if (cusps[i] - cusps[i-1] <= k) g.push(cusps[i]);
    else { centers.push(g[Math.floor(g.length/2)]); g = [cusps[i]]; }
  }
  centers.push(g[Math.floor(g.length/2)]);
  const findIdxByArc = (sc) => { let lo=0,hi=n-1; while(hi-lo>1){const m=(lo+hi)>>1; if(sArr[m]<=sc)lo=m; else hi=m;} return lo; };
  const replaceRanges = [];
  for (const c of centers) {
    const sc = sArr[c];
    const sA = Math.max(0, sc - radius), sB = Math.min(Ltot, sc + radius);
    const iA = findIdxByArc(sA), iB = Math.min(n-1, findIdxByArc(sB)+1);
    if (iA <= 0 || iB >= n-1 || iB - iA < 2) continue;
    const A = pos[iA], B = pos[iB], C = pos[c];
    const din = [C[0]-A[0], C[1]-A[1], C[2]-A[2]];
    const ndin = Math.hypot(din[0],din[1],din[2]);
    if (ndin < 1e-9) continue;
    const u = din.map((x)=>x/ndin);
    let AB = [B[0]-A[0], B[1]-A[1], B[2]-A[2]];
    const proj = AB[0]*u[0]+AB[1]*u[1]+AB[2]*u[2];
    let nrm = [AB[0]-proj*u[0], AB[1]-proj*u[1], AB[2]-proj*u[2]];
    let nn = Math.hypot(nrm[0],nrm[1],nrm[2]);
    if (nn < 1e-6) {
      const refs = [[0,0,1],[0,1,0],[1,0,0]];
      for (const r of refs) { nrm=[u[1]*r[2]-u[2]*r[1], u[2]*r[0]-u[0]*r[2], u[0]*r[1]-u[1]*r[0]]; nn=Math.hypot(nrm[0],nrm[1],nrm[2]); if(nn>1e-6)break; }
    }
    nrm = nrm.map((x)=>x/nn);
    // 转角: 入方向 u 到出方向 w 需转过的角(折返≈π)。
    const dout = [B[0]-C[0], B[1]-C[1], B[2]-C[2]];
    const ndout = Math.hypot(dout[0],dout[1],dout[2]) || 1e-9;
    const w = dout.map((x)=>x/ndout);
    let cosTurn = u[0]*w[0]+u[1]*w[1]+u[2]*w[2];
    cosTurn = Math.max(-1, Math.min(1, cosTurn));
    const turnAngle = Math.acos(cosTurn);   // 转过的角(0~π); 折返≈π
    // ★ clothoid-圆弧-clothoid 折返过渡: 曲率 0→1/R(渐增)→恒(圆弧)→1/R→0(渐减), G2 连续,
    //   向心加速度连续无突变 → 无残留尖点、不被 junction 误压、向心加速度可控。
    // 在局部 2D 坐标(ex=u 入方向, ey=nrm 凸出方向)生成, 再映射回 3D。
    const R = radius;
    const ratio = 0.6;   // clothoid 段占转角比例(越大越柔, 1=纯clothoid)
    const nPer = Math.max(16, Math.round((Math.PI*R)/(Ltot/n)));
    const pts2d = clothoidTurn2D(turnAngle, R, ratio, nPer);
    // 2D 点起点(0,0)起始切向 +x; 平移使起点 = A, 局部 ex=u, ey=nrm
    const poseA = flat[iA].pose, poseB = flat[iB].pose;
    const arcPoses = [];
    const m = pts2d.length;
    for (let q = 0; q < m; q++) {
      const lx = pts2d[q].x, ly = pts2d[q].y;
      const px = A[0] + u[0]*lx + nrm[0]*ly;
      const py = A[1] + u[1]*lx + nrm[1]*ly;
      const pz = A[2] + u[2]*lx + nrm[2]*ly;
      const tt = q/(m-1);
      arcPoses.push({ x:px, y:py, z:pz, roll:poseA.roll+(poseB.roll-poseA.roll)*tt, pitch:poseA.pitch+(poseB.pitch-poseA.pitch)*tt, yaw:poseA.yaw+(poseB.yaw-poseA.yaw)*tt });
    }
    replaceRanges.push({ iA, iB, arcPoses, vlin: flat[c].vlin });
  }
  if (!replaceRanges.length) return flat;
  replaceRanges.sort((a,b)=>a.iA-b.iA);
  const out = [];
  let cur = 0;
  for (const r of replaceRanges) {
    for (let i = cur; i <= r.iA; i++) out.push(flat[i]);
    for (const p of r.arcPoses) out.push({ pose:p, vlin:r.vlin, dwell:0 });
    cur = r.iB;
  }
  for (let i = cur; i < n; i++) out.push(flat[i]);
  // 去除相邻重复点(圆弧拼接/弧顶可能产生): 重复点会使弧长 Lc=0、junction 方向异常 → 误压速到 0。
  const dedup = [out[0]];
  for (let i = 1; i < out.length; i++) {
    const a = dedup[dedup.length - 1].pose, b = out[i].pose;
    if (Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z) > 1e-6) dedup.push(out[i]);
  }
  return dedup;
}

// ---- 运动学限制 (用于时间参数化 / 速度规划) ----
// ★ 速度规划基准 = 末端笛卡尔弧长 (而非关节弧长)。原因: 六轴臂走直线时, 关节↔末端的
//   换算效率(Lj/Lc)沿途变化; 若维持"关节匀速", 末端速度就忽快忽慢 → 中途凹谷/台阶/顿挫。
//   改成沿末端弧长规划 → 末端做干净的匀速 S 曲线, 关节速度随 Lj/Lc 自动起伏配合(并受
//   JOINT_VMAX 保护, 不超电机上限)。所有速度量纲 = 末端线速度 m/s。
// 关节速度上限 rad/s (取自 v2.5.urdf 的 <limit velocity="1">)。仍作为硬约束: 在 Lj/Lc 大
//   (关节使不上劲)处自动把末端速度压低, 保证电机不超速。
const JOINT_VMAX = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
// 末端笛卡尔加速度 / jerk 上限 (m/s², m/s³)。控制末端起停柔和度:
//   ACC_MAX 调大→提速/刹车更迅捷; JERK_MAX 调大→加速度变化更快(略顿), 调小→更绵密柔和。
//   加减速对称 → 起停对称(不起步快/收尾拖, 也不收尾急刹顿一下)。
const ACC_MAX = 0.6;    // 末端加速度上限 m/s² (起步/收尾柔和; 推拉运镜经验值)
const JERK_MAX = 6.0;   // 末端 jerk 上限 m/s³ (≈ 10×ACC; 调小更绵密柔和)
const ACC_DN = ACC_MAX;   // 减速段 = 加速段 (对称: 收尾不顿不拖)
const JERK_DN = JERK_MAX; // 减速段 jerk = 加速段
const ACC_LAT = 0.6;    // 法向(向心)加速度上限 m/s²: 控制末端轨迹转弯/高曲率处自动减速
const V_FLOOR = 0.002;  // 末端速度下限 m/s (一般段避免除零/停滞)
const CUSP_EPS = 1e-4;  // 拐角/反转点允许触底的末端速度 m/s (≈停; 只有真 cusp 会到这)
// ★ 圆弧折返: 推拉往返的折返点是末端方向 180° 反转的尖点(cusp), 硬折返必须速度过零 →
//   "趴在 0 一段"。开启圆弧折返后, 在 cusp 处用一小段圆弧绕过(末端走 U 型弯而非 V 型尖角),
//   保持最低切向速度连续转向 → 折返点不再趴 0, 运动流畅。代价: 末端在折返点偏离原直线
//   最多 TURN_RADIUS(走个小圆角)。设 TURN_RADIUS=0 关闭(回到硬折返尖点)。
const TURN_RADIUS = 0.02;   // 圆弧折返半径 m (=20mm; 调大更圆滑偏离更多, 调小更贴近尖点)
const CUSP_COS = -0.7;      // 判定 cusp 的方向余弦阈值: < 此值(>~134°反转)才圆弧化
// 低速段压缩: 折返/收尾处速度按 S 曲线无限趋近 0, 会在"几乎为 0"区停留多个采样点(肉眼=
// "趴在 0 一段"). 这里把连续的超低速点(末端速度 < V_TRIM_MOVE)在时间上压缩, 只保留进入/
// 离开两点, 删掉中间几乎不动的冗余点 → 缩短可见的"趴 0"停留, 而位置序列仍精确(删的点末端
// 位移 < 该阈值×dt, 亚毫米级, 伺服落位吸收)。设 0 关闭。
const V_TRIM_MOVE = 0.012;  // 低速压缩阈值 m/s (≈12mm/s; 调大压得更狠但可能略显急, 调小更柔)
// 拐角(junction-deviation)限速: 跨圆滑窗口比较末端弦方向, 反转(cusp)→停、尖角→减速。
//   JUNCTION_DEV 调大→拐角过得更快(偏差更大); 调小→拐角更慢更贴合。单位 m(末端笛卡尔空间)。
const JUNCTION_DEV = 0.002;
const TURN_STENCIL = 3; // 拐角检测步距 (≈BLEND_WIN/2): 跨过圆滑窗口看真实转角, 否则 180°被摊薄漏检
const V_END = 0.0;      // 终点速度: 整条平滑减速到停 (闭式公式精确归零)
// 拐角圆滑(滑动平均)窗口(路径采样点, 奇数)。把尖角倒成小圆角, 抑制 IK 抖动。
const BLEND_WIN = 7;
// 输出采样步长 (s): 固定时间网格, ≤ 后端下发节拍。
const DT_OUT = 0.02;

// ===== 闭式七段 S 曲线: 单段求解 (point-to-point, jerk-limited) =====
// jerk-limited 把速度从 va 变到 vb 所需的弧长与时长 (七段中的加/减半程, a:0→±amax→0)。
function _ramp(va, vb, amax, jmax) {
  const dv = Math.abs(vb - va);
  if (dv < 1e-12) return { dist: 0, time: 0 };
  const dvJ = amax * amax / jmax;
  let T;
  if (dv <= dvJ) { T = 2 * (Math.sqrt(dv * jmax) / jmax); }
  else { const tj = amax / jmax, tc = (dv - dvJ) / amax; T = 2 * tj + tc; }
  return { dist: 0.5 * (va + vb) * T, time: T };
}
// _ramp 内部 t 时刻速度
function _rampVelAt(va, vb, amax, jmax, t) {
  const sign = Math.sign(vb - va), dv = Math.abs(vb - va);
  if (dv < 1e-12) return va;
  const dvJ = amax * amax / jmax;
  if (dv <= dvJ) {
    const apk = Math.sqrt(dv * jmax), tj = apk / jmax, T = 2 * tj;
    if (t <= 0) return va; if (t >= T) return vb;
    if (t < tj) return va + sign * 0.5 * jmax * t * t;
    const td = t - tj, vMid = va + sign * 0.5 * jmax * tj * tj, aMid = jmax * tj;
    return vMid + sign * (aMid * td - 0.5 * jmax * td * td);
  } else {
    const tj = amax / jmax, tc = (dv - dvJ) / amax, T = 2 * tj + tc;
    if (t <= 0) return va; if (t >= T) return vb;
    if (t < tj) return va + sign * 0.5 * jmax * t * t;
    if (t < tj + tc) { const td = t - tj, vAtTj = va + sign * 0.5 * jmax * tj * tj; return vAtTj + sign * amax * td; }
    const td = t - tj - tc, vAtTc = va + sign * (0.5 * jmax * tj * tj + amax * tc);
    return vAtTc + sign * (amax * td - 0.5 * jmax * td * td);
  }
}
// 单段: 已知起末速度 vs/ve、段弧长 D、巡航上限 vmax → 七段速度剖面 (终点速度精确)。
//   加速半程用 (accUp,jerkUp), 减速半程用 (accDn,jerkDn) → 非对称 S 曲线(起柔收脆)。
function _solveSeg(vs, ve, D, vmax, accUp, jerkUp, accDn, jerkDn) {
  vs = Math.max(0, vs); ve = Math.max(0, ve);
  vmax = Math.max(vmax, vs, ve, 1e-4);
  const distAt = (vp) => { const up = _ramp(vs, vp, accUp, jerkUp), dn = _ramp(vp, ve, accDn, jerkDn); return { d: up.dist + dn.dist, up, dn }; };
  let vpeak, up, dn, cruiseDist;
  const dAtMax = distAt(vmax);
  if (dAtMax.d <= D + 1e-12) { vpeak = vmax; up = dAtMax.up; dn = dAtMax.dn; cruiseDist = D - dAtMax.d; }
  else {
    let lo = Math.max(vs, ve), hi = vmax;
    if (distAt(lo).d > D + 1e-9) {
      // 段太短: 端点速度差所需弧长 > D。保端点速度, 用时间缩放使积分弧长==D。
      const dec = ve < vs, am = dec ? accDn : accUp, jm = dec ? jerkDn : jerkUp;
      const r = _ramp(vs, ve, am, jm), baseT = Math.max(r.time, 1e-6);
      const Tp = (vs + ve) > 1e-9 ? 2 * D / (vs + ve) : baseT;
      return { T: Tp, velAt: (t) => _rampVelAt(vs, ve, am, jm, baseT > 0 ? (t / Tp) * baseT : 0) };
    }
    for (let it = 0; it < 60; it++) { const mid = 0.5 * (lo + hi); if (distAt(mid).d <= D) lo = mid; else hi = mid; }
    vpeak = lo; const dd = distAt(vpeak); up = dd.up; dn = dd.dn; cruiseDist = Math.max(0, D - dd.d);
  }
  const Tup = up.time, Tcruise = vpeak > 1e-9 ? cruiseDist / vpeak : 0, Tdn = dn.time, T = Tup + Tcruise + Tdn;
  return {
    T,
    velAt: (t) => {
      if (t <= Tup) return _rampVelAt(vs, vpeak, accUp, jerkUp, t);
      if (t <= Tup + Tcruise) return vpeak;
      return _rampVelAt(vpeak, ve, accDn, jerkDn, t - Tup - Tcruise);
    },
  };
}

// 加减速可达速度包络(只限 acc, 前向+后向遍历): 作为限速牌的"加减速整形"。
//   前向受加速上限 accUp、后向受减速上限 accDn (非对称: 收尾更利落)。
function _accEnvelope(vcap, ds, accUp, accDn, v0, v1) {
  const N = vcap.length, v = vcap.slice();
  v[0] = Math.min(v[0], v0); v[N - 1] = Math.min(v[N - 1], v1);
  for (let i = 1; i < N; i++) { const l = Math.sqrt(v[i - 1] * v[i - 1] + 2 * accUp * ds); if (v[i] > l) v[i] = l; }
  for (let i = N - 2; i >= 0; i--) { const l = Math.sqrt(v[i + 1] * v[i + 1] + 2 * accDn * ds); if (v[i] > l) v[i] = l; }
  v[0] = Math.min(v[0], v0); v[N - 1] = Math.min(v[N - 1], v1);
  return v;
}

// 沿弧长 sc 在 (s[], arr[]) 上线性插值。
function interpByArc(arr, s, sc) {
  const M = s.length;
  if (sc <= s[0]) return arr[0].slice();
  if (sc >= s[M - 1]) return arr[M - 1].slice();
  let lo = 0, hi = M - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (s[mid] <= sc) lo = mid; else hi = mid; }
  const a = (sc - s[lo]) / ((s[hi] - s[lo]) || 1e-9);
  return arr[lo].map((v, j) => v + (arr[hi][j] - v) * a);
}

// 关节空间三点外接圆半径 (Menger 曲率), 直线返回 Infinity (Kahan 海伦公式)。
function curvatureRadius(a, b, c) {
  let ab = 0, bc = 0, ca = 0;
  for (let j = 0; j < a.length; j++) { ab += (b[j] - a[j]) ** 2; bc += (c[j] - b[j]) ** 2; ca += (c[j] - a[j]) ** 2; }
  ab = Math.sqrt(ab); bc = Math.sqrt(bc); ca = Math.sqrt(ca);
  if (ab < 1e-9 || bc < 1e-9) return Infinity;
  const s = [ab, bc, ca].sort((x, y) => y - x);
  const area2 = (s[0] + (s[1] + s[2])) * (s[2] - (s[0] - s[1])) * (s[2] + (s[0] - s[1])) * (s[0] + (s[1] - s[2]));
  if (area2 <= 0) return Infinity;
  const area = 0.25 * Math.sqrt(area2);
  if (area < 1e-12) return Infinity;
  return (ab * bc * ca) / (4 * area);
}

// 关节轨迹拐角圆滑: 每个内点取窗口内关节角滑动平均(端点锚定)。直线/已平滑段不变,
// 只把尖角倒圆; 顺带抹掉 IK 微抖。P(末端位置)同样平均, 用于笛卡尔限速换算。
function smoothJointPath(Q, P, win) {
  const M = Q.length;
  if (win < 3 || M < 3) return { Q, P };
  const h = Math.floor(win / 2);
  const Qs = Q.map((q) => q.slice());
  const Ps = P.map((p) => p.slice());
  for (let i = 1; i < M - 1; i++) {
    const lo = Math.max(0, i - h), hi = Math.min(M - 1, i + h), n = hi - lo + 1;
    const aq = [0, 0, 0, 0, 0, 0], ap = [0, 0, 0];
    for (let k = lo; k <= hi; k++) { for (let j = 0; j < 6; j++) aq[j] += Q[k][j]; for (let j = 0; j < 3; j++) ap[j] += P[k][j]; }
    for (let j = 0; j < 6; j++) Qs[i][j] = aq[j] / n;
    for (let j = 0; j < 3; j++) Ps[i][j] = ap[j] / n;
  }
  return { Q: Qs, P: Ps };
}

// 计算每点路径速度上限(限速牌): 关节限速 + 笛卡尔限速 + 曲率限速。
// 返回 vmaxArc[](rad/s, 逐点) 与累计关节弧长 s[]。
function speedCaps(Q, P, vcap) {
  const M = Q.length;
  // ★ 弧长基准 = 末端笛卡尔弧长 Lc。s[] 累积末端走过的距离(m)。
  const s = new Array(M); s[0] = 0;
  const Lj = new Array(M - 1), Lc = new Array(M - 1);
  for (let i = 0; i < M - 1; i++) {
    let j2 = 0; for (let j = 0; j < 6; j++) { const d = Q[i + 1][j] - Q[i][j]; j2 += d * d; }
    Lj[i] = Math.sqrt(j2);                                   // 关节弧长(rad), 仅用于关节限速换算
    Lc[i] = Math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1], P[i + 1][2] - P[i][2]);
    s[i + 1] = s[i] + Math.max(Lc[i], 1e-12);                // 末端弧长累积
  }
  // 逐段末端速度上限 vseg (m/s): 取 min(用户末端限速, 关节不超 1.0 反算的末端速度)。
  //   末端走 Lc 时关节 j 走 dQj; 末端速度 V(m/s) → 关节 j 速度 = V·|dQj|/Lc。
  //   要求 ≤ JOINT_VMAX[j] → V ≤ JOINT_VMAX[j]·Lc/|dQj|。Lj/Lc 大(关节使劲)处自动压低末端速度。
  const vseg = new Array(M - 1);
  for (let i = 0; i < M - 1; i++) {
    let vlim = vcap[i + 1] || vcap[i] || 0.05;               // 用户末端线速度上限 m/s
    if (Lc[i] > 1e-9) {
      for (let j = 0; j < 6; j++) {
        const dQj = Math.abs(Q[i + 1][j] - Q[i][j]);
        if (dQj > 1e-9) vlim = Math.min(vlim, JOINT_VMAX[j] * Lc[i] / dQj);  // 关节限速→末端速度
      }
    }
    vseg[i] = Math.max(V_FLOOR, vlim);
  }
  // 逐点上限 = 相邻段较小者, 再叠加末端轨迹曲率限速(末端走直线时 ρ=∞ 不限速)
  const vmaxArc = new Array(M);
  vmaxArc[0] = vseg[0]; vmaxArc[M - 1] = vseg[M - 2];
  for (let i = 1; i < M - 1; i++) {
    vmaxArc[i] = Math.min(vseg[i - 1], vseg[i]);
    const rho = curvatureRadius(P[i - 1], P[i], P[i + 1]);   // ★ 末端空间曲率(原为关节空间)
    if (isFinite(rho)) vmaxArc[i] = Math.min(vmaxArc[i], Math.sqrt(ACC_LAT * rho));
  }
  // 拐角/反转限速 (junction-deviation): 跨 ±h 看末端弦方向夹角。
  // ★ 圆弧折返修正: 只有"真尖点"(实际曲率半径 < TURN_RADIUS 的硬反转)才压到 ~0;
  //   已被 roundCusps 圆弧化的折返(曲率半径 ≈ TURN_RADIUS 的平滑 U 弯)交给曲率限速(向心
  //   加速度)处理, 不再被 junction 压 0 → 折返保持最低切向速度连续转向, 不趴 0。
  const h = Math.min(TURN_STENCIL, Math.floor((M - 1) / 2));
  const RADIUS_GUARD = (typeof TURN_RADIUS === 'number' && TURN_RADIUS > 0) ? TURN_RADIUS * 0.6 : 0;
  for (let i = h; i >= 1 && i < M - h; i++) {
    const d1 = [P[i][0] - P[i - h][0], P[i][1] - P[i - h][1], P[i][2] - P[i - h][2]];
    const d2 = [P[i + h][0] - P[i][0], P[i + h][1] - P[i][1], P[i + h][2] - P[i][2]];
    const n1 = Math.hypot(d1[0], d1[1], d1[2]), n2 = Math.hypot(d2[0], d2[1], d2[2]);
    if (n1 < 1e-9 || n2 < 1e-9) continue;
    let dot = (d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]) / (n1 * n2);
    const cosTheta = -Math.max(-1, Math.min(1, dot));        // 直行→-1, 反转→+1
    const sinHalf = Math.sqrt(Math.max(0, 0.5 * (1 - cosTheta)));
    if (sinHalf >= 1 - 1e-6) continue;                       // 直行: 不额外限速
    // 该点末端实际曲率半径(用相邻点, 最贴合局部): 圆弧≈TURN_RADIUS, 真尖点→极小。
    // 半径 ≥ RADIUS_GUARD(平滑圆弧, 含共线 Infinity)→ 交给曲率限速, 不压 0; 仅真尖点压。
    const rhoLocal = curvatureRadius(P[i - 1], P[i], P[i + 1]);
    if (RADIUS_GUARD > 0 && (!isFinite(rhoLocal) || rhoLocal >= RADIUS_GUARD)) continue;
    const Rj = JUNCTION_DEV * sinHalf / (1 - sinHalf);       // 拐角等效半径(m), 真尖点→0
    vmaxArc[i] = Math.min(vmaxArc[i], Math.sqrt(ACC_LAT * Rj));
  }
  return { s, vmaxArc, Ltot: s[M - 1] };
}

// 由一段(无中间停顿)的关节路径生成 jerk-limited 时间轨迹。
//   Q,P: 逐点关节角/末端位置;  vcap: 逐点笛卡尔限速(m/s);
//   vStart,vEnd: 段起末末端速度(m/s, 用于段间连续衔接, 整条末段=V_END=0)。
// 返回从 t=0 起的 { t, pos, jointsRad, ee, vEndActual }(vEndActual 供下段做起速)。
function generateSegment(Q, P, vcap, vStart, vEnd, toMotorCdeg) {
  const M = Q.length;
  // 限速牌 + 弧长 (先算出, sampleAt 需用到 _sArr)
  const { s: _sArr, vmaxArc, Ltot } = speedCaps(Q, P, vcap);
  const sampleAt = (sc) => {
    const q = interpByArc(Q, _sArr, sc);
    return { q, motor: q.map((a, j) => toMotorCdeg(j, a)), p: interpByArc(P, _sArr, sc) };
  };
  if (Ltot <= 1e-9 || M < 2) {
    const o = sampleAt(0);
    return { t: [0], pos: [o.motor], jointsRad: [o.q], ee: [o.p], vEndActual: 0 };
  }
  // acc 包络(限速牌经加减速整形) → 闭式 S 曲线的"巡航上限剖面"
  const ds = Ltot / (M - 1);
  // 把逐点 vmaxArc 重采样到等弧长网格(speedCaps 的 s 是累计末端笛卡尔弧长, 非等距, 这里用插值)
  const Ngrid = Math.max(50, Math.min(20000, Math.ceil(Ltot / 0.0008) + 1));
  const dsg = Ltot / (Ngrid - 1);
  const vcapGrid = new Array(Ngrid);
  for (let i = 0; i < Ngrid; i++) {
    const sc = i * dsg;
    // 在 (_sArr, vmaxArc) 上插值
    const vv = interpScalar(_sArr, vmaxArc, sc);
    // 地板降到 CUSP_EPS: 一般段 vmaxArc 已≥V_FLOOR, 仅真实拐角/反转点会触底 →
    // 允许速度降到 ~0 实现平滑停-走 (否则被 V_FLOOR 卡住 → 仍残留爬行/小顿挫)。
    vcapGrid[i] = Math.max(CUSP_EPS, vv);
  }
  const vtar = _accEnvelope(vcapGrid, dsg, ACC_MAX, ACC_DN, vStart, vEnd);
  const vtarAt = (sc) => {
    if (sc <= 0) return vtar[0]; if (sc >= Ltot) return vtar[Ngrid - 1];
    const fi = sc / dsg, i = Math.floor(fi), fr = fi - i;
    return vtar[i] + (vtar[Math.min(Ngrid - 1, i + 1)] - vtar[i]) * fr;
  };

  // 极小点切分 → 逐段闭式七段 S 曲线
  const nodes = [0];
  for (let i = 1; i < Ngrid - 1; i++) if (vtar[i] < vtar[i - 1] - 1e-9 && vtar[i] <= vtar[i + 1] + 1e-9) nodes.push(i);
  nodes.push(Ngrid - 1);
  const nd = [nodes[0]]; for (let k = 1; k < nodes.length; k++) if (nodes[k] > nd[nd.length - 1]) nd.push(nodes[k]);
  const nodeV = nd.map((idx, k) => k === 0 ? vStart : (k === nd.length - 1 ? vEnd : vtar[idx]));

  const subs = []; let tAcc = 0;
  for (let k = 0; k < nd.length - 1; k++) {
    const ia = nd[k], ib = nd[k + 1], D = (ib - ia) * dsg;
    if (D <= 1e-9) continue;
    let vseg = V_FLOOR; for (let i = ia; i <= ib; i++) vseg = Math.max(vseg, vtar[i]);
    const seg = _solveSeg(nodeV[k], nodeV[k + 1], D, vseg, ACC_MAX, JERK_MAX, ACC_DN, JERK_DN);
    // 预积分该段细网格弧长
    const Tk = seg.T, nf = Math.max(2, Math.ceil(Tk / (DT_OUT * 0.1)) + 1), dt = Tk / (nf - 1);
    const tau = new Array(nf), arc = new Array(nf), vv = new Array(nf);
    tau[0] = 0; vv[0] = seg.velAt(0); arc[0] = 0;
    for (let i = 1; i < nf; i++) { tau[i] = i * dt; vv[i] = seg.velAt(i * dt); arc[i] = arc[i - 1] + 0.5 * (vv[i] + vv[i - 1]) * dt; }
    const sc = arc[nf - 1] > 1e-12 ? D / arc[nf - 1] : 1; for (let i = 0; i < nf; i++) arc[i] *= sc;
    subs.push({ T: Tk, tBase: tAcc, sBase: ia * dsg, tau, arc, vv });
    tAcc += Tk;
  }
  const Ttot = tAcc;

  // 采样到 DT_OUT
  const t = [], pos = [], jointsRad = [], ee = [];
  const n = Math.max(2, Math.ceil(Ttot / DT_OUT) + 1);
  let si = 0;
  let lastV = vEnd;
  for (let m = 0; m < n; m++) {
    const tq = Math.min(Ttot, m * DT_OUT);
    while (si < subs.length - 1 && subs[si].tBase + subs[si].T <= tq) si++;
    const S = subs[si], lt = tq - S.tBase;
    let lo = 0, hi = S.tau.length - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (S.tau[mid] <= lt) lo = mid; else hi = mid; }
    const span = S.tau[hi] - S.tau[lo], fr = span > 1e-9 ? (lt - S.tau[lo]) / span : 0;
    const arcLoc = S.arc[lo] + (S.arc[hi] - S.arc[lo]) * fr;
    lastV = S.vv[lo] + (S.vv[hi] - S.vv[lo]) * fr;
    const o = sampleAt(Math.min(Ltot, S.sBase + arcLoc));
    t.push(+tq.toFixed(4)); jointsRad.push(o.q); pos.push(o.motor); ee.push(o.p);
  }
  return { t, pos, jointsRad, ee, vEndActual: vEnd };
}

// 标量按弧长插值 (speedCaps 的 s 非等距)
function interpScalar(s, arr, sc) {
  const M = s.length;
  if (sc <= s[0]) return arr[0];
  if (sc >= s[M - 1]) return arr[M - 1];
  let lo = 0, hi = M - 1;
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (s[mid] <= sc) lo = mid; else hi = mid; }
  const a = (sc - s[lo]) / ((s[hi] - s[lo]) || 1e-9);
  return arr[lo] + (arr[hi] - arr[lo]) * a;
}

// 仅几何路径(末端世界坐标点), 不做 IK, 用于展示区预览。
export function cartesianPath(planner, homeMatrix) {
  const wps = planner.waypoints;
  if (wps.length < 1) return [];
  const flat = wps.length < 2 ? [{ pose: wps[0].pose }] : flatten(wps);
  return flat.map((s) => {
    const p = new THREE.Vector3().setFromMatrixPosition(relPoseToWorld(homeMatrix, s.pose));
    return [p.x, p.y, p.z];
  });
}

/**
 * 由航点序列构建关节轨迹 (沿笛卡尔路径逐点 IK + jerk-limited 闭式 S 曲线时间参数化)。
 * 返回 { t:[s], pos:[[6]cdeg], jointsRad:[[6]], ee:[[x,y,z]] } 或 null。
 */
export function buildTrajectory(planner, robot, homeMatrix, toMotorCdeg) {
  const wps = planner.waypoints;
  if (wps.length < 2) return null;
  let flat = flatten(wps);
  flat = roundCusps(flat, TURN_RADIUS, CUSP_COS);   // ★ 圆弧折返: 尖点→U型弯, 折返不趴0

  // 1) 逐点 IK
  const Qraw = [], Praw = [], vcap = [], dwell = [];
  let prev = (wps[0].joints || [0, 0, 0, 0, 0, 0]).slice();
  for (const s of flat) {
    const world = relPoseToWorld(homeMatrix, s.pose);
    const res = solveIK(robot, world, prev);
    prev = res.angles;
    const p = new THREE.Vector3().setFromMatrixPosition(world);
    Qraw.push(res.angles.slice()); Praw.push([p.x, p.y, p.z]); vcap.push(s.vlin); dwell.push(s.dwell || 0);
  }

  // 2) 按"有停顿(dwell>0)"的点切段; 每段一条连续 S 曲线
  const cuts = [0];
  for (let i = 1; i < flat.length - 1; i++) if (dwell[i] > 0.001) cuts.push(i);
  cuts.push(flat.length - 1);

  // 3) 逐段规划再拼接。停顿点处两端速度=0(真停顿); 非停顿不存在(都被 dwell 切开),
  //    故每段起末速度均为 0 —— 但整段内部由闭式 S 曲线柔和加减速, 起停对称。
  const t = [], pos = [], jointsRad = [], ee = [];
  let tOff = 0;
  for (let c = 0; c < cuts.length - 1; c++) {
    const a = cuts[c], b = cuts[c + 1];
    if (b <= a) continue;
    const { Q, P } = smoothJointPath(Qraw.slice(a, b + 1), Praw.slice(a, b + 1), BLEND_WIN);
    const seg = generateSegment(Q, P, vcap.slice(a, b + 1), 0, 0, toMotorCdeg);
    for (let k = (c === 0 ? 0 : 1); k < seg.t.length; k++) {
      t.push(+(tOff + seg.t[k]).toFixed(4));
      jointsRad.push(seg.jointsRad[k]); pos.push(seg.pos[k]); ee.push(seg.ee[k]);
    }
    tOff += seg.t[seg.t.length - 1];
    // 停顿点: 原地保持 dwell 秒
    if (b < flat.length - 1 && dwell[b] > 0.001) {
      tOff += dwell[b];
      t.push(+tOff.toFixed(4));
      jointsRad.push(jointsRad[jointsRad.length - 1].slice());
      pos.push(pos[pos.length - 1].slice());
      ee.push(ee[ee.length - 1].slice());
    }
  }
  return { t, pos, jointsRad, ee };
}