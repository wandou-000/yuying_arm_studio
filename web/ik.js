// ik.js — 数值逆运动学 (阻尼最小二乘 DLS), 基于 robot.js 的正运动学
import * as THREE from 'three';

// 6x6 线性方程组求解 (高斯消元, 带部分主元)
function solve6(A, b) {
  const n = b.length;
  const M = A.map((row, i) => row.concat([b[i]]));
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    if (Math.abs(M[piv][col]) < 1e-12) continue;
    [M[col], M[piv]] = [M[piv], M[col]];
    const d = M[col][col];
    for (let c = col; c <= n; c++) M[col][c] /= d;
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col];
      if (f === 0) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row) => row[n]);
}

// 两个位姿矩阵之间的 6 维误差 [dx,dy,dz, wx,wy,wz] (位置差 + 旋转向量)
export function poseError(cur, target) {
  const pc = new THREE.Vector3().setFromMatrixPosition(cur);
  const pt = new THREE.Vector3().setFromMatrixPosition(target);
  const dp = pt.clone().sub(pc);
  // 旋转误差: R_err = R_target * R_cur^T → 轴角向量
  const qc = new THREE.Quaternion().setFromRotationMatrix(cur);
  const qt = new THREE.Quaternion().setFromRotationMatrix(target);
  const qe = qt.clone().multiply(qc.clone().invert());
  if (qe.w < 0) { qe.x = -qe.x; qe.y = -qe.y; qe.z = -qe.z; qe.w = -qe.w; }
  let angle = 2 * Math.acos(Math.min(1, Math.max(-1, qe.w)));
  const s = Math.sqrt(1 - qe.w * qe.w);
  let wx = 0, wy = 0, wz = 0;
  if (s > 1e-6) {
    wx = qe.x / s * angle; wy = qe.y / s * angle; wz = qe.z / s * angle;
  }
  return [dp.x, dp.y, dp.z, wx, wy, wz];
}

// 数值雅可比 (6x6): 每个关节微扰, 观察末端位姿变化
function jacobian(robot, angles) {
  const eps = 1e-4;
  const base = robot.eeMatrixFor(angles);
  const J = [[], [], [], [], [], []];
  for (let j = 0; j < 6; j++) {
    const a = angles.slice();
    a[j] += eps;
    const e = poseError(base, robot.eeMatrixFor(a)); // (扰动后 - 基准)/eps
    for (let r = 0; r < 6; r++) J[r][j] = e[r] / eps;
  }
  return J;
}

/**
 * 求解 IK: 从 seedAngles 出发, 使末端到达 targetMatrix。
 * 返回 { ok, angles, err }。posWeight 让位置/姿态权衡 (姿态单位是弧度, 量级不同)。
 */
export function solveIK(robot, targetMatrix, seedAngles, opts = {}) {
  const maxIter = opts.maxIter || 60;
  const lambda = opts.lambda || 0.05;       // 阻尼
  const posTol = opts.posTol || 0.001;      // 1mm
  const rotTol = opts.rotTol || 0.01;       // ~0.6°
  const limits = robot.jointLimits();
  let q = seedAngles.slice();

  for (let it = 0; it < maxIter; it++) {
    const cur = robot.eeMatrixFor(q);
    const e = poseError(cur, targetMatrix);
    const posErr = Math.hypot(e[0], e[1], e[2]);
    const rotErr = Math.hypot(e[3], e[4], e[5]);
    if (posErr < posTol && rotErr < rotTol) {
      return { ok: true, angles: q, err: posErr };
    }
    const J = jacobian(robot, q);
    // dq = J^T (J J^T + λ²I)^-1 e
    const JJt = [];
    for (let r = 0; r < 6; r++) {
      JJt[r] = [];
      for (let c = 0; c < 6; c++) {
        let s = 0;
        for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k];
        JJt[r][c] = s + (r === c ? lambda * lambda : 0);
      }
    }
    const y = solve6(JJt, e);
    const dq = new Array(6).fill(0);
    for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let r = 0; r < 6; r++) s += J[r][j] * y[r];
      dq[j] = s;
    }
    // 步长限幅, 防止发散
    let maxStep = 0;
    for (let j = 0; j < 6; j++) maxStep = Math.max(maxStep, Math.abs(dq[j]));
    const cap = 0.2; // 单步最大 0.2 rad
    const scale = maxStep > cap ? cap / maxStep : 1;
    for (let j = 0; j < 6; j++) {
      q[j] += dq[j] * scale;
      if (limits[j]) q[j] = Math.min(limits[j][1], Math.max(limits[j][0], q[j]));
    }
  }
  const cur = robot.eeMatrixFor(q);
  const e = poseError(cur, targetMatrix);
  const posErr = Math.hypot(e[0], e[1], e[2]);
  const rotErr = Math.hypot(e[3], e[4], e[5]);
  return { ok: posErr < posTol * 5 && rotErr < rotTol * 5, angles: q, err: posErr };
}

// 由相对原点的 (x,y,z,roll,pitch,yaw) 构造目标世界矩阵: world = home * relative
export function relPoseToWorld(homeMatrix, pose) {
  const t = new THREE.Matrix4().makeTranslation(pose.x, pose.y, pose.z);
  const e = new THREE.Euler(pose.roll, pose.pitch, pose.yaw, 'XYZ');
  const r = new THREE.Matrix4().makeRotationFromEuler(e);
  const rel = t.multiply(r);
  return homeMatrix.clone().multiply(rel);
}

// 世界矩阵 → 相对原点位姿 {x,y,z,roll,pitch,yaw}
export function worldToRelPose(homeMatrix, worldMatrix) {
  const rel = homeMatrix.clone().invert().multiply(worldMatrix);
  const p = new THREE.Vector3().setFromMatrixPosition(rel);
  const e = new THREE.Euler().setFromRotationMatrix(rel, 'XYZ');
  return { x: p.x, y: p.y, z: p.z, roll: e.x, pitch: e.y, yaw: e.z };
}
