// robot.js — 解析 URDF, 加载 STL 网格, 计算正运动学 (FK)
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const DEG = Math.PI / 180;

// package://e3/meshes/Link1.STL -> /assets/meshes/Link1.STL
function resolveMesh(filename) {
  return filename.replace(/^package:\/\/[^/]+\//, '/assets/');
}

function parseVec3(str, def = [0, 0, 0]) {
  if (!str) return def.slice();
  const p = str.trim().split(/\s+/).map(Number);
  return [p[0] || 0, p[1] || 0, p[2] || 0];
}

// 由 URDF 的 origin(xyz, rpy) 构造 4x4 变换矩阵: T = Trans * Rz * Ry * Rx
function originMatrix(xyz, rpy) {
  const m = new THREE.Matrix4().makeTranslation(xyz[0], xyz[1], xyz[2]);
  const rx = new THREE.Matrix4().makeRotationX(rpy[0]);
  const ry = new THREE.Matrix4().makeRotationY(rpy[1]);
  const rz = new THREE.Matrix4().makeRotationZ(rpy[2]);
  const rot = new THREE.Matrix4().multiplyMatrices(rz, ry).multiply(rx);
  return m.multiply(rot);
}

export class RobotModel {
  constructor() {
    this.group = new THREE.Group();
    this.links = new Map();     // name -> { name, mesh, visualMatrix, color }
    this.joints = [];           // {name, type, parent, child, origin, axis}
    this.movable = [];          // 可动关节 (revolute/continuous), 顺序即关节1..N
    this.root = null;
    this.eeLink = null;
    this._worldCache = new Map();
  }

  async load(urdfUrl) {
    const text = await (await fetch(urdfUrl)).text();
    const xml = new DOMParser().parseFromString(text, 'application/xml');

    // ---- 解析 links ----
    for (const link of xml.querySelectorAll('robot > link')) {
      const name = link.getAttribute('name');
      const visual = link.querySelector('visual');
      let meshFile = null, vorigin = [0, 0, 0], vrpy = [0, 0, 0], color = null, scale = [1, 1, 1];
      if (visual) {
        const o = visual.querySelector('origin');
        if (o) { vorigin = parseVec3(o.getAttribute('xyz')); vrpy = parseVec3(o.getAttribute('rpy')); }
        const mesh = visual.querySelector('geometry mesh');
        if (mesh) {
          meshFile = mesh.getAttribute('filename');
          scale = parseVec3(mesh.getAttribute('scale'), [1, 1, 1]);
        }
        const col = visual.querySelector('material color');
        if (col) color = parseVec3(col.getAttribute('rgba'));
      }
      this.links.set(name, {
        name, meshFile, color, scale,
        visualMatrix: originMatrix(vorigin, vrpy),
        mesh: null,
        worldMatrix: new THREE.Matrix4(),
      });
    }

    // ---- 解析 joints ----
    const childSet = new Set();
    for (const joint of xml.querySelectorAll('robot > joint')) {
      const name = joint.getAttribute('name');
      const type = joint.getAttribute('type');
      const parent = joint.querySelector('parent')?.getAttribute('link');
      const child = joint.querySelector('child')?.getAttribute('link');
      const o = joint.querySelector('origin');
      const xyz = parseVec3(o?.getAttribute('xyz'));
      const rpy = parseVec3(o?.getAttribute('rpy'));
      const a = joint.querySelector('axis');
      const axis = parseVec3(a?.getAttribute('xyz'), [1, 0, 0]);
      const lim = joint.querySelector('limit');
      const lower = lim ? parseFloat(lim.getAttribute('lower') || '-3.14') : -Math.PI;
      const upper = lim ? parseFloat(lim.getAttribute('upper') || '3.14') : Math.PI;
      const j = { name, type, parent, child, originMat: originMatrix(xyz, rpy), axis, angle: 0, lower, upper };
      this.joints.push(j);
      childSet.add(child);
      if (type === 'revolute' || type === 'continuous') this.movable.push(j);
    }

    // 根节点 = 不作为任何 joint child 的 link
    for (const name of this.links.keys()) {
      if (!childSet.has(name)) { this.root = name; break; }
    }
    // 末端 = 最后一个 link (load) 或最后一个关节的 child
    this.eeLink = this.joints.length ? this.joints[this.joints.length - 1].child : this.root;

    // ---- 加载 STL ----
    const loader = new STLLoader();
    const tasks = [];
    for (const link of this.links.values()) {
      if (!link.meshFile) continue;
      const url = resolveMesh(link.meshFile);
      const c = link.color || [0.8, 0.82, 0.93, 1];
      const task = loader.loadAsync(url).then((geo) => {
        geo.computeVertexNormals();
        const mat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(c[0], c[1], c[2]),
          metalness: 0.35, roughness: 0.55,
          transparent: (c[3] !== undefined && c[3] < 1),
          opacity: c[3] !== undefined ? c[3] : 1,
        });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.matrixAutoUpdate = false;
        mesh.scale.set(link.scale[0], link.scale[1], link.scale[2]);
        link.mesh = mesh;
        this.group.add(mesh);
      }).catch((e) => console.warn('STL 加载失败', url, e));
      tasks.push(task);
    }
    await Promise.all(tasks);
    this.setJointAngles([0, 0, 0, 0, 0, 0]);
    // 记录"六轴全 0"时末端的世界位姿, 作为末端位姿显示与笛卡尔遥控的原点参考
    this.homeMatrix = this._worldOf(this.eeLink).clone();
    return this;
  }

  // 当前末端 (eeLink) 的世界变换矩阵
  getEEMatrix() {
    return this._worldOf(this.eeLink).clone();
  }

  // 给定关节角(弧度数组), 计算末端世界变换矩阵, 不改变当前显示状态
  eeMatrixFor(anglesRad) {
    const saved = this.movable.map((j) => j.angle);
    this.setJointAngles(anglesRad);
    const m = this._worldOf(this.eeLink).clone();
    for (let i = 0; i < this.movable.length; i++) this.movable[i].angle = saved[i];
    this._updateFK();
    return m;
  }

  jointLimits() {
    return this.movable.map((j) => [j.lower, j.upper]);
  }

  // 设置可动关节角 (弧度数组, 顺序对应 movable[]/关节1..N)
  setJointAngles(anglesRad) {
    for (let i = 0; i < this.movable.length; i++) {
      this.movable[i].angle = anglesRad[i] || 0;
    }
    this._updateFK();
  }

  _localMatrix(joint) {
    const m = joint.originMat.clone();
    if (joint.type === 'revolute' || joint.type === 'continuous') {
      const axis = new THREE.Vector3(joint.axis[0], joint.axis[1], joint.axis[2]);
      if (axis.lengthSq() > 1e-9) axis.normalize();
      const r = new THREE.Matrix4().makeRotationAxis(axis, joint.angle);
      m.multiply(r);
    }
    return m;
  }

  _worldOf(linkName) {
    if (this._worldCache.has(linkName)) return this._worldCache.get(linkName);
    if (linkName === this.root) {
      const id = new THREE.Matrix4();
      this._worldCache.set(linkName, id);
      return id;
    }
    const joint = this.joints.find((j) => j.child === linkName);
    if (!joint) {
      const id = new THREE.Matrix4();
      this._worldCache.set(linkName, id);
      return id;
    }
    const parentWorld = this._worldOf(joint.parent);
    const world = parentWorld.clone().multiply(this._localMatrix(joint));
    this._worldCache.set(linkName, world);
    return world;
  }

  _updateFK() {
    this._worldCache = new Map();
    for (const link of this.links.values()) {
      const world = this._worldOf(link.name);
      link.worldMatrix.copy(world);
      if (link.mesh) {
        const m = world.clone().multiply(link.visualMatrix);
        link.mesh.matrix.copy(m);
        // 保留 scale
        if (link.scale[0] !== 1 || link.scale[1] !== 1 || link.scale[2] !== 1) {
          link.mesh.matrix.scale(new THREE.Vector3(link.scale[0], link.scale[1], link.scale[2]));
        }
        link.mesh.matrixWorldNeedsUpdate = true;
      }
    }
  }

  getEndEffectorPosition() {
    const w = this._worldOf(this.eeLink);
    const p = new THREE.Vector3();
    p.setFromMatrixPosition(w);
    return p;
  }

  // 给定关节角数组, 计算末端位置 (不改变当前显示状态)
  fkEndEffector(anglesRad) {
    const saved = this.movable.map((j) => j.angle);
    this.setJointAngles(anglesRad);
    const p = this.getEndEffectorPosition().clone();
    for (let i = 0; i < this.movable.length; i++) this.movable[i].angle = saved[i];
    this._updateFK();
    return p;
  }

  boundingBox() {
    const box = new THREE.Box3();
    for (const link of this.links.values()) {
      if (link.mesh) box.expandByObject(link.mesh);
    }
    return box;
  }
}

export { DEG };
