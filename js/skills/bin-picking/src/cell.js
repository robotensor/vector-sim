import { fnv64 } from './hash.js';

const STATE = 'mjSTATE_INTEGRATION';

export function yawCosSin(c) {
  const q = c.quat;
  if (Array.isArray(q) && q.length === 4 && q.every(Number.isFinite) && q[1] === 0 && q[2] === 0) {
    const n = q[0] * q[0] + q[3] * q[3];
    if (n > 0.5 && n < 2) return [(q[0] * q[0] - q[3] * q[3]) / n, (2 * q[0] * q[3]) / n];
  }
  if (!c.yaw) return [1, 0];
  return [Math.cos(c.yaw), Math.sin(c.yaw)];
}

export class Cell {
  constructor(mj, model, scene) {
    this.mj = mj;
    this.m = model;
    this.d = new mj.MjData(model);
    this.scene = scene;
    const M = (this.meta = scene.meta);
    this.cfg = M.config;
    this.R = M.robot;
    this.n = M.parts.length;
    this.active = new Uint8Array(this.n);
    this.stateSpec = mj.mjtState[STATE].value;
    this.nstate = mj.mj_stateSize(model, this.stateSpec);
    this.stateBuf = new mj.DoubleBuffer(this.nstate);
    this.bodyPart = new Int32Array(M.nbody).fill(-1);
    for (const p of M.parts) this.bodyPart[p.body] = p.i;
    this.isLeft = new Uint8Array(M.nbody);
    this.isRight = new Uint8Array(M.nbody);
    this.isGrip = new Uint8Array(M.nbody);
    for (const b of this.R.pad_left) this.isLeft[b] = 1;
    for (const b of this.R.pad_right) this.isRight[b] = 1;
    for (const b of this.R.gripper_bodies) this.isGrip[b] = 1;
    const m = model;
    this.hulls = M.parts.map((p) => p.hull_geoms.map((g) => {
      const mesh = m.geom_dataid[g];
      const a = m.mesh_vertadr[mesh], k = m.mesh_vertnum[mesh];
      return { g, v: Float32Array.from(m.mesh_vert.subarray(3 * a, 3 * (a + k))) };
    }));
    const c = M.crate, pile = c.kind === 'pile';
    if (pile) {
      const T = M.config.cell.table;
      this.crate = { kind: 'pile', x: (T.x[0] + T.x[1]) / 2, y: (T.y[0] + T.y[1]) / 2, cos: 1, sin: 0,
                     ix: (T.x[1] - T.x[0]) / 2, iy: (T.y[1] - T.y[0]) / 2, floor: c.floor_z, rim: c.rim_z };
    } else {
      const [cs, sn] = yawCosSin(c);
      this.crate = { kind: 'bin', x: c.pos[0], y: c.pos[1], cos: cs, sin: sn,
                     ix: c.inner_half[0], iy: c.inner_half[1], floor: c.floor_z, rim: c.rim_z };
    }
    this.ticks = 0;
    this.floorGeom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'table');
    for (let i = 0; i < this.n; i++) this._setPart(i, false);
  }

  static create(mj, files, scene) {
    const vfs = new mj.MjVFS();
    const entries = files instanceof Map ? files.entries() : Object.entries(files);
    for (const [p, bytes] of entries) if (p.endsWith('.stl')) vfs.addBuffer(p, bytes);
    const model = mj.MjModel.from_xml_string(scene.xml, vfs);
    vfs.delete();
    return new Cell(mj, model, scene);
  }

  dispose() {
    this.stateBuf.delete();
    this.d.delete();
    this.m.delete();
  }

  reset() {
    const { mj, m, d, R } = this;
    mj.mj_resetData(m, d);
    R.arm_joints_qpos.forEach((a, k) => { d.qpos[a] = R.q0[k]; });
    for (let i = 0; i < this.n; i++) this._setPart(i, false);
    this.setCtrl([...R.q0, 0]);
    this.ticks = 0;
    mj.mj_forward(m, d);
  }

  setCtrl(ctrl) {
    const { d, R } = this;
    for (let k = 0; k < 6; k++) d.ctrl[R.arm_actuators[k]] = ctrl[k];
    d.ctrl[R.gripper_actuator] = ctrl[6] * this.cfg.control.gripper_ctrl_max;
  }

  ctrl() {
    const { d, R } = this;
    return [...R.arm_actuators.map((a) => d.ctrl[a]), d.ctrl[R.gripper_actuator] / this.cfg.control.gripper_ctrl_max];
  }

  step(ctrl) {
    if (ctrl) this.setCtrl(ctrl);
    const { mj, m, d } = this;
    for (let k = 0; k < this.meta.substeps; k++) mj.mj_step(m, d);
    this.ticks++;
  }

  setRobotPose(q) {
    const { d, R } = this;
    R.arm_joints_qpos.forEach((a, k) => { d.qpos[a] = q[k]; });
    R.arm_dofs.forEach((a) => { d.qvel[a] = 0; });
    this.setCtrl([...q, 0]);
    this.mj.mj_forward(this.m, this.d);
  }

  _setPart(i, on) {
    const { m } = this;
    const p = this.meta.parts[i];
    for (const g of p.hull_geoms) { m.geom_contype[g] = on ? 1 : 0; m.geom_conaffinity[g] = on ? 1 : 0; }
    m.body_contype[p.body] = on ? 1 : 0;
    m.body_conaffinity[p.body] = on ? 1 : 0;
    m.body_gravcomp[p.body] = on ? 0 : 1;
    this.active[i] = on ? 1 : 0;
  }

  spawn(i, pos, quat) {
    const { d } = this;
    const p = this.meta.parts[i];
    this._setPart(i, true);
    for (let k = 0; k < 3; k++) d.qpos[p.qpos + k] = pos[k];
    for (let k = 0; k < 4; k++) d.qpos[p.qpos + 3 + k] = quat[k];
    for (let k = 0; k < 6; k++) d.qvel[p.dof + k] = 0;
    this.mj.mj_forward(this.m, this.d);
  }

  stackZ(i, x, y, quat, floorZ = 0, clearance = 0.005) {
    const { mj, m, d } = this, P = this.meta.parts, xp = d.xpos, ri = P[i].radius;
    let top = floorZ;
    const near = [];
    for (let j = 0; j < this.n; j++) {
      if (!this.active[j] || j === i) continue;
      const b = 3 * P[j].body, rj = P[j].radius;
      const dx = xp[b] - x, dy = xp[b + 1] - y;
      if (dx * dx + dy * dy < (ri + rj) * (ri + rj)) { top = Math.max(top, xp[b + 2] + rj); near.push(j); }
    }
    let z = top + ri + clearance;
    this.spawn(i, [x, y, z], quat);
    for (let round = 0; round < 8; round++) {
      const reach = z - floorZ + 0.05;
      let gap = reach;
      for (const g of P[i].hull_geoms) {
        if (this.floorGeom >= 0) gap = Math.min(gap, mj.mj_geomDistance(m, d, g, this.floorGeom, reach, null));
        for (const j of near) for (const h of P[j].hull_geoms) gap = Math.min(gap, mj.mj_geomDistance(m, d, g, h, reach, null));
      }
      if (!(gap - clearance > 0.001 && gap < reach)) break;
      z -= gap - clearance;
      this.spawn(i, [x, y, z], quat);
    }
    return z;
  }

  maxPartSpeed() {
    const v = this.d.qvel, P = this.meta.parts;
    let m2 = 0;
    for (let i = 0; i < this.n; i++) {
      if (!this.active[i]) continue;
      const k = P[i].dof, s2 = v[k] * v[k] + v[k + 1] * v[k + 1] + v[k + 2] * v[k + 2];
      if (s2 > m2) m2 = s2;
    }
    return Math.sqrt(m2);
  }

  park(i) {
    const { d } = this;
    const p = this.meta.parts[i], pk = this.cfg.cell.park;
    this._setPart(i, false);
    const pos = [pk.x - pk.dx * i, 0, pk.z];
    for (let k = 0; k < 3; k++) d.qpos[p.qpos + k] = pos[k];
    d.qpos[p.qpos + 3] = 1;
    for (let k = 4; k < 7; k++) d.qpos[p.qpos + k] = 0;
    for (let k = 0; k < 6; k++) d.qvel[p.dof + k] = 0;
    this.mj.mj_forward(this.m, this.d);
  }

  snapshot() {
    this.mj.mj_getState(this.m, this.d, this.stateBuf, this.stateSpec);
    return { state: Float64Array.from(this.stateBuf.GetView()), active: Uint8Array.from(this.active), ticks: this.ticks };
  }

  restore(s) {
    for (let i = 0; i < this.n; i++) this._setPart(i, !!s.active[i]);
    this.mj.mj_setState(this.m, this.d, s.state, this.stateSpec);
    this.ticks = s.ticks;
    this.mj.mj_forward(this.m, this.d);
  }

  hash() {
    this.mj.mj_getState(this.m, this.d, this.stateBuf, this.stateSpec);
    const st = this.stateBuf.GetView();
    return fnv64([new Uint8Array(st.buffer, st.byteOffset, st.byteLength), this.active]);
  }

  partZ(i) { return this.d.xpos[3 * this.meta.parts[i].body + 2]; }

  inside(i) {
    const b = this.meta.parts[i].body, x = this.d.xpos, c = this.crate, r = this.cfg.rules;
    const dx = x[3 * b] - c.x, dy = x[3 * b + 1] - c.y, z = x[3 * b + 2];
    const lx = c.cos * dx + c.sin * dy, ly = -c.sin * dx + c.cos * dy;
    return Math.abs(lx) <= c.ix && Math.abs(ly) <= c.iy && z >= c.floor - 0.01 && z <= c.rim + r.inside_above_rim;
  }

  bottomZ(i) {
    const { d } = this;
    let lo = Infinity;
    for (const { g, v } of this.hulls[i]) {
      const p = 3 * g, R = 9 * g;
      const x2 = d.geom_xmat[R + 6], y2 = d.geom_xmat[R + 7], z2 = d.geom_xmat[R + 8], oz = d.geom_xpos[p + 2];
      for (let k = 0; k < v.length; k += 3) {
        const z = oz + x2 * v[k] + y2 * v[k + 1] + z2 * v[k + 2];
        if (z < lo) lo = z;
      }
    }
    return lo;
  }

  contacts() {
    const { m, d, n } = this;
    const left = new Uint8Array(n), right = new Uint8Array(n), touch = new Uint8Array(n);
    let padAny = false;
    const con = d.contact;
    try {
      for (let k = 0, nc = d.ncon; k < nc; k++) {
        const c = con.get(k);
        const b1 = m.geom_bodyid[c.geom1], b2 = m.geom_bodyid[c.geom2];
        c.delete();
        for (const [a, b] of [[b1, b2], [b2, b1]]) {
          const i = this.bodyPart[a];
          if (i < 0 || !this.active[i]) continue;
          if (this.isGrip[b]) touch[i] = 1;
          if (this.isLeft[b]) { left[i] = 1; padAny = true; }
          if (this.isRight[b]) { right[i] = 1; padAny = true; }
        }
      }
    } finally {
      con.delete();
    }
    const held = new Uint8Array(n);
    for (let i = 0; i < n; i++) held[i] = left[i] && right[i] ? 1 : 0;
    return { held, touch, padAny };
  }

  gripClosed() {
    return this.d.qpos[this.R.driver_qpos] >= this.cfg.control.closed_driver;
  }

  tick(t, giveUp = false) {
    const { n, d } = this;
    const z = new Float64Array(n), bottomZ = new Float64Array(n), speed = new Float64Array(n);
    const inside = new Uint8Array(n), floor = new Uint8Array(n);
    const { held, touch, padAny } = this.contacts();
    const fb = this.cfg.rules.floor_below;
    let nHeld = 0;
    for (let i = 0; i < n; i++) {
      if (!this.active[i]) { bottomZ[i] = -Infinity; continue; }
      const p = this.meta.parts[i];
      z[i] = this.partZ(i);
      bottomZ[i] = this.bottomZ(i);
      const v0 = d.qvel[p.dof], v1 = d.qvel[p.dof + 1], v2 = d.qvel[p.dof + 2];
      speed[i] = Math.sqrt(v0 * v0 + v1 * v1 + v2 * v2);
      inside[i] = this.inside(i) ? 1 : 0;
      floor[i] = z[i] < -fb ? 1 : 0;
      nHeld += held[i];
    }
    const emptyClose = this.gripClosed() && nHeld === 0 && !padAny;
    return { t, held, touch, bottomZ, z, speed, inside, floor, active: Uint8Array.from(this.active), emptyClose, giveUp };
  }

  proprio() {
    const { d, R } = this;
    return { q: R.arm_joints_qpos.map((a) => d.qpos[a]), grip: Math.min(1, Math.max(0, d.qpos[R.driver_qpos] / 0.8)) };
  }

  tcp() {
    const s = this.R.tcp_site, d = this.d;
    return { pos: Array.from(d.site_xpos.subarray(3 * s, 3 * s + 3)), mat: Array.from(d.site_xmat.subarray(9 * s, 9 * s + 9)) };
  }

  activeParts() {
    const out = [];
    for (let i = 0; i < this.n; i++) if (this.active[i]) out.push(i);
    return out;
  }

  monitorOptions(target) {
    const r = this.cfg.rules;
    return { target, liftZ: this.crate.rim + r.lift_clear, holdS: r.hold_s, liftDz: r.lift_dz,
             tryS: r.try_s, endOnMiss: r.end_on_miss, settleSpeed: r.settle_speed,
             outSettleS: r.out_settle_s, lostS: r.lost_s };
  }
}
