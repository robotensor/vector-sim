import { fnv64 } from './hash.js';

const STATE = 'mjSTATE_INTEGRATION';
// crate_hits: a tick counts when a robot-vs-crate/table contact pushes harder than this (N), unless rules.crate_hit_force
export const CRATE_HIT_FORCE = 75;

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
    this.sleep = (model.opt.enableflags & mj.mjtEnableBit.mjENBL_SLEEP.value) !== 0;
    this.floorGeom = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM.value, 'table');
    // rules: pile scenes (meta.config.rules.mode == "pile") and the robot-vs-crate/table contacts behind crate_hits
    this.pileMode = (this.cfg.rules || {}).mode === 'pile';
    if (this.pileMode) this._fingerSides(model);
    this.crateBody = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY.value, 'crate');
    this.isRobot = new Uint8Array(M.nbody);
    for (const b of this.R.robot_bodies || []) this.isRobot[b] = 1;
    this.forceBuf = new mj.DoubleBuffer(6);
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
    this.forceBuf.delete();
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
    // resets: MuJoCo's automatic reset on a diverged state (time jumps back), counted for the pile rules
    for (let k = 0; k < this.meta.substeps; k++) {
      const t0 = d.time;
      mj.mj_step(m, d);
      if (!(d.time > t0)) this.resets = (this.resets || 0) + 1;
    }
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
    if (p.frozen) { this.active[i] = 0; return; }  // a frozen part is a static collider, never an active part
    for (const g of p.hull_geoms) { m.geom_contype[g] = on ? 1 : 0; m.geom_conaffinity[g] = on ? 1 : 0; }
    m.body_contype[p.body] = on ? 1 : 0;
    m.body_conaffinity[p.body] = on ? 1 : 0;
    m.body_gravcomp[p.body] = on ? 0 : 1;
    this.active[i] = on ? 1 : 0;
  }

  spawn(i, pos, quat, forward = true) {
    const { d } = this;
    const p = this.meta.parts[i];
    this._setPart(i, true);
    for (let k = 0; k < 3; k++) d.qpos[p.qpos + k] = pos[k];
    for (let k = 0; k < 4; k++) d.qpos[p.qpos + 3 + k] = quat[k];
    for (let k = 0; k < 6; k++) d.qvel[p.dof + k] = 0;
    if (forward) this.mj.mj_forward(this.m, this.d);
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

  // Rain fill: the height for part i dropped at (x, y), its bounding sphere clear of every part's bounding sphere and of
  // the floor by `clearance`, plus `drop`.
  rainZ(i, x, y, floorZ, clearance, drop) {
    const P = this.meta.parts, xp = this.d.xpos, ri = P[i].radius;
    let z = floorZ + ri;
    for (let j = 0; j < this.n; j++) {
      if (!this.active[j] || j === i) continue;
      const b = 3 * P[j].body, rr = ri + P[j].radius;
      const dx = xp[b] - x, dy = xp[b + 1] - y, d2 = dx * dx + dy * dy;
      if (d2 < rr * rr) z = Math.max(z, xp[b + 2] + Math.sqrt(rr * rr - d2));
    }
    return z + clearance + drop;
  }

  // Pile surface height over the bin floor: a height map on a `cell`-sized grid over the inner floor (each cell the top
  // of the highest hull piece whose box covers the cell centre, the floor if none), its q-quantile, as a share of the
  // floor-to-rim depth.
  pileLevel(cell = 0.01, q = 0.5) {
    const c = this.crate, d = this.d;
    const nx = Math.max(1, Math.round((2 * c.ix) / cell)), ny = Math.max(1, Math.round((2 * c.iy) / cell));
    const hx = (2 * c.ix) / nx, hy = (2 * c.iy) / ny, H = new Float64Array(nx * ny).fill(c.floor);
    for (let i = 0; i < this.n; i++) {
      if (!this.active[i] || !this.inside(i)) continue;
      for (const { g, v } of this.hulls[i]) {
        const p = 3 * g, R = 9 * g, X = d.geom_xmat, o = d.geom_xpos;
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, top = -Infinity;
        for (let k = 0; k < v.length; k += 3) {
          const wx = o[p] + X[R] * v[k] + X[R + 1] * v[k + 1] + X[R + 2] * v[k + 2] - c.x;
          const wy = o[p + 1] + X[R + 3] * v[k] + X[R + 4] * v[k + 1] + X[R + 5] * v[k + 2] - c.y;
          const wz = o[p + 2] + X[R + 6] * v[k] + X[R + 7] * v[k + 1] + X[R + 8] * v[k + 2];
          const lx = c.cos * wx + c.sin * wy, ly = -c.sin * wx + c.cos * wy;
          if (lx < x0) x0 = lx; if (lx > x1) x1 = lx; if (ly < y0) y0 = ly; if (ly > y1) y1 = ly; if (wz > top) top = wz;
        }
        const a0 = Math.max(0, Math.ceil((x0 + c.ix) / hx - 0.5)), a1 = Math.min(nx - 1, Math.floor((x1 + c.ix) / hx - 0.5));
        const b0 = Math.max(0, Math.ceil((y0 + c.iy) / hy - 0.5)), b1 = Math.min(ny - 1, Math.floor((y1 + c.iy) / hy - 0.5));
        for (let a = a0; a <= a1; a++) for (let b = b0; b <= b1; b++) if (top > H[a * ny + b]) H[a * ny + b] = top;
      }
    }
    const s = Array.from(H).sort((u, w) => u - w);
    const z = s[Math.min(s.length - 1, Math.max(0, Math.floor(q * (s.length - 1))))];
    return (z - c.floor) / (c.rim - c.floor);
  }

  // Top-down view of the pile on a `cell` grid (hull-piece boxes, as pileLevel): per active part inside the bin the
  // share of its cells where it is the highest part, and the centre and half sizes of its box in crate axes.
  surface(cell = 0.005) {
    const c = this.crate, d = this.d, n = this.n;
    const nx = Math.max(1, Math.round((2 * c.ix) / cell)), ny = Math.max(1, Math.round((2 * c.iy) / cell));
    const hx = (2 * c.ix) / nx, hy = (2 * c.iy) / ny, H = new Float64Array(nx * ny).fill(-Infinity), L = new Int32Array(nx * ny).fill(-1);
    const box = [], own = new Float64Array(n), tot = new Float64Array(n), out = new Array(n).fill(null);
    for (let i = 0; i < n; i++) {
      if (!this.active[i] || !this.inside(i)) continue;
      const B = { x0: Infinity, x1: -Infinity, y0: Infinity, y1: -Infinity, top: -Infinity, pieces: [] };
      for (const { g, v } of this.hulls[i]) {
        const p = 3 * g, R = 9 * g, X = d.geom_xmat, o = d.geom_xpos;
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity, top = -Infinity;
        for (let k = 0; k < v.length; k += 3) {
          const wx = o[p] + X[R] * v[k] + X[R + 1] * v[k + 1] + X[R + 2] * v[k + 2] - c.x;
          const wy = o[p + 1] + X[R + 3] * v[k] + X[R + 4] * v[k + 1] + X[R + 5] * v[k + 2] - c.y;
          const wz = o[p + 2] + X[R + 6] * v[k] + X[R + 7] * v[k + 1] + X[R + 8] * v[k + 2];
          const lx = c.cos * wx + c.sin * wy, ly = -c.sin * wx + c.cos * wy;
          if (lx < x0) x0 = lx; if (lx > x1) x1 = lx; if (ly < y0) y0 = ly; if (ly > y1) y1 = ly; if (wz > top) top = wz;
        }
        B.pieces.push([x0, x1, y0, y1, top]);
        B.x0 = Math.min(B.x0, x0); B.x1 = Math.max(B.x1, x1); B.y0 = Math.min(B.y0, y0); B.y1 = Math.max(B.y1, y1); B.top = Math.max(B.top, top);
      }
      box[i] = B;
      for (const [x0, x1, y0, y1, top] of B.pieces) {
        const a0 = Math.max(0, Math.ceil((x0 + c.ix) / hx - 0.5)), a1 = Math.min(nx - 1, Math.floor((x1 + c.ix) / hx - 0.5));
        const b0 = Math.max(0, Math.ceil((y0 + c.iy) / hy - 0.5)), b1 = Math.min(ny - 1, Math.floor((y1 + c.iy) / hy - 0.5));
        for (let a = a0; a <= a1; a++) for (let b = b0; b <= b1; b++) {
          const k = a * ny + b;
          if (top > H[k]) { H[k] = top; L[k] = i; }
        }
      }
    }
    for (let i = 0; i < n; i++) {
      const B = box[i];
      if (!B) continue;
      const a0 = Math.max(0, Math.ceil((B.x0 + c.ix) / hx - 0.5)), a1 = Math.min(nx - 1, Math.floor((B.x1 + c.ix) / hx - 0.5));
      const b0 = Math.max(0, Math.ceil((B.y0 + c.iy) / hy - 0.5)), b1 = Math.min(ny - 1, Math.floor((B.y1 + c.iy) / hy - 0.5));
      for (let a = a0; a <= a1; a++) for (let b = b0; b <= b1; b++) {
        const k = a * ny + b;
        let under = false;
        for (const [x0, x1, y0, y1] of B.pieces) {
          const lx = -c.ix + (a + 0.5) * hx, ly = -c.iy + (b + 0.5) * hy;
          if (lx >= x0 && lx <= x1 && ly >= y0 && ly <= y1) { under = true; break; }
        }
        if (!under) continue;
        tot[i]++;
        if (L[k] === i) own[i]++;
      }
      out[i] = { exposed: tot[i] ? own[i] / tot[i] : 0, cx: (B.x0 + B.x1) / 2, cy: (B.y0 + B.y1) / 2,
                 hx: (B.x1 - B.x0) / 2, hy: (B.y1 - B.y0) / 2, top: B.top };
    }
    return out;
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
    const s = { state: Float64Array.from(this.stateBuf.GetView()), active: Uint8Array.from(this.active), ticks: this.ticks };
    if (this.sleep) s.asleep = this.asleepMask();
    return s;
  }

  // Every active part asleep (sleeping on).
  partsAsleep() {
    if (!this.sleep) return false;
    const A = this.d.tree_asleep, T = this.m.body_treeid, P = this.meta.parts;
    for (let i = 0; i < this.n; i++) if (this.active[i] && A[T[P[i].body]] < 0) return false;
    return true;
  }

  // Sleeping trees (1 = asleep), one byte per kinematic tree.
  asleepMask() {
    const a = this.d.tree_asleep, out = new Uint8Array(this.m.ntree);
    for (let t = 0; t < out.length; t++) out[t] = a[t] >= 0 ? 1 : 0;
    return out;
  }

  restore(s) {
    const { mj, m, d } = this;
    for (let i = 0; i < this.n; i++) this._setPart(i, !!s.active[i]);
    if (this.sleep) this._resetAsleep(s);
    mj.mj_setState(m, d, s.state, this.stateSpec);
    this.ticks = s.ticks;
    mj.mj_forward(m, d);
  }

  // With sleeping on, a restore must not depend on what the data held before (mj_setState keeps a sleeping tree asleep
  // when its bytes do not change), and the state API does not carry the sleep state. So: reset the data with the
  // snapshot's configuration as qpos0 and its sleeping trees marked "initialized asleep" (mj_resetData then puts them
  // to sleep at exactly these poses), then set the state. Same snapshot -> same data, in any cell, any process.
  _resetAsleep(s) {
    const { mj, m, d } = this, nq = m.nq, q0 = Float64Array.from(m.qpos0), pol = Int32Array.from(m.tree_sleep_policy);
    const A = s.asleep, INIT = mj.mjtSleepPolicy.mjSLEEP_INIT.value;
    try {
      if (A) {
        for (let k = 0; k < nq; k++) m.qpos0[k] = s.state[1 + k];
        for (let t = 0; t < m.ntree; t++) if (A[t]) m.tree_sleep_policy[t] = INIT;
      }
      mj.mj_resetData(m, d);
    } finally {
      for (let k = 0; k < nq; k++) m.qpos0[k] = q0[k];
      for (let t = 0; t < m.ntree; t++) m.tree_sleep_policy[t] = pol[t];
    }
  }

  hash() {
    this.mj.mj_getState(this.m, this.d, this.stateBuf, this.stateSpec);
    const st = this.stateBuf.GetView();
    const parts = [new Uint8Array(st.buffer, st.byteOffset, st.byteLength), this.active];
    if (this.sleep) parts.push(this.asleepMask());
    return fnv64(parts);
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

  // Pile scenes: a finger is its pad's chain of bodies up to the gripper base (pad, follower, spring link), so a part
  // clamped between one pad and the opposite finger's link is held. Older scenes keep the pads only.
  _fingerSides(m) {
    const grip = this.isGrip, par = m.body_parentid;
    for (const [pads, side] of [[this.R.pad_left, this.isLeft], [this.R.pad_right, this.isRight]]) {
      for (const p of pads) {
        for (let a = p; a > 0 && grip[a] && grip[par[a]]; a = par[a]) side[a] = 1;
      }
    }
  }

  // held: between both fingers (pads only in older scenes); touch: touching the gripper; padAny: a pad touches a part; crateF: the largest normal
  // force (N) of a robot-vs-crate/table contact; carried (pile scenes): parts touching the gripper or, through
  // part-part contacts, a part that does.
  contacts() {
    const { m, d, n } = this;
    const left = new Uint8Array(n), right = new Uint8Array(n), touch = new Uint8Array(n);
    const pairs = this.pileMode ? [] : null, rob = this.isRobot, crate = this.crateBody, table = this.floorGeom;
    let padAny = false, crateF = 0;
    const con = d.contact;
    try {
      for (let k = 0, nc = d.ncon; k < nc; k++) {
        const c = con.get(k);
        const g1 = c.geom1, g2 = c.geom2, b1 = m.geom_bodyid[g1], b2 = m.geom_bodyid[g2], ea = c.efc_address;
        c.delete();
        if (ea >= 0 && ((rob[b1] && (b2 === crate || g2 === table)) || (rob[b2] && (b1 === crate || g1 === table)))) {
          this.mj.mj_contactForce(m, d, k, this.forceBuf);
          const f = Math.abs(this.forceBuf.GetView()[0]);
          if (f > crateF) crateF = f;
        }
        if (pairs) {
          const i = this.bodyPart[b1], j = this.bodyPart[b2];
          if (i >= 0 && j >= 0 && i !== j) pairs.push(i, j);
        }
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
    let carried = null;
    if (pairs) {
      carried = Uint8Array.from(touch);
      for (let grew = true; grew;) {
        grew = false;
        for (let k = 0; k < pairs.length; k += 2) {
          const i = pairs[k], j = pairs[k + 1];
          if (carried[i] !== carried[j]) { carried[i] = carried[j] = 1; grew = true; }
        }
      }
    }
    return { held, touch, padAny, crateF, carried };
  }

  gripClosed() {
    return this.d.qpos[this.R.driver_qpos] >= this.cfg.control.closed_driver;
  }

  tick(t, giveUp = false) {
    const { n, d } = this;
    const z = new Float64Array(n), bottomZ = new Float64Array(n), speed = new Float64Array(n);
    const inside = new Uint8Array(n), floor = new Uint8Array(n);
    const { held, touch, padAny, crateF, carried } = this.contacts();
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
    const R = this.R, s3 = 3 * R.tcp_site, q = new Float64Array(6);
    for (let k = 0; k < 6; k++) q[k] = d.qpos[R.arm_joints_qpos[k]];
    const tcp = [d.site_xpos[s3], d.site_xpos[s3 + 1], d.site_xpos[s3 + 2]];
    return { t, held, touch, bottomZ, z, speed, inside, floor, active: Uint8Array.from(this.active), emptyClose, giveUp,
             tcp, q, crateF, carried, resets: this.resets || 0 };
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
             outSettleS: r.out_settle_s, lostS: r.lost_s, mode: this.pileMode ? 'pile' : 'classic',
             crateHitForce: Number.isFinite(r.crate_hit_force) ? r.crate_hit_force : CRATE_HIT_FORCE };
  }
}
