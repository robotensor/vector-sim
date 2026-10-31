import { TryMonitor } from './rules.js';
import { f64ToB64, b64ToF64, b64encode, b64decode } from './hash.js';

export const CTRL = 7;
export const LOG_FORMAT = 'vector-episode/2';

const FILL_DEFAULTS = { floor_z: 0, clearance: 0.005, calm_speed: 0.05, calm_max_ticks: 20, settle_max_ticks: 120,
                        rest_speed: 0.01, rest_ticks: 6 };
const EDIT_SETTLE_TICKS = 120;

export function fillPlan(meta) {
  const F = meta.fill || {}, S = F.spawns;
  if (!Array.isArray(S)) throw new Error('scene has no fill schedule (meta.fill.spawns)');
  const mode = F.mode === 'place' ? 'place' : S.length && S.every((s) => s && Array.isArray(s.xy)) ? 'stack' : F.mode === 'stack' ? 'stack' : null;
  if (!mode) throw new Error('scene fill schedule is the old drop kind or malformed: recompose the scene with this vector-sim');
  for (const s of S) {
    const ok = s && Number.isInteger(s.part) && s.part >= 0 && s.part < meta.parts.length && Array.isArray(s.quat) && s.quat.length === 4
      && (mode === 'place' ? Array.isArray(s.pos) && s.pos.length === 3 : Array.isArray(s.xy) && s.xy.length === 2)
      && [...s.quat, ...(s.pos || s.xy)].every(Number.isFinite);
    if (!ok) throw new Error('scene fill schedule is malformed: recompose the scene with this vector-sim');
  }
  const out = { mode, spawns: S };
  for (const [k, d] of Object.entries(FILL_DEFAULTS)) {
    const v = F[k] === undefined || F[k] === null ? d : F[k];
    if (!Number.isFinite(v)) throw new Error(`scene meta.fill.${k} is not a number`);
    out[k] = v;
  }
  out.calm_max_ticks = Math.min(200, Math.max(1, Math.floor(out.calm_max_ticks)));
  out.settle_max_ticks = Math.min(1200, Math.max(1, Math.floor(out.settle_max_ticks)));
  out.rest_ticks = Math.min(out.settle_max_ticks, Math.max(1, Math.floor(out.rest_ticks)));
  out.max_ticks = (mode === 'stack' ? S.length * (out.calm_max_ticks + 1) : 0) + out.settle_max_ticks + 1;
  return out;
}

export class Episode {
  constructor(cell) {
    this.cell = cell;
    this.segments = [];
    this._units = [];
    this._unitBase = 0;
    this._open = null;
    this.tries = 0;
    this.cur = null;
    this.lastTry = null;
    this.base = null;
    this.phase = 'new';
  }

  get units() { return this._unitBase + this._units.length; }

  unitSegments(k) {
    const u = this._units[k - this._unitBase];
    return u || null;
  }

  _endUnit(segs) {
    this._units.push(segs);
    this._open = null;
    const c = this.cell, snap = c.snapshot();
    c.restore(snap);
    this.base = { snap, tries: this.tries };
    this.phase = 'ready';
    return this.units - 1;
  }

  checkpoint() {
    if (!this.base) throw new Error('no unit finished yet');
    const { snap, tries } = this.base;
    return { state: f64ToB64(snap.state), active: b64encode(snap.active), ticks: snap.ticks, tries };
  }

  restoreCheckpoint(cp) {
    const snap = { state: b64ToF64(cp.state), active: Uint8Array.from(b64decode(cp.active)), ticks: cp.ticks };
    if (snap.state.length !== this.cell.nstate) throw new Error('checkpoint state size');
    if (snap.active.length !== this.cell.n) throw new Error('checkpoint active size');
    if (!Number.isInteger(cp.tries) || cp.tries < 0) throw new Error('checkpoint tries');
    this.cell.restore(snap);
    this.segments = [];
    this._units = [];
    this._unitBase = cp.tries + 1;
    this._open = null;
    this.tries = cp.tries;
    this.cur = this.lastTry = null;
    this.base = { snap, tries: cp.tries };
    this.phase = 'ready';
    return this;
  }

  static restoreCheckpoint(cell, cp) {
    return new Episode(cell).restoreCheckpoint(cp);
  }

  beginFill() {
    const c = this.cell;
    c.reset();
    const F = fillPlan(c.meta);
    this.fill = { k: 0, next: 0, since: 0, calm: true, rest: 0, F, done: false, hold: [...c.meta.robot.q0, 0] };
    this.segments = [];
    this._units = [];
    this._unitBase = 0;
    this.tries = 0;
    this.phase = 'fill';
  }

  fillStep() {
    const f = this.fill, c = this.cell, F = f.F, S = F.spawns;
    if (F.mode === 'place' && f.k === 0) {
      for (const s of S) c.spawn(s.part, s.pos, s.quat);
      f.next = S.length;
    }
    if (f.next < S.length && (f.calm || f.since >= F.calm_max_ticks)) {
      const s = S[f.next++];
      c.stackZ(s.part, s.xy[0], s.xy[1], s.quat, F.floor_z, F.clearance);
      f.since = 0;
    }
    c.step(f.hold);
    f.k++;
    f.since++;
    const v = c.maxPartSpeed();
    if (f.next < S.length && f.k < F.max_ticks) {
      f.calm = v < F.calm_speed;
      return false;
    }
    f.rest = v < F.rest_speed ? f.rest + 1 : 0;
    if (f.rest >= F.rest_ticks || f.since >= F.settle_max_ticks || f.k >= F.max_ticks) {
      const t = c.tick(0);
      const out = [];
      for (let i = 0; i < c.n; i++) if (c.active[i] && !t.inside[i]) out.push(i);
      for (const i of out) c.park(i);
      const seg = { kind: 'fill', ticks: f.k, removed: out, hash: c.hash() };
      this.segments.push(seg);
      f.done = true;
      this._open = [seg];
      this.phase = 'setup';
      return true;
    }
    return false;
  }

  _settle() {
    const c = this.cell, F = fillPlan(c.meta), hold = c.ctrl();
    let k = 0, rest = 0;
    while (k < EDIT_SETTLE_TICKS) {
      c.step(hold);
      k++;
      rest = c.maxPartSpeed() < F.rest_speed ? rest + 1 : 0;
      if (rest >= F.rest_ticks) break;
    }
    const t = c.tick(0), out = [];
    for (let i = 0; i < c.n; i++) if (c.active[i] && !t.inside[i]) out.push(i);
    for (const i of out) c.park(i);
    return { ticks: k, removed: out };
  }

  _edit(seg) {
    if (this.phase !== 'setup') throw new Error(`the scene can be arranged only after the fill, before the first try (${this.phase})`);
    seg.hash = this.cell.hash();
    this.segments.push(seg);
    this._open.push(seg);
    return seg;
  }

  placePart(i, pos, quat) {
    const c = this.cell;
    if (!(Number.isInteger(i) && i >= 0 && i < c.n)) throw new Error('no such part');
    if (!(pos.length === 3 && quat.length === 4 && [...pos, ...quat].every(Number.isFinite))) throw new Error('bad pose');
    if (this.phase !== 'setup') throw new Error('the scene can be arranged only after the fill, before the first try');
    const T = c.meta.config.cell.table;
    if (pos[0] < T.x[0] || pos[0] > T.x[1] || pos[1] < T.y[0] || pos[1] > T.y[1] || pos[2] < 0 || pos[2] > 1.5) throw new Error('off the table');
    const qn = Math.sqrt(quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2);
    if (!(qn > 0.5 && qn < 2)) throw new Error('bad orientation');
    c.spawn(i, pos, quat);
    const r = this._settle();
    return this._edit({ kind: 'place', part: i, pos: pos.slice(), quat: quat.slice(), ticks: r.ticks, removed: r.removed });
  }

  removePart(i) {
    const c = this.cell;
    if (!(Number.isInteger(i) && i >= 0 && i < c.n) || !c.active[i]) throw new Error('no such part in the scene');
    if (this.phase !== 'setup') throw new Error('the scene can be arranged only after the fill, before the first try');
    c.park(i);
    return this._edit({ kind: 'remove', part: i });
  }

  setRobot(q) {
    const c = this.cell, R = c.meta.robot;
    if (this.phase !== 'setup') throw new Error('the scene can be arranged only after the fill, before the first try');
    if (!(q.length === 6 && q.every(Number.isFinite))) throw new Error('6 joint angles');
    R.joint_range.forEach(([lo, hi], k) => { if (q[k] < lo || q[k] > hi) throw new Error(`joint ${k + 1} out of range`); });
    c.setRobotPose(q);
    const r = this._settle();
    return this._edit({ kind: 'robot', q: q.slice(), ticks: r.ticks, removed: r.removed });
  }

  sealSetup() {
    if (this.phase !== 'setup') return this.units - 1;
    return this._endUnit(this._open);
  }

  beginTry(target) {
    if (this.phase === 'setup') this.sealSetup();
    if (this.phase !== 'ready') throw new Error(`a try cannot start now (${this.phase})`);
    const c = this.cell;
    const first = c.tick(0);
    this.cur = { kind: 'try', i: this.tries++, target, ctrl: [], calls: [], verdict: null, ticks: 0,
                 mon: new TryMonitor(first, c.monitorOptions(target)), start: c.snapshot() };
    this._open = [];
    this.phase = 'try';
    return this.cur;
  }

  abortTry() {
    if (!this.cur || this.cur.kind !== 'try') return;
    this.cell.restore(this.cur.start);
    this.tries--;
    this.cur = null;
    this._open = null;
    this.phase = 'ready';
  }

  noteCall(id, obsHash, giveUp = false) {
    const e = { tick: this.cur.ticks, id, obs: obsHash };
    if (giveUp) e.give_up = true;
    this.cur.calls.push(e);
  }

  stepTry(ctrl, giveUp = false) {
    const c = this.cell, cur = this.cur;
    if (!cur) throw new Error('no try under way');
    c.step(ctrl);
    for (let x = 0; x < CTRL; x++) cur.ctrl.push(ctrl[x]);
    cur.ticks++;
    const t = cur.ticks * c.meta.tick_s;
    const s = c.tick(t, giveUp);
    const v = cur.mon.update(s);
    if (v) {
      cur.verdict = { ...v, tick: cur.ticks };
      cur.lastTick = s;
      this._close();
    }
    return v;
  }

  _close() {
    const cur = this.cur, c = this.cell;
    const seg = { kind: 'try', n: cur.ticks, ctrl: f64ToB64(cur.ctrl), verdict: cur.verdict, hash: c.hash(),
                  i: cur.i, target: cur.target, calls: cur.calls };
    this.segments.push(seg);
    this._open.push(seg);
    cur.end = c.snapshot();
    cur.seg = seg;
    this.lastTry = cur;
    this.phase = cur.verdict.outcome === 'success' ? 'picked' : 'failed';
    this.cur = null;
  }

  endTry() {
    if (!['picked', 'failed'].includes(this.phase)) throw new Error(`no finished try to end (${this.phase})`);
    const c = this.cell, T = this.lastTry;
    c.restore(T.end);
    const removed = T.mon.toRemove(T.lastTick, T.verdict);
    for (const i of removed) c.park(i);
    const seg = { kind: 'take', try: T.i, removed, hash: c.hash() };
    this.segments.push(seg);
    const segs = this._open;
    segs.push(seg);
    const unit = this._endUnit(segs);
    return { removed, unit, segments: segs };
  }

  log(sceneId) {
    if (this._unitBase !== 0) throw new Error('an episode restored from a checkpoint has no full log');
    return { format: LOG_FORMAT, scene_id: sceneId, xml_sha256: this.cell.meta.xml_sha256, units: this._units };
  }
}
