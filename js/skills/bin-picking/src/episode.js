import { TryMonitor } from './rules.js';
import { f64ToB64, b64ToF64, b64encode, b64decode } from './hash.js';
import { classify } from './plan.js';

export const CTRL = 7;
export const LOG_FORMAT = 'vector-episode/2';

const FILL_DEFAULTS = { floor_z: 0, clearance: 0.005, calm_speed: 0.05, calm_max_ticks: 20, settle_max_ticks: 120,
                        rest_speed: 0.01, rest_ticks: 6 };
const EDIT_SETTLE_TICKS = 120;

export function fillPlan(meta) {
  const F = meta.fill || {}, S = F.spawns;
  if (!Array.isArray(S)) throw new Error('scene has no fill schedule (meta.fill.spawns)');
  if (F.mode === 'rain') return rainPlan(meta, F, S);
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

const RAIN_DEFAULTS = { floor_z: 0, clearance: 0.003, drop: 0.01, level: 0.6, level_cell: 0.01, level_q: 0.5,
                        batch_min_ticks: 6, calm_speed: 0.05, calm_max_ticks: 40, settle_max_ticks: 200,
                        rest_speed: 0.01, rest_ticks: 6 };

// Rain fill: batches of parts (meta.fill.spawns, each {part, xy, quat, batch}) drop into the bin one batch at a time,
// each part just above the parts under it. A batch falls once the pile is calm (or after calm_max_ticks); the rain stops
// when the pile surface reaches `level` of the bin depth or the schedule ends, then the pile settles.
function rainPlan(meta, F, S) {
  const bad = () => new Error('scene fill schedule is malformed: recompose the scene with this vector-sim');
  const seen = new Uint8Array(meta.parts.length), batches = [];
  for (let k = 0; k < S.length; k++) {
    const s = S[k];
    const ok = s && Number.isInteger(s.part) && s.part >= 0 && s.part < meta.parts.length && !seen[s.part]
      && Array.isArray(s.quat) && s.quat.length === 4 && Array.isArray(s.xy) && s.xy.length === 2
      && [...s.quat, ...s.xy].every(Number.isFinite) && Number.isInteger(s.batch);
    if (!ok) throw bad();
    seen[s.part] = 1;
    if (s.batch === batches.length) batches.push([k, k + 1]);
    else if (s.batch === batches.length - 1) batches[s.batch][1] = k + 1;
    else throw bad();
  }
  const out = { mode: 'rain', spawns: S, batches };
  for (const [k, d] of Object.entries(RAIN_DEFAULTS)) {
    const v = F[k] === undefined || F[k] === null ? d : F[k];
    if (!Number.isFinite(v)) throw new Error(`scene meta.fill.${k} is not a number`);
    out[k] = v;
  }
  out.batch_min_ticks = Math.min(200, Math.max(1, Math.floor(out.batch_min_ticks)));
  out.calm_max_ticks = Math.min(400, Math.max(out.batch_min_ticks, Math.floor(out.calm_max_ticks)));
  out.settle_max_ticks = Math.min(2400, Math.max(1, Math.floor(out.settle_max_ticks)));
  out.rest_ticks = Math.min(out.settle_max_ticks, Math.max(1, Math.floor(out.rest_ticks)));
  out.level_cell = Math.min(0.1, Math.max(0.002, out.level_cell));
  out.level_q = Math.min(1, Math.max(0, out.level_q));
  out.max_ticks = batches.length * (out.calm_max_ticks + 1) + out.settle_max_ticks + 1;
  const B = F.bury;
  if (B !== undefined && B !== null) {
    const ok = B && Array.isArray(B.parts) && B.parts.length >= 1 && B.parts.length <= 24
      && (B.ring_n === undefined || (Number.isInteger(B.ring_n) && B.ring_n >= 0 && B.ring_n < B.parts.length))
      && (B.cap === undefined || (Number.isFinite(B.cap) && B.cap >= 0 && B.cap <= 2))
      && B.parts.every((i) => Number.isInteger(i) && i >= 0 && i < meta.parts.length && !seen[i])
      && new Set(B.parts).size === B.parts.length && Array.isArray(B.quats) && B.quats.length === B.parts.length
      && B.quats.every((q) => Array.isArray(q) && q.length === 4 && q.every(Number.isFinite))
      && [B.ring, B.wall, B.size].every(Number.isFinite) && Number.isInteger(B.tries) && B.tries >= 1 && B.tries <= 12
      && Number.isInteger(B.salt);
    if (!ok) throw bad();
    out.bury = B;
    out.max_ticks += (B.tries + 1) * out.settle_max_ticks;
  }
  return out;
}

// The parts a burying batch may target, best first: `top` parts (classify, ray meshes) at most `size` x the batch's
// first part (bounding radius), clear of the walls, nearest the crate centre (the camera axis); order rotated by the schedule's salt.
export function buryCandidates(cell, B) {
  const S = cell.surface(), c = cell.crate, K = classify(cell), ok = [];
  for (let i = 0; i < cell.n; i++) {
    const s = S[i];
    if (!s || K.cls[i] !== 'top') continue;
    if (Math.abs(s.cx) > c.ix - B.wall || Math.abs(s.cy) > c.iy - B.wall) continue;
    // small parts first (the batch's first part covers them best), then nearest the centre
    const big = cell.meta.parts[i].radius > B.size * cell.meta.parts[B.parts[0]].radius;
    ok.push({ i, key: (big ? 1 : 0) + Math.hypot(s.cx, s.cy) });
  }
  ok.sort((a, b) => a.key - b.key || a.i - b.i);
  const top = ok.slice(0, Math.ceil(B.tries / 2)).map((o) => o.i), k = top.length ? (B.salt >>> 0) % top.length : 0;
  return [...top.slice(k), ...top.slice(0, k)];
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
    const cp = { state: f64ToB64(snap.state), active: b64encode(snap.active), ticks: snap.ticks, tries };
    if (snap.asleep) cp.asleep = b64encode(snap.asleep);
    return cp;
  }

  restoreCheckpoint(cp) {
    const snap = { state: b64ToF64(cp.state), active: Uint8Array.from(b64decode(cp.active)), ticks: cp.ticks };
    if (snap.state.length !== this.cell.nstate) throw new Error('checkpoint state size');
    if (snap.active.length !== this.cell.n) throw new Error('checkpoint active size');
    if (!Number.isInteger(cp.tries) || cp.tries < 0) throw new Error('checkpoint tries');
    if (cp.asleep !== undefined) {
      snap.asleep = Uint8Array.from(b64decode(cp.asleep));
      if (!this.cell.sleep || snap.asleep.length !== this.cell.m.ntree) throw new Error('checkpoint sleep state size');
    }
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
    if (F.mode === 'rain') return this._rainStep();
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
    if (c.sleep) f.rest = c.partsAsleep() ? F.rest_ticks : 0;  // sleeping scenes: settled when every part sleeps
    else f.rest = v < F.rest_speed ? f.rest + 1 : 0;
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

  _rainStep() {
    const f = this.fill, c = this.cell, F = f.F, S = F.spawns;
    if (f.k === 0) Object.assign(f, { batch: 0, raining: true, level: 0, spawned: 0, settle: 0 });
    if (f.raining && (f.k === 0 || (f.since >= F.batch_min_ticks && (f.calm || f.since >= F.calm_max_ticks)))) {
      if (f.k > 0) {
        f.level = c.pileLevel(F.level_cell, F.level_q);
        if (f.level >= F.level) f.raining = false;
      }
      if (f.batch >= F.batches.length) f.raining = false;
      if (!f.raining && F.bury && f.cover === undefined) {
        f.buryList = buryCandidates(c, F.bury);
        f.cover = -1;
        f.rainSpawned = f.spawned;
        if (f.buryList.length) { f.burySnap = c.snapshot(); this._bury(0); }
      }
      if (f.raining && f.batch < F.batches.length) {
        const [a, b] = F.batches[f.batch++];
        for (let k = a; k < b; k++) {
          const s = S[k];
          c.spawn(s.part, [s.xy[0], s.xy[1], c.rainZ(s.part, s.xy[0], s.xy[1], F.floor_z, F.clearance, F.drop)], s.quat, false);
        }
        c.mj.mj_forward(c.m, c.d);
        f.spawned = b;
        f.since = 0;
      } else f.raining = false;
    }
    c.step(f.hold);
    f.k++;
    f.since++;
    const v = c.maxPartSpeed();
    if (f.raining && f.k < F.max_ticks) {
      f.calm = v < F.calm_speed;
      return false;
    }
    if (f.buryTry !== undefined && f.buryTry >= 0) {
      // the burying batch fell on T: once it rests, T must classify hidden (at most hidden_max_depth layers, camera-blind); else retry
      f.buryTicks++;
      if (!(c.partsAsleep() || f.buryTicks >= F.settle_max_ticks)) return false;
      const K = classify(c), T = f.cover, ok = K.cls[T] === 'hidden';
      (f.buryLog ||= []).push({ T, cls: K.cls[T], vis: K.vis[T], depth: K.depth[T], cam: K.camPx[T],
                                above: K.above[T].map((j) => [j, K.depth[j]]) });
      if (!ok && f.buryTry + 1 < F.bury.tries) {
        c.restore(f.burySnap);
        this._bury(f.buryTry + 1);
        return false;
      }
      if (!ok) f.cover = -1;
      f.buryTry = -1;
    }
    f.settle++;
    // sleeping on: the pile has settled when every part is asleep; else by speed
    if (c.sleep) f.rest = c.partsAsleep() ? F.rest_ticks : 0;
    else f.rest = v < F.rest_speed ? f.rest + 1 : 0;
    if (f.rest >= F.rest_ticks || f.settle >= F.settle_max_ticks || f.k >= F.max_ticks) {
      f.asleep = c.partsAsleep();
      const t = c.tick(0);
      const out = [];
      for (let i = 0; i < c.n; i++) if (c.active[i] && !t.inside[i]) out.push(i);
      for (const i of out) c.park(i);
      f.level = c.pileLevel(F.level_cell, F.level_q);
      const seg = { kind: 'fill', ticks: f.k, removed: out, hash: c.hash() };
      this.segments.push(seg);
      f.done = true;
      this._open = [seg];
      this.phase = 'setup';
      return true;
    }
    return false;
  }

  // Try t of the burying batch: its first part straight over T, the next `ring_n` (default: all the others) on a ring
  // around it (radius `ring` x the two bounding radii; the ring turns and tightens on every second round of
  // candidates), then the rest stacked over T as a second layer (on a circle of `cap` x the first part's radius).
  _bury(t) {
    const f = this.fill, c = this.cell, F = f.F, B = F.bury, L = f.buryList.length, P = c.meta.parts;
    f.buryTry = t;
    f.cover = f.buryList[t % L];
    f.buryTicks = 0;
    const round = Math.floor(t / L), S = c.surface()[f.cover], b = c.crate;
    const x = b.x + b.cos * S.cx - b.sin * S.cy, y = b.y + b.sin * S.cx + b.cos * S.cy;
    const ring = B.ring * (round % 2 ? 0.75 : 1), a0 = ((B.salt % 360) * Math.PI) / 180 + (round * Math.PI) / 5;
    const [p0, ...others] = B.parts, r0 = P[p0].radius;
    const nRing = B.ring_n === undefined ? others.length : B.ring_n, rest = others.slice(0, nRing), cap = others.slice(nRing);
    c.stackZ(p0, x, y, B.quats[0], F.floor_z, F.clearance);   // laid just onto T
    rest.forEach((p, k) => {
      const a = a0 + (2 * Math.PI * k) / rest.length, d = ring * (r0 + P[p].radius);
      // keep the ring part inside the walls (crate frame)
      let lx = S.cx + d * Math.cos(a), ly = S.cy + d * Math.sin(a);
      const mx = Math.max(0, b.ix - P[p].radius), my = Math.max(0, b.iy - P[p].radius);
      lx = Math.min(mx, Math.max(-mx, lx)); ly = Math.min(my, Math.max(-my, ly));
      const px = b.x + b.cos * lx - b.sin * ly, py = b.y + b.sin * lx + b.cos * ly;
      c.stackZ(p, px, py, B.quats[k + 1], F.floor_z, F.clearance);
    });
    cap.forEach((p, k) => {
      const a = a0 + Math.PI / cap.length + (2 * Math.PI * k) / cap.length, d = (B.cap ?? 0.5) * r0;
      let lx = S.cx + d * Math.cos(a), ly = S.cy + d * Math.sin(a);
      const mx = Math.max(0, b.ix - P[p].radius), my = Math.max(0, b.iy - P[p].radius);
      lx = Math.min(mx, Math.max(-mx, lx)); ly = Math.min(my, Math.max(-my, ly));
      const px = b.x + b.cos * lx - b.sin * ly, py = b.y + b.sin * lx + b.cos * ly;
      c.stackZ(p, px, py, B.quats[1 + nRing + k], F.floor_z, F.clearance);
    });
    f.spawned = f.rainSpawned + B.parts.length;
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

  // targetClass (pile scenes): the target's class from the scene (target_class), carried into the verdict.
  beginTry(target, targetClass = null) {
    if (this.phase === 'setup') this.sealSetup();
    if (this.phase !== 'ready') throw new Error(`a try cannot start now (${this.phase})`);
    const c = this.cell;
    const first = c.tick(0);
    this.cur = { kind: 'try', i: this.tries++, target, ctrl: [], calls: [], verdict: null, ticks: 0,
                 mon: new TryMonitor(first, { ...c.monitorOptions(target), targetClass }), start: c.snapshot() };
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
