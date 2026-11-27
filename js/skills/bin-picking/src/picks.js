import { Camera } from './obs.js';
import { pickTarget, pickSeed, isPile } from './plan.js';
import { CTRL } from './episode.js';
import { b64ToF64 } from './hash.js';

export const PICKS_DEFAULT = 10;

// Picks per scene: pile scenes `rules.picks` (10 when missing), other scenes 1.
export function pickCount(cell) {
  if (!isPile(cell)) return 1;
  const p = cell.cfg.rules.picks;
  return Number.isInteger(p) && p >= 1 ? p : PICKS_DEFAULT;
}

// A scene of several picks in one bin (pile scenes). Pick 0 is on the scene's target; after each pick's verdict its
// take-away runs (the outs, and the target when picked) and pick k + 1 gets a target chosen on the state right then by
// pickTarget(cell, pickSeed(seed, k + 1), { exclude: the targets of failed picks }). The robot goes on from where it
// is. The scene ends after `picks` picks, when no target is left, or at a physics_error.
export class PickRun {
  // camera: a Camera of this cell to render the image each target choice needs (lastImage() hands it out for the
  // observation of that state); else one is made and freed each time.
  constructor(ep, { target, target_class = null, seed = null, picks = null, camera = null } = {}) {
    this.camera = camera;
    this.ep = ep;
    this.cell = ep.cell;
    this.planned = picks ?? pickCount(this.cell);
    this.seed = seed ?? this.cell.meta.seed ?? 0;
    this.first = { pick: 0, target, target_class };
    this.picks = [];
    this.failed = [];
    this.k = -1;
    this.done = false;
    this.ticks = 0;
    this.image = null;
  }

  start() {
    this._begin(this.first);
    return this.first;
  }

  _begin(nx) {
    this.k = nx.pick;
    this.next = nx;
    this.ep.beginTry(nx.target, nx.target_class);
  }

  // One tick of the current pick. null while it runs; at its verdict { pick, verdict, record, next } (next null when
  // the scene is over).
  step(row, giveUp = false) {
    if (this.done) throw new Error('the scene is over');
    const v = this.ep.stepTry(row, giveUp);
    this.ticks++;
    if (!v) return null;
    const T = this.ep.lastTry, nx = this.next;
    const record = { pick: this.k, target: nx.target, target_class: nx.target_class, outcome: v.outcome, t: v.t,
                     tick: T.verdict.tick, part: v.part, detail: v.detail, ...T.mon.metrics(), target_class: nx.target_class };
    this.picks.push(record);
    this.ep.endTry();
    if (v.outcome !== 'success') this.failed.push(nx.target);
    let next = null;
    if (v.outcome !== 'physics_error' && this.k + 1 < this.planned) {
      const k = this.k + 1, cam = this.camera || new Camera(this.cell);
      let img;
      try { img = cam.render(); } finally { if (cam !== this.camera) cam.dispose(); }
      this.image = { ticks: this.cell.ticks, img };
      const p = pickTarget(this.cell, pickSeed(this.seed, k), { exclude: this.failed, seg: img.seg });
      if (p.target !== null) next = { pick: k, target: p.target, target_class: p.target_class };
    }
    if (next) this._begin(next);
    else this.done = true;
    return { pick: record.pick, verdict: v, record, next };
  }

  // The camera image rendered for the last target choice, while the cell is still in that state (else null).
  lastImage() {
    return this.image && this.image.ticks === this.cell.ticks && !this.ep.cur?.ticks ? this.image.img : null;
  }

  // The current pick's metrics so far (a scene stopped mid-pick).
  partial() {
    const cur = this.ep.cur;
    if (!cur || this.done) return null;
    return { pick: this.k, outcome: null,
             t: Math.round(1e6 * cur.ticks * this.cell.meta.tick_s) / 1e6, tick: cur.ticks, part: null, detail: null,
             ...cur.mon.metrics(), target: this.next.target, target_class: this.next.target_class };
  }

  totals(records = this.picks) {
    const out = new Set();
    let successes = 0, tcp = 0, joint = 0, hits = 0, tick = 0;
    for (const r of records) {
      if (r.outcome === 'success') successes++;
      for (const i of r.out_parts) out.add(i);
      tcp += r.tcp_path_m; joint += r.joint_path_rad; hits += r.crate_hits; tick += r.tick;
    }
    const r6 = (x) => Math.round(x * 1e6) / 1e6;
    return { picks_planned: this.planned, successes, outs: records.reduce((a, r) => a + r.outs, 0),
             out_parts: [...out].sort((a, b) => a - b), tcp_path_m: r6(tcp), joint_path_rad: r6(joint), crate_hits: hits,
             tick, t: r6(tick * this.cell.meta.tick_s) };
  }
}

// Replays a whole multi-pick scene's rows from the scene's checkpoint (already restored into `ep`): each row is one tick
// of the current pick, and the row after a pick's verdict is the first of the next pick. `hold` keeps the last row
// until the current pick's verdict when the rows run out (expert semantics; the scene stops there).
export function replayPicks(ep, start, rows, { hold = false, every = 0, limit = null, onTick = null } = {}) {
  const flat = typeof rows === 'string' ? b64ToF64(rows) : rows, n = flat.length / CTRL;
  if (limit !== null && n > limit) throw new Error(`the log has ${n} rows, more than the budget of ${limit}`);
  const run = new PickRun(ep, start), cell = ep.cell, marks = [];
  run.start();
  let k = 0, row = null, held = 0;
  for (; k < n && !run.done; k++) {
    row = Array.from(flat.subarray(CTRL * k, CTRL * (k + 1)));
    run.step(row);
    if (onTick) onTick(run, row, false);
    if (every && ((k + 1) % every === 0 || run.done || k === n - 1)) marks.push([k + 1, cell.hash()]);
  }
  if (hold && !run.done) {
    if (!row) { row = cell.ctrl().slice(); row[6] = 0; }
    const k0 = run.k;
    while (!run.done && run.k === k0) { run.step(row); held++; if (onTick) onTick(run, row, true); }
  }
  return { run, rows: k, extra: n - k, held, marks, hash: cell.hash() };
}
