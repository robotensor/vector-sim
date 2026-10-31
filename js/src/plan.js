import { Episode, CTRL } from './episode.js';
import { Camera } from './obs.js';
import { b64ToF64, fnv64 } from './hash.js';

export function planRows(ctrl) {
  let flat;
  if (typeof ctrl === 'string') flat = b64ToF64(ctrl);
  else if (ctrl instanceof Float64Array) flat = ctrl;
  else if (Array.isArray(ctrl) && ctrl.length && Array.isArray(ctrl[0])) flat = Float64Array.from(ctrl.flat());
  else flat = Float64Array.from(ctrl || []);
  if (flat.length % CTRL) throw new Error(`plan length ${flat.length} is not a multiple of ${CTRL}`);
  for (let k = 0; k < flat.length; k++) if (!Number.isFinite(flat[k])) throw new Error('plan has a non-finite value');
  return flat;
}

export function maxRows(cell) {
  return Math.ceil(cell.cfg.rules.try_s / cell.meta.tick_s - 1e-9);
}

export function runPlan(cell, start, ctrl, { limit = null } = {}) {
  const flat = planRows(ctrl), n = flat.length / CTRL;
  const cap = limit ?? maxRows(cell);
  if (n > cap) throw new Error(`plan has ${n} rows, more than a try lasts (${cap})`);
  const ep = Episode.restoreCheckpoint(cell, start.checkpoint);
  ep.beginTry(start.target);
  let v = null, k = 0, row = null;
  for (; k < n && !v; k++) {
    row = Array.from(flat.subarray(CTRL * k, CTRL * (k + 1)));
    v = ep.stepTry(row);
  }
  let held = 0;
  if (!v) {
    if (!row) { row = cell.ctrl().slice(); row[6] = 0; }
    while (!v) { v = ep.stepTry(row); held++; }
  }
  const tick = ep.lastTry.verdict.tick;
  return { ...v, tick, hash: cell.hash(), n: k, held, ctrl: ep.lastTry.seg.ctrl };
}

export function planHash(ctrl) {
  const flat = planRows(ctrl);
  return fnv64([new Uint8Array(flat.buffer, flat.byteOffset, flat.byteLength)]);
}

export function chooseTarget(cell, seed, { minPx = 100 } = {}) {
  const cam = new Camera(cell);
  try {
    const { seg } = cam.render(), px = new Uint32Array(cell.n + 1);
    for (let k = 0; k < seg.length; k++) if (seg[k] >= 1 && seg[k] <= cell.n) px[seg[k]]++;
    const tick = cell.tick(0), ok = [];
    for (let i = 0; i < cell.n; i++) if (cell.active[i] && tick.inside[i] && px[i + 1] >= minPx) ok.push(i);
    if (!ok.length) return null;
    let z = (Number(seed) >>> 0) + 0x9e3779b9 | 0;
    z = Math.imul(z ^ (z >>> 16), 0x85ebca6b); z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35); z = (z ^ (z >>> 16)) >>> 0;
    return ok[z % ok.length];
  } finally {
    cam.dispose();
  }
}
