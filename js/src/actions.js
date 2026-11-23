import { Episode, CTRL } from './episode.js';
import { b64encode, b64decode } from './hash.js';

export const ACTIONS_FORMAT = 'vector-actions/1';
export const CHAIN_FORMAT = 'vector-console/1';
export const EMPTY_ROWS_HASH = 'cbf29ce484222325';

const P_LO = 0x1b3, TWO32 = 4294967296;

export class RowsHash {
  constructor(hex = EMPTY_ROWS_HASH) {
    this.hi = parseInt(hex.slice(0, 8), 16) >>> 0;
    this.lo = parseInt(hex.slice(8, 16), 16) >>> 0;
    this.view = new DataView(new ArrayBuffer(8));
  }

  bytes(u8) {
    let hi = this.hi, lo = this.lo;
    for (let i = 0; i < u8.length; i++) {
      lo = (lo ^ u8[i]) >>> 0;
      const t = lo * P_LO;
      const carry = Math.floor(t / TWO32);
      hi = (hi * P_LO + carry + ((lo << 8) >>> 0)) >>> 0;
      lo = t >>> 0;
    }
    this.hi = hi;
    this.lo = lo;
    return this;
  }

  text(s) { return this.bytes(new TextEncoder().encode(String(s))); }

  rows(flat) {
    const v = this.view, b = new Uint8Array(v.buffer);
    for (let k = 0; k < flat.length; k++) {
      const x = flat[k];
      v.setFloat64(0, x === 0 ? 0 : x, true);
      this.bytes(b);
    }
    return this;
  }

  hex() { return this.hi.toString(16).padStart(8, '0') + this.lo.toString(16).padStart(8, '0'); }
}

export function rowsHash(rows) { return new RowsHash().rows(flatRows(rows)).hex(); }

export function flatRows(rows) {
  let flat;
  if (typeof rows === 'string') flat = decodeRows(rows);
  else if (rows instanceof Float64Array) flat = rows;
  else if (Array.isArray(rows) && rows.length && Array.isArray(rows[0])) flat = Float64Array.from(rows.flat());
  else flat = Float64Array.from(rows || []);
  if (flat.length % CTRL) throw new Error(`rows length ${flat.length} is not a multiple of ${CTRL}`);
  for (let k = 0; k < flat.length; k++) if (!Number.isFinite(flat[k])) throw new Error('rows hold a non-finite value');
  return flat;
}

export function encodeRows(rows) {
  const flat = Float64Array.from(rows instanceof Float64Array ? rows : flatRows(rows));
  for (let k = 0; k < flat.length; k++) if (flat[k] === 0) flat[k] = 0;
  return b64encode(new Uint8Array(flat.buffer));
}

export function decodeRows(s) {
  const u = b64decode(String(s));
  if (u.length % (8 * CTRL)) throw new Error(`rows of ${u.length} bytes are not whole rows`);
  return new Float64Array(u.slice().buffer);
}

export function chainSeed(nonce, session, sceneId) {
  return new RowsHash().text(`${CHAIN_FORMAT}:${nonce}:${session}:${sceneId}`).hex();
}

export function chainNext(prev, k, a, b, sh, flat) {
  return new RowsHash().text(`${prev}:${k}:${a}:${b}:${sh}:`).rows(flat).hex();
}

export function probeRows(ctrl, ticks = 40) {
  const rows = [];
  for (let k = 1; k <= ticks; k++) {
    const u = k <= ticks / 2 ? k : ticks - k;
    const row = ctrl.slice(0, 6).map((q, j) => q + 0.003 * u * (j % 2 ? 1 : -1));
    row.push(Math.min(1, Math.max(0, (k - ticks / 4) / (ticks / 2))));
    rows.push(row);
  }
  return rows;
}

export function probe(cell, checkpoint, { ticks = 40, every = 5 } = {}) {
  const ep = Episode.restoreCheckpoint(cell, checkpoint);
  const start = cell.hash(), rows = probeRows(cell.ctrl(), ticks), marks = [];
  for (let k = 0; k < rows.length; k++) {
    cell.step(rows[k]);
    if ((k + 1) % every === 0 || k === rows.length - 1) marks.push([k + 1, cell.hash()]);
  }
  ep.restoreCheckpoint(checkpoint);
  return { ticks, every, start, marks };
}

export function replayRows(cell, start, rows, { limit = null, every = 1, hold = false } = {}) {
  const flat = flatRows(rows), n = flat.length / CTRL;
  if (limit !== null && n > limit) throw new Error(`the log has ${n} rows, more than the budget of ${limit}`);
  const ep = Episode.restoreCheckpoint(cell, start.checkpoint);
  ep.beginTry(start.target);
  const marks = [];
  let v = null, k = 0, row = null;
  for (; k < n && !v; k++) {
    row = Array.from(flat.subarray(CTRL * k, CTRL * (k + 1)));
    v = ep.stepTry(row);
    if (every && ((k + 1) % every === 0 || v || k === n - 1)) marks.push([k + 1, cell.hash()]);
  }
  let held = 0;
  if (!v && hold) {
    if (!row) { row = cell.ctrl().slice(); row[6] = 0; }
    while (!v) { v = ep.stepTry(row); held++; }
  }
  const tick = v ? ep.lastTry.verdict.tick : k;
  return { outcome: v ? v.outcome : null, part: v ? v.part : null, detail: v ? v.detail : null,
           tick, t: Math.round(1e6 * tick * cell.meta.tick_s) / 1e6, rows: k, extra: n - k, held,
           hash: cell.hash(), marks };
}
