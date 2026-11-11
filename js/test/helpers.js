import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { findBundle, MISSING } from '../src/bundle.js';

const HERE = path.dirname(fileURLToPath(import.meta.url));
export const ROOT = path.resolve(HERE, '..', '..');
export const ASSETS = findBundle();
export const SKIP = ASSETS ? false : MISSING;
const FIXTURES = JSON.parse(fs.readFileSync(path.join(HERE, 'fixtures', 'scenes.json'), 'utf8'));
export const SETTINGS = FIXTURES.settings;
export const BUNDLE = ASSETS ? fixtureBundle(ASSETS) : null;

function fixtureBundle(base) {
  const parts = new Map();
  for (const sc of Object.values(FIXTURES.scenes))
    for (const [p, sha] of Object.entries(sc.meta.file_sha256 || {})) {
      const m = /^(assets\/lib\/[^/]+)\/ray\.stl$/.exec(p);
      if (m) parts.set(m[1], sha);
    }
  const index = fs.readFileSync(path.join(base, 'library', 'index.jsonl'), 'utf8');
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'vector-sim-fixtures-'));
  process.on('exit', () => fs.rmSync(dir, { recursive: true, force: true }));
  const link = (from, to) => fs.symlinkSync(path.join(base, from), path.join(dir, to));
  for (const e of fs.readdirSync(base)) if (e !== 'assets') link(e, e);
  fs.mkdirSync(path.join(dir, 'assets', 'lib'), { recursive: true });
  for (const e of fs.readdirSync(path.join(base, 'assets'))) if (e !== 'lib') link(path.join('assets', e), path.join('assets', e));
  for (const [to, sha] of parts) {
    const k = index.indexOf(`"ray":"${sha}"`);
    if (k < 0) throw new Error(`the asset bundle at ${base} has no part with the fixture mesh ${sha.slice(0, 12)}`);
    const row = JSON.parse(index.slice(index.lastIndexOf('\n', k) + 1, index.indexOf('\n', k)));
    link(path.dirname(row.files.ray), to);
  }
  return dir;
}

export function compose(settings = SETTINGS, seed = 11) {
  if (JSON.stringify(settings) !== JSON.stringify(SETTINGS)) throw new Error('only the fixture settings are composed here');
  const sc = FIXTURES.scenes[String(seed)];
  if (!sc) throw new Error(`no fixture scene for seed ${seed}`);
  return JSON.parse(JSON.stringify(sc));
}

let PICKS = null;

// Rows of picks recorded on the fixture scenes, for the tests that need a try that ends in a verdict. Keyed by the
// cell's state hash, the target and the depth.
export function recordedPick(cell, target, { depth = 0.0 } = {}) {
  PICKS ??= JSON.parse(fs.readFileSync(path.join(HERE, 'fixtures', 'picks.json'), 'utf8')).picks;
  const key = `${cell.hash()}:${target}:${depth}`, b64 = PICKS[key];
  if (!b64) throw new Error(`no pick recorded for ${key}`);
  const buf = Buffer.from(b64, 'base64'), flat = new Float64Array(buf.buffer, buf.byteOffset, buf.length / 8);
  const rows = [];
  for (let k = 0; k < flat.length; k += 7) rows.push(Array.from(flat.subarray(k, k + 7)));
  return rows;
}

export function highestPart(cell, margin = 0.08) {
  let best = -1;
  const c = cell.crate;
  for (let i = 0; i < cell.n; i++) {
    if (!cell.active[i] || !cell.inside(i)) continue;
    const b = cell.meta.parts[i].body, x = cell.d.xpos;
    const dx = x[3 * b] - c.x, dy = x[3 * b + 1] - c.y;
    const lx = c.cos * dx + c.sin * dy, ly = -c.sin * dx + c.cos * dy;
    if (Math.abs(lx) > c.ix - margin || Math.abs(ly) > c.iy - margin) continue;
    if (best < 0 || cell.partZ(i) > cell.partZ(best)) best = i;
  }
  return best;
}
