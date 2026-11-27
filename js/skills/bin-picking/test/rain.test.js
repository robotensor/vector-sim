import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, fillPlan, replayUnit } from '../src/index.js';
import { ASSETS, SKIP } from './helpers.js';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const RAIN = JSON.parse(fs.readFileSync(path.join(HERE, 'fixtures', 'rain.json'), 'utf8'));

function fill(cell) {
  const ep = new Episode(cell);
  ep.beginFill();
  while (!ep.fillStep());
  return ep;
}

// contacts of every part: sleeping trees skip collisions, so look with sleeping switched off for one forward pass
function contacts(cell) {
  const o = cell.m.opt, flags = o.enableflags;
  o.enableflags = flags & ~cell.mj.mjtEnableBit.mjENBL_SLEEP.value;
  cell.mj.mj_forward(cell.m, cell.d);
  o.enableflags = flags;
  const touch = new Uint8Array(cell.n), con = cell.d.contact;
  let pen = 0;
  for (let k = 0; k < cell.d.ncon; k++) {
    const c = con.get(k);
    pen = Math.min(pen, c.dist);
    for (const g of [c.geom1, c.geom2]) { const i = cell.bodyPart[cell.m.geom_bodyid[g]]; if (i >= 0) touch[i] = 1; }
    c.delete();
  }
  con.delete();
  return { touch, pen };
}

test('rain schedule: batches must be contiguous, parts unique', () => {
  const M = RAIN.meta, F = fillPlan(M);
  assert.equal(F.mode, 'rain');
  assert.equal(F.batches.at(-1)[1], M.fill.spawns.length);
  assert.ok(F.batches.every(([a, b], k) => b > a && (k === 0 || F.batches[k - 1][1] === a)));
  const bad = (spawns) => () => fillPlan({ ...M, fill: { ...M.fill, spawns } });
  const S = M.fill.spawns;
  assert.throws(bad([S[0], { ...S[1], batch: S[0].batch + 2 }]), /malformed/);
  assert.throws(bad([S[0], { ...S[1], part: S[0].part }]), /malformed/);
  assert.throws(bad([{ ...S[0], xy: [0] }]), /malformed/);
});

test('rain fill: a settled, supported pile that sleeps; same fill twice; checkpoint continues bit for bit', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const { files } = bundleFiles(ASSETS, RAIN);
  const cell = Cell.create(mj, files, RAIN);
  assert.ok(cell.sleep, 'pile scenes simulate with sleeping on');
  const ep = fill(cell), seg = ep.segments[0], f = ep.fill, F = f.F;
  const act = cell.activeParts();
  assert.ok(f.level >= F.level || f.spawned === F.spawns.length, `filled to ${f.level} (target ${F.level})`);
  assert.equal(act.length, f.spawned - seg.removed.length);
  assert.ok(act.every((i) => cell.inside(i)));
  ep.sealSetup();
  const cp = ep.checkpoint(), hash = cell.hash();

  const again = Cell.create(mj, files, RAIN), ep2 = fill(again);
  assert.equal(ep2.segments[0].hash, seg.hash, 'the fill is deterministic');
  assert.equal(ep2.segments[0].ticks, seg.ticks);
  again.dispose();

  // after a restore every part rests on something, nothing sinks into another
  Episode.restoreCheckpoint(cell, cp);
  const { touch, pen } = contacts(cell);
  assert.deepEqual(act.filter((i) => !touch[i]), [], 'every part touches a part, the crate or the floor');
  assert.ok(pen > -0.003, `deepest contact ${(-1000 * pen).toFixed(2)} mm`);

  // the restored pile is asleep (the checkpoint keeps the sleep state) and stays where it was
  Episode.restoreCheckpoint(cell, cp);
  const pos = () => act.map((i) => Array.from(cell.d.xpos.subarray(3 * cell.meta.parts[i].body, 3 * cell.meta.parts[i].body + 3)));
  const p0 = pos();
  let k = 0;
  for (; k < 60 && cell.d.ntree_awake > 1; k++) cell.step(null);
  assert.ok(cell.d.ntree_awake <= 1, `asleep after ${k} ticks (${cell.d.ntree_awake} trees awake)`);
  const drift = Math.max(...pos().map((p, j) => Math.hypot(p[0] - p0[j][0], p[1] - p0[j][1], p[2] - p0[j][2])));
  assert.ok(drift < 0.002, `the pile stays put (max ${(1000 * drift).toFixed(2)} mm)`);

  // a fixed ctrl sequence from the checkpoint gives the same state in this cell (used data) and in a fresh one
  const rows = [];
  const q0 = cell.meta.robot.q0;
  for (let t = 0; t < 30; t++) rows.push([...q0.map((q, j) => q + (j === 1 ? 0.004 * t : 0)), t > 15 ? 1 : 0]);
  const run = (c) => { Episode.restoreCheckpoint(c, cp); for (const r of rows) c.step(r); return c.hash(); };
  const fresh = Cell.create(mj, files, RAIN);
  const h1 = run(cell), h2 = run(cell), h3 = run(fresh);
  assert.equal(h1, h2);
  assert.equal(h1, h3);
  Episode.restoreCheckpoint(fresh, cp);
  assert.equal(fresh.hash(), hash, 'the checkpoint restores the settled state');

  // taking away a part that carries another wakes the sleeping pile and the other part falls
  for (let t = 0; t < 60 && fresh.d.ntree_awake > 1; t++) fresh.step(null);
  assert.ok(fresh.d.ntree_awake <= 1);
  const probe = Cell.create(mj, files, RAIN);
  Episode.restoreCheckpoint(probe, cp);
  contacts(probe);
  let carrier = -1, rider = -1, best = 0.01;
  const con = probe.d.contact;
  for (let c = 0; c < probe.d.ncon; c++) {
    const C = con.get(c), a = probe.bodyPart[probe.m.geom_bodyid[C.geom1]], b = probe.bodyPart[probe.m.geom_bodyid[C.geom2]];
    C.delete();
    if (a < 0 || b < 0) continue;
    const [lo, hi] = probe.partZ(a) < probe.partZ(b) ? [a, b] : [b, a];
    if (probe.partZ(hi) - probe.partZ(lo) > best) { best = probe.partZ(hi) - probe.partZ(lo); carrier = lo; rider = hi; }
  }
  con.delete();
  probe.dispose();
  assert.ok(carrier >= 0, 'a part lies on another');
  const z0 = fresh.partZ(rider);
  fresh.park(carrier);
  for (let t = 0; t < 20; t++) fresh.step(null);
  assert.ok(fresh.partZ(rider) < z0 - 0.002, `part ${rider} falls when ${carrier} is taken away (${(1000 * (fresh.partZ(rider) - z0)).toFixed(1)} mm)`);
  fresh.dispose();

  // the verifier's unit-0 replay reruns the fill to the same state
  const r = replayUnit(Cell.create(mj, files, RAIN), { index: 0, segments: [seg], checkObs: false });
  assert.equal(r.error, null);
  assert.deepEqual(r.checkpoint, cp);
  cell.dispose();
});

test('bury schedule: ring_n and cap are checked', () => {
  const M = RAIN.meta, S = M.fill.spawns, keep = S.slice(0, -6), parts = S.slice(-6).map((s) => s.part);
  const B = { parts, quats: S.slice(-6).map((s) => s.quat), ring: 0.75, size: 0.8, wall: 0.02, tries: 2, salt: 7 };
  const plan = (b) => () => fillPlan({ ...M, fill: { ...M.fill, spawns: keep, bury: b } });
  assert.equal(plan({ ...B, ring_n: 3, cap: 0.5 })().bury.ring_n, 3);
  assert.throws(plan({ ...B, ring_n: 6 }), /malformed/);
  assert.throws(plan({ ...B, ring_n: 1.5 }), /malformed/);
  assert.throws(plan({ ...B, cap: -1 }), /malformed/);
});

test('bury fill: one part on the target, a ring, a second layer; deterministic, everything stays in the bin', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const S = RAIN.meta.fill.spawns, parts = S.slice(-6).map((s) => s.part);
  const sc = JSON.parse(JSON.stringify(RAIN));
  sc.meta.fill.spawns = S.slice(0, -6);
  sc.meta.fill.bury = { parts, quats: S.slice(-6).map((s) => s.quat), ring_n: 3, cap: 0.5, ring: 0.75, size: 2, wall: 0.02,
                        tries: 2, salt: 7 };
  const { files } = bundleFiles(ASSETS, sc);
  const run = () => { const c = Cell.create(mj, files, sc), ep = fill(c); return { c, ep }; };
  const a = run(), b = run();
  assert.equal(a.ep.segments[0].hash, b.ep.segments[0].hash, 'the bury is deterministic');
  const f = a.ep.fill;
  assert.ok(f.buryLog && f.buryLog.length >= 1, 'the burying batch was dropped');
  assert.ok(parts.every((i) => a.c.active[i] && a.c.inside(i)), 'every bury part ends in the bin');
  a.c.dispose(); b.c.dispose();
});
