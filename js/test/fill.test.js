import test from 'node:test';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, Camera, observe, obsHash, replay, replayUnit } from '../src/index.js';
import { compose, BUNDLE, SKIP, SETTINGS, recordedPick, highestPart } from './helpers.js';

const createHash = () => crypto.createHash('sha256');

test('placed flat, a few partly over others, no interpenetration, still after the fill, one try replays', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const scene = compose(SETTINGS, 21);
  const M = scene.meta, F = M.fill;
  assert.equal(M.crate.kind, 'bin');
  assert.ok(Number.isInteger(M.crate.max_parts) && M.crate.max_parts >= M.parts.length);
  assert.equal(F.mode, 'place');
  assert.equal(F.spawns.length, M.parts.length);
  assert.ok(!('cell' in M) && !/table_net/.test(scene.xml), 'no net');
  const C = M.config.fill, top = F.spawns.filter((s) => s.layer === 2);
  assert.ok(F.spawns.every((s) => s.layer === 1 || s.layer === 2));
  assert.ok(top.length >= 1 && top.length === F.leaning, 'overlap present');
  assert.ok(top.length <= Math.floor(C.overlap_share * M.parts.length) && C.max_layers === 2);
  const { files } = bundleFiles(BUNDLE, scene);

  const probe = Cell.create(mj, files, scene);
  for (const s of F.spawns) probe.spawn(s.part, s.pos, s.quat);
  const con = probe.d.contact;
  for (let k = 0; k < probe.d.ncon; k++) {
    const c = con.get(k), b1 = probe.m.geom_bodyid[c.geom1], b2 = probe.m.geom_bodyid[c.geom2];
    c.delete();
    assert.ok(!(probe.bodyPart[b1] >= 0 && probe.bodyPart[b2] >= 0), 'parts interpenetrate');
  }
  con.delete();
  const xy = (i) => { const b = 3 * M.parts[i].body; return [probe.d.xpos[b], probe.d.xpos[b + 1]]; };
  for (const s of F.spawns) {
    assert.ok(probe.inside(s.part), `part ${s.part} placed outside the walls`);
    const lo = probe.bottomZ(s.part);
    if (s.layer === 1) assert.ok(lo > -0.003 && lo < 0.01, `part ${s.part} lowest point ${lo}`);
    else {
      assert.ok(lo > 0.005, `top part ${s.part} rests on another (lowest point ${lo})`);
      const [x, y] = xy(s.part);
      assert.ok(F.spawns.some((o) => o.layer === 1 && Math.hypot(x - xy(o.part)[0], y - xy(o.part)[1])
        < M.parts[s.part].radius + M.parts[o.part].radius), `top part ${s.part} over a floor part`);
    }
  }
  probe.dispose();

  const cell = Cell.create(mj, files, scene);
  const ep = new Episode(cell), cam = new Camera(cell);
  ep.beginFill();
  let vfill = 0;
  while (!ep.fillStep()) vfill = Math.max(vfill, cell.maxPartSpeed());
  const fill = ep.segments[0];
  assert.deepEqual(fill.removed, []);
  assert.ok(fill.ticks <= F.settle_max_ticks);
  assert.ok(vfill < 0.3, `the parts barely move while they settle (max ${vfill.toFixed(3)} m/s)`);
  for (const s of top) assert.ok(cell.bottomZ(s.part) > 0.005, `top part ${s.part} still lies over another after the fill`);
  const act = cell.activeParts(), b = (i) => 3 * M.parts[i].body;
  const p0 = act.map((i) => [...cell.d.xpos.subarray(b(i), b(i) + 3)]);
  const snap = cell.snapshot();
  let vmax = 0, vtop = 0;
  const speed = (i) => { const q = M.parts[i].dof, v = cell.d.qvel; return Math.hypot(v[q], v[q + 1], v[q + 2]); };
  for (let k = 0; k < 80; k++) {
    cell.step(null);
    vmax = Math.max(vmax, cell.maxPartSpeed());
    for (const s of top) vtop = Math.max(vtop, speed(s.part));
  }
  const drift = Math.max(...act.map((i, j) => Math.hypot(...[0, 1, 2].map((a) => cell.d.xpos[b(i) + a] - p0[j][a]))));
  console.log(`fill ${fill.ticks} ticks, max ${vfill.toFixed(3)} m/s; then 4 s: max speed ${(1000 * vmax).toFixed(2)} mm/s ` +
    `(top parts ${(1000 * vtop).toFixed(2)}), drift ${(1000 * drift).toFixed(2)} mm`);
  assert.ok(vmax < 0.005 && vtop < 0.002 && drift < 0.002, 'the settled parts stay still');
  cell.restore(snap);

  ep.sealSetup();
  const cp0 = ep.checkpoint();
  const target = highestPart(cell);
  assert.ok(target >= 0);
  ep.beginTry(target);
  const chunks = {};
  let v = null;
  for (let n = 0; !v; n++) {
    assert.ok(n < 20, 'a verdict');
    const rows = recordedPick(cell, target);
    chunks[`c${n}`] = { ctrl: rows };
    ep.noteCall(`c${n}`, obsHash(observe(cell, cam), createHash));
    for (const r of rows) { v = ep.stepTry(r); if (v) break; }
  }
  console.log('try:', v.outcome, v.detail);
  ep.endTry();
  const log = ep.log('fill'), hash = cell.hash(), cp1 = ep.checkpoint();
  cam.dispose(); cell.dispose();

  let cp = null;
  for (const [k, want] of [[0, cp0], [1, cp1]]) {
    const fresh = Cell.create(mj, files, scene);
    const r = replayUnit(fresh, { checkpoint: cp, index: k, segments: log.units[k], chunks, createHash });
    fresh.dispose();
    assert.equal(r.error, null, `unit ${k}`);
    assert.deepEqual(r.checkpoint, want, `checkpoint after unit ${k}`);
    cp = r.checkpoint;
  }
  const whole = Cell.create(mj, files, scene);
  const full = replay(whole, log, { chunks, createHash });
  whole.dispose();
  assert.equal(full.error, null);
  assert.equal(full.hash, hash);
  assert.equal(full.tries, 1);
});
