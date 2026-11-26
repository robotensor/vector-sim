import test from 'node:test';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, Camera, observe, obsHash, replay, replayUnit } from '../src/index.js';
import { compose, BUNDLE, SKIP, SETTINGS, recordedPick, highestPart } from './helpers.js';

const createHash = () => crypto.createHash('sha256');
const FAIL_DEPTH = -0.06;

function runRecorded(ep, cell, target, depth, onChunk = null) {
  let v = null;
  for (let n = 0; !v; n++) {
    if (n > 20) throw new Error('no verdict');
    const rows = recordedPick(cell, target, { depth });
    if (onChunk) onChunk(rows);
    for (const r of rows) { v = ep.stepTry(r); if (v) break; }
  }
  return v;
}

function live(mj, files, scene, { nTries = 3 } = {}) {
  const cell = Cell.create(mj, files, scene);
  const ep = new Episode(cell), cam = new Camera(cell);
  ep.beginFill();
  while (!ep.fillStep());
  ep.sealSetup();
  const chunks = {}, checkpoints = [ep.checkpoint()];
  let id = 0;
  for (let t = 0; t < nTries; t++) {
    const target = highestPart(cell);
    if (target < 0) break;
    ep.beginTry(target);
    runRecorded(ep, cell, target, t === 1 ? FAIL_DEPTH : 0.0, (rows) => {
      const cid = `c${id++}`;
      chunks[cid] = { ctrl: rows };
      ep.noteCall(cid, obsHash(observe(cell, cam), createHash));
    });
    const u = ep.endTry();
    assert.equal(u.unit, t + 1);
    assert.deepEqual(u.segments, ep.unitSegments(u.unit));
    checkpoints.push(ep.checkpoint());
  }
  const out = { log: ep.log('test'), chunks, checkpoints, hash: cell.hash(), units: ep.units };
  cam.dispose(); cell.dispose();
  return out;
}

const kinds = (log) => log.units.map((u) => u.map((s) => s.kind + (s.verdict ? ':' + s.verdict.outcome : '')).join(' ')).join(' | ');

function verifyUnits(mj, files, scene, run) {
  let cp = null;
  const results = [];
  for (let k = 0; k < run.log.units.length; k++) {
    const cell = Cell.create(mj, files, scene);
    const r = replayUnit(cell, { checkpoint: cp, index: k, segments: run.log.units[k], chunks: run.chunks, createHash });
    cell.dispose();
    assert.equal(r.error, null, `unit ${k}`);
    assert.ok(r.ok);
    assert.deepEqual(r.checkpoint, run.checkpoints[k], `checkpoint after unit ${k}`);
    results.push(r);
    cp = r.checkpoint;
  }
  return results;
}

function unit(mj, files, scene, opts) {
  const cell = Cell.create(mj, files, scene);
  const r = replayUnit(cell, { createHash, ...opts });
  cell.dispose();
  return r;
}

test('units verify on their own and the whole log replays', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const scene = compose(SETTINGS, 5);
  const { files } = bundleFiles(BUNDLE, scene);
  assert.ok(files.size < 60, `${files.size} files`);

  const A = live(mj, files, scene);
  console.log('A:', kinds(A.log));
  assert.equal(A.units, 4);
  assert.ok(A.log.units.slice(1).every((u) => u.length === 2 && u[0].kind === 'try' && u[1].kind === 'take'));

  const ra = verifyUnits(mj, files, scene, A);
  assert.equal(ra[0].try, null);
  assert.ok(ra.slice(1).every((r, k) => r.try.i === k && !('case' in r)));
  assert.equal(ra[1].try.verdict.outcome, 'success');
  assert.notEqual(ra[2].try.verdict.outcome, 'success');

  const cell = Cell.create(mj, files, scene);
  const full = replay(cell, A.log, { chunks: A.chunks, createHash });
  cell.dispose();
  assert.equal(full.error, null);
  assert.equal(full.hash, A.hash);
  assert.equal(full.tries, 3);
  assert.ok(!('cases' in full));
  assert.equal(full.successes, ra.filter((r) => r.try && r.try.verdict.outcome === 'success').length);

  const forge = (seg) => { const b = Buffer.from(seg.ctrl, 'base64'); b.writeDoubleLE(b.readDoubleLE(8) + 1e-9, 8); seg.ctrl = b.toString('base64'); };
  const bad = structuredClone(A.log.units[1]);
  forge(bad[0]);
  const x1 = unit(mj, files, scene, { checkpoint: A.checkpoints[0], index: 1, segments: bad, chunks: A.chunks });
  assert.equal(x1.ok, false);
  console.log('forged try ctrl:', x1.error);
  const x2 = unit(mj, files, scene, { checkpoint: A.checkpoints[0], index: 2, segments: A.log.units[2], chunks: A.chunks });
  assert.equal(x2.ok, false);
  const T = A.log.units[2];
  const extra = [T[0], T[0], T[1]];
  const x3 = unit(mj, files, scene, { checkpoint: A.checkpoints[1], index: 2, segments: extra, chunks: A.chunks });
  assert.deepEqual([x3.ok, x3.error], [false, 'a unit is its try and its take-away']);
});
