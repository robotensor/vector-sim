import test from 'node:test';
import assert from 'node:assert/strict';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, runPlan, RowsHash, rowsHash, encodeRows, decodeRows, chainSeed, chainNext, flatRows,
         probe, replayRows, yawCosSin, EMPTY_ROWS_HASH } from '../src/index.js';
import { compose, BUNDLE, SKIP, recordedPick, highestPart } from './helpers.js';

const ROWS = [[0.5, -1.25, 3.0, -0.0, 1e-300, 2.5e10, 1.0], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0]];

function bigFnv(rows) {
  let h = 0xcbf29ce484222325n;
  const v = new DataView(new ArrayBuffer(8));
  for (const r of rows) for (const x of r) {
    v.setFloat64(0, x === 0 ? 0 : x, true);
    for (let k = 0; k < 8; k++) h = ((h ^ BigInt(v.getUint8(k))) * 0x100000001b3n) & 0xffffffffffffffffn;
  }
  return h.toString(16).padStart(16, '0');
}

test('rows hash is the harness action hash', () => {
  assert.equal(rowsHash([]), EMPTY_ROWS_HASH);
  assert.equal(rowsHash(ROWS), '8110603038cb9e9a');
  let seed = 7;
  const rnd = () => { seed = (Math.imul(seed, 1103515245) + 12345) >>> 0; return seed / 4294967296; };
  for (let n = 0; n < 20; n++) {
    const rows = Array.from({ length: 1 + (n % 5) }, () => Array.from({ length: 7 }, () => (rnd() - 0.5) * 10 ** Math.floor(rnd() * 8 - 4)));
    assert.equal(rowsHash(rows), bigFnv(rows));
    const h = new RowsHash();
    for (const r of rows) h.rows(Float64Array.from(r));
    assert.equal(h.hex(), bigFnv(rows));
  }
});

test('chain links match the published vectors', () => {
  const seed = chainSeed('n0nce', 's1', 'scene-a');
  assert.equal(seed, 'a84b0246363373a6');
  assert.equal(chainNext(seed, 0, 0, 2, '0123456789abcdef', flatRows(ROWS)), '84e03fe56edce983');
  assert.notEqual(chainNext(seed, 1, 0, 2, '0123456789abcdef', flatRows(ROWS)), '84e03fe56edce983');
  assert.notEqual(chainSeed('n0nce', 's2', 'scene-a'), seed);
});

test('rows encode as float64 and refuse bad values', () => {
  const b = encodeRows(ROWS);
  assert.deepEqual(Array.from(decodeRows(b)), ROWS.flat().map((x) => (x === 0 ? 0 : x)));
  assert.throws(() => flatRows([[0, 0, 0, 0, 0, 0, NaN]]));
  assert.throws(() => flatRows([1, 2, 3]));
  assert.throws(() => decodeRows('AAAA'));
});

test('crate yaw comes from the quaternion with exact arithmetic', () => {
  assert.deepEqual(yawCosSin({ yaw: 0, quat: [1, 0, 0, 0] }), [1, 0]);
  const y = 0.7, [c, s] = yawCosSin({ yaw: y, quat: [Math.cos(y / 2), 0, 0, Math.sin(y / 2)] });
  assert.ok(Math.abs(c - Math.cos(y)) < 1e-15 && Math.abs(s - Math.sin(y)) < 1e-15);
  assert.deepEqual(yawCosSin({ yaw: 0 }), [1, 0]);
});

test('replayRows and the probe are deterministic', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const sc = compose();
  const files = bundleFiles(BUNDLE, sc).files;
  const cell = Cell.create(mj, files, sc);
  const ep = new Episode(cell);
  ep.beginFill();
  while (!ep.fillStep());
  ep.sealSetup();
  const checkpoint = ep.checkpoint(), h0 = cell.hash();
  const target = highestPart(cell);
  const rows = recordedPick(cell, target);
  while (rows.length < 320) rows.push(rows.at(-1).slice());
  const p1 = probe(cell, checkpoint), p2 = probe(cell, checkpoint);
  assert.deepEqual(p1, p2);
  assert.equal(p1.start, h0);
  assert.equal(cell.hash(), h0);
  const a = replayRows(cell, { checkpoint, target }, rows);
  const b = replayRows(cell, { checkpoint, target }, encodeRows(rows));
  assert.deepEqual(a, b);
  assert.ok(a.outcome);
  assert.equal(a.rows + a.extra, rows.length);
  assert.equal(a.marks.length, a.rows);
  assert.equal(a.marks.at(-1)[1], a.hash);
  const plan = runPlan(cell, { checkpoint, target }, rows.slice(0, a.rows));
  assert.equal(plan.outcome, a.outcome);
  assert.equal(plan.hash, a.hash);
  const cut = replayRows(cell, { checkpoint, target }, rows.slice(0, 10));
  assert.equal(cut.outcome, null);
  assert.deepEqual(cut.marks, a.marks.slice(0, 10));
  assert.throws(() => replayRows(cell, { checkpoint, target }, rows, { limit: 5 }));
  cell.dispose();
});
