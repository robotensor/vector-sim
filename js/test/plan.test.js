import test from 'node:test';
import assert from 'node:assert/strict';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, runPlan, planHash, planRows, chooseTarget, maxRows } from '../src/index.js';
import { compose, BUNDLE, SKIP, SETTINGS, recordedPick } from './helpers.js';

const mj = await mujoco();
const scene = compose(SETTINGS, 11);
const { files } = SKIP ? { files: null } : bundleFiles(BUNDLE, scene);

function settled() {
  const cell = Cell.create(mj, files, scene), ep = new Episode(cell);
  ep.beginFill();
  while (!ep.fillStep());
  ep.sealSetup();
  return { cell, ep, checkpoint: ep.checkpoint(), hash: cell.hash() };
}

test('a plan replays the same in a fresh cell, and any change shows', { skip: SKIP }, () => {
  const { cell, checkpoint, hash } = settled();
  const target = chooseTarget(cell, 5);
  assert.ok(Number.isInteger(target));
  assert.equal(chooseTarget(cell, 5), target, 'target choice is reproducible');
  const start = { checkpoint, target };
  Episode.restoreCheckpoint(cell, checkpoint);
  assert.equal(cell.hash(), hash, 'restoring the checkpoint gives the settled state');
  const rows = recordedPick(cell, target);
  const a = runPlan(cell, start, rows);
  cell.dispose();

  const fresh = Cell.create(mj, files, scene);
  const b = runPlan(fresh, start, rows);
  assert.deepEqual({ ...b, ctrl: null }, { ...a, ctrl: null });
  assert.equal(b.ctrl, a.ctrl);
  assert.equal(planHash(rows), planHash(planRows(a.ctrl).subarray(0, 7 * rows.length)));
  const bent = rows.map((r) => r.slice());
  bent[Math.floor(bent.length / 2)][0] += 1e-3;
  assert.notEqual(runPlan(fresh, start, bent).hash, a.hash);
  assert.throws(() => runPlan(fresh, start, 'AAAA'));
  assert.throws(() => runPlan(fresh, start, Array.from({ length: maxRows(fresh) + 1 }, () => rows[0])));
  fresh.dispose();
});

test('a short plan holds its last row until the rules decide', { skip: SKIP }, () => {
  const { cell, checkpoint } = settled();
  const r = runPlan(cell, { checkpoint, target: chooseTarget(cell, 1) }, []);
  assert.ok(['missed', 'timed_out'].includes(r.outcome), r.outcome);
  assert.equal(r.tick, maxRows(cell));
  assert.ok(r.held > 0);
  cell.dispose();
});
