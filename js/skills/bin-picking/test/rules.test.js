import test from 'node:test';
import assert from 'node:assert/strict';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, Camera, observe, obsWire, b64decode, TryMonitor, pickTarget, classify, TARGET_CLASSES, PickRun, replayPicks,
         pickSeed, pickCount } from '../src/index.js';
import { compose, BUNDLE, SKIP, SETTINGS, recordedPick, highestPart } from './helpers.js';

const O = { target: 0, liftZ: 0.3, holdS: 0.5, liftDz: 0.03, tryS: 3, endOnMiss: false, settleSpeed: 0.02, outSettleS: 0.2,
            lostS: 0.2, mode: 'pile', crateHitForce: 75, targetClass: 'buried' };
const N = 4, DT = 0.05;

function st(t, f = {}) {
  const s = { t, held: new Uint8Array(N), touch: new Uint8Array(N), bottomZ: new Float64Array(N).fill(0.05),
              z: new Float64Array(N).fill(0.08), speed: new Float64Array(N), inside: new Uint8Array(N).fill(1),
              floor: new Uint8Array(N), active: new Uint8Array(N).fill(1), emptyClose: false, giveUp: false,
              tcp: [0.5, 0, 0.6], q: new Float64Array(6), crateF: 0, carried: new Uint8Array(N) };
  for (const [k, v] of Object.entries(f)) {
    if (s[k] && ArrayBuffer.isView(s[k]) && v && typeof v === 'object' && !ArrayBuffer.isView(v)) for (const [i, x] of Object.entries(v)) s[k][i] = x;
    else s[k] = v;
  }
  return s;
}

// Steps a monitor through ticks 1..: each tick's state from f(k, t); returns the verdict and its tick.
function run(f, o = {}) {
  const mon = new TryMonitor(st(0), { ...O, ...o });
  for (let k = 1; k < 1000; k++) {
    const t = Math.round(k * DT * 1e6) / 1e6, v = mon.update(st(t, f(k, t)));
    if (v) return { ...v, k, mon };
  }
  throw new Error('no verdict');
}

const lift = { held: { 0: 1 }, bottomZ: { 0: 0.31 }, z: { 0: 0.34 } };

test('pile rules: success needs the target held above the line, alone, for hold_s', () => {
  const v = run(() => lift);
  assert.equal(v.outcome, 'success');
  assert.equal(v.k, 11);
  assert.equal(v.part, 0);
  assert.deepEqual(Object.keys(v).sort(), ['crate_hits', 'detail', 'joint_path_rad', 'k', 'mon', 'out_parts', 'outcome', 'outs',
                                           'part', 't', 'target_class', 'tcp_path_m']);
  assert.equal(v.target_class, 'buried');
  assert.equal(v.outs, 0);
  const notTarget = run(() => ({ held: { 1: 1 }, bottomZ: { 1: 0.31 }, z: { 1: 0.34 } }));
  assert.equal(notTarget.outcome, 'timed_out', 'lifting another part is no verdict');
  assert.equal(notTarget.k, 60);
});

test('pile rules: two or more parts above the line at once end the pick as multi_pick; they leave with the gripper', () => {
  const two = run((k) => (k < 3 ? lift : { ...lift, bottomZ: { 0: 0.31, 2: 0.32 }, held: { 0: 1, 2: 1 } }));
  assert.equal(two.outcome, 'multi_pick');
  assert.equal(two.k, 3);
  assert.match(two.detail, /2 parts above the line \(0, 2\)/);
  assert.deepEqual(two.mon.toRemove(st(two.t, { bottomZ: { 0: 0.31, 2: 0.32 } }), two), [0, 2]);
  const others = run(() => ({ bottomZ: { 1: 0.31, 3: 0.4 } }));
  assert.equal(others.outcome, 'multi_pick', 'any two parts count, the target or not');
  assert.equal(others.k, 1);
  const outside = run(() => ({ ...lift, bottomZ: { 0: 0.31, 2: 0.32 } }), {});
  assert.equal(outside.outcome, 'multi_pick');
  const notOurs = new TryMonitor(st(0, { inside: { 2: 0 } }), O);
  let v = null;
  for (let k = 1; k < 20 && !v; k++) v = notOurs.update(st(k * DT, { ...lift, bottomZ: { 0: 0.31, 2: 0.4 } }));
  assert.equal(v.outcome, 'success', 'a part that was not in the bin when the pick began does not count');
});

test('pile rules: a dropped target is no verdict; at the end it is slipped, missed, knocked_out or timed_out', () => {
  const dropped = run((k) => (k < 5 ? lift : { z: { 0: 0.2 } }));
  assert.equal(dropped.outcome, 'slipped');
  assert.equal(dropped.k, 60, 'only at the time limit');
  const retry = run((k) => (k < 5 || k > 30 ? lift : {}));
  assert.equal(retry.outcome, 'success', 'the target can be picked again');
  const regrab = run((k) => (k < 5 || k > 50 ? { ...lift, bottomZ: { 0: 0.2 } } : {}));
  assert.equal(regrab.outcome, 'timed_out', 'lifted, held at the end, not above the line');
  assert.equal(run(() => ({ emptyClose: true })).outcome, 'missed');
  assert.equal(run((k) => ({ emptyClose: true, held: k === 3 ? { 1: 1 } : {} })).outcome, 'timed_out');
  const gave = run((k) => ({ giveUp: k === 7 }));
  assert.deepEqual([gave.outcome, gave.k], ['timed_out', 7]);
  const out = run((k) => (k > 3 ? { inside: { 0: 0 } } : {}));
  assert.equal(out.outcome, 'knocked_out');
  assert.deepEqual([out.outs, out.out_parts], [1, [0]]);
  const gaveOut = run((k) => ({ giveUp: k === 2, inside: { 0: 0 }, speed: { 0: 1 } }));
  assert.deepEqual([gaveOut.outcome, gaveOut.k, gaveOut.out_parts], ['knocked_out', 2, [0]], 'outside and not held at the end');
});

test('pile rules: outs are parts at rest outside the bin (or on the floor), not carried; they never end the try', () => {
  const v = run((k) => (k > 2 ? { inside: { 2: 0, 3: 0 }, speed: { 3: k < 40 ? 1 : 0 } } : {}));
  assert.equal(v.outcome, 'timed_out');
  assert.deepEqual([v.outs, v.out_parts], [2, [2, 3]]);
  const mon = new TryMonitor(st(0), O);
  for (let k = 1; k <= 20; k++) mon.update(st(k * DT, { inside: { 1: 0 }, carried: { 1: 1 }, touch: { 1: 1 } }));
  assert.equal(mon.metrics().outs, 0, 'a part on the gripper outside the bin is not out');
  for (let k = 21; k <= 23; k++) mon.update(st(k * DT, { inside: { 1: 0 } }));
  assert.equal(mon.metrics().outs, 0, 'released: not out before it rests out_settle_s');
  mon.update(st(24 * DT, { inside: { 1: 0 } }));
  mon.update(st(25 * DT, { inside: { 1: 0 } }));
  assert.equal(mon.metrics().outs, 1);
  mon.update(st(26 * DT, {}));
  assert.equal(mon.metrics().outs, 1, 'an out stays an out');
  const fl = new TryMonitor(st(0), O);
  fl.update(st(DT, { inside: { 3: 0 }, floor: { 3: 1 }, speed: { 3: 2 } }));
  assert.deepEqual(fl.metrics().out_parts, [3], 'on the floor: out at once');
  const s = run((k) => ({ ...lift, inside: { 3: k > 8 ? 0 : 1 }, speed: { 3: 1 } }));
  assert.deepEqual([s.outcome, s.out_parts], ['success', [3]], 'at success, every non-held part outside counts');
  const held = run(() => ({ ...lift, inside: { 0: 0 } }));
  assert.deepEqual([held.outcome, held.outs], ['success', 0], 'the held target outside the footprint is not out');
});

test('pile rules: path, joint path and crate hits', () => {
  const v = run((k) => ({ tcp: [0.5 + 0.01 * k, 0, 0.6], q: Float64Array.of(0.02 * k, -0.01 * k, 0, 0, 0, 0), crateF: k % 2 ? 80 : 74,
                         giveUp: k === 10 }));
  assert.equal(v.tcp_path_m, 0.1);
  assert.equal(v.joint_path_rad, 0.3);
  assert.equal(v.crate_hits, 5);
});

test('old scenes: the monitor keeps its verdicts (wrong_part, mid-try knocked_out) and shape', () => {
  const o = { ...O, mode: 'classic', targetClass: null };
  const w = run(() => ({ held: { 1: 1 }, bottomZ: { 1: 0.31 }, z: { 1: 0.34 } }), o);
  assert.deepEqual([w.outcome, w.k, w.part], ['wrong_part', 11, 1]);
  const k = run((j) => (j > 3 ? { inside: { 2: 0 } } : {}), o);
  assert.deepEqual([k.outcome, k.part], ['knocked_out', 2]);
  for (const x of [w, k]) {
    const { mon, k: _, ...rest } = x;
    assert.deepEqual(Object.keys(rest).sort(), ['detail', 'outcome', 'part', 't']);
  }
  assert.equal(k.mon.metrics().outs, 1);
});

const mj = await mujoco();
const scene = compose(SETTINGS, 21);
const { files } = SKIP ? { files: null } : bundleFiles(BUNDLE, scene);
const pileScene = () => {
  const sc = JSON.parse(JSON.stringify(scene));
  sc.meta.config.rules.mode = 'pile';
  return sc;
};

function settled(sc) {
  const cell = Cell.create(mj, files, sc), ep = new Episode(cell);
  ep.beginFill();
  while (!ep.fillStep());
  ep.sealSetup();
  return { cell, ep, checkpoint: ep.checkpoint() };
}

function pick(cell, ep, target, targetClass = null) {
  ep.beginTry(target, targetClass);
  let v = null;
  const rows = recordedPick(cell, target);
  for (const r of rows) { v = ep.stepTry(r); if (v) break; }
  while (!v) v = ep.stepTry(rows[rows.length - 1]);
  return { ...ep.lastTry.verdict, hash: cell.hash() };
}

test('a pile-mode scene: same physics, verdict with metrics, uint16 seg', { skip: SKIP }, () => {
  const a = settled(scene), b = settled(pileScene());
  assert.equal(a.cell.hash(), b.cell.hash(), 'the rules mode does not change the physics');
  const target = highestPart(a.cell);
  const va = pick(a.cell, a.ep, target), vb = pick(b.cell, b.ep, target, 'top');
  assert.equal(va.outcome, 'success');
  assert.deepEqual(Object.keys(va).sort(), ['detail', 'hash', 'outcome', 'part', 't', 'tick']);
  assert.equal(vb.outcome, 'success');
  assert.equal(vb.tick, va.tick);
  assert.equal(vb.hash, va.hash);
  assert.equal(vb.target_class, 'top');
  assert.ok(vb.tcp_path_m > 0.1 && vb.joint_path_rad > 0.5, JSON.stringify(vb));
  assert.equal(vb.crate_hits, 0);
  assert.equal(vb.outs, 0);
  assert.deepEqual(a.ep.lastTry.mon.metrics(), { ...b.ep.lastTry.mon.metrics(), target_class: null }, 'old scenes get the same metrics');

  Episode.restoreCheckpoint(a.cell, a.checkpoint);
  Episode.restoreCheckpoint(b.cell, b.checkpoint);
  const ca = new Camera(a.cell), cb = new Camera(b.cell);
  const oa = observe(a.cell, ca), ob = observe(b.cell, cb);
  assert.ok(oa.seg instanceof Uint8Array && !('seg_dtype' in oa));
  assert.ok(ob.seg instanceof Uint16Array);
  assert.equal(ob.seg_dtype, 'uint16');
  const map = (x) => (x === 254 ? 65534 : x === 255 ? 65535 : x);
  assert.deepEqual(Array.from(ob.seg), Array.from(oa.seg, map), 'same labels, crate 65534, robot 65535');
  assert.ok(ob.seg.includes(65534));
  const w = obsWire(ob, b64encodeNode);
  assert.equal(w.seg_dtype, 'uint16');
  const raw = b64decode(w.seg);
  assert.equal(raw.length, 2 * ob.seg.length);
  assert.deepEqual(Array.from(new Uint16Array(raw.buffer, raw.byteOffset, ob.seg.length)), Array.from(ob.seg), 'little-endian');
  assert.ok(!('seg_dtype' in obsWire(oa, b64encodeNode)));
  ca.dispose(); cb.dispose();

  const h0 = b.cell.hash(), C = classify(b.cell);
  assert.equal(b.cell.hash(), h0, 'classify leaves the state');
  for (const k of C.cls) assert.ok(k === null || TARGET_CLASSES.includes(k));
  const p = pickTarget(b.cell, 5);
  assert.ok(Number.isInteger(p.target) && TARGET_CLASSES.includes(p.target_class), JSON.stringify(p));
  assert.equal(C.cls[p.target], p.target_class);
  assert.deepEqual(pickTarget(b.cell, 5), p, 'seeded');
  const seen = new Set();
  for (let s = 0; s < 40; s++) seen.add(pickTarget(b.cell, s).target_class);
  for (const k of seen) assert.ok(C.counts[k] > 0);
  assert.deepEqual(pickTarget(a.cell, 5), { target: pickTarget(a.cell, 5).target, target_class: null, counts: null });
  a.cell.dispose(); b.cell.dispose();
});

function b64encodeNode(u8) { return Buffer.from(u8.buffer, u8.byteOffset, u8.byteLength).toString('base64'); }

test('a pile-mode scene: each finger is its pad chain up to the gripper base; older scenes keep the pads only', { skip: SKIP }, () => {
  const names = (cell, side) => [...side].flatMap((v, b) => v ? [mj.mj_id2name(cell.m, mj.mjtObj.mjOBJ_BODY.value, b)] : []).sort();
  const old = Cell.create(mj, files, scene), pile = Cell.create(mj, files, pileScene());
  assert.deepEqual(names(old, old.isLeft), ['g_left_pad', 'g_left_silicone_pad']);
  assert.deepEqual(names(pile, pile.isLeft), ['g_left_follower', 'g_left_pad', 'g_left_silicone_pad', 'g_left_spring_link']);
  assert.deepEqual(names(pile, pile.isRight), ['g_right_follower', 'g_right_pad', 'g_right_silicone_pad', 'g_right_spring_link']);
});

test('a pile-mode scene: a part carried out of the bin is no out until it is let go and rests', { skip: SKIP }, () => {
  const { cell, ep } = settled(pileScene());
  const P = highestPart(cell), other = cell.activeParts().find((i) => i !== P);
  ep.beginTry(other, 'top');
  const lz = ep.cur.mon.o.liftZ;
  let v = null, last = null;
  for (const r of recordedPick(cell, P)) {
    v = ep.stepTry(r);
    last = r;
    const s = cell.tick(0);
    if (v || (s.held[P] && s.bottomZ[P] > lz + 0.02)) break;
  }
  assert.equal(v, null, 'lifting another part is no verdict');
  const outs = () => ep.cur.mon.metrics().out_parts;
  for (let j = 1; j <= 30; j++) { const r = last.slice(); r[0] += 0.04 * j; assert.equal(ep.stepTry(r), null); if (j === 30) last = r; }
  for (let j = 0; j < 20; j++) assert.equal(ep.stepTry(last), null);
  const s = cell.tick(0);
  assert.deepEqual([s.inside[P], s.held[P], s.carried[P]], [0, 1, 1], 'held outside the bin');
  assert.deepEqual(outs(), []);
  const open = last.slice();
  open[6] = 0;
  for (let j = 0; j < 40; j++) assert.equal(ep.stepTry(open), null, 'an out does not end the try');
  assert.deepEqual(outs(), [P]);
  while (!v) v = ep.stepTry(open);
  assert.deepEqual([v.outcome, v.outs, v.out_parts, v.target_class], ['timed_out', 1, [P], 'top']);
  assert.ok(v.tcp_path_m > 1 && v.joint_path_rad > 2);
  cell.dispose();
});

test('pile rules: a simulation reset mid-pick ends it as physics_error', () => {
  const v = run((k) => ({ resets: k > 4 ? 1 : 0 }));
  assert.deepEqual([v.outcome, v.k], ['physics_error', 5]);
});

test('a pile scene of several picks: targets by (seed, k), failed targets skipped, the replay agrees', { skip: SKIP }, () => {
  const sc = pileScene();
  Object.assign(sc.meta.config.rules, { picks: 3, try_s: 1, target: { min_px: 10, buried_min_px: 5 } });
  const { cell, ep, checkpoint } = settled(sc);
  assert.equal(pickCount(cell), 3);
  assert.equal(pickSeed(7, 0), 7);
  const p0 = pickTarget(cell, 7), start = { target: p0.target, target_class: p0.target_class, seed: 7 };
  const runA = new PickRun(ep, start);
  runA.start();
  const hold = [...cell.R.q0, 0], rows = [];
  const ends = [];
  while (!runA.done) { rows.push(hold); const r = runA.step(hold); if (r) ends.push(r); }
  assert.equal(ends.length, 3);
  assert.deepEqual(runA.picks.map((p) => p.tick), [20, 20, 20]);
  const targets = runA.picks.map((p) => p.target);
  assert.equal(new Set(targets).size, 3, 'a failed target is not chosen again');
  assert.equal(ends[2].next, null);
  assert.deepEqual(runA.totals().picks_planned, 3);
  const hash = cell.hash();
  const fresh = Cell.create(mj, files, sc);
  const R = replayPicks(Episode.restoreCheckpoint(fresh, checkpoint), start, Float64Array.from(rows.flat()));
  assert.deepEqual(R.run.picks.map((p) => p.target), targets);
  assert.equal(R.hash, hash);
  assert.deepEqual([R.rows, R.extra], [60, 0]);
  const part = replayPicks(Episode.restoreCheckpoint(fresh, checkpoint), start, Float64Array.from(rows.slice(0, 25).flat()), { hold: true });
  assert.deepEqual([part.run.picks.length, part.held, part.run.done], [2, 15, false], 'hold ends with the current pick');
  cell.dispose(); fresh.dispose();
});
