import { Episode, LOG_FORMAT, CTRL } from './episode.js';
import { Camera, observe, obsHash } from './obs.js';
import { b64ToF64 } from './hash.js';

const MAX_EDITS = 500;

export function replayUnit(cell, { checkpoint = null, index, segments, chunks = null, createHash = null, checkObs = true } = {}) {
  if (checkObs && !createHash) throw new Error('checkObs needs createHash (e.g. () => crypto.createHash("sha256"))');
  let at = -1;
  const fail = (error) => ({ ok: false, error, at, checkpoint: null, try: null });
  if (!Number.isInteger(index) || index < 0) return fail('bad unit index');
  if (!Array.isArray(segments) || !segments.length) return fail('no segments');

  if (index === 0) {
    if (checkpoint) return fail('unit 0 starts from the scene, not a checkpoint');
    at = 0;
    const s = segments[0];
    if (s.kind !== 'fill') return fail('unit 0 starts with the fill');
    if (segments.length > 1 + MAX_EDITS) return fail('too many edits');
    const ep = new Episode(cell);
    ep.beginFill();
    let n = 0;
    while (!ep.fillStep()) if (++n > 100000) return fail('fill does not settle');
    const got = ep.segments[0];
    if (got.ticks !== s.ticks || got.hash !== s.hash) return fail(`fill differs (${got.ticks}/${got.hash} vs ${s.ticks}/${s.hash})`);
    if (!sameList(got.removed, s.removed)) return fail('fill take-away differs');
    for (let k = 1; k < segments.length; k++) {
      at = k;
      const e = segments[k];
      let g;
      try {
        if (e.kind === 'place') g = ep.placePart(e.part, e.pos, e.quat);
        else if (e.kind === 'remove') g = ep.removePart(e.part);
        else if (e.kind === 'robot') g = ep.setRobot(e.q);
        else return fail(`unexpected ${e.kind} segment in unit 0`);
      } catch (err) { return fail(`edit refused: ${err.message}`); }
      if (g.hash !== e.hash || (g.ticks ?? null) !== (e.ticks ?? null)) return fail(`${e.kind} edit differs`);
      if (g.removed && !sameList(g.removed, e.removed)) return fail(`${e.kind} edit take-away differs`);
    }
    ep.sealSetup();
    return { ok: true, error: null, at: null, checkpoint: ep.checkpoint(), try: null };
  }

  if (!checkpoint) return fail('a try unit needs the checkpoint before it');
  if (checkpoint.tries !== index - 1) return fail(`the checkpoint is not the one before unit ${index}`);
  const S = segments[0], L = segments[segments.length - 1];
  at = 0;
  if (S.kind !== 'try') return fail('a unit starts with its try');
  if (S.i !== index - 1) return fail(`unit ${index} holds try ${index - 1}, not ${S.i}`);
  if (segments.length !== 2 || L.kind !== 'take') return fail('a unit is its try and its take-away');

  const ep = new Episode(cell);
  try { ep.restoreCheckpoint(checkpoint); } catch (e) { return fail(`bad checkpoint: ${e.message}`); }
  const cam = checkObs ? new Camera(cell) : null;
  try {
    const i = S.i;
    if (!Number.isInteger(S.target) || S.target < 0 || S.target >= cell.n || !cell.active[S.target]) return fail('bad target');
    const target = S.target;

    const ctrl = decodeCtrl(S);
    if (!ctrl) return fail('ctrl size');
    const calls = [...(S.calls || [])].sort((a, b) => a.tick - b.tick);
    if (!calls.length || calls[0].tick !== 0) return fail('a try starts with a policy call');
    ep.beginTry(target);
    let ci = 0, cur = null;
    for (let j = 0; j < S.n; j++) {
      let giveUp = false;
      while (ci < calls.length && calls[ci].tick === j) {
        const c = calls[ci++];
        if (cam && obsHash(observe(cell, cam), createHash) !== c.obs) return fail(`observation of call ${c.id} differs at tick ${j}`);
        if (chunks) {
          const ch = chunks[c.id];
          if (!ch) return fail(`call ${c.id} was not issued`);
          cur = { start: j, rows: ch.ctrl || [], give_up: !!ch.give_up };
        }
        if (c.give_up) giveUp = true;
      }
      const row = ctrl.subarray(CTRL * j, CTRL * j + CTRL);
      if (chunks) {
        if (cur.give_up) {
          if (!giveUp) return fail('give-up not honoured');
          const hold = j > 0 ? ctrl.subarray(CTRL * (j - 1), CTRL * j) : cell.ctrl();
          for (let x = 0; x < CTRL; x++) if (hold[x] !== row[x]) return fail('a give-up tick must hold the last ctrl');
        } else {
          const want = cur.rows[j - cur.start];
          if (!want) return fail(`tick ${j} beyond the policy's chunk`);
          for (let x = 0; x < CTRL; x++) if (want[x] !== row[x]) return fail(`ctrl at tick ${j} is not the policy's`);
        }
      }
      const v = ep.stepTry(Array.from(row), giveUp);
      if (v && j !== S.n - 1) return fail(`the try ended at tick ${j + 1}, the log says ${S.n}`);
      if (!v && j === S.n - 1) return fail('the try did not end where the log says');
    }
    if (ci !== calls.length) return fail('calls after the end of the try');
    const T = ep.lastTry;
    if (!sameVerdict(T.verdict, S.verdict)) return fail(`verdict differs: ${T.verdict.outcome} vs ${S.verdict && S.verdict.outcome}`);
    if (T.seg.hash !== S.hash) return fail('state after the try differs');

    at = segments.length - 1;
    if (L.try !== i) return fail('take-away of another try');
    const { removed } = ep.endTry();
    if (!sameList(removed, L.removed)) return fail('take-away differs');
    if (ep.segments[ep.segments.length - 1].hash !== L.hash) return fail('state after the take-away differs');
    return { ok: true, error: null, at: null, checkpoint: ep.checkpoint(), try: { i, target, verdict: T.verdict } };
  } catch (e) {
    return fail(`replay error: ${e.message}`);
  } finally {
    if (cam) cam.dispose();
  }
}

export function replay(cell, log, { chunks = null, checkObs = true, createHash = null } = {}) {
  if (checkObs && !createHash) throw new Error('checkObs needs createHash (e.g. () => crypto.createHash("sha256"))');
  let tries = 0, successes = 0;
  const fail = (at, error) => ({ ok: false, error, at, tries, successes });
  if (!log || log.format !== LOG_FORMAT) return fail(-1, 'unknown log format');
  if (log.xml_sha256 && log.xml_sha256 !== cell.meta.xml_sha256) return fail(-1, 'the log is for another scene');
  const units = log.units || [];
  if (!units.length) return fail(-1, 'no fill');
  let cp = null;
  for (let k = 0; k < units.length; k++) {
    const r = replayUnit(cell, { checkpoint: cp, index: k, segments: units[k], chunks, createHash, checkObs });
    if (!r.ok) return fail(k, `unit ${k}${r.at !== null && r.at >= 0 ? ` segment ${r.at}` : ''}: ${r.error}`);
    cp = r.checkpoint;
    if (r.try) {
      tries++;
      if (r.try.verdict.outcome === 'success') successes++;
    }
  }
  return { ok: true, error: null, at: null, tries, successes, units: units.length, hash: cell.hash() };
}

function decodeCtrl(s) {
  if (!Number.isInteger(s.n) || s.n < 1 || typeof s.ctrl !== 'string') return null;
  const c = b64ToF64(s.ctrl);
  return c.length === CTRL * s.n ? c : null;
}

function sameVerdict(a, b) {
  return !!a && !!b && a.outcome === b.outcome && a.part === b.part && a.tick === b.tick;
}

function sameList(a, b) {
  return Array.isArray(a) && Array.isArray(b) && a.length === b.length && a.every((x, k) => x === b[k]);
}
