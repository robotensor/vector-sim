import { observe, segBytes } from './obs.js';

export async function runTry(ep, cam, act, { onTick = null, pace = null, stop = null } = {}) {
  const cell = ep.cell;
  if (ep.phase === 'setup' && typeof ep.sealSetup === 'function') ep.sealSetup();
  let o = observe(cell, cam);
  let r = await act(o, 0);
  if (r.target === null || r.target === undefined) return null;
  ep.beginTry(r.target);
  let k = 0;
  for (;;) {
    ep.noteCall(r.call, r.obs_hash, !!r.give_up);
    if (r.give_up || !r.ctrl || !r.ctrl.length) {
      const v = ep.stepTry(cell.ctrl(), true);
      if (onTick) onTick(ep, v);
      return v;
    }
    for (const row of r.ctrl) {
      if (stop && stop()) throw new Error('stopped');
      const v = ep.stepTry(row);
      if (onTick) onTick(ep, v);
      if (pace) await pace();
      if (v) return v;
    }
    o = observe(cell, cam);
    r = await act(o, ++k);
  }
}

export function obsWire(o, b64) {
  const w = { width: o.width, height: o.height, q: o.q, grip: o.grip,
              depth: b64(new Uint8Array(o.depth.buffer, o.depth.byteOffset, o.depth.byteLength)), seg: b64(segBytes(o.seg)) };
  if (o.seg instanceof Uint16Array) w.seg_dtype = 'uint16';
  return w;
}
