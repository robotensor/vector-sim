export const FAILURES = ['missed', 'slipped', 'knocked_out', 'timed_out', 'wrong_part'];
export const OUTCOMES = ['success', ...FAILURES];

export class TryMonitor {
  constructor(first, o) {
    this.o = o;
    this.target = o.target;
    const n = first.z.length;
    this.ours = new Uint8Array(n);
    for (let i = 0; i < n; i++) this.ours[i] = first.active[i] && first.inside[i] ? 1 : 0;
    this.z0 = Float64Array.from(first.z);
    this.lifted = new Uint8Array(n);
    this.lastTouch = new Float64Array(n).fill(-Infinity);
    this.slowSince = new Float64Array(n).fill(NaN);
    this.lostSince = null;
    this.okSince = null;
    this.held = null;
    this.emptySeen = false;
    this.heldSeen = false;
  }

  update(s) {
    const o = this.o, t = s.t, n = s.z.length;
    let nHeld = 0, top = -1;
    for (let i = 0; i < n; i++) {
      const h = s.held[i] && s.active[i];
      if (h) {
        nHeld++;
        if (top < 0 || s.z[i] > s.z[top]) top = i;
        if (s.z[i] - this.z0[i] >= o.liftDz) this.lifted[i] = 1;
      }
      if (s.touch[i] && s.active[i]) this.lastTouch[i] = t;
      if (s.speed[i] < o.settleSpeed) { if (Number.isNaN(this.slowSince[i])) this.slowSince[i] = t; }
      else this.slowSince[i] = NaN;
    }
    this.emptySeen ||= !!(s.emptyClose && nHeld === 0);
    this.heldSeen ||= nHeld > 0;
    if (top >= 0) this.held = top;
    for (let i = 0; i < n; i++) {
      if (!(this.ours[i] && s.active[i] && !s.inside[i] && !(s.held[i]))) continue;
      const rest = Number.isNaN(this.slowSince[i]) ? 0 : t - this.slowSince[i];
      if (rest >= o.outSettleS || s.floor[i]) {
        if (this.lifted[i]) return v('slipped', t, i, `part ${i} fell out of the gripper outside the bin`);
        return v('knocked_out', t, i, `part ${i} left the bin`);
      }
    }
    const h = this.held;
    if (h !== null && this.lifted[h] && !(s.held[h] && s.active[h])) {
      if (this.lostSince === null) this.lostSince = t;
      if (t - this.lostSince >= o.lostS - 1e-9) return v('slipped', t, h, `part ${h} slipped after the lift`);
    } else this.lostSince = null;
    let above = 0, heldAbove = -1;
    for (let i = 0; i < n; i++) {
      if (this.ours[i] && s.active[i] && s.bottomZ[i] >= o.liftZ) {
        above++;
        if (s.held[i]) heldAbove = i;
      }
    }
    if (nHeld === 1 && above === 1 && heldAbove >= 0) {
      if (this.okSince === null) this.okSince = t;
      if (t - this.okSince >= o.holdS - 1e-9) {
        const mm = Math.round(1000 * (s.bottomZ[heldAbove] - o.liftZ));
        if (this.target !== null && this.target !== undefined && heldAbove !== this.target)
          return v('wrong_part', t, heldAbove, `picked part ${heldAbove}, not the target ${this.target}`);
        return v('success', t, heldAbove, `held ${mm} mm above the line`);
      }
    } else this.okSince = null;
    if (s.giveUp || (o.endOnMiss && s.emptyClose && nHeld === 0))
      return v('missed', t, this.guess(s), s.giveUp ? 'gave up' : 'closed on nothing');
    if (t >= o.tryS - 1e-9) {
      if (this.emptySeen && !this.heldSeen) return v('missed', t, this.guess(s), `closed on nothing, ${o.tryS} s`);
      return v('timed_out', t, this.guess(s), `${o.tryS} s`);
    }
    return null;
  }

  guess(s) {
    if (this.target !== null && this.target !== undefined) return this.target;
    if (this.held !== null) return this.held;
    let best = -1, bt = -Infinity;
    for (let i = 0; i < s.z.length; i++) if (s.active[i] && this.lastTouch[i] > bt) { bt = this.lastTouch[i]; best = i; }
    return best >= 0 ? best : null;
  }

  toRemove(s, verdict) {
    const out = new Set();
    for (let i = 0; i < s.z.length; i++) if (s.active[i] && (s.held[i] || !s.inside[i] || s.floor[i])) out.add(i);
    if ((verdict.outcome === 'success' || verdict.outcome === 'wrong_part') && verdict.part !== null) out.add(verdict.part);
    if (this.target !== null && this.target !== undefined && s.active[this.target]) out.add(this.target);
    return [...out].sort((a, b) => a - b);
  }
}

function v(outcome, t, part, detail) {
  return { outcome, t, part: part === undefined ? null : part, detail };
}
