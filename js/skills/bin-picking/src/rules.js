export const FAILURES = ['missed', 'slipped', 'knocked_out', 'timed_out', 'wrong_part', 'multi_pick'];
// pile scenes: a pick that ends in a diverged simulation (MuJoCo reset itself); no verdict on the expert
export const PHYSICS_ERROR = 'physics_error';
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
    this.pile = o.mode === 'pile';
    this.targetClass = o.targetClass ?? null;
    this.restSince = new Float64Array(n).fill(NaN);
    this.out = new Uint8Array(n);
    this.outParts = [];
    this.lastTcp = first.tcp ? first.tcp.slice() : null;
    this.lastQ = first.q ? Float64Array.from(first.q) : null;
    this.tcpPath = 0;
    this.jointPath = 0;
    this.crateHits = 0;
    this.resets0 = first.resets || 0;
  }

  update(s) {
    this._track(s);
    return this.pile ? this._pile(s) : this._classic(s);
  }

  // Per-try metrics and outs, in every scene. An out: a part that was in the bin at the start of the try, is outside
  // it now, not carried by the gripper (pile scenes: not touching it, directly or through other parts; old scenes: not
  // held), and either on the floor or slower than settle_speed for out_settle_s since it was last carried. Outs stay
  // outs.
  _track(s) {
    const o = this.o, t = s.t, n = s.z.length, carry = s.carried || s.held;
    if (s.tcp && this.lastTcp) {
      const a = this.lastTcp, b = s.tcp;
      this.tcpPath += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    }
    if (s.tcp) this.lastTcp = s.tcp.slice();
    if (s.q && this.lastQ) for (let k = 0; k < 6; k++) this.jointPath += Math.abs(s.q[k] - this.lastQ[k]);
    if (s.q) this.lastQ = Float64Array.from(s.q);
    if (s.crateF > o.crateHitForce) this.crateHits++;
    for (let i = 0; i < n; i++) {
      if (!s.active[i] || carry[i] || s.held[i] || s.speed[i] >= o.settleSpeed) { this.restSince[i] = NaN; continue; }
      if (Number.isNaN(this.restSince[i])) this.restSince[i] = t;
    }
    for (let i = 0; i < n; i++) {
      if (this.out[i] || !this.ours[i] || !s.active[i] || s.inside[i] || s.held[i]) continue;
      const rest = Number.isNaN(this.restSince[i]) ? 0 : t - this.restSince[i];
      if (s.floor[i] || (!carry[i] && rest >= o.outSettleS - 1e-9)) this._markOut(i);
    }
  }

  _markOut(i) {
    this.out[i] = 1;
    this.outParts.push(i);
  }

  metrics() {
    return { outs: this.outParts.length, out_parts: this.outParts.slice().sort((a, b) => a - b),
             tcp_path_m: r6(this.tcpPath), joint_path_rad: r6(this.jointPath), crate_hits: this.crateHits,
             target_class: this.targetClass };
  }

  // Pile scenes: the try ends on success, on multi_pick (two or more parts above the lift line at the same tick:
  // only one part may be picked at a time), at the time limit or on a give-up. Dropping the target, holding another
  // part or pushing parts around is no verdict; outs are counted. At the end every part outside the bin and not held
  // counts as an out too, then: success; knocked_out (the target is an out); slipped (the target was lifted by
  // lift_dz and is not held now); missed (the grip closed on nothing and nothing was ever held); timed_out.
  _pile(s) {
    const o = this.o, t = s.t, n = s.z.length, T = this.target;
    if ((s.resets || 0) !== this.resets0) return this._end(s, PHYSICS_ERROR, 'the simulation diverged and was reset');
    let nHeld = 0;
    for (let i = 0; i < n; i++) {
      if (!(s.held[i] && s.active[i])) continue;
      nHeld++;
      if (s.z[i] - this.z0[i] >= o.liftDz) this.lifted[i] = 1;
    }
    this.emptySeen ||= !!(s.emptyClose && nHeld === 0);
    this.heldSeen ||= nHeld > 0;
    const up = [];
    for (let i = 0; i < n; i++) if (this.ours[i] && s.active[i] && s.bottomZ[i] >= o.liftZ) up.push(i);
    if (up.length >= 2) {
      this.multiParts = up;
      return this._end(s, 'multi_pick', `${up.length} parts above the line (${up.join(', ')})`);
    }
    const alone = T !== null && T !== undefined && s.active[T] && s.held[T] && s.bottomZ[T] >= o.liftZ;
    if (alone) {
      if (this.okSince === null) this.okSince = t;
      if (t - this.okSince >= o.holdS - 1e-9) {
        const mm = Math.round(1000 * (s.bottomZ[T] - o.liftZ));
        return this._end(s, 'success', `held ${mm} mm above the line`);
      }
    } else this.okSince = null;
    if (s.giveUp || t >= o.tryS - 1e-9) {
      const why = s.giveUp ? 'gave up' : `${o.tryS} s`;
      for (let i = 0; i < n; i++) if (!this.out[i] && this.ours[i] && s.active[i] && !s.inside[i] && !s.held[i]) this._markOut(i);
      if (T !== null && T !== undefined && this.out[T]) return this._end(s, 'knocked_out', `the target left the bin, ${why}`);
      if (T !== null && T !== undefined && this.lifted[T] && !(s.held[T] && s.active[T]))
        return this._end(s, 'slipped', `the target was lifted and dropped, ${why}`);
      if (this.emptySeen && !this.heldSeen) return this._end(s, 'missed', `closed on nothing, ${why}`);
      return this._end(s, 'timed_out', why);
    }
    return null;
  }

  _end(s, outcome, detail) {
    if (outcome === 'success' || outcome === 'multi_pick') {
      for (let i = 0; i < s.z.length; i++) if (!this.out[i] && this.ours[i] && s.active[i] && !s.inside[i] && !s.held[i]) this._markOut(i);
    }
    const out = v(outcome, s.t, this.target, detail);
    if (this.outParts.length) out.detail += `; ${this.outParts.length} out of the bin`;
    return Object.assign(out, this.metrics());
  }

  _classic(s) {
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
    // pile scenes: the outs and a picked target go, and on multi_pick every part above the line (they leave with the
    // gripper); a failed target and a part still in the gripper stay
    if (this.pile) {
      const out = new Set(this.outParts);
      for (let i = 0; i < s.z.length; i++) if (s.active[i] && (!s.inside[i] || s.floor[i]) && !s.held[i]) out.add(i);
      if (verdict.outcome === 'success' && verdict.part !== null) out.add(verdict.part);
      if (verdict.outcome === 'multi_pick') for (const i of this.multiParts || []) out.add(i);
      return [...out].filter((i) => s.active[i]).sort((a, b) => a - b);
    }
    const out = new Set();
    for (let i = 0; i < s.z.length; i++) if (s.active[i] && (s.held[i] || !s.inside[i] || s.floor[i])) out.add(i);
    if ((verdict.outcome === 'success' || verdict.outcome === 'wrong_part') && verdict.part !== null) out.add(verdict.part);
    if (this.target !== null && this.target !== undefined && s.active[this.target]) out.add(this.target);
    return [...out].sort((a, b) => a - b);
  }
}

function r6(x) { return Math.round(x * 1e6) / 1e6; }

function v(outcome, t, part, detail) {
  return { outcome, t, part: part === undefined ? null : part, detail };
}
