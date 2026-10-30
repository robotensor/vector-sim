export class IK {
  constructor(cell) {
    const mj = cell.mj;
    this.cell = cell;
    this.mj = mj;
    this.d = new mj.MjData(cell.m);
    this.nv = cell.meta.nv;
    this.jacp = new mj.DoubleBuffer(3 * this.nv);
    this.jacr = new mj.DoubleBuffer(3 * this.nv);
    this.qadr = cell.R.arm_joints_qpos;
    this.dofs = cell.R.arm_dofs;
    this.range = cell.R.joint_range;
  }

  dispose() { this.jacp.delete(); this.jacr.delete(); this.d.delete(); }

  fk(q) {
    const { mj, d } = this, m = this.cell.m, s = this.cell.R.tcp_site;
    this.qadr.forEach((a, k) => { d.qpos[a] = q[k]; });
    mj.mj_kinematics(m, d);
    return { pos: Array.from(d.site_xpos.subarray(3 * s, 3 * s + 3)), mat: Array.from(d.site_xmat.subarray(9 * s, 9 * s + 9)) };
  }

  solve(q0, pos, mat, { iters = 30, damping = 0.02, maxStep = 0.3, rotWeight = 0.3, tol = 1e-4 } = {}) {
    const { mj, d } = this, m = this.cell.m, s = this.cell.R.tcp_site;
    const q = q0.slice();
    let ep = Infinity, er = Infinity;
    for (let it = 0; it < iters; it++) {
      this.qadr.forEach((a, k) => { d.qpos[a] = q[k]; });
      mj.mj_kinematics(m, d);
      mj.mj_comPos(m, d);
      const p = d.site_xpos.subarray(3 * s, 3 * s + 3), R = d.site_xmat.subarray(9 * s, 9 * s + 9);
      const e = [pos[0] - p[0], pos[1] - p[1], pos[2] - p[2], 0, 0, 0];
      for (let i = 0; i < 3; i++) {
        const a = [R[i], R[3 + i], R[6 + i]], b = [mat[i], mat[3 + i], mat[6 + i]];
        e[3] += 0.5 * (a[1] * b[2] - a[2] * b[1]);
        e[4] += 0.5 * (a[2] * b[0] - a[0] * b[2]);
        e[5] += 0.5 * (a[0] * b[1] - a[1] * b[0]);
      }
      ep = Math.hypot(e[0], e[1], e[2]);
      er = Math.hypot(e[3], e[4], e[5]);
      if (ep < tol && er < 10 * tol) break;
      for (let k = 3; k < 6; k++) e[k] *= rotWeight;
      mj.mj_jacSite(m, d, this.jacp, this.jacr, s);
      const jp = this.jacp.GetView(), jr = this.jacr.GetView();
      const J = [];
      for (let r = 0; r < 3; r++) J.push(this.dofs.map((c) => jp[r * this.nv + c]));
      for (let r = 0; r < 3; r++) J.push(this.dofs.map((c) => rotWeight * jr[r * this.nv + c]));
      const A = J.map((ri, i) => J.map((rj, j) => ri.reduce((acc, x, k) => acc + x * rj[k], 0) + (i === j ? damping * damping : 0)));
      const y = solve6(A, e);
      let dq = this.dofs.map((_, k) => J.reduce((acc, row, i) => acc + row[k] * y[i], 0));
      const big = Math.max(...dq.map(Math.abs));
      if (big > maxStep) dq = dq.map((x) => (x * maxStep) / big);
      for (let k = 0; k < 6; k++) {
        const [lo, hi] = this.range[k];
        q[k] = Math.min(hi, Math.max(lo, q[k] + dq[k]));
      }
    }
    return { q, err: ep, rot: er };
  }
}

function solve6(A, b) {
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const piv = M[c][c] || 1e-12;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / piv;
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((r, i) => r[n] / (r[i] || 1e-12));
}

export function downMat(yaw = 0) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [c, s, 0, s, -c, 0, 0, 0, -1];
}
