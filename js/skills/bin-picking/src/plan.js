import { Episode, CTRL } from './episode.js';
import { Camera } from './obs.js';
import { b64ToF64, fnv64 } from './hash.js';

export function planRows(ctrl) {
  let flat;
  if (typeof ctrl === 'string') flat = b64ToF64(ctrl);
  else if (ctrl instanceof Float64Array) flat = ctrl;
  else if (Array.isArray(ctrl) && ctrl.length && Array.isArray(ctrl[0])) flat = Float64Array.from(ctrl.flat());
  else flat = Float64Array.from(ctrl || []);
  if (flat.length % CTRL) throw new Error(`plan length ${flat.length} is not a multiple of ${CTRL}`);
  for (let k = 0; k < flat.length; k++) if (!Number.isFinite(flat[k])) throw new Error('plan has a non-finite value');
  return flat;
}

export function maxRows(cell) {
  return Math.ceil(cell.cfg.rules.try_s / cell.meta.tick_s - 1e-9);
}

export function runPlan(cell, start, ctrl, { limit = null } = {}) {
  const flat = planRows(ctrl), n = flat.length / CTRL;
  const cap = limit ?? maxRows(cell);
  if (n > cap) throw new Error(`plan has ${n} rows, more than a try lasts (${cap})`);
  const ep = Episode.restoreCheckpoint(cell, start.checkpoint);
  ep.beginTry(start.target, start.target_class ?? null);
  let v = null, k = 0, row = null;
  for (; k < n && !v; k++) {
    row = Array.from(flat.subarray(CTRL * k, CTRL * (k + 1)));
    v = ep.stepTry(row);
  }
  let held = 0;
  if (!v) {
    if (!row) { row = cell.ctrl().slice(); row[6] = 0; }
    while (!v) { v = ep.stepTry(row); held++; }
  }
  const tick = ep.lastTry.verdict.tick;
  return { ...ep.lastTry.mon.metrics(), ...v, tick, hash: cell.hash(), n: k, held, ctrl: ep.lastTry.seg.ctrl };
}

export function planHash(ctrl) {
  const flat = planRows(ctrl);
  return fnv64([new Uint8Array(flat.buffer, flat.byteOffset, flat.byteLength)]);
}

// The scene's target: in a pile scene pickTarget's (by class), else a part in the bin with at least minPx camera pixels.
export function chooseTarget(cell, seed, opts = {}) {
  return isPile(cell) ? pickTarget(cell, seed).target : chooseVisible(cell, seed, opts);
}

function chooseVisible(cell, seed, { minPx = 100 } = {}) {
  const cam = new Camera(cell);
  try {
    const { seg } = cam.render(), px = new Uint32Array(cell.n + 1);
    for (let k = 0; k < seg.length; k++) if (seg[k] >= 1 && seg[k] <= cell.n) px[seg[k]]++;
    const tick = cell.tick(0), ok = [];
    for (let i = 0; i < cell.n; i++) if (cell.active[i] && tick.inside[i] && px[i + 1] >= minPx) ok.push(i);
    if (!ok.length) return null;
    let z = (Number(seed) >>> 0) + 0x9e3779b9 | 0;
    z = Math.imul(z ^ (z >>> 16), 0x85ebca6b); z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35); z = (z ^ (z >>> 16)) >>> 0;
    return ok[z % ok.length];
  } finally {
    cam.dispose();
  }
}

// Pile scenes (meta.config.rules.mode == "pile"): target classes, from the settled state.
// - Top-down view: a `res` grid over the bin floor, looked at straight down. Each part's silhouette and its top surface
//   height on every grid point come from its ray mesh (the mesh the camera sees), rasterised; the first part on a grid
//   point is the highest one there.
// - vis(i): share of i's silhouette points where it is the first part.
// - above(i): parts j higher than i on >= overlap_px of i's silhouette points (they lie on or over i).
// - depth(i): the most parts stacked over i on at least max(overlap_px, depth_share * silhouette) of its points; 0 when
//   nothing (beyond that share) lies over it.
// - pressed(i): a part in contact with i (hull contacts) whose top is more than press_dz above i's top.
// - cam_px(i): pixels of i in the expert's camera seg image (`opts.seg`: that image of this state, else rendered).
// Classes (at most one per part; parts outside the bin or with depth > 1 have none):
// - hidden: depth == 1, vis <= hidden_vis (covered from straight above), cam_px < hidden_cam_share x the pixels it
//   would have alone in the camera (pile removed), and every part covering >= depth_share of it has depth 0 (one
//   layer of parts with nothing on them). A part the camera cannot see (cam_px <= hidden_max_px) that is not hidden
//   has no class.
// - top: above(i) empty and not pressed, cam_px >= min_px.
// - blocked: above(i) not empty or pressed, vis >= vis_blocked, cam_px >= min_px.
// - buried: vis_buried <= vis < vis_blocked, cam_px >= buried_min_px.
export const TARGET_CLASSES = ['top', 'blocked', 'buried', 'hidden'];
export const TARGET_DEFAULTS = { shares: { top: 0.2, blocked: 0.4, buried: 0.25, hidden: 0.15 }, res: 0.003,
                                 min_px: 100, buried_min_px: 20, vis_blocked: 0.6, vis_buried: 0.1, overlap_px: 3,
                                 depth_share: 0.1, hidden_max_px: 0, hidden_vis: 0.05,
                                 hidden_cam_share: 0.15, press_dz: 0.005 };

export function isPile(cell) { return (cell.cfg.rules || {}).mode === 'pile'; }

export function targetSettings(cell) {
  const T = (cell.cfg.rules || {}).target || {};
  const out = { ...TARGET_DEFAULTS, ...T, shares: { ...TARGET_DEFAULTS.shares, ...(T.shares || {}) } };
  for (const k of TARGET_CLASSES) if (!(Number.isFinite(out.shares[k]) && out.shares[k] >= 0)) throw new Error(`rules.target.shares.${k}`);
  return out;
}

// Each part's ray mesh (what the camera sees): vertices in the geom frame and triangles; per cell, read once.
const MESHES = new WeakMap();
function rayMeshes(cell) {
  let out = MESHES.get(cell);
  if (out) return out;
  const m = cell.m;
  out = cell.meta.parts.map((p) => {
    const g = p.ray_geom, mesh = m.geom_dataid[g];
    const va = m.mesh_vertadr[mesh], vn = m.mesh_vertnum[mesh], fa = m.mesh_faceadr[mesh], fn = m.mesh_facenum[mesh];
    return { g, v: Float64Array.from(m.mesh_vert.subarray(3 * va, 3 * (va + vn))), f: Int32Array.from(m.mesh_face.subarray(3 * fa, 3 * (fa + fn))) };
  });
  MESHES.set(cell, out);
  return out;
}

export function classify(cell, opts = {}) {
  const T = { ...targetSettings(cell), ...opts };
  const { m, d, n } = cell, c = cell.crate;
  const nx = Math.max(1, Math.round((2 * c.ix) / T.res)), ny = Math.max(1, Math.round((2 * c.iy) / T.res));
  const hx = (2 * c.ix) / nx, hy = (2 * c.iy) / ny, N = nx * ny;
  const label = new Int32Array(N).fill(-1);
  const sil = new Array(n).fill(null);
  const meshes = rayMeshes(cell), X = d.geom_xmat, O = d.geom_xpos;
  for (let i = 0; i < n; i++) {
    if (!cell.active[i]) continue;
    // the part's ray mesh in the bin frame, rasterised straight down: the highest surface on each grid point
    const { g, v, f } = meshes[i], p3 = 3 * g, R = 9 * g, nv = v.length / 3;
    const U = new Float64Array(nv), V = new Float64Array(nv), Z = new Float64Array(nv);
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (let k = 0; k < nv; k++) {
      const a = v[3 * k], b2 = v[3 * k + 1], e = v[3 * k + 2];
      const wx = O[p3] + X[R] * a + X[R + 1] * b2 + X[R + 2] * e - c.x;
      const wy = O[p3 + 1] + X[R + 3] * a + X[R + 4] * b2 + X[R + 5] * e - c.y;
      Z[k] = O[p3 + 2] + X[R + 6] * a + X[R + 7] * b2 + X[R + 8] * e;
      // grid coordinates: cell (a, b) has its centre at (a, b)
      U[k] = (c.cos * wx + c.sin * wy + c.ix) / hx - 0.5;
      V[k] = (-c.sin * wx + c.cos * wy + c.iy) / hy - 0.5;
      if (U[k] < x0) x0 = U[k]; if (U[k] > x1) x1 = U[k]; if (V[k] < y0) y0 = V[k]; if (V[k] > y1) y1 = V[k];
    }
    const a0 = Math.max(0, Math.ceil(x0)), a1 = Math.min(nx - 1, Math.floor(x1));
    const b0 = Math.max(0, Math.ceil(y0)), b1 = Math.min(ny - 1, Math.floor(y1));
    const S = { px: new Int32Array(0), z: new Float64Array(0), a0, a1, b0, b1, top: -Infinity };
    sil[i] = S;
    if (a1 < a0 || b1 < b0) continue;
    const W = b1 - b0 + 1, H = new Float64Array((a1 - a0 + 1) * W).fill(-Infinity);
    for (let t = 0; t < f.length; t += 3) {
      const i0 = f[t], i1 = f[t + 1], i2 = f[t + 2];
      const ux = U[i0], uy = V[i0], vx = U[i1] - ux, vy = V[i1] - uy, wx = U[i2] - ux, wy = V[i2] - uy;
      const den = vx * wy - vy * wx;
      if (Math.abs(den) < 1e-12) continue;
      const ta0 = Math.max(a0, Math.ceil(Math.min(ux, U[i1], U[i2]))), ta1 = Math.min(a1, Math.floor(Math.max(ux, U[i1], U[i2])));
      const tb0 = Math.max(b0, Math.ceil(Math.min(uy, V[i1], V[i2]))), tb1 = Math.min(b1, Math.floor(Math.max(uy, V[i1], V[i2])));
      for (let a = ta0; a <= ta1; a++) for (let bb = tb0; bb <= tb1; bb++) {
        const px = a - ux, py = bb - uy;
        const s1 = (px * wy - py * wx) / den, s2 = (vx * py - vy * px) / den;
        if (s1 < -1e-9 || s2 < -1e-9 || s1 + s2 > 1 + 1e-9) continue;
        const z = Z[i0] + s1 * (Z[i1] - Z[i0]) + s2 * (Z[i2] - Z[i0]), q = (a - a0) * W + (bb - b0);
        if (z > H[q]) H[q] = z;
      }
    }
    const px = [], z = [];
    for (let a = a0; a <= a1; a++) for (let bb = b0; bb <= b1; bb++) {
      const h = H[(a - a0) * W + (bb - b0)];
      if (h > -Infinity) { px.push(a * ny + bb); z.push(h); if (h > S.top) S.top = h; }
    }
    S.px = Int32Array.from(px); S.z = Float64Array.from(z);
  }
  // the first part on each ray: the highest silhouette there
  const zTop = new Float64Array(N).fill(-Infinity);
  for (let i = 0; i < n; i++) {
    const S = sil[i];
    if (!S) continue;
    for (let q = 0; q < S.px.length; q++) if (S.z[q] > zTop[S.px[q]]) { zTop[S.px[q]] = S.z[q]; label[S.px[q]] = i; }
  }

  const camPx = new Int32Array(n);
  let seg = opts.seg;
  if (!seg) {
    const cam = new Camera(cell);
    try { seg = cam.render().seg; } finally { cam.dispose(); }
  }
  for (let k = 0; k < seg.length; k++) if (seg[k] >= 1 && seg[k] <= n) camPx[seg[k] - 1]++;
  // camera pixels a part would have with the rest of the pile removed (only asked for parts that may be hidden)
  const camAlone = (i) => {
    const mj = cell.mj, K = cell.meta.camera, cm = new Camera(cell), g = cell.meta.parts[i].ray_geom, was = m.geom_group[g];
    const N = K.width * K.height, gid = new mj.IntBuffer(N), dist = new mj.DoubleBuffer(N);
    m.geom_group[g] = 5;
    try {
      mj.mj_multiRay(m, d, K.pos, cm.vec, [0, 0, 0, 0, 0, 1], false, -1, gid, dist, null, N, K.max_depth);
      const G = gid.GetView();
      let c = 0;
      for (let k = 0; k < N; k++) if (G[k] === g) c++;
      return c;
    } finally { m.geom_group[g] = was; gid.delete(); dist.delete(); cm.dispose(); }
  };

  const vis = new Float64Array(n), above = Array.from({ length: n }, () => []), depth = new Int32Array(n);
  const share = Array.from({ length: n }, () => []);
  const zAt = new Float64Array(N).fill(NaN), cnt = new Int32Array(N);
  const ids = [];
  for (let i = 0; i < n; i++) if (sil[i] && sil[i].px.length) ids.push(i);
  for (const i of ids) {
    const S = sil[i];
    let own = 0;
    for (let q = 0; q < S.px.length; q++) { if (label[S.px[q]] === i) own++; zAt[S.px[q]] = S.z[q]; }
    vis[i] = own / S.px.length;
    if (own < S.px.length) {
      for (const j of ids) {
        if (j === i) continue;
        const R = sil[j];
        if (R.a0 > S.a1 || R.a1 < S.a0 || R.b0 > S.b1 || R.b1 < S.b0) continue;
        let over = 0;
        for (let q = 0; q < R.px.length; q++) { const h = zAt[R.px[q]]; if (h === h && R.z[q] > h + 1e-4) over++; }
        if (over < T.overlap_px) continue;
        above[i].push(j);
        share[i].push(over / S.px.length);
        for (let q = 0; q < R.px.length; q++) { const h = zAt[R.px[q]]; if (h === h && R.z[q] > h + 1e-4) cnt[R.px[q]]++; }
      }
      // depth: the most parts stacked over i on at least depth_share of its rays (and overlap_px rays)
      const hist = new Int32Array(above[i].length + 2), need = Math.max(T.overlap_px, T.depth_share * S.px.length);
      for (let q = 0; q < S.px.length; q++) hist[cnt[S.px[q]]]++;
      let tail = 0;
      for (let k = hist.length - 1; k > 0; k--) { tail += hist[k]; if (tail >= need) { depth[i] = k; break; } }
    }
    for (let q = 0; q < S.px.length; q++) { zAt[S.px[q]] = NaN; cnt[S.px[q]] = 0; }
  }

  const pressed = new Uint8Array(n), con = d.contact;
  try {
    for (let k = 0, nc = d.ncon; k < nc; k++) {
      const ct = con.get(k);
      const i = cell.bodyPart[m.geom_bodyid[ct.geom1]], j = cell.bodyPart[m.geom_bodyid[ct.geom2]];
      ct.delete();
      if (i < 0 || j < 0 || i === j || !sil[i] || !sil[j]) continue;
      if (sil[j].top > sil[i].top + T.press_dz) pressed[i] = 1;
      if (sil[i].top > sil[j].top + T.press_dz) pressed[j] = 1;
    }
  } finally { con.delete(); }

  const cls = new Array(n).fill(null);
  for (let i = 0; i < n; i++) {
    if (!cell.active[i] || !cell.inside(i) || !sil[i] || !sil[i].px.length || depth[i] > 1) continue;
    if (depth[i] === 1 && vis[i] <= T.hidden_vis
        && above[i].every((j, k) => share[i][k] < T.depth_share || depth[j] === 0)) {
      const alone = camAlone(i);
      if (alone > 0 && camPx[i] < T.hidden_cam_share * alone) { cls[i] = 'hidden'; continue; }
    }
    if (camPx[i] <= T.hidden_max_px) continue;
    if (!above[i].length && !pressed[i]) { if (camPx[i] >= T.min_px) cls[i] = 'top'; }
    else if (vis[i] >= T.vis_blocked) { if (camPx[i] >= T.min_px) cls[i] = 'blocked'; }
    else if (vis[i] >= T.vis_buried) { if (camPx[i] >= T.buried_min_px) cls[i] = 'buried'; }
  }
  const counts = Object.fromEntries(TARGET_CLASSES.map((k) => [k, 0]));
  for (const k of cls) if (k) counts[k]++;
  return { cls, counts, vis, depth, above, share, pressed, camPx, silPx: sil.map((s) => (s ? s.px.length : 0)) };
}

function mix(seed, salt) {
  let z = ((Number(seed) >>> 0) + Math.imul(salt, 0x9e3779b9) + 0x9e3779b9) | 0;
  z = Math.imul(z ^ (z >>> 16), 0x85ebca6b); z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35);
  return (z ^ (z >>> 16)) >>> 0;
}

// The seed of pick k of a scene (multi-pick pile scenes): the scene seed for pick 0.
export function pickSeed(seed, k) {
  return k === 0 ? Number(seed) >>> 0 : mix(mix(seed, 0x5049434b), k);
}

// The target of a scene and its class: pile scenes draw a class by `rules.target.shares` (classes with no part drop
// out and the rest keep their ratio), then a part of that class, both from the seed. Old scenes: chooseTarget, class null.
// `cover` (rain fills; the fill's cover part index, -1 when the scene has none or no part fitted): hidden targets come
// from the cover only — the covered part when it classifies `hidden`, else the class is drawn among the other three.
// `exclude`: parts never to choose (multi-pick scenes: targets an earlier pick failed on).
export function pickTarget(cell, seed, opts = {}) {
  const { cover, exclude, ...cls } = opts;
  if (!isPile(cell)) return { target: chooseVisible(cell, seed, cls), target_class: null, counts: null };
  const C = classify(cell, cls), T = targetSettings(cell);
  if (Number.isInteger(cover)) {
    if (cover >= 0 && C.cls[cover] === 'hidden') return { target: cover, target_class: 'hidden', counts: C.counts };
    T.shares = { ...T.shares, hidden: 0 };
  }
  const by = Object.fromEntries(TARGET_CLASSES.map((k) => [k, []]));
  const skip = new Set(exclude || []);
  C.cls.forEach((k, i) => { if (k && !skip.has(i)) by[k].push(i); });
  const live = TARGET_CLASSES.filter((k) => by[k].length && T.shares[k] > 0);
  const pool = live.length ? live : TARGET_CLASSES.filter((k) => by[k].length);
  if (!pool.length) return { target: null, target_class: null, counts: C.counts };
  const w = pool.map((k) => (live.length ? T.shares[k] : 1)), tot = w.reduce((a, b) => a + b, 0);
  let u = (mix(seed, 1) / 4294967296) * tot, k = 0;
  while (k < pool.length - 1 && u >= w[k]) { u -= w[k]; k++; }
  const list = by[pool[k]];
  return { target: list[mix(seed, 2) % list.length], target_class: pool[k], counts: C.counts };
}
