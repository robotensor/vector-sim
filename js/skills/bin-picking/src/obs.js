export class Camera {
  constructor(cell, spec = null) {
    const mj = cell.mj, c = spec || cell.meta.camera;
    this.cell = cell;
    this.c = c;
    const N = (this.N = c.width * c.height);
    const R = c.R_cam_to_world;
    const vec = new Float64Array(3 * N);
    for (let v = 0; v < c.height; v++) for (let u = 0; u < c.width; u++) {
      const x = (u - c.cx) / c.fx, y = (v - c.cy) / c.fy, k = 3 * (v * c.width + u);
      vec[k] = R[0][0] * x + R[0][1] * y + R[0][2];
      vec[k + 1] = R[1][0] * x + R[1][1] * y + R[1][2];
      vec[k + 2] = R[2][0] * x + R[2][1] * y + R[2][2];
    }
    this.vec = vec;
    this.gid = new mj.IntBuffer(N);
    this.dist = new mj.DoubleBuffer(N);
    this.group = [1, 1, 1, 0, 0, 0];
    const M = cell.meta, label = new Uint8Array(M.ngeom);
    const robot = new Uint8Array(M.nbody);
    for (const b of M.robot.robot_bodies) robot[b] = 1;
    for (let g = 0; g < M.ngeom; g++) {
      const b = cell.m.geom_bodyid[g], i = cell.bodyPart[b];
      label[g] = i >= 0 ? i + 1 : robot[b] ? 255 : 0;
    }
    const crateBody = cell.mj.mj_name2id(cell.m, cell.mj.mjtObj.mjOBJ_BODY.value, 'crate');
    for (let g = 0; g < M.ngeom; g++) if (cell.m.geom_bodyid[g] === crateBody) label[g] = 254;
    this.label = label;
  }

  render() {
    const { cell, c, N } = this;
    cell.mj.mj_multiRay(cell.m, cell.d, c.pos, this.vec, this.group, true, -1, this.gid, this.dist, null, N, c.max_depth);
    const dist = this.dist.GetView(), gid = this.gid.GetView();
    const depth = new Uint16Array(N), seg = new Uint8Array(N);
    for (let k = 0; k < N; k++) {
      const z = dist[k];
      if (z < 0 || gid[k] < 0) continue;
      depth[k] = Math.min(65535, Math.round(z / c.depth_unit));
      seg[k] = this.label[gid[k]];
    }
    return { width: c.width, height: c.height, depth, seg };
  }

  renderRGB(img = null) {
    const { cell, c, N } = this;
    img ||= this.render();
    if (!this.colors) {
      const m = cell.m, M = cell.meta, col = new Uint8Array(3 * M.ngeom);
      const srgb = (x) => Math.round(255 * (x <= 0.0031308 ? 12.92 * x : 1.055 * Math.pow(x, 1 / 2.4) - 0.055));
      for (let g = 0; g < M.ngeom; g++) {
        const b = m.geom_bodyid[g], i = cell.bodyPart[b];
        const look = i >= 0 ? (M.part_types[M.parts[i].type] || {}).look : null;
        const rgb = look && Array.isArray(look.color) ? look.color.map((x) => srgb(Math.min(1, Math.max(0, x))))
          : [0, 1, 2].map((k) => Math.round(255 * Math.min(1, Math.max(0, m.geom_rgba[4 * g + k]))));
        col.set(rgb, 3 * g);
      }
      this.colors = col;
    }
    const col = this.colors, gid = this.gid.GetView(), out = new Uint8Array(3 * N);
    const z0 = c.pos[2] - cell.crate.rim, span = 0.3;
    for (let k = 0; k < N; k++) {
      const g = gid[k];
      if (g < 0 || !img.depth[k]) continue;
      const f = 1 - 0.4 * Math.min(1, Math.max(0, (img.depth[k] * c.depth_unit - z0) / span));
      for (let j = 0; j < 3; j++) out[3 * k + j] = Math.round(col[3 * g + j] * f);
    }
    return out;
  }

  dispose() { this.gid.delete(); this.dist.delete(); }
}

export function observe(cell, camera) {
  const img = camera.render();
  const { q, grip } = cell.proprio();
  return { ...img, q, grip };
}

export function obsBytes(o) {
  return [new Uint8Array(o.depth.buffer, o.depth.byteOffset, o.depth.byteLength), o.seg,
          new Uint8Array(Float64Array.from(o.q).buffer), new Uint8Array(Float64Array.of(o.grip).buffer)];
}

export function obsHash(o, createHash) {
  const h = createHash();
  for (const b of obsBytes(o)) h.update(b);
  return h.digest('hex');
}
