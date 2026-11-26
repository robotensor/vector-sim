#!/usr/bin/env node
import fs from 'node:fs';
import { parseArgs } from 'node:util';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, b64ToF64 } from '../src/index.js';
import { ASSETS as BUNDLE } from '../test/helpers.js';
const { values: a, positionals: [inp, out] } = parseArgs({ allowPositionals: true, options: { try: { type: 'string', default: '0' }, every: { type: 'string', default: '10' } } });
const run = JSON.parse(fs.readFileSync(inp));
const mj = await mujoco(); const { files } = bundleFiles(BUNDLE, run.scene);
const cell = Cell.create(mj, files, run.scene), ep = new Episode(cell);
ep.beginFill(); while (!ep.fillStep());
const frames = [];
for (const unit of run.log.units.slice(1)) {
  const s = unit[0];
  const ctrl = b64ToF64(s.ctrl);
  ep.beginTry(s.target);
  for (let j = 0; j < s.n; j++) {
    ep.stepTry(Array.from(ctrl.subarray(7 * j, 7 * j + 7)));
    if (s.i === +a.try && (j % +a.every === 0 || j === s.n - 1)) {
      const b = cell.meta.parts[s.target].body;
      frames.push({ tick: j + 1, qpos: Array.from(cell.d.qpos), tcp: cell.tcp(), target: Array.from(cell.d.xpos.subarray(3 * b, 3 * b + 3)), grip: cell.proprio().grip });
    }
  }
  if (s.i === +a.try) break;
  ep.endTry();
}
for (const f of frames) console.log(f.tick, 'tcp', f.tcp.pos.map((v) => v.toFixed(3)).join(','), 'target', f.target.map((v) => v.toFixed(3)).join(','), 'grip', f.grip.toFixed(2), 'tcp_z_axis', [f.tcp.mat[2], f.tcp.mat[5], f.tcp.mat[8]].map((v) => v.toFixed(2)).join(','));
fs.writeFileSync(out, JSON.stringify({ scene: run.scene, frames }));
