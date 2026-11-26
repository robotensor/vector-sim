#!/usr/bin/env node
import crypto from 'node:crypto';
import { mujoco, bundleFiles } from '../src/node.js';
import { findBundle, MISSING } from '../src/bundle.js';
import { Cell } from '../src/cell.js';
import { replayUnit } from '../src/replay.js';

const i = process.argv.indexOf('--bundle');
const bundle = i > 0 ? process.argv[i + 1] : process.env.VECTOR_SIM_BUNDLE || findBundle();
if (!bundle) { console.error(MISSING); process.exit(2); }
const chunks = [];
for await (const c of process.stdin) chunks.push(c);
const t0 = performance.now();
let out;
try {
  const job = JSON.parse(Buffer.concat(chunks).toString('utf8'));
  const mj = await mujoco();
  const { files } = bundleFiles(bundle, job.scene);
  const cell = Cell.create(mj, files, job.scene);
  try {
    out = replayUnit(cell, { checkpoint: job.checkpoint || null, index: job.index, segments: job.segments,
                             chunks: job.chunks || {}, createHash: () => crypto.createHash('sha256') });
  } finally {
    cell.dispose();
  }
} catch (e) {
  out = { ok: false, error: `verifier error: ${e.message}`, at: null, checkpoint: null, try: null };
}
out.replay_s = Math.round(performance.now() - t0) / 1000;
process.stdout.write(JSON.stringify(out));
