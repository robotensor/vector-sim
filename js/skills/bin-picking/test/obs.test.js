import test from 'node:test';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import { mujoco, bundleFiles } from '../src/node.js';
import { Cell, Episode, Camera, observe, obsBytes, obsHash, obsWire, b64encode, b64decode } from '../src/index.js';
import { compose, BUNDLE, SKIP, SETTINGS } from './helpers.js';

test('the observation has no poses; its hash covers depth | seg | q | grip; the wire format round-trips', { skip: SKIP }, async () => {
  const mj = await mujoco();
  const scene = compose(SETTINGS, 3);
  const cell = Cell.create(mj, bundleFiles(BUNDLE, scene).files, scene);
  const ep = new Episode(cell), cam = new Camera(cell);
  ep.beginFill();
  while (!ep.fillStep());
  const o = observe(cell, cam);
  assert.deepEqual(Object.keys(o).sort(), ['depth', 'grip', 'height', 'q', 'seg', 'width']);
  assert.equal(o.depth.length, o.width * o.height);
  assert.equal(o.seg.length, o.width * o.height);
  assert.ok(o.seg.some((l) => l >= 1 && l < 254), 'parts are labelled');

  const bytes = obsBytes(o);
  assert.equal(bytes.length, 4);
  const h = crypto.createHash('sha256');
  h.update(Buffer.from(o.depth.buffer, o.depth.byteOffset, o.depth.byteLength));
  h.update(Buffer.from(o.seg));
  h.update(Buffer.from(Float64Array.from(o.q).buffer));
  h.update(Buffer.from(Float64Array.of(o.grip).buffer));
  assert.equal(obsHash(o, () => crypto.createHash('sha256')), h.digest('hex'));

  const w = obsWire(o, b64encode);
  assert.deepEqual(Object.keys(w).sort(), ['depth', 'grip', 'height', 'q', 'seg', 'width']);
  const d = b64decode(w.depth);
  assert.deepEqual(new Uint16Array(d.slice().buffer), o.depth);
  assert.deepEqual(b64decode(w.seg), o.seg);
  assert.equal(obsHash(observe(cell, cam), () => crypto.createHash('sha256')), obsHash(o, () => crypto.createHash('sha256')));
  cam.dispose();
  cell.dispose();
});
