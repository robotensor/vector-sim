import test from 'node:test';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const PKG_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const PKG = JSON.parse(fs.readFileSync(path.join(PKG_DIR, 'package.json'), 'utf8'));
const SRC = path.join(PKG_DIR, 'skills', 'bin-picking', 'src');
const SRC_FILES = ['actions.js', 'bundle.js', 'cell.js', 'driver.js', 'episode.js', 'hash.js', 'ik.js', 'index.js', 'node.js',
                   'obs.js', 'picks.js', 'plan.js', 'replay.js', 'rules.js'];

test('the package is @vector/sim', () => {
  assert.equal(PKG.name, '@vector/sim');
  assert.deepEqual(fs.readdirSync(SRC).filter((f) => f.endsWith('.js')).sort(), SRC_FILES);
});

test('@vector/sim is the bin-picking engine index', async () => {
  const root = await import('@vector/sim');
  assert.equal(root, await import('@vector/sim/skills/bin-picking/src/index.js'));
  assert.equal(root, await import('@vector/sim/index.js'));
  assert.equal(typeof root.Cell, 'function');
});

test('@vector/sim/testing is the bin-picking test helpers', async () => {
  const testing = await import('@vector/sim/testing');
  assert.equal(testing, await import('@vector/sim/skills/bin-picking/test/helpers.js'));
  for (const k of ['ROOT', 'ASSETS', 'SKIP', 'SETTINGS', 'BUNDLE', 'compose']) assert.ok(k in testing, k);
  assert.equal(testing.ROOT, path.dirname(PKG_DIR));
});

test('@vector/sim/<file> is the same module as @vector/sim/skills/bin-picking/src/<file>', async () => {
  for (const f of SRC_FILES) {
    const a = await import(`@vector/sim/${f}`);
    assert.equal(a, await import(`@vector/sim/skills/bin-picking/src/${f}`), f);
  }
});

test('every bin exists, is executable and starts with #!/usr/bin/env node', () => {
  assert.deepEqual(Object.keys(PKG.bin).sort(), ['vector-verify', 'vector-verify-unit']);
  for (const [name, rel] of Object.entries(PKG.bin)) {
    const p = path.join(PKG_DIR, rel);
    assert.ok(fs.statSync(p).isFile(), name);
    assert.doesNotThrow(() => fs.accessSync(p, fs.constants.X_OK), `${name} is not executable`);
    assert.ok(fs.readFileSync(p, 'utf8').startsWith('#!/usr/bin/env node\n'), name);
  }
});

test('simPin() names this package and the pinned files', async () => {
  const { simPin, PIN_FILES } = await import('@vector/sim/node.js');
  assert.deepEqual(PIN_FILES, ['actions.js', 'cell.js', 'episode.js', 'hash.js', 'obs.js', 'picks.js', 'plan.js', 'rules.js']);
  const h = crypto.createHash('sha256');
  for (const f of PIN_FILES) h.update(`${f}\0`).update(fs.readFileSync(path.join(SRC, f))).update('\0');
  const pin = simPin();
  assert.equal(pin.sim, PKG.version);
  assert.equal(pin.code, h.digest('hex'));
  assert.match(pin.wasm, /^[0-9a-f]{64}$/);
  assert.equal(pin.mujoco, PKG.dependencies['@mujoco/mujoco']);
});
