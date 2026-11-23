import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { bundleFiles } from '../src/node.js';

test('scene asset paths must stay inside the bundle (links the bundle itself holds are its own)', () => {
  const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'vsim-paths-'));
  const dir = path.join(tmp, 'bundle');
  fs.mkdirSync(path.join(dir, 'library'), { recursive: true });
  fs.writeFileSync(path.join(dir, 'manifest.json'), JSON.stringify({ bundle: 'b', files: [{ path: 'library/a.stl' }] }));
  fs.writeFileSync(path.join(dir, 'library', 'a.stl'), 'solid a');
  fs.writeFileSync(path.join(tmp, 'secret.stl'), 'secret');
  fs.symlinkSync(path.join(tmp, 'secret.stl'), path.join(dir, 'library', 'link.stl'));
  try {
    assert.equal(bundleFiles(dir).files.get('library/a.stl').length, 7);
    assert.equal(bundleFiles(dir, { meta: { files: ['library/a.stl'] } }).files.size, 1);
    assert.equal(bundleFiles(dir, { xml: '<mesh file="library/a.stl"/>', meta: {} }).files.size, 1);
    assert.equal(bundleFiles(dir, { meta: { files: ['library/link.stl'] } }).files.size, 1);
    assert.throws(() => bundleFiles(dir, { meta: { files: ['library/none.stl'] } }), /ENOENT/);
    for (const p of ['../secret.stl', 'library/../../secret.stl', path.join(tmp, 'secret.stl'),
                     './library/a.stl', 'library//a.stl', 'library\\a.stl'])
      assert.throws(() => bundleFiles(dir, { meta: { files: [p] } }), /not a file inside the bundle/, p);
    assert.throws(() => bundleFiles(dir, { xml: '<mesh file="../secret.stl"/>', meta: {} }), /inside the bundle/);
  } finally {
    fs.rmSync(tmp, { recursive: true, force: true });
  }
});
