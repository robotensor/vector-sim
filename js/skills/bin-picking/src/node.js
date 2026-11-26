import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import load from '@mujoco/mujoco';

let _mj = null;
export async function mujoco() { return (_mj ||= await load()); }

export function bundleFiles(dir, scene = null) {
  const man = JSON.parse(fs.readFileSync(path.join(dir, 'manifest.json'), 'utf8'));
  let paths;
  if (scene) {
    const sc = scene.scene || scene;
    paths = Array.isArray(sc.meta?.files) ? sc.meta.files.map((f) => (typeof f === 'string' ? f : f.path))
      : [...sc.xml.matchAll(/\sfile="([^"]+)"/g)].map((m) => m[1]);
  } else paths = man.files.map((f) => f.path);
  const root = path.resolve(dir);
  const files = new Map();
  for (const p of paths) if (typeof p === 'string' && p.endsWith('.stl')) files.set(p, new Uint8Array(fs.readFileSync(inside(root, p))));
  return { files, bundle: man.bundle };
}

export function inside(root, p) {
  const bad = () => new Error(`asset path ${JSON.stringify(String(p).slice(0, 200))} is not a file inside the bundle`);
  if (typeof p !== 'string' || !p || p.includes('\0') || p.includes('\\') || path.isAbsolute(p)) throw bad();
  if (p.split('/').some((s) => s === '..' || s === '.' || s === '')) throw bad();
  const full = path.resolve(root, p);
  if (!full.startsWith(root + path.sep)) throw bad();
  return full;
}

export const PIN_FILES = ['actions.js', 'cell.js', 'episode.js', 'hash.js', 'rules.js'];

let _pin = null;
export function simPin() {
  if (_pin) return _pin;
  const src = path.dirname(fileURLToPath(import.meta.url));
  const code = crypto.createHash('sha256');
  for (const f of PIN_FILES) code.update(`${f}\0`).update(fs.readFileSync(path.join(src, f))).update('\0');
  const req = createRequire(import.meta.url);
  const wasmPath = req.resolve('@mujoco/mujoco/mujoco.wasm'), pkgDir = path.dirname(wasmPath);
  const wasm = crypto.createHash('sha256').update(fs.readFileSync(wasmPath)).digest('hex');
  const sim = JSON.parse(fs.readFileSync(path.join(src, '..', '..', '..', 'package.json'), 'utf8')).version;
  const mujoco = JSON.parse(fs.readFileSync(path.join(pkgDir, 'package.json'), 'utf8')).version;
  return (_pin = { sim, code: code.digest('hex'), mujoco, wasm });
}
