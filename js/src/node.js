import fs from 'node:fs';
import path from 'node:path';
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
  const files = new Map();
  for (const p of paths) if (p.endsWith('.stl')) files.set(p, new Uint8Array(fs.readFileSync(path.join(dir, p))));
  return { files, bundle: man.bundle };
}
