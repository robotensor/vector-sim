import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));

export const MISSING = 'no asset bundle: set VECTOR_ENV_BUNDLE to a bundle folder, or run '
  + 'VECTOR_ENV_ASSETS_SOURCE=<hf://repo or pack folder> vector-env assets pull '
  + '(it fills VECTOR_ENV_CACHE, default ~/.cache/vector-env/assets/<version>)';

const expand = (p) => path.resolve(p.startsWith('~/') ? path.join(os.homedir(), p.slice(2)) : p);
const isBundle = (d) => fs.existsSync(path.join(d, 'manifest.json'));

export function envDir() {
  return expand(process.env.VECTOR_ENV_DIR || path.resolve(HERE, '..', '..', '..', 'vector-env'));
}

export function pinnedVersion(dir = envDir()) {
  if (process.env.VECTOR_ENV_ASSETS_VERSION) return process.env.VECTOR_ENV_ASSETS_VERSION;
  let text;
  try { text = fs.readFileSync(path.join(dir, 'vector_env', 'env.yaml'), 'utf8'); } catch { return null; }
  const m = /^assets:[ \t]*\r?\n(?:[ \t]+.*\r?\n)*?[ \t]+version:[ \t]*["']?([^"'\s#]+)/m.exec(text);
  return m ? m[1] : null;
}

export function findBundle() {
  const e = process.env.VECTOR_ENV_BUNDLE || process.env.VECTOR_SIM_BUNDLE;
  if (e) return isBundle(expand(e)) ? expand(e) : null;
  const dir = envDir();
  if (isBundle(path.join(dir, 'bundle'))) return path.join(dir, 'bundle');
  const v = pinnedVersion(dir);
  if (!v) return null;
  const cache = process.env.VECTOR_ENV_CACHE ? expand(process.env.VECTOR_ENV_CACHE) : path.join(os.homedir(), '.cache', 'vector-env', 'assets');
  return isBundle(path.join(cache, v)) ? path.join(cache, v) : null;
}
