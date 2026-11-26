export function fnv64(chunks) {
  let h1 = 0x811c9dc5 | 0, h2 = 0xcbf29ce4 | 0;
  for (const c of chunks) {
    const b = c instanceof Uint8Array ? c : new Uint8Array(c.buffer, c.byteOffset, c.byteLength);
    for (let i = 0; i < b.length; i++) {
      h1 = Math.imul(h1 ^ b[i], 0x01000193);
      h2 = Math.imul(h2 ^ b[i] ^ (h1 >>> 24), 0x01000193);
    }
  }
  return (h2 >>> 0).toString(16).padStart(8, '0') + (h1 >>> 0).toString(16).padStart(8, '0');
}

export function b64encode(u8) {
  if (typeof Buffer !== 'undefined') return Buffer.from(u8.buffer, u8.byteOffset, u8.byteLength).toString('base64');
  let s = '';
  for (let i = 0; i < u8.length; i += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(i, i + 0x8000));
  return btoa(s);
}

export function b64decode(s) {
  if (typeof Buffer !== 'undefined') { const b = Buffer.from(s, 'base64'); return new Uint8Array(b.buffer, b.byteOffset, b.length); }
  const bin = atob(s), u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
}

export const f64ToB64 = (a) => b64encode(new Uint8Array(Float64Array.from(a).buffer));
export const b64ToF64 = (s) => { const u = b64decode(s); return new Float64Array(u.slice().buffer); };
