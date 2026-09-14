// Ratakan tanah — tiap layer diputar supaya tanahnya datar di z = 0.
//
// Bawaannya MENYALA: tiap berkas yang dibuka langsung diratakan sebelum masuk
// daftar layer. Tombol di panel atas mematikannya, dan mematikan berarti
// memutar balik dengan matriks yang sama persis — bukan mencari tanah lagi —
// jadi nyala/mati bolak-balik tidak menggeser apa pun.
//
// Bidang tanahnya dicari backend (/tanah). Di sini hanya menerapkan matriks ke
// titik, ke seluruh tumpukan undo layer itu, dan ke warnanya.

import { toast } from './hud.js';
import * as layers from './layers.js';
import { sampel } from './palet.js';

let _aktif = true;
let _sibuk = false;
const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }
const beritahu = () => _pendengar.forEach(f => f());

export function aktifkah() { return _aktif; }
export function sibukkah() { return _sibuk; }

// ============================================================
// Warna ketinggian
// ============================================================
// mcaptopc.py mewarnai titik menurut Z dengan viridis. Sesudah diratakan,
// warna lama itu masih menggambar kemiringan aslinya — tanah yang sudah datar
// tampak bergradasi dari biru ke kuning. Maka warna seperti itu dihitung ulang;
// warna lain (abu-abu, warna asli sensor) tidak disentuh.
const viridis = (t, out, o) => sampel('viridis', t, out, o);

function rentangZ(cloud) {
  let lo = Infinity, hi = -Infinity;
  for (let i = 2; i < cloud.length; i += 6) {
    if (cloud[i] < lo) lo = cloud[i];
    if (cloud[i] > hi) hi = cloud[i];
  }
  return [lo, hi - lo];
}

/** Apakah warna cloud ini viridis dari Z-nya sendiri? Terukur pada scan
 *  lapangan dan merged.ply: galat rata ~0,003. Ambang 0,03 memberi ruang
 *  untuk perataan warna oleh voxel tanpa meloloskan warna jenis lain. */
function warnaDariZ(cloud) {
  const n = cloud.length / 6;
  if (n < 100) return false;
  const [lo, span] = rentangZ(cloud);
  if (!(span > 0)) return false;
  const c = [0, 0, 0];
  const lompat = Math.max(1, Math.floor(n / 4000));
  let galat = 0, k = 0;
  for (let i = 0; i < n; i += lompat) {
    const o = i * 6;
    viridis((cloud[o + 2] - lo) / span, c, 0);
    galat += Math.abs(cloud[o + 3] - c[0]) + Math.abs(cloud[o + 4] - c[1])
           + Math.abs(cloud[o + 5] - c[2]);
    k++;
  }
  return galat / (3 * k) < 0.03;
}

// ============================================================
// Matriks
// ============================================================
/** Cloud baru: p' = R p + t, dan (bila perlu) warna viridis dari Z baru. */
function ubahCloud(cloud, M, warnai) {
  const out = new Float32Array(cloud.length);
  const [r0, r1, r2] = M;
  for (let o = 0; o < cloud.length; o += 6) {
    const x = cloud[o], y = cloud[o + 1], z = cloud[o + 2];
    out[o] = r0[0] * x + r0[1] * y + r0[2] * z + r0[3];
    out[o + 1] = r1[0] * x + r1[1] * y + r1[2] * z + r1[3];
    out[o + 2] = r2[0] * x + r2[1] * y + r2[2] * z + r2[3];
    out[o + 3] = cloud[o + 3]; out[o + 4] = cloud[o + 4]; out[o + 5] = cloud[o + 5];
  }
  if (warnai) {
    const [lo, span] = rentangZ(out);
    const s = span > 0 ? span : 1;
    for (let o = 0; o < out.length; o += 6) viridis((out[o + 2] - lo) / s, out, o + 3);
  }
  return out;
}

/** Kebalikan transformasi kaku: R' = Rᵀ, t' = −Rᵀ t. */
function balik(M) {
  const R = [0, 1, 2].map(i => [0, 1, 2].map(j => M[j][i]));
  const t = [0, 1, 2].map(i => -(R[i][0] * M[0][3] + R[i][1] * M[1][3] + R[i][2] * M[2][3]));
  return [[...R[0], t[0]], [...R[1], t[1]], [...R[2], t[2]], [0, 0, 0, 1]];
}

async function mintaTanah(cloud) {
  const n = cloud.length / 6, xyz = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    xyz[i * 3] = cloud[i * 6]; xyz[i * 3 + 1] = cloud[i * 6 + 1]; xyz[i * 3 + 2] = cloud[i * 6 + 2];
  }
  const r = await fetch('/tanah', { method: 'POST', body: xyz });
  if (!r.ok) throw new Error((await r.json().catch(() => ({}))).detail || r.statusText);
  return r.json();
}

// ============================================================
// Untuk io.js — sebelum layer ditambahkan
// ============================================================
/** → { cloud, rata }. `rata` disimpan di layer dan dipakai lagi oleh tombol. */
export async function siapkan(cloud) {
  const rata = { M: null, miring: null, terpasang: false, tanpaTanah: false,
                 warnaZ: warnaDariZ(cloud) };
  if (!_aktif) return { cloud, rata };
  try {
    const h = await mintaTanah(cloud);
    if (!h.ditemukan) { rata.tanpaTanah = true; return { cloud, rata }; }
    rata.M = h.matriks;
    rata.miring = h.miring_deg;
    rata.terpasang = true;
    return { cloud: ubahCloud(cloud, rata.M, rata.warnaZ), rata };
  } catch (e) {
    // Server tak terjangkau bukan alasan menolak berkasnya — muat apa adanya.
    toast('Perataan tanah gagal: ' + e.message, true);
    return { cloud, rata };
  }
}

/** Potongan kalimat untuk toast "Dimuat: …". */
export function keterangan(rata) {
  if (!rata) return '';
  if (rata.terpasang) return ` · tanah diratakan ${rata.miring.toFixed(1).replace('.', ',')}°`;
  if (rata.tanpaTanah) return ' · tanah tak ditemukan, tidak diratakan';
  return '';
}

// ============================================================
// Tombol panel atas
// ============================================================
export async function setAktif(on) {
  if (_sibuk || on === _aktif) return;
  _sibuk = true;
  _aktif = on;
  beritahu();
  let jumlah = 0, tanpa = 0;
  try {
    for (const L of layers.daftar()) {
      const r = L.rata;
      if (!r) continue;
      if (on && !r.terpasang) {
        // Layer yang dibuka selagi tombol mati belum punya matriks.
        if (!r.M && !r.tanpaTanah) {
          const h = await mintaTanah(L.cloud);
          if (h.ditemukan) { r.M = h.matriks; r.miring = h.miring_deg; }
          else r.tanpaTanah = true;
        }
        if (!r.M) { tanpa++; continue; }
        layers.petakanCloud(L.id, c => ubahCloud(c, r.M, r.warnaZ));
        r.terpasang = true;
        jumlah++;
      } else if (!on && r.terpasang) {
        const B = balik(r.M);
        layers.petakanCloud(L.id, c => ubahCloud(c, B, r.warnaZ));
        r.terpasang = false;
        jumlah++;
      }
    }
  } catch (e) {
    toast('Perataan tanah gagal: ' + e.message, true);
  } finally {
    _sibuk = false;
    beritahu();
  }
  return { jumlah, tanpa };
}
