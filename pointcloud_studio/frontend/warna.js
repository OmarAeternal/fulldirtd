// Pewarnaan tampilan dan ukuran titik — isi tab "View".
//
// Warna di sini hanya warna TAMPILAN: yang ditimpa atribut warna geometry,
// bukan `cloud` layer. Ekspor PLY/XYZ tetap membawa warna berkas, jadi
// mengganti palet untuk presentasi tidak pernah mengubah data.
//
// Skala warna dihitung dari gabungan layer yang TERLIHAT, supaya legenda selalu
// cocok dengan yang ada di layar — termasuk saat melangkah dengan ‹ ›.

import * as layers from './layers.js';
import { setSkalaTitik, setLatar } from './viewer.js';
import { sampel, KATEGORI } from './palet.js';

export const BAWAAN = {
  mode: 'z',              // asli | z | jarak | layer | tunggal
  palet: 'viridis',
  sebaran: 'persentil',   // linear | persentil | kuantil | manual
  lo: 0, hi: 2,           // rentang manual, meter
  tingkat: 0,             // 0 = halus; 2–16 = pita warna bertingkat
  tunggal: '#8fb0ff',
  legenda: true,
  skala: 1,               // 1–5 × ukuran titik bawaan
  latar: '#0e1116',
};

// Pilihan cepat di tab View; warna lain lewat pemilih warna.
export const LATAR = [
  ['#0e1116', 'Gelap'], ['#000000', 'Hitam'], ['#3a3f47', 'Abu-abu'],
  ['#e9ecef', 'Terang'], ['#ffffff', 'Putih'],
];

// Disimpan di browser: presentasi di depan juri tidak perlu disetel ulang
// tiap kali halaman dibuka. Kalau penyimpanan diblokir, bawaan saja.
const KUNCI = 'pcs.view.v1';
let _s = muat();
function muat() {
  try {
    const j = JSON.parse(localStorage.getItem(KUNCI));
    if (j && typeof j === 'object') return { ...BAWAAN, ...j };
  } catch { /* abaikan */ }
  return { ...BAWAAN };
}
function simpan() {
  try { localStorage.setItem(KUNCI, JSON.stringify(_s)); } catch { /* abaikan */ }
}

const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }
const beritahu = () => _pendengar.forEach(f => f());

export function state() { return { ..._s }; }

export function set(ubah) {
  Object.assign(_s, ubah);
  simpan();
  if ('skala' in ubah) setSkalaTitik(_s.skala);
  if ('latar' in ubah) setLatar(_s.latar);
  // Ukuran dan latar tidak mengubah warna titik — tak perlu mewarnai ulang.
  if (Object.keys(ubah).some(k => k !== 'skala' && k !== 'latar')) terapkan();
  beritahu();
}

export function kembalikan() {
  _s = { ...BAWAAN };
  simpan();
  setSkalaTitik(_s.skala);
  setLatar(_s.latar);
  terapkan();
  beritahu();
}

export const pakaiGradasi = (mode) => mode === 'z' || mode === 'jarak';

// ============================================================
// Nilai per titik
// ============================================================
// Posisi sensor: titik asal berkas. Sesudah diratakan, titik asal ikut
// berpindah ke kolom translasi matriks perataan.
function sensor(L) {
  const r = L.rata;
  return r && r.terpasang && r.M ? [r.M[0][3], r.M[1][3], r.M[2][3]] : [0, 0, 0];
}

function nilaiLayer(L, mode) {
  const c = L.cloud, n = c.length / 6, v = new Float32Array(n);
  if (mode === 'z') {
    for (let i = 0; i < n; i++) v[i] = c[i * 6 + 2];
  } else {
    const [sx, sy, sz] = sensor(L);
    for (let i = 0; i < n; i++) {
      const dx = c[i * 6] - sx, dy = c[i * 6 + 1] - sy, dz = c[i * 6 + 2] - sz;
      v[i] = Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
  }
  return v;
}

// ============================================================
// Bobot per luas
// ============================================================
// Persentil dan kuantil dihitung per LUAS, bukan per titik. Badan drone yang
// ikut terscan menumpuk ribuan titik dalam 30 cm dari sensor; dihitung per
// titik, persentil 98% scan 0182 jatuh tepat di ketinggian drone (3,3 m) dan
// seluruh tanah serta orang terjepit di satu ujung palet. Tiap kubus 10 cm
// diberi bobot total satu, dibagi rata ke titik-titik di dalamnya.
const KUBUS = 0.10;
const _bobot = new WeakMap();        // cloud → Float32Array; basi = cloud baru

function bobotLayer(L) {
  const c = L.cloud;
  let w = _bobot.get(c);
  if (w) return w;
  const n = c.length / 6, kunci = new Float64Array(n), isi = new Map();
  for (let i = 0; i < n; i++) {
    // Kunci numerik, bukan string: 21 bit per sumbu cukup untuk ±100 km.
    const ix = Math.floor(c[i * 6] / KUBUS) + 1048576;
    const iy = Math.floor(c[i * 6 + 1] / KUBUS) + 1048576;
    const iz = Math.floor(c[i * 6 + 2] / KUBUS) + 1048576;
    const k = (ix * 2097152 + iy) * 2097152 + iz;
    kunci[i] = k;
    isi.set(k, (isi.get(k) || 0) + 1);
  }
  w = new Float32Array(n);
  for (let i = 0; i < n; i++) w[i] = 1 / isi.get(kunci[i]);
  _bobot.set(c, w);
  return w;
}

// ============================================================
// Skala: nilai → t ∈ [0,1]
// ============================================================
// Satu histogram 4096 kelas melayani ketiga sebaran yang butuh statistik:
// persentil dan kuantil dibaca darinya tanpa mengurutkan ratusan ribu titik.
const KELAS = 4096;

function bangunSkala(nilai, bobot) {
  let min = Infinity, max = -Infinity, total = 0;
  for (const v of nilai) {
    for (let i = 0; i < v.length; i++) {
      if (v[i] < min) min = v[i];
      if (v[i] > max) max = v[i];
    }
  }
  if (min > max) return null;
  const lebar = (max - min) || 1;
  const hist = new Float64Array(KELAS);
  nilai.forEach((v, j) => {
    const w = bobot[j];
    for (let i = 0; i < v.length; i++) {
      hist[Math.min(KELAS - 1, Math.floor((v[i] - min) / lebar * KELAS))] += w[i];
      total += w[i];
    }
  });
  const cdf = new Float64Array(KELAS + 1);
  for (let b = 0; b < KELAS; b++) cdf[b + 1] = cdf[b] + hist[b] / total;

  /** nilai pada kuantil p */
  const q = (p) => {
    let b = 0;
    while (b < KELAS - 1 && cdf[b + 1] < p) b++;
    const isi = cdf[b + 1] - cdf[b];
    const f = isi > 0 ? (p - cdf[b]) / isi : 0;
    return min + (b + Math.min(1, Math.max(0, f))) / KELAS * lebar;
  };
  /** kuantil dari nilai v */
  const F = (v) => {
    const x = Math.min(KELAS, Math.max(0, (v - min) / lebar * KELAS));
    const b = Math.min(KELAS - 1, Math.floor(x));
    return cdf[b] + (cdf[b + 1] - cdf[b]) * (x - b);
  };
  return { min, max, q, F };
}

function pitakan(t, k) {
  t = Math.min(1, Math.max(0, t));
  return k >= 2 ? Math.min(k - 1, Math.floor(t * k)) / (k - 1) : t;
}

// Skala yang terakhir dipakai — dibaca legenda dan panel (isian manual).
let _skala = null;
export function skalaTerakhir() { return _skala; }

// ============================================================
// Terapkan ke semua layer
// ============================================================
function hexKeRgb(h) {
  const m = /^#?([0-9a-f]{6})$/i.exec(h || '');
  const x = m ? parseInt(m[1], 16) : 0x8fb0ff;
  return [(x >> 16 & 255) / 255, (x >> 8 & 255) / 255, (x & 255) / 255];
}

export function terapkan() {
  const semua = layers.daftar();
  const { mode } = _s;
  _skala = null;

  if (mode === 'asli') {
    for (const L of semua) layers.tulisWarna(L.id, null);
  } else if (mode === 'layer' || mode === 'tunggal') {
    const satu = hexKeRgb(_s.tunggal);
    semua.forEach((L, j) => {
      const w = mode === 'layer' ? KATEGORI[j % KATEGORI.length] : satu;
      const n = L.cloud.length / 6, rgb = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) { rgb[i * 3] = w[0]; rgb[i * 3 + 1] = w[1]; rgb[i * 3 + 2] = w[2]; }
      layers.tulisWarna(L.id, rgb);
    });
  } else {
    const terlihat = new Set(layers.terlihat().map(L => L.id));
    const nilai = new Map(semua.map(L => [L.id, nilaiLayer(L, mode)]));
    const tampak = semua.filter(L => terlihat.has(L.id));
    const s = bangunSkala(tampak.map(L => nilai.get(L.id)), tampak.map(bobotLayer));
    if (s) {
      let lo = s.min, hi = s.max, keT;
      if (_s.sebaran === 'persentil') { lo = s.q(0.02); hi = s.q(0.98); }
      if (_s.sebaran === 'manual') { lo = +_s.lo; hi = +_s.hi; }
      if (_s.sebaran === 'kuantil') keT = s.F;
      else { const r = (hi - lo) || 1; keT = (v) => (v - lo) / r; }

      const k = _s.tingkat, pal = _s.palet;
      for (const L of semua) {
        const v = nilai.get(L.id), rgb = new Float32Array(v.length * 3);
        for (let i = 0; i < v.length; i++) sampel(pal, pitakan(keT(v[i]), k), rgb, i * 3);
        layers.tulisWarna(L.id, rgb);
      }
      _skala = {
        mode, sebaran: _s.sebaran, palet: pal, tingkat: k, lo, hi,
        // nilai yang diwakili t — untuk label legenda
        nilaiDi: _s.sebaran === 'kuantil' ? s.q : (t) => lo + t * (hi - lo),
        terpotong: _s.sebaran === 'persentil' || _s.sebaran === 'manual',
        rataSemua: layers.terlihat().every(L => L.rata && L.rata.terpasang),
      };
    }
  }
  gambarLegenda();
}

// ============================================================
// Legenda di pojok viewport
// ============================================================
const fmt = (v, rentang) => v.toFixed(rentang >= 20 ? 1 : 2).replace('.', ',').replace('-', '−');
const aman = (s) => String(s).replace(/[&<>"]/g, (c) =>
  ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

/** Gambar gradasi (dengan pita bila ada) ke kanvas mana pun — dipakai legenda
 *  dan pratinjau palet di panel. */
export function gambarGradasi(kanvas, palet, tingkat) {
  const g = kanvas.getContext('2d'), w = kanvas.width, h = kanvas.height;
  const img = g.createImageData(w, h), c = [0, 0, 0];
  for (let x = 0; x < w; x++) {
    sampel(palet, pitakan(x / (w - 1), tingkat), c, 0);
    for (let y = 0; y < h; y++) {
      const o = (y * w + x) * 4;
      img.data[o] = c[0] * 255; img.data[o + 1] = c[1] * 255;
      img.data[o + 2] = c[2] * 255; img.data[o + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
}

function gambarLegenda() {
  const box = document.getElementById('legenda');
  if (!box) return;
  const tampak = layers.terlihat();
  if (!_s.legenda || !tampak.length || _s.mode === 'asli' || _s.mode === 'tunggal'
      || (pakaiGradasi(_s.mode) && !_skala)) {
    box.classList.add('hide');
    return;
  }
  box.classList.remove('hide');

  if (_s.mode === 'layer') {
    const semua = layers.daftar();
    box.innerHTML = '<div class="lg-judul">Warna per layer</div>' + tampak.map(L => {
      const w = KATEGORI[semua.indexOf(L) % KATEGORI.length].map(x => Math.round(x * 255));
      return `<div class="lg-baris"><span class="lg-kotak" style="background:rgb(${w})"></span>` +
             `${aman(L.nama)}</div>`;
    }).join('');
    return;
  }

  const S = _skala;
  const judul = S.mode === 'z' ? (S.rataSemua ? 'Tinggi dari tanah' : 'Ketinggian Z')
                               : 'Jarak dari sensor';
  const catatan = { linear: 'linear', persentil: 'persentil 2–98%',
                    kuantil: 'kuantil — jarak label tidak sama', manual: 'rentang manual' }[S.sebaran];
  // Label di tiap batas pita bila pitanya ≤ 4 (lebih dari itu labelnya
  // bertumpuk di lebar 256 px); selebihnya lima label rata.
  const titikT = S.tingkat >= 2 && S.tingkat <= 4
    ? Array.from({ length: S.tingkat + 1 }, (_, j) => j / S.tingkat)
    : [0, 0.25, 0.5, 0.75, 1];
  const rentang = Math.abs(S.nilaiDi(1) - S.nilaiDi(0));
  const tik = titikT.map((t, j) => {
    let teks = fmt(S.nilaiDi(t), rentang);
    if (S.terpotong && j === 0) teks = '≤' + teks;
    if (S.terpotong && j === titikT.length - 1) teks = '≥' + teks;
    const posisi = j === 0 ? 'left:0' : j === titikT.length - 1 ? 'right:0'
                  : `left:${(t * 100).toFixed(1)}%;transform:translateX(-50%)`;
    return `<span style="${posisi}">${teks}</span>`;
  }).join('');
  box.innerHTML = `<div class="lg-judul">${judul} (m) <span class="lg-cat">· ${catatan}</span></div>` +
    '<canvas width="256" height="12"></canvas>' + `<div class="lg-tik">${tik}</div>`;
  gambarGradasi(box.querySelector('canvas'), S.palet, S.tingkat);
}

// ============================================================
// init
// ============================================================
export function init() {
  setSkalaTitik(_s.skala);
  setLatar(_s.latar);
  layers.onUbah(terapkan);
  terapkan();
}
