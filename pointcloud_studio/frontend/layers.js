// Daftar layer — pengganti `cloud` global yang dulu jadi sumber kebenaran.
//
// Tiap layer memegang cloud-nya sendiri, objek scene-nya sendiri, dan tumpukan
// undo-nya sendiri. Semua perubahan titik lewat gantiCloud(), yang menyiarkan
// onUbah() — jadi tidak ada cara mengubah titik tanpa panel ikut menyusul.

import * as THREE from 'three';
import { scene, matPoints, matDense, setBounds, diag } from './viewer.js';

// Tiap entri undo adalah salinan penuh cloud layer (~24 byte per titik: 400
// ribu titik ≈ 38 MB). Dulu 12 untuk satu berkas; sekarang tiap layer punya
// tumpukan sendiri, jadi angkanya berlipat. Delapan cukup untuk membatalkan
// serangkaian salah-hapus tanpa membuat tab membengkak.
const MAKS_UNDO = 8;

const _layers = [];
let _aktifId = null;
let _urutan = 0;
let _mode = 'points';          // points | dense | mesh

const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }

function beritahu() {
  setBounds(boundsGabungan());
  _pendengar.forEach(f => f());
}

function _cari(id) { return _layers.find(L => L.id === id) || null; }

// ============================================================
// Geometry & bounds
// ============================================================
function bangunGeometry(cloud) {
  const n = cloud.length / 6;
  const pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    pos[i * 3] = cloud[i * 6];     pos[i * 3 + 1] = cloud[i * 6 + 1]; pos[i * 3 + 2] = cloud[i * 6 + 2];
    col[i * 3] = cloud[i * 6 + 3]; col[i * 3 + 1] = cloud[i * 6 + 4]; col[i * 3 + 2] = cloud[i * 6 + 5];
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  return geo;
}

function hitungBounds(cloud) {
  const n = cloud.length / 6;
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < n; i++) {
    for (let k = 0; k < 3; k++) {
      const v = cloud[i * 6 + k];
      if (v < mn[k]) mn[k] = v;
      if (v > mx[k]) mx[k] = v;
    }
  }
  return { min: mn, max: mx };
}

// ============================================================
// Tampilan
// ============================================================
function terapkanTampilan() {
  for (const L of _layers) {
    L.points.material = _mode === 'dense' ? matDense : matPoints;
    L.points.visible = L.terlihat && _mode !== 'mesh';
  }
}

export function setModeTampilan(m) { _mode = m; terapkanTampilan(); }
export function modeTampilan() { return _mode; }

// ============================================================
// CRUD
// ============================================================
export function tambah({ nama, cloud, ket = '' }) {
  const L = {
    id: ++_urutan,
    nama,
    cloud,
    ket,
    terlihat: true,
    bounds: hitungBounds(cloud),
    undo: [],
    points: new THREE.Points(bangunGeometry(cloud), matPoints),
  };
  scene.add(L.points);
  _layers.push(L);
  _aktifId = L.id;
  terapkanTampilan();
  beritahu();
  return L.id;
}

export function tutup(id) {
  const i = _layers.findIndex(L => L.id === id);
  if (i < 0) return;
  const L = _layers[i];
  scene.remove(L.points);
  L.points.geometry.dispose();
  _layers.splice(i, 1);
  if (_aktifId === id) {
    // pindah ke tetangga di bawahnya; kalau itu baris terakhir, ke atasnya
    const gantinya = _layers[i] || _layers[i - 1] || null;
    _aktifId = gantinya ? gantinya.id : null;
    if (gantinya) gantinya.terlihat = true;
  }
  terapkanTampilan();
  beritahu();
}

export function setAktif(id) {
  const L = _cari(id);
  if (!L) return;
  _aktifId = id;
  L.terlihat = true;          // menjadikan aktif juga menampilkannya
  terapkanTampilan();
  beritahu();
}

export function setTerlihat(id, on) {
  const L = _cari(id);
  if (!L) return;
  L.terlihat = !!on;
  terapkanTampilan();
  beritahu();
}

/** Satu-satunya jalan mengubah titik sebuah layer. */
export function gantiCloud(id, cloudBaru, { simpanUndo = true } = {}) {
  const L = _cari(id);
  if (!L) return;
  if (simpanUndo) {
    L.undo.push(L.cloud);
    if (L.undo.length > MAKS_UNDO) L.undo.shift();
  }
  L.cloud = cloudBaru;
  L.bounds = hitungBounds(cloudBaru);
  L.points.geometry.dispose();
  L.points.geometry = bangunGeometry(cloudBaru);
  beritahu();
}

export function undoLayer(id) {
  const L = _cari(id);
  if (!L || !L.undo.length) return false;
  gantiCloud(id, L.undo.pop(), { simpanUndo: false });
  return true;
}

// ============================================================
// Pembacaan
// ============================================================
export function daftar() { return _layers.slice(); }
export function aktif() { return _cari(_aktifId); }
export function terlihat() { return _layers.filter(L => L.terlihat); }
export function objekTerlihat() { return terlihat().map(L => L.points); }

/** Ambang raycast titik, ikut skala data: nilai tetap 0,05 m terlalu besar
 *  untuk ruangan kecil dan terlalu kecil untuk gedung. */
export function ambangRaycast() {
  const b = boundsGabungan();
  return b ? Math.max(0.02, diag(b) * 0.004) : 0.05;
}

export function jumlahTitikGabungan() {
  return terlihat().reduce((s, L) => s + L.cloud.length / 6, 0);
}

export function boundsGabungan() {
  const v = terlihat();
  if (!v.length) return null;
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (const L of v) {
    for (let k = 0; k < 3; k++) {
      if (L.bounds.min[k] < mn[k]) mn[k] = L.bounds.min[k];
      if (L.bounds.max[k] > mx[k]) mx[k] = L.bounds.max[k];
    }
  }
  return { min: mn, max: mx };
}

/** Float32 [x,y,z]*N dari semua layer terlihat — untuk /mesh dan /analyze. */
export function xyzBufferGabungan() {
  const v = terlihat();
  const total = v.reduce((s, L) => s + L.cloud.length / 6, 0);
  const out = new Float32Array(total * 3);
  let j = 0;
  for (const L of v) {
    const n = L.cloud.length / 6;
    for (let i = 0; i < n; i++) {
      out[j * 3] = L.cloud[i * 6];
      out[j * 3 + 1] = L.cloud[i * 6 + 1];
      out[j * 3 + 2] = L.cloud[i * 6 + 2];
      j++;
    }
  }
  return out;
}
