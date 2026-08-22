// Alat ukur: jarak dua titik dan sudut tiga titik.
//
// Klik pertama menancapkan ujung, garis mengikuti kursor dengan angka hidup,
// klik terakhir mengunci. Hasil menetap di scene sampai dihapus — mengukur lagi
// tidak menimpa yang sudah ada.

import * as THREE from 'three';
import { toast, setHint } from './hud.js';
import { renderer, vp, scene, kamera, onFrame } from './viewer.js';
import * as layers from './layers.js';
import * as grid from './grid.js';

const TINGGI_LABEL_PX = 22;
const WARNA_JARAK = 0xffcc44;
const WARNA_SUDUT = 0x4cd6ff;
const WARNA_PRATINJAU = 0xdde4ee;

let _aktif = false;
let _snap = 'titik';      // 'titik' | 'grid'
let _tipe = 'jarak';      // 'jarak' | 'sudut'
let _sedang = [];         // titik yang sudah diklik pada pengukuran berjalan
let _urut = 0;
const _hasil = [];        // { id, tipe, titik:[Vector3], nilai, objek }

const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }
function beritahu() { _pendengar.forEach(f => f()); }

const raycaster = new THREE.Raycaster();

// Hasil ukur tidak menerima clippingPlanes: mengiris ketinggian Z tidak boleh
// memotong pengukuran yang sudah dibuat.
const matJarak = new THREE.LineBasicMaterial({ color: WARNA_JARAK });
const matSudut = new THREE.LineBasicMaterial({ color: WARNA_SUDUT });
const matSorot = new THREE.LineBasicMaterial({ color: 0xffffff });
const matPratinjau = new THREE.LineDashedMaterial({ color: WARNA_PRATINJAU,
  dashSize: 0.08, gapSize: 0.05, transparent: true, opacity: 0.85 });

// ============================================================
// Label — sprite bertekstur canvas, ukuran tetap di layar
// ============================================================
const FONT = 'bold 40px system-ui, "Segoe UI", sans-serif';
const _ukurTeks = document.createElement('canvas').getContext('2d');
const _semuaLabel = [];

function kotakBulat(g, x, y, w, h, r) {
  g.beginPath();
  g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r);
  g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r);
  g.arcTo(x, y, x + w, y, r);
  g.closePath();
}

function buatLabel(teks, warnaHex) {
  _ukurTeks.font = FONT;
  const w = Math.ceil(_ukurTeks.measureText(teks).width) + 34;
  const h = 62;
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const g = c.getContext('2d');
  g.font = FONT;
  g.fillStyle = 'rgba(14,17,22,0.85)';
  kotakBulat(g, 2, 2, w - 4, h - 4, 12); g.fill();
  g.strokeStyle = '#' + warnaHex.toString(16).padStart(6, '0');
  g.lineWidth = 3;
  kotakBulat(g, 2, 2, w - 4, h - 4, 12); g.stroke();
  g.fillStyle = '#ffffff';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(teks, w / 2, h / 2 + 1);

  const sp = new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(c), depthTest: false, transparent: true,
    sizeAttenuation: false }));
  sp.userData.aspek = w / h;
  sp.renderOrder = 10;
  _semuaLabel.push(sp);
  return sp;
}

function buangLabel(sp) {
  const i = _semuaLabel.indexOf(sp);
  if (i >= 0) _semuaLabel.splice(i, 1);
  sp.material.map.dispose();
  sp.material.dispose();
}

// Dengan sizeAttenuation:false, tinggi sprite di layar = scale.y dikali faktor
// yang berbeda antara kamera perspektif dan ortho — dan di ortho ikut berubah
// saat zoom. Jadi dihitung ulang tiap frame, bukan sekali saat dibuat.
onFrame(() => {
  if (!_semuaLabel.length) return;
  const cam = kamera(), H = vp.clientHeight || 1;
  const satuan = cam.isPerspectiveCamera
    ? 2 * Math.tan(THREE.MathUtils.degToRad(cam.fov) / 2) / H
    : (cam.top - cam.bottom) / (cam.zoom * H);
  const t = TINGGI_LABEL_PX * satuan;
  for (const sp of _semuaLabel) sp.scale.set(t * sp.userData.aspek, t, 1);
});

// ============================================================
// Penanda ujung
// ============================================================
const geoBola = new THREE.SphereGeometry(1, 12, 8);
function buatPenanda(p, warna) {
  const m = new THREE.Mesh(geoBola, new THREE.MeshBasicMaterial({ color: warna,
    depthTest: false, transparent: true, opacity: 0.9 }));
  m.position.copy(p);
  m.renderOrder = 9;
  const b = layers.boundsGabungan();
  m.scale.setScalar(b ? Math.max(0.01, diagonal(b) * 0.004) : 0.03);
  return m;
}
function diagonal(b) {
  return Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
}

// ============================================================
// Titik di bawah kursor
// ============================================================
function titikDiKursor(e) {
  const r = vp.getBoundingClientRect();
  const m = new THREE.Vector2(
    ((e.clientX - r.left) / vp.clientWidth) * 2 - 1,
    -((e.clientY - r.top) / vp.clientHeight) * 2 + 1);
  raycaster.setFromCamera(m, kamera());

  if (_snap === 'grid') return grid.titikDiBidang(raycaster);

  raycaster.params.Points.threshold = layers.ambangRaycast();
  const kena = raycaster.intersectObjects(layers.objekTerlihat(), false)[0];
  return kena ? kena.point.clone() : null;
}

// ============================================================
// Nilai terukur
// ============================================================
function sudutDi(a, b, c) {
  const u = new THREE.Vector3().subVectors(a, b);
  const v = new THREE.Vector3().subVectors(c, b);
  // atan2(|u×v|, u·v), bukan acos(u·v): acos kehilangan presisi di dekat 0° dan 180°.
  const silang = new THREE.Vector3().crossVectors(u, v).length();
  return THREE.MathUtils.radToDeg(Math.atan2(silang, u.dot(v)));
}

const nJarak = (d) => `${d.toFixed(3).replace('.', ',')} m`;
const nSudut = (a) => `${a.toFixed(1).replace('.', ',')}°`;

// ============================================================
// Pratinjau (karet)
// ============================================================
let pratinjau = null;

function buangPratinjau() {
  if (!pratinjau) return;
  scene.remove(pratinjau);
  pratinjau.traverse((o) => {
    if (o.isSprite) buangLabel(o);
    else if (o.geometry && o.geometry !== geoBola) o.geometry.dispose();
  });
  pratinjau = null;
}

function gambarPratinjau(kursor) {
  buangPratinjau();
  if (!_sedang.length) return;
  const g = new THREE.Group();

  const warna = _tipe === 'sudut' ? WARNA_SUDUT : WARNA_JARAK;
  for (const p of _sedang) g.add(buatPenanda(p, warna));

  const titik = kursor ? _sedang.concat([kursor]) : _sedang.slice();
  if (titik.length >= 2) {
    const garis = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(titik), matPratinjau);
    garis.computeLineDistances();
    g.add(garis);
  }

  // Label hidup hanya saat ada kursor. Tepat setelah sebuah titik ditancapkan
  // kursornya berimpit dengan titik itu, dan sudutnya akan terbaca 0,0° —
  // angka yang salah dan terlihat seperti alatnya rusak.
  if (kursor) {
    if (_tipe === 'jarak') {
      const d = _sedang[0].distanceTo(kursor);
      const lab = buatLabel(nJarak(d), WARNA_JARAK);
      lab.position.lerpVectors(_sedang[0], kursor, 0.5);
      g.add(lab);
    } else if (titik.length === 3) {
      const a = sudutDi(titik[0], titik[1], titik[2]);
      const lab = buatLabel(nSudut(a), WARNA_SUDUT);
      lab.position.copy(titik[1]);
      g.add(lab);
    }
  }
  pratinjau = g;
  scene.add(g);
}

// ============================================================
// Hasil yang menetap
// ============================================================
function busur(a, b, c, r) {
  const u = new THREE.Vector3().subVectors(a, b).normalize();
  const v = new THREE.Vector3().subVectors(c, b).normalize();
  const n = new THREE.Vector3().crossVectors(u, v);
  if (n.lengthSq() < 1e-12) return null;      // ketiganya segaris
  n.normalize();
  const w = new THREE.Vector3().crossVectors(n, u).normalize();
  const total = Math.atan2(new THREE.Vector3().crossVectors(u, v).length(), u.dot(v));
  const titik = [];
  for (let i = 0; i <= 24; i++) {
    const t = total * i / 24;
    titik.push(new THREE.Vector3()
      .addScaledVector(u, Math.cos(t) * r)
      .addScaledVector(w, Math.sin(t) * r)
      .add(b));
  }
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(titik), matSudut);
}

function kunci() {
  const titik = _sedang.map((p) => p.clone());
  const g = new THREE.Group();
  let nilai, lab;

  if (_tipe === 'jarak') {
    nilai = titik[0].distanceTo(titik[1]);
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(titik), matJarak));
    lab = buatLabel(nJarak(nilai), WARNA_JARAK);
    lab.position.lerpVectors(titik[0], titik[1], 0.5);
    titik.forEach((p) => g.add(buatPenanda(p, WARNA_JARAK)));
    toast(`Jarak: ${nJarak(nilai)}`);
  } else {
    nilai = sudutDi(titik[0], titik[1], titik[2]);
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(titik), matSudut));
    const r = Math.min(titik[1].distanceTo(titik[0]), titik[1].distanceTo(titik[2])) * 0.28;
    const b = busur(titik[0], titik[1], titik[2], r);
    if (b) g.add(b);
    lab = buatLabel(nSudut(nilai), WARNA_SUDUT);
    lab.position.copy(titik[1]);
    titik.forEach((p) => g.add(buatPenanda(p, WARNA_SUDUT)));
    toast(`Sudut: ${nSudut(nilai)}`);
  }

  g.add(lab);
  scene.add(g);
  _hasil.push({ id: ++_urut, tipe: _tipe, titik, nilai, objek: g });
  _sedang = [];
  buangPratinjau();
  setHint(`Terukur ${_tipe === 'jarak' ? nJarak(nilai) : nSudut(nilai)} — klik untuk mulai lagi`);
  beritahu();
}

// ============================================================
// API
// ============================================================
export function measureMode() { return _aktif; }
export function snap() { return _snap; }
export function tipe() { return _tipe; }
export function daftarHasil() { return _hasil.slice(); }

export function setMeasure(on) {
  _aktif = !!on;
  if (!_aktif) batal();
}

export function setSnap(s) {
  _snap = s;
  // Memilih snap grid saat grid mati jelas maunya menyalakan grid, bukan error.
  if (s === 'grid' && !grid.aktifkah()) grid.setAktif(true);
  batal();
  beritahu();
}

export function setTipe(t) { _tipe = t; batal(); beritahu(); }

export function batal() {
  _sedang = [];
  buangPratinjau();
  beritahu();
}

function buangHasil(h) {
  scene.remove(h.objek);
  h.objek.traverse((o) => {
    if (o.isSprite) buangLabel(o);
    else if (o.geometry && o.geometry !== geoBola) o.geometry.dispose();
  });
}

export function hapusHasil(id) {
  const i = _hasil.findIndex((h) => h.id === id);
  if (i < 0) return;
  buangHasil(_hasil[i]);
  _hasil.splice(i, 1);
  beritahu();
}

export function hapusSemua() {
  _hasil.forEach(buangHasil);
  _hasil.length = 0;
  beritahu();
}

/** Sorot satu hasil — dipakai saat kursor menggantung di barisnya. */
export function sorot(id, on) {
  const h = _hasil.find((x) => x.id === id);
  if (!h) return;
  h.objek.traverse((o) => {
    if (!o.isLine) return;
    o.material = on ? matSorot : (h.tipe === 'jarak' ? matJarak : matSudut);
  });
}

/** CSV memakai titik desimal, bukan koma: berkasnya untuk pandas/Excel, dan
 *  koma desimal bertabrakan dengan koma pemisah kolom. */
export function csv() {
  const baris = ['no,tipe,nilai,satuan,x1,y1,z1,x2,y2,z2,x3,y3,z3'];
  _hasil.forEach((h, i) => {
    const k = [];
    for (let j = 0; j < 3; j++) {
      const p = h.titik[j];
      k.push(p ? `${p.x.toFixed(4)},${p.y.toFixed(4)},${p.z.toFixed(4)}` : ',,');
    }
    const nilai = h.tipe === 'jarak' ? h.nilai.toFixed(4) : h.nilai.toFixed(2);
    baris.push(`${i + 1},${h.tipe},${nilai},${h.tipe === 'jarak' ? 'm' : 'deg'},${k.join(',')}`);
  });
  return baris.join('\n');
}

// ============================================================
// Penangan
// ============================================================
function tempatkan(e) {
  if (_snap === 'titik' && !layers.terlihat().length) {
    toast('Tidak ada layer yang terlihat', true);
    return;
  }
  const p = titikDiKursor(e);
  if (!p) {
    toast(_snap === 'grid' ? 'Di luar kotak grid' : 'Tak ada titik di sana — coba zoom');
    return;
  }
  _sedang.push(p);
  const perlu = _tipe === 'jarak' ? 2 : 3;
  if (_sedang.length >= perlu) { kunci(); return; }
  setHint(_tipe === 'jarak'
    ? 'Titik pertama ditancapkan — gerakkan lalu klik ujung kedua'
    : `Titik ${_sedang.length} dari 3 — sudut diukur di titik kedua`);
  gambarPratinjau(null);      // karetnya menyusul saat kursor bergerak
  beritahu();
}

export function pasangPenangan() {
  let turun = null;

  renderer.domElement.addEventListener('pointerdown', (e) => {
    turun = { x: e.clientX, y: e.clientY };
  });

  renderer.domElement.addEventListener('pointerup', (e) => {
    const t = turun; turun = null;
    if (!_aktif || !t) return;
    if (grid.sedangDiseret()) return;      // itu menyeret gizmo grid
    // 'click' juga menyala setelah memutar pandangan; ambang 4 px yang sama
    // dengan kotak seleksi memisahkan klik dari orbit.
    if (Math.hypot(e.clientX - t.x, e.clientY - t.y) > 4) return;
    tempatkan(e);
  });

  renderer.domElement.addEventListener('pointermove', (e) => {
    if (!_aktif || !_sedang.length) return;
    gambarPratinjau(titikDiKursor(e));
  });

  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && _sedang.length) { batal(); setHint(''); }
  });
}
