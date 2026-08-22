// Grid referensi: bidang bergaris yang bisa digeser dan diputar bebas, dipakai
// sebagai acuan pengukuran. Ditempelkan ke lantai atau dinding, lalu alat ukur
// bisa mendarat di bidangnya alih-alih menebak titik di awan yang berisik.
//
// Grid dibangun di bidang XY *lokal* dengan normal +Z lokal, jadi orientasi
// identitas berarti mendatar — cocok dengan data LiDAR yang Z-up.

import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { toast, setHint } from './hud.js';
import { scene, renderer, vp, kamera, kontrol, onKameraGanti, pusat, diag }
  from './viewer.js';
import * as layers from './layers.js';

// Lewat batas ini frame rate anjlok tanpa gambar jadi lebih berguna: spasi 5 cm
// pada ruangan 40 m sudah 800 garis per sumbu.
const MAKS_GARIS = 400;

// Urutan Euler 'ZXY' dipilih supaya "Putar Z" selalu berarti berputar terhadap
// sumbu Z *dunia* — rotasi Z berada paling luar. Setelah grid dimiringkan ke
// dinding, memutar Z tetap berperilaku seperti kompas.
const URUTAN_EULER = 'ZXY';

const grid = new THREE.Group();
grid.visible = false;
scene.add(grid);

let _aktif = false;
let _spasi = 0.1;
let _ukuran = 10;
let _ukuranEfektif = 10;
let _dipangkas = false;
let _modeGizmo = 'translate';
let _snapGizmo = true;
let _menulisUI = false;
let _pernahDiukur = false;   // ukuran bawaan hanya ditebak sekali dari data
let _mode3Titik = false;
let _ditahan = false;        // gizmo disembunyikan sementara (mode Ukur)

let tc = null;
let garisMinor = null, garisMayor = null, sumbuX = null, sumbuY = null;

const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }
function beritahu() { _pendengar.forEach(f => f()); }

// Grid tidak menerima clippingPlanes: saat mengiris ketinggian Z, acuan harus
// tetap terlihat. depthWrite mati supaya garis tidak menutupi titik di belakangnya.
const matMinor = new THREE.LineBasicMaterial({ color: 0x3c4655, transparent: true,
  opacity: 0.6, depthWrite: false });
const matMayor = new THREE.LineBasicMaterial({ color: 0x63748c, transparent: true,
  opacity: 0.95, depthWrite: false });
const matSumbuX = new THREE.LineBasicMaterial({ color: 0xc05555, transparent: true,
  opacity: 0.95, depthWrite: false });
const matSumbuY = new THREE.LineBasicMaterial({ color: 0x46a066, transparent: true,
  opacity: 0.95, depthWrite: false });

// ============================================================
// Geometry
// ============================================================
function buang(o) {
  if (!o) return;
  grid.remove(o);
  o.geometry.dispose();
}

function bangunGeometry() {
  let n = Math.round(_ukuran / _spasi);
  _dipangkas = n > MAKS_GARIS;
  if (_dipangkas) n = MAKS_GARIS;
  if (n < 2) n = 2;
  n = 2 * Math.round(n / 2);            // genap, supaya ada garis tepat di tengah
  _ukuranEfektif = n * _spasi;
  const s = _ukuranEfektif / 2;
  const tengah = n / 2;

  const minor = [], mayor = [];
  for (let i = 0; i <= n; i++) {
    if (i === tengah) continue;         // sumbu digambar terpisah, berwarna
    const t = -s + i * _spasi;
    const arr = ((i - tengah) % 5 === 0) ? mayor : minor;
    arr.push(-s, t, 0, s, t, 0);        // garis sejajar X lokal
    arr.push(t, -s, 0, t, s, 0);        // garis sejajar Y lokal
  }

  buang(garisMinor); buang(garisMayor); buang(sumbuX); buang(sumbuY);

  const seg = (arr, mat) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(arr), 3));
    return new THREE.LineSegments(g, mat);
  };
  garisMinor = seg(minor, matMinor);
  garisMayor = seg(mayor, matMayor);

  // Dua garis tengah berwarna: tanpanya, grid yang sudah diputar tidak bisa
  // dibedakan orientasinya.
  sumbuX = seg([-s, 0, 0, s, 0, 0], matSumbuX);
  sumbuY = seg([0, -s, 0, 0, s, 0], matSumbuY);

  grid.add(garisMinor, garisMayor, sumbuX, sumbuY);
}

// ============================================================
// Gizmo
// ============================================================
function pastikanTC() {
  if (tc) return tc;
  tc = new TransformControls(kamera(), renderer.domElement);
  tc.setSpace('local');   // memutar grid yang sudah miring terasa lebih masuk akal
  tc.attach(grid);
  tc.addEventListener('dragging-changed', (e) => { kontrol().enabled = !e.value; });
  tc.addEventListener('change', () => { if (!_menulisUI) beritahu(); });
  scene.add(tc);
  // toggleOrtho membuang dan membuat ulang kamera; gizmo memegang referensinya
  // sendiri dan akan melenceng kalau tidak ikut ditukar.
  onKameraGanti((cam) => { tc.camera = cam; });
  terapkanSnap();
  return tc;
}

function terapkanGizmo() {
  const t = pastikanTC();
  const hidup = _aktif && _modeGizmo !== 'mati' && !_mode3Titik && !_ditahan;
  t.enabled = hidup;
  t.visible = hidup;
  if (_modeGizmo !== 'mati') t.setMode(_modeGizmo);
}

/** Sembunyikan gizmo sementara tanpa melupakan mode yang dipilih.
 *
 *  Dipakai saat mode Ukur menyala. Lengan gizmo berukuran tetap di layar, jadi
 *  ia menutupi petak dunia yang besar tepat di pusat grid — dan tiap klik di
 *  situ menggenggam gizmo alih-alih menancapkan titik ukur. Menggeser grid dan
 *  mengukur juga tidak mungkin dilakukan bersamaan dengan satu tetikus. Matikan
 *  mode Ukur untuk menyetel grid lagi. */
export function setGizmoDitahan(on) {
  _ditahan = !!on;
  terapkanGizmo();
  beritahu();
}

function terapkanSnap() {
  if (!tc) return;
  tc.setTranslationSnap(_snapGizmo ? _spasi : null);
  tc.setRotationSnap(_snapGizmo ? THREE.MathUtils.degToRad(15) : null);
}

/** True saat gizmo sedang dipegang — measure.js memakainya supaya menyeret
 *  gizmo tidak sekalian menancapkan titik ukur.
 *
 *  Sengaja hanya `dragging`, bukan `axis !== null`. `axis` sudah terisi begitu
 *  kursor sekadar *melintas* di atas gizmo, dan gizmo menempati sepetak layar
 *  tepat di pusat grid — persis tempat orang sering ingin mengukur. Memakai
 *  `axis` membuat klik di sekitar situ tertelan diam-diam.
 *
 *  `dragging` disetel TransformControls pada pointerdown, dan penangan
 *  measure.js terpasang lebih dulu sehingga membacanya sebelum direset di
 *  pointerup — jadi klik yang benar-benar mengenai gizmo tetap tersaring. */
export function sedangDiseret() {
  return !!tc && tc.dragging;
}

// ============================================================
// Aktif & parameter
// ============================================================
export function aktifkah() { return _aktif; }

export function setAktif(on) {
  _aktif = !!on;
  grid.visible = _aktif;
  if (_aktif && !garisMinor) {
    bangunGeometry();
    kePusatData();
  }
  terapkanGizmo();
  beritahu();
}

export function spasi() { return _spasi; }
export function setSpasi(m) {
  _spasi = Math.max(0.01, +m || 0.1);
  bangunGeometry();
  terapkanSnap();
  beritahu();
}

export function ukuran() { return _ukuran; }
export function ukuranEfektif() { return _ukuranEfektif; }
export function dipangkas() { return _dipangkas; }
export function setUkuran(m) {
  _ukuran = Math.min(200, Math.max(1, +m || 10));
  bangunGeometry();
  beritahu();
}

export function modeGizmo() { return _modeGizmo; }
export function setModeGizmo(m) { _modeGizmo = m; terapkanGizmo(); beritahu(); }
export function snapGizmo() { return _snapGizmo; }
export function setSnapGizmo(on) { _snapGizmo = !!on; terapkanSnap(); beritahu(); }

// ============================================================
// Transform
// ============================================================
export function posisi() {
  return { x: grid.position.x, y: grid.position.y, z: grid.position.z };
}

export function setPosisi(x, y, z) {
  _menulisUI = true;
  grid.position.set(+x || 0, +y || 0, +z || 0);
  _menulisUI = false;
  beritahu();
}

const _euler = new THREE.Euler();
export function rotasiDeg() {
  _euler.setFromQuaternion(grid.quaternion, URUTAN_EULER);
  const d = THREE.MathUtils.radToDeg;
  return { z: d(_euler.z), x: d(_euler.x), y: d(_euler.y) };
}

export function setRotasiDeg(z, x, y) {
  _menulisUI = true;
  const r = THREE.MathUtils.degToRad;
  grid.quaternion.setFromEuler(
    new THREE.Euler(r(+x || 0), r(+y || 0), r(+z || 0), URUTAN_EULER));
  _menulisUI = false;
  beritahu();
}

export function datar() {
  _menulisUI = true;
  grid.quaternion.identity();
  _menulisUI = false;
  beritahu();
}

export function kePusatData() {
  const b = layers.boundsGabungan();
  if (!b) { beritahu(); return; }
  const c = pusat(b);
  _menulisUI = true;
  grid.position.set(c[0], c[1], c[2]);
  _menulisUI = false;
  // Ukuran bawaan mengikuti data: cukup besar untuk menutupi denahnya, tapi
  // tidak sampai membanjiri layar.
  if (!_pernahDiukur) {
    const dx = b.max[0] - b.min[0], dy = b.max[1] - b.min[1];
    setUkuran(Math.min(40, Math.max(4, Math.ceil(Math.max(dx, dy)))));
    _pernahDiukur = true;
  }
  beritahu();
}

const _Z = new THREE.Vector3(0, 0, 1);
/** Tempelkan grid ke bidang: normal (tidak harus ternormalisasi) + satu titik. */
export function pasangKeBidang(normal, titik) {
  const n = normal.clone();
  if (n.lengthSq() < 1e-12) return false;
  n.normalize();
  // Balik normal bila membelakangi kamera, supaya sisi terang menghadap penonton.
  const keKamera = new THREE.Vector3().subVectors(kamera().position, titik);
  if (n.dot(keKamera) < 0) n.negate();
  _menulisUI = true;
  grid.position.copy(titik);
  grid.quaternion.setFromUnitVectors(_Z, n);
  _menulisUI = false;
  beritahu();
  return true;
}

// ============================================================
// Perpotongan sinar dengan bidang grid
// ============================================================
// Analitis, bukan raycast ke mesh: lebih tepat dan tidak bergantung pada
// visibilitas objek pembantu.
const _bidang = new THREE.Plane();
const _normal = new THREE.Vector3();
const _kena = new THREE.Vector3();
const _lokal = new THREE.Vector3();

export function titikDiBidang(raycaster) {
  if (!_aktif) return null;
  _normal.copy(_Z).applyQuaternion(grid.quaternion);
  _bidang.setFromNormalAndCoplanarPoint(_normal, grid.position);
  if (!raycaster.ray.intersectPlane(_bidang, _kena)) return null;  // sinar sejajar
  _lokal.copy(_kena);
  grid.worldToLocal(_lokal);
  const s = _ukuranEfektif / 2;
  if (Math.abs(_lokal.x) > s || Math.abs(_lokal.y) > s) return null;  // di luar kotak
  return _kena.clone();
}

// ============================================================
// Pasang di 3 titik
// ============================================================
let _titik3 = [];
const _ray3 = new THREE.Raycaster();

export function mode3Titik() { return _mode3Titik; }

export function mulai3Titik() {
  if (!layers.terlihat().length) { toast('Tidak ada layer yang terlihat', true); return; }
  if (!_aktif) setAktif(true);
  _mode3Titik = true;
  _titik3 = [];
  terapkanGizmo();
  setHint('Pasang grid: klik titik 1 dari 3 di permukaan yang jadi acuan');
  beritahu();
}

export function batal3Titik() {
  if (!_mode3Titik) return;
  _mode3Titik = false;
  _titik3 = [];
  terapkanGizmo();
  setHint('');
  beritahu();
}

function klik3Titik(e) {
  const r = vp.getBoundingClientRect();
  const m = new THREE.Vector2(
    ((e.clientX - r.left) / vp.clientWidth) * 2 - 1,
    -((e.clientY - r.top) / vp.clientHeight) * 2 + 1);
  _ray3.setFromCamera(m, kamera());
  _ray3.params.Points.threshold = layers.ambangRaycast();
  // Pemasangan selalu memakai titik cloud, apa pun snap yang dipilih di alat
  // ukur — memasang grid ke bidangnya sendiri tidak ada gunanya.
  const kena = _ray3.intersectObjects(layers.objekTerlihat(), false)[0];
  if (!kena) { toast('Tak ada titik di sana — coba zoom'); return; }

  _titik3.push(kena.point.clone());
  if (_titik3.length < 3) {
    setHint(`Pasang grid: klik titik ${_titik3.length + 1} dari 3`);
    return;
  }

  const [a, b, c] = _titik3;
  const normal = new THREE.Vector3()
    .crossVectors(new THREE.Vector3().subVectors(b, a),
                  new THREE.Vector3().subVectors(c, a));
  if (normal.length() < 1e-6) {
    toast('Tiga titik terlalu segaris — pilih titik yang lebih menyebar', true);
    _titik3 = [];
    setHint('Pasang grid: klik titik 1 dari 3');
    return;
  }
  const tengah = new THREE.Vector3().add(a).add(b).add(c).multiplyScalar(1 / 3);
  pasangKeBidang(normal, tengah);
  _mode3Titik = false;
  _titik3 = [];
  terapkanGizmo();
  setHint('');
  toast('Grid dipasang di bidang tiga titik');
  beritahu();
}

// ============================================================
// Penangan
// ============================================================
export function pasangPenangan() {
  let turun = null;
  renderer.domElement.addEventListener('pointerdown', (e) => {
    turun = { x: e.clientX, y: e.clientY };
  });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!_mode3Titik || !turun) { turun = null; return; }
    const jauh = Math.hypot(e.clientX - turun.x, e.clientY - turun.y);
    turun = null;
    if (jauh > 4) return;          // itu memutar pandangan, bukan mengklik
    klik3Titik(e);
  });
}

export function objek() { return grid; }
