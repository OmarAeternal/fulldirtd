// Panggung Three.js: renderer, scene, kamera, kontrol orbit, gizmo sumbu,
// material, dan bidang irisan. Tidak tahu apa-apa soal layer atau titik —
// modul di atasnya yang mengisi scene.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const vp = document.getElementById('viewport');

export const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.localClippingEnabled = true;
renderer.autoClear = false;               // perlu untuk overlay gizmo
vp.appendChild(renderer.domElement);

export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);

// Data LiDAR memakai Z sebagai sumbu vertikal → set 'up' = Z agar orbit terasa natural.
export const UP = new THREE.Vector3(0, 0, 1);
const persp = new THREE.PerspectiveCamera(60, 1, 0.01, 5000);
const ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -5000, 5000);
persp.up.copy(UP); ortho.up.copy(UP);
persp.position.set(6, -6, 4);

// Kamera dan kontrol ditukar oleh toggleOrtho, jadi keduanya disediakan lewat
// fungsi — konsumen yang menyimpan referensinya akan memegang yang basi.
let camera = persp;
let controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.screenSpacePanning = true;       // geser mengikuti bidang layar (lebih intuitif)

export function kamera() { return camera; }
export function kontrol() { return controls; }

const pendengarKamera = [];
export function onKameraGanti(fn) { pendengarKamera.push(fn); }

scene.add(new THREE.AmbientLight(0xffffff, 0.65));
const dir = new THREE.DirectionalLight(0xffffff, 0.9);
dir.position.set(5, 10, 7); scene.add(dir);
const dir2 = new THREE.DirectionalLight(0xffffff, 0.4);
dir2.position.set(-5, -3, -6); scene.add(dir2);

// ---- Gizmo sumbu XYZ (pojok kiri bawah), ikut berputar dengan pandangan ----
const gizmoScene = new THREE.Scene();
const gizmoCam = new THREE.PerspectiveCamera(50, 1, 0.1, 10);
gizmoCam.up.copy(UP);
function gizmoArrow(vx, vy, vz, color) {
  return new THREE.ArrowHelper(new THREE.Vector3(vx, vy, vz),
    new THREE.Vector3(0, 0, 0), 1.0, color, 0.32, 0.18);
}
gizmoScene.add(gizmoArrow(1, 0, 0, 0xff5555)); // X merah
gizmoScene.add(gizmoArrow(0, 1, 0, 0x4fce6b)); // Y hijau
gizmoScene.add(gizmoArrow(0, 0, 1, 0x5b8dff)); // Z biru
function gizmoLabel(txt, color, pos) {
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 46px sans-serif'; g.fillStyle = color;
  g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(txt, 32, 34);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(c), depthTest: false, transparent: true }));
  sp.position.copy(pos); sp.scale.setScalar(0.55);
  return sp;
}
gizmoScene.add(gizmoLabel('X', '#ff8080', new THREE.Vector3(1.4, 0, 0)));
gizmoScene.add(gizmoLabel('Y', '#7fe098', new THREE.Vector3(0, 1.4, 0)));
gizmoScene.add(gizmoLabel('Z', '#8fb0ff', new THREE.Vector3(0, 0, 1.4)));

// Clipping planes untuk irisan (pada sumbu Z)
export const clipLo = new THREE.Plane(new THREE.Vector3(0, 0, 1), 1e9);   // z >= lo
export const clipHi = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e9);  // z <= hi
export const clipPlanes = [clipLo, clipHi];

// ============================================================
// Bounds
// ============================================================
// Bounds gabungan layer terlihat disimpan di sini supaya resize(), setView(),
// dan frustum ortho tidak perlu menerimanya sebagai argumen di tiap panggilan.
// layers.js yang menyetelnya tiap kali daftar layer berubah.
let _bounds = null;

export function setBounds(b) { _bounds = b; updateOrthoFrustum(); }
export function bounds() { return _bounds; }

export function pusat(b) {
  return [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2,
          (b.min[2] + b.max[2]) / 2];
}
export function diag(b) {
  return Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
}

// ============================================================
// Materials
// ============================================================
function circleTexture() {
  const s = 64, c = document.createElement('canvas'); c.width = c.height = s;
  const g = c.getContext('2d');
  g.beginPath(); g.arc(s / 2, s / 2, s / 2 - 2, 0, Math.PI * 2);
  g.fillStyle = '#fff'; g.fill();
  return new THREE.CanvasTexture(c);
}
const sprite = circleTexture();

export const matPoints = new THREE.PointsMaterial({ size: 0.02, vertexColors: true,
  sizeAttenuation: true, clippingPlanes: clipPlanes });
export const matDense = new THREE.PointsMaterial({ size: 0.12, vertexColors: true,
  sizeAttenuation: true, map: sprite, alphaTest: 0.5, transparent: false,
  clippingPlanes: clipPlanes });
export const matMesh = new THREE.MeshStandardMaterial({ color: 0x8fa6c4,
  roughness: 0.85, metalness: 0.0, side: THREE.DoubleSide, flatShading: true,
  vertexColors: true, clippingPlanes: clipPlanes });

// ============================================================
// Render loop
// ============================================================
// Label ukur berukuran tetap di layar, jadi skalanya harus dihitung ulang
// tiap kali viewport berubah — measure.js mendaftar lewat sini.
const pendengarResize = [];
export function onResize(fn) { pendengarResize.push(fn); }

// Label ukur berukuran tetap di layar. Skalanya bergantung pada zoom dan jenis
// kamera (rumusnya beda antara perspektif dan ortho), jadi harus dihitung ulang
// tiap frame — bukan sekali saat dibuat.
const pendengarFrame = [];
export function onFrame(fn) { pendengarFrame.push(fn); }

export function resize() {
  const w = vp.clientWidth, h = vp.clientHeight;
  renderer.setSize(w, h);
  persp.aspect = w / h; persp.updateProjectionMatrix();
  updateOrthoFrustum();
  pendengarResize.forEach(f => f(w, h));
}
window.addEventListener('resize', resize);

function renderGizmo() {
  const size = 104, pad = 12;
  const d = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
  gizmoCam.position.copy(d.multiplyScalar(4));
  gizmoCam.up.copy(camera.up);
  gizmoCam.lookAt(0, 0, 0);
  renderer.clearDepth();
  renderer.setScissorTest(true);
  renderer.setViewport(pad, pad, size, size);
  renderer.setScissor(pad, pad, size, size);
  renderer.render(gizmoScene, gizmoCam);
  renderer.setScissorTest(false);
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  pendengarFrame.forEach(f => f());
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, vp.clientWidth, vp.clientHeight);
  renderer.clear();
  renderer.render(scene, camera);
  renderGizmo();
}
animate();

// ============================================================
// Kamera
// ============================================================
export function updateOrthoFrustum() {
  if (!_bounds) return;
  const r = diag(_bounds) / 2 || 1;
  const asp = vp.clientWidth / vp.clientHeight;
  ortho.left = -r * asp; ortho.right = r * asp; ortho.top = r; ortho.bottom = -r;
  ortho.updateProjectionMatrix();
}

export function frameCamera() {
  if (!_bounds) return;
  const c = pusat(_bounds), r = diag(_bounds) / 2 || 1;
  camera.up.copy(UP);
  controls.target.set(c[0], c[1], c[2]);
  camera.position.set(c[0] + r * 1.5, c[1] - r * 1.5, c[2] + r * 1.1);
  updateOrthoFrustum();
  controls.update();
}

export function setView(axis) {
  if (!_bounds) return;
  const c = pusat(_bounds), r = diag(_bounds) || 1;
  camera.up.copy(UP);
  // 'top' diberi sedikit offset agar tidak gimbal-lock tepat di sumbu atas
  const p = { top:   [c[0] + r * 0.001, c[1] - r * 0.001, c[2] + r],
              front: [c[0], c[1] - r, c[2]],
              side:  [c[0] + r, c[1], c[2]] }[axis];
  camera.position.set(p[0], p[1], p[2]);
  controls.target.set(c[0], c[1], c[2]);
  controls.update();
}

export function toggleOrtho() {
  const btn = document.getElementById('vOrtho');
  const useOrtho = camera === persp;
  const oldPos = camera.position.clone(), oldTarget = controls.target.clone();
  camera = useOrtho ? ortho : persp;
  camera.position.copy(oldPos);
  updateOrthoFrustum();
  controls.dispose();
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.copy(oldTarget);
  controls.update();
  btn.classList.toggle('on', useOrtho);
  // Gizmo grid memegang referensi kamera sendiri dan harus ikut ditukar.
  pendengarKamera.forEach(f => f(camera));
}
