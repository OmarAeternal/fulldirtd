// Pilih area dengan kotak → hapus atau crop. Menyasar layer aktif saja:
// menghapus titik itu operasi merusak, jadi sasarannya harus tunggal dan jelas.

import * as THREE from 'three';
import { toast } from './hud.js';
import { renderer, vp, kamera, kontrol, clipLo, clipHi } from './viewer.js';
import * as layers from './layers.js';

let _selMode = false;
let _selAction = 'delete';       // delete | crop

export function selMode() { return _selMode; }
export function setSelAction(a) { _selAction = a; }
export function selAction() { return _selAction; }

export function setSel(on) {
  _selMode = on;
  kontrol().enabled = !on;
}

export function undo() {
  const L = layers.aktif();
  if (!L) { toast('Belum ada data', true); return; }
  if (!layers.undoLayer(L.id)) { toast('Tidak ada yang bisa di-undo'); return; }
  toast(`Undo — ${L.nama}`);
}

// ============================================================
// Kotak seleksi
// ============================================================
export function pasangPenangan() {
  const selbox = document.getElementById('selbox');
  let dragging = false, sx = 0, sy = 0;

  renderer.domElement.addEventListener('pointerdown', (e) => {
    if (!_selMode || !layers.aktif()) return;
    dragging = true;
    const r = vp.getBoundingClientRect();
    sx = e.clientX - r.left; sy = e.clientY - r.top;
    selbox.style.display = 'block';
    selbox.style.left = sx + 'px'; selbox.style.top = sy + 'px';
    selbox.style.width = '0px'; selbox.style.height = '0px';
  });

  renderer.domElement.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    const r = vp.getBoundingClientRect();
    const x = e.clientX - r.left, y = e.clientY - r.top;
    selbox.style.left = Math.min(sx, x) + 'px';
    selbox.style.top = Math.min(sy, y) + 'px';
    selbox.style.width = Math.abs(x - sx) + 'px';
    selbox.style.height = Math.abs(y - sy) + 'px';
  });

  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!dragging) return;
    dragging = false;
    selbox.style.display = 'none';
    const r = vp.getBoundingClientRect();
    const ex = e.clientX - r.left, ey = e.clientY - r.top;
    const x0 = Math.min(sx, ex), x1 = Math.max(sx, ex);
    const y0 = Math.min(sy, ey), y1 = Math.max(sy, ey);
    if (x1 - x0 < 4 || y1 - y0 < 4) return;
    terapkanSeleksi(x0, y0, x1, y1);
  });
}

function terapkanSeleksi(x0, y0, x1, y1) {
  const L = layers.aktif();
  if (!L) { toast('Belum ada data', true); return; }
  if (!L.terlihat) {
    // Kotak seleksi bekerja dengan memproyeksikan titik ke layar; kalau
    // titiknya tak terlihat, pemakai tidak bisa melihat apa yang akan terhapus.
    toast('Layer aktif sedang disembunyikan', true);
    return;
  }

  const cam = kamera(), w = vp.clientWidth, h = vp.clientHeight;
  const n = L.cloud.length / 6, v = new THREE.Vector3();
  const zlo = -clipLo.constant, zhi = clipHi.constant;
  const inside = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const z = L.cloud[i * 6 + 2];
    if (z < zlo || z > zhi) { inside[i] = 0; continue; } // hormati irisan
    v.set(L.cloud[i * 6], L.cloud[i * 6 + 1], z).project(cam);
    const px = (v.x * 0.5 + 0.5) * w, py = (-v.y * 0.5 + 0.5) * h;
    inside[i] = (px >= x0 && px <= x1 && py >= y0 && py <= y1 && v.z < 1) ? 1 : 0;
  }

  // keep = (crop ? inside : !inside)
  const keepInside = _selAction === 'crop';
  let keptCount = 0;
  for (let i = 0; i < n; i++) if ((inside[i] === 1) === keepInside) keptCount++;
  if (keptCount === 0) { toast('Semua titik akan terhapus — dibatalkan', true); return; }
  if (keptCount === n) { toast('Tidak ada titik di kotak'); return; }

  const out = new Float32Array(keptCount * 6);
  let j = 0;
  for (let i = 0; i < n; i++) {
    if ((inside[i] === 1) === keepInside) {
      for (let k = 0; k < 6; k++) out[j * 6 + k] = L.cloud[i * 6 + k];
      j++;
    }
  }
  layers.gantiCloud(L.id, out);
  toast(`${(n - keptCount).toLocaleString('id')} titik dihapus dari ${L.nama}`);
}
