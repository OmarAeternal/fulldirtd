// Semua percakapan dengan backend (/load, /open, /mesh, /analyze) dan ekspor
// berkas. Backend stateless — titik terkini selalu dikirim dari sini.

import * as THREE from 'three';
import { toast, setHint } from './hud.js';
import { scene, matMesh, frameCamera } from './viewer.js';
import * as layers from './layers.js';

let meshObj = null;
let meshDirty = true;
export function objekMesh() { return meshObj; }

// Mesh dibangun dari gabungan layer terlihat, jadi perubahan daftar layer
// (tambah, tutup, sembunyikan, hapus area) selalu membuatnya basi.
layers.onUbah(() => { meshDirty = true; });

// Spec 2 memakai bidang RANSAC terakhir untuk memasang grid.
let _bidangTerakhir = [];
const _pendengarAnalisis = [];
export function onAnalisis(fn) { _pendengarAnalisis.push(fn); }
export function bidangTerakhir() { return _bidangTerakhir; }

// ============================================================
// Memuat
// ============================================================
// Bagian yang sama untuk kedua sumber titik: unggahan lewat /load dan berkas
// dari disk lewat /open. Keduanya membalas format yang persis sama — biner
// Float32 [x,y,z,r,g,b]*N dengan statistik di header X-Stats.
async function terimaTitik(res, label) {
  if (!res.ok) {
    const j = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(j.detail || 'gagal');
  }
  const stats = JSON.parse(res.headers.get('X-Stats'));
  const cloud = new Float32Array(await res.arrayBuffer());

  // Kalau titiknya dioptimasi, katakan — jangan biarkan orang mengira ini
  // seluruh data saat sedang mengukur dimensi.
  let ket = '';
  const dsRaw = res.headers.get('X-Downsample');
  if (dsRaw) {
    const ds = JSON.parse(dsRaw);
    if (ds.voxel && ds.n_kirim < ds.n_asli) {
      ket = `voxel ${(ds.voxel * 100).toFixed(1)} cm, dari ${ds.n_asli.toLocaleString('id')}`;
    }
    if (ds.melebihi_batas) {
      toast('Titik masih di atas batas walau voxel sudah maksimum.', true);
    }
  }

  // Kamera hanya di-frame saat layer pertama masuk. Layer berikutnya tidak
  // menggeser pandangan — kamera meloncat di tengah pengukuran itu menyebalkan.
  const pertama = layers.daftar().length === 0;
  layers.tambah({ nama: label, cloud, ket });
  if (pertama) frameCamera();

  meshDirty = true;
  if (layers.modeTampilan() === 'mesh') await bangunMesh();
  toast(`Dimuat: ${stats.n.toLocaleString('id')} titik — ${label}`);
}

export async function muatBerkas(file) {
  const fd = new FormData();
  fd.append('file', file);
  await terimaTitik(await fetch('/load', { method: 'POST', body: fd }), file.name);
}

// Dipakai saat halaman dibuka dengan ?file=… oleh perintah `pcs`.
export async function muatDariPath(path, voxel, full) {
  const q = new URLSearchParams({ path, voxel });
  if (full) q.set('full', '1');
  await terimaTitik(await fetch('/open?' + q.toString()), path.split('/').pop());
}

/** Muat berurutan, bukan Promise.all: progres kelihatan di baris hint, dan
 *  satu berkas gagal tidak membatalkan sisanya. */
export async function muatBanyak(daftar, muatSatu) {
  let gagal = 0;
  for (let i = 0; i < daftar.length; i++) {
    const item = daftar[i];
    const nama = typeof item === 'string' ? item.split('/').pop() : item.name;
    setHint(daftar.length > 1
      ? `Memuat ${i + 1}/${daftar.length}: ${nama}…`
      : 'Memuat & memproses…');
    try {
      await muatSatu(item);
    } catch (e) {
      gagal++;
      toast(`Gagal memuat ${nama}: ${e.message}`, true);
    }
  }
  if (gagal && daftar.length > 1) {
    toast(`${gagal} dari ${daftar.length} berkas gagal dimuat`, true);
  }
  if (gagal && !layers.daftar().length) setHint('');
}

// ============================================================
// Mesh
// ============================================================
export async function bangunMesh() {
  if (!layers.terlihat().length) { toast('Tidak ada layer yang terlihat', true); return; }
  setHint('Membangun mesh…');
  try {
    const res = parseFloat(document.getElementById('meshRes').value) / 10; // 0.2–2.0°
    const r = await fetch(`/mesh?res=${res}&jump=0.25`, {
      method: 'POST', body: layers.xyzBufferGabungan() });
    if (!r.ok) throw new Error((await r.json()).detail || 'gagal');
    const ab = await r.arrayBuffer(); const dv = new DataView(ab);
    const nV = dv.getInt32(0, true), nF = dv.getInt32(4, true);
    const verts = new Float32Array(ab, 8, nV * 3);
    const faces = new Int32Array(ab, 8 + nV * 12, nF * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(verts.slice(), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(warnaKetinggian(verts), 3));
    geo.setIndex(new THREE.BufferAttribute(new Uint32Array(faces), 1));
    geo.computeVertexNormals();
    if (!meshObj) { meshObj = new THREE.Mesh(geo, matMesh); scene.add(meshObj); }
    else { meshObj.geometry.dispose(); meshObj.geometry = geo; }
    meshObj.visible = layers.modeTampilan() === 'mesh';
    meshDirty = false;
    setHint(`Mesh: ${nF.toLocaleString('id')} segitiga`);
  } catch (e) {
    toast('Mesh gagal: ' + e.message, true);
    setHint('');
  }
}

export async function pastikanMesh() {
  if (meshDirty || !meshObj) await bangunMesh();
}

function warnaKetinggian(verts) {
  const n = verts.length / 3; const col = new Float32Array(n * 3);
  let zmin = Infinity, zmax = -Infinity;
  for (let i = 0; i < n; i++) {
    const z = verts[i * 3 + 2];
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
  }
  const span = (zmax - zmin) || 1;
  for (let i = 0; i < n; i++) {
    const t = (verts[i * 3 + 2] - zmin) / span;
    // ramp biru→hijau→kuning
    col[i * 3] = Math.min(1, t * 1.6);
    col[i * 3 + 1] = Math.min(1, 0.4 + t * 0.6);
    col[i * 3 + 2] = 1 - t * 0.8;
  }
  return col;
}

// ============================================================
// Analisis
// ============================================================
export async function analisis(render) {
  if (!layers.terlihat().length) { toast('Tidak ada layer yang terlihat', true); return; }
  const box = document.getElementById('analysis');
  box.innerHTML = '<div class="empty">Menganalisis…</div>';
  try {
    const r = await fetch('/analyze', { method: 'POST', body: layers.xyzBufferGabungan() });
    if (!r.ok) throw new Error((await r.json()).detail || 'gagal');
    const d = await r.json();
    _bidangTerakhir = d.planes || [];
    render(d);
    _pendengarAnalisis.forEach(f => f(_bidangTerakhir));
  } catch (e) {
    box.innerHTML = `<div class="warn">Gagal: ${e.message}</div>`;
  }
}

// ============================================================
// Ekspor — layer aktif saja
// ============================================================
function unduh(name, blob) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

function layerUntukEkspor() {
  const L = layers.aktif();
  if (!L) { toast('Belum ada data', true); return null; }
  return L;
}

function namaDasar(L) { return L.nama.replace(/\.[^.]+$/, ''); }

export function eksporXYZ() {
  const L = layerUntukEkspor(); if (!L) return;
  const c = L.cloud, n = c.length / 6, lines = new Array(n);
  for (let i = 0; i < n; i++) {
    lines[i] = `${c[i * 6].toFixed(4)} ${c[i * 6 + 1].toFixed(4)} ${c[i * 6 + 2].toFixed(4)} ` +
      `${Math.round(c[i * 6 + 3] * 255)} ${Math.round(c[i * 6 + 4] * 255)} ${Math.round(c[i * 6 + 5] * 255)}`;
  }
  const nama = `${namaDasar(L)}_edited.xyz`;
  unduh(nama, new Blob([lines.join('\n')], { type: 'text/plain' }));
  toast('Diekspor: ' + nama);
}

export function eksporPLY() {
  const L = layerUntukEkspor(); if (!L) return;
  const c = L.cloud, n = c.length / 6;
  const header = `ply\nformat binary_little_endian 1.0\ncomment PointCloud Studio\n` +
    `element vertex ${n}\nproperty float x\nproperty float y\nproperty float z\n` +
    `property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n`;
  const hb = new TextEncoder().encode(header);
  const body = new ArrayBuffer(n * 15); const dv = new DataView(body);
  for (let i = 0; i < n; i++) {
    const o = i * 15;
    dv.setFloat32(o, c[i * 6], true);
    dv.setFloat32(o + 4, c[i * 6 + 1], true);
    dv.setFloat32(o + 8, c[i * 6 + 2], true);
    dv.setUint8(o + 12, Math.round(c[i * 6 + 3] * 255));
    dv.setUint8(o + 13, Math.round(c[i * 6 + 4] * 255));
    dv.setUint8(o + 14, Math.round(c[i * 6 + 5] * 255));
  }
  const nama = `${namaDasar(L)}_edited.ply`;
  unduh(nama, new Blob([hb, body], { type: 'application/octet-stream' }));
  toast('Diekspor: ' + nama);
}
