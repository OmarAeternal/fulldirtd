// Panel sidebar dan wiring toolbar. Satu langganan layers.onUbah() mengurus
// seluruh penyegaran, jadi tidak ada jalur yang bisa mengubah titik tanpa
// panel ikut menyusul.

import * as THREE from 'three';
import { toast, setHint } from './hud.js';
import { clipLo, clipHi, frameCamera, setView, toggleOrtho } from './viewer.js';
import * as layers from './layers.js';
import * as edit from './edit.js';
import * as measure from './measure.js';
import * as grid from './grid.js';
import * as io from './io.js';

const el = (id) => document.getElementById(id);
const setBtn = (id, on) => el(id).classList.toggle('on', on);

// Nama berkas disisipkan lewat innerHTML dan boleh mengandung '<'.
const aman = (s) => String(s).replace(/[&<>"]/g, (c) =>
  ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// ============================================================
// Panel Layer
// ============================================================
function renderPanelLayer() {
  const box = el('layerList');
  const daftar = layers.daftar();
  if (!daftar.length) {
    box.innerHTML = '<div class="empty">Belum ada layer.</div>';
    return;
  }
  box.innerHTML = '';
  const aktifId = layers.aktif()?.id;
  for (const L of daftar) {
    const row = document.createElement('div');
    row.className = 'lyr' + (L.id === aktifId ? ' aktif' : '')
                          + (L.terlihat ? '' : ' mati');
    const judul = L.ket ? `${L.nama} (${L.ket})` : L.nama;
    row.innerHTML =
      `<span class="eye" title="Sembunyikan / tampilkan">${L.terlihat ? '◉' : '○'}</span>` +
      `<span class="nm" title="${aman(judul)}">${aman(L.nama)}</span>` +
      `<span class="cnt">${(L.cloud.length / 6).toLocaleString('id')}</span>` +
      `<span class="x" title="Tutup layer">✕</span>`;
    row.querySelector('.eye').onclick = () => layers.setTerlihat(L.id, !L.terlihat);
    row.querySelector('.nm').onclick = () => layers.setAktif(L.id);
    row.querySelector('.x').onclick = () => layers.tutup(L.id);
    box.appendChild(row);
  }
}

// ============================================================
// Statistik — gabungan layer terlihat
// ============================================================
function updateStats() {
  const b = layers.boundsGabungan();
  if (!b) {
    for (const id of ['sN', 'sX', 'sY', 'sZ']) el(id).textContent = '–';
    return;
  }
  el('sN').textContent = layers.jumlahTitikGabungan().toLocaleString('id');
  el('sX').textContent = (b.max[0] - b.min[0]).toFixed(2) + ' m';
  el('sY').textContent = (b.max[1] - b.min[1]).toFixed(2) + ' m';
  el('sZ').textContent = (b.max[2] - b.min[2]).toFixed(2) + ' m';
}

// ============================================================
// Irisan Z
// ============================================================
// Posisi slider (0–1000) dipertahankan saat daftar layer berubah; yang
// dihitung ulang adalah ketinggian meter yang diwakilinya.
function applySlice() {
  const b = layers.boundsGabungan();
  if (!b) {
    clipLo.constant = 1e9; clipHi.constant = 1e9;
    el('loVal').textContent = '–'; el('hiVal').textContent = '–';
    return;
  }
  const t = (v) => b.min[2] + v * (b.max[2] - b.min[2]);
  const lo = parseInt(el('sliceLo').value) / 1000;
  const hi = parseInt(el('sliceHi').value) / 1000;
  const zlo = t(Math.min(lo, hi)), zhi = t(Math.max(lo, hi));
  clipLo.constant = -zlo;   // z >= zlo
  clipHi.constant = zhi;    // z <= zhi
  el('loVal').textContent = zlo.toFixed(2) + ' m';
  el('hiVal').textContent = zhi.toFixed(2) + ' m';
}

// ============================================================
// Mode tampilan & alat
// ============================================================
async function setMode(m) {
  layers.setModeTampilan(m);
  setBtn('mPoints', m === 'points');
  setBtn('mDense', m === 'dense');
  setBtn('mMesh', m === 'mesh');
  if (m === 'mesh') await io.pastikanMesh();
  const mesh = io.objekMesh();
  if (mesh) mesh.visible = m === 'mesh';
}

// Tiga mode klik saling meniadakan: Pilih area, Ukur, dan Pasang grid 3 titik.
// Grid sendiri (tampil/tidak) berdiri sendiri — boleh menyala di mode apa pun.
function setSel(on) {
  if (on) { setMeasure(false); grid.batal3Titik(); }
  edit.setSel(on);
  setBtn('tSelect', on);
  if (on) setHint('Mode Pilih: tarik kotak di viewport');
  else hintLayer();
}

function setMeasure(on) {
  if (on) { setSel(false); grid.batal3Titik(); }
  measure.setMeasure(on);
  // Gizmo ditahan selama mengukur: lengannya menutupi petak layar yang besar
  // tepat di pusat grid, dan klik di situ akan menggenggam gizmo alih-alih
  // menancapkan titik ukur.
  grid.setGizmoDitahan(on);
  setBtn('tMeasure', on);
  if (on) {
    setHint(measure.tipe() === 'jarak'
      ? 'Mode Ukur: klik untuk menancapkan ujung, klik lagi untuk mengunci'
      : 'Mode Ukur sudut: klik 3 titik, sudut diukur di titik kedua');
  } else hintLayer();
}

function hintLayer() {
  const L = layers.aktif();
  setHint(L ? `${L.nama} · ${(L.cloud.length / 6).toLocaleString('id')} titik · aktif`
            : '');
}

// ============================================================
// Analisis
// ============================================================
function renderAnalysis(d) {
  const w = d.walls, rows = [];
  rows.push(`<div><span class="k">Tinggi (${w.metode_tinggi}):</span> <span class="val good">${w.tinggi_m?.toFixed(2)} m</span></div>`);
  rows.push(`<div><span class="k">Panjang (bbox):</span> <span class="val">${w.panjang_bbox_m?.toFixed(2)} m</span></div>`);
  rows.push(`<div><span class="k">Lebar (bbox):</span> <span class="val">${w.lebar_bbox_m?.toFixed(2)} m</span></div>`);
  rows.push(`<div><span class="k">Bidang terdeteksi:</span> <span class="val">${d.planes.length}</span></div>`);
  if (w.rmse_planaritas_rata_m != null)
    rows.push(`<div><span class="k">RMSE planaritas (rata):</span> <span class="val">${(w.rmse_planaritas_rata_m * 100).toFixed(1)} cm</span></div>`);
  if (w.ortogonalitas && w.ortogonalitas.length) {
    rows.push(`<div class="k" style="margin-top:5px">Ortogonalitas dinding:</div>`);
    w.ortogonalitas.slice(0, 4).forEach(o => {
      const cls = o.deviasi_dari_90 <= 2 ? 'good' : 'warn';
      rows.push(`<div style="padding-left:8px">dinding ${o.pasangan.join('–')}: <span class="${cls}">${o.sudut_deg}°</span> <span class="small">(Δ${o.deviasi_dari_90}°)</span></div>`);
    });
  }
  if (d.notes && d.notes.length) rows.push(`<div class="warn small">${d.notes.join(' ')}</div>`);
  el('analysis').innerHTML = rows.join('');
}

// ============================================================
// Panel Grid
// ============================================================
// Kotak isian dan gizmo adalah dua tampilan dari state yang sama. Penulisan
// kotak isian dari gizmo dipagari `menulisKotak` supaya mengetik angka tidak
// langsung ditimpa balik oleh event yang dipicunya sendiri.
let menulisKotak = false;

function renderPanelGrid() {
  const on = grid.aktifkah();
  setBtn('gridOn', on);
  el('gridOn').textContent = on ? '▦ Grid aktif' : '▦ Aktifkan grid';
  el('gridBody').classList.toggle('mati', !on);

  const m = grid.modeGizmo();
  setBtn('gzMove', m === 'translate');
  setBtn('gzRot', m === 'rotate');
  setBtn('gzOff', m === 'mati');
  el('gzSnap').checked = grid.snapGizmo();
  setBtn('gPasang3', grid.mode3Titik());
  el('gzTahan').style.display =
    (measure.measureMode() && m !== 'mati') ? 'block' : 'none';

  menulisKotak = true;
  const p = grid.posisi(), r = grid.rotasiDeg();
  const isi = (id, v, d) => {
    const box = el(id);
    if (document.activeElement !== box) box.value = v.toFixed(d);
  };
  isi('gPX', p.x, 3); isi('gPY', p.y, 3); isi('gPZ', p.z, 3);
  isi('gRZ', r.z, 1); isi('gRX', r.x, 1); isi('gRY', r.y, 1);
  if (document.activeElement !== el('gUkuran')) el('gUkuran').value = grid.ukuran();
  el('gSpasi').value = String(grid.spasi());
  menulisKotak = false;

  el('gNote').innerHTML = grid.dipangkas()
    ? `<div class="note">Ukuran dipangkas ke ${grid.ukuranEfektif().toFixed(1)} m (batas 400 garis)</div>`
    : '';
}

function bacaKotakGrid() {
  if (menulisKotak) return;
  grid.setPosisi(el('gPX').value, el('gPY').value, el('gPZ').value);
  grid.setRotasiDeg(el('gRZ').value, el('gRX').value, el('gRY').value);
}

function isiBidangRansac(bidang) {
  const s = el('gBidang');
  s.innerHTML = '';
  if (!bidang.length) {
    s.innerHTML = '<option value="">Bidang RANSAC — belum dianalisis</option>';
    return;
  }
  s.innerHTML = '<option value="">Pasang ke bidang RANSAC…</option>' +
    bidang.map((b, i) =>
      `<option value="${i}">${aman(b.kind)} · ${b.n_inliers.toLocaleString('id')} titik` +
      ` · RMSE ${(b.rmse_m * 100).toFixed(1)} cm</option>`).join('');
}

// ============================================================
// Panel Ukuran
// ============================================================
function renderPanelUkuran() {
  setBtn('uSnapTitik', measure.snap() === 'titik');
  setBtn('uSnapGrid', measure.snap() === 'grid');
  setBtn('uJarak', measure.tipe() === 'jarak');
  setBtn('uSudut', measure.tipe() === 'sudut');

  const box = el('ukurList');
  const hasil = measure.daftarHasil();
  if (!hasil.length) {
    box.innerHTML = '<div class="empty">Belum ada ukuran.</div>';
    return;
  }
  box.innerHTML = '';
  hasil.forEach((h, i) => {
    const row = document.createElement('div');
    row.className = 'ukr';
    const nilai = h.tipe === 'jarak'
      ? h.nilai.toFixed(3).replace('.', ',') + ' m'
      : h.nilai.toFixed(1).replace('.', ',') + '°';
    row.innerHTML = `<span class="no">#${i + 1}</span>` +
      `<span class="tp">${h.tipe}</span>` +
      `<span class="vl">${nilai}</span>` +
      `<span class="x" title="Hapus ukuran ini">✕</span>`;
    row.onmouseenter = () => measure.sorot(h.id, true);
    row.onmouseleave = () => measure.sorot(h.id, false);
    row.querySelector('.x').onclick = () => {
      measure.sorot(h.id, false);
      measure.hapusHasil(h.id);
    };
    box.appendChild(row);
  });
}

function unduhCsv() {
  if (!measure.daftarHasil().length) { toast('Belum ada hasil ukur', true); return; }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([measure.csv()], { type: 'text/csv' }));
  a.download = 'ukuran.csv';
  a.click();
  URL.revokeObjectURL(a.href);
  toast('Diekspor: ukuran.csv');
}

// ============================================================
// init
// ============================================================
export function init() {
  edit.pasangPenangan();
  measure.pasangPenangan();
  grid.pasangPenangan();

  layers.onUbah(() => {
    renderPanelLayer();
    updateStats();
    applySlice();
    el('drop').classList.toggle('hide', layers.daftar().length > 0);
    if (!edit.selMode() && !measure.measureMode()) hintLayer();
  });

  el('btnOpen').onclick = () => el('file').click();
  el('btnOpen2').onclick = () => el('file').click();
  el('file').onchange = (e) => {
    const f = [...e.target.files];
    if (f.length) io.muatBanyak(f, io.muatBerkas);
    e.target.value = '';        // supaya berkas yang sama bisa dibuka lagi
  };

  el('mPoints').onclick = () => setMode('points');
  el('mDense').onclick = () => setMode('dense');
  el('mMesh').onclick = () => setMode('mesh');

  el('tSelect').onclick = () => setSel(!edit.selMode());
  el('tMeasure').onclick = () => setMeasure(!measure.measureMode());
  el('btnUndo').onclick = edit.undo;

  el('vTop').onclick = () => setView('top');
  el('vFront').onclick = () => setView('front');
  el('vSide').onclick = () => setView('side');
  el('vOrtho').onclick = toggleOrtho;
  el('btnFrame').onclick = () => {
    if (!layers.boundsGabungan()) { toast('Belum ada layer yang terlihat'); return; }
    frameCamera();
  };

  el('selDelete').onclick = () => {
    edit.setSelAction('delete'); setBtn('selDelete', true); setBtn('selCrop', false);
  };
  el('selCrop').onclick = () => {
    edit.setSelAction('crop'); setBtn('selCrop', true); setBtn('selDelete', false);
  };

  el('sliceLo').oninput = applySlice;
  el('sliceHi').oninput = applySlice;

  el('meshRes').oninput = (e) => {
    el('resVal').textContent = (parseFloat(e.target.value) / 10).toFixed(1) + '°';
  };
  el('meshRes').onchange = () => {
    if (layers.modeTampilan() === 'mesh') io.bangunMesh();
  };

  el('btnAnalyze').onclick = () => io.analisis(renderAnalysis);
  el('expPly').onclick = io.eksporPLY;
  el('expXyz').onclick = io.eksporXYZ;

  // ---- Grid ----
  grid.onUbah(renderPanelGrid);
  const nyalakanGrid = () => { grid.setAktif(!grid.aktifkah()); };
  el('gridOn').onclick = nyalakanGrid;
  el('tGrid').onclick = nyalakanGrid;
  el('gzMove').onclick = () => grid.setModeGizmo('translate');
  el('gzRot').onclick = () => grid.setModeGizmo('rotate');
  el('gzOff').onclick = () => grid.setModeGizmo('mati');
  el('gzSnap').onchange = (e) => grid.setSnapGizmo(e.target.checked);
  el('gSpasi').onchange = (e) => grid.setSpasi(e.target.value);
  el('gUkuran').oninput = (e) => { if (!menulisKotak) grid.setUkuran(e.target.value); };
  for (const id of ['gPX', 'gPY', 'gPZ', 'gRZ', 'gRX', 'gRY']) {
    el(id).oninput = bacaKotakGrid;
  }
  el('gDatar').onclick = grid.datar;
  el('gPusat').onclick = grid.kePusatData;
  el('gPasang3').onclick = () => {
    if (grid.mode3Titik()) { grid.batal3Titik(); return; }
    setSel(false); setMeasure(false);
    grid.mulai3Titik();
  };
  el('gBidang').onchange = (e) => {
    const i = parseInt(e.target.value, 10);
    e.target.value = '';
    const b = io.bidangTerakhir()[i];
    if (!b) return;
    grid.setAktif(true);
    grid.pasangKeBidang(new THREE.Vector3(...b.normal), new THREE.Vector3(...b.centroid));
    toast(`Grid dipasang ke bidang ${b.kind}`);
  };
  io.onAnalisis(isiBidangRansac);

  // ---- Ukuran ----
  measure.onUbah(renderPanelUkuran);
  el('uSnapTitik').onclick = () => measure.setSnap('titik');
  el('uSnapGrid').onclick = () => measure.setSnap('grid');
  el('uJarak').onclick = () => { measure.setTipe('jarak'); if (measure.measureMode()) setMeasure(true); };
  el('uSudut').onclick = () => { measure.setTipe('sudut'); if (measure.measureMode()) setMeasure(true); };
  el('uHapus').onclick = () => {
    if (!measure.daftarHasil().length) { toast('Belum ada hasil ukur'); return; }
    measure.hapusSemua();
    toast('Semua ukuran dihapus');
  };
  el('uCsv').onclick = unduhCsv;

  // Esc: batalkan apa pun yang sedang setengah jalan.
  window.addEventListener('keydown', (e) => {
    if (e.key !== 'Escape') return;
    if (grid.mode3Titik()) { grid.batal3Titik(); hintLayer(); }
    else if (measure.measureMode()) hintLayer();
  });

  renderPanelLayer();
  updateStats();
  applySlice();
  renderPanelGrid();
  renderPanelUkuran();
}
