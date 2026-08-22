// Titik masuk: pasang UI, tangani seret-lepas, dan muat berkas yang diminta
// lewat query. Semua logikanya ada di modul-modul di sebelah.

import { setHint } from './hud.js';
import { resize } from './viewer.js';
import * as layers from './layers.js';
import * as io from './io.js';
import * as ui from './ui.js';

ui.init();
resize();

// ============================================================
// Seret & lepas — seluruh berkas yang dijatuhkan, bukan hanya yang pertama
// ============================================================
const drop = document.getElementById('drop');
['dragenter', 'dragover'].forEach(ev => window.addEventListener(ev, (e) => {
  e.preventDefault();
  drop.classList.remove('hide');
  drop.classList.add('over');
}));
['dragleave', 'drop'].forEach(ev => window.addEventListener(ev, (e) => {
  e.preventDefault();
  drop.classList.remove('over');
  if (ev === 'dragleave' && layers.daftar().length) drop.classList.add('hide');
}));
window.addEventListener('drop', (e) => {
  e.preventDefault();
  const f = [...e.dataTransfer.files];
  if (f.length) io.muatBanyak(f, io.muatBerkas);
  else if (layers.daftar().length) drop.classList.add('hide');
});

// ============================================================
// ?file=… dari perintah `pcs`
// ============================================================
// Parameter halaman sengaja bernama `file` (boleh berulang untuk banyak
// berkas), lalu diterjemahkan jadi `path` saat memanggil /open.
const q = new URLSearchParams(location.search);
const berkas = q.getAll('file');
if (berkas.length) {
  const voxel = q.get('voxel') || '0.01';
  const full = q.get('full') === '1';
  io.muatBanyak(berkas, (p) => io.muatDariPath(p, voxel, full));
} else {
  setHint('Tarik file .ply / .xyz untuk mulai');
}
