// Tab panel kanan (Alat | View) dan isian tab View. Semua keadaan tinggal di
// warna.js; di sini hanya membaca isian dan menampilkan keadaan itu kembali.

import * as warna from './warna.js';
import { NAMA_PALET } from './palet.js';

const el = (id) => document.getElementById(id);

const KET_SEBARAN = {
  persentil: 'Mengabaikan 2% luas terbawah dan teratas (titik nyasar), sisanya dibagi ' +
             'rata. Dihitung per luas, jadi badan drone yang rapat tidak menarik skala.',
  linear: 'Dibagi rata dari titik terendah ke tertinggi. Satu titik nyasar bisa ' +
          'memampatkan semua warna ke satu ujung.',
  kuantil: 'Tiap warna menutupi luas yang sama: kontras paling tinggi. ' +
           'Baca nilainya dari legenda — jarak antarlabel tidak sama.',
  manual: 'Isi rentangnya sendiri, supaya beberapa scan memakai skala yang sama ' +
          'saat dibandingkan.',
};

function pilihTab(nama) {
  for (const t of ['Alat', 'View']) {
    el('tab' + t).classList.toggle('hide', t !== nama);
    el('tabBtn' + t).classList.toggle('on', t === nama);
  }
  try { localStorage.setItem('pcs.tab', nama); } catch { /* abaikan */ }
}

function render() {
  const s = warna.state();
  el('vMode').value = s.mode;
  el('vPalet').value = s.palet;
  el('vSebaran').value = s.sebaran;
  el('vSebaranKet').textContent = KET_SEBARAN[s.sebaran];
  const tingkat = s.tingkat >= 2 ? s.tingkat : 1;
  el('vTingkat').value = tingkat;
  el('vTingkatVal').textContent = tingkat >= 2 ? `${tingkat} tingkat` : 'halus';
  el('vTunggal').value = s.tunggal;
  el('vLegenda').checked = s.legenda;
  el('vSkala').value = s.skala;
  el('vSkalaVal').textContent = (+s.skala).toFixed(1).replace('.', ',') + '×';
  for (const id of ['vLo', 'vHi']) {
    if (document.activeElement !== el(id)) el(id).value = s[id === 'vLo' ? 'lo' : 'hi'];
  }

  const gradasi = warna.pakaiGradasi(s.mode);
  for (const n of document.querySelectorAll('[data-untuk]')) {
    const u = n.dataset.untuk;
    const tampil = u === 'gradasi' ? gradasi
                 : u === 'manual' ? gradasi && s.sebaran === 'manual'
                 : u === 'tunggal' ? s.mode === 'tunggal' : true;
    n.classList.toggle('hide', !tampil);
  }
  warna.gambarGradasi(el('vPratinjau'), s.palet, s.tingkat);

  el('vLatar').value = s.latar;
  for (const b of el('vLatarPilih').children) {
    b.classList.toggle('on', b.dataset.warna === s.latar.toLowerCase());
  }
}

export function init() {
  el('tabBtnAlat').onclick = () => pilihTab('Alat');
  el('tabBtnView').onclick = () => pilihTab('View');
  let tab = 'Alat';
  try { tab = localStorage.getItem('pcs.tab') === 'View' ? 'View' : 'Alat'; } catch { /* abaikan */ }
  pilihTab(tab);

  el('vPalet').innerHTML = Object.entries(NAMA_PALET)
    .map(([k, v]) => `<option value="${k}">${v}</option>`).join('');

  el('vMode').onchange = (e) => {
    const s = warna.state();
    // Rentang manual bersatuan mode lama (tinggi ≠ jarak). Isi ulang dari
    // persentil mode baru supaya tidak mewarisi angka yang tak bermakna.
    if (s.sebaran === 'manual' && warna.pakaiGradasi(e.target.value) && e.target.value !== s.mode) {
      warna.set({ mode: e.target.value, sebaran: 'persentil' });
      const sk = warna.skalaTerakhir();
      warna.set(sk ? { sebaran: 'manual', lo: +sk.nilaiDi(0).toFixed(2), hi: +sk.nilaiDi(1).toFixed(2) }
                   : { sebaran: 'manual' });
    } else {
      warna.set({ mode: e.target.value });
    }
  };
  el('vPalet').onchange = (e) => warna.set({ palet: e.target.value });
  el('vSebaran').onchange = (e) => {
    const ubah = { sebaran: e.target.value };
    // Rentang manual dimulai dari rentang yang sedang tampil, bukan angka acak.
    const sk = warna.skalaTerakhir();
    if (ubah.sebaran === 'manual' && sk) {
      ubah.lo = +sk.nilaiDi(0).toFixed(2);
      ubah.hi = +sk.nilaiDi(1).toFixed(2);
    }
    warna.set(ubah);
  };
  const bacaRentang = () => {
    const lo = parseFloat(el('vLo').value), hi = parseFloat(el('vHi').value);
    if (Number.isFinite(lo) && Number.isFinite(hi) && hi > lo) warna.set({ lo, hi });
  };
  el('vLo').onchange = bacaRentang;
  el('vHi').onchange = bacaRentang;
  // Slider tingkat mewarnai ulang semua titik; `input` cukup ringan untuk
  // ratusan ribu titik, jadi hasilnya terlihat sambil menggeser.
  el('vTingkat').oninput = (e) => {
    const k = parseInt(e.target.value, 10);
    warna.set({ tingkat: k >= 2 ? k : 0 });
  };
  el('vTunggal').oninput = (e) => warna.set({ tunggal: e.target.value });
  el('vLegenda').onchange = (e) => warna.set({ legenda: e.target.checked });
  // Tombol latar: kotak berwarna latarnya sendiri, teks dengan kontras terbalik.
  el('vLatarPilih').innerHTML = warna.LATAR.map(([hex, nama]) => {
    const terang = parseInt(hex.slice(1, 3), 16) > 128;
    return `<button data-warna="${hex}" title="${nama}" ` +
           `style="background:${hex};color:${terang ? '#111' : '#ddd'}">${nama}</button>`;
  }).join('');
  for (const b of el('vLatarPilih').children) {
    b.onclick = () => warna.set({ latar: b.dataset.warna });
  }
  el('vLatar').oninput = (e) => warna.set({ latar: e.target.value });
  el('vSkala').oninput = (e) => warna.set({ skala: parseFloat(e.target.value) });
  el('vBawaan').onclick = warna.kembalikan;

  warna.onUbah(render);
  warna.init();
  render();
}
