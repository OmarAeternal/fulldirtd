// Pesan sekilas (toast) dan baris keterangan di kiri bawah viewport.
//
// Modul daun: tidak mengimpor apa pun, jadi siapa pun boleh mengimpornya tanpa
// membuat lingkaran. edit.js dan io.js sama-sama perlu memberi kabar ke layar,
// dan keduanya diimpor ui.js — kalau toast tinggal di ui.js, lingkarannya
// langsung terbentuk.

let toastT;

export function toast(msg, isErr = false) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = isErr ? 'err' : '';
  el.style.display = 'block';
  clearTimeout(toastT);
  toastT = setTimeout(() => el.style.display = 'none', isErr ? 5000 : 2500);
}

export function setHint(t) { document.getElementById('hint').textContent = t; }
