# Spec 1 — Layer multi-berkas + modularisasi frontend

**Tanggal:** 2026-07-31 · **Status:** disetujui, siap direncanakan
**Konteks:** PointCloud Studio saat ini hanya bisa membuka satu berkas. Membandingkan
dua hasil scan ruangan yang sama, atau menyatukan beberapa sweep, berarti menutup dan
membuka ulang berkas satu per satu.

Spec ini adalah **bagian pertama dari dua**. Bagian kedua
(`2026-07-31-grid-ukur-design.md`, ditulis setelah spec ini selesai diimplementasikan)
menambahkan grid referensi dan alat ukur yang lebih baik. Urutan ini dipilih karena
layer mengubah model state inti; membangun grid & alat ukur di atas model yang sudah
benar berarti tiap fungsi ditulis sekali saja.

## Tujuan

1. Membuka beberapa berkas point cloud sekaligus sebagai layer terpisah.
2. Menyembunyikan/menampilkan tiap layer, dan memilih satu layer aktif.
3. Memecah `frontend/app.js` (595 baris) jadi modul-modul berfokus, supaya Spec 2
   punya tempat yang jelas untuk menaruh grid & alat ukur.

## Di luar cakupan (YAGNI)

- **Pewarnaan per layer.** Semua layer memakai warna asli dari berkasnya. Untuk
  membedakan dua scan yang mirip, sembunyikan salah satunya.
- **Transformasi per layer** (geser/putar layer untuk menyelaraskan dua scan).
  Itu registrasi point cloud — masalah tersendiri, bukan bagian dari layer.
- **Menggabung layer jadi satu** atau ekspor gabungan. Ekspor selalu satu layer.
- **Menyusun ulang urutan layer.** Urutan panel = urutan dibuka.
- **Alat tes JavaScript.** Proyek ini belum punya, dan spec ini tidak memasangnya.
  Logika layer diverifikasi manual lewat browser; yang diuji pytest hanya perubahan
  di sisi Python (lihat "Tes").

---

## Model state

`cloud` — satu `Float32Array` global yang jadi sumber kebenaran — diganti daftar layer.
Tiap layer memegang datanya sendiri, objek scene-nya sendiri, dan riwayat undo-nya
sendiri.

```js
// Layer = {
//   id,          nomor urut unik (naik terus, tidak dipakai ulang)
//   nama,        nama berkas, mis. "sweep_1.ply"
//   cloud,       Float32Array [x,y,z,r,g,b]*N  ← sumber kebenaran layer ini
//   points,      THREE.Points di scene
//   terlihat,    bool
//   bounds,      {min:[3], max:[3]}
//   undo: [],    tumpukan Float32Array, maks 8, per-layer
//   ket,         keterangan downsample untuk baris hint
// }
const layers = [];      // urutan tampil di panel = urutan dibuka
let aktifId = null;     // null bila belum ada layer
```

**Undo per-layer, bukan global.** Tombol ↶ Undo membatalkan edit terakhir di layer
aktif. Tumpukan global akan melompat antar berkas dan sulit ditebak.

**Batas undo turun dari 12 jadi 8 per layer.** Tiap entri adalah salinan penuh cloud
layer itu (~24 byte per titik: 400.000 titik ≈ 38 MB). Dengan satu berkas, 12 entri
bisa diterima; dengan beberapa layer yang tiap-tiapnya menyimpan riwayat sendiri,
angkanya berlipat. Delapan cukup untuk membatalkan serangkaian salah-hapus tanpa
membuat tab browser membengkak.

**Nama layer duplikat dibiarkan.** Membuka `sweep_1.ply` dua kali menghasilkan dua
baris bernama sama; yang membedakan adalah `id`, dan panel tidak perlu menomori ulang.

---

## Pemecahan `app.js`

Modul disusun searah — tidak ada impor melingkar. Sebuah modul boleh mengimpor apa pun
yang ada **di bawahnya**, tidak pernah di atasnya (mis. `ui.js` boleh mengimpor
`viewer.js` langsung untuk bidang irisan).

```
app.js       bootstrap: baca ?file=, pasang seret-lepas, panggil ui.init()
  ↓
ui.js        panel Layer, panel Statistik, panel Analisis, irisan Z, wiring toolbar
  ↓
io.js        /load /open /mesh /analyze, ekspor PLY/XYZ
edit.js      box-select hapus/crop, undo
measure.js   ukur jarak 2 titik (diperluas besar-besaran di Spec 2)
  ↓
layers.js    daftar layer, tambah/tutup/aktif/terlihat, bounds & xyzBuffer gabungan
  ↓
viewer.js    renderer, scene, kamera persp/ortho, OrbitControls, gizmo XYZ,
             material, bidang irisan, animate/resize, setView/frameCamera

hud.js       toast + hint — daun, tanpa dependensi, boleh diimpor siapa saja
```

### Menghindari impor melingkar

Dua lingkaran mengintai kalau modul disusun asal:

1. `layers.js` perlu memberi tahu `ui.js` bahwa daftar berubah — tapi `ui.js` sudah
   mengimpor `layers.js`. Diselesaikan dengan langganan, bukan impor balik:

   ```js
   // layers.js
   export function onUbah(fn) { pendengar.push(fn); }
   function beritahu() { pendengar.forEach(f => f()); }
   ```

   `ui.js` memanggil `layers.onUbah(...)` untuk menggambar ulang panel Layer dan
   panel Statistik. `io.js` juga mendaftar, untuk menandai `meshDirty` dan membangun
   ulang mesh bila mode tampilan sedang Mesh.

2. `edit.js` perlu `toast()` dan perlu memicu pembaruan statistik — keduanya di
   `ui.js`, yang mengimpor `edit.js`. Diselesaikan dua arah:
   - `toast`/`setHint` dipindah ke `hud.js`, modul daun tanpa dependensi (isinya
     hanya menyentuh `#toast` dan `#hint`). Siapa pun boleh mengimpornya.
   - `afterEdit` tidak lagi memanggil `updateStats`/`refreshMesh` langsung. Ia
     memanggil `layers.gantiCloud(id, cloudBaru)`, yang membangun ulang geometry,
     menghitung ulang bounds layer itu, lalu `beritahu()`. Pendengar yang sudah
     terdaftar (ui, io) mengurus sisanya.

   Hasil sampingannya: undo, hapus, crop, dan tutup-layer semuanya lewat satu jalur
   yang sama, jadi tidak ada cara memperbarui titik tanpa panel ikut menyusul.

### Isi tiap modul

| Modul | Isi (dipindahkan dari `app.js` kecuali disebut baru) |
|---|---|
| `viewer.js` | renderer, scene, `UP`, `persp`/`ortho`, `controls`, gizmo XYZ + `renderGizmo`, `animate`, `resize`, `circleTexture`, `matPoints`/`matDense`/`matMesh`, `clipLo`/`clipHi`, `frameCamera`, `updateOrthoFrustum`, `setView`, `toggleOrtho`. `frameCamera` menerima bounds sebagai argumen (tadinya membaca global `bounds`). |
| `layers.js` | **baru:** `tambah`, `tutup`, `setAktif`, `setTerlihat`, `aktif()`, `terlihat()`, `onUbah`. **dipindah:** `rebuildPoints` (jadi per-layer), `computeBounds` (per-layer), `xyzBuffer` (jadi gabungan yang terlihat), `boundsGabungan` (baru: union bounds layer terlihat), `applyViewMode`. |
| `edit.js` | `applySelection`, `afterEdit`, `undo`, penangan pointerdown/move/up kotak seleksi. Semua menyasar layer aktif. |
| `measure.js` | `raycaster`, `measurePts`, `drawMeasure`, penangan klik ukur. **berubah:** raycast tidak lagi ke satu objek `points`, melainkan ke `layers.objekTerlihat()` — mengukur ke layer mana pun yang terlihat, bukan hanya layer aktif. Mengukur adalah operasi baca; membatasinya ke layer aktif hanya akan menyulitkan. |
| `io.js` | `terimaTitik`, `loadFile`, `loadFromPath`, `refreshMesh`, `heightColors`, `analyze`, `download`, `exportPLY`, `exportXYZ`. **baru:** `muatBanyak(daftar)` — pemuatan berurutan. |
| `ui.js` | `updateStats`, `renderAnalysis`, `setupSliceRange`, `applySlice`, `sliceZ`, `setMode`, `setSel`, `setMeasure`, `setBtn`, seluruh wiring `document.getElementById(...).onclick`. **baru:** `renderPanelLayer`. |
| `hud.js` | `toast`, `setHint`. Modul daun tanpa dependensi. |
| `app.js` | seret-lepas, baca `?file=`/`?voxel=`/`?full=`, panggil `ui.init()` dan `resize()`. |

Ini refactor murni — perilaku tidak berubah kecuali yang disebut di bagian berikutnya.

---

## Cakupan operasi

| Operasi | Berlaku ke |
|---|---|
| Hapus / crop area | layer **aktif** saja |
| Undo | layer **aktif** saja |
| Ekspor PLY/XYZ | layer **aktif** saja — nama berkas `<nama_layer_tanpa_ekstensi>_edited.ply` |
| Statistik (jumlah, P×L×T) | **gabungan** semua layer terlihat |
| Mesh | **gabungan** semua layer terlihat → satu objek mesh |
| Analisis RANSAC | **gabungan** semua layer terlihat |
| Irisan Z | **global** — rentang slider dari bounds gabungan yang terlihat |
| Mode tampilan (Titik/Padat/Mesh) | **global** |

Alasan pembagiannya: menghapus area adalah operasi merusak, jadi harus jelas sasarannya
— satu layer. Analisis tidak merusak dan justru berguna melintasi beberapa sweep dari
ruangan yang sama, jadi memakai gabungan.

**Kasus tepi:**

- Tidak ada layer aktif (daftar kosong) → tombol hapus/crop/undo/ekspor menampilkan
  toast "Belum ada data" seperti sekarang.
- **Menjadikan layer aktif juga membuatnya terlihat.** Klik nama layer yang sedang
  disembunyikan → layer itu jadi aktif *dan* ◉. Ini menutup sebagian besar jalan
  menuju keadaan "aktif tapi tak terlihat".
- Layer aktif disembunyikan secara sengaja (klik ◉ pada baris yang sedang aktif) →
  hapus/crop **ditolak** dengan toast `Layer aktif sedang disembunyikan`. Kotak
  seleksi bekerja dengan memproyeksikan titik ke layar; kalau titiknya tidak terlihat,
  pemakai tidak bisa melihat apa yang akan terhapus. Undo tetap boleh.
- Semua layer disembunyikan → statistik menampilkan `–`, Mesh & Analisis menolak
  dengan toast "Tidak ada layer yang terlihat".
- Layer aktif ditutup → aktif pindah ke tetangga **di bawahnya** dalam daftar; kalau
  itu baris terakhir, pindah ke atasnya; kalau daftar habis, `aktifId = null` dan
  overlay `#drop` muncul lagi.
- Irisan Z: saat daftar layer berubah, rentang Z gabungan ikut berubah. Posisi slider
  (0–1000) **dipertahankan**, jadi ketinggian potong dihitung ulang terhadap rentang
  baru. Nilai meter di label diperbarui.

Baris hint kiri-bawah selalu menyebut layer aktif supaya tidak salah sasaran:

```
sweep_1.ply · 412.883 titik · aktif
```

---

## Panel Layer

Ditaruh di sidebar paling atas, di atas "Statistik".

```
┌─ Layer ────────────────────────┐
│ ◉  sweep_1.ply    412.883   ✕ │  ← baris tebal = aktif
│ ◉  sweep_2.ply    380.100   ✕ │
│ ○  sweep_3.ply    291.442   ✕ │
└────────────────────────────────┘
```

| Aksi | Hasil |
|---|---|
| klik ◉/○ | sembunyi/tampil layer itu |
| klik nama | jadikan layer aktif (baris jadi tebal) |
| klik ✕ | tutup layer, bebaskan geometry-nya |

Di bawah daftar: tombol **"Sesuaikan pandangan"** — frame ulang kamera ke bounds
gabungan layer yang terlihat.

Saat daftar kosong, panel menampilkan `<div class="empty">Belum ada layer.</div>`,
mengikuti pola panel Analisis yang sudah ada.

### Menambah layer

Tiga jalur, semuanya **menambah** dan tidak pernah menimpa:

- 📂 Buka — `<input type="file">` diberi atribut `multiple`
- seret-lepas — `e.dataTransfer.files` dibaca seluruhnya, bukan `[0]` saja
- `pcs a.ply b.ply` — lihat bagian "`pcs`"

Berkas dimuat **berurutan, bukan serentak**: progres kelihatan di baris hint
(`Memuat 2/3: sweep_2.ply…`), dan satu berkas gagal tidak membatalkan sisanya. Berkas
yang gagal dilaporkan lewat toast; sisanya tetap masuk. Bila beberapa gagal, toast
menyebut jumlahnya: `2 dari 3 berkas gagal dimuat`.

Layer yang baru masuk otomatis jadi layer aktif dan terlihat.

**Kamera hanya di-frame otomatis saat layer pertama masuk** (yaitu saat daftar
sebelumnya kosong). Layer berikutnya tidak menggeser pandangan — kamera meloncat di
tengah pengukuran itu menyebalkan. Gunakan "Sesuaikan pandangan" untuk frame ulang.

---

## Backend & `pcs`

Backend sudah stateless dan tidak perlu tahu soal layer: `/load` dan `/open` dipanggil
sekali per berkas. Dua penyesuaian:

### `pcs.py`

- Argumen `file` jadi `nargs="*"` (dari `nargs="?"`).
- `siapkan_berkas` mengembalikan **daftar** path, bukan satu path. Daftar kosong bila
  tidak ada argumen berkas.
- Semua berkas divalidasi keberadaannya **sebelum** konversi MCAP mana pun dijalankan
  dan sebelum server dinyalakan — supaya salah ketik di berkas ke-3 tidak membuang
  waktu mengonversi berkas ke-1.
- `bangun_url` memakai parameter `file` berulang:
  `/?file=<a>&file=<b>&voxel=0.01`. `urlencode` dipanggil dengan `doseq=True`.
  `quote_via=urllib.parse.quote` tetap dipertahankan (path memuat spasi).
- `--voxel`, `--full`, `--force`, `-t` berlaku untuk semua berkas — tidak per berkas.

Frontend membaca dengan `urlParams.getAll('file')`, lalu memuatnya berurutan.
Satu berkas tetap bekerja seperti sekarang; URL lama `?file=x` tetap sah.

### `/load` ikut di-downsample

Saat ini hanya `/open` yang melewati `downsample.optimize`; berkas yang diunggah atau
diseret dikirim resolusi penuh. Dengan multi-berkas itu jadi masalah nyata — seret tiga
berkas besar dan browser tercekik.

`/load` disamakan dengan `/open`: melewati `downsample.optimize` dengan voxel dari
query (default `0.01`), menerima `full=1`, dan mengirim header `X-Downsample` dengan
bentuk yang sama. Frontend sudah menangani header itu di `terimaTitik`, jadi tidak ada
perubahan di sisi penerima.

Bagian `/load` dan `/open` yang bertumpang tindih (parse → validasi kosong →
downsample → susun Response) diangkat jadi satu penolong `_respons_titik(nama, data,
voxel, full)`; kedua endpoint memanggilnya.

---

## Penanganan error

| Keadaan | Perilaku |
|---|---|
| Satu berkas dari beberapa gagal di-parse | toast merah menyebut nama berkasnya; berkas lain tetap dimuat |
| Semua berkas gagal | toast merah + hint kosong + `#drop` tetap terlihat |
| Berkas tidak ada (argumen `pcs`) | `SystemExit` sebelum server dinyalakan, menyebut path yang salah |
| Ekstensi tidak didukung (argumen `pcs`) | `SystemExit`, pesan sama seperti sekarang |
| Mesh/Analisis tanpa layer terlihat | toast "Tidak ada layer yang terlihat" |

---

## Tes

Pytest yang ada harus tetap hijau tanpa diubah, kecuali yang menyangkut perubahan
`/load` di bawah.

Tambahan:

**`tests/test_load_endpoint.py`** (baru)
- `/load` mengirim header `X-Downsample` dengan bentuk yang sama seperti `/open`
- `/load?full=1` mengembalikan jumlah titik penuh
- `/load` dengan voxel besar mengurangi jumlah titik
- `/load` dengan berkas tanpa titik valid → 400

**`tests/test_pcs.py`** (tambahan)
- `bangun_url` tanpa berkas → URL polos tanpa query
- `bangun_url` satu berkas → satu parameter `file` (bentuk lama tetap sah)
- `bangun_url` banyak berkas → parameter `file` berulang, urutan terjaga
- `bangun_url` dengan path memuat spasi → dikodekan `%20`, bukan `+`
- `siapkan_berkas` tanpa argumen → daftar kosong
- `siapkan_berkas` beberapa `.ply` → daftar path absolut, urutan terjaga
- `siapkan_berkas` dengan berkas ke-2 tidak ada → `SystemExit`, dan konversi MCAP
  tidak pernah dipanggil (dibuktikan lewat monkeypatch pada `konversi_mcap`)
- `jenis_berkas` menolak ekstensi tak didukung — tetap seperti sekarang

Verifikasi manual di browser (tidak otomatis):
buka 3 berkas → sembunyikan satu → statistik ikut berubah → hapus area di layer aktif
→ layer lain utuh → undo → tutup layer aktif → aktif pindah → tutup semua → `#drop`
muncul lagi.

## Stack

Tidak ada dependensi baru. Modul ES dimuat langsung oleh browser lewat
`<script type="module">` seperti sekarang; tidak ada bundler.
