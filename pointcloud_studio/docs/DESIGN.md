# PointCloud Studio — Dokumen Desain

**Tanggal:** 2026-07-23 · **Status:** disetujui, dalam implementasi
**Konteks:** alat bantu skripsi 3D LiDAR Scanner — eksplorasi & pembersihan point cloud hasil scan ruangan.

## Tujuan
Aplikasi lokal untuk menjelajahi hasil scan (PLY/XYZ) dengan lebih nyaman dari CloudCompare:
lihat titik/padat/mesh, blok-hapus area (mis. atap), ukur jarak, iris ketinggian, ekspor hasil,
dan analisis dimensi ruangan otomatis (terhubung ke tabel validasi Kategori A & B).

## Pendekatan (C — Hybrid)
- **Frontend (browser, Three.js):** rendering & interaksi real-time — putar/zoom, box-select, ukur, iris, statistik, ekspor.
- **Backend (Python, FastAPI + Open3D):** komputasi berat — parsing file, meshing berbasis-sudut, RANSAC dimensi/planaritas/ortogonalitas.
- **Komunikasi:** HTTP di `localhost:8000`; titik dikirim sebagai biner Float32 (efisien untuk ratusan ribu titik).
- **Isolasi:** folder sendiri `pointcloud_studio/` dengan venv sendiri; tidak menyentuh `ros2_ws`.
- **Offline:** Three.js di-vendor lokal setelah unduhan pertama.

## Arsitektur
```
frontend/  (modul ES, tanpa bundler)      ──HTTP──►  backend/server.py (FastAPI)
  app.js    bootstrap                                ├─ loader.py   parse PLY/XYZ → Nx6 (xyz+rgb)
   └ ui.js    panel + toolbar                        ├─ downsample.py optimasi voxel
      ├ io.js     panggilan backend + ekspor         ├─ mesh.py     meshing sudut (depth-map)
      ├ edit.js   box-select hapus/crop              └─ analysis.py Open3D RANSAC → dimensi, RMSE
      ├ grid.js   grid referensi + gizmo
      ├ measure.js alat ukur (memakai grid.js)
      └ layers.js daftar layer  ← sumber kebenaran titik
         └ viewer.js Three.js: kamera, kontrol, material
  hud.js  toast + hint (daun, tanpa dependensi)
run.sh: venv + deps + uvicorn + buka browser
```
Modul disusun searah — sebuah modul hanya mengimpor yang ada di bawahnya. `layers.js`
tidak mengimpor `ui.js`; ia menyiarkan `onUbah()` dan `ui.js` mendaftar ke situ.

## Endpoint API
| Metode | Path | Fungsi |
|---|---|---|
| GET | `/` | sajikan frontend |
| POST | `/load` | upload file → parse → optimasi voxel → titik (biner) + statistik |
| GET | `/open` | baca path di disk → sama persis dengan `/load` (dipakai `pcs`) |
| GET | `/versi` | PID + umur kode yang dimuat proses ini (dipakai `pcs` untuk deteksi server basi) |
| POST | `/mesh` | terima titik (biner) + param → kembalikan vertices+faces |
| POST | `/analyze` | terima titik → RANSAC → dimensi ruangan, RMSE planaritas, sudut antar-dinding |

Backend **stateless**: frontend adalah sumber kebenaran titik (yang sudah diedit); untuk mesh/analyze,
titik terkini dikirim ke backend.

## Meshing berbasis-sudut (mesh.py)
Memanfaatkan scanner diam di satu titik (origin = sensor):
1. Tiap titik → koordinat bola (r, azimut, elevasi) dari origin.
2. Petakan ke grid (azimut × elevasi), resolusi default ~0.5° (dapat diatur); tiap sel simpan titik terdekat.
3. Sambung sel bertetangga → 2 segitiga per kotak.
4. **Lewati** kotak bila lompatan kedalaman > ambang atau sel kosong → celah dibiarkan bolong (jujur, tak mengarang permukaan).

## Fitur frontend
Layer multi-berkas (hide/show, layer aktif) · Titik/Padat/Mesh (toggle) · box-select hapus/crop
(+undo) · grid referensi yang bisa digeser/diputar (gizmo + panel numerik, pasang 3 titik,
pasang ke bidang RANSAC) · ukur jarak & sudut dengan pratinjau karet, snap titik/grid,
hasil menetap + ekspor CSV · slider iris Z · panel statistik live (jumlah, P×L×T) ·
tombol Analisis Ruangan (RANSAC) · ekspor PLY/XYZ.

**Grid & ukur.** Grid adalah `THREE.Group` bergaris di bidang XY lokal; transformnya
digerakkan `TransformControls` (vendored) dan kotak isian numerik yang saling menulis
lewat flag anti-lingkaran. Rotasi disimpan quaternion, ditampilkan Euler urutan `ZXY`
supaya "Putar Z" selalu terhadap Z dunia. Perpotongan sinar dengan bidang grid dihitung
analitis, bukan raycast ke mesh. Alat ukur memakai `pointerdown`/`pointerup` dengan
ambang 4 px — `click` juga menyala setelah orbit. Label memakai sprite
`sizeAttenuation:false` yang diskalakan tiap frame, karena rumus tinggi-layar berbeda
antara kamera perspektif dan ortho. Sudut dihitung `atan2(|u×v|, u·v)`, bukan `acos`.
Grid dan hasil ukur sengaja tanpa `clippingPlanes` supaya irisan Z tidak memotongnya.

**Layer.** `cloud` global diganti daftar layer; tiap layer memegang cloud, objek scene, dan
tumpukan undo sendiri (maks 8 — tiap entri salinan penuh, ~24 byte/titik). Edit dan ekspor
menyasar layer **aktif**; statistik, mesh, dan analisis memakai **gabungan** layer terlihat.
Semua perubahan titik lewat `layers.gantiCloud()`, jadi tidak ada jalur yang bisa mengubah
titik tanpa panel ikut menyusul.

## Kesegaran (jangan pernah menyajikan yang basi)
Alat ini sering diubah sambil dipakai, jadi dua jalur diam-diam bisa menyajikan versi lama:

- **Browser.** Starlette mengirim ETag/Last-Modified tapi bukan `Cache-Control`; tanpa
  info kesegaran eksplisit browser boleh memakai caching heuristik dan menampilkan
  `app.js` lama tanpa tanda apa pun. Middleware menambahkan `Cache-Control: no-cache`
  pada `/` dan `/static/*` — tetap disimpan, tapi wajib divalidasi; ETag membuat
  validasi itu murah (304).
- **Proses server.** Python memuat modul sekali saat proses mulai, sementara `pcs`
  sengaja memakai ulang server yang sudah jalan. `/versi` melaporkan PID dan mtime
  sumber saat impor; `pcs` membandingkannya dengan disk dan menyalakan ulang bila
  lebih tua. Server yang terlalu tua untuk punya `/versi` tidak dibunuh otomatis —
  `pcs` menolak menebak proses mana yang miliknya dan menyuruh pemakainya menghentikan
  sendiri.

## Penanganan error
File tak didukung → pesan; file besar → tawaran downsample; server mati → banner; titik <ambang untuk mesh → pesan.

## Di luar cakupan (YAGNI)
Tanpa login/cloud/editing warna. Layer tidak punya warna sendiri (pakai warna asli berkas),
tidak bisa digeser/diputar sendiri (itu registrasi point cloud — masalah lain), dan tidak
bisa digabung jadi satu. Fokus: eksplorasi + bersih-bersih + ukur + ekspor.

## Stack
Python 3.12, FastAPI, uvicorn, Open3D, numpy<2, scipy ·
Three.js r160 (vendored) + OrbitControls + TransformControls.
