# PointCloud Studio

Alat lokal untuk menjelajahi & membersihkan point cloud hasil scan 3D LiDAR (PLY/XYZ).
Dibuat untuk proyek skripsi 3D LiDAR Scanner — lebih nyaman dari CloudCompare untuk
eksplorasi cepat, blok-hapus area, ukur jarak, dan analisis dimensi ruangan.

## Menjalankan

Cara tercepat — satu perintah, langsung dengan berkasnya:

```bash
pcs                             # buka aplikasinya saja, pilih berkas dari sana
pcs merged.ply                  # langsung dengan berkasnya
pcs sweep_1.ply sweep_2.ply     # banyak berkas — tiap berkas jadi satu layer
pcs scan_0007_1sweep_0.mcap     # MCAP dikonversi otomatis (pakai cache)
```

`pcs` menyalakan server bila belum jalan (atau memakai yang sudah ada — dan menyalakan
ulang kalau server itu memuat kode yang lebih tua dari berkas di disk) lalu membuka
browser. Bila diberi berkas, halaman langsung memuatnya; tanpa berkas, aplikasinya
terbuka kosong dan berkas dipilih lewat tombol "Buka" atau seret-lepas. Berkas yang
salah ketik ketahuan sebelum konversi MCAP mana pun dijalankan. Pilihan:

| Pilihan | Arti |
|---|---|
| `--voxel M` | ukuran voxel dalam meter (bawaan `0.01`) |
| `--full` | kirim resolusi penuh, tanpa optimasi kerapatan |
| `--port N` | port server (bawaan `8000`) |
| `--force` | konversi ulang MCAP walau hasil lama masih segar |
| `-t TOPIK` | paksa topik MCAP tertentu, mis. `/map_3d` |

**Optimasi kerapatan.** Semua berkas dioptimasi dengan voxel sebelum dikirim ke browser —
baik yang lewat `pcs` maupun yang diunggah/diseret ke jendela:
ruang dibagi kubus bersisi `--voxel` dan tiap kubus menyisakan satu titik. Bawaan 1 cm
(10.000 titik per m² permukaan) ada di bawah tingkat kebisingan sensor LiDAR, jadi yang
hilang hanya titik yang menumpuk di tempat sama — pada data scan biasanya menyisakan
sekitar sepertiga titik. Baris petunjuk selalu menyebut voxel yang terpakai dan jumlah
titik aslinya. Berkas di disk tidak pernah diubah; pakai `--full` bila perlu semuanya.

Atau tanpa argumen berkas, seperti sebelumnya:

```bash
cd "/home/bromarku/riset td/pointcloud_studio"
./run.sh
```

Saat pertama kali, `run.sh` otomatis membuat virtualenv dan memasang dependensi
(termasuk Open3D ~400MB — sekali unduh saja). Setelah itu server menyala di
`http://127.0.0.1:8000` dan browser terbuka otomatis. Tekan `Ctrl+C` untuk berhenti.

> Semua diproses **lokal di laptop** — tidak ada data yang diunggah ke internet.

## Cara pakai
1. **Tarik file** `.ply` / `.xyz` ke jendela (atau klik "Buka"). Boleh beberapa sekaligus —
   tiap berkas jadi satu **layer**, dan membuka berkas baru tidak pernah menimpa yang lama.
2. **Panel Layer** → ◉ sembunyi/tampil · klik nama untuk menjadikannya **aktif** · ✕ tutup.
3. **Mode tampilan**: Titik · Padat · Mesh (permukaan).
4. **Pilih area** ▭ → tarik kotak → hapus atap/lantai/noise (atau "Simpan di dalam" untuk crop).
5. **Grid** ▦ → bidang acuan yang bisa digeser & diputar (lihat di bawah).
6. **Ukur** 📏 → klik menancapkan ujung, garis mengikuti kursor dengan angka hidup,
   klik lagi mengunci. Hasilnya menetap dan bisa diekspor ke CSV.
7. **Irisan Z** → slider untuk menyembunyikan atap/lantai (lihat denah).
8. **Analisis dimensi** → RANSAC deteksi dinding/lantai → tinggi, RMSE planaritas, ortogonalitas.
9. **Ekspor** → simpan hasil editan ke PLY/XYZ baru (file asli tak tersentuh).

## Grid referensi & pengukuran

Grid adalah bidang bergaris yang bisa ditaruh di mana saja — di lantai, di dinding
miring, di ketinggian tertentu — lalu dipakai sebagai acuan ukur.

**Memindahkan grid:** seret gizmo di viewport (tombol **Geser** / **Putar**), atau ketik
angkanya di panel. Keduanya menampilkan state yang sama. **Snap gizmo** mengunci geseran
ke kelipatan spasi dan putaran ke 15°.

**Menempelkan grid ke permukaan** — tiga cara, semuanya di panel Grid:

| Cara | Kapan dipakai |
|---|---|
| **📐 Pasang di 3 titik** | klik 3 titik di lantai/dinding → grid langsung sejajar bidang itu |
| **Bidang RANSAC ▾** | setelah menjalankan Analisis, pilih bidang yang terdeteksi |
| **Datar (XY)** / **Ke pusat data** | kembalikan ke mendatar, atau pindahkan ke tengah data |

**Snap pengukuran** — dua pilihan:

- **Titik** — klik nempel ke titik cloud terdekat (ambangnya ikut skala data).
- **Grid** — klik mendarat di bidang grid, titik cloud diabaikan. Berguna kalau awannya
  berisik: taruh grid di lantai, lalu ukur denah tanpa terganggu titik nyasar.

**Tipe** — **Jarak** (2 klik) atau **Sudut** (3 klik, sudut diukur di titik kedua).
`Esc` membatalkan pengukuran yang sedang berjalan. Tiap hasil punya label di viewport
dan baris di panel; menggantung kursor di barisnya menyorotnya, ✕ menghapusnya, dan
**💾 CSV** mengunduh semuanya (koordinat ujung ikut) untuk tabel validasi.

> Gizmo grid disembunyikan selama mode Ukur — lengannya menutupi petak layar yang besar
> tepat di pusat grid. Matikan "📏 Ukur" untuk menyetel grid lagi.

### Layer: apa kena apa

| Operasi | Berlaku ke |
|---|---|
| Hapus/crop area · Undo · Ekspor | layer **aktif** saja |
| Statistik · Mesh · Analisis RANSAC | **gabungan** semua layer yang terlihat |
| Irisan Z · mode tampilan | seluruh scene |

Menghapus titik itu merusak, jadi sasarannya tunggal dan jelas. Analisis tidak merusak
dan justru berguna melintasi beberapa sweep dari ruangan yang sama, jadi memakai gabungan.

## Navigasi 3D
| Aksi | Kontrol |
|---|---|
| Putar | drag kiri |
| Geser | drag kanan |
| Zoom | scroll |
| Preset | tombol Atas / Depan / Samping |
| Ortografis | tombol Ortho (matikan perspektif untuk ukur akurat) |

## Struktur
```
pointcloud_studio/
├── pcs.py                 perintah `pcs` (siapkan berkas → server → browser)
├── run.sh                 launcher tanpa argumen berkas
├── requirements.txt
├── backend/
│   ├── server.py          FastAPI (load, open, mesh, analyze)
│   ├── loader.py          parser PLY/XYZ
│   ├── downsample.py      optimasi kerapatan berbasis voxel
│   ├── mesh.py            meshing berbasis-sudut
│   └── analysis.py        RANSAC dimensi (Open3D)
├── frontend/
│   ├── index.html
│   ├── app.js             bootstrap: seret-lepas, ?file=…
│   ├── ui.js              panel sidebar + wiring toolbar
│   ├── io.js              /load /open /mesh /analyze + ekspor
│   ├── edit.js            box-select hapus/crop + undo
│   ├── grid.js            grid referensi + gizmo transform
│   ├── measure.js         alat ukur jarak/sudut + CSV
│   ├── layers.js          daftar layer (sumber kebenaran titik)
│   ├── viewer.js          panggung Three.js: kamera, kontrol, material
│   ├── hud.js             toast + baris keterangan
│   └── vendor/            Three.js (offline)
├── tests/                 pytest
└── docs/DESIGN.md         dokumen desain
```

## Tes

```bash
cd "/home/bromarku/riset td/pointcloud_studio"
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q
```

Dua penyesuaian lingkungan itu wajib bila ROS 2 sedang di-source di shell: `PYTHONPATH`
milik ROS membuat venv melihat paket `/opt/ros`, dan plugin pytest bawaan ROS
(`launch_testing`) gagal termuat sehingga pengumpulan tes berhenti sebelum mulai.

Detail teknis: lihat `docs/DESIGN.md`.
