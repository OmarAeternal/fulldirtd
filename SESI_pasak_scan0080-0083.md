# Sesi `pasak` — 23 Agustus 2026

Catatan serah-terima. Baca ini dulu sebelum lanjut.

**Melanjutkan sesi ini dengan konteksnya utuh:**

```bash
cd ~/"riset td/cloudcom" && claude --resume 808ce82e-0d2a-4b4d-ab20-1a524fcb8eff
```

---

## Pertanyaan awalnya

Bisakah mapping akurat di tempat semi-outdoor yang minim fitur? Contohnya
`scan_008*` yang memetakan tulisan FILKOM: hasilnya sejajar tembok tapi salah
tempat. Bisakah dibuat algoritma yang mencocokkan lewat fitur lain — benda dan
sejenisnya?

Jawabannya bisa, tapi tidak dengan cara yang paling wajar. Lihat §3.

---

## SUDAH DIKERJAKAN

### 1. Diagnosis terukur — tiga temuan baru

Semuanya diukur hari ini pada `scan_0080`–`0083`, bukan dugaan.

**a. Yang tak terkunci tinggal 2 DoF mendatar.** Dua run `outmerge` yang cuma
beda `--range` (028 = 6 m, 029 = 15 m) memberi sudut relatif konsisten
(relatif ke 0083: 0080 di −91,4° lawan −91,2°; 0081 di −179,4° lawan −178,9°;
0082 di +88,6° lawan +88,2°) tapi jarak antar posisi sensor **beda sampai
0,8 m** (0080↔0082: 1,41 m lawan 0,60 m). Rotasi kasar benar; geseran tidak.

**b. Rig sendiri ikut ter-scan dan tak pernah di-mask.** Gugus tetap 311–431
titik di titik asal sensor (pusat ≈ (−0,2, 0, +1,05), ukuran ≈ 0,3×0,4×0,9) di
keempat scan; ada gugus kedua di z ≈ 1,6. Tripod/mount/operator. Ia bergerak
**bersama** sensor, jadi selalu cocok sempurna dan menarik ICP ke arah
menumpuk-tripod.

**c. Fitness melawan peta gabungan menggelembung.** `outmerge` melapor 0,61–0,85
"BAIK" untuk keempatnya. Diukur **berpasangan**:

| pasangan | fitness@10cm | tajam@5cm | terbaik di sapuan ±1,6 m / ±12° |
|---|---|---|---|
| 0080→0082 | 0,218 | 0,046 | 0,100 di yaw +6°, geser (+0,4, −0,4) |
| 0080→0081 | 0,121 | 0,024 | 0,094 di yaw +6°, geser (0,0, +0,4) |
| 0081→0082 | 0,211 | 0,050 | 0,159 di yaw 0°, geser (**+1,2**, +0,2) |
| 0082→0083 | 0,043 | 0,005 | 0,114 di yaw −9°, geser (−0,2, −1,0) |

Sebabnya `outmerge` menilai tiap scan melawan **peta yang sedang tumbuh** — 3
scan lain, ~3× lebih padat, jadi ~3× lebih banyak kesempatan menemukan pasangan
dalam 10 cm. Angkanya mengikuti kepadatan, bukan kebenaran. **Sumber kepercayaan
diri palsu yang ketiga.**

**Kesimpulan: peta gabungan `scan_0080`–`0083` yang ada sekarang hampir pasti
salah.**

Catatan kehati-hatian: perbaikan yaw +6°/−9° di kolom terakhir **belum kuat** —
itu puncak tertinggi dari 2.601 sel pada statistik bernilai ~0,1, dan puncak
begitu cenderung menggelembung. Yang bisa dipegang: yaw benar sampai orde 90°,
belum tentu sampai derajat.

### 2. Jurang antar ukuran

Satu scan melawan dirinya sendiri, digeser menyusuri tembok:

| geser | fitness@10cm | tajam@3cm |
|---|---|---|
| 0,0 m | 1,000 | 1,000 |
| 0,4 m | 0,459 | 0,133 |
| 0,8 m | 0,426 | 0,094 |
| 1,2 m | 0,407 | 0,083 |

`fitness@10cm` jenuh. `tajam@3cm` tajam. Tapi antara **dua scan sungguhan**,
tajam@3cm di seluruh jendela ±1,5 m tak pernah lewat 0,033 dan dua sel teratas
beda 2% (nisbah 0,98). Satu terlalu longgar, satunya terlalu ketat, tak ada yang
pas di tengah.

### 3. JALAN BUNTU yang sudah diuji — jangan diulang

Pencocokan konstelasi otomatis lewat titik-pusat gugus benda + klik-maksimum
pada jarak berpasangan:

| pasangan | benda | calon | klik konsisten | sisa |
|---|---|---|---|---|
| 0080↔0082 | 11 vs 6 | 16 | 2 | — |
| 0080↔0081 | 15 vs 14 | 56 | 3 | rata 2,99 m |
| 0081↔0083 | 11 vs 21 | 85 | 3 | rata 0,86 m |

Sebabnya fisik, bukan bug: dari satu sudut pandang hanya separuh benda terlihat,
jadi titik-pusatnya bias ke arah sensor sebesar ~separuh tebal benda. Bias itu
lebih besar dari toleransi yang diperlukan. **Menambah parameter tidak menolong.**

Inilah yang membentuk rancangan: manusia menentukan korespondensinya — satu-
satunya bagian yang mesin gagal — mesin mengerjakan seluruh geometrinya.

### 4. `pasak` — inti selesai, 30 tes lulus

Spesifikasi: `ros2_ws/cloudcom/docs/superpowers/specs/2026-08-23-pasak-design.md`
Kode: `ros2_ws/cloudcom/pasak.py` + `test_pasak.py`
Commit: `b5058ff` (spesifikasi), `6b8576f` (kode)

Kunci matematisnya: tanah mengunci roll/pitch/z, dua benda yang ditunjuk manusia
mengunci yaw/x/y. Dua jangkar = 4 batasan untuk 3 anu, tertentu penuh dengan satu
sisa yang bisa dicek. **Tanpa pencarian**, jadi tanpa minimum lokal dan tanpa
alias periodik — beda dari `clomerge`/`outmerge`/`clomerged` yang semuanya mencari.

Yang selesai dan teruji:
- `buang_rig` — rig di titik asal dibuang; tembok dekat (x = 0,49 m) selamat
  karena syaratnya "menonjol DAN di dalam radius"
- `atlas_bidang` — **ditulis ulang tanpa RANSAC.** `segment_plane` diparalelkan
  Open3D dan TIDAK bisa diulang walau di-seed; terukur membuat daftar benda
  berbeda antar eksekusi. Kini kisi arah normal tetap + histogram jarak + SVD.
- `kerangka_tanah` — SVD dengan perekrutan ulang, selalu dari potongan 6 m
  supaya tak lagi ikut berubah mengikuti `--range` (dulu 7,35° lawan 9,38° untuk
  scan_0081, = 17 cm meleset di 5 m)
- `nilai` — berpasangan saja; acuan dijarangkan ke kisi tetap dulu sehingga
  kepadatan tidak bisa membeli nilai
- `pasang` — Kabsch 2-D + ICP yang arah lemahnya diredam (`REDAM = 0.1`);
  peringatan wajib untuk salah tunjuk, yaw pinjaman, dan redaman bocor

Batas yang **diuji dan dicatat**, bukan disembunyikan:
- Menukar dua jangkar TIDAK terdeteksi lewat jarak (jaraknya tak berubah). Yang
  menangkapnya cuma nilai akhir — sebab itu laporan wajib memuat tajam@3cm.
- Pose akhir terulang sampai 1e-6 m, bukan bit-per-bit: ICP menjumlah paralel.
  Jawaban jangkarnya sendiri persis sama, dan itu yang diuji ketat.

### 5. Sudah dijalankan pada data asli

```
pasak siapkan scan_008[0-3]_1sweep_0.mcap   →   out/_pasak/001
```

13–16 benda per scan. Isinya: `awan/`, `<nama>_benda.ply`, `benda.json`,
`usulan.json`, `pasangan.json` (masih kosong).

---

## BELUM DIKERJAKAN — lanjut dari sini

### Langkah 1 (paling penting): buktikan jangkarnya memang ada

Ini risiko yang belum terjawab. Rancangannya berdiri di atas asumsi tiap
pasangan scan bertetangga punya **minimal dua benda yang sama-sama terlihat**.
Belum bisa dibuktikan tanpa tahu korespondensinya — melingkar. Yang bisa
membuktikan cuma mata di pcs.

```bash
pcs out/_pasak/001/scan_0083_1sweep_0_benda.ply
pcs out/_pasak/001/scan_0080_1sweep_0_benda.ply
```

Tiap benda satu warna; nomor + posisinya ada di `benda.json`. Cari dua benda yang
sama di kedua scan, lalu tulis ke `out/_pasak/001/pasangan.json`:

```json
{"pasangan": [
  {"a": "scan_0080_1sweep_0", "b": "scan_0083_1sweep_0", "jangkar": [[2, 4], [5, 1]]}
]}
```

lalu:

```bash
pasak selesaikan out/_pasak/001
pcs out/_pasak/001/merged_check.ply
```

Kalau dua jangkar untuk satu pasangan saja sudah memberi peta yang benar secara
visual, seluruh pendekatan ini terbukti dan sisanya tinggal mengulang.

**Kalau ternyata tidak ada dua benda yang sama-sama terlihat**, mundurnya sudah
ada di kode: satu jangkar + normal tembok (yaw-nya lalu bersandar pada tembok,
dan laporannya bilang begitu). Kalau itu pun tidak ada, pendekatannya yang harus
ditinjau ulang, bukan parameternya.

**Usulan otomatis** ada di `usulan.json` — diperingkat kemiripan sifat, sebagai
pemendek daftar saja. Yang teratas per pasangan:

| pasangan | usulan teratas | beda |
|---|---|---|
| 0081↔0082 | #11 = #12 | 0,702 |
| 0080↔0082 | #11 = #1 | 0,919 |
| 0080↔0082 | #3 = #7 | 1,006 |
| 0080↔0083 | #13 = #8 | 1,146 |
| 0081↔0083 | #1 = #6 | 1,243 |

**Periksa, jangan percaya** — ini cuma kemiripan sifat, bukan bukti geometris.

### Langkah 2: modul pemetik di pointcloud_studio

Belum dikerjakan sama sekali. Rencananya (spesifikasi §3.5):
- `pointcloud_studio/frontend/pasak.js` — mode baru. Yang sudah bisa dipakai
  ulang: raycaster picking dengan pembeda klik-vs-putar 4 px
  (`measure.js:127`, `measure.js:359`), multi-layer (`layers.js`), `GET /open`.
- `POST /pasak/pasangan` di `backend/server.py`, ~30 baris — terima daftar
  pasangan, tulis ke `pasangan.json`.
- Klik dikembalikan sebagai **koordinat 3-D**; yang memetakannya ke ID benda
  adalah `pasak.benda_di_ringkasan()` di sisi backend (kotak pembatas dulu, baru
  pusat terdekat). pcs tetap **hanya pemetik**; seluruh geometri di `pasak.py`.

Kerjakan ini **sesudah** langkah 1 — tak ada gunanya membangun UI untuk alur yang
belum terbukti.

### Langkah 3: sesudah 0080–0083 benar

- `scan_0072`–`0075` — masalah sama (geseran tembok), sudah lama menggantung
- `scan_0064`–`0071` — masalahnya **berbeda**: ambiguitas sudut ruangan yang
  mirip satu sama lain, bukan geseran sepanjang tembok. `pasak` mungkin
  membantu, tapi jangan dianggap otomatis jawabannya.

---

## Kalibrasi terukur — jangan diulang

- `--range 6` (baku) memberi tajam 0,25–0,46; `--range 10` memburuk ke 0,08–0,12
- `--sharp 0.02` di `clomerged` membuat semuanya GAGAL. Jangan di bawah 0.03.
- `--slide-range 3.0 --weak-ratio 0.2` di `clomerged` LEBIH BURUK daripada baku
- Menaikkan `--seeds`/`--step-deg` pada `outmerge` untuk set 0064–0071 sudah
  dicoba dan mengulang jawaban yang sama — bukan soal kerapatan sapuan

## Menjalankan tesnya

```bash
cd ~/"riset td/ros2_ws/cloudcom"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_pasak.py -v
```
