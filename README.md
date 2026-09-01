# riset td — pemindai 3D LiDAR & pengolah point cloud

Perkakas untuk penelitian skripsi: sebuah LiDAR 2D yang dimiringkan motor stepper,
dipasangi IMU, dijalankan di Raspberry Pi 5, menghasilkan rekaman `.mcap` —
lalu rekaman itu diubah jadi point cloud 3D dan beberapa scan dari posisi berbeda
digabungkan jadi satu peta ruangan.

Repo ini memuat **pengolah datanya** (yang kamu pakai sehari-hari) sekaligus
**perangkat lunak pemindainya** (hanya perlu kalau merakit alatnya).

> **Baru di sini?** Mulai dari [`SETUP.md`](SETUP.md) — pemasangan dari nol,
> lengkap dengan perintah untuk Fedora, Ubuntu, dan Arch.
> Ringkasnya: **butuh Python 3.12, bukan 3.13**, dan **tidak butuh Ubuntu**
> selama kamu hanya mengolah data.

---

## Mulai cepat

Setelah `SETUP.md` selesai:

```bash
cd cloudcom                              # folder data

clomcap scan_0080_1sweep_0.mcap          # satu scan → CloudCompare + grid
pcs scan_0080_1sweep_0.mcap              # atau lihat di browser

clomerge scan_0080*.mcap scan_0081*.mcap # gabungkan beberapa scan
```

---

## Di mana berkas pentingnya

```
riset td/
├── SETUP.md              ← pemasangan di komputer baru
├── README.md             ← berkas ini
├── perintah.sh           ← definisi perintah; sumberkan dari ~/.bashrc
│
├── cloudcom/             ← DATA. Rekaman .mcap dan seluruh hasil olahan
│   ├── scan_0080_1sweep_0.mcap   … scan 0080–0125
│   └── out/              ← semua hasil, tidak pernah menimpa yang lama
│       ├── <nama_scan>/  ← hasil clomcap per scan
│       ├── _merge/001…   ← hasil clomerge, bernomor urut
│       ├── _merged/      ← hasil clomerged
│       ├── _merge_out/   ← hasil clomergeout
│       ├── _outmerge/    ← hasil outmerge
│       └── _pasak/001…   ← hasil pasak
│
├── ros2_ws/cloudcom/     ← PROGRAM pengolah data
│   ├── CARA_PAKAI.txt    ← rujukan lengkap tiap perintah (385 baris)
│   ├── requirements.txt
│   ├── mcaptopc.py       ← .mcap → point cloud
│   ├── clomcap.py        ← + grid + buka CloudCompare
│   ├── clomerge.py       ← registrasi otomatis (RANSAC + ICP)
│   ├── clomerged.py      ← registrasi sadar-fitur, untuk tembok polos
│   ├── outmerge.py       ← registrasi luar ruang, peta tumbuh
│   ├── pasak.py          ← registrasi berjangkar, manusia menunjuk
│   ├── docs/             ← catatan cara kerja tiap algoritma
│   └── tes/  data/       ← data uji
│
├── pointcloud_studio/    ← penampil & penyunting berbasis browser
│   ├── README.md         ← rujukan lengkap pcs
│   └── run.sh
│
└── ros2_ws/src/          ← ROS 2: pengambilan data dari perangkat keras
    ├── sllidar_ros2/     ← driver LiDAR
    ├── stepper_controller/
    └── wit_ros2_imu/
```

**Dua folder bernama `cloudcom`.** Yang di akar berisi **data**; yang di
`ros2_ws/` berisi **program**. Membingungkan, tapi begitulah adanya.

Hasil selalu masuk ke `cloudcom/out/`, di mana pun kamu menjalankan perintahnya.
Pindahkan dengan `export CLOUDCOM_OUT="/tempat/lain"`.

---

## Alur kerjanya

```
   .mcap                .ply              beberapa .ply           satu peta
 (rekaman)  ──────▶  (point cloud)  ──────▶  (tiap scan)   ──────▶  tergabung
             clomcap                clomerge / clomerged /
                                    outmerge / pasak
                       │                                              │
                       └──────────  pcs / CloudCompare  ──────────────┘
                                    (lihat, ukur, periksa)
```

Tiap scan punya titik nol sendiri — posisi sensor saat merekam. Jadi kalau
beberapa scan dibuka bersamaan tanpa registrasi, semuanya menumpuk di satu
tempat. Itulah pekerjaan tahap penggabungan: mencari geseran dan putaran tiap
scan terhadap scan acuan.

---

## Perkakasnya

| Perintah | Untuk apa | Rujukan |
|---|---|---|
| `clomcap` | satu scan → point cloud + grid → CloudCompare | CARA_PAKAI §1 |
| `clomcaps` | banyak scan → satu jendela CloudCompare | CARA_PAKAI §2 |
| `clomerge` | registrasi otomatis dalam ruangan (RANSAC + ICP) | CARA_PAKAI §3 |
| `clomergeout` | versi luar ruang, scan melingkar searah | CARA_PAKAI §4 |
| `outmerge` | luar ruang, peta yang tumbuh bertahap | CARA_PAKAI §5 |
| `clomerged` | sadar fitur — untuk tempat bertembok polos | CARA_PAKAI §6 |
| `pasak` | registrasi berjangkar, manusia menunjuk penambat | bagian di bawah |
| `pcs` | lihat, ukur, bersihkan, ekspor — di browser | `pointcloud_studio/README.md` |
| `clocom` | buka CloudCompare biasa | CARA_PAKAI §7 |

Semua perintah jalan dari folder mana pun. Paling enak dari `cloudcom/`, supaya
nama berkasnya bisa diketik apa adanya.

### Memilih alat penggabung

- **Dalam ruangan, berciri jelas** (perabot, tiang, sudut) → `clomerge`
- **Dalam ruangan, tembok polos** → `clomerged`
- **Luar ruangan** → `outmerge`
- **Semuanya gagal, atau hasilnya "rapi tapi salah tempat"** → `pasak`

---

## `pasak` — registrasi berjangkar

Belum tercatat di `CARA_PAKAI.txt`, jadi ditulis di sini.

Registrasi otomatis mencari sendiri jawabannya, dan di tempat minim ciri ia bisa
mendarat di jawaban yang salah dengan yakin. `pasak` membalik pembagian kerja:
**manusia menunjuk benda mana yang sama**, mesin menyelesaikan geometrinya.
Tanpa pencarian, jadi tanpa minimum lokal.

Tanah mengunci roll, pitch, dan Z. Dua benda yang kamu tunjuk mengunci yaw, X, Y.

Alurnya dua tahap dengan manusia di tengah:

**1. Siapkan** — memetik benda-benda menonjol dari tiap scan:

```bash
cd cloudcom
pasak siapkan scan_0080*.mcap scan_0081*.mcap scan_0082*.mcap scan_0083*.mcap
```

Hasilnya ke `out/_pasak/NNN/`, berisi satu `*_benda.ply` per scan.

| Pilihan | Arti |
|---|---|
| `--tegakkan` | pakai tembok sebagai acuan tegak, bukan tanah — kalau tanahnya miring |
| `--max-tapak M` | tapak mendatar terbesar yang masih dianggap benda (baku 1,5 m); naikkan kalau ada benda lebar yang tak terpetik |
| `--range M` | potong sejauh ini dari sensor |

**2. Tunjuk jangkarnya** — di `pcs`:

```bash
pcs out/_pasak/002/*_benda.ply
```

Klik satu benda di scan A, lalu benda yang sama di scan B. Catat nomornya ke
`out/_pasak/002/pasangan.json`:

```json
{
 "pasangan": [
  {"a": "scan_0080_1sweep_0", "b": "scan_0083_1sweep_0", "jangkar": [[1, 0], [2, 1]]},
  {"a": "scan_0082_1sweep_0", "b": "scan_0083_1sweep_0", "jangkar": [[10, 5], [8, 4]], "icp": false}
 ]
}
```

`"jangkar": [[1, 0], [2, 1]]` berarti benda #1 di scan A = benda #0 di scan B,
dan benda #2 di A = benda #1 di B. Dua pasang cukup mengunci penuh.

`"icp": false` mematikan perapian ICP untuk tepi itu — **perlu kalau tampalannya
tipis.** Terukur pada tepi 0082: jangkar sudah tepat 1,1 cm, lalu ICP menyeretnya
1,62 m. `--redam` tidak menolong, karena seretannya tegak lurus arah lemah.

Ada `usulan.json` berisi tebakan otomatis. Periksa, jangan percaya.

**3. Selesaikan:**

```bash
pasak selesaikan out/_pasak/002
```

| Pilihan | Arti |
|---|---|
| `--sambung-lantai` | samakan kemiringan antar scan lewat lantai bersama |
| `--redam 0` | bekukan gerak sepanjang arah lemah; `1` = ICP biasa |

Keluarannya: `merged.ply`, `merged_check.ply` (tiap scan satu warna),
`laporan.txt`, dan beberapa PNG peta.

---

## Membaca hasil — dan mengapa angkanya bisa menipu

**Cara memeriksa yang benar: buka `merged_check.ply`.** Tiap scan diberi satu
warna. Kalau satu dinding tampak **dobel dengan dua warna berbeda**, registrasi
scan itu meleset. Kalau warnanya menyatu di permukaan yang sama, berarti benar.

### Angka yang TIDAK boleh dipakai sebagai bukti

Sudah terukur di data 0080–0083 bahwa metrik bawaan **memihak jawaban yang salah**:

| Metrik | Yang terjadi |
|---|---|
| `tajam@3cm` | pose benar **0,091** · meleset satu benda **0,109** · meleset lebih jauh **0,111** — makin tergelincir makin tinggi |
| `fitness@10cm` | **0,293** untuk pose yang meleset 0,73 m, lawan **0,170** untuk yang benar — ia mengikuti kepadatan |
| "benda mendarat di benda" | kalau semua benda segaris, apa pun yang memetakan garis ke dirinya sendiri akan lolos. Terukur: menukar dua benda simetris memutar pose **180°** tanpa ketahuan |

`fitness@10cm` juga **jenuh**: meleset 0,4 m dan 1,2 m tidak terbedakan.

### Wasit yang sah

1. **Kesinambungan struktur di luar benda jangkar.** Dinding panjang harus
   tersambung utuh dari semua scan, dengan jangkauan yang saling melengkapi.
   Pose yang salah membuat dinding besar berdiri sendirian tanpa sambungan.
2. **Arah hadap tiap scan harus mengisi kuadran berbeda.** Dua scan dengan yaw
   nyaris sama adalah tanda bahaya.
3. **Sumber kedua yang mandiri.** Sudut dari `outmerge` ternyata bisa dipercaya
   walau geserannya tidak — jadi yaw yang cocok dalam beberapa derajat adalah
   dukungan yang sah.

### Satu hal yang sering disalahartikan

**Liputan azimut tiap scan sempit, bukan 360°.** Scan 0083 menaruh 96% titiknya
di sektor −60°…+60°; 6 dari 12 arah kosong. Akibatnya tampalan antar scan cuma
200–700 titik, dan ada daerah yang hanya diliput satu scan. **Daerah satu warna
di `merged_check.ply` itu wajar** — jangan langsung ditafsirkan sebagai pose
yang salah sebelum memeriksa liputannya.

---

## Kalau ada yang salah

| Pesan / gejala | Sebabnya | Perbaikannya |
|---|---|---|
| `No matching distribution found for open3d` | Python 3.13 | pasang Python 3.12, buat ulang venv-nya |
| `clomcap: command not found` | `perintah.sh` belum disumberkan | `source perintah.sh`, lalu tambahkan ke `~/.bashrc` |
| `[ERROR] venv cloudcom belum dibuat` | langkah 3 SETUP.md terlewat | buat venv dan pasang `requirements.txt` |
| `[ERROR] Open3D belum terpasang di venv ini` | Open3D belum ada | `.venv/bin/python -m pip install open3d==0.19.0` |
| `[ERROR] 'flatpak' tidak ditemukan` | CloudCompare tak bisa dibuka otomatis | pasang flatpak, atau buka berkas `.ply` yang disebut secara manual |
| `[ERROR] Perlu dekompresi zstd` | berkas `.mcap.zstd` | pasang `zstandard` di venv |
| `ModuleNotFoundError: No module named 'lark'` saat pytest | venv ikut membaca pustaka ROS 2 | jalankan dengan `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` di depannya |
| `[ERROR] pasangan.json masih kosong` | jangkar belum ditunjuk | isi `pasangan.json`, lihat bagian `pasak` |
| `[ERROR] Nol titik valid diekstrak` | topik MCAP-nya salah | tambahkan `-t /map_3d` |
| `[ERROR] Semua titik terbuang` | parameter terlalu ketat | longgarkan `--range` atau `--voxel` |
| `[nama] tanah tidak ditemukan — DILEWATI` | lantai tak terdeteksi | coba `--tegakkan` untuk memakai tembok sebagai acuan |
| Dinding **dobel** di `merged_check.ply` | registrasi meleset | `--tries 8` atau `12`, lalu `--voxel 0.25`; kalau tetap, pakai `pasak` |
| Hasil meleset **90°** | RANSAC mendarat di jawaban simetris | perbanyak `--tries`; ganti scan acuan |
| Registrasi lambat | voxel terlalu halus | perbesar `--voxel` |

Registrasi memakai RANSAC yang **acak** — menjalankan perintah yang sama persis
bisa memberi hasil berbeda. Karena itu tiap run disimpan ke folder bernomor
sendiri dan tidak pernah menimpa yang sebelumnya.

Untuk memastikan pemasanganmu sehat:

```bash
cd ros2_ws/cloudcom && .venv/bin/python -m pytest -q -m "not slow"
```

337 tes harus lulus (tanpa `-m "not slow"` ikut satu tes registrasi utuh yang
lambat). Kalau ada yang gagal, jangan lanjut mengolah data — ada
versi pustaka yang tidak cocok, dan angka hasilmu tidak akan sebanding dengan
yang tercatat.

---

## Dokumen lain

| Berkas | Isi |
|---|---|
| [`SETUP.md`](SETUP.md) | pemasangan di komputer baru |
| [`ros2_ws/cloudcom/CARA_PAKAI.txt`](ros2_ws/cloudcom/CARA_PAKAI.txt) | rujukan lengkap tiap perintah beserta seluruh pilihannya |
| [`pointcloud_studio/README.md`](pointcloud_studio/README.md) | rujukan lengkap `pcs` |
| `ros2_ws/cloudcom/docs/` | catatan cara kerja algoritmanya |
| `SESI_*.md` | catatan sesi kerja — apa yang dicoba dan apa hasilnya |
| [`ros2_ws/PANDUAN_SISTEM.md`](ros2_ws/PANDUAN_SISTEM.md) | merakit & menyalakan perangkat kerasnya |
| [`ros2_ws/PANDUAN_PORT_SERIAL.md`](ros2_ws/PANDUAN_PORT_SERIAL.md) | penetapan port USB yang tidak berpindah |

## Data

Repo ini memuat scan **0080 ke atas** (31 rekaman) beserta hasil olahannya.
Scan 0001–0079 dan tiga folder percobaan penggabungan lama tidak ikut karena
melebihi batas ukuran GitHub — mintalah kalau perlu.
