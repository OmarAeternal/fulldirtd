# Cara kerja clomerge

Penjelasan alur `clomerge.py` — bagaimana beberapa scan dari posisi berbeda
disatukan jadi satu point cloud, dan bagaimana program tahu posisi yang benar.

Ditulis untuk dibaca tanpa latar belakang khusus. Nomor baris merujuk
`clomerge.py`.

---

## Masalahnya

Tiap scan punya **origin sendiri**. Titik (0, 0, 0) di sebuah scan artinya
"posisi sensor saat merekam" — bukan satu titik tetap di ruangan. Jadi kalau
empat scan dari empat posisi langsung ditumpuk, keempatnya menumpuk di tempat
yang sama, seolah-olah semua direkam dari titik yang persis sama.

Yang dicari `clomerge` adalah **transformasi rigid** untuk tiap scan: berapa
harus digeser dan diputar agar jatuh di posisi yang benar relatif terhadap scan
acuan. "Rigid" artinya bentuknya tidak diubah sama sekali — hanya digeser dan
diputar, tidak diregangkan atau diperbesar.

Transformasi ini disimpan sebagai **matriks 4×4**. Satu matriks memuat rotasi
dan translasi sekaligus; mengalikannya dengan koordinat sebuah titik
menghasilkan koordinat barunya ([`apply_transform`](../clomerge.py), baris 99).

## Jawaban singkatnya

**Program tidak pernah "tahu" posisi yang benar.** Ia menebak ribuan kali, lalu
memilih tebakan yang membuat paling banyak titik saling berimpit.

Itu ide intinya. Sisa dokumen ini menjelaskan bagaimana tebakan itu dibuat agar
tidak asal, dan bagaimana hasilnya dinilai.

---

## Tahap 1 — Menggambarkan bentuk lokal (FPFH)

**Baris 251, `preprocess()`**

Sebelum bisa mencocokkan, tiap titik perlu "identitas" — sesuatu yang membuatnya
bisa dikenali dari scan lain.

Caranya: untuk tiap titik, lihat tetangga-tetangganya, hitung sudut-sudut antara
arah hadap (normal) mereka, lalu rangkum jadi satu **vektor 33 angka**. Vektor
ini disebut FPFH (*Fast Point Feature Histogram*).

Analogi: seperti sidik jari, tapi untuk bentuk permukaan. Titik di sudut siku
ruangan menghasilkan pola angka yang berbeda dari titik di tengah dinding datar.

**Kenapa ini bekerja:** angka-angka itu **tidak berubah kalau cloud digeser atau
diputar**, karena yang diukur adalah hubungan antar tetangga, bukan koordinat
absolut. Sudut pojok ruangan tetap "terasa" seperti sudut pojok, dari posisi
sensor mana pun. Inilah yang memungkinkan pencocokan tanpa tahu posisi awal.

Sebelum ini, cloud di-*downsample* dulu jadi kisi voxel (`--voxel`, default
0,15 m) supaya perhitungannya ringan. Ini hanya untuk registrasi — **hasil akhir
selalu memakai titik resolusi penuh**.

## Tahap 2 — Menebak dengan RANSAC

**Baris 262, `global_register()`**

RANSAC = *RANdom SAmple Consensus*. Cara kerjanya berulang:

1. Ambil **3 titik acak** dari scan A.
2. Cari 3 titik di scan B yang vektor FPFH-nya paling mirip.
3. Tiga pasang titik sudah cukup untuk menghitung satu transformasi rigid.
4. Terapkan transformasi itu ke seluruh scan A, lalu **hitung berapa titik yang
   jatuh dekat titik scan B**. Itu skornya.
5. Ulangi sampai 200.000 kali. Simpan yang skornya tertinggi.

Jadi "benar" di sini punya arti yang sangat konkret: **posisi yang membuat paling
banyak permukaan saling menempel.** Kalau dua scan memotret dinding dan pojok
yang sama, biasanya hanya ada satu peletakan yang membuat semuanya berimpit
sekaligus — itulah yang menang.

Kenapa 3 titik? Itu jumlah minimum untuk menentukan posisi dan orientasi benda
kaku di ruang 3D. Semakin sedikit titik yang diambil per tebakan, semakin besar
peluang ketiganya kebetulan benar semua.

Tahap ini disebut **registrasi global**: tidak butuh tebakan awal sama sekali.

## Tahap 3 — Merapikan dengan ICP

**Baris 276, `refine()`**

Hasil RANSAC masih kasar — biasanya meleset beberapa sentimeter. ICP
(*Iterative Closest Point*) merapikannya dengan mengulang langkah sederhana:

1. Untuk tiap titik di scan A, cari titik terdekat di scan B.
2. Hitung geseran+putaran kecil yang memperkecil jarak semua pasangan itu.
3. Terapkan, lalu ulangi dari langkah 1.

Tiap putaran hasilnya makin rapat, sampai berhenti berubah.

Yang dipakai di sini varian **point-to-plane**: jaraknya diukur tegak lurus ke
*permukaan* tetangga, bukan ke titiknya. Bedanya penting — dua permukaan datar
yang saling menempel boleh bergeser menyusuri permukaan tanpa dihukum, sehingga
ICP bisa "meluncur" ke posisi benar alih-alih tersangkut.

ICP dijalankan **bertingkat**, kasar ke halus (baris 331): hasil voxel 2× jadi
tebakan awal untuk voxel 1×, lalu untuk voxel 0,5×. Ini mencegah ICP tersangkut
di minimum lokal — masalah klasiknya, ketika ia menemukan posisi yang "cukup
enak" tapi bukan yang terbaik, lalu tidak mau keluar dari situ.

**Batas penting ICP:** ia hanya bisa memperbaiki tebakan yang sudah mendekati
benar. Kalau RANSAC melesetnya jauh, ICP tidak akan menyelamatkannya — ia justru
akan memantapkan posisi yang salah.

## Tahap 4 — Menilai, hanya pada dinding

**Baris 295, `wall_subset()` · baris 189, `quality_verdict()`**

Penilaian **sengaja membuang lantai dan plafon**. Alasannya penting:

Lantai cocok dengan lantai **diputar berapa pun** terhadap sumbu tegak. Padahal
di ruangan biasa, lantai dan plafon justru bagian dengan titik terbanyak. Kalau
keduanya ikut dinilai, solusi yang melenceng 90° tetap mendapat skor tinggi
karena lantainya memang "pas" — dan bisa mengalahkan solusi yang benar.

Hanya **permukaan tegak** yang benar-benar menentukan arah hadap. Titik dianggap
tegak bila normalnya mendatar (`WALL_NZ = 0.5`, baris 56). Cloud yang memang
tidak punya permukaan tegak jatuh kembali memakai semua titik.

Hasilnya dinilai dua angka, keduanya diukur pada jarak tetap 0,10 m
(`EVAL_DIST`):

| Angka | Arti |
|---|---|
| **fitness** | Berapa bagian titik yang menemukan pasangan. Naik seiring besarnya irisan dan benarnya posisi. |
| **rmse** | Serapat apa pasangan itu, dalam meter. |

**Dua-duanya harus bagus.** Fitness tinggi dengan rmse besar berarti "cocok tapi
longgar" — banyak titik berpasangan, tapi tidak ada yang benar-benar rapat.

| Vonis | Syarat |
|---|---|
| **BAIK** | fitness ≥ 0,40 **dan** rmse ≤ 0,06 m |
| **RAGU** | fitness ≥ 0,20 |
| **GAGAL** | di bawah itu |

Ambangnya sengaja **tidak ikut berubah** bersama `--voxel`. Kalau penilaian
memakai ambang yang melebar mengikuti voxel, skornya naik sendiri saat voxel
diperbesar — dan jadi tidak bisa dibandingkan antar-run.

## Tahap 5 — Mengulang beberapa kali

**Baris 316, `register_pair()`**

RANSAC bersifat acak **dan** multi-thread, jadi hasilnya berbeda tiap dijalankan
walau datanya persis sama. Kadang mendarat di jawaban yang melenceng 90°.

Karena itu seluruh rangkaian tahap 1–4 diulang `--tries` kali (default 4), lalu
diambil yang **skor dindingnya terbaik**. Ini juga sebabnya `--tries` lebih
banyak adalah obat pertama kalau ada scan yang meleset.

## Tahap 6 — Semua ke satu acuan, bukan berantai

**Baris 420, `register_all()`**

Scan pertama jadi acuan. Semua scan lain dicocokkan **langsung ke dia**, bukan
A→B→C→D.

**Untungnya:** error satu scan tidak diwariskan ke scan berikutnya.
**Ruginya:** tiap scan harus punya irisan dengan **scan pertama**, bukan sekadar
dengan tetangganya. Di luar ruangan ini sering patah — lihat bagian Batasan.

## Tahap 7 — Meratakan lantai

**Baris 108 `find_floor_plane()` · baris 151 `level_transform()`**

Hasil gabungan mewarisi kemiringan scan acuan. Kalau sensor berdiri miring,
seluruh gabungan ikut miring dan lantai tidak sejajar grid.

Cara mencari lantai: bidang dikupas satu per satu dengan RANSAC (prinsip sama
seperti tahap 2, tapi modelnya bidang, bukan transformasi — 3 titik acak
menentukan satu bidang, lalu dihitung berapa titik yang dekat dengannya). Bidang
yang **mendatar** dan **cukup besar** dikumpulkan sebagai calon, lalu yang
**paling bawah** dipilih.

Bagian terakhir itu yang membedakan lantai dari plafon — keduanya sama
mendatarnya dan sering sama luasnya.

Lalu seluruh cloud diputar dengan **rotasi terkecil** yang membawa normal lantai
ke atas (rumus Rodrigues), dan digeser agar lantai jatuh di Z=0. Karena
rotasinya terkecil, ia **tidak menyuntikkan putaran terhadap sumbu tegak sama
sekali** — arah hadap dinding tetap seperti aslinya, hanya kemiringannya yang
dikoreksi.

**Grid tidak pernah ikut diputar.** Yang berputar adalah cloud-nya. Grid tetap
dibangun sejajar sumbu di Z=0, hanya saja dihitung *setelah* cloud diluruskan.

---

## Cara memeriksa hasilnya

Buka **`merged_check.ply`** — di situ tiap scan diberi satu warna berbeda.

- Kalau satu dinding terlihat **dobel dengan dua warna berbeda** → registrasi
  scan itu meleset.
- Kalau warnanya **saling menyatu di permukaan yang sama** → benar.

Ini pemeriksaan paling andal, jauh lebih meyakinkan daripada angka fitness.

Kalau ada yang RAGU atau GAGAL:

1. `--tries` lebih banyak (8 atau 12) — paling sering menolong
2. `--voxel` lebih besar (0,25 atau 0,3)
3. Coba scan lain sebagai acuan — taruh di urutan pertama
4. Kalau irisan memang terlalu kecil (<30%), tidak ada parameter yang bisa
   menambal; perlu scan tambahan di posisi antara

---

## Batasan yang perlu diketahui

**Butuh irisan.** Kira-kira 30% ke atas antar scan. Tanpa bagian yang sama,
tidak ada peletakan yang bisa membuat titiknya berimpit — dan tidak ada
parameter yang bisa memperbaikinya.

**Topologi bintang.** Semua harus beririsan dengan scan pertama. Di luar
ruangan, scan ke-5 bisa tidak melihat apa pun dari scan ke-1. Perbaikannya:
*pose graph* — cocokkan setiap pasang yang beririsan, lalu optimasi semuanya
sekaligus. Syaratnya jadi longgar: cukup beririsan dengan **salah satu** scan
lain. Belum diterapkan.

**Objek simetris.** Kalau bentuknya mendekati simetri putar (kotak, silinder,
bangunan bujur sangkar tanpa penanda), secara geometri memang tidak ada beda
antara orientasi yang benar dan yang terputar. Bukan kekurangan algoritma.

**Cloud harus kaku.** Tiap scan diasumsikan potret dari satu titik pandang tetap.
Kalau alat bergerak selama menyapu (mis. di drone yang melayang), cloud-nya
sendiri sudah melar sebelum registrasi dimulai. Lihat
[slam-3d-drone.md](slam-3d-drone.md).

---

## Referensi

### Video — paling mudah untuk memulai

Cyrill Stachniss (Universitas Bonn) punya seri penjelasan yang jernih, banyak
gambar, dan tidak bertele-tele:

- [Iterative Closest Point (ICP) — 5 Minutes with Cyrill](https://www.youtube.com/watch?v=QWDM4cFdKrE)
  — ringkas, mulai dari sini
- [Point-to-Plane and Generalized ICP — 5 Minutes with Cyrill](https://www.youtube.com/watch?v=2hC9IG6MFD0)
  — varian yang dipakai `refine()`
- [ICP & Point Cloud Registration Part 1: Known Data Association & SVD](https://www.youtube.com/watch?v=dhzLQfDBx2Q)
  — kuliah penuh
- [ICP & Point Cloud Registration Part 2: Unknown Data Association](https://www.youtube.com/watch?v=ktRqKxddjJk)
  — bagian yang paling relevan dengan kasus kita
- [Indeks seri "5 Minutes with Cyrill"](http://www.ipb.uni-bonn.de/5min/)

### Dokumentasi — langsung ke kode yang dipakai

- [Open3D — Global registration](https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html)
  — **paling penting.** Persis pustaka dan fungsi yang dipakai `clomerge`:
  FPFH, RANSAC, lalu penghalusan ICP. Ada contoh kode lengkap.
- [PCL — Plane model segmentation](https://pointclouds.org/documentation/tutorials/planar_segmentation.html)
  — RANSAC untuk mencari bidang; prinsip yang dipakai `find_floor_plane()`

### Bacaan lebih dalam

- [Slide kuliah ICP, Uni Bonn (PDF)](https://www.ipb.uni-bonn.de/html/teaching/msr2-2020/sse2-03-icp.pdf)
  — matematikanya, tapi masih terbaca
- [ICP Algorithm: Theory, Practice and its SLAM-oriented Taxonomy (arXiv)](https://arxiv.org/pdf/2206.06435)
  — survei menyeluruh varian ICP
- [3D RANSAC Algorithm for LiDAR PCD Segmentation (Medium)](https://medium.com/@ajithraj_gangadharan/3d-ransac-algorithm-for-lidar-pcd-segmentation-315d2a51351)
  — RANSAC dijelaskan dengan kode, gaya santai

### Makalah asli

- **FPFH** — Rusu, Blodow, Beetz, *"Fast Point Feature Histograms (FPFH) for 3D
  Registration"*, IEEE ICRA 2009.
  [ACM DL](https://dl.acm.org/doi/10.5555/1703435.1703733) ·
  [Semantic Scholar (ada PDF)](https://www.semanticscholar.org/paper/Fast-Point-Feature-Histograms-(FPFH)-for-3D-Rusu-Blodow/940dd2fa074ad97d5e8efa7e867b1f4460cfb8d5)
- **ICP** — Besl & McKay, *"A Method for Registration of 3-D Shapes"*, IEEE PAMI
  1992. (Makalah asli ICP; cari judulnya di Google Scholar.)
- **Point-to-plane ICP** — Chen & Medioni, *"Object Modelling by Registration of
  Multiple Range Images"*, 1991.
- **RANSAC** — Fischler & Bolles, *"Random Sample Consensus"*, Comm. ACM 1981.

> Empat makalah terakhir tidak kusertakan tautannya karena belum kuverifikasi —
> cari dengan judul persisnya di Google Scholar, semuanya mudah ditemukan dan
> tersedia bebas.

---

## Kata kunci untuk mencari sendiri

**Inti alur clomerge**

```
point cloud registration
global registration
FPFH feature matching
RANSAC point cloud alignment
ICP iterative closest point
point-to-plane ICP
coarse to fine registration
rigid transformation 4x4 matrix
```

**Penilaian & masalah yang muncul**

```
registration fitness inlier RMSE
ICP local minimum problem
registration degeneracy symmetry
overlap ratio point cloud registration
```

**Perataan lantai**

```
RANSAC plane segmentation
ground plane extraction point cloud
plane normal estimation
Rodrigues rotation formula
align vector to axis rotation matrix
```

**Langkah berikutnya (pose graph)**

```
pose graph optimization
multiway registration open3d
loop closure detection
robust kernel pose graph
```

**Untuk konteks drone/SLAM**

```
LiDAR inertial odometry
FAST-LIO2
point cloud deskewing motion compensation
2D lidar SLAM slam_toolbox
```

**Dalam bahasa Indonesia** — hasilnya jauh lebih sedikit, tapi bisa dicoba:

```
registrasi point cloud
penggabungan point cloud
algoritma ICP
segmentasi bidang RANSAC
```

> Saran: untuk topik ini, **cari dalam bahasa Inggris**. Materi berbahasa
> Indonesia sangat terbatas dan istilahnya sering tidak seragam.
