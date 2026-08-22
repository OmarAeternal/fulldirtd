# `pasak` — registrasi berjangkar untuk tempat minim fitur

Rancangan, 23 Agustus 2026. Menggantikan pencarian dengan penyelesaian pada
derajat kebebasan yang tembok tidak bisa kunci.

Terkait: `registrasi-degenerasi-tembok`, `clomerged-status`,
`SESI_registrasi_scan0064-0071.md`.

---

## 1. Masalahnya, sesudah diukur ulang

Scan di selasar FILKOM (`scan_0080`–`0083`, 13 Agustus 2026) selalu menempel
rapi ke tembok tapi salah tempat. Tiga hal ditemukan lewat pengukuran pada 23
Agustus 2026, dan ketiganya mengubah pemahaman sebelumnya.

### 1.1 Yang tidak terkunci tinggal dua derajat kebebasan mendatar

Dua jalan `outmerge` yang hanya berbeda `--range` (028 = 6 m, 029 = 15 m)
memberi sudut relatif yang **konsisten** — relatif terhadap 0083, scan 0080
duduk di −91,4° lawan −91,2°, scan 0081 di −179,4° lawan −178,9°, scan 0082 di
+88,6° lawan +88,2°. Tetapi jarak antar posisi sensor **berbeda sampai 0,8 m**
(0080↔0082: 1,41 m lawan 0,60 m).

Penugasan kasar 90° sudah benar. Geseran mendatar tidak. Ini bukan soal 6 DoF
melainkan 2.

Catatan kehati-hatian: sapuan kasar (§1.3) menemukan perbaikan yaw sebesar +6°
dan −9°. Angka itu belum kuat — ia puncak tertinggi dari 2.601 sel pada
statistik bernilai ~0,1, dan puncak semacam itu cenderung menggelembung. Yang
bisa dipegang: **yaw benar sampai orde 90°, belum tentu sampai derajat.** Karena
itu rancangan ini tidak menganggap yaw sudah selesai.

### 1.2 Rig-nya sendiri ikut ter-scan dan tidak pernah dibuang

Di keempat scan ada gugus tetap di titik asal sensor:

| scan | pusat | ukuran | titik |
|---|---|---|---|
| 0080 | (−0,14, −0,01, +1,12) | 0,20×0,39×0,54 | 311 |
| 0081 | (−0,19, +0,01, +1,03) | 0,27×0,39×0,89 | 394 |
| 0082 | (−0,20, +0,02, +1,07) | 0,31×0,39×0,91 | 379 |
| 0083 | (−0,21, +0,00, +1,08) | 0,32×0,39×0,88 | 431 |

Plus gugus kedua yang lebih kecil di z ≈ 1,6 pada 0081–0083. Ini tripod, mount,
dan operator. Ia **bergerak bersama sensor**, jadi ia selalu cocok sempurna dan
menarik ICP ke arah menumpuk-tripod. Tidak di-mask di `clomerge`, `outmerge`,
maupun `clomerged`.

### 1.3 Fitness melawan peta gabungan menggelembung

Pada pose yang `outmerge/029` nyatakan BAIK (fitness 0,61–0,85), nilai
**berpasangan langsung**:

| pasangan | fitness@10cm | tajam@5cm | terbaik di sapuan ±1,6 m / ±12° |
|---|---|---|---|
| 0080→0082 | 0,218 | 0,046 | 0,100 di yaw +6°, geser (+0,4, −0,4) |
| 0080→0081 | 0,121 | 0,024 | 0,094 di yaw +6°, geser (0,0, +0,4) |
| 0081→0082 | 0,211 | 0,050 | 0,159 di yaw 0°, geser (**+1,2**, +0,2) |
| 0082→0083 | 0,043 | 0,005 | 0,114 di yaw −9°, geser (−0,2, −1,0) |

`outmerge` menilai tiap scan melawan **peta gabungan yang sedang tumbuh** — tiga
scan lain sekaligus, kira-kira tiga kali lebih padat. Tiap titik dapat kira-kira
tiga kali lebih banyak kesempatan menemukan pasangan dalam 10 cm. Angkanya
mengikuti kepadatan peta, bukan kebenaran. Sebuah scan bisa mendapat 0,7 melawan
peta padat sambil tidak cocok dengan satu pun scan penyusunnya.

**Peta gabungan `scan_0080`–`0083` yang ada sekarang hampir pasti salah.**

### 1.4 Jurang antara dua ukuran yang tersedia

Satu scan dinilai melawan dirinya sendiri, digeser menyusuri tembok:

| geser | fitness@10cm | tajam@3cm |
|---|---|---|
| 0,0 m | 1,000 | 1,000 |
| 0,4 m | 0,459 | 0,133 |
| 0,8 m | 0,426 | 0,094 |
| 1,2 m | 0,407 | 0,083 |

`fitness@10cm` jenuh: 0,4 m dan 1,2 m tak terbedakan. `tajam@3cm` membedakan
dengan tajam. Tetapi antara **dua scan sungguhan**, tajam@3cm di seluruh jendela
±1,5 m tidak pernah melewati 0,033, dan dua sel teratas hanya berbeda 2%
(nisbah 0,98) — tidak bisa dipakai memilih. Terlalu ketat untuk bertahan
melewati perbedaan sudut pandang dan pencuplikan.

Satu ukuran terlalu longgar, satunya terlalu ketat, tidak ada yang pas di
tengah. **Inti kegagalannya ada di sini.**

### 1.5 Pencocokan konstelasi naif sudah dicoba dan gagal

Klik-maksimum pada jarak berpasangan antar titik-pusat gugus benda:

| pasangan | benda | calon | klik konsisten | sisa |
|---|---|---|---|---|
| 0080↔0082 | 11 vs 6 | 16 | 2 | — |
| 0080↔0081 | 15 vs 14 | 56 | 3 | rata 2,99 m |
| 0081↔0083 | 11 vs 21 | 85 | 3 | rata 0,86 m |

Titik-pusat gugus **tidak terulang** antar sudut pandang: dari satu sisi hanya
separuh benda yang terlihat, jadi pusatnya bias ke arah sensor sebesar kira-kira
separuh tebal benda. Bias itu lebih besar dari toleransi yang diperlukan.

Ini menutup jalan "cocokkan pakai benda secara otomatis" dalam bentuk paling
lugas, dan itulah alasan rancangan ini meminta manusia menentukan
**korespondensinya** — satu-satunya bagian yang mesin gagal — sambil tetap
menyerahkan seluruh geometrinya ke mesin.

### 1.6 Bonus: pengupasan bidang tidak stabil

Jumlah bidang latar yang ditemukan berkisar 5 sampai 8 antar scan, dan titik
menonjol mencapai 34–42% dari seluruh titik. Terlalu banyak: sisa tembok dan
kanopi bocor lewat sebagai "ciri". Perataan tanah juga bergeser mengikuti
`--range` — scan_0081 diratakan 7,35° di run 028 tapi 9,38° di run 029, selisih
2° yang berarti 17 cm meleset pada jarak 5 m.

---

## 2. Kunci matematisnya

Enam derajat kebebasan dibagi ke sumber yang masing-masing kuat di bidangnya:

| DoF | dikunci oleh | status |
|---|---|---|
| roll, pitch, z | bidang tanah | sudah jalan (`level_transform`) |
| yaw, x, y | dua benda yang sama, ditunjuk manusia | baru |

Dua jangkar memberi empat batasan untuk tiga anu: **tertentu penuh, dengan satu
sisa yang bisa diperiksa**. Tidak ada pencarian, jadi tidak ada minimum lokal,
tidak ada alias periodik, dan tembok tidak diberi kesempatan menggelincirkan
apa pun.

Inilah bedanya dengan semua yang sudah dicoba: `clomerge`, `outmerge`, dan
`clomerged` semuanya **mencari**; `pasak` **menyelesaikan**.

Dengan satu jangkar saja ia mundur ke jangkar (x, y) + normal tembok (yaw), dan
laporannya wajib menyebut bahwa yaw-nya berasal dari tembok, bukan dari jangkar.

### Kenapa klik yang kasar tetap cukup

Klik manusia hanya menetapkan **benda mana berpasangan dengan benda mana**.
Posisi tepatnya tidak diambil dari klik. Alur nilainya:

1. Kabsch 2-D pada titik-pusat jangkar → pose kasar, meleset kira-kira ±0,2 m
   dan ±6° (bias sudut pandang dari §1.5 belum hilang di sini).
2. Perapian sadar-degenerasi menyelesaikan sisanya.

Langkah 2 aman justru karena langkah 1 mendarat di lembah yang benar. Ketelitian
bukan tanggung jawab manusia; pemilihan lembah yang tanggung jawabnya.

---

## 3. Unit

Tiap unit berdiri sendiri, punya satu tujuan, dan bisa diuji terpisah.

### 3.1 `buang_rig(xyz, atlas, radius=0.7) -> (xyz, jumlah_dibuang)`

Buang titik yang **menonjol dan** berada dalam silinder `radius` dari sumbu
sensor (x=0, y=0), pada ketinggian berapa pun.

Syarat "menonjol dan", bukan "semua dalam radius": scan_0081 punya bidang tembok
di x = +0,49 — di dalam radius — dan tembok itu harus tetap hidup. Hanya yang
bukan bagian dari bidang latar yang dibuang.

Melaporkan berapa titik dibuang supaya bisa diaudit.

### 3.2 `kerangka_tanah(xyz) -> T`

`outmerge.level_transform` yang ada, dengan satu perubahan: bidang tanah
**selalu dicari dari potongan 6 m**, tidak peduli `--range` yang diminta
pengguna. Menyembuhkan ketidakstabilan §1.6.

Mengunci roll, pitch, z.

### 3.3 `benda(xyz) -> list[Benda]`

Kupas bidang (`clomerged.plane_atlas`) → titik menonjol → DBSCAN (eps 0,12 m,
min 10 titik pada voxel 0,03 m) → saring:

- ≥ 60 titik
- tapak mendatar < 1,5 m
- tinggi < 2,5 m

Saringan itu sudah diuji pada data asli: ia membuang sisa tembok dan kanopi yang
bocor lewat pengupasan sambil menyisakan 6–21 benda per scan.

Tiap `Benda` membawa: `pusat`, `jumlah_titik`, `ukuran` (xyz), `tinggi_dari_tanah`,
`jarak_ke_tembok`, dan awan titiknya sendiri.

### 3.4 Penyerahan ke `pcs`

`pasak siapkan` menulis, untuk tiap scan:

- `pasak/<nama>_benda.ply` — hanya titik benda, tiap benda satu warna tetap
- `pasak/<nama>_benda.json` — ID, pusat, sifat tiap benda

Lalu **usulan pasangan otomatis** yang diperingkat kemiripan sifat (tinggi,
jumlah titik, ukuran) ditulis ke `pasak/usulan.json`. Manusia membenarkan atau
membetulkan; ia tidak mulai dari nol.

### 3.5 `pcs` — modul `pasak.js` + satu endpoint

Yang sudah ada dan dipakai ulang: raycaster picking dengan pembeda klik-vs-putar
4 px (`measure.js:127`, `measure.js:359`), multi-layer dengan visibilitas
(`layers.js`), `GET /open` by path.

Yang ditambah:

- `frontend/pasak.js` — mode baru. Dua layer benda dimuat, klik satu bercak di
  scan A lalu bercak pasangannya di scan B, pasangan tercatat dan tergambar
  sebagai garis penghubung. Klik memilih **benda**, bukan titik: setiap titik di
  `<nama>_benda.ply` membawa ID bendanya di kanal warna (ID = R + 256·G, kanal B
  untuk tampilan), jadi raycaster mengenai satu titik lalu ID-nya dibaca
  langsung. Tidak ada pencarian tetangga terdekat, tidak ada ambiguitas di tepi
  antar benda.
- `POST /pasak/pasangan` di `backend/server.py` — terima daftar pasangan, tulis
  ke `pasak/pasangan.json`. Kira-kira 30 baris.

pcs tetap **hanya pemetik**. Seluruh geometri ada di `pasak.py`. Batas itu
dijaga supaya keduanya bisa diuji sendiri-sendiri.

### 3.6 `pasang(benda_a, benda_b, pasangan) -> T`

1. Kabsch 2-D pada pusat jangkar → yaw, x, y. Dua jangkar cukup, dan sudah
   memberi satu sisa yang bisa diperiksa (lihat §5); tiga atau lebih memberi
   sisa per-jangkar sehingga penunjuk yang salah bisa ditunjuk namanya.
2. **Perapian sadar-degenerasi.** ICP bertingkat seperti sekarang
   (`ICP_SCALES = (0.60, 0.30, 0.15, 0.08)`), tetapi arah lemah — dari
   `clomerged.weak_direction`, yang sudah menghitung penguraian eigen Hessian
   ICP — diredam terhadap jawaban jangkar. Setiap pembaruan pose diproyeksikan,
   dan komponen sepanjang arah lemah dikalikan faktor redam `REDAM = 0.1`
   (dapat diatur lewat `--redam`). Nilai 0 berarti arah lemah dibekukan penuh
   pada jawaban jangkar; 1 berarti tanpa redaman, yaitu perilaku ICP sekarang.
   Baku 0,1 memberi ICP ruang beberapa sentimeter untuk memperbaiki kesalahan
   pusat jangkar tanpa memberinya ruang untuk menggelincir semeteran.

ICP boleh memoles; ia tidak boleh menggelincir balik. Inilah bedanya dengan
`unslide` di `clomerged`, yang **menyapu** arah lemah dan karena itu masih bisa
mendarat di puncak alias yang salah (tercatat: `--slide-range 3.0` menggeser
scan 0074 dari −0,06 m ke −1,90 m).

### 3.7 `lapor(...)`

Nilai **berpasangan saja, tidak pernah melawan peta gabungan.** Ini bukan
pilihan gaya: menilai melawan peta yang tumbuh adalah cara `outmerge`
menghasilkan 0,61–0,85 untuk pose yang berpasangan hanya 0,04–0,22 (§1.3).

Tiap pasangan melaporkan fitness@10cm, tajam@3cm, dan **jumlah titik yang
benar-benar bertampalan** — tanpa yang ketiga, dua yang pertama tak bisa
ditafsirkan. Tiap pose mencatat asal-usulnya: jangkar mana, dari siapa, dan
apakah yaw-nya dari jangkar atau dari tembok.

---

## 4. Alur data

```
scan_00XX.mcap
   └─ mcaptopc ──► .ply

pasak siapkan scan_008[0-3]_1sweep_0.mcap
   ├─ buang_rig ──► kerangka_tanah ──► benda
   └─ tulis  pasak/<nama>_benda.ply
             pasak/<nama>_benda.json
             pasak/usulan.json

pcs (mode pasak)
   └─ manusia menunjuk ──► POST /pasak/pasangan ──► pasak/pasangan.json

pasak selesaikan
   ├─ baca pasangan.json
   ├─ pasang  (Kabsch 2-D → perapian sadar-degenerasi)
   └─ tulis  pasak/merged.ply
             pasak/merged_check.ply
             pasak/laporan.txt
```

Dua fasa terpisah dengan berkas di antaranya, bukan satu proses interaktif:
ekstraksi bisa diulang tanpa mengulang penunjukan, dan penunjukan bisa diperbaiki
tanpa mengulang ekstraksi.

---

## 5. Penanganan kesalahan

| keadaan | perilaku |
|---|---|
| benda < 2 di salah satu scan | tolak pasangan itu, sebut nama scan-nya, lanjut ke pasangan lain |
| hanya 1 jangkar diberikan | pakai normal tembok untuk yaw, laporan **wajib** menyebutnya |
| tidak ada bidang tembok saat mode 1-jangkar | gagalkan pasangan itu; jangan mengarang yaw |
| 2 jangkar, selisih jarak \|d(a₁,a₂) − d(b₁,b₂)\| > 0,3 m | peringatkan: hampir pasti salah tunjuk. Dua jangkar memberi 4 batasan untuk 3 anu, dan sisa satu-satunya itu justru selisih jarak ini — murah dan tajam |
| ≥3 jangkar, sisa Kabsch > 0,5 m | peringatkan, sebut jangkar mana yang paling menyimpang |
| perapian menggeser > 0,5 m dari jawaban jangkar | peringatkan; itu tanda redamannya bocor |
| pasangan.json menyebut ID benda yang tidak ada | gagalkan dengan pesan jelas, jangan diam-diam dilewati |
| graf pasangan tidak terhubung | bangun peta untuk tiap komponen terhubung, laporkan bahwa ada lebih dari satu |

Prinsipnya satu: **mengaku lebih baik daripada menebak.** Sudah ada tiga sumber
kepercayaan diri palsu yang terukur di proyek ini; jangan tambah yang keempat.

---

## 6. Pengujian

TDD, mengikuti pola `test_*.py` yang sudah ada di `ros2_ws/cloudcom/`.

**Adegan buatan** (tembok + tanah + tiga benda, geometri diketahui persis):

- `pasang` mengembalikan geseran 1,2 m yang disuntikkan, sisa < 2 cm
- `pasang` mengembalikan yaw 8° yang disuntikkan
- dengan satu jangkar + tembok, hasilnya sama dalam 5 cm
- perapian sadar-degenerasi tidak menggeser > 5 cm dari jawaban jangkar walaupun
  tembok ditambah 10× lebih banyak titik

**`buang_rig`:**

- gugus di titik asal terbuang
- bidang tembok pada jarak 0,49 m dari sumbu **tidak** terbuang
- jumlah yang dilaporkan cocok dengan jumlah yang benar-benar hilang

**Penilaian:**

- nilai berpasangan **tidak** naik ketika awan acuan digandakan kepadatannya —
  uji yang langsung menangkap cacat §1.3
- `tajam@3cm` = 1,0 untuk awan melawan dirinya sendiri tanpa geseran

**Determinisme:**

- dua eksekusi berturut-turut memberi matriks yang identik
  (`o3d.utility.random.seed`; bidang lewat SVD, bukan `segment_plane`, di jalur
  yang hasilnya masuk ke laporan)

**Data asli** (`scan_0080`–`0083`), tanpa nilai ambang yang diklaim di muka —
angkanya dicatat sebagai garis dasar, dan penilaian akhir tetap mata manusia di
pcs, sesuai pelajaran dari `clomerged-status`.

---

## 7. Cakupan

Di dalam: `scan_0080`–`0083`, satu peta yang benar dan terverifikasi.

Di luar untuk sekarang: `scan_0072`–`0075` (masalah sama, dikerjakan setelah ini
terbukti) dan `scan_0064`–`0071` (masalahnya **berbeda** — ambiguitas sudut
ruangan yang mirip satu sama lain, bukan geseran sepanjang tembok; `pasak` mungkin
membantu tapi belum tentu jawabannya).

Tidak diubah: `clomerge`, `outmerge`, `clomerged` tetap utuh sebagai pembanding.
Penting untuk skripsi, dan penting supaya ada tempat kembali kalau `pasak`
meleset.
