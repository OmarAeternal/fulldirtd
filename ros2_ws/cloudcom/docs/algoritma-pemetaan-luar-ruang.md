# Catatan rancangan: algoritma pemetaan untuk tempat luar-ruang

Ditulis 2 September 2026. Isinya bukan teori — semuanya berasal dari angka yang
terukur pada `scan_0064`–`0083` (selasar FILKOM), dan tiap klaim menyebut
angkanya supaya bisa dibantah.

**Asumsi yang dipakai di sini:** objek yang dipindai sama dan terjangkau dari
tiap posisi berdiri; ciri objek utama, objek lain, maupun pola area bisa dibaca.
Yang TIDAK diasumsikan: tanah rata, tembok cukup untuk registrasi, atau metrik
tampalan bisa dipercaya.

---

## 1. Kenapa cara indoor gagal di luar ruang

Empat sebab, semuanya terukur:

**a. Tembok mendominasi jumlah titik, tapi tidak membawa informasi posisi.**
Tembok + tanah mengunci 5 dari 6 derajat kebebasan. Yang tersisa geseran
mendatar menyusuri tembok, dan justru itu yang tidak terkunci. Di FILKOM hanya
huruf timbul (relief 5–15 cm) yang bisa menguncinya, dan jumlah titiknya kalah
jauh dari tembok.

**b. Liputan tiap posisi berdiri SEMPIT, bukan 360°.** Terukur: scan_0083
menaruh 96% titiknya di sektor −60°…+60°, dan 6 dari 12 arah kosong. Akibatnya
dua posisi berdiri bisa hampir saling membelakangi dan hanya berbagi 200–700
titik. Semua algoritma yang mengandalkan tampalan besar langsung lumpuh.

**c. Tanah tidak rata, dan tidak boleh dianggap rata.** Tanah luar ruang punya
kemiringan buangan air. Terukur: z rata titik tanah bergeser 10–17 cm antara
cincin 1–2 m dan 5–6 m dari sensor. Meratakan ke tanah lokal memiringkan seluruh
scan.

**d. Rig sendiri ikut ter-scan.** Gugus tetap 311–431 titik di titik asal sensor
(tripod/mount/operator). Ia bergerak BERSAMA sensor, jadi selalu cocok sempurna
dan menarik registrasi ke arah menumpuk-tripod. Harus dibuang sebelum apa pun.

---

## 2. Empat sumber kepercayaan diri palsu — semuanya terukur MEMIHAK jawaban salah

Ini bagian terpenting catatan ini. Bukan "metrik ini lemah", tapi "metrik ini
memberi nilai LEBIH TINGGI untuk jawaban yang salah".

**1. Metrik jarak longgar jenuh.** `fitness@10cm` jauh lebih longgar daripada
jarak antar titik (3–4 cm). Satu scan dinilai melawan DIRINYA SENDIRI, digeser
menyusuri tembok, masih memberi 0,459 pada 0,4 m dan 0,407 pada 1,2 m. Ia
mengukur "apakah aku menempel di suatu permukaan", bukan "apakah aku di tempat
yang benar".

**2. Menilai melawan peta gabungan menggelembung.** Menilai tiap scan melawan
peta yang sedang tumbuh (3 scan lain, ~3× lebih padat) memberi 0,61–0,85 "BAIK"
untuk pose yang diukur BERPASANGAN cuma 0,04–0,22. Angkanya mengikuti kepadatan,
bukan kebenaran. **Selalu nilai berpasangan, dan selalu sertakan jumlah titik
bertampalan** — tanpa itu angkanya tak bisa ditafsirkan.

**3. Metrik ketat pun bisa memihak.** `tajam@3cm` dibuat untuk mengobati (1).
Terukur pada satu tepi: pose yang benar 0,091; pose yang meleset SATU huruf
0,109; pose yang meleset lebih jauh 0,111. Makin tergelincir makin tinggi
nilainya, karena adegannya berulang. Dan `fitness@10cm` pada tepi lain memberi
0,293 untuk pose yang meleset 0,73 m lawan 0,170 untuk yang benar.

**4. Sisa jangkar buta terhadap salah-pasang.** Dua jangkar yang kebetulan
berjarak pas SELALU bisa dipasangkan rapi. Terukur dua kali:
- menukar dua huruf memutar pose **180°** tanpa mengubah sisa jangkar sedikit
  pun (0,023 m di kedua pilihan)
- pemasangan yang salah punya sisa **0,023 m** sementara yang benar 0,199 m —
  yang salah tampak empat kali lebih meyakinkan

**Kesimpulan yang harus dipegang:** di adegan berulang dan bertampalan tipis,
metrik tampalan bukan sekadar lemah — ia menyesatkan secara sistematis. Jangan
pernah memilih antara dua pose berdasarkan metrik tampalan saja.

---

## 3. Wasit yang sah

Kalau metrik tidak bisa jadi wasit, apa yang bisa. Empat yang terbukti bekerja:

**a. Benda yang TIDAK ikut dijangkar.** Aturan pokoknya: **jangan pernah
menjangkar dengan semua benda yang dikenali; selalu sisakan minimal satu sebagai
pemeriksa.** Jangkar dipaksa berimpit secara definisi, jadi ia tak bisa
memvonis dirinya sendiri. Benda bebas bisa. Terukur: pemasangan yang salah
meninggalkan benda ketiga **melayang 1,42 m di luar deret**, dan tidak ada satu
pun angka di laporan yang menyebutkannya — hanya mata manusia yang melihatnya.

**b. Benda yang tersisa tanpa penjelasan = tanda bahaya.** Kalau sesudah
dipasang masih ada benda besar yang tidak jatuh di mana-mana, pemasangannya
salah, seberapa pun bagus angkanya.

**c. Kesinambungan struktur di LUAR ciri penjangkar.** Dinding panjang harus
tersambung dari beberapa posisi berdiri dengan jangkauan yang saling
melengkapi. Pose yang salah membuat dinding besar berdiri sendirian tanpa
sambungan. Ini wasit yang paling sulit ditipu, karena ia memakai bagian adegan
yang tidak dipakai menjangkar.

**d. Arah hadap harus masuk akal sebagai satu himpunan.** Kalau pemindaian
dilakukan dari empat sisi, arah hadap keempatnya harus mengisi empat kuadran.
**Dua posisi berdiri dengan yaw sama persis adalah tanda bahaya** — begitulah
kesalahan 180° ketahuan.

**Tambahan:** algoritma pencari kasar (mis. FPFH+RANSAC) ternyata memberi SUDUT
yang bisa dipercaya (terukur cocok dalam 0,2°–2,6° dengan jawaban akhir) meski
GESERANnya salah. Pakai ia sebagai penghasil hipotesis sudut, jangan sebagai
jawaban.

---

## 4. Kemiringan — dua masalah berbeda yang sering dicampur

Ini bagian yang paling banyak memakan waktu, dan sumber kekeliruan terbesar.

### 4a. Bedakan dua pertanyaan

1. **Tegak mutlak** — mana yang benar-benar searah gravitasi?
2. **Kesepakatan relatif** — apakah keempat posisi berdiri sepakat soal "atas"?

**Untuk menggabungkan peta, (2) jauh lebih penting daripada (1).** Kalau semua
scan miring sama besar, petanya utuh dan cuma perlu satu putaran global di
akhir. Kalau mereka miring berbeda-beda, potongannya saling menembus dan tidak
ada putaran global yang bisa menyelamatkannya.

### 4b. Meratakan ke tanah LOKAL adalah cacat, bukan penyederhanaan

Lantai itu SATU benda fisik dan ia memang miring. Meratakan tiap scan ke
petak lantainya sendiri memaksa empat potongan lantai yang sama menjadi datar
dengan cara yang berbeda-beda. Sesudah itu potongannya tidak bisa disambung
lagi. Terukur: sesudah perataan tanah, tembok yang seharusnya tegak condong
4,2–8,7°, dan keempat scan saling miring sampai **9,6°**.

**Ciri untuk membedakan sebabnya:** ukur jarak tegak lurus sensor ke tembok pada
beberapa pita ketinggian, dengan bidang dipasang TERPISAH tiap pita.
- Kalau angkanya berubah **lurus** terhadap ketinggian → seluruh scan terputar
  kaku (masalah acuan tegak).
- Kalau **melengkung** → geometri di dalam scan yang terdistorsi (masalah
  kalibrasi sensor).
Terukur di sini: lurus, sisa ≤1 cm sepanjang 1,6 m. Jadi ini masalah acuan,
bukan distorsi. Hipotesis skala sudut elevasi sudah diuji dan **ditolak** — ia
hanya cocok bila tinggi sensor dipaksa ke 0,00 m, tidak fisis.

### 4c. Memakai tembok sebagai acuan tegak: dicoba, MEMPERBURUK

Tembok dibangun tegak lurus gravitasi, jadi kelihatannya acuan yang lebih baik
daripada tanah. Terukur: setelah ditegakkan ke tembok, ketegakan tembok memang
membaik (8,2° → 0,1°), tapi **kesambungan lantai memburuk dari 9,3 cm jadi
21,1 cm**.

Sebabnya penting dan berlaku umum: satu arah normal tembok hanya mengunci SATU
dari dua derajat kemiringan. Memiringkan awan pada sumbu normal tembok
memetakan bidang tembok ke dirinya sendiri, jadi tembok buta terhadapnya. Sumbu
yang tersisa tetap memakai jawaban tanah — dan **kerangka campuran seperti itu
tidak konsisten antar scan**.

> **Aturan:** untuk menyambung, satu acuan yang konsisten lebih berharga
> daripada dua acuan yang masing-masing lebih benar sendiri-sendiri.

Tembok baru layak jadi acuan tegak bila ada **dua arah normal yang berbeda**
(sebaran azimut > 20°). Di selasar hanya ada satu arah (sebaran 0,1–0,5°).

### 4d. Yang berhasil: paksa scan SEPAKAT soal tinggi lantai

Aturannya sederhana dan tidak butuh model permukaan: **kalau dua posisi berdiri
melihat petak lantai yang sama, tinggi lantai yang mereka laporkan harus sama.**
Lantai boleh semiring apa pun — yang penting satu.

Terukur: beda tinggi lantai antar scan turun dari 9,3 cm (rata) / 4,4 cm
(median) menjadi **2,5 cm / 1,5 cm**. Koreksi yang diperlukan kecil dan masuk
akal — paling besar 2,0° dan 4 cm — jauh lebih kecil daripada koreksi tembok
(sampai 8,4°). Koreksi kecil untuk perbaikan besar adalah tanda cacat yang benar
sedang disentuh.

**Cara memasangnya, dan ini yang membuat perbedaan antara berhasil dan gagal:**

- **Jangan** memasang permukaan lantai bebas lalu menarik tiap scan ke sana.
  Dicoba: menyimpang, sisa naik 17 → 25 cm dan koreksi membengkak sampai 35°.
  Sebabnya permukaannya ikut bergerak mengikuti scan — tidak ada yang mengikat.
- **Lakukan** minimisasi selisih ANTAR PASANGAN scan per petak. Tidak ada model
  permukaan, jadi tidak ada yang bisa melar.
- Anu per scan cukup tiga: roll, pitch, geser tegak.
- Kunci gauge dengan memaksa rata-rata koreksi nol, bukan dengan mematok satu
  scan sebagai acuan — agar tak ada scan yang diistimewakan.
- Beri tarikan lemah ke koreksi nol (ridge) sebagai penjaga degenerasi.
- Ulangi 4–5 kali; petak berubah keanggotaan saat lantai mendekat.

**Penyaringan petak menentukan segalanya.** Petak "lantai" yang tercemar benda
di atasnya (tepi trotoar, kaki huruf, pot) membuat ukuran tak berarti: dengan
semua petak, beda terukur 17,6 cm; dengan hanya petak yang sebaran tingginya
< 5 cm, 11,5 cm — dan yang pertama tak bisa diperbaiki karena selisihnya memang
bukan soal kemiringan. Syarat yang dipakai: petak 30 cm, minimal 20 titik,
sebaran persentil 10–90 di bawah 5 cm, pita |z| < 35 cm.

**Batasnya, sebutkan jangan sembunyikan:** hanya ditemukan ~25–36 petak yang
dilihat ≥2 scan dengan syarat itu — sekitar 3 m² lantai bersama. Itu tipis untuk
9 anu bebas. Ia bekerja di sini, tapi jangan diandalkan bila liputan lebih
buruk.

### 4e. Tegak mutlak diselesaikan TERAKHIR, sekali, untuk semua

Setelah scan sepakat satu sama lain, seluruh peta mungkin masih miring serempak
(di sini tembok masih condong 4–8°). Itu kesalahan yang sama untuk semuanya —
satu putaran global menyelesaikannya, dan ia tidak bisa merusak kesambungan.
Acuannya bisa tembok, IMU, atau apa pun yang tersedia. **Jangan dikerjakan lebih
awal**; kalau dikerjakan per scan sebelum mereka sepakat, hasilnya seperti 4c.

---

## 5. Urutan yang disarankan

1. **Buang rig.** Silinder di titik asal sensor. Syaratnya "menonjol DAN di dalam
   radius", supaya tembok dekat tidak ikut terbuang.
2. **Ratakan kasar ke tanah.** Bukan karena benar, tapi karena butuh kerangka
   awal yang seragam. Selalu dari potongan berjari-jari tetap, jangan mengikuti
   `--range`, supaya jawabannya tidak berubah mengikuti parameter lain.
3. **Petik benda.** Kupas bidang latar, guguskan sisanya, saring ukuran.
4. **Kenali korespondensi.** Bagian yang mesin gagal; serahkan ke mata manusia
   (lihat §6).
5. **Selesaikan pose mendatar** dari jangkar. Yaw/x/y saja — roll/pitch/z sudah
   dari langkah 2.
6. **Sambungkan lantai** (§4d). Ini memperbaiki kemiringan relatif.
7. **Ulangi langkah 5** dengan jangkar yang sama; pose mendatar berubah sedikit
   karena kemiringan berubah.
8. **Tegakkan global** (§4e), sekali, di akhir.

Langkah 5–7 boleh diulang dua kali. Terukur: sesudah langkah 6, titik
bertampalan naik dari 398 ke 2.323 pada satu tepi — hampir enam kali lipat.

---

## 6. Ciri mana yang layak dipakai, dan jebakannya

**Yang bekerja:** benda tegak berdiri sendiri, tinggi ≥ 1 m, terisi ≥ 500 titik,
menempel tanah, berbentuk khas. Huruf timbul, tiang, pilar.

**Jebakan yang terukur:**

- **Titik-pusat benda yang cuma terlihat sebagian itu BIAS**, ke arah sensor,
  sebesar ~separuh tebal benda. Terukur: deret huruf yang sama memberi jarak
  0,78 · 1,00 · 1,46 m dari satu posisi dan 0,79 · 1,26 · 1,76 m dari posisi
  lain — menciut makin jauh hurufnya. **Pilih jangkar dari benda yang paling
  banyak titiknya, bukan yang paling dekat di daftar.** Kesalahan ini dilakukan
  di sesi ini: dipakai benda 188 titik padahal ada yang 1.173 titik.
- **Ciri yang SEGARIS tidak bisa memvonis pergeseran sepanjang garis itu.**
  Semua huruf satu deret terletak di satu garis, jadi apa pun yang memetakan
  garis itu ke dirinya sendiri lolos pemeriksaan "huruf mendarat di huruf yang
  benar". Butuh ciri di luar garis.
- **Ciri simetris tertukar tanpa ketahuan.** Huruf O dan M sama-sama simetris
  kiri-kanan; dari belakang keduanya tampak sama. Tertukarnya memutar pose 180°
  tanpa mengubah angka apa pun.
- **Deret berjarak hampir seragam menghasilkan alias periodik.** Tergeser satu
  huruf tetap "menempel rapi". Redaman gerak harus lebih kecil daripada jarak
  pengulangan adegan — bukan angka tetap. Terukur: redaman baku memberi 0,87 m
  padahal jarak antar huruf 0,78 m; cukup untuk melompat satu huruf.
- **Ambang ukuran membuang benda sungguhan tanpa bersuara.** Terukur: huruf
  terlebar ditolak karena tapaknya 1,65 m melewati ambang 1,50 m — padahal ia
  benda terpadat kedua di scan itu dengan 1.956 titik. Setiap ambang penyaring
  harus bisa dilaporkan: benda apa yang dibuang, dan kenapa.

**Pola area** (jarak antar benda, urutan, susunan) berguna sebagai
**pemendek daftar**, bukan bukti. Terukur: pencocokan konstelasi otomatis lewat
titik-pusat gugus memberi 2–3 pasang konsisten dari 16–85 calon, dengan sisa
0,86–3,0 m. Sebabnya fisik (bias titik-pusat), bukan bug — **menambah parameter
tidak menolong.**

---

## 7. Peran manusia, dan bagaimana meminimalkannya

Yang mesin gagal hanya SATU hal: menentukan benda A di sini = benda mana di
sana. Seluruh geometrinya mesin bisa. Rancangan yang baik memisahkan keduanya
dengan tegas: manusia menunjuk korespondensi, mesin menyelesaikan sisanya
**tanpa pencarian** — jadi tanpa minimum lokal dan tanpa alias periodik.

Dua jangkar + tanah sudah menentukan penuh (4 batasan untuk 3 anu, dengan satu
sisa yang bisa dicek). Satu jangkar + normal tembok juga bisa, tapi yaw-nya lalu
bersandar pada tembok dan laporannya wajib mengatakan begitu.

**Yang paling menolong mata manusia** (terukur mengubah tugas dari mustahil jadi
mudah):
- kartu rupa tiap benda, **diperbesar sendiri-sendiri** — sumbu tetap untuk
  semua benda membuat benda kecil tak terbaca dan sempat membuat huruf tak
  dikenali sama sekali
- peta tampak atas per scan, dengan scan lain ditampilkan abu-abu — ini cara
  termudah membedakan "salah tempat" dari "tidak terliput"
- daftar calon yang sudah dipendekkan oleh kemiripan sifat, dengan peringatan
  jelas bahwa itu bukan bukti

---

## 8. Sudah diuji dan GAGAL — jangan diulang

- Menaikkan kerapatan sapuan pencarian (`--seeds`, `--step-deg`) untuk memecah
  ambiguitas sudut ruangan: mengulang jawaban yang sama. Bukan soal kerapatan.
- Pencocokan konstelasi otomatis lewat titik-pusat gugus: 2–3 klik dari puluhan
  calon, sisa 0,86–3,0 m. Sebabnya fisik.
- Melonggarkan ambang metrik ketat di bawah derau sampling: semuanya GAGAL.
- Menyapu arah lemah mencari puncak: masih mendarat di puncak alias yang salah;
  tercatat menggeser satu scan dari −0,06 m ke −1,90 m.
- Meratakan per scan ke tembok sebelum scan saling sepakat: memperburuk
  kesambungan lantai hampir dua kali lipat (§4c).
- Memasang permukaan lantai bebas lalu menarik scan ke sana: menyimpang (§4d).
- Model skala sudut elevasi untuk menjelaskan kemiringan: ditolak, hanya cocok
  bila tinggi sensor dipaksa nol (§4b).

---

## 9. Yang masih terbuka

- Perataan lantai bersandar pada ~3 m² lantai bersama. Belum diketahui berapa
  batas bawahnya sebelum ia gagal.
- Tegak mutlak belum diselesaikan; peta akhir masih miring serempak 4–8°.
- Belum ada cara otomatis memeriksa "ada benda tersisa yang melayang" — padahal
  itu wasit paling tajam yang ditemukan. Layak dijadikan pemeriksaan wajib di
  laporan: untuk tiap benda yang dikenali tapi tidak dijangkar, sebutkan ia
  mendarat di mana dan seberapa jauh.
