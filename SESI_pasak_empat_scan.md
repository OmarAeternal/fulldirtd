# Sesi `pasak` lanjutan — 1 September 2026

Keempat scan `scan_0080`–`0083` **berhasil digabung**. Catatan serah-terima.
Baca ini dulu sebelum lanjut.

Melanjutkan sesi ini: jalankan `claude --resume` dari `~/riset td` lalu pilih
sesi bertanggal 1 September 2026. Sesi sebelumnya:
`SESI_pasak_scan0080-0083.md`.

---

## Hasilnya

`out/_pasak/002` — keempat scan tergabung, satu kelompok. Slot `001` versi lama,
jangan dipakai.

```bash
cd ~/"riset td/pointcloud_studio"
.venv/bin/python pcs.py ~/"riset td/cloudcom/out/_pasak/002/merged_check.ply"
```

Penilaian mata pengguna: *"posisinya hampir mendekati asli, masalah terlihat
hanya di kemiringan"*.

| scan | yaw pasak | yaw outmerge | beda | posisi sensor | warna di merged_check |
|---|---|---|---|---|---|
| 0083 | 0,0° | 0,0° | — | (0,00 · 0,00) barat | merah |
| 0080 | −91,7° | −91,4° | 0,3° | (2,80 · 4,92) utara | hijau |
| 0081 | +178,4° | −179,4° | 2,2° | (3,47 · 1,52) timur | biru |
| 0082 | +88,1° | +88,6° | 0,5° | (2,82 · −5,57) selatan | kuning |

Urutan warna = urutan `list(pose)` di `_tulis_hasil`, ikut urutan BFS dari akar.
**Jangan diduga; baca urutannya di `laporan.txt` bagian "Asal-usul tiap pose".**

Adegannya selasar dengan **dua dinding sejajar**: dinding huruf di x≈1,33 dan
dinding lain di x≈4,05 (kerangka 0083). Huruf FILKOM timbul ~1,8 m, berdiri
bebas, dipindai dari **empat sisi** (depan-belakang-kanan-kiri, kanan-kiri agak
serong).

## `pasangan.json` yang dipakai

Semua dijalankan dengan `--redam 0.0`.

```json
{"pasangan": [
  {"a":"scan_0080_1sweep_0","b":"scan_0083_1sweep_0","jangkar":[[1,0],[2,1]]},
  {"a":"scan_0082_1sweep_0","b":"scan_0083_1sweep_0","jangkar":[[8,5],[7,4]],"icp":false},
  {"a":"scan_0081_1sweep_0","b":"scan_0083_1sweep_0","jangkar":[[8,2],[7,7]],"icp":false}
]}
```

Slot `002` dibuat dengan `siapkan ... --max-tapak 1.8`. **Nomor benda di slot
lain akan berbeda** — jangan salin angka ini ke slot baru tanpa memetakan ulang.

Huruf di slot 002: 0080 F#0 I#1 L#2 K#3 O#7 · 0083 I#0 L#1 K#2 O#4 M#5 ·
0082 M#8 O#7 · 0081 K#8.

## TIGA TITIK BUTA VERIFIKASI — ini temuan pokok sesi ini

Ketiganya terukur **memihak jawaban yang salah**, bukan sekadar lemah.

1. **`tajam@3cm`.** Tepi 0080: redam 0 (benar) → 0,091; redam 0,1 (meleset satu
   huruf) → 0,109; redam 1,0 (meleset lebih jauh) → 0,111. Makin tergelincir
   makin tinggi nilainya.
2. **`fitness@10cm`.** Tepi 0082: 0,293 untuk pose yang meleset 0,73 m lawan
   0,170 untuk yang benar. Ia mengikuti kepadatan tampalan.
3. **Identitas huruf** — wasit yang dipakai untuk menolak dua di atas. Semua
   huruf SEGARIS, jadi apa pun yang memetakan garis huruf ke dirinya sendiri
   lolos uji "I mendarat di I". Terukur: menukar O dengan M memutar pose **180°**
   tanpa mengubah sisa jangkar sedikit pun (0,023 m di kedua pilihan). O dan M
   sama-sama simetris kiri-kanan sehingga dari belakang tampak sama — mata pun
   bisa tertukar, dan memang tertukar sekali di sesi ini.

**Wasit yang akhirnya sah: kesinambungan struktur DI LUAR ciri penjangkar.**
Dinding x≈4,05 tersambung penuh dari empat scan dengan jangkauan y yang saling
melengkapi (0082 −8…−2, 0083 −4…1, 0080 2…6, 0081 −4…3). Pose yang salah membuat
dinding besar berdiri sendirian tanpa sambungan.

**Wasit kedua: arah hadap harus mengisi empat kuadran.** Dua scan dengan yaw
sama persis adalah tanda bahaya — begitulah tukar O/M ketahuan.

## Liputan azimut tiap scan SEMPIT — bukan 360°

0083 menaruh 96% titiknya di sektor −60°…+60°, 6 dari 12 arah kosong. Yang lain
3–4 arah kosong. Akibatnya tampalan cuma 200–700 titik, dan ada daerah yang
hanya satu scan meliputnya. **Jangan tafsirkan daerah satu-warna sebagai pose
salah tanpa mengecek liputan dulu** — sempat menyesatkan sesi ini setengah jam.

## Dua cacat `pasak` yang diperbaiki

Commit `7a00207` di branch `pasak-tapak-dan-icp`. 34 tes lulus.

1. `BENDA_MAX_TAPAK = 1.5` membuang huruf M tanpa bersuara — tapak M 1,65 m
   (huruf terlebar) padahal 1.956 titik, terpadat kedua di scan_0083. Kini ada
   `siapkan --max-tapak`, bawaan tetap 1,5.
2. **ICP merusak jangkar bila tampalan tipis.** Tepi 0082: jangkar tepat 1,1 cm,
   ICP menyeretnya 1,62 m. `--redam` tak menolong — ia hanya meredam arah lemah,
   seretannya tegak lurus tembok. Kini `pasangan.json` menerima `"icp": false`
   per tepi. Di tepi 0080 (tampalan 723 titik) ICP justru memperbaiki
   15 cm → 5 cm, jadi keputusannya memang per tepi.

## BELUM DIKERJAKAN

### 1. Kemiringan sisa — ini yang paling terlihat

Dinding yang seharusnya tegak condong, konsisten di dalam tiap scan:

| scan | dinding 1 | dinding 2 | jarak dinding |
|---|---|---|---|
| 0082 | +8,1° | +8,3° | 0,95 m |
| 0080 | +7,6° | +7,2° | 1,09 m |
| 0083 | +4,3° | +4,6° | 1,29 m |
| 0081 | −1,4° | −1,4° | 2,03 m |

Keempat scan saling miring sampai **9,6°**.

**Sudah dipastikan BUKAN:**
- bukan kesalahan registrasi — perataan itu transformasi kaku, jadi sudut
  tanah–dinding tak berubah olehnya; kalau sesudah diratakan dinding condong 8°,
  di data mentah pun tanah dan dinding bertemu di 82°
- bukan distorsi sensor — dindingnya LURUS (sisa ≤1 cm sepanjang 1,6 m), bukan
  melengkung; model skala sudut elevasi sudah diuji dan **ditolak** (hanya cocok
  bila tinggi sensor dipaksa 0,00 m, tidak fisis). **Jangan ulangi uji ini.**

**Sumbernya:** `kerangka_tanah` meratakan tiap scan ke bidang tanah lokalnya,
dan tanah FILKOM tidak rata (z rata titik tanah bergeser 10–17 cm antara cincin
1–2 m dan 5–6 m). `pasak` secara rancangan **mengunci** roll/pitch dari tanah,
jadi ia tak akan pernah memperbaikinya sendiri.

**IMU sudah dikesampingkan atas keputusan pengguna** — jangan usulkan lagi tanpa
alasan baru.

**Jalan yang tersisa:** sesudah jangkar terpasang, perhalus roll/pitch dengan
menyejajarkan dinding bersama antar scan. Datanya sudah ada — dua dinding
sejajar terlihat oleh keempat scan dan tiap scan memberi normal dinding yang
konsisten di dalam dirinya sendiri. Cukup untuk dua derajat kebebasan tersisa.

### 2. yaw 0081 meleset 2,2°

Terbesar di antara keempatnya. Tiga jangkar memberi 0,1° — K (0083 #2),
0083 #7, dan 0080 #15 — tapi ketiganya tersebar di **dua** scan sehingga
`pasangan.json` belum bisa menyatakannya dalam satu tepi. Perlu trik
pinjam-benda (pindahkan 0080 #15 ke katalog 0083) atau penyelesaian serentak.

### 3. Modul pemetik di pointcloud_studio

Masih belum dikerjakan, sama seperti sesi lalu. `pasangan.json` diisi tangan.
Rencananya tetap seperti spesifikasi §3.5.

## Alat bantu yang dibuat sesi ini

Ada di `out/_pasak/002/`:
- `rupa2_00XX.png` — kartu rupa tiap benda, diperbesar sendiri-sendiri. **Ini
  yang membuat huruf bisa dikenali mata.** Jangan pakai sumbu tetap untuk semua
  benda; huruf kecil jadi tak terbaca.
- `peta_akhir.png` — tiap scan sendiri-sendiri di kerangka gabungan, yang lain
  abu-abu. Cara termudah membedakan "salah tempat" dari "tidak terliput".
- `peta_searah.png`, `hipotesis_0082.png` — peta tampak atas per scan dan
  perbandingan dua hipotesis.

Skripnya ada di scratchpad sesi, tidak disalin ke repo.

## Menjalankan tesnya

```bash
cd ~/"riset td/ros2_ws/cloudcom"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_pasak.py -v
```

## Kalibrasi terukur — jangan diulang

- `--redam 0` untuk dinding huruf; bawaan 0,1 memberi ICP 0,87 m sementara jarak
  antar huruf cuma 0,78 m — cukup untuk melompat satu huruf
- `--max-tapak 1.8` membuka M dan satu huruf lagi di 0082; yang ikut lolos cuma
  kepingan datar setinggi 0,14–0,86 m, tidak berbahaya
- `--range 6` (baku) memberi tajam 0,25–0,46; `--range 10` memburuk ke 0,08–0,12
- `--sharp 0.02` di `clomerged` membuat semuanya GAGAL; jangan di bawah 0,03
- menaikkan `--seeds`/`--step-deg` pada `outmerge` untuk 0064–0071 sudah dicoba
  dan mengulang jawaban yang sama
