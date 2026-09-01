# Sesi `registrasi scan0064-0071` — 10 Agustus 2026

Catatan serah-terima. Baca ini dulu sebelum lanjut.

**Melanjutkan sesi ini dengan konteksnya utuh:**

```bash
cd ~/"riset td/cloudcom" && claude --resume 5d015d97-8c2d-4a39-a145-aa9f7fe70c06
```

---

## Konteks

8 scan di ruangan yang sama, 2 posisi berbeda:
- **Set sudut**: `scan_0064`–`0067` (rig ditaruh di tiap sudut ruangan)
- **Set sisi**: `scan_0068`–`0071` (rig ditaruh di tiap sisi ruangan)

Pertanyaan awal: kenapa hasil registrasi/merge dari 4 scan selalu jelek, dan
apakah ada cara memperbaikinya (fitur `clomerge`/`outmerge` yang sudah ada di
`ros2_ws/cloudcom/` — FPFH + RANSAC + ICP, lihat
`ros2_ws/cloudcom/docs/cara-kerja-clomerge.md`).

## SUDAH DIKERJAKAN

### 1. Diagnosis set sudut (0064–0067) — `out/_merge/027` (clomerge, sudah ada sebelum sesi ini)

Vonis: 0064 acuan BAIK, 0065 RAGU, 0066 GAGAL, 0067 GAGAL.

Ditarik kolom translasi tiap matriks (posisi sensor tiap scan di kerangka
acuan):

| Scan | posisi (x,y) | jarak ke 0064 |
|---|---|---|
| 0064 (acuan) | (0, 0) | — |
| 0065 | (4,83, 5,65) | 7,43 m |
| 0066 | (1,63, −1,77) | 2,41 m |
| 0067 | (0,06, −0,78) | 0,78 m |

0064/0066/0067 menumpuk dalam kotak ~2,4 m padahal seharusnya 3 sudut ruangan
berbeda (berjarak beberapa meter). Kesimpulan: **bukan noise, tapi salah kunci
global** — RANSAC/FPFH mencocokkan *bentuk* sudut-dinding (mirip di ruangan
segi-empat) tanpa bisa membedakan sudut yang mana. Ini limitasi "objek
simetris" yang sudah didokumentasikan di `cara-kerja-clomerge.md`.

### 2. Set sisi (0068–0071) via `outmerge` — run baru di sesi ini

**Run `out/_outmerge/020`** (default `--step-deg 2.0 --seeds 14 --range 10`):
semua RAGU, fitness 0068=0,43 · 0069=0,55 · 0070=0,40 · 0071=0,27. Lebih baik
dari set sudut (overlap dinding lebih besar di posisi sisi).

**Verifikasi visual pengguna** di `merged_check.ply`: 0069 (hijau) & 0070
(biru) tampak benar; **0071 (kuning) bertumpuk dengan 0068 (merah)**, padahal
harusnya di sisi kiri, terpisah.

Dikonfirmasi angka: translasi 0068=(9,83, 0,72), 0071=(9,42, 1,12) → jarak
**0,55 m** — memang bertumpuk.

**Run `out/_outmerge/021`** (`--step-deg 1.0 --seeds 24`, sapuan lebih rapat):
hasil untuk 0071 **nyaris identik** — jarak ke 0068 tetap 0,57 m. Jadi bukan
soal sapuan kurang rapat; algoritma konsisten menemukan posisi ini sebagai
"terbaik" yang tersedia buat pasangan 0068–0071.

**Run `out/_outmerge/022`** (uji pasangan 0070+0071 saja, tanpa peta
gabungan): jawaban **beda lagi** — yaw 91,7°, geser (7,11, −3,68), fitness
cuma 0,16 (lebih rendah dari 0,27 di run map-based). Tiga konteks pencocokan
berbeda → tiga jawaban berbeda, tak satu pun percaya diri tinggi.

**Kesimpulan:** posisi 0071 relatif terhadap yang lain adalah **ambiguitas
struktural**, bukan artefak parameter pencarian. Ruangan di sekitar posisi itu
tidak punya ciri pembeda yang cukup untuk FPFH/RANSAC mengunci dengan yakin.
Menambah `--seeds`/`--step-deg` lagi kemungkinan besar mengulang jawaban yang
sama.

## BELUM DIKERJAKAN — lanjut dari sini

1. **Perbaiki 0071 manual** — dibahas tapi belum dieksekusi. Rencana: buka
   `out/scan_0071_1sweep_0/scan_0071_1sweep_0.ply` bareng `out/_outmerge/021/merged.ply`
   di CloudCompare (`clomcaps`), align manual titik-ke-titik pakai penilaian
   visual manusia (yang tidak tertangkap FPFH).
2. **Set sudut (0064–0067) belum dibetulkan sama sekali** — masih GAGAL/RAGU,
   lebih parah dari set sisi. Kemungkinan butuh perlakuan sama (manual align)
   atau scan ulang dengan penanda.
3. **Perbaikan jangka panjang**: taruh benda/penanda asimetris di ruangan
   sebelum scan ulang, supaya FPFH punya ciri unik yang memutus simetri
   sudut/sisi yang mirip. Atau tambah scan di posisi antara (overlap
   langsung) sebagai jangkar.
4. **Terhubung ke masalah terbuka sesi 3 Agustus** (`SESI_scanfix3agus.md`):
   sumbu-miring/motor-slip belum diuji tuntas (Uji 1 & Uji 2 belum
   dijalankan). Kalau itu masih ada, tiap scan individual sudah sedikit
   terdistorsi secara internal sebelum dicocokkan — belum dikonfirmasi
   pengaruhnya ke ambiguitas registrasi di atas, tapi patut dicurigai.
   Catatan tambahan: koreksi perataan lantai per scan di `outmerge` run
   020/021 lumayan besar (2,8°–7,3°) — beda-beda tiap posisi, konsisten
   dengan rig yang tidak level sama antar penempatan.

## File hasil yang relevan

```
cloudcom/out/_merge/027/          clomerge, set sudut 0064-0067 (lama)
cloudcom/out/_outmerge/011/       outmerge, set sudut 0064-0067 (lama)
cloudcom/out/_outmerge/020/       outmerge, set sisi 0068-0071, default
cloudcom/out/_outmerge/021/       outmerge, set sisi 0068-0071, sapuan rapat
cloudcom/out/_outmerge/022/       outmerge, pasangan 0070+0071 saja (uji)
```

Tiap folder punya `merged.ply`, `merged_check.ply` (warna per-scan, buka ini
dulu buat cek visual), `grid.ply`, `transforms.txt`.
