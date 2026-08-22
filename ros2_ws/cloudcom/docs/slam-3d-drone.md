# SLAM 3D dari drone — analisis kelayakan rig LiDAR 2D + stepper

Tanggal: 2026-07-29

Dokumen ini merangkum perhitungan kecepatan stepper untuk SLAM 3D dari drone,
beserta kesimpulan dan rekomendasinya. Semua angka dasar diukur langsung dari
bag rekaman yang ada, bukan dari spesifikasi di atas kertas.

## Angka dasar rig (diukur dari bag)

| Besaran | Nilai | Sumber |
|---|---|---|
| Laju pindai LiDAR | ~10,2 Hz | `/scan`: 1.838 pesan / 179,7 detik (`scan_0005_3sweep_0.mcap`) |
| Durasi satu sapuan | 59 detik | rentang `/stepper/angle` (`scan_0001_1sweep_0.mcap`) |
| Titik per cloud | ~267.000 | `out/016_0/016_0.ply` |
| Kecepatan stepper saat ini | ~0,5 RPM | 30 / 59 detik |
| IMU | **0 pesan** | `/imu/data_raw` ada di bag tapi kosong |
| GPS | tidak ada | tidak ada topik GNSS sama sekali |

`/tf` berisi 3.107–9.426 pesan, tapi jumlahnya mengikuti `/stepper/angle` —
jadi itu geometri sapuan stepper, bukan posisi alat di dunia.

## Batas keras

LiDAR memindai 10 bidang per detik. Angka ini tetap, apa pun kecepatan stepper.

```
garis per sapuan  ×  sapuan per detik  =  10
```

**RPM bukan parameter yang bisa menyelesaikan masalah.** Jumlah data yang masuk
tetap 10 bidang/detik; RPM hanya menentukan cara mengirisnya — sapuan cepat
tapi tipis, atau sapuan lambat tapi tebal.

## Rumus

Dengan asumsi stepper berputar menerus dan cakupan penuh tercapai tiap setengah
putaran (LiDAR sudah memindai 360° pada bidangnya):

```
periode sapuan     = 30 / RPM            detik
garis per sapuan   = 300 / RPM
jarak antar garis  = 0,6 × RPM           derajat
gerak maks drone   = 0,2 / periode       m/detik
```

Baris terakhir memakai patokan: drone tidak boleh berpindah lebih dari 0,2 m
selama satu sapuan, agar cloud tidak melar melebihi ketelitian pencocokan.

## Tabel

| RPM | Periode sapuan | Garis/sapuan | Jarak garis | Gerak maks drone |
|---:|---:|---:|---:|---:|
| 60 | 0,5 dtk | 5 | 36° | 0,40 m/dtk |
| 30 | 1 dtk | 10 | 18° | 0,20 m/dtk |
| 20 | 1,5 dtk | 15 | 12° | 0,13 m/dtk |
| 15 | 2 dtk | 20 | 9° | 0,10 m/dtk |
| 12 | 2,5 dtk | 25 | 7,2° | 0,08 m/dtk |
| 10 | 3 dtk | 30 | 6° | 0,067 m/dtk |
| 8 | 3,75 dtk | 38 | 4,8° | 0,053 m/dtk |
| 6 | 5 dtk | 50 | 3,6° | 0,040 m/dtk |
| 5 | 6 dtk | 60 | 3° | 0,033 m/dtk |
| 4 | 7,5 dtk | 75 | 2,4° | 0,027 m/dtk |
| 3 | 10 dtk | 100 | 1,8° | 0,020 m/dtk |
| 2 | 15 dtk | 150 | 1,2° | 0,013 m/dtk |
| 1 | 30 dtk | 300 | 0,6° | 0,0067 m/dtk |
| **0,5** | **59 dtk** | **590** | **0,3°** | **0,0034 m/dtk** |

Baris terakhir adalah **kondisi rig saat ini**. Dari situlah 267.000 titik dan
kerapatan 0,3° berasal.

Pembanding: Velodyne VLP-16 memberi 16 garis berjarak **2°**, sepuluh kali per
detik.

## Dua zona, tanpa zona tengah yang enak

**Di atas ~20 RPM** — cukup cepat mengikuti drone, tapi hanya 5–15 garis per
sapuan, berjarak 12–36°. Pada jarak 10 meter, dua garis bersebelahan terpisah
2–6 meter. Objek seukuran manusia bisa lolos di antara dua garis tanpa terekam.

**Di bawah ~6 RPM** — kerapatan bagus (setara atau lebih baik dari VLP-16),
tapi sapuan 5 detik ke atas menuntut drone bergerak di bawah 4 cm/detik.
Hanyutan drone yang melayang saja biasanya sudah melebihi itu. Artinya ini
sebenarnya sudah kembali menjadi pemindaian statis.

**Titik temu: 12–20 RPM**, dan di situ pun hasilnya 15–25 garis dengan drone
merayap 8–13 cm/detik.

## Rekomendasi

### Kalau tetap menempuh jalur ini

**Mulai dari 15 RPM** — 20 garis per sapuan, sapuan 2 detik, drone 10 cm/detik.
Titik paling seimbang di tabel, dan cukup untuk membuktikan apakah pendekatannya
layak diteruskan sebelum keluar biaya.

Tiga syarat yang harus menyertainya:

1. **IMU wajib menyala** dan dipakai untuk *deskewing* — tiap garis pindai
   ditempatkan memakai pose hasil interpolasi pada saat garis itu direkam,
   bukan satu pose untuk seluruh sapuan. Tanpa ini, sapuan 2 detik melar total.
2. **Drone terbang sangat pelan**, sesuai kolom terakhir tabel.
3. **Putaran menerus, bukan bolak-balik.** Bolak-balik berarti pengereman dan
   percepatan di tiap ujung, dan hentakan itu masuk ke IMU sebagai derau.
   Putaran menerus butuh slip ring.

Periksa juga torsi stepper di RPM target: bila kehilangan langkah, sudut yang
dilaporkan salah dan seluruh cloud melenceng.

### Dua cara nyata menaikkan kemampuan

1. **Naikkan laju pindai LiDAR.** Bila modelnya mendukung 20 Hz (RPLIDAR
   A2/A3/S1 sebagian bisa), **seluruh tabel menggandakan diri** — 20 RPM memberi
   30 garis, bukan 15. Ini paling murah, hanya perubahan konfigurasi.
   **Belum diperiksa: model LiDAR yang dipakai.**
2. **LiDAR 3D sungguhan** (mis. Livox Mid-360), bila target akhirnya peta rapat
   dari drone yang bergerak. Ini satu-satunya jalan menuju SLAM 3D yang benar
   dari wahana terbang; bukan pekerjaan perangkat lunak.

### Kesimpulan utama

Rig LiDAR 2D + stepper **tidak punya titik operasi yang baik untuk SLAM 3D dari
drone yang melayang.** Bukan karena RPM-nya belum ketemu, melainkan karena laju
datanya 10 bidang/detik dan itu tidak bisa dinaikkan dengan memutar stepper
lebih cepat atau lebih lambat.

## Alternatif yang lebih menjanjikan

**A. SLAM 2D** — kunci stepper mendatar, jalankan `slam_toolbox` (sudah tersedia
di ROS 2 Jazzy). Hasilnya denah 2D + lintasan, bukan peta 3D. Bekerja dengan
perangkat yang ada sekarang, sanggup dijalankan Raspberry Pi. Cocok bila yang
dibutuhkan denah dan jalur.

**B. Drone sebagai pengangkut** — drone mendarat atau bertengger di tiap posisi,
menyapu 59 detik seperti sekarang, lalu pindah. Kualitas 3D tetap setinggi
sekarang. Alur `clomerge` yang sudah ada tinggal ditambah pose graph.
**Ini yang memberi hasil terbaik dengan usaha paling sedikit.**

**C. Ganti LiDAR** — lihat poin 2 di atas.

## Catatan tentang pose graph (untuk opsi B)

`clomerge` sekarang memakai topologi bintang: semua scan dicocokkan ke scan
pertama. Di luar ruangan ini patah, karena scan ke-5 bisa tidak melihat apa pun
dari scan ke-1.

Gantinya: cocokkan **setiap pasang yang beririsan**, bentuk graf, lalu optimasi
global sekaligus (`open3d.pipelines.registration.global_optimization`, dengan
kernel robust agar pasangan yang salah ditolak otomatis).

Syaratnya berubah dari "tiap scan harus beririsan dengan **scan pertama**"
menjadi "cukup beririsan dengan **salah satu scan lain**" — jauh lebih longgar
di lapangan. Bila lintasan menutup kembali ke titik awal, *loop closure*-nya
dipakai untuk membagi rata error alih-alih menumpuknya di scan terakhir.

Prosedur lapangan: irisan ≥40% antar posisi berdekatan, dan usahakan lintasan
menutup kembali ke titik awal.

## Yang masih terbuka

- **Model LiDAR belum diketahui** — menentukan apakah laju 20 Hz mungkin, dan
  ini pengungkit termurah yang tersedia.
- **`/imu/data_raw` masih 0 pesan** — semua jalur di atas membutuhkannya
  (deskewing, perataan gravitasi, FAST-LIO2, penguncian roll–pitch).
- **Belum ada data outdoor sungguhan** untuk menguji. Penyetelan tahap
  berikutnya sangat bergantung pada seperti apa medannya.
