# Sesi `scanfix3agus` — 3 Agustus 2026

Catatan serah-terima. Baca ini dulu besok sebelum lanjut.

**Melanjutkan sesi ini dengan konteksnya utuh:**

```bash
cd ~/"riset td" && claude --resume 9e82d4e8-d7fc-40dd-baf7-a40be6ad0260
```

---

## SUDAH SELESAI

### 1. Port serial LiDAR & IMU — beres, tinggal diuji reboot

Akar masalah: dua aturan udev yang identik, sama-sama mencocokkan VID:PID
`10c4:ea60`. Kedua adapter (CP2102 dan CP2102N) memakai VID:PID yang sama, jadi
`/dev/rplidar` dan `/dev/imu_usb` mendarat acak tiap boot. Node LiDAR sempat
membuka port IMU — satu penyebab, dua gejala.

Yang diubah di Raspi:

- `/etc/udev/rules.d/99-lidar-imu.rules` — dua baris, dicocokkan per **serial**
  (LiDAR `6aa92f87fbe5ed11b5f6d3a80b2af5ab`, IMU `0001`)
- `sllidar_c1_launch.py:12` — `/dev/ttyUSB1` → `/dev/rplidar`

Cadangan aturan lama: `~/riset td/backup/udev-raspi-2026-08-03/`
Panduan lengkap: `~/riset td/ros2_ws/PANDUAN_PORT_SERIAL.md`

**Terbukti:** IMU kini terbit stabil 100,3 Hz tanpa error, quaternion
ternormalisasi sempurna (roll −0,77°, pitch −0,14° saat rig datar — angka itu
adalah bias pemasangan, dipakai sebagai `imu_roll_offset_deg` /
`imu_pitch_offset_deg`).

**BELUM diuji:** reboot. Symlink udev ditetapkan saat boot; sejauh ini baru
ditiru lewat `udevadm trigger`.

```bash
sudo reboot
# tunggu ~40 detik, SSH lagi:
ls -l /dev/rplidar /dev/imu_usb    # harus dua device BERBEDA
```

### 2. Paket `sweep_mappingimu` — terkirim dan terpasang

Mapping 3D dengan koreksi orientasi IMU per sinar. Sudah di-rsync ke Raspi dan
`ros2 pkg list` menampilkan `sweep_mapping` dan `sweep_mappingimu`.

```bash
ros2 launch sweep_mappingimu sweep_mappingimu.launch.py sweeps:=1 delay:=0.009375 imu_roll_offset_deg:=-0.77 imu_pitch_offset_deg:=-0.14
```

`sweep_mapping` yang lama **tidak berubah cara pakainya** — `delay:=0.009375`
tetap berlaku persis seperti dulu. `rpm:=` ditambahkan sebagai alternatif
opsional (dipakai hanya kalau diisi > 0).

### 3. Dua alat analisis offline

Keduanya di `~/ros2_ws/src/sweep_mappingimu/scripts/` di Raspi.

| Alat | Gunanya |
|---|---|
| `bandingkan_imu.py` | olah satu bag lewat dua node (dengan/tanpa koreksi IMU), keluarkan dua PLY + statistik |
| `diagnosa_sumbu.py` | pecah sweep jadi paruh A (0-180°) dan paruh B (180-360°), ukur selisihnya |

Keduanya membaca bag langsung — tidak perlu `ros2 bag play` maupun
`use_sim_time`. Swauji `diagnosa_sumbu.py` terhadap `mapping_3d_sweep`
melaporkan beda **0,0e+00 m**, jadi angkanya bukan artefak alat ukur.

---

## MASALAH TERBUKA — lanjut dari sini besok

### Tembok miring: satu sweep mengukur ruangan 2×, dan kedua salinan tidak cocok

Cakupan ganda itu **normal** — memutar bidang pindai 180° saja sudah menutupi
seluruh bola, jadi 360° berarti tiap arah diukur dua kali. (Konsekuensinya:
scan bisa dipotong separuh waktu tanpa kehilangan cakupan.)

Yang **tidak** normal: kedua salinan tidak bertumpuk.

| Rekaman | Detik/sweep | Selisih antar paruh |
|---|---|---|
| `scan_0005` (lama) | ~62 s | **0,74°** |
| `scan_0004` (lama) | ~39 s | **0,87°** |
| `scan_0026` (baru) | ~31 s | **2,31°** |
| `scan_0025` (baru) | ~32 s | **2,60°** |

Rekaman baru **2× lebih cepat** dan kesalahannya **3× lebih besar**. Korelasinya
searah dan rapi. Pembagian titiknya juga jadi timpang (`scan_0026`: paruh A
67.598 vs paruh B 86.050; bag lama seimbang 497k vs 474k).

**Koreksi penting terhadap kesimpulan awal sesi ini:** selisih yang membesar
seiring jarak memang menandakan kesalahan sudut, bukan pergeseran — tapi itu
mencakup **dua** penyebab, bukan satu:

- arah sumbu putar tidak sejajar → geometris, **tidak peduli kecepatan**
- motor kehilangan step → **memburuk saat dipercepat**

`diagnosa_sumbu.py` bisa memisahkan sudut vs pergeseran, tapi **tidak** bisa
memisahkan sumbu-miring vs motor-slip. Data yang ada menunjuk ke slip.

### Dua uji yang harus dijalankan besok

**Uji 1 — scan sama, motor dipelankan.** Jangan pindahkan rig, jangan ubah
apa pun selain kecepatan:

```bash
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=1 delay:=0.018
```

```bash
python3 ~/ros2_ws/src/sweep_mappingimu/scripts/diagnosa_sumbu.py ~/bags/NAMA_BARU --out-dir ~/hasil
```

- turun ke ~0,8° → **motor slip.** Perbaikannya cukup jangan pakai 2 RPM,
  tidak ada kode yang perlu diubah.
- tetap ~2,4° → **sumbunya memang miring**, dan bag lama yang lebih baik itu
  kebetulan saja. Baru di titik itu kalibrasi sumbu jadi masuk akal.

**Uji 2 — penanda fisik.** Tempel selotip di piringan LiDAR, jalankan
`sweeps:=5 delay:=0.009375`, lihat apakah penandanya kembali persis ke posisi
semula. Meleset = slip, terbukti tanpa olah data sama sekali.

### Kalau ternyata perlu kalibrasi sumbu

Yang sudah diketahui cuma **besarnya** (0,7–2,6° tergantung kecepatan), belum
**arahnya** — meleset ke atas, bawah, kiri, atau kanan. Arah menentukan tanda
koreksinya, jadi harus diukur dulu sebelum ada kode ditulis.

Rancangannya juga belum dibahas: apakah koreksi ditaruh di node realtime, di
alat offline, atau keduanya.

---

## HAL LAIN YANG BELUM SELESAI

**Bag lama tidak punya data IMU.** `/imu/data_raw` isinya 0 pesan di semua
rekaman sebelum 3 Agustus — node IMU selalu crash akibat rebutan port.
Perbandingan dengan/tanpa koreksi IMU hanya bisa dari rekaman baru.

**Koreksi IMU belum pernah diuji di rig sungguhan.** Baru terbukti lewat uji
sintetis: rig dimiringkan 10°, lantai keluar datar di z = −1 dengan simpangan
baku 2×10⁻⁸ m; data yang sama tanpa koreksi meleset 6,82 m. Perlu satu rekaman
sambil rig digoyang tangan, lalu `bandingkan_imu.py`.

**Gagasan yang belum dikerjakan:** alat offline yang membaca bag mentah, memfusi
ulang dengan koreksi IMU, lalu menulis bag baru berisi `/map_3d` terkoreksi —
sehingga rantai `mcaptopc → clomerge → outmerge` jalan tanpa diubah. Tidak bisa
ditaruh sebagai parameter di `clomerge` karena rantai itu bermula dari `/map_3d`
yang sudah terlanjur difusikan; waktu absolut dan sudut stepper per titik sudah
hilang di sana.

---

## ATURAN KERJA YANG BERLAKU

**Berkas port di laptop adalah cadangan — jangan diedit di laptop.**
`sllidar_c1_launch.py`, `wit_ros2_imu.py`, `rviz_and_imu.launch.py` hanya diubah
di Raspi. Arahnya **tarik dari Raspi**, bukan dorong dari laptop.

Berlawanan dengan `sweep_mapping` dan `sweep_mappingimu`, yang justru ditulis di
laptop lalu dikirim lewat rsync:

```bash
cd ~/"riset td/ros2_ws/src/newcodeshere"
rsync -avz --exclude='__pycache__' sweep_mapping sweep_mappingimu raspitampan:~/ros2_ws/src/
```

**Saat menempel perintah ke SSH:** satu perintah = satu baris. Tanpa `\`
sambungan, tanpa heredoc. Perintah panjang terpotong saat ditempel dan
menghasilkan berkas cacat yang tetap terlihat berhasil — itu terjadi dua kali
hari ini. Selalu `cat -n` berkas aturan setelah menulisnya.
