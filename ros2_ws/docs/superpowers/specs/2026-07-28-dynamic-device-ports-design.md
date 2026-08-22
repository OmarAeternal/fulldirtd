# Penamaan Port Serial Stabil untuk IMU & LiDAR

**Tanggal:** 2026-07-28
**Status:** Disetujui, siap masuk tahap plan
**Platform:** Raspberry Pi 5 (`raspitampan-desktop`), Ubuntu + ROS 2 Jazzy

---

## 1. Masalah

IMU dan LiDAR sama-sama tersambung lewat chip USB-serial Silicon Labs CP210x
dengan VID:PID **identik** (`10c4:ea60`). Dua udev rule yang terpasang di Pi
hanya mencocokkan VID:PID:

```
# /etc/udev/rules.d/imu_usb.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="imu_usb"

# /etc/udev/rules.d/rplidar.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
```

Karena kedua pola cocok dengan kedua device, masing-masing device mengklaim
kedua nama. Nama symlink tidak unik, jadi device yang diproses terakhir yang
menang.

### Bug yang sudah aktif

Kondisi nyata di Pi saat ini:

```
/dev/imu_usb  -> ttyUSB1
/dev/rplidar  -> ttyUSB1
```

Keduanya menunjuk **device yang sama, yaitu LiDAR**. IMU di `ttyUSB0` tidak
punya symlink sama sekali.

`wit_ros2_imu.py:153` membuka `/dev/imu_usb` — artinya node IMU selama ini
membuka port LiDAR, bersamaan dengan `sllidar_node` yang membuka
`/dev/ttyUSB1`. Dua proses berebut satu port serial.

Konsekuensi: `/imu/data_raw` tidak pernah berisi data IMU yang sah, dan
baudrate IMU yang sebenarnya belum pernah terverifikasi.

### Bukti hardware

Diambil dari Pi dengan kedua device terpasang:

| Device | Node | Chip | `ATTRS{serial}` | Port fisik |
|---|---|---|---|---|
| LiDAR RPLIDAR C1 | `ttyUSB1` | CP2102**N** | `6aa92f87fbe5ed11b5f6d3a80b2af5ab` | bus 2 port 2 |
| IMU WIT | `ttyUSB0` | CP2102 | `0001` | bus 4 port 1 |

Serial kedua device **berbeda**, jadi pembedaan berbasis serial layak dipakai.

Fakta pendukung lain:

- User `raspitampan` sudah anggota grup `dialout`
- Tidak ada systemd service yang meng-autostart ROS
- Sumber kebenaran kode ada di laptop; deploy lewat SSH (edit langsung) + rsync

---

## 2. Tujuan & Non-Tujuan

### Tujuan

1. IMU dan LiDAR selalu dapat nama device yang tetap dan benar, tidak peduli
   urutan enumerasi kernel atau port USB fisik mana yang dipakai
2. Semua port dan baudrate bisa di-override dari command line tanpa edit kode
   dan tanpa rebuild
3. Seluruh perubahan bisa dikembalikan ke keadaan semula
4. Menghapus pemasang rule rusak agar masalah ini tidak kembali

### Non-Tujuan

- **Auto-deteksi runtime** (probing device untuk mengenali jenisnya). Ditolak
  karena masalah yang dipecahkan adalah penamaan, bukan identifikasi. Biaya
  kompleksitasnya tidak sepadan; argumen override sudah jadi jaring pengaman.
- **Mendukung penggantian unit sensor otomatis.** Kalau unit diganti, serial
  berubah dan rule harus di-update manual. `check_sensors.sh` mencetak serial
  yang terdeteksi agar update itu jadi pekerjaan satu menit.
- **Menentukan baudrate IMU yang benar.** Itu hasil verifikasi lapangan setelah
  perbaikan ini, bukan bagian dari perubahan ini.
- **Merapikan paket `sllidar_ros2`.** Kode vendor, dibiarkan utuh.

---

## 3. Desain

### 3.1 udev rule — sumber kebenaran tunggal

File repo: `src/stepper_controller/udev/99-td-sensors.rules`
Terpasang di: `/etc/udev/rules.d/99-td-sensors.rules`

```
# Penamaan port serial stabil untuk sensor TD.
# Menggantikan imu_usb.rules dan rplidar.rules yang hanya mencocokkan VID:PID
# sehingga bentrok (IMU dan LiDAR sama-sama CP210x 10c4:ea60).

# LiDAR — SLAMTEC RPLIDAR C1 (CP2102N)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="6aa92f87fbe5ed11b5f6d3a80b2af5ab", \
  SYMLINK+="td_lidar", MODE="0660", GROUP="dialout"

# IMU — WIT (CP2102, serial default pabrik "0001")
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  ATTRS{serial}=="0001", \
  ATTRS{product}=="CP2102 USB to UART Bridge Controller", \
  SYMLINK+="td_imu", MODE="0660", GROUP="dialout"
```

Keputusan desain:

- **Serial sebagai pembeda utama.** Deterministik, tidak tergantung port fisik.
- **`ATTRS{product}` sebagai pengaman tambahan untuk IMU.** Serial `0001` adalah
  default pabrik dan tidak unik secara global. LiDAR memakai CP2102**N**
  sedangkan IMU CP2102, jadi product string keduanya memang berbeda. Kalau
  ternyata `/dev/td_imu` tidak terbentuk, baris inilah yang pertama dicurigai —
  hapus baris itu dan rule tetap benar untuk dua device yang ada sekarang.
- **`MODE="0660" GROUP="dialout"` menggantikan `MODE:="0777"`.** User sudah
  anggota `dialout`, jadi akses tetap jalan tanpa memberi izin tulis ke semua
  user di sistem.
- **Prefix `td_`** dipilih agar tidak bertabrakan dengan nama lama (`imu_usb`,
  `rplidar`) selama masa transisi.

### 3.2 Script

Semua di `src/stepper_controller/scripts/`.

**`install_udev_rules.sh`** — dijalankan dengan `sudo`:

1. Buat `/etc/udev/rules.d/td-backup-<YYYYmmdd-HHMMSS>/`
2. Salin `imu_usb.rules` dan `rplidar.rules` ke sana apa adanya (kalau ada)
3. Hapus kedua file lama dari `/etc/udev/rules.d/`
4. Pasang `99-td-sensors.rules`
5. `udevadm control --reload-rules && udevadm trigger --subsystem-match=tty`
6. **Verifikasi:** cek `/dev/td_imu` dan `/dev/td_lidar` ada dan menunjuk ke
   target yang berbeda. Kalau tidak, cetak diagnosis dan `exit 1`.

Idempoten: aman dijalankan berulang. Backup lama tidak pernah ditimpa karena
namanya ber-timestamp.

**`rollback_udev_rules.sh`** — dijalankan dengan `sudo`:

1. Cari backup terbaru (atau terima timestamp sebagai argumen)
2. Hapus `99-td-sensors.rules`
3. Kembalikan `imu_usb.rules` dan `rplidar.rules`
4. Reload + trigger
5. Laporkan symlink hasil akhir

Hasilnya persis keadaan sebelum perubahan — termasuk bug `/dev/imu_usb ->
ttyUSB1`, karena tujuan rollback adalah mengembalikan keadaan, bukan memperbaiki.

**`check_sensors.sh`** — tanpa `sudo`, alat diagnosis lapangan. Mencetak:

- Apakah `/dev/td_imu` dan `/dev/td_lidar` ada, menunjuk ke mana
- Permission dan group tiap device
- Semua device serial yang terdeteksi berikut `ATTRS{serial}`-nya, agar kalau
  ada unit yang diganti, serial barunya bisa langsung disalin ke rule
- Apakah user ada di grup `dialout`
- Peringatan kalau rule lama terdeteksi terpasang lagi

### 3.3 Perubahan kode — `wit_ros2_imu.py`

Kondisi sekarang: konstruktor menerima `port_name` tapi mengabaikannya;
`serial.Serial()` memakai `/dev/imu_usb` dan `460800` hardcoded; deklarasi
parameter dikomentari; `main()` mengoper `/dev/ttyACM0` yang tidak pernah
terpakai. Blok `except` hanya `print(e)` lalu jalan terus, sehingga `wt_imu`
tidak terdefinisi dan node crash dengan pesan yang menyesatkan.

Perubahan:

- Deklarasikan parameter `port` (default `/dev/td_imu`) dan `baud`
  (default `460800`)
- Baca keduanya di konstruktor, oper ke `driver_loop`
- `serial.Serial()` memakai nilai parameter
- `main()` memanggil `IMUDriverNode()` tanpa argumen
- Kalau port gagal dibuka: log error lewat `get_logger().error()` dengan nama
  port yang dicoba, lalu hentikan node — jangan lanjut dengan state rusak

Penanganan error ini masuk lingkup karena "port salah" adalah persis skenario
yang mau ditangani; gagal diam-diam justru menghilangkan manfaat perubahan.

### 3.4 Plumbing launch

**`wit_ros2_imu/launch/rviz_and_imu.launch.py`** — tambah
`DeclareLaunchArgument` untuk `imu_port` (default `/dev/td_imu`) dan `imu_baud`
(default `460800`), lalu teruskan ke parameter node.

`imu_baud` harus dibungkus `ParameterValue(..., value_type=int)`, karena
`LaunchConfiguration` menghasilkan string sedangkan parameter `baud`
dideklarasikan sebagai integer.

**`stepper_controller/launch/mapping_system.launch.py`** — deklarasikan
`imu_port`, `imu_baud`, `lidar_port` (default `/dev/td_lidar`), `lidar_baud`
(default `460800`), lalu oper ke sub-launch lewat `launch_arguments`. Untuk
LiDAR, dipetakan ke nama argumen milik vendor: `serial_port` dan
`serial_baudrate`.

**`sllidar_ros2/launch/sllidar_c1_launch.py`** — **tidak disentuh.** Sudah punya
`DeclareLaunchArgument('serial_port')` dan `DeclareLaunchArgument('serial_baudrate')`,
jadi cukup dioper dari launch induk. Paket vendor dibiarkan bersih agar mudah
di-update.

**`stepper_controller/setup.py`** — pasang folder `udev/` dan `scripts/` ke
`share/stepper_controller/` supaya script bisa ditemukan lewat
`ros2 pkg prefix` setelah build.

### 3.5 Menghapus pemasang rule rusak

- `src/wit_ros2_imu/bind_usb.sh` — **dihapus**. Script ini menyalin
  `imu_usb.rules` ke `/etc/udev/rules.d/`; membiarkannya berarti meninggalkan
  cara mudah untuk mengembalikan bug.
- `src/wit_ros2_imu/imu_usb.rules` — **dihapus** bersama pemasangnya.
- `src/sllidar_ros2/scripts/create_udev_rules.sh` dan `rplidar.rules` —
  **dibiarkan** (kode vendor), tapi dicatat di `PANDUAN_SISTEM.md` sebagai
  "jangan dijalankan".

---

## 4. Dampak File Lengkap

### Di Raspberry Pi (sistem)

| File | Aksi |
|---|---|
| `/etc/udev/rules.d/imu_usb.rules` | dihapus (di-backup dulu) |
| `/etc/udev/rules.d/rplidar.rules` | dihapus (di-backup dulu) |
| `/etc/udev/rules.d/99-td-sensors.rules` | dibuat baru |
| `/etc/udev/rules.d/td-backup-<timestamp>/` | dibuat baru (isi backup) |
| `/etc/udev/rules.d/99-gpio.rules` | **tidak disentuh** — dipakai stepper |

Efek samping: nama `/dev/imu_usb` dan `/dev/rplidar` hilang.

### Di repo

| File | Aksi |
|---|---|
| `src/wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py` | diubah |
| `src/wit_ros2_imu/launch/rviz_and_imu.launch.py` | diubah |
| `src/wit_ros2_imu/bind_usb.sh` | dihapus |
| `src/wit_ros2_imu/imu_usb.rules` | dihapus |
| `src/stepper_controller/launch/mapping_system.launch.py` | diubah |
| `src/stepper_controller/setup.py` | diubah |
| `src/stepper_controller/udev/99-td-sensors.rules` | baru |
| `src/stepper_controller/scripts/install_udev_rules.sh` | baru |
| `src/stepper_controller/scripts/rollback_udev_rules.sh` | baru |
| `src/stepper_controller/scripts/check_sensors.sh` | baru |
| `PANDUAN_SISTEM.md` | diubah — §3 tabel port, §12 catatan udev |
| `src/sllidar_ros2/**` | **tidak disentuh** |

---

## 5. Rollback

Tiga lapis yang berdiri sendiri.

### Lapis 1 — udev

`sudo rollback_udev_rules.sh` mengembalikan kedua rule lama dari backup
ber-timestamp dan menghapus rule baru. Keadaan `/etc/udev/rules.d/` kembali
persis seperti semula.

### Lapis 2 — kode

Karena Pi kadang diedit langsung lewat SSH, git branch di laptop saja tidak
cukup — Pi bisa punya perubahan lokal yang belum pernah naik ke laptop.

Dua langkah:

1. **Sebelum rsync**, bandingkan checksum file yang akan ditimpa antara Pi dan
   laptop. Kalau ada yang berbeda, berhenti dan selesaikan dulu — jangan biarkan
   rsync menimpa edit lokal diam-diam.
2. **Sebelum rsync**, buat tarball di Pi berisi semua file yang akan ditimpa
   atau dihapus (daftarnya di §4), simpan di `~/td-backup-<timestamp>.tar.gz`.
   Ekstrak tarball itu mengembalikan seluruh source ke keadaan semula.

Di sisi laptop, semua pekerjaan tetap di branch `feat/dynamic-ports` agar
`git checkout main` mengembalikan sumber kebenaran.

### Lapis 3 — override tanpa rollback apa pun

Jaring pengaman di lapangan. Kalau udev bermasalah di lokasi, jalankan dengan
port eksplisit tanpa menyentuh rule dan tanpa rebuild:

```bash
ros2 launch stepper_controller mapping_system.launch.py \
  imu_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

---

## 6. Skenario Gagal

| Skenario | Gejala | Penanganan |
|---|---|---|
| Serial salah ketik di rule | symlink tidak terbentuk | `install_udev_rules.sh` verifikasi sendiri dan `exit 1` — gagalnya berisik |
| `ATTRS{product}` tidak cocok | `/dev/td_imu` tidak ada, `td_lidar` ada | hapus baris `ATTRS{product}`, pasang ulang |
| Device tidak tercolok | symlink hilang | node gagal buka port dengan pesan berisi nama port — bukan diam-diam nyambung ke device salah |
| Baudrate IMU salah | IMU tidak publish atau datanya sampah | `imu_baud:=9600` vs `imu_baud:=460800` dari command line |
| Unit sensor diganti | serial baru, symlink hilang | `check_sensors.sh` mencetak serial device terdeteksi → salin ke rule |
| Rule lama dihapus sebelum kode baru masuk | IMU mati total | urutan deploy wajib: kode dulu, rule belakangan |
| `bind_usb.sh` dijalankan lagi | bug bentrok kembali | file dihapus dari repo |
| rsync menimpa edit lokal di Pi | perubahan hilang | verifikasi checksum sebelum rsync |

---

## 7. Urutan Deploy

Urutannya mengikat. `/dev/imu_usb` masih dipakai kode lama, jadi rule tidak
boleh diganti sebelum kode baru terpasang.

1. Verifikasi checksum file terdampak: Pi vs laptop
2. Buat tarball backup di Pi
3. rsync source dari laptop ke Pi
4. `colcon build --packages-select wit_ros2_imu stepper_controller`
5. Tes dengan port lama eksplisit:
   `ros2 launch ... imu_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1`
   Ini pertama kalinya node IMU benar-benar bicara dengan IMU — di sinilah
   baudrate yang benar ditentukan (coba `460800`, lalu `9600`)
6. `sudo install_udev_rules.sh`
7. `check_sensors.sh` — pastikan kedua symlink benar dan berbeda target
8. Jalankan dengan default (tanpa argumen override)
9. Tes tukar kabel fisik

---

## 8. Kriteria Penerimaan

1. `/dev/td_lidar` menunjuk ke device dengan serial `6aa92f87fbe5ed11b5f6d3a80b2af5ab`
2. `/dev/td_imu` menunjuk ke device dengan serial `0001`
3. Keduanya menunjuk ke node `ttyUSB` yang **berbeda**
4. `/dev/imu_usb` dan `/dev/rplidar` sudah tidak ada
5. `ros2 topic hz /scan` menunjukkan laju sesuai konfigurasi `scan_frequency`
   di `sllidar_c1_launch.py`, yaitu sekitar 12 Hz
6. `ros2 topic hz /imu/data_raw` menunjukkan laju wajar, dan `ros2 topic echo`
   menunjukkan data masuk akal — `linear_acceleration.z` sekitar 9.8 saat alat
   diam dan datar. Ini pertama kalinya data IMU sah muncul.
7. **Tes utama:** tukar posisi fisik kabel USB IMU dan LiDAR, jalankan ulang
   tanpa mengubah apa pun — kriteria 1–6 tetap terpenuhi
8. `sudo rollback_udev_rules.sh` mengembalikan `/etc/udev/rules.d/` ke keadaan
   semula, dibuktikan dengan membandingkan daftar file dan isinya

---

## 9. Risiko yang Diterima

- **Serial IMU `0001` tidak unik secara global.** Cukup untuk membedakan dari
  LiDAR yang serialnya panjang dan acak, dan `ATTRS{product}` mempersempitnya
  lebih jauh. Kalau nanti ada CP2102 lain dengan serial `0001` dicolok, rule
  perlu diperketat lagi — misalnya dengan mencocokkan port USB fisik.
- **Penggantian unit sensor butuh update rule manual.** Konsekuensi sadar dari
  menolak auto-deteksi runtime; `check_sensors.sh` membuatnya cepat.
- **`sllidar_ros2/scripts/create_udev_rules.sh` masih ada** dan bisa memasang
  kembali rule rusak. Dibiarkan karena kode vendor; mitigasinya berupa catatan
  di `PANDUAN_SISTEM.md` dan peringatan di `check_sensors.sh`.
- **Baudrate IMU masih belum diketahui** saat spec ini ditulis. Default
  dipertahankan di `460800` agar perubahan ini hanya mengubah satu variabel
  (port), bukan dua sekaligus.
