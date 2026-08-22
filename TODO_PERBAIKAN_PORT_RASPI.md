# TODO — Perbaikan port serial, DIKERJAKAN DI RASPI

**Status:** diagnosis SELESAI dan terkonfirmasi. Perbaikan menunggu dikerjakan.
**Dibuat:** 3 Agustus 2026 · **Direvisi:** 3 Agustus 2026 setelah uji langsung
**Dikerjakan di:** Raspberry Pi (`raspitampan`), **BUKAN** di laptop

> Kode di laptop ini sengaja **dibiarkan apa adanya sebagai cadangan**.
> Jangan ubah `sllidar_c1_launch.py`, `wit_ros2_imu.py`, atau
> `rviz_and_imu.launch.py` di sini. Semua perubahan port dilakukan langsung
> di Raspi. Kalau nanti laptop dan Raspi perlu disamakan, tarik dari Raspi.

---

## Akar masalahnya

Di Raspi ada **dua aturan udev yang identik**:

```
imu_usb.rules   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="imu_usb"
rplidar.rules   KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
```

Keduanya hanya mencocokkan VID:PID `10c4:ea60` — dan **CP2102 maupun CP2102N
sama-sama `10c4:ea60`**. Jadi tiap aturan kena ke kedua adapter: setiap device
mendapat symlink `imu_usb` DAN `rplidar`, lalu yang menang adalah device yang
kebetulan diproses terakhir. Symlink-nya mendarat acak tiap boot.

Itulah sebab sesungguhnya dari "gagal kalau tanpa monitor/keyboard/mouse".
`/dev/ttyUSB1` yang dipaku di `sllidar_c1_launch.py` hanyalah tambalan atas
gejala — ditulis saat LiDAR kebetulan mendarat di sana.

Gejala yang muncul saat symlink jatuh ke device yang salah:

```
[sllidar_node]  Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!
[wit_ros2_imu]  SerialException: device reports readiness to read but returned
                no data (device disconnected or multiple access on port?)
```

Satu penyebab, dua gejala: node LiDAR membuka port IMU, dan IMU kehilangan
portnya karena direbut.

## Peta perangkat — TERKONFIRMASI lewat uji langsung 3 Agustus 2026

| Adapter | Serial | Perangkat | Bukti |
|---|---|---|---|
| CP2102**N** | `6aa92f87fbe5ed11b5f6d3a80b2af5ab` | **LiDAR** RPLIDAR C1 | `S/N B9D7E1F4C2E398C0BCEA9AF3370C4806`, FW 1.01, health OK |
| CP2102 | `0001` | **IMU** WIT | `SL_RESULT_OPERATION_TIMEOUT` saat dicoba sebagai LiDAR |

Cara mengujinya lagi kalau perlu — jalankan sllidar langsung ke tiap path
`by-id`, yang mana pun yang menyebut `SLLidar health status : OK` itulah LiDAR:

```bash
pkill -f sllidar_node; pkill -f wit_ros2_imu
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_6aa92f87fbe5ed11b5f6d3a80b2af5ab-if00-port0
```

---

## Langkah 1 — ganti ketiga aturan dengan satu — SELESAI 3 Agustus 2026

Hasil akhir `/etc/udev/rules.d/99-lidar-imu.rules` (tepat dua baris):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="6aa92f87fbe5ed11b5f6d3a80b2af5ab", MODE="0666", SYMLINK+="rplidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666", SYMLINK+="imu_usb"
```

Terverifikasi: `/dev/rplidar -> ttyUSB0`, `/dev/imu_usb -> ttyUSB1`.

Ketiga aturan lama **dihapus dengan `rm`**, jadi tidak ada cadangan di Raspi.
Aslinya tersimpan di laptop: `~/riset td/backup/udev-raspi-2026-08-03/`.

Pelajaran saat mengerjakan: perintah yang panjang terpotong saat ditempel lewat
SSH, dan potongannya menghasilkan file yang cacat tapi *terlihat* berhasil.
Selalu `cat -n` berkas aturan setelah menulisnya — jumlah barisnya harus persis
seperti yang diharapkan, dan tiap baris aturan harus dimulai `SUBSYSTEM==`.
Untuk Raspi, satu perintah = satu baris, tanpa `\` sambungan dan tanpa heredoc.

### Perintah aslinya (arsip)

Aturan lama **dipindahkan, bukan dihapus** — kalau perbaikan ini ternyata
bermasalah, mengembalikannya tinggal satu perintah. Salinan cadangannya juga ada
di laptop: `~/riset td/backup/udev-raspi-2026-08-03/`.

```bash
mkdir -p ~/udev_backup_2026-08-03 && sudo mv /etc/udev/rules.d/imu_usb.rules /etc/udev/rules.d/rplidar.rules /etc/udev/rules.d/99-rplidar.rules ~/udev_backup_2026-08-03/ && sudo chown -R raspitampan: ~/udev_backup_2026-08-03 && ls -l ~/udev_backup_2026-08-03

echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="6aa92f87fbe5ed11b5f6d3a80b2af5ab", MODE="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-lidar-imu.rules

echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE="0666", SYMLINK+="imu_usb"' | sudo tee -a /etc/udev/rules.d/99-lidar-imu.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
sleep 2
ls -l /dev/rplidar /dev/imu_usb
```

Perhatikan `tee -a` pada perintah kedua — tanpa `-a`, baris pertama tertimpa.

Hasil yang benar: **dua nama menunjuk dua device berbeda.**

```
/dev/imu_usb -> ttyUSB1
/dev/rplidar -> ttyUSB0
```

Kalau masih menunjuk device yang sama, cabut-colok kedua USB atau reboot.

`MODE` diturunkan dari `0777` ke `0666`: bit execute pada char device tidak
bermakna, hak baca-tulis untuk semua tetap sama.

Catatan bentuk perintah: **jangan pakai heredoc** (`<<'EOF'`) kalau menyalin
dari blok kode yang ter-indentasi — penutup `EOF` yang punya spasi di depan
tidak mengakhiri heredoc, dan shell akan menelan perintah-perintah berikutnya
ke dalam isi file.

## Langkah 2 — satu baris di sllidar (baru boleh setelah Langkah 1 benar)

Berkas: `~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py`, baris 12.

```bash
grep -n "ttyUSB" ~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py
# harus keluar tepat satu baris:  12:        default='/dev/ttyUSB1'

sed -i "s|default='/dev/ttyUSB1'|default='/dev/rplidar'|" \
  ~/ros2_ws/src/sllidar_ros2/launch/sllidar_c1_launch.py

cd ~/ros2_ws && colcon build --packages-select sllidar_ros2 && source install/setup.bash
```

Satu baris ini memperbaiki **ketiga** launch yang meng-include berkas tersebut:

- `sweep_mapping/launch/sweep_mapping.launch.py`
- `sweep_mappingimu/launch/sweep_mappingimu.launch.py`
- `stepper_controller/launch/mapping_system.launch.py`

Jangan ganti seluruh berkasnya dengan salinan laptop — versi di Raspi belum
tentu identik.

## Verifikasi

```bash
ros2 launch sweep_mapping sweep_mapping.launch.py sweeps:=1
```

Yang harus hilang: `SL_RESULT_OPERATION_TIMEOUT`, `multiple access on port`,
dan `Stepper jalan tapi /scan KOSONG` di 5 detik pertama.
Yang harus muncul: `Published cloud: <ratusan ribu> titik`.

Lalu **reboot tanpa monitor, keyboard, dan mouse, dan ulangi.** Itu kondisi
yang dulu gagal, dan satu-satunya bukti yang benar-benar berarti.

---

## JANGAN sentuh sisi IMU

Setelah Langkah 1, `/dev/imu_usb` sudah pasti benar dan
`wit_ros2_imu.py:153` memakainya langsung. Biarkan.

Jebakan kalau suatu saat tergoda "merapikan" node IMU supaya membaca parameter:

```python
# rviz_and_imu.launch.py:10-11  — dikirim ke node, tapi DIABAIKAN
parameters=[{'port': '/dev/ttyUSB0'},     # <-- ini port LiDAR!
            {"baud": 9600}]                # <-- kode sebenarnya pakai 460800
```

Dua nilai itu aman **justru karena diabaikan** (`wit_ros2_imu.py:143-144`,
baris `get_parameter` dikomentari). Begitu node dibuat membacanya, IMU langsung
diarahkan ke port LiDAR dengan baud yang salah. Kalau mau dirapikan,
`rviz_and_imu.launch.py` **harus** diperbaiki dalam perubahan yang sama.

## Yang tidak terdampak sama sekali

Tidak ada satu pun yang membuka serial:

- `mapping_3d_sweep.py`, `mapping_3d_imu.py`
- `stepper_sweep_node.py` (GPIO, bukan serial), `bag_recorder_node.py`
- `cloudcom/`, `pointcloud_studio/` — bekerja dari file mcap
- FAST-LIO — bekerja dari topic, bukan port

## Sudah tidak berlaku

Catatan lama soal adapter `usb-1a86_USB_Single_Serial_58DD027507` (CH340/CH9102)
yang sempat muncul di `/dev/serial/by-id/`: seluruh pemeriksaan berikutnya hanya
menemukan dua adapter Silicon Labs. Kemungkinan besar keluaran itu berasal dari
terminal atau mesin lain. Tidak perlu ditelusuri kecuali muncul lagi.
