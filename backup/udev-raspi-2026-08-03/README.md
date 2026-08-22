# Cadangan aturan udev Raspi — sebelum perbaikan 3 Agustus 2026

Isi `/etc/udev/rules.d/` di Raspi (`raspitampan`) **sebelum** ketiganya diganti
satu file `99-lidar-imu.rules` yang mencocokkan per serial.

| Berkas | Asal |
|---|---|
| `imu_usb.rules` | sudah ada sejak lama, pembuatnya tidak diketahui |
| `rplidar.rules` | sudah ada sejak lama, komentarnya bawaan panduan Slamtec |
| `99-rplidar.rules` | dibuat 3 Agustus 2026 saat mencoba memperbaiki, ternyata tidak berefek |

## Kenapa diganti

`imu_usb.rules` dan `rplidar.rules` **identik** — keduanya hanya mencocokkan
VID:PID `10c4:ea60`. Padahal CP2102 (IMU) dan CP2102N (LiDAR) sama-sama
`10c4:ea60`, jadi tiap aturan kena ke kedua adapter. Setiap device mendapat
symlink `imu_usb` DAN `rplidar`, dan yang menang adalah device yang kebetulan
diproses terakhir — acak tiap boot.

`99-rplidar.rules` menambahkan pencocokan per serial untuk LiDAR, tapi tidak
menolong: `rplidar.rules` yang lama tetap membuat device satunya ikut mengklaim
nama `rplidar`, jadi perebutannya tidak hilang.

Rinciannya lengkap di `../../TODO_PERBAIKAN_PORT_RASPI.md`.

## Sumber salinan ini

Direkonstruksi dari keluaran `sudo cat` pada 3 Agustus 2026. Isi barisnya persis,
tapi **spasi/baris kosong di ujung berkas tidak dijamin sama** — `cat`
menggabungkan berkas sehingga batasnya tidak terlihat pasti. Untuk udev hal itu
tidak berpengaruh; aturan dibaca per baris.

Salinan yang benar-benar byte-per-byte ada di Raspi:
`~/udev_backup_2026-08-03/` (dipindahkan dengan `mv`, bukan dihapus).

## Cara mengembalikan kalau perbaikan bermasalah

Di Raspi:

```bash
sudo mv ~/udev_backup_2026-08-03/*.rules /etc/udev/rules.d/
sudo rm -f /etc/udev/rules.d/99-lidar-imu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Kalau folder di Raspi ikut hilang, kirim ulang dari sini:

```bash
rsync -avz ~/"riset td/backup/udev-raspi-2026-08-03"/*.rules raspitampan:~/pulih/
# lalu di Raspi:  sudo mv ~/pulih/*.rules /etc/udev/rules.d/
```

Ingat bahwa mengembalikan ini berarti **mengembalikan bug-nya** — symlink akan
acak lagi tiap boot. Hanya lakukan kalau perbaikan barunya ternyata lebih buruk.
