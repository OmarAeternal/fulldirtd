# SETUP — memasang di komputer baru

Panduan ini membawamu dari `git clone` sampai bisa menggabungkan scan sendiri.
Perkiraan waktu: 20–30 menit, sebagian besar hanya menunggu unduhan.

---

## 1. Apa isi repo ini

Ada **dua perangkat yang berdiri sendiri**, dipakai bergantian:

| Perangkat | Letak | Gunanya |
|---|---|---|
| **cloudcom** | `ros2_ws/cloudcom/` | Mengubah rekaman `.mcap` jadi point cloud, lalu menggabungkan beberapa scan jadi satu peta |
| **PointCloud Studio** (`pcs`) | `pointcloud_studio/` | Melihat, mengukur, dan menyunting point cloud lewat browser |

Selain itu ada `ros2_ws/src/` — paket ROS 2 untuk **mengambil** scan dari perangkat
kerasnya (LiDAR + IMU + motor di Raspberry Pi). Bagian ini **tidak perlu kamu pasang**
kecuali kamu memang mau merakit alatnya. Lihat bagian 6.

---

## 2. Sistem operasi — apa yang sebenarnya dibutuhkan

Repo ini dikembangkan di Ubuntu 24.04, tapi **Ubuntu tidak wajib.**
Yang mengunci ke Ubuntu 24.04 hanyalah ROS 2 Jazzy di `ros2_ws/src/`, dan itu bagian
yang tidak kamu butuhkan untuk mengolah data.

Seluruh cloudcom dan PointCloud Studio adalah **Python biasa** — tidak ada satu pun
yang mengimpor ROS. Berkas `.mcap` dibaca lewat pustaka `mcap`, bukan lewat ROS.
Jadi Fedora, Arch, Debian, openSUSE, semuanya jalan.

### Yang benar-benar mengikat: **Python 3.12**

Bukan 3.13. Open3D 0.19.0 adalah rilis terbaru dan **belum menyediakan wheel untuk
Python 3.13**. Kalau kamu memaksa dengan 3.13, `pip install` akan gagal dengan
"No matching distribution found for open3d".

Ini penting untuk **pengguna Fedora**: Fedora 41 ke atas memakai Python 3.13 sebagai
bawaan, jadi 3.12 perlu dipasang terlebih dulu. Perintahnya ada di langkah 2 di bawah.

### CloudCompare

Dibuka lewat **flatpak**. Fedora sudah membawa flatpak sejak lama, jadi tidak ada
masalah. Kalau flatpak tidak ada, skrip tetap menyelesaikan pekerjaannya dan hanya
memberi tahu letak berkas hasilnya, supaya kamu buka sendiri.

### Windows dan macOS

Bagian Python-nya jalan (Open3D menyediakan wheel untuk keduanya), tapi pembuka
CloudCompare otomatis memakai `flatpak` dan `setsid` yang khas Linux — jadi di sana
berkas hasil harus dibuka manual. Belum diuji. Lihat bagian 7 soal Docker.

---

## 3. Pemasangan

### Langkah 1 — ambil repo

```bash
git clone https://github.com/OmarAeternal/fulldirtd.git
cd fulldirtd
```

Boleh di-clone ke folder bernama apa saja; semua skrip mencari letaknya sendiri.

### Langkah 2 — pastikan ada Python 3.12

```bash
python3.12 --version
```

Kalau belum ada:

```bash
# Fedora
sudo dnf install python3.12

# Ubuntu / Debian
sudo apt install python3.12 python3.12-venv

# Arch
yay -S python312
```

### Langkah 3 — pasang cloudcom

```bash
cd ros2_ws/cloudcom
python3.12 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
cd ../..
```

Open3D besarnya sekitar 400 MB, jadi bagian ini yang paling lama.

### Langkah 4 — pasang PointCloud Studio

```bash
cd pointcloud_studio
./run.sh
```

`run.sh` membuat venv-nya sendiri dan langsung membuka browser. Tekan `Ctrl+C`
untuk berhenti. Cukup sekali; setelah ini `run.sh` langsung menyala.

> Kedua venv **sengaja dipisah**: PointCloud Studio menahan `numpy<2`, sedangkan
> cloudcom berjalan di numpy 2.x. Jangan digabung.

### Langkah 5 — pasang CloudCompare

```bash
# Fedora & Ubuntu sama saja
flatpak install flathub org.cloudcompare.CloudCompare
```

Kalau flatpak belum ada di Fedora: `sudo dnf install flatpak`

### Langkah 6 — daftarkan perintahnya

```bash
echo "source \"$(pwd)/perintah.sh\"" >> ~/.bashrc
source ~/.bashrc
```

Setelah ini tersedia perintah: `clomcap`, `clomcaps`, `clomerge`, `clomergeout`,
`outmerge`, `clomerged`, `pasak`, `pcs`, dan `clocom` — jalan dari folder mana pun.

---

## 4. Pastikan pemasanganmu benar

Jalankan tesnya:

```bash
cd ros2_ws/cloudcom
.venv/bin/python -m pytest -q
```

Semua harus lulus. Kalau ada yang gagal, **jangan lanjut** — laporkan hasilnya,
karena artinya ada versi pustaka yang tidak cocok dan angka hasil olahanmu nanti
tidak akan sama dengan yang tercatat di penelitian ini.

> Kalau pytest berhenti dengan `ModuleNotFoundError: No module named 'lark'`,
> berarti venv-mu ikut membaca pustaka ROS 2 yang terpasang di sistem. Jalankan
> dengan `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` di depannya. Di komputer tanpa ROS 2,
> ini tidak akan terjadi.

Tes registrasi utuh memang lambat. Untuk pemeriksaan cepat:

```bash
.venv/bin/python -m pytest -q -m "not slow"
```

Lalu coba satu scan sungguhan:

```bash
cd ../../cloudcom
clomcap scan_0080_1sweep_0.mcap
```

CloudCompare harus terbuka berisi point cloud beserta grid ukurnya.

---

## 5. Data scan

Repo ini **hanya memuat scan 0080 ke atas** (31 rekaman) beserta hasil olahannya di
`cloudcom/out/`. Itu sudah cukup untuk belajar dan mengulang hasil terakhir.

Yang **tidak** ada di sini, karena melebihi batas ukuran GitHub:

- scan 0001–0079 (±840 MB)
- `cloudcom/out/_merge`, `_merge_out`, `_outmerge` (±2,3 GB, percobaan penggabungan lama)

Minta ke pemilik repo kalau butuh; dikirim lewat Drive atau cakram keras.

Semua isi `cloudcom/out/` sebenarnya **hasil hitungan** — kalau punya `.mcap` mentahnya,
semuanya bisa kamu lahirkan ulang sendiri.

Baca `ros2_ws/cloudcom/CARA_PAKAI.txt` untuk daftar lengkap perintah dan pilihannya.

---

## 6. Kalau mau merakit alat pemindainya

Baru di sinilah Ubuntu 24.04 jadi wajib, karena ROS 2 Jazzy hanya mendukung itu.
Yang dibutuhkan: Raspberry Pi 5, RPLiDAR, IMU WitMotion, dan motor stepper pemiring.

Panduannya:

- `ros2_ws/PANDUAN_SISTEM.md` — rangkaian dan cara menyalakan
- `ros2_ws/PANDUAN_PORT_SERIAL.md` — penetapan port USB yang tidak berpindah-pindah
- `backup/udev-raspi-2026-08-03/` — aturan udev-nya *(tidak ikut di repo ini)*

Untuk sekadar mengolah data, lewati bagian ini sepenuhnya.

---

## 7. Perlukah Docker?

**Untuk Linux (termasuk Fedora): tidak.** Masalahnya cuma versi Python, dan itu
selesai dengan memasang `python3.12` — jauh lebih ringan daripada menyiapkan Docker,
apalagi CloudCompare dan PointCloud Studio keduanya butuh tampilan grafis, yang di
dalam Docker berarti mengurus penerusan X11/Wayland. Repot tanpa imbalan.

**Docker baru masuk akal kalau** ada yang memakai Windows atau macOS dan ingin hasil
yang persis sama. Bahkan begitu, cara yang lebih murah: jalankan pengolahannya dalam
wadah tanpa tampilan, lalu buka berkas `.ply` hasilnya dengan CloudCompare yang
dipasang biasa di sistem masing-masing. Dua langkah, tapi tidak ada X11 yang perlu
diakali.

Belum ada `Dockerfile` di repo ini. Kalau nanti dibutuhkan, mintalah.

---

## 8. Masalah yang sudah diketahui

**`CARA_PAKAI.txt` masih menyebut path lama.** Di dalamnya tertulis
`CC="$HOME/riset td/ros2_ws/cloudcom"`. Abaikan — `perintah.sh` sudah menemukan
letak repo sendiri, jadi `$CC` tidak perlu kamu setel.

**Hasil selalu masuk ke `cloudcom/out/`,** di mana pun kamu menjalankan perintahnya.
Mau pindah? `export CLOUDCOM_OUT="/tempat/lain"`

**Ada dua folder bernama mirip.** `cloudcom/` di akar berisi *data*; kodenya di
`ros2_ws/cloudcom/`. Membingungkan, tapi begitulah adanya.
