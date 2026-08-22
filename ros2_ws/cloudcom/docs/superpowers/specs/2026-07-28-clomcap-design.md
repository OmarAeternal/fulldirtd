# clomcap — satu perintah: MCAP → point cloud + grid → CloudCompare

Tanggal: 2026-07-28

## Tujuan

Menyiapkan CloudCompare secepat mungkin dari satu file hasil scan. Satu baris
perintah menerima file `.mcap`, mengekstrak topik point cloud (`/map_3d`),
menulis `.ply`, membuat grid referensi yang pas dengan luas scan, lalu membuka
CloudCompare berisi keduanya — tanpa langkah manual di antaranya.

Kecepatan adalah kriteria utama. Setiap langkah yang tidak dibutuhkan untuk
menampilkan point cloud di CloudCompare bersifat opsional.

Sebelum ini, alurnya tiga perintah manual: `mcaptopc_cli.py`, lalu
`make_grid.py`, lalu `clocom` dengan dua path file diketik tangan.

## Ruang lingkup

Termasuk:

- Skrip driver `clomcap.py` di `cloudcom/`.
- Fungsi bash `clomcap` di `~/.bash_aliases`, bersebelahan dengan `clocom`.
- Tes pytest untuk fungsi-fungsi murni driver.

Tidak termasuk:

- Perubahan pada `mcaptopc.py`, `mcaptopc_cli.py`, `make_grid.py`, atau
  `cleanpc.py`. Ketiganya dipakai ulang sebagai pustaka lewat `import`, tidak
  diubah. Alur lama tetap berjalan seperti sebelumnya.
- Penghapusan `tes/grid_1m.ply`. File itu tidak lagi dipakai alur ini, tapi
  dibiarkan di tempatnya.
- Filter/pembersihan point cloud (`cleanpc.py`) di dalam alur ini.

## Antarmuka

```
clomcap <file> [-t TOPIK] [--force] [--png] [--spacing M] [--margin M] [--no-grid]
```

| Argumen      | Arti                                                                  |
| ------------ | --------------------------------------------------------------------- |
| `<file>`     | `.mcap`, `.mcap.zstd`, `.ply`, atau `.xyz`                            |
| `-t TOPIK`   | paksa topik tertentu; default deteksi otomatis (prefer `/map_3d`)     |
| `--force`    | konversi ulang walau cache masih segar                                |
| `--png`      | buat juga PNG visualisasi matplotlib (default: tidak, demi kecepatan) |
| `--spacing`  | jarak garis grid dalam meter (default 1.0)                            |
| `--margin`   | margin grid di luar data dalam meter (default 1.0)                    |
| `--no-grid`  | buka point cloud saja, tanpa grid                                     |

Contoh:

```bash
clomcap tes/016/016_0.mcap
clomcap scan_0003_3sweep_0.mcap -t /map_3d --force
clomcap tes/016/016_0_pointcloud.xyz        # file lama, langsung dibuka
```

## Arsitektur

`clomcap.py` adalah driver tipis. Kerja berat sudah ada di modul yang ada dan
dipakai ulang lewat `import`, bukan `subprocess` — menghindari biaya start
interpreter dan membuat penanganan error langsung berupa exception.

Dipakai dari `mcaptopc.py`: `inspect_mcap`, `extract_pointcloud2_frames`,
`extract_laserscan_frames`, `frames_to_pointcloud`, `export_ply`, `visualise`,
konstanta `POINTCLOUD2_SCHEMAS` / `LASER_SCAN_SCHEMAS` / `PREFERRED_PC2_TOPIC`.

`pick_lidar_topic` sengaja **tidak** dipakai: bila tidak ada schema LiDAR yang
dikenali, fungsi itu masuk `while True` yang memanggil `input()`, dan pada
`EOFError` berputar tanpa henti (`mcaptopc.py:200-208`). Perintah ini harus
non-interaktif, jadi driver punya `select_topic(topics, override)` sendiri yang
menyalin prioritas yang sama (`/map_3d` → PointCloud2 lain → LaserScan) tapi
keluar dengan `[ERROR]` + daftar topik alih-alih bertanya. `mcaptopc.py` tetap
tidak diubah.

Dipakai dari `mcaptopc_cli.py`: `looks_zstd`, `decompress_zstd`, `clean_stem`.

Dipakai dari `make_grid.py`: `build_grid`, `write_ply`, `ply_xyz_minmax`.

Driver menyumbang lima fungsi murni (mudah dites, tanpa I/O eksternal):

| Fungsi                          | Tanggung jawab                                                  |
| ------------------------------- | --------------------------------------------------------------- |
| `classify_input(path)`          | → `"mcap"` \| `"cloud"`; menentukan perlu konversi atau tidak    |
| `select_topic(topics, override)` | → `(topik, schema)`; non-interaktif, error bila tak ada yang cocok |
| `out_paths(src, root)`          | → `(dir, ply, png, grid)` untuk satu input                       |
| `is_cache_fresh(src, ply)`      | → bool, berdasarkan keberadaan dan mtime                         |
| `xyz_minmax(path)`              | → `(min, max)` XYZ dari file `.xyz` teks                         |
| `bounds_of(path)`               | dispatch `.ply` → `ply_xyz_minmax`, `.xyz` → `xyz_minmax`        |

Dan dua fungsi ber-efek: `convert_to_ply(...)` dan `launch_cloudcompare(paths)`.

## Alur

1. **Klasifikasi.** `classify_input` melihat ekstensi. `.ply`/`.xyz` → langkah 4
   memakai file itu apa adanya di tempatnya (tidak disalin ke `out/`).
   `.mcap`/`.mcap.zstd` → lanjut.
2. **Cache.** `is_cache_fresh` benar bila `out/<stem>/<stem>.ply` ada dan
   `mtime`-nya ≥ `mtime` sumber. Bila segar dan tanpa `--force`, langkah 3
   dilewati seluruhnya.
3. **Konversi.** Dekompresi zstd ke file sementara bila perlu (dihapus di
   `finally`) → `inspect_mcap` → pilih topik → ekstrak sesuai schema →
   `np.vstack` frame → `export_ply` ke `out/<stem>/<stem>.ply`. PNG lewat
   `visualise` hanya bila `--png`.
4. **Grid.** Kecuali `--no-grid`: `bounds_of(cloud)` → `build_grid` dengan
   `xmin/ymin` = min − margin dan `xmax/ymax` = max + margin, `z=0`,
   spasi `--spacing`, mayor tiap 5 m → `write_ply` ke `out/<stem>/grid.ply`.
   Grid selalu dibuat ulang; biayanya kecil dan menjamin cakupannya benar.
   Untuk input `.ply`/`.xyz` di luar `out/`, grid tetap ditulis ke
   `out/<stem>/grid.ply`.
5. **Buka.** `setsid flatpak run org.cloudcompare.CloudCompare <grid> <cloud>`
   dengan stdio ke `DEVNULL`, tanpa `wait` — proses lepas dari terminal, sama
   seperti `clocom`. Driver keluar segera setelah spawn.

## Layout output

```
cloudcom/out/<stem>/
    <stem>.ply      point cloud hasil konversi
    <stem>_viz.png  hanya bila --png
    grid.ply        grid auto-fit scan ini
```

`<stem>` untuk input `.mcap`/`.mcap.zstd` berasal dari `clean_stem`:
`016_0.mcap` → `016_0`, `scan_0003_3sweep_0.mcap.zstd` → `scan_0003_3sweep_0`.
Untuk input `.ply`/`.xyz`, `clean_stem` tidak berlaku (ia hanya membuang
`.mcap`/`.zstd`) — stem-nya adalah nama file tanpa ekstensinya sendiri:
`016_0_pointcloud.xyz` → `016_0_pointcloud`. Dalam kasus ini hanya `grid.ply`
yang ditulis ke `out/<stem>/`; point cloud dibuka dari lokasi aslinya.

Akar `out/` selalu `cloudcom/out/`, ditentukan relatif
terhadap lokasi `clomcap.py` (bukan direktori kerja), agar hasilnya sama dari
mana pun perintah dijalankan.

`out/` ditambahkan ke `cloudcom/.gitignore`.

## Penanganan error

Semua kegagalan fatal keluar dengan pesan berawalan `[ERROR]` dan status ≠ 0.

| Kondisi                     | Perilaku                                                |
| --------------------------- | ------------------------------------------------------- |
| file input tidak ada        | error, sebut path-nya                                    |
| ekstensi tak dikenal        | error, sebut ekstensi yang didukung                      |
| topik `-t` tidak ada        | error + daftar topik yang tersedia di file itu            |
| tidak ada frame valid       | error                                                    |
| nol titik                   | error                                                    |
| `flatpak` tidak ditemukan   | error, sebut path `.ply` yang sudah jadi agar bisa dibuka manual |
| `visualise` gagal (`--png`) | `[WARN]`, lanjut — CloudCompare tetap dibuka             |
| pembuatan grid gagal        | `[WARN]`, lanjut — CloudCompare dibuka tanpa grid        |

Dua yang terakhir sengaja tidak fatal: tujuan perintah ini adalah membuka
CloudCompare, dan kegagalan pelengkap tidak boleh menghalanginya.

## Fungsi bash

Ditambahkan ke `~/.bash_aliases`, tidak mengubah `clocom` yang sudah ada:

```bash
# MCAP/PLY/XYZ → CloudCompare + grid, satu perintah
clomcap() {
    "$HOME/riset td/ros2_ws/cloudcom/.venv/bin/python" \
        "$HOME/riset td/ros2_ws/cloudcom/clomcap.py" "$@"
}
```

Memanggil interpreter venv langsung, jadi tidak perlu `source activate` dan
tidak mengubah environment shell pemanggil. Proses CloudCompare di-detach di
dalam `clomcap.py` lewat `setsid`, bukan di bash.

## Pengujian

`cloudcom/test_clomcap.py`, pytest, senada `test_cleanpc.py`. Hanya fungsi murni
— tidak ada tes yang membuka window atau memanggil flatpak sungguhan.

- `classify_input`: `.mcap`, `.mcap.zstd`, `.ply`, `.xyz`, huruf besar-kecil,
  ekstensi tak dikenal → raise.
- `out_paths`: stem benar untuk `.mcap` dan `.mcap.zstd`; akar selalu di bawah
  `out/`.
- `is_cache_fresh`: ply tidak ada → False; ply lebih tua dari sumber → False;
  ply lebih baru → True. Pakai `tmp_path` dan `os.utime`.
- `xyz_minmax`: file `.xyz` kecil buatan sendiri → min/max benar; baris kosong
  dan komentar dilewati; file kosong → raise.
- `bounds_of`: dispatch ke fungsi yang benar; ekstensi lain → raise.
- `launch_cloudcompare`: `subprocess.Popen` di-monkeypatch, verifikasi urutan
  argumen (grid sebelum cloud) dan `flatpak` tidak ada → error yang benar.

## Keputusan yang perlu dicatat

- **PNG default mati.** Sebelumnya `mcaptopc_cli.py` selalu membuatnya. Di sini
  tujuannya kecepatan, jadi jadi opt-in lewat `--png`.
- **Tidak ada ekspor `.xyz`.** Format itu tidak dibutuhkan CloudCompare dan
  ukurannya besar (7,5 MB untuk scan 4 MB). `.xyz` tetap diterima sebagai input.
- **Grid tidak di-cache.** Point cloud di-cache karena mahal; grid murah dan
  bergantung pada batas cloud, jadi selalu dibuat ulang.
- **Grid sebelum cloud di argumen CloudCompare.** Urutan ini membuat grid
  menjadi entitas pertama di DB tree, konsisten setiap kali.
