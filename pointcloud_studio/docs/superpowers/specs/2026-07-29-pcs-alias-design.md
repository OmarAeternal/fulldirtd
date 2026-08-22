# `pcs` — membuka berkas langsung di PointCloud Studio

Tanggal: 2026-07-29

## Masalah

PointCloud Studio hanya bisa memuat berkas lewat unggahan di browser: `POST /load`
menerima `UploadFile`, dan `app.js` memanggilnya dari tombol "Buka" atau drag-and-drop.
Tidak ada jalur untuk membuka berkas dari path.

Akibatnya, untuk melihat hasil `clomerge` pemakainya harus menyalakan `run.sh`, menunggu
browser terbuka, menekan "Buka", lalu menyusuri pohon direktori sampai
`out/_merge/merged_check.ply`. Perkakas sebelah (`clomcap`, `clomcaps`, `clomerge`) semuanya
satu perintah dengan argumen berkas; PointCloud Studio jadi terasa asing.

Berkas MCAP sama sekali tidak bisa dibuka: `loader.parse()` hanya paham PLY dan XYZ.

## Sasaran

Satu perintah yang menerima berkas dan menampilkannya:

```
pcs merged.ply
pcs scan_0007_1sweep_0.mcap
```

## Bukan sasaran

- **Banyak berkas sekaligus** (`pcs a.ply b.ply`). `app.js` menyimpan satu cloud;
  `loadFile()` menggantikan isi lama, bukan menambah. Mendukung banyak cloud berarti
  merombak model datanya — proyek tersendiri.
- **Parser MCAP di dalam PointCloud Studio.** Konversi tetap milik `ros2_ws/cloudcom`.
- **Mengubah berkas asli di disk.** Optimasi kerapatan hanya berlaku pada salinan
  dalam memori yang dikirim ke browser.

## Antarmuka perintah

```
pcs BERKAS [--voxel M] [--full] [--port N] [--force] [-t TOPIK]

BERKAS      .ply, .xyz, .mcap, atau .mcap.zstd
--voxel M   ukuran voxel dalam meter (default 0.01)
--full      lewati optimasi, kirim resolusi penuh
--port N    port server (default 8000)
--force     konversi ulang MCAP walau hasil lama masih segar
-t TOPIK    paksa topik MCAP tertentu, mis. /map_3d
```

Alurnya:

1. Berkas diselesaikan jadi path PLY/XYZ (MCAP dikonversi, memakai cache bila segar).
2. Server dipastikan hidup: dipakai ulang bila sudah mendengar di port itu, dinyalakan
   terlepas dari terminal bila belum.
3. Browser dibuka ke `http://127.0.0.1:PORT/?file=<path>&voxel=<m>[&full=1]`.

Perhatikan dua lapis parameter yang namanya sengaja berbeda: URL halaman memakai `file=`,
lalu frontend menerjemahkannya jadi panggilan API `/open?path=`. Bedanya menjaga agar
parameter halaman dan parameter endpoint bisa berubah sendiri-sendiri. Bila `--full`
dipakai, `full=1` ikut di URL halaman dan diteruskan ke `/open`.

## Optimasi kerapatan

Downsample berbasis voxel, bukan pembuangan acak: ruang dibagi kubus bersisi `voxel`
dan tiap kubus menyisakan satu titik. Bentuk geometri terjaga; yang hilang hanya titik
yang menumpuk di tempat yang sama.

Diukur pada `out/_merge/merged.ply` hasil `clomerge` scan 0007–0010 (2.400.009 titik,
bbox 13,2 × 17,2 × 4,6 m). Perhatikan bahwa `out/_merge/` ditimpa setiap kali `clomerge`
dijalankan, jadi angka di bawah ini adalah potret satu keluaran, bukan sifat tetap
berkasnya:

| voxel | titik/m² permukaan | hasil | sisa |
|---|---|---|---|
| 0,5 cm | 40.000 | 959.235 | 40,0 % |
| **1 cm (default)** | **10.000** | **810.101** | **33,8 %** |
| 2 cm | 2.500 | 544.462 | 22,7 % |
| 5 cm | 400 | 165.476 | 6,9 % |

Default 1 cm dipilih karena berada di bawah tingkat kebisingan sensor LiDAR, sehingga
tidak ada detail nyata yang hilang, sementara jumlah titik turun ke sepertiga.

**Batas atas 3.000.000 titik.** Bila setelah voxel jumlahnya masih di atas batas, voxel
digandakan lalu diukur lagi, paling banyak 6 kali penggandaan (jadi voxel akhir maksimum
64× voxel awal — untuk default 1 cm berarti mentok di 64 cm). Bila setelah itu masih di
atas batas, yang ada dikirim apa adanya disertai peringatan; perintah tidak gagal. Ini
jaring pengaman untuk berkas
yang jauh lebih besar dari data sekarang, bukan jalur yang biasa terpakai.

`--full` melewati seluruh mekanisme ini.

## Perubahan kode

### `backend/server.py` — endpoint `GET /open`

```
GET /open?path=<absolut>&voxel=<meter>&full=<0|1>
```

Membaca berkas dari disk, menerapkan optimasi kerapatan, lalu membalas **dalam format
yang persis sama dengan `/load`**: body biner Float32 `[x,y,z,r,g,b] * N` little-endian
dengan statistik di header `X-Stats`. Kesamaan format ini disengaja — frontend memakai
satu jalur render untuk kedua sumber.

Header tambahan `X-Downsample` berisi JSON `{voxel, n_asli, n_kirim}` supaya frontend
bisa memberi tahu pemakai bahwa yang dilihat sudah dioptimasi.

Validasi: path harus absolut, menunjuk berkas biasa yang ada, dan berekstensi
`.ply`/`.xyz`. Path yang gagal validasi dibalas 400, bukan 404, supaya tidak bisa dipakai
menebak keberadaan berkas.

### `frontend/app.js` — muat otomatis dari URL

`loadFile()` (baris 150) sekarang mengerjakan dua hal: mengambil data lewat `POST /load`,
lalu merender hasilnya. Bagian kedua dipisah jadi fungsi tersendiri yang menerima
`ArrayBuffer` + stats. Dengan begitu jalur unggahan dan jalur path memakai kode render
yang sama.

Saat halaman dimuat, `?file=` dibaca dari URL. Bila ada, `/open` dipanggil dan hasilnya
masuk ke fungsi render yang sama. Bila `X-Downsample` menunjukkan titik berkurang,
keterangannya ditampilkan di baris petunjuk.

### `pointcloud_studio/pcs.py` — perintahnya

Konversi MCAP dijalankan dengan memanggil interpreter proyek sebelah:

```
"$HOME/riset td/ros2_ws/cloudcom/.venv/bin/python" -c "<panggil clomcaps.prepare_cloud>"
```

`prepare_cloud()` (`clomcaps.py:99`) sudah menangani `.ply`/`.xyz` apa adanya, konversi
`.mcap`, dan cache. Memakainya lewat subprocess menjaga kedua venv tetap terpisah:
PointCloud Studio tidak perlu pustaka MCAP, `ros2_ws` tidak perlu FastAPI.

Server dianggap hidup bila ada yang mendengar di port itu **dan** `GET /` membalas 200.
Bila port dipakai proses lain yang bukan PointCloud Studio, perintah berhenti dengan
saran memakai `--port`.

### `~/.bash_aliases` — alias

```bash
# Point cloud (PLY/XYZ/MCAP) → PointCloud Studio di browser
pcs() {
    "$HOME/riset td/pointcloud_studio/.venv/bin/python" \
        "$HOME/riset td/pointcloud_studio/pcs.py" "$@"
}
```

Mengikuti pola `clomcap`/`clomcaps`/`clomerge` yang sudah ada di berkas itu.

## Penanganan galat

| Keadaan | Perilaku |
|---|---|
| Berkas tidak ada | Berhenti, sebut path yang dicari |
| Ekstensi tidak dikenal | Berhenti, sebutkan yang didukung |
| MCAP tapi venv `ros2_ws` hilang | Berhenti, sebut path venv yang diharapkan |
| Konversi MCAP gagal | Teruskan pesan galat dari `prepare_cloud` |
| Port dipakai proses lain | Berhenti, sarankan `--port` |
| Server gagal menyala | Tampilkan log uvicorn, jangan buka browser |
| Browser tak bisa dibuka | Cetak URL-nya agar bisa disalin manual |

## Pengujian

Proyek ini belum punya tes. Yang ditambahkan hanya untuk bagian yang murni logika,
dengan pytest:

- **Pemilihan voxel** — di bawah batas voxel tidak berubah; di atas batas voxel
  digandakan sampai muat; `--full` tidak mengubah apa pun; berkas kecil tidak rusak.
- **Validasi path `/open`** — path relatif, direktori, berkas tidak ada, dan ekstensi
  asing semuanya ditolak 400; berkas PLY sah diterima 200 dengan `X-Stats` yang benar.
- **Argumen `pcs`** — nilai bawaan, penguraian flag, penolakan ekstensi tak dikenal.

Jalur browser dan pembukaan jendela tidak ditest otomatis; diperiksa manual dengan
`pcs merged.ply`.

## Yang bisa dipakai untuk memeriksa hasil

```
pcs "$HOME/riset td/ros2_ws/cloudcom/out/_merge/merged_check.ply"
```

Berhasil bila browser terbuka, cloud tampil tanpa perlu klik apa pun, dan baris petunjuk
menyebut jumlah titik beserta keterangan voxel.

Hasil sebenarnya pada keluaran `clomerge` scan 0003–0006 (1.456.757 titik):
voxel 1 cm → **467.594 titik terkirim (32,1 %)**, `melebihi_batas` false. Rasionya
sejalan dengan pengukuran scan 0007–0010 di atas (33,8 %).
