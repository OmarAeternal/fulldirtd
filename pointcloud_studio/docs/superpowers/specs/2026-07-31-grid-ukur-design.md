# Spec 2 — Grid referensi + alat ukur

**Tanggal:** 2026-07-31 · **Status:** disetujui, siap direncanakan
**Prasyarat:** Spec 1 (`2026-07-31-layer-multi-file-design.md`) sudah diimplementasikan —
modul `viewer.js`, `layers.js`, `measure.js`, `hud.js` sudah ada dan `cloud` global
sudah diganti daftar layer.

**Konteks:** mengukur di PointCloud Studio sekarang berarti klik dua titik dan berharap
mengenai titik yang benar. Tidak ada acuan geometris, tidak ada umpan balik sebelum
klik kedua, dan hasilnya langsung tertimpa oleh pengukuran berikutnya.

## Tujuan

1. **Grid referensi** yang bisa digeser dan diputar bebas di ruang 3D, dipasang di
   lantai atau dinding, sebagai acuan pengukuran.
2. **Alat ukur karet (rubber-band)**: klik pertama menancapkan ujung, garis mengikuti
   kursor dengan angka hidup, klik kedua mengunci.
3. **Hasil ukur menetap** — tergambar di scene dengan label, terdaftar di sidebar,
   bisa diekspor ke CSV untuk tabel validasi skripsi.

## Di luar cakupan (YAGNI)

- **Snap ke perpotongan grid.** Klik pada grid mendarat persis di bawah kursor, tidak
  menempel ke simpul.
- **Tegak-lurus ke bidang grid** sebagai jenis pengukuran tersendiri.
- **Rincian ΔX/ΔY/ΔZ** dan **kunci sumbu** saat menarik garis.
- **Beberapa grid sekaligus.** Tepat satu grid, hidup atau mati.
- **Skala grid (scale) lewat gizmo.** Ukuran grid hanya lewat panel numerik.
- **Menyimpan/memuat sesi pengukuran.** Ekspor CSV jalan keluarnya; tutup tab, hilang.

## Perubahan backend

**Tidak ada.** Seluruh spec ini di frontend. "Pasang grid ke bidang RANSAC" memakai
respons `/analyze` yang sudah memuat `normal`, `d`, `centroid`, dan `kind` untuk tiap
bidang. Ekspor CSV dibuat di browser.

---

## Dependensi baru

`frontend/vendor/TransformControls.js` — TransformControls r160, di-vendor dari
`unpkg.com/three@0.160.0/examples/jsm/controls/TransformControls.js` (40 KB), mengikuti
pola OrbitControls yang sudah ada. Satu-satunya impornya adalah `three`, yang sudah ada
di importmap. Ditambahkan ke importmap `index.html`:

```json
"three/addons/controls/TransformControls.js": "/static/vendor/TransformControls.js"
```

Tidak ada dependensi Python baru.

---

## Modul baru

```
ui.js
  ↓
grid.js      objek grid, transform, gizmo, pemasangan ke bidang
measure.js   ditulis ulang: rubber-band, jarak & sudut, hasil menetap, CSV
  ↓
layers.js
  ↓
viewer.js
```

`measure.js` mengimpor `grid.js` (untuk snap ke bidang grid). `grid.js` tidak mengimpor
`measure.js`. Arah tetap searah seperti Spec 1.

---

## `grid.js` — grid referensi

### Bentuk

Grid dibangun di **bidang XY lokal**, berpusat di titik asal grup, dengan normal =
+Z lokal. Jadi orientasi identitas = grid mendatar, cocok dengan data LiDAR yang Z-up.

```js
const grid = new THREE.Group();   // position + quaternion = transform grid
// isi grup:
//   garisMinor  LineSegments  — tiap `spasi`
//   garisMayor  LineSegments  — tiap 5 sel, lebih terang
//   sumbuX      Line          — garis tengah lokal X, merah gelap
//   sumbuY      Line          — garis tengah lokal Y, hijau gelap
```

Dua garis tengah berwarna itu penting: tanpanya, grid yang sudah diputar tidak bisa
dibedakan orientasinya.

Material memakai `depthWrite: false` dan `transparent: true` supaya grid tidak menutupi
titik di belakangnya. Grid **tidak** menerima `clippingPlanes` — saat mengiris
ketinggian Z, grid harus tetap terlihat sebagai acuan.

### Parameter

| Parameter | Bawaan | Rentang |
|---|---|---|
| `spasi` | 0,1 m | pilihan: 0,05 · 0,1 · 0,25 · 0,5 · 1,0 m |
| `ukuran` | dihitung dari data | 1–200 m, kotak isian |

`ukuran` bawaan = `max(dx, dy)` bounds gabungan layer terlihat, dibulatkan ke atas ke
kelipatan meter, dibatasi 4–40 m. Bila belum ada layer, 10 m.

**Batas jumlah garis.** `n = ukuran / spasi`. Bila `n > 400`, `ukuran` dipangkas jadi
`400 × spasi` dan panel menampilkan catatan `ukuran dipangkas ke <x> m (batas 400 garis)`.
Tanpa batas ini, spasi 5 cm pada ruangan 40 m menghasilkan 800 garis per sumbu dan
frame rate anjlok.

Geometry dibangun ulang hanya saat `spasi` atau `ukuran` berubah — bukan saat digeser
atau diputar.

### Transform: gizmo + panel numerik

**Gizmo** — `TransformControls` dipasang ke grup grid, ditambahkan ke `scene`.

```js
tc = new TransformControls(camera, renderer.domElement);
tc.attach(grid);
tc.addEventListener('dragging-changed', e => { controls.enabled = !e.value; });
scene.add(tc);
```

Mode gizmo dipilih lewat tiga tombol di panel: **Geser** (`setMode('translate')`) ·
**Putar** (`setMode('rotate')`) · **Mati** (`tc.enabled = false`, gizmo disembunyikan).
Skala sengaja tidak disediakan.

Sakelar **Snap gizmo** (bawaan nyala): `setTranslationSnap(spasi)` dan
`setRotationSnap(THREE.MathUtils.degToRad(15))`. Dengan snap nyala, memindahkan grid
tepat 1 m atau memutarnya tepat 90° jadi mudah; dimatikan untuk penyetelan halus.

`tc.setSpace('local')` — memutar grid yang sudah miring terasa lebih masuk akal relatif
terhadap bidangnya sendiri.

**Panel numerik** — enam kotak isian:

| Kotak | Isi |
|---|---|
| Posisi X / Y / Z | `grid.position`, meter, step 0,01 |
| Putar Z | yaw, derajat, step 0,5 |
| Miring X | pitch, derajat, step 0,5 |
| Miring Y | roll, derajat, step 0,5 |

Rotasi disimpan sebagai quaternion di `grid.quaternion`; kotak isian adalah Euler
dengan urutan **`'ZXY'`**. Urutan itu dipilih supaya "Putar Z" selalu berarti berputar
terhadap sumbu Z **dunia** — rotasi Z berada paling luar. Jadi setelah memiringkan grid
ke dinding, memutar Z tetap berperilaku seperti kompas, bukan berputar terhadap normal
dinding.

Kotak isian dan gizmo adalah dua tampilan dari state yang sama:
- gizmo diseret → `tc` melempar `change` → kotak isian ditulis ulang dari
  `grid.position` dan `new THREE.Euler().setFromQuaternion(grid.quaternion, 'ZXY')`
- kotak isian diketik → `grid.position` / `grid.quaternion` disetel dari isinya

Untuk mencegah lingkaran umpan balik (menulis kotak isian memicu event `input` yang
menulis balik transform, yang membulatkan sudut sedikit demi sedikit), penulisan
kotak isian dari gizmo dipagari flag `sedangMenulisUI`. Kotak isian juga diformat 3
desimal untuk meter dan 1 desimal untuk derajat.

### Memasang grid ke bidang

Tiga jalan, semuanya di panel Grid:

**1. `Pasang di 3 titik`** — klik tombol, lalu klik 3 titik di point cloud. Normal
bidang = `(p2−p1) × (p3−p1)` dinormalkan; posisi = rata-rata ketiga titik; quaternion
dari `setFromUnitVectors(+Z, normal)`.

- Normal dibalik bila `normal · (posisiKamera − pusat) < 0`, supaya sisi terang grid
  menghadap penonton.
- Bila ketiga titik nyaris segaris (`|normal| < 1e-6` sebelum normalisasi), ditolak
  dengan toast `Tiga titik terlalu segaris — pilih titik yang lebih menyebar`.
- Esc membatalkan; hint menunjukkan kemajuan `Pasang grid: klik titik 2 dari 3`.
- Mode ini mematikan sementara mode Ukur dan mode Pilih supaya kliknya tidak rebutan.

**2. `Bidang RANSAC ▾`** — daftar pilihan yang terisi setelah Analisis dijalankan,
memakai `d.planes` dari respons `/analyze`. Tiap entri diberi label dari `kind`,
`n_inliers`, dan `rmse_m`:

```
lantai/plafon · 48.201 titik · RMSE 0,8 cm
dinding · 31.550 titik · RMSE 1,2 cm
```

Memilih entri memasang grid: posisi = `centroid`, quaternion dari `normal`. Daftar
kosong (`Belum dianalisis`) sampai Analisis dijalankan.

**3. Tombol cepat** — `Datar (XY)` mengembalikan rotasi ke identitas;
`Ke pusat data` memindahkan grid ke pusat bounds gabungan layer terlihat tanpa
mengubah rotasi.

### Perpotongan sinar dengan bidang grid

Dipakai `measure.js` untuk snap. Analitis, bukan raycast ke mesh — lebih tepat dan
tidak bergantung pada visibilitas objek:

```js
export function titikDiBidang(raycaster) {
  const normal = new THREE.Vector3(0, 0, 1).applyQuaternion(grid.quaternion);
  _bidang.setFromNormalAndCoplanarPoint(normal, grid.position);
  const kena = raycaster.ray.intersectPlane(_bidang, new THREE.Vector3());
  if (!kena) return null;                       // sinar sejajar bidang
  const lokal = grid.worldToLocal(kena.clone());
  const s = ukuranEfektif / 2;
  if (Math.abs(lokal.x) > s || Math.abs(lokal.y) > s) return null;  // di luar kotak
  return kena;
}
```

---

## `measure.js` — alat ukur

### State

```js
let aktif = false;
let snap = 'titik';     // 'titik' | 'grid'
let tipe = 'jarak';     // 'jarak' | 'sudut'
let sedang = [];        // titik yang sudah diklik pada pengukuran berjalan
const hasil = [];       // { id, tipe, titik:[Vector3], nilai, objek:THREE.Group }
```

### Menentukan titik di bawah kursor

| Snap | Cara |
|---|---|
| `titik` | raycast ke `layers.objekTerlihat()`; ambil `intersects[0]`. Ambang `raycaster.params.Points.threshold` = `max(0,02, diagonal × 0,004)` — nilai tetap 0,05 sekarang terlalu besar untuk ruangan kecil dan terlalu kecil untuk gedung. |
| `grid` | `grid.titikDiBidang(raycaster)`. Titik cloud diabaikan total. |

Bila snap `grid` dipilih sementara grid mati, grid otomatis dinyalakan (bukan ditolak
dengan pesan error — itu yang jelas dimaui pemakainya).

### Klik, bukan `click`

Penangan sekarang memakai event `click`, yang juga menyala setelah memutar pandangan —
orbit lalu lepas tombol akan menancapkan titik ukur yang tidak diminta. Diganti pasangan
`pointerdown`/`pointerup`: posisi pointerdown disimpan, dan pengukuran hanya
ditancapkan bila pointerup terjadi dalam **4 piksel** dari situ. Ambang yang sama sudah
dipakai kotak seleksi.

Klik juga diabaikan bila `tc.dragging` atau `tc.axis !== null` — supaya menyeret gizmo
grid tidak sekalian menancapkan titik ukur.

### Karet (rubber-band)

Saat `sedang.length > 0`, tiap `pointermove` menghitung ulang titik di bawah kursor dan
memperbarui garis pratinjau serta labelnya:

| Tipe | Setelah 1 klik | Setelah 2 klik |
|---|---|---|
| `jarak` | garis `p0 → kursor`, label jarak hidup | — (dikunci di klik ke-2) |
| `sudut` | garis `p0 → kursor` | garis `p0 → p1` dan `p1 → kursor`, label sudut hidup di `p1` |

Garis pratinjau putih redup dan memakai `LineDashedMaterial` supaya jelas bedanya dari
hasil yang sudah terkunci. Bila kursor tidak mengenai apa pun, garis pratinjau
disembunyikan sampai kursor kembali mengenai sesuatu.

`Esc` membatalkan pengukuran berjalan dan mengosongkan `sedang`.

Klik terakhir (ke-2 untuk jarak, ke-3 untuk sudut) mengunci hasil, memasukkannya ke
`hasil`, lalu mengosongkan `sedang` — siap mengukur lagi tanpa klik tambahan.

### Hasil yang menetap

Tiap hasil adalah `THREE.Group` berisi:

- **garis** — `LineBasicMaterial`, kuning `0xffcc44` untuk jarak, biru muda `0x4cd6ff`
  untuk sudut
- **penanda ujung** — bola kecil di tiap titik, jari-jari `diagonal × 0,004`, supaya
  terlihat titik mana persisnya yang diukur
- **label** — sprite bertekstur canvas

Label memakai `sizeAttenuation: false` dan `depthTest: false`: ukurannya tetap di layar
berapa pun jaraknya, dan tidak tertutup titik. Skala sprite dihitung dari tinggi
viewport supaya tingginya ≈ 22 px, dan dihitung ulang saat jendela diubah ukurannya.
Label jarak diletakkan di tengah garis; label sudut di simpul tengah.

Untuk sudut, digambar juga busur kecil di simpul (`ArcCurve` pada bidang yang dibentuk
kedua lengan) supaya terbaca sudut mana yang dimaksud.

Hasil tidak menerima `clippingPlanes` — mengiris ketinggian Z tidak boleh memotong
pengukuran yang sudah dibuat.

### Nilai yang dihitung

- **jarak** = `a.distanceTo(b)`, ditampilkan 3 desimal dalam meter (`2,340 m`)
- **sudut** = sudut di titik tengah B antara `A−B` dan `C−B`, ditampilkan 1 desimal
  dalam derajat (`89,4°`). Dihitung lewat `atan2(|u × v|, u · v)` — bukan
  `acos(u·v)` — karena `acos` kehilangan presisi untuk sudut dekat 0° dan 180°.

Angka di layar dan di sidebar memakai koma desimal (`toLocaleString('id')`), mengikuti
sisa aplikasi. CSV memakai titik (lihat di bawah).

---

## Panel sidebar

Dua panel baru. Panel Grid ditaruh di bawah "Irisan Ketinggian (Z)"; panel Ukuran
ditaruh tepat di bawahnya.

```
┌─ Grid ───────────────────────────┐
│ [▦ Aktifkan grid]                │
│ Gizmo:  [Geser][Putar][Mati]     │
│ Snap gizmo:            [x]       │
│ Spasi:  [0,1 m ▾]                │
│ Ukuran: [10.0] m                 │
│ Posisi  X[ 0,000] Y[ 0,000]      │
│         Z[ 0,000]                │
│ Putar Z [   0,0]°                │
│ Miring  X[  0,0]° Y[  0,0]°      │
│ [Pasang di 3 titik]              │
│ [Bidang RANSAC          ▾]       │
│ [Datar (XY)] [Ke pusat data]     │
└──────────────────────────────────┘

┌─ Ukuran ─────────────────────────┐
│ Snap: [Titik][Grid]              │
│ Tipe: [Jarak][Sudut]             │
│ ──────────────────────────────── │
│ #1  jarak   2,340 m           ✕  │
│ #2  sudut   89,4°             ✕  │
│ #3  jarak   0,872 m           ✕  │
│ ──────────────────────────────── │
│ [Hapus semua]  [💾 CSV]          │
└──────────────────────────────────┘
```

Isi panel Grid diredupkan (`opacity` turun, `disabled`) saat grid mati, kecuali tombol
"Aktifkan grid" sendiri.

Menggantung kursor di baris hasil menyorot pengukuran itu di viewport (garisnya menebal
sesaat) — supaya tahu baris mana yang mana sebelum menekan ✕.

Header dapat satu tombol baru di sebelah "📏 Ukur": **"▦ Grid"**, kembar dari
"Aktifkan grid" di panel.

### Ekspor CSV

Tombol 💾 CSV mengunduh `ukuran.csv`:

```
no,tipe,nilai,satuan,x1,y1,z1,x2,y2,z2,x3,y3,z3
1,jarak,2.3401,m,0.1000,0.2000,0.0000,2.2000,1.0500,0.6200,,,
2,sudut,89.42,deg,1.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,1.0000,0.0000
```

Angka memakai **titik desimal**, bukan koma — berkasnya ditujukan untuk pandas/Excel,
dan koma desimal bertabrakan dengan koma pemisah kolom. Baris jarak mengosongkan tiga
kolom terakhir. Tombol menampilkan toast bila belum ada hasil.

---

## Interaksi antar-mode

Tiga mode klik saling meniadakan: **Pilih area**, **Ukur**, dan **Pasang grid 3 titik**.
Menyalakan salah satu mematikan dua lainnya. Grid sendiri (tampil/tidak) berdiri
sendiri — grid boleh menyala di mode apa pun.

| Keadaan | `OrbitControls` | Gizmo grid |
|---|---|---|
| bebas | aktif | aktif bila mode gizmo ≠ Mati |
| mode Pilih area | mati (sudah sejak dulu) | dimatikan sementara |
| mode Ukur | aktif | **ditahan** (lihat catatan) |
| menyeret gizmo | mati (`dragging-changed`) | menyeret |
| mode Pasang 3 titik | aktif | dimatikan sementara |

> **Catatan — diubah saat implementasi.** Rancangan awal membiarkan gizmo tetap
> aktif selama mode Ukur. Dicoba di browser, itu tidak bisa dipakai: lengan
> `TransformControls` berukuran tetap **di layar**, jadi ia menutupi petak dunia
> yang besar tepat di pusat grid — persis tempat orang ingin mengukur — dan tiap
> klik di situ menggenggam gizmo alih-alih menancapkan titik ukur. Menggeser grid
> dan mengukur juga tidak mungkin dilakukan bersamaan dengan satu tetikus.
> Sekarang gizmo disembunyikan selama mode Ukur (`grid.setGizmoDitahan`), dengan
> catatan di panel yang menjelaskan cara mengembalikannya.
>
> Penjaga `grid.sedangDiseret()` juga dipersempit dari `dragging || axis !== null`
> jadi `dragging` saja: `axis` sudah terisi begitu kursor sekadar *melintas* di
> atas gizmo, sehingga klik di sekitarnya tertelan diam-diam.

**Kamera ortho.** `toggleOrtho` membuang dan membuat ulang `OrbitControls`. Ia juga
harus menyetel ulang `tc.camera = camera`, kalau tidak gizmo akan memakai kamera lama
dan penempatannya melenceng. Ini tanggung jawab `viewer.js`: `toggleOrtho` melempar
callback `onKameraGanti` yang didengar `grid.js`.

Mengukur dalam mode ortografis tetap dianjurkan — README sudah menyebutnya.

---

## Kasus tepi

| Keadaan | Perilaku |
|---|---|
| Ukur snap `titik` tapi tidak ada layer terlihat | toast `Tidak ada layer yang terlihat` |
| Ukur snap `grid` tapi kursor di luar kotak grid | garis pratinjau disembunyikan; klik diabaikan |
| Grid dimatikan sementara ada hasil ukur di bidangnya | hasil ukur tetap ada; grid hanya acuan visual |
| `spasi` diubah saat snap gizmo nyala | `setTranslationSnap` diperbarui ke spasi baru |
| Layer terakhir ditutup | grid dan hasil ukur **tetap** — keduanya tidak dimiliki layer mana pun |
| Semua layer ditutup lalu berkas baru dibuka | kamera di-frame ulang; grid tidak dipindahkan |
| Pasang 3 titik dengan snap `grid` aktif | pemasangan selalu memakai titik cloud, apa pun snap yang dipilih — memasang grid ke bidangnya sendiri tidak ada gunanya |
| Ekspor CSV tanpa hasil | toast `Belum ada hasil ukur` |
| Ukur sudut ditinggalkan setelah 2 klik lalu mode dimatikan | `sedang` dikosongkan, pratinjau dibuang, tidak ada hasil setengah jadi |

---

## Tes

Tidak ada perubahan Python, jadi pytest yang ada harus tetap hijau apa adanya —
itu sendiri sudah jadi pemeriksaan bahwa Spec 2 tidak menyentuh backend.

Proyek ini belum punya alat tes JavaScript dan spec ini tidak memasangnya (sama seperti
Spec 1). Verifikasi manual di browser:

**Grid**
1. Aktifkan grid → grid mendatar muncul di pusat data
2. Gizmo Geser → seret sumbu Z → grid naik; kotak isian Z ikut berubah
3. Ketik Z = 1,5 → grid pindah ke situ; gizmo ikut
4. Gizmo Putar → putar → kotak Putar Z / Miring ikut berubah, tanpa melayang sendiri
   saat gizmo dilepas
5. Snap gizmo nyala → geser terkunci ke kelipatan spasi, putar ke kelipatan 15°
6. Ubah spasi ke 1 m → garis dibangun ulang, ukuran tetap
7. Ukuran 200 m dengan spasi 0,05 → dipangkas ke 20 m, catatan muncul
8. Pasang di 3 titik pada lantai → grid menempel di lantai
9. Jalankan Analisis → pilih bidang dari daftar → grid menempel
10. Datar (XY) → rotasi kembali nol

**Ukur**
11. Snap Titik, tipe Jarak → klik → garis mengikuti kursor dengan angka hidup → klik
    kedua mengunci → hasil masuk daftar
12. Ukur lagi → hasil pertama **tetap** ada
13. Esc di tengah pengukuran → batal bersih
14. Orbit pandangan (drag lalu lepas) di mode Ukur → **tidak** menancapkan titik
15. Snap Grid → klik mendarat di bidang grid, bukan di titik cloud
16. Tipe Sudut → 3 klik → busur + sudut muncul di simpul tengah
17. Seret gizmo grid saat mode Ukur menyala → tidak menancapkan titik ukur
18. Zoom jauh/dekat → label tetap seukuran di layar dan tetap terbaca
19. Slider irisan Z → grid dan hasil ukur tidak ikut terpotong
20. ✕ di satu baris → hanya itu yang hilang; Hapus semua → bersih
21. 💾 CSV → berkas terunduh, kolom jarak & sudut terisi benar
22. Toggle Ortho saat gizmo tampil → gizmo tetap pas di grid

## Stack

Tidak ada dependensi Python baru. Satu berkas JS baru di-vendor
(`TransformControls.js`). Tidak ada bundler; modul ES dimuat langsung oleh browser.
