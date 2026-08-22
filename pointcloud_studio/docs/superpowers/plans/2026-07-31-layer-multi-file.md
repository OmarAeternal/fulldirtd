# Layer Multi-Berkas + Modularisasi Frontend — Rencana Implementasi

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** PointCloud Studio bisa membuka beberapa berkas point cloud sekaligus sebagai layer yang bisa disembunyikan/ditampilkan, dan `frontend/app.js` dipecah jadi modul-modul berfokus.

**Architecture:** `cloud` (satu `Float32Array` global) diganti daftar objek layer di `layers.js`, masing-masing memegang cloud, objek `THREE.Points`, dan tumpukan undo sendiri. `app.js` dipecah jadi tujuh modul searah — `hud.js` (daun) → `viewer.js` → `layers.js` → `edit.js`/`measure.js`/`io.js` → `ui.js` → `app.js`. Impor balik dihindari lewat langganan `layers.onUbah()`. Backend tetap stateless; `/load` disamakan dengan `/open` (ikut di-downsample) dan `pcs` menerima banyak berkas.

**Tech Stack:** Python 3.12 · FastAPI · Open3D · numpy · pytest · Three.js r160 (vendored) · modul ES tanpa bundler.

**Spec:** `docs/superpowers/specs/2026-07-31-layer-multi-file-design.md`

## Global Constraints

- Bahasa kode, komentar, dan teks UI: **Indonesia**, mengikuti kodebase yang ada.
- Tidak ada dependensi baru — Python maupun JavaScript.
- Tidak ada bundler. Modul ES dimuat langsung browser lewat importmap di `index.html`.
- Perintah tes wajib memakai dua penyesuaian lingkungan ROS:
  `env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q`
- Batas undo: **8** entri per layer (turun dari 12 global).
- Format biner titik tidak berubah: Float32 little-endian `[x,y,z,r,g,b] * N`.
- Backend tetap **stateless** — frontend adalah sumber kebenaran titik.
- Commit setelah tiap task. Identitas git: `git -c user.name="Omar Ibrahim Attallah" -c user.email="omaribra.attallah@gmail.com"`.

## Struktur Berkas

| Berkas | Tanggung jawab |
|---|---|
| `backend/server.py` (ubah) | `_respons_titik()` dipakai bersama `/load` & `/open` |
| `tests/test_load_endpoint.py` (baru) | perilaku downsample `/load` |
| `pcs.py` (ubah) | `nargs="*"`, `siapkan_berkas` → daftar, `bangun_url` → `file` berulang |
| `tests/test_pcs.py` (ubah) | banyak berkas, validasi sebelum konversi |
| `frontend/hud.js` (baru) | `toast`, `setHint` — daun tanpa dependensi |
| `frontend/viewer.js` (baru) | renderer, scene, kamera, kontrol, gizmo XYZ, material, bidang irisan |
| `frontend/layers.js` (baru) | daftar layer, CRUD, bounds & buffer gabungan, langganan `onUbah` |
| `frontend/edit.js` (baru) | box-select hapus/crop, undo — menyasar layer aktif |
| `frontend/measure.js` (baru) | ukur jarak 2 titik — raycast ke semua layer terlihat |
| `frontend/io.js` (baru) | `/load` `/open` `/mesh` `/analyze`, ekspor, pemuatan berurutan |
| `frontend/ui.js` (baru) | panel Layer/Statistik/Analisis, irisan Z, wiring toolbar |
| `frontend/app.js` (tulis ulang) | bootstrap: seret-lepas, baca `?file=`, `ui.init()` |
| `frontend/index.html` (ubah) | panel Layer, `multiple` pada input berkas |

---

### Task 1: `/load` ikut di-downsample

**Files:**
- Modify: `backend/server.py:51-107`
- Test: `tests/test_load_endpoint.py` (baru)

**Interfaces:**
- Produces: `server._respons_titik(nama: str, data: bytes, voxel: float, full: bool) -> Response` — parse, validasi kosong, downsample, susun `Response` dengan header `X-Stats` + `X-Downsample`. Dipakai `/load` dan `/open`.

- [ ] **Step 1: Tulis tes yang gagal**

Buat `tests/test_load_endpoint.py`:

```python
"""Tes endpoint POST /load — unggahan berkas.

/load dulu mengirim resolusi penuh sementara /open sudah di-downsample.
Dengan multi-berkas, tiga unggahan besar mencekik browser — jadi keduanya
sekarang lewat jalur yang sama.
"""
import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

import server


@pytest.fixture
def client():
    return TestClient(server.app)


def isi_xyz(n=4000, sisi=1.0):
    """Berkas XYZ ascii berisi n titik acak dalam kubus `sisi` meter."""
    rng = np.random.default_rng(7)
    xyz = rng.random((n, 3)) * sisi
    baris = ["%.4f %.4f %.4f 128 128 128" % tuple(p) for p in xyz]
    return "\n".join(baris).encode()


def unggah(client, isi, nama="awan.xyz", **kueri):
    return client.post("/load", files={"file": (nama, isi, "text/plain")},
                       params=kueri)


def test_mengirim_header_downsample(client):
    r = unggah(client, isi_xyz())
    assert r.status_code == 200
    info = json.loads(r.headers["X-Downsample"])
    assert set(info) == {"voxel", "n_asli", "n_kirim", "melebihi_batas"}
    assert info["n_asli"] == 4000


def test_voxel_besar_mengurangi_titik(client):
    r = unggah(client, isi_xyz(), voxel=0.5)
    info = json.loads(r.headers["X-Downsample"])
    assert info["n_kirim"] < info["n_asli"]
    assert json.loads(r.headers["X-Stats"])["n"] == info["n_kirim"]


def test_full_mengirim_semua_titik(client):
    r = unggah(client, isi_xyz(), full=1)
    info = json.loads(r.headers["X-Downsample"])
    assert info["voxel"] is None
    assert info["n_kirim"] == info["n_asli"] == 4000


def test_berkas_tanpa_titik_ditolak(client):
    r = unggah(client, b"", nama="kosong.xyz")
    assert r.status_code == 400


def test_ekstensi_tak_terbaca_ditolak(client):
    r = unggah(client, b"bukan point cloud", nama="catatan.pdf")
    assert r.status_code == 400
```

- [ ] **Step 2: Jalankan tes, pastikan gagal**

```bash
cd "/home/bromarku/riset td/pointcloud_studio"
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_load_endpoint.py -q
```

Diharapkan: `test_mengirim_header_downsample` GAGAL dengan `KeyError: 'X-Downsample'`.

- [ ] **Step 3: Angkat penolong bersama di `server.py`**

Ganti badan `/load` dan `/open` supaya keduanya memanggil satu penolong. Hapus juga `import json` yang menggantung di dalam fungsi `load` (baris 65) — modul sudah mengimpornya di atas.

```python
def _respons_titik(nama: str, data: bytes, voxel: float, full: bool) -> Response:
    """Bytes berkas → Response titik biner + header statistik.

    Jalur yang sama untuk /load (unggahan) dan /open (path di disk): parse,
    tolak yang kosong, optimasi kerapatan, lalu kirim Float32 murni dengan
    statistik di header supaya body tidak tercampur metadata.
    """
    try:
        pts = loader.parse(nama, data)
    except Exception as e:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=f"Gagal membaca file: {e}")
    if len(pts) == 0:
        raise HTTPException(status_code=400, detail="File tidak memuat titik valid.")

    pts, info = downsample.optimize(pts, voxel=voxel, full=full)
    return Response(
        content=pts.astype("<f4").tobytes(),
        media_type="application/octet-stream",
        headers={"X-Stats": json.dumps(_stats(pts)),
                 "X-Downsample": json.dumps(info),
                 "Access-Control-Expose-Headers": "X-Stats, X-Downsample"})


def _param_optimasi(request: Request) -> tuple:
    """→ (voxel, full) dari query, dengan bawaan yang sama di kedua endpoint."""
    full = request.query_params.get("full", "0") in ("1", "true", "yes")
    return float(request.query_params.get("voxel", 0.01)), full


@app.post("/load")
async def load(request: Request, file: UploadFile = File(...)):
    """Terima file PLY/XYZ → parse → kembalikan titik biner + statistik.

    Respons: body = Float32 little-endian [x,y,z,r,g,b] * N.
    Statistik dikirim di header 'X-Stats' (JSON) agar body tetap biner murni.
    """
    voxel, full = _param_optimasi(request)
    return _respons_titik(file.filename or "upload", await file.read(), voxel, full)


@app.get("/open")
async def open_path(request: Request):
    """Muat berkas dari path di disk → titik biner, format sama dengan /load.

    Dipakai oleh perintah `pcs`, yang membuka browser ke `/?file=<path>`.
    Server hanya mendengar di 127.0.0.1, tapi path tetap divalidasi: yang
    ditolak semuanya dibalas 400 (bukan 404) supaya balasan ini tidak bisa
    dipakai menebak berkas mana yang ada di disk.
    """
    mentah = request.query_params.get("path", "")
    p = pathlib.Path(mentah)
    if (not mentah or not p.is_absolute()
            or p.suffix.lower() not in EKSTENSI_CLOUD or not p.is_file()):
        raise HTTPException(
            status_code=400,
            detail="Path tidak sah: harus absolut, menunjuk berkas yang ada, "
                   f"dan berekstensi {'/'.join(EKSTENSI_CLOUD)}.")

    voxel, full = _param_optimasi(request)
    return _respons_titik(p.name, p.read_bytes(), voxel, full)
```

- [ ] **Step 4: Jalankan seluruh tes**

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q
```

Diharapkan: semua LULUS, termasuk `tests/test_open_endpoint.py` yang tidak diubah.

- [ ] **Step 5: Commit**

```bash
git add backend/server.py tests/test_load_endpoint.py
git commit -m "Samakan /load dengan /open: ikut di-downsample"
```

---

### Task 2: `pcs` menerima banyak berkas

**Files:**
- Modify: `pcs.py:55-69` (`bangun_url`), `pcs.py:107-123` (`siapkan_berkas`), `pcs.py:176-204` (parser & `run`)
- Test: `tests/test_pcs.py`

**Interfaces:**
- Consumes: tidak ada dari Task 1.
- Produces: `pcs.siapkan_berkas(args) -> list[pathlib.Path]` (daftar kosong bila tanpa berkas) · `pcs.bangun_url(port: int, berkas: list, voxel: float, full: bool) -> str` (parameter `file` berulang).

- [ ] **Step 1: Tulis tes yang gagal**

Ganti `test_siapkan_berkas_kosong_mengembalikan_none`, `test_berkas_boleh_kosong`, dan `test_url_tanpa_berkas_polos_tanpa_kueri` di `tests/test_pcs.py`, lalu tambahkan sisanya. Yang lama memakai `None`/satu path; sekarang daftar.

```python
def test_berkas_boleh_kosong():
    a = pcs.build_parser().parse_args([])
    assert a.file == []


def test_banyak_berkas_diurai():
    a = pcs.build_parser().parse_args(["a.ply", "b.ply", "c.xyz"])
    assert a.file == ["a.ply", "b.ply", "c.xyz"]


def test_url_tanpa_berkas_polos_tanpa_kueri():
    assert pcs.bangun_url(8000, [], 0.01, False) == "http://127.0.0.1:8000/"


def test_url_tanpa_berkas_mengabaikan_voxel_dan_full():
    assert pcs.bangun_url(8123, [], 0.005, True) == "http://127.0.0.1:8123/"


def test_url_satu_berkas_tetap_bentuk_lama():
    url = pcs.bangun_url(8123, ["/data/a.ply"], 0.005, False)
    assert url.startswith("http://127.0.0.1:8123/?")
    assert url.count("file=") == 1
    assert "voxel=0.005" in url


def test_url_banyak_berkas_mengulang_parameter_file():
    url = pcs.bangun_url(8000, ["/data/a.ply", "/data/b.ply"], 0.01, False)
    assert url.count("file=") == 2
    # urutan terjaga: a sebelum b
    assert url.index("a.ply") < url.index("b.ply")
    # voxel hanya sekali, berlaku untuk semua berkas
    assert url.count("voxel=") == 1


def test_url_mengkodekan_spasi_pada_semua_berkas():
    url = pcs.bangun_url(8000, ["/home/riset td/a.ply", "/home/riset td/b.ply"],
                         0.01, False)
    assert url.count("riset%20td") == 2
    assert " " not in url


def test_siapkan_berkas_kosong_mengembalikan_daftar_kosong():
    a = pcs.build_parser().parse_args([])
    assert pcs.siapkan_berkas(a) == []


def test_siapkan_berkas_banyak_menjaga_urutan(tmp_path):
    nama = ["b.ply", "a.ply", "c.xyz"]
    for n in nama:
        (tmp_path / n).write_text("")
    a = pcs.build_parser().parse_args([str(tmp_path / n) for n in nama])
    assert [p.name for p in pcs.siapkan_berkas(a)] == nama


def test_siapkan_berkas_hasilnya_absolut(tmp_path, monkeypatch):
    (tmp_path / "a.ply").write_text("")
    monkeypatch.chdir(tmp_path)
    a = pcs.build_parser().parse_args(["a.ply"])
    assert pcs.siapkan_berkas(a)[0].is_absolute()


def test_berkas_hilang_gagal_sebelum_konversi(tmp_path, monkeypatch):
    """Salah ketik di berkas ke-2 tidak boleh membuang waktu mengonversi ke-1."""
    (tmp_path / "a.mcap").write_text("")
    dipanggil = []
    monkeypatch.setattr(pcs, "konversi_mcap",
                        lambda *a, **k: dipanggil.append(a))
    a = pcs.build_parser().parse_args(
        [str(tmp_path / "a.mcap"), str(tmp_path / "tidak_ada.ply")])
    with pytest.raises(SystemExit):
        pcs.siapkan_berkas(a)
    assert dipanggil == []


def test_ekstensi_asing_gagal_sebelum_konversi(tmp_path, monkeypatch):
    (tmp_path / "a.mcap").write_text("")
    (tmp_path / "catatan.pdf").write_text("")
    dipanggil = []
    monkeypatch.setattr(pcs, "konversi_mcap",
                        lambda *a, **k: dipanggil.append(a))
    a = pcs.build_parser().parse_args(
        [str(tmp_path / "a.mcap"), str(tmp_path / "catatan.pdf")])
    with pytest.raises(SystemExit):
        pcs.siapkan_berkas(a)
    assert dipanggil == []
```

- [ ] **Step 2: Jalankan tes, pastikan gagal**

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_pcs.py -q
```

Diharapkan: `test_berkas_boleh_kosong` GAGAL (`a.file` masih `None`).

- [ ] **Step 3: Ubah `pcs.py`**

`bangun_url` — `doseq=True` supaya `file` boleh berulang:

```python
def bangun_url(port: int, berkas, voxel: float, full: bool) -> str:
    """URL halaman yang langsung memuat semua berkas di `berkas`.

    Tiap berkas jadi satu parameter `file`; `voxel`/`full` disebut sekali dan
    berlaku untuk semuanya. Path dikodekan dengan quote, bukan quote_plus:
    path di sini mengandung spasi ("riset td"), dan '+' akan terbaca sebagai
    spasi oleh sebagian pembaca query sementara '%20' selalu benar.
    """
    if not berkas:
        return f"http://127.0.0.1:{port}/"

    q = [("file", str(b)) for b in berkas]
    q.append(("voxel", voxel))
    if full:
        q.append(("full", "1"))
    kueri = urllib.parse.urlencode(q, quote_via=urllib.parse.quote)
    return f"http://127.0.0.1:{port}/?{kueri}"
```

`siapkan_berkas` — validasi semuanya lebih dulu, konversi belakangan:

```python
def siapkan_berkas(args):
    """Argumen berkas → daftar path PLY/XYZ absolut yang siap dibuka.

    → [] bila tidak ada berkas yang diminta; aplikasinya dibuka kosong dan
    pemakainya memilih sendiri lewat tombol "Buka" atau seret-lepas.

    Semua berkas divalidasi sebelum konversi MCAP mana pun dijalankan: salah
    ketik di berkas ketiga tidak boleh membuang menit-menit mengonversi yang
    pertama.
    """
    sumber = []
    for mentah in args.file:
        src = pathlib.Path(mentah).expanduser()
        if not src.is_file():
            raise SystemExit(f"[ERROR] File tidak ditemukan: {src}")
        src = src.resolve()
        sumber.append((src, jenis_berkas(src)))   # jenis_berkas menolak yang asing

    return [src if jenis == "cloud" else konversi_mcap(src, args.topic, args.force)
            for src, jenis in sumber]
```

Parser dan `run`:

```python
    ap.add_argument("file", nargs="*", default=[],
                    help="berkas .ply, .xyz, .mcap, atau .mcap.zstd; "
                         "boleh lebih dari satu (tiap berkas jadi satu layer), "
                         "boleh juga dikosongkan untuk membuka aplikasinya saja")
```

```python
def run(args) -> None:
    berkas = siapkan_berkas(args)
    pastikan_server(args.port)

    url = bangun_url(args.port, berkas, args.voxel, args.full)
    for b in berkas:
        print(f"  {b}")
    if not berkas:
        print(f"  {url}")
    if not webbrowser.open(url):
        print(f"  Browser tidak bisa dibuka otomatis. Buka sendiri:\n  {url}")
```

Perbarui juga docstring modul di atas `pcs.py` supaya menyebut banyak berkas:

```python
"""pcs — buka point cloud di PointCloud Studio dengan satu perintah.

    pcs merged.ply
    pcs sweep_1.ply sweep_2.ply      # tiap berkas jadi satu layer
    pcs scan_0007_1sweep_0.mcap
...
```

- [ ] **Step 4: Jalankan seluruh tes**

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q
```

Diharapkan: semua LULUS.

- [ ] **Step 5: Commit**

```bash
git add pcs.py tests/test_pcs.py
git commit -m "pcs menerima banyak berkas sekaligus"
```

---

### Task 3: `hud.js` + `viewer.js` — pecahan pertama, tanpa perubahan perilaku

**Files:**
- Create: `frontend/hud.js`, `frontend/viewer.js`
- Modify: `frontend/app.js` (impor dari kedua modul baru, buang kode yang pindah)

**Interfaces:**
- Produces:
  - `hud.js`: `toast(msg, isErr=false)` · `setHint(t)`
  - `viewer.js`: `renderer` · `scene` · `vp` · `UP` · `clipLo` · `clipHi` · `kamera()` → kamera aktif · `kontrol()` → OrbitControls aktif · `matPoints` · `matDense` · `matMesh` · `resize()` · `frameCamera(bounds)` · `updateOrthoFrustum(bounds)` · `setView(axis, bounds)` · `toggleOrtho()` · `onKameraGanti(fn)` (dipakai Spec 2)

Kamera dan kontrol diekspor lewat **fungsi**, bukan `let` yang diekspor langsung: `toggleOrtho` mengganti keduanya, dan konsumen harus selalu melihat yang terbaru.

- [ ] **Step 1: Buat `frontend/hud.js`**

```js
// Pesan sekilas (toast) dan baris keterangan di kiri bawah viewport.
// Modul daun: tidak mengimpor apa pun, jadi siapa pun boleh mengimpornya
// tanpa membuat lingkaran.

let toastT;

export function toast(msg, isErr = false) {
  const el = document.getElementById('toast');
  el.textContent = msg; el.className = isErr ? 'err' : '';
  el.style.display = 'block';
  clearTimeout(toastT);
  toastT = setTimeout(() => el.style.display = 'none', isErr ? 5000 : 2500);
}

export function setHint(t) { document.getElementById('hint').textContent = t; }
```

- [ ] **Step 2: Buat `frontend/viewer.js`**

Pindahkan dari `app.js` apa adanya: blok "Three.js dasar" (baris 23–79), `resize`, `animate`, `renderGizmo`, blok "Materials", `frameCamera`, `updateOrthoFrustum`, `setView`, `toggleOrtho`. Tiga penyesuaian:

1. `frameCamera`, `updateOrthoFrustum`, dan `setView` menerima `bounds` sebagai argumen — tadinya membaca global.
2. `camera`/`controls` tidak diekspor langsung; disediakan `kamera()`/`kontrol()`.
3. `toggleOrtho` memanggil pendengar `onKameraGanti` setelah kamera ditukar.

```js
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export const vp = document.getElementById('viewport');

export const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.localClippingEnabled = true;
renderer.autoClear = false;               // perlu untuk overlay gizmo
vp.appendChild(renderer.domElement);

export const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);

// Data LiDAR memakai Z sebagai sumbu vertikal → set 'up' = Z agar orbit terasa natural.
export const UP = new THREE.Vector3(0, 0, 1);
const persp = new THREE.PerspectiveCamera(60, 1, 0.01, 5000);
const ortho = new THREE.OrthographicCamera(-1, 1, 1, -1, -5000, 5000);
persp.up.copy(UP); ortho.up.copy(UP);
persp.position.set(6, -6, 4);

let camera = persp;
let controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.screenSpacePanning = true;       // geser mengikuti bidang layar (lebih intuitif)

export function kamera() { return camera; }
export function kontrol() { return controls; }

const pendengarKamera = [];
export function onKameraGanti(fn) { pendengarKamera.push(fn); }
```

Lanjutkan dengan lampu, gizmo XYZ, bidang irisan, dan material — semuanya disalin apa adanya dari `app.js`, dengan `export` pada `clipLo`, `clipHi`, `matPoints`, `matDense`, `matMesh`.

`animate` memakai `camera`/`controls` yang hidup. `resize` diekspor. Versi baru yang menerima bounds:

```js
export function frameCamera(bounds) {
  if (!bounds) return;
  const c = pusat(bounds), r = diag(bounds) / 2 || 1;
  camera.up.copy(UP);
  controls.target.set(c[0], c[1], c[2]);
  camera.position.set(c[0] + r * 1.5, c[1] - r * 1.5, c[2] + r * 1.1);
  updateOrthoFrustum(bounds);
  controls.update();
}

export function toggleOrtho(bounds) {
  const btn = document.getElementById('vOrtho');
  const useOrtho = camera === persp;
  const oldPos = camera.position.clone(), oldTarget = controls.target.clone();
  camera = useOrtho ? ortho : persp;
  camera.position.copy(oldPos);
  updateOrthoFrustum(bounds);
  controls.dispose();
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.target.copy(oldTarget); controls.update();
  btn.classList.toggle('on', useOrtho);
  pendengarKamera.forEach(f => f(camera));   // gizmo grid perlu tahu (Spec 2)
}
```

`pusat(bounds)` dan `diag(bounds)` jadi penolong lokal di `viewer.js`, diekspor juga karena `layers.js` dan `ui.js` memerlukannya.

- [ ] **Step 3: Ramping-kan `app.js`**

Buang semua yang sudah pindah, ganti dengan impor:

```js
import { toast, setHint } from './hud.js';
import { renderer, scene, vp, kamera, kontrol, clipLo, clipHi,
         matPoints, matDense, matMesh, resize, frameCamera,
         updateOrthoFrustum, setView, toggleOrtho } from './viewer.js';
```

Sisanya tetap di `app.js` untuk sementara — dipecah di task berikutnya. Ganti pemakaian `camera` jadi `kamera()` dan `controls` jadi `kontrol()`.

- [ ] **Step 4: Verifikasi di browser**

```bash
env -u PYTHONPATH .venv/bin/python -m uvicorn backend.server:app --port 8000
```

Buka `http://127.0.0.1:8000/`, muat sebuah `.ply`. Periksa: titik tampil · orbit/zoom jalan · gizmo XYZ di pojok kiri bawah berputar · Atas/Depan/Samping/Ortho jalan · slider irisan Z jalan · box-select hapus jalan · ukur jalan. Konsol browser **tanpa error**.

- [ ] **Step 5: Commit**

```bash
git add frontend/hud.js frontend/viewer.js frontend/app.js
git commit -m "Pecah hud.js dan viewer.js dari app.js"
```

---

### Task 4: `layers.js` — model layer menggantikan `cloud` global

**Files:**
- Create: `frontend/layers.js`
- Modify: `frontend/app.js`

**Interfaces:**
- Consumes: `viewer.js` (`scene`, `matPoints`, `matDense`, `matMesh`, `pusat`, `diag`)
- Produces:
  - `tambah({nama, cloud, ket}) -> id` — bangun geometry, masukkan ke scene, jadikan aktif & terlihat
  - `tutup(id)` · `setAktif(id)` (juga membuatnya terlihat) · `setTerlihat(id, on)`
  - `daftar()` → array layer (baca saja) · `aktif()` → layer|null · `terlihat()` → array layer terlihat
  - `objekTerlihat()` → array `THREE.Points` — untuk raycast
  - `gantiCloud(id, cloudBaru, {simpanUndo=true})` — satu-satunya jalan mengubah titik; bangun ulang geometry + bounds, lalu `beritahu()`
  - `undoLayer(id)` → bool
  - `boundsGabungan()` → `{min,max}`|null · `xyzBufferGabungan()` → `Float32Array` `[x,y,z]*N`
  - `jumlahTitikGabungan()` → number
  - `setModeTampilan(m)` · `onUbah(fn)`

- [ ] **Step 1: Tulis `frontend/layers.js`**

```js
import * as THREE from 'three';
import { scene, matPoints, matDense } from './viewer.js';

// Tiap layer memegang cloud-nya sendiri, objek scene-nya sendiri, dan
// tumpukan undo-nya sendiri. Tumpukan global akan melompat antar berkas dan
// sulit ditebak — undo hanya berlaku pada layer aktif.
const MAKS_UNDO = 8;

const _layers = [];
let _aktifId = null;
let _urutan = 0;
let _mode = 'points';          // points | dense | mesh

const _pendengar = [];
export function onUbah(fn) { _pendengar.push(fn); }
function beritahu() { _pendengar.forEach(f => f()); }
```

Lalu `bangunGeometry(cloud)` (pindahan `rebuildPoints`), `hitungBounds(cloud)` (pindahan `computeBounds`), dan fungsi-fungsi di daftar Interfaces. Yang penting:

```js
export function gantiCloud(id, cloudBaru, { simpanUndo = true } = {}) {
  const L = _cari(id); if (!L) return;
  if (simpanUndo) {
    L.undo.push(L.cloud);
    if (L.undo.length > MAKS_UNDO) L.undo.shift();
  }
  L.cloud = cloudBaru;
  L.bounds = hitungBounds(cloudBaru);
  const geo = bangunGeometry(cloudBaru);
  L.points.geometry.dispose();
  L.points.geometry = geo;
  beritahu();
}

export function tutup(id) {
  const i = _layers.findIndex(L => L.id === id); if (i < 0) return;
  const L = _layers[i];
  scene.remove(L.points);
  L.points.geometry.dispose();
  _layers.splice(i, 1);
  if (_aktifId === id) {
    // pindah ke tetangga di bawah; kalau itu baris terakhir, ke atasnya
    const gantinya = _layers[i] || _layers[i - 1] || null;
    _aktifId = gantinya ? gantinya.id : null;
    if (gantinya) gantinya.terlihat = true;
  }
  terapkanTampilan();
  beritahu();
}

export function setAktif(id) {
  const L = _cari(id); if (!L) return;
  _aktifId = id;
  L.terlihat = true;          // menjadikan aktif juga menampilkannya
  terapkanTampilan();
  beritahu();
}
```

`terapkanTampilan()` menyetel `L.points.visible = L.terlihat && _mode !== 'mesh'` dan `L.points.material` sesuai `_mode`.

`boundsGabungan()` menggabungkan bounds semua layer terlihat; `null` bila tidak ada.

- [ ] **Step 2: Sambungkan `app.js` ke `layers.js`**

Buang `cloud`, `points`, `bounds`, `undoStack` global. `terimaTitik` memanggil `layers.tambah(...)`. `applySelection`/`undo`/`exportPLY`/`exportXYZ` menyasar `layers.aktif()`. `refreshMesh`/`analyze` memakai `layers.xyzBufferGabungan()`. `updateStats` memakai `layers.boundsGabungan()` dan `layers.jumlahTitikGabungan()`.

Kamera hanya di-frame bila daftar sebelumnya kosong:

```js
const pertama = layers.daftar().length === 0;
const id = layers.tambah({ nama: label, cloud: baru, ket });
if (pertama) frameCamera(layers.boundsGabungan());
```

- [ ] **Step 3: Verifikasi di browser**

Muat satu berkas. Periksa perilaku **persis seperti sebelumnya**: statistik benar · hapus area jalan · undo jalan (maks 8) · mesh jalan · analisis jalan · ekspor jalan · irisan Z jalan. Konsol tanpa error.

- [ ] **Step 4: Commit**

```bash
git add frontend/layers.js frontend/app.js
git commit -m "Ganti cloud global dengan model layer"
```

---

### Task 5: `edit.js`, `measure.js`, `io.js` — pecahan kedua

**Files:**
- Create: `frontend/edit.js`, `frontend/measure.js`, `frontend/io.js`
- Modify: `frontend/app.js`

**Interfaces:**
- Consumes: `layers.js`, `viewer.js`, `hud.js`
- Produces:
  - `edit.js`: `setSel(on)` · `selMode()` · `setSelAction(a)` · `undo()` · `pasangPenangan()`
  - `measure.js`: `setMeasure(on)` · `measureMode()` · `pasangPenangan()`
  - `io.js`: `muatBerkas(file)` · `muatDariPath(path, voxel, full)` · `muatBanyak(daftar)` · `refreshMesh()` · `analisis()` · `eksporPLY()` · `eksporXYZ()` · `onAnalisis(fn)`

- [ ] **Step 1: `edit.js`**

Pindahkan `applySelection`, `afterEdit`, `undo`, dan penangan pointer kotak seleksi. Perubahan penting — `afterEdit` tidak lagi memanggil `updateStats`/`refreshMesh` langsung; semuanya lewat `layers.gantiCloud`, dan pendengar mengurus sisanya:

```js
function terapkanSeleksi(x0, y0, x1, y1) {
  const L = layers.aktif();
  if (!L) { toast('Belum ada data', true); return; }
  if (!L.terlihat) { toast('Layer aktif sedang disembunyikan', true); return; }

  const cam = kamera(), w = vp.clientWidth, h = vp.clientHeight;
  const n = L.cloud.length / 6, v = new THREE.Vector3();
  const zlo = -clipLo.constant, zhi = clipHi.constant;
  const inside = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const z = L.cloud[i * 6 + 2];
    if (z < zlo || z > zhi) { inside[i] = 0; continue; }  // hormati irisan
    v.set(L.cloud[i * 6], L.cloud[i * 6 + 1], z).project(cam);
    const px = (v.x * 0.5 + 0.5) * w, py = (-v.y * 0.5 + 0.5) * h;
    inside[i] = (px >= x0 && px <= x1 && py >= y0 && py <= y1 && v.z < 1) ? 1 : 0;
  }
  const keepInside = _selAction === 'crop';
  let keptCount = 0;
  for (let i = 0; i < n; i++) if ((inside[i] === 1) === keepInside) keptCount++;
  if (keptCount === 0) { toast('Semua titik akan terhapus — dibatalkan', true); return; }
  if (keptCount === n) { toast('Tidak ada titik di kotak'); return; }

  const out = new Float32Array(keptCount * 6); let j = 0;
  for (let i = 0; i < n; i++) {
    if ((inside[i] === 1) === keepInside) {
      for (let k = 0; k < 6; k++) out[j * 6 + k] = L.cloud[i * 6 + k];
      j++;
    }
  }
  layers.gantiCloud(L.id, out);
  toast(`${(n - keptCount).toLocaleString('id')} titik dihapus dari ${L.nama}`);
}
```

`undo()` memanggil `layers.undoLayer(layers.aktif()?.id)`.

- [ ] **Step 2: `measure.js`**

Pindahkan kode ukur apa adanya, dengan satu perubahan: raycast ke `layers.objekTerlihat()` alih-alih satu objek `points`.

```js
const hit = raycaster.intersectObjects(layers.objekTerlihat(), false)[0];
if (!hit) { toast('Tak ada titik di sana — coba zoom'); return; }
```

(Modul ini ditulis ulang besar-besaran di Spec 2 — di sini cukup dipindahkan supaya `app.js` bersih dan Spec 2 punya tempat.)

- [ ] **Step 3: `io.js`**

Pindahkan `terimaTitik`, `loadFile`, `loadFromPath`, `refreshMesh`, `heightColors`, `analyze`, `download`, `exportPLY`, `exportXYZ`. Tambahkan pemuatan berurutan:

```js
// Berurutan, bukan Promise.all: progres kelihatan di baris hint, dan satu
// berkas gagal tidak membatalkan sisanya.
export async function muatBanyak(daftar, muatSatu) {
  let gagal = 0;
  for (let i = 0; i < daftar.length; i++) {
    const nama = namaDari(daftar[i]);
    setHint(`Memuat ${i + 1}/${daftar.length}: ${nama}…`);
    try { await muatSatu(daftar[i]); }
    catch (e) { gagal++; toast(`Gagal memuat ${nama}: ${e.message}`, true); }
  }
  if (gagal && daftar.length > 1) {
    toast(`${gagal} dari ${daftar.length} berkas gagal dimuat`, true);
  }
}
```

`refreshMesh` dan `analisis` menolak bila `layers.terlihat().length === 0` dengan toast `Tidak ada layer yang terlihat`. Ekspor memakai nama layer aktif:

```js
const dasar = L.nama.replace(/\.[^.]+$/, '');
download(`${dasar}_edited.ply`, blob);
```

`analisis()` menyimpan `d.planes` dan memanggil pendengar `onAnalisis` — Spec 2 memakainya untuk mengisi daftar bidang RANSAC.

- [ ] **Step 4: Verifikasi di browser**

Ulangi pemeriksaan Task 4. Tambahan: ekspor menghasilkan berkas bernama `<nama_layer>_edited.ply`.

- [ ] **Step 5: Commit**

```bash
git add frontend/edit.js frontend/measure.js frontend/io.js frontend/app.js
git commit -m "Pecah edit.js, measure.js, io.js dari app.js"
```

---

### Task 6: `ui.js` + panel Layer + multi-berkas

**Files:**
- Create: `frontend/ui.js`
- Modify: `frontend/app.js`, `frontend/index.html`

**Interfaces:**
- Consumes: seluruh modul sebelumnya
- Produces: `ui.js`: `init()` — memasang seluruh wiring dan langganan

- [ ] **Step 1: Panel Layer di `index.html`**

Tambahkan di `aside`, **sebelum** `<h3>Statistik</h3>`:

```html
    <h3>Layer</h3>
    <div id="layerList"><div class="empty">Belum ada layer.</div></div>
    <button class="btn" id="btnFrame" style="width:100%;margin-top:6px">
      ⤢ Sesuaikan pandangan</button>
```

Tambahkan `multiple` pada input berkas:

```html
    <input type="file" id="file" accept=".ply,.xyz,.txt,.asc" multiple>
```

CSS untuk baris layer:

```css
  .lyr { display:flex; align-items:center; gap:6px; font-size:12px; padding:3px 0;
    border-bottom:1px solid var(--line); }
  .lyr:last-child { border-bottom:none; }
  .lyr .eye { cursor:pointer; width:14px; text-align:center; user-select:none; }
  .lyr .nm { flex:1; cursor:pointer; overflow:hidden; text-overflow:ellipsis;
    white-space:nowrap; }
  .lyr.aktif .nm { font-weight:700; color:#fff; }
  .lyr .cnt { color:var(--dim); font-variant-numeric:tabular-nums; font-size:11px; }
  .lyr .x { cursor:pointer; color:var(--dim); padding:0 2px; }
  .lyr .x:hover { color:var(--danger); }
  .lyr.mati .nm, .lyr.mati .cnt { opacity:.45; }
```

- [ ] **Step 2: `ui.js`**

Pindahkan `updateStats`, `renderAnalysis`, `setupSliceRange`, `applySlice`, `sliceZ`, `setMode`, `setBtn`, dan seluruh wiring `onclick`. Tambahkan penggambar panel Layer:

```js
function renderPanelLayer() {
  const box = document.getElementById('layerList');
  const daftar = layers.daftar();
  if (!daftar.length) {
    box.innerHTML = '<div class="empty">Belum ada layer.</div>';
    return;
  }
  box.innerHTML = '';
  const aktifId = layers.aktif()?.id;
  for (const L of daftar) {
    const el = document.createElement('div');
    el.className = 'lyr' + (L.id === aktifId ? ' aktif' : '')
                         + (L.terlihat ? '' : ' mati');
    el.innerHTML =
      `<span class="eye">${L.terlihat ? '◉' : '○'}</span>` +
      `<span class="nm" title="${L.nama}">${L.nama}</span>` +
      `<span class="cnt">${(L.cloud.length / 6).toLocaleString('id')}</span>` +
      `<span class="x" title="Tutup layer">✕</span>`;
    el.querySelector('.eye').onclick = () => layers.setTerlihat(L.id, !L.terlihat);
    el.querySelector('.nm').onclick = () => layers.setAktif(L.id);
    el.querySelector('.x').onclick = () => layers.tutup(L.id);
    box.appendChild(el);
  }
}
```

`L.nama` disisipkan lewat `innerHTML`, jadi harus di-escape — nama berkas boleh mengandung `<`:

```js
const aman = s => s.replace(/[&<>"]/g, c =>
  ({ '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;' }[c]));
```

Pakai `aman(L.nama)` di kedua tempat.

Satu langganan mengurus semua penyegaran:

```js
layers.onUbah(() => {
  renderPanelLayer();
  updateStats();
  perbaruiIrisan();          // rentang Z gabungan berubah saat daftar berubah
  document.getElementById('drop').classList.toggle('hide', layers.daftar().length > 0);
  const L = layers.aktif();
  setHint(L ? `${L.nama} · ${(L.cloud.length / 6).toLocaleString('id')} titik · aktif` : '');
});
```

`perbaruiIrisan()` menjaga posisi slider (0–1000) dan menghitung ulang nilai meternya terhadap `layers.boundsGabungan()`.

Tombol `#btnFrame` memanggil `frameCamera(layers.boundsGabungan())`.

- [ ] **Step 3: `app.js` jadi bootstrap saja**

```js
import * as ui from './ui.js';
import * as io from './io.js';
import { resize } from './viewer.js';
import { setHint } from './hud.js';

ui.init();
resize();

// Seret-lepas: seluruh berkas yang dijatuhkan, bukan hanya yang pertama.
const drop = document.getElementById('drop');
['dragenter', 'dragover'].forEach(ev => window.addEventListener(ev, (e) => {
  e.preventDefault(); drop.classList.remove('hide'); drop.classList.add('over');
}));
['dragleave', 'drop'].forEach(ev => window.addEventListener(ev, (e) => {
  e.preventDefault(); drop.classList.remove('over');
}));
window.addEventListener('drop', (e) => {
  e.preventDefault();
  const f = [...e.dataTransfer.files];
  if (f.length) io.muatBanyak(f, io.muatBerkas);
});

// `pcs` membuka halaman ini dengan ?file=<path>, boleh berulang untuk banyak
// berkas. Parameter halaman sengaja bernama `file`, lalu diterjemahkan jadi
// `path` saat memanggil /open.
const q = new URLSearchParams(location.search);
const berkas = q.getAll('file');
if (berkas.length) {
  const voxel = q.get('voxel') || '0.01', full = q.get('full') === '1';
  io.muatBanyak(berkas, (p) => io.muatDariPath(p, voxel, full));
} else {
  setHint('Tarik file .ply / .xyz untuk mulai');
}
```

- [ ] **Step 4: Verifikasi di browser**

1. Buka 3 berkas sekaligus lewat 📂 Buka → 3 baris layer, baris ketiga tebal (aktif)
2. Seret 2 berkas → bertambah, tidak menimpa
3. Klik ◉ pada satu layer → hilang dari viewport, statistik berubah
4. Klik nama layer yang mati → jadi aktif **dan** tampil lagi
5. Hapus area → hanya layer aktif berkurang; layer lain utuh
6. Undo → kembali
7. Sembunyikan layer aktif lalu coba hapus area → ditolak dengan toast
8. Tutup layer aktif → aktif pindah ke tetangga
9. Tutup semua → overlay "Tarik file" muncul lagi
10. `pcs a.ply b.ply` → dua layer, kamera hanya di-frame sekali
11. Slider irisan Z bekerja pada bounds gabungan
12. Mesh & Analisis memakai gabungan layer terlihat

- [ ] **Step 5: Jalankan tes Python**

```bash
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/ -q
```

Diharapkan: semua LULUS (tidak tersentuh Task 3–6, tapi pastikan tidak ada regresi).

- [ ] **Step 6: Perbarui dokumentasi**

`README.md`: bagian "Cara pakai" dapat butir layer; tabel `pcs` menyebut banyak berkas; "Struktur" mencantumkan modul frontend baru. `docs/DESIGN.md`: bagian "Fitur frontend" dan "Arsitektur" menyebut layer dan pemecahan modul.

- [ ] **Step 7: Commit**

```bash
git add frontend/ui.js frontend/app.js frontend/index.html README.md docs/DESIGN.md
git commit -m "Panel Layer + buka banyak berkas sekaligus"
```

---

## Self-Review

**Cakupan spec:** model state → Task 4 · pemecahan modul → Task 3/5/6 · cakupan operasi → Task 4/5 · panel Layer → Task 6 · menambah layer (Buka/seret/`pcs`) → Task 2/6 · pemuatan berurutan → Task 5 · kamera hanya di-frame pertama kali → Task 4 · `/load` downsample → Task 1 · `pcs` multi-berkas → Task 2 · kasus tepi (aktif ditutup, aktif disembunyikan, semua disembunyikan, irisan ikut berubah) → Task 4/5/6 · tes → Task 1/2.

**Konsistensi nama:** `gantiCloud`, `setAktif`, `setTerlihat`, `tutup`, `daftar`, `aktif`, `terlihat`, `objekTerlihat`, `boundsGabungan`, `xyzBufferGabungan`, `onUbah` dipakai konsisten di Task 4–6. `kamera()`/`kontrol()` dipakai konsisten sejak Task 3.
