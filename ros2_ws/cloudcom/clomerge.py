#!/usr/bin/env python3
"""clomerge.py — banyak scan dari posisi berbeda → satu point cloud tergabung.

Tiap scan MCAP punya origin sendiri (posisi sensor saat merekam), jadi tidak
bisa langsung ditumpuk. Script ini mencari transformasi rigid tiap scan ke
scan acuan secara otomatis:

    kasar  : FPFH feature + RANSAC (tidak perlu tebakan awal)
    halus  : ICP point-to-plane

Hasil di out/_merge/:
    merged.ply         gabungan, warna viridis menurut Z gabungan
    merged_check.ply   gabungan, tiap scan satu warna — untuk memeriksa hasil
    transforms.txt     matriks 4x4 tiap scan + skor kualitasnya

Pemakaian:
    python clomerge.py scan_0003_1sweep_0.mcap scan_0004_1sweep_0.mcap
    python clomerge.py *.mcap --voxel 0.08 --drop-failed
"""
import argparse
import os
import sys
from pathlib import Path

# agar `import clomcap` dst. menemukan file di folder yang sama
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

import clomcap
import clomcaps
import make_grid as mg
from clomcap import launch_cloudcompare  # noqa: F401  (dipakai lewat global modul)

MERGE_DIRNAME = "_merge"

# Ukuran voxel (m) untuk downsample saat registrasi. Bukan resolusi hasil —
# hasil akhir selalu memakai titik resolusi penuh.
DEFAULT_VOXEL = 0.15

# Jarak (m) untuk menilai hasil, tetap dan tidak ikut berubah bersama voxel.
# Ini penting: kalau penilaian memakai ambang yang melebar mengikuti voxel,
# skornya naik sendiri saat voxel diperbesar dan jadi tidak bisa dibandingkan.
EVAL_DIST = 0.10

# Ambang penilaian kualitas, semuanya diukur pada EVAL_DIST dan HANYA pada
# permukaan tegak (lihat wall_subset).
FITNESS_BAIK = 0.40     # ≥ ini, dan rmse rapat → dianggap berhasil
FITNESS_RAGU = 0.20     # ≥ ini → meragukan, di bawahnya dianggap gagal
RMSE_BAIK = 0.06        # simpangan maksimum (m) untuk dianggap berhasil

# Titik dianggap permukaan tegak bila normalnya mendatar (|nz| di bawah ini).
# Hanya permukaan tegak yang menentukan rotasi terhadap sumbu Z: lantai dan
# plafon sama saja diputar berapa pun, dan jumlahnya jauh lebih banyak — kalau
# ikut dinilai, solusi yang melenceng 90° bisa menang.
WALL_NZ = 0.5
MIN_WALL_POINTS = 200   # di bawah ini, jatuh kembali menilai dengan semua titik

# RANSAC bersifat acak DAN multi-thread, jadi hasilnya berbeda antar-run walau
# seed-nya sama. Karena itu dijalankan beberapa kali lalu diambil yang terbaik.
DEFAULT_TRIES = 4

# Warna pembeda antar scan pada merged_check.ply (ColorBrewer Set1).
SCAN_COLORS = [
    (228, 26, 28), (77, 175, 74), (55, 126, 184), (255, 200, 0),
    (152, 78, 163), (255, 127, 0), (166, 86, 40), (247, 129, 191),
]

BAIK, RAGU, GAGAL = "BAIK", "RAGU", "GAGAL"

# ── Perataan lantai ────────────────────────────────────────────────────────────
# Hasil registrasi mewarisi kemiringan scan acuan: kalau sensor berdiri miring,
# seluruh gabungan ikut miring dan lantai tidak sejajar grid. Karena itu bidang
# lantai dicari, lalu SELURUH cloud diputar agar lantai mendatar — grid sendiri
# tidak pernah diputar, supaya tetap sejajar sumbu.
FLOOR_DIST = 0.05       # toleransi RANSAC bidang (m)
FLOOR_NZ = 0.8          # |nz| minimal agar sebuah bidang dianggap mendatar
FLOOR_MIN_FRAC = 0.15   # bidang harus mencakup minimal sekian bagian titik
FLOOR_TRIES = 5         # berapa bidang dikupas sebelum menyerah


def _o3d():
    """Impor Open3D saat dibutuhkan, dengan pesan yang jelas bila belum ada."""
    try:
        import open3d as o3d
    except ImportError:
        raise SystemExit(
            "[ERROR] Open3D belum terpasang di venv ini.\n"
            "        Pasang dengan:\n"
            "            cd \"$HOME/riset td/ros2_ws/cloudcom\"\n"
            "            .venv/bin/python -m pip install open3d")
    return o3d


# ═══════════════════════════════════════════════════════════════════════════════
# Geometri dasar
# ═══════════════════════════════════════════════════════════════════════════════

def apply_transform(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Terapkan matriks homogen 4x4 ke array titik (N,3)."""
    xyz = np.asarray(xyz, dtype=np.float64)
    T = np.asarray(T, dtype=np.float64)
    if xyz.size == 0:
        return xyz.reshape(0, 3)
    return xyz @ T[:3, :3].T + T[:3, 3]


def find_floor_plane(xyz: np.ndarray):
    """Cari bidang lantai. → (normal menghadap atas, titik pada bidang), atau None.

    Bidang dikupas satu per satu dengan RANSAC. Yang mendatar dan cukup besar
    dikumpulkan sebagai calon, lalu yang PALING BAWAH dipilih — inilah yang
    membedakan lantai dari plafon, yang sama mendatarnya dan sering sama luasnya.
    """
    o3d = _o3d()
    xyz = np.asarray(xyz, dtype=np.float64)
    total = len(xyz)
    if total < 100:
        return None

    sisa = to_o3d(xyz)
    calon = []
    for _ in range(FLOOR_TRIES):
        if len(sisa.points) < max(100, FLOOR_MIN_FRAC * total):
            break
        try:
            plane, idx = sisa.segment_plane(FLOOR_DIST, 3, 1000)
        except (RuntimeError, ValueError):
            break
        if len(idx) < FLOOR_MIN_FRAC * total:
            break

        pts = np.asarray(sisa.points)[idx]
        n = np.asarray(plane[:3], dtype=np.float64)
        norm = np.linalg.norm(n)
        if norm > 0:
            n /= norm
            if abs(n[2]) >= FLOOR_NZ:
                calon.append((float(pts[:, 2].mean()),
                              n if n[2] > 0 else -n,
                              pts.mean(axis=0)))

        sisa = sisa.select_by_index(idx, invert=True)

    if not calon:
        return None
    _, n, titik = min(calon, key=lambda c: c[0])
    return n, titik


def level_transform(xyz: np.ndarray):
    """Matriks 4x4 yang mendatarkan lantai dan menaruhnya di Z=0. None bila gagal.

    Rotasinya adalah rotasi TERKECIL yang membawa normal lantai ke +Z (Rodrigues).
    Karena terkecil, ia tidak menyuntikkan putaran terhadap sumbu Z sama sekali:
    arah hadap dinding tetap seperti aslinya, hanya kemiringannya yang dikoreksi.
    """
    hasil = find_floor_plane(xyz)
    if hasil is None:
        return None
    n, titik = hasil

    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s, c = float(np.linalg.norm(v)), float(np.dot(n, z))

    if s < 1e-9:
        # sudah mendatar (atau terbalik, yang tidak mungkin karena n dibuat ke atas)
        R = np.eye(3)
    else:
        vx = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))

    L = np.eye(4)
    L[:3, :3] = R
    L[2, 3] = -float((R @ np.asarray(titik, dtype=np.float64))[2])
    return L


def level_angle_deg(L: np.ndarray) -> float:
    """Berapa derajat kemiringan yang dikoreksi oleh L."""
    R = np.asarray(L, dtype=np.float64)[:3, :3]
    kosinus = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(kosinus)))


def quality_verdict(fitness: float, rmse: float) -> str:
    """Nilai hasil registrasi → BAIK / RAGU / GAGAL.

    Keduanya diukur pada EVAL_DIST. fitness = bagian titik sumber yang
    menemukan pasangan (naik seiring besarnya overlap dan benarnya posisi);
    rmse = simpangan rata-rata pasangan itu. Dua-duanya harus bagus: fitness
    tinggi dengan rmse besar berarti "cocok tapi longgar".
    """
    if fitness >= FITNESS_BAIK and rmse <= RMSE_BAIK:
        return BAIK
    if fitness >= FITNESS_RAGU:
        return RAGU
    return GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# Pewarnaan
# ═══════════════════════════════════════════════════════════════════════════════

def color_by_scan(counts: list) -> np.ndarray:
    """Satu warna per scan. `counts` = jumlah titik tiap scan, berurutan."""
    blocks = []
    for i, n in enumerate(counts):
        c = SCAN_COLORS[i % len(SCAN_COLORS)]
        blocks.append(np.tile(np.array(c, dtype=np.uint8), (int(n), 1)))
    if not blocks:
        return np.zeros((0, 3), dtype=np.uint8)
    return np.vstack(blocks)


def color_by_height(xyz: np.ndarray) -> np.ndarray:
    """Viridis menurut Z, skalanya dihitung atas seluruh titik yang diberikan."""
    xyz = np.asarray(xyz)
    n = len(xyz)
    if n == 0:
        return np.zeros((0, 3), dtype=np.uint8)

    z = xyz[:, 2]
    zmin, zmax = float(z.min()), float(z.max())
    norm = (z - zmin) / (zmax - zmin) if zmax > zmin else np.zeros(n)

    try:
        import matplotlib
        cmap = matplotlib.colormaps["viridis"]
        return (cmap(norm)[:, :3] * 255).astype(np.uint8)
    except Exception:  # noqa: BLE001  — greyscale sudah cukup sebagai cadangan
        grey = (norm * 255).astype(np.uint8)
        return np.column_stack([grey, grey, grey])


# ═══════════════════════════════════════════════════════════════════════════════
# Registrasi
# ═══════════════════════════════════════════════════════════════════════════════

def to_o3d(xyz: np.ndarray):
    """numpy (N,3) → Open3D PointCloud."""
    o3d = _o3d()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    return pcd


def preprocess(pcd, voxel: float):
    """Downsample + normal + fitur FPFH, bahan untuk RANSAC."""
    o3d = _o3d()
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return down, fpfh


def global_register(src_down, ref_down, src_fpfh, ref_fpfh, voxel: float):
    """Pencocokan kasar tanpa tebakan awal (FPFH + RANSAC)."""
    o3d = _o3d()
    dist = voxel * 1.5
    return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, ref_down, src_fpfh, ref_fpfh, True, dist,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(200_000, 0.999))


def refine(src, ref, init, voxel: float):
    """Perapihan ICP point-to-plane dari tebakan `init`."""
    o3d = _o3d()
    for p in (src, ref):
        if not p.has_normals():
            p.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    return o3d.pipelines.registration.registration_icp(
        src, ref, voxel * 1.5, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())


def evaluate(src, ref, T, eval_dist: float = EVAL_DIST) -> tuple:
    """Skor hasil registrasi pada jarak tetap. → (fitness, rmse)."""
    o3d = _o3d()
    r = o3d.pipelines.registration.evaluate_registration(src, ref, eval_dist, T)
    return float(r.fitness), float(r.inlier_rmse)


def wall_subset(pcd):
    """Sisakan permukaan tegak saja, untuk dipakai menilai hasil registrasi.

    Lantai dan plafon cocok pada rotasi Z berapa pun sekaligus mendominasi
    jumlah titik, jadi kalau ikut dinilai skornya nyaris tidak bisa membedakan
    solusi yang benar dari yang melenceng 90°. Cloud yang memang tidak punya
    permukaan tegak jatuh kembali memakai semua titik.
    """
    o3d = _o3d()
    p = o3d.geometry.PointCloud(pcd)
    if not p.has_normals():
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=EVAL_DIST * 3, max_nn=30))

    pts = np.asarray(p.points)
    tegak = np.abs(np.asarray(p.normals)[:, 2]) < WALL_NZ
    if int(tegak.sum()) < MIN_WALL_POINTS:
        return pcd.voxel_down_sample(EVAL_DIST / 2)
    return to_o3d(pts[tegak]).voxel_down_sample(EVAL_DIST / 2)


def register_pair(src_xyz: np.ndarray, ref_xyz: np.ndarray, voxel: float,
                  tries: int = DEFAULT_TRIES) -> tuple:
    """Cari transformasi src → ref. → (T 4x4, fitness, rmse).

    Kasar dengan FPFH+RANSAC, lalu ICP bertingkat dari kasar ke halus: hasil
    tiap tingkat jadi tebakan awal tingkat berikutnya, sehingga ICP tidak
    tersangkut di minimum lokal seperti bila langsung dijalankan halus.

    RANSAC mendarat di jawaban berbeda tiap dijalankan — kadang di solusi yang
    melenceng 90°. Karena itu seluruh rangkaian diulang `tries` kali dan yang
    dipakai adalah yang skor dindingnya terbaik.
    """
    src, ref = to_o3d(src_xyz), to_o3d(ref_xyz)
    src_down, src_fpfh = preprocess(src, voxel)
    ref_down, ref_fpfh = preprocess(ref, voxel)

    skala = (voxel * 2, voxel, voxel * 0.5)
    tingkat = [(preprocess(src, s)[0], preprocess(ref, s)[0]) for s in skala]
    src_wall, ref_wall = wall_subset(src), wall_subset(ref)

    terbaik = None
    for _ in range(max(1, int(tries))):
        T = global_register(src_down, ref_down, src_fpfh, ref_fpfh,
                            voxel).transformation
        for (a, b), s in zip(tingkat, skala):
            T = refine(a, b, T, s).transformation
        fitness, rmse = evaluate(src_wall, ref_wall, T)
        if terbaik is None or fitness > terbaik[1]:
            terbaik = (np.asarray(T), fitness, rmse)

    return terbaik


# ═══════════════════════════════════════════════════════════════════════════════
# Alur utama
# ═══════════════════════════════════════════════════════════════════════════════

def read_cloud_xyz(path: str) -> np.ndarray:
    """Baca .ply/.xyz → array (N,3)."""
    o3d = _o3d()
    pcd = o3d.io.read_point_cloud(str(path))
    xyz = np.asarray(pcd.points)
    if len(xyz) == 0:
        raise SystemExit(f"[ERROR] Nol titik terbaca dari {path}")
    return xyz


def merge_dir() -> Path:
    """Folder induk semua hasil merge."""
    return Path(clomcap.OUT_ROOT) / MERGE_DIRNAME


def next_merge_slot(parent) -> Path:
    """Folder bernomor berikutnya di bawah `parent`, mis. .../_merge/003.

    Tiap run masuk folder sendiri agar hasil lama tidak tertimpa — registrasi
    bergantung pada RANSAC yang acak, jadi run yang sama bisa memberi hasil
    berbeda dan yang sebelumnya sering masih layak dibandingkan.

    Nomor diambil dari yang terbesar + 1, bukan mengisi lubang yang kosong,
    supaya urutan nomor selalu mencerminkan urutan waktu.
    """
    parent = Path(parent)
    tertinggi = 0
    if parent.is_dir():
        for p in parent.iterdir():
            if p.is_dir() and p.name.isdigit():
                tertinggi = max(tertinggi, int(p.name))
    return parent / f"{tertinggi + 1:03d}"


def write_transforms(path, names: list, results: list, L=None) -> None:
    """Simpan matriks tiap scan + skornya, agar bisa diperiksa atau dipakai ulang.

    Bila `L` diberikan (perataan lantai), ia sudah dipanggang ke tiap matriks —
    jadi menerapkan matriks di sini pada scan aslinya mereproduksi merged.ply
    apa adanya, termasuk perataannya.
    """
    lines = ["# Transformasi tiap scan ke kerangka acuan (baris 1 = acuan).",
             "# Kolom skor: fitness (makin tinggi makin baik), rmse (meter).", ""]
    if L is not None:
        lines.append(f"# Perataan lantai: {level_angle_deg(L):.2f}° dikoreksi, "
                     "lantai digeser ke Z=0.")
        lines.append("# Sudah termasuk di dalam tiap matriks di bawah.")
        for row in np.asarray(L):
            lines.append("#   " + "  ".join(f"{v: .6f}" for v in row))
        lines.append("")

    for name, r in zip(names, results):
        lines.append(f"{name}")
        lines.append(f"  status  : {r['verdict']}")
        if r["is_ref"]:
            lines.append("  catatan : acuan"
                         + ("" if L is None else ", hanya diratakan"))
        else:
            lines.append(f"  fitness : {r['fitness']:.4f}")
            lines.append(f"  rmse    : {r['rmse']:.4f} m")
        lines.append("  matriks :")
        for row in np.asarray(r["T"]):
            lines.append("    " + "  ".join(f"{v: .6f}" for v in row))
        lines.append("")
    Path(path).write_text("\n".join(lines))


def register_all(clouds: list, names: list, voxel: float,
                 tries: int = DEFAULT_TRIES) -> list:
    """Registrasi semua cloud ke cloud pertama. → daftar hasil per scan.

    Semua discocokkan ke acuan yang sama, bukan berantai, supaya error tidak
    menumpuk dari satu scan ke scan berikutnya.
    """
    ref_xyz = read_cloud_xyz(clouds[0])
    results = [{
        "name": names[0], "is_ref": True, "T": np.eye(4),
        "fitness": 1.0, "rmse": 0.0, "verdict": BAIK, "xyz": ref_xyz,
    }]
    print(f"  Acuan : {names[0]}  ({len(ref_xyz):,} titik)")

    for cloud, name in zip(clouds[1:], names[1:]):
        xyz = read_cloud_xyz(cloud)
        print(f"\n  Mencocokkan {name}  ({len(xyz):,} titik, {tries}x percobaan) …")
        T, fitness, rmse = register_pair(xyz, ref_xyz, voxel, tries)
        verdict = quality_verdict(fitness, rmse)
        print(f"    fitness {fitness:.3f}   rmse {rmse:.4f} m   → {verdict}")
        results.append({
            "name": name, "is_ref": False, "T": T,
            "fitness": fitness, "rmse": rmse, "verdict": verdict,
            "xyz": apply_transform(xyz, T),
        })
    return results


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="clomerge",
        description="Banyak scan dari posisi berbeda → registrasi otomatis → satu PLY.")
    ap.add_argument("files", nargs="+",
                    help="dua atau lebih file .mcap, .mcap.zstd, .ply, atau .xyz; "
                         "yang pertama dipakai sebagai acuan")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d); default deteksi otomatis")
    ap.add_argument("--force", action="store_true",
                    help="konversi ulang MCAP walau hasil sebelumnya masih segar")
    ap.add_argument("--png", action="store_true",
                    help="buat juga PNG visualisasi saat konversi")
    ap.add_argument("--voxel", type=float, default=DEFAULT_VOXEL,
                    help=f"ukuran voxel registrasi dalam meter (default {DEFAULT_VOXEL}); "
                         "perbesar bila ruangan luas atau prosesnya lama")
    ap.add_argument("--tries", type=int, default=DEFAULT_TRIES,
                    help=f"berapa kali registrasi diulang per scan, diambil yang "
                         f"terbaik (default {DEFAULT_TRIES}); perbanyak bila hasilnya "
                         "masih meleset")
    ap.add_argument("--drop-failed", action="store_true", dest="drop_failed",
                    help="buang scan yang registrasinya GAGAL, jangan ikut digabung")
    ap.add_argument("--no-level", action="store_true", dest="no_level",
                    help="jangan ratakan lantai; biarkan kemiringan scan acuan")
    ap.add_argument("--no-grid", action="store_true", dest="no_grid",
                    help="jangan sertakan grid referensi")
    ap.add_argument("--spacing", type=float, default=1.0,
                    help="jarak garis grid dalam meter (default 1.0)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="margin grid di luar data dalam meter (default 1.0)")
    ap.add_argument("--no-open", action="store_true", dest="no_open",
                    help="jangan buka CloudCompare setelah selesai")
    return ap


def run(args) -> None:
    srcs = clomcaps.dedupe_inputs(args.files)
    if len(srcs) < 2:
        raise SystemExit("[ERROR] Butuh minimal dua berkas untuk digabungkan.")

    print("=" * 55)
    print(f"  Masukan : {len(srcs)} berkas")
    print(f"  Acuan   : {os.path.basename(srcs[0])}")
    print(f"  Voxel   : {args.voxel} m")
    print(f"  Percobaan per scan : {args.tries}")
    print("=" * 55)

    clouds, names = [], []
    for i, src in enumerate(srcs, 1):
        print(f"\n[{i}/{len(srcs)}] {src}")
        try:
            clouds.append(clomcaps.prepare_cloud(src, args))
            names.append(os.path.basename(src))
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [GAGAL] Dilewati: {e}")

    if len(clouds) < 2:
        raise SystemExit("\n[ERROR] Kurang dari dua berkas yang berhasil disiapkan.")

    print("\n" + "-" * 55)
    print("  Registrasi")
    print("-" * 55)
    results = register_all(clouds, names, args.voxel, args.tries)

    dipakai = results
    if args.drop_failed:
        dipakai = [r for r in results if r["verdict"] != GAGAL]
        dibuang = [r["name"] for r in results if r["verdict"] == GAGAL]
        if dibuang:
            print(f"\n  Dibuang (GAGAL): {', '.join(dibuang)}")
        if len(dipakai) < 2:
            raise SystemExit("\n[ERROR] Setelah membuang yang gagal, "
                             "tersisa kurang dari dua scan.")

    xyz = np.vstack([r["xyz"] for r in dipakai])
    counts = [len(r["xyz"]) for r in dipakai]

    # Ratakan SEBELUM mewarnai dan sebelum grid dibangun: color_by_height jadi
    # bermakna "tinggi dari lantai", dan grid dihitung dari cloud yang sudah lurus.
    L = None
    if not args.no_level:
        L = level_transform(xyz)
        if L is None:
            print("\n  [WARN] Bidang lantai tidak ditemukan — hasil dibiarkan "
                  "apa adanya.\n         Grid tetap dibuat, tapi lantai mungkin "
                  "tidak sejajar dengannya.")
        else:
            print(f"\n  ✔ Lantai diratakan: {level_angle_deg(L):.2f}° dikoreksi, "
                  "lantai ditaruh di Z=0")
            xyz = apply_transform(xyz, L)
            for r in dipakai:
                r["T"] = L @ np.asarray(r["T"])

    d = next_merge_slot(merge_dir())
    d.mkdir(parents=True, exist_ok=True)
    merged, check = d / "merged.ply", d / "merged_check.ply"

    mg.write_ply(str(merged), xyz.astype(np.float32), color_by_height(xyz))
    mg.write_ply(str(check), xyz.astype(np.float32), color_by_scan(counts))
    write_transforms(d / "transforms.txt", [r["name"] for r in dipakai], dipakai, L)

    print("\n" + "=" * 55)
    print(f"  Total titik : {len(xyz):,}")
    for r, n in zip(dipakai, counts):
        warna = SCAN_COLORS[dipakai.index(r) % len(SCAN_COLORS)]
        print(f"    {r['verdict']:5s}  {r['name']}  ({n:,} titik, RGB {warna})")
    print(f"\n  Hasil run ini → {d}")
    print("    merged.ply  merged_check.ply  grid.ply  transforms.txt")
    print("  (run sebelumnya tetap tersimpan di folder bernomor sebelahnya)")

    ragu = [r["name"] for r in dipakai if r["verdict"] != BAIK]
    if ragu:
        print(f"\n  [PERIKSA] Registrasi belum meyakinkan untuk: {', '.join(ragu)}")
        print("            Buka merged_check.ply — kalau dinding terlihat dobel "
              "beda warna,\n            registrasinya meleset. Coba --voxel lebih besar.")
    print("=" * 55)

    files = []
    if not args.no_grid:
        # Grid dibangun dari merged.ply yang SUDAH diratakan, dan seperti biasa
        # sejajar sumbu di Z=0 — grid tidak pernah diputar mengikuti cloud.
        grid = clomcaps.make_grid_file([str(merged)], d / "grid.ply",
                                       args.spacing, args.margin)
        if grid is not None:
            files.append(str(grid))
    files.extend([str(check), str(merged)])

    if not args.no_open:
        print(f"  Membuka CloudCompare … ({len(files)} berkas)")
        launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
