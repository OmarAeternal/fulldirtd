#!/usr/bin/env python3
"""clomcap.py — satu perintah: MCAP → point cloud + grid → CloudCompare.

Menerima .mcap/.mcap.zstd (dikonversi) atau .ply/.xyz (dipakai apa adanya),
membuat grid referensi yang pas dengan luas data, lalu membuka CloudCompare
berisi keduanya. Hasil disimpan di cloudcom/out/<stem>/.

Pemakaian:
    python clomcap.py tes/016/016_0.mcap
    python clomcap.py scan_0003_3sweep_0.mcap -t /map_3d --force
    python clomcap.py tes/016/016_0_pointcloud.xyz
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# agar `import mcaptopc` dst. menemukan file di folder yang sama
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

import make_grid as mg
import mcaptopc as mp
import mcaptopc_cli as mcli

def default_out_root() -> Path:
    """Folder induk semua hasil.

    Sengaja di luar ros2_ws: hasil olahan menumpang di folder data (tempat
    .mcap disimpan), bukan di dalam workspace ROS yang ikut dibangun colcon
    dan dilacak git. Bisa dialihkan lewat env var CLOUDCOM_OUT tanpa
    mengubah kode.
    """
    env = os.environ.get("CLOUDCOM_OUT")
    if env:
        return Path(env).expanduser()
    return Path.home() / "riset td" / "cloudcom" / "out"


OUT_ROOT = default_out_root()

MCAP_EXTS = (".mcap", ".mcap.zstd")
CLOUD_EXTS = (".ply", ".xyz")

FLATPAK_APP = "org.cloudcompare.CloudCompare"

GRID_STEP = 0.03    # kerapatan titik sepanjang garis (m)
GRID_MAJOR = 5.0    # garis mayor tiap N meter


def classify_input(path: str) -> str:
    """→ 'mcap' bila perlu konversi, 'cloud' bila sudah berupa point cloud."""
    low = str(path).lower()
    if low.endswith(MCAP_EXTS):
        return "mcap"
    if low.endswith(CLOUD_EXTS):
        return "cloud"
    raise SystemExit(
        f"[ERROR] Ekstensi tidak didukung: {os.path.basename(path)}\n"
        f"        Didukung: .mcap, .mcap.zstd, .ply, .xyz")


def stem_of(path: str) -> str:
    """Nama dasar tanpa ekstensi, sesuai jenis input.

    clean_stem() hanya membuang .mcap/.zstd, jadi untuk .ply/.xyz kita
    buang ekstensinya sendiri.
    """
    if classify_input(path) == "mcap":
        return mcli.clean_stem(path)
    return Path(path).stem


def out_paths(src: str, root: Path | None = None) -> dict:
    """Path keluaran untuk satu input. Tidak membuat direktori."""
    root = OUT_ROOT if root is None else Path(root)
    stem = stem_of(src)
    d = root / stem
    return {
        "dir": d,
        "ply": d / f"{stem}.ply",
        "png": d / f"{stem}_viz.png",
        "grid": d / "grid.ply",
    }


def is_cache_fresh(src: str, ply: Path) -> bool:
    """True bila `ply` sudah ada, tidak kosong, dan tidak lebih tua dari `src`."""
    ply = Path(ply)
    if not ply.is_file() or ply.stat().st_size == 0:
        return False
    return ply.stat().st_mtime >= os.stat(src).st_mtime


def xyz_minmax(path: str) -> tuple:
    """Baca min/max XYZ dari file .xyz teks (3 kolom pertama, pisah spasi)."""
    pts = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(("#", "//")):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except ValueError:
                continue
    if not pts:
        raise SystemExit(f"[ERROR] Tidak ada titik valid di {path}")
    a = np.asarray(pts, dtype=np.float64)
    return a.min(axis=0), a.max(axis=0)


def bounds_of(path: str) -> tuple:
    """Min/max XYZ dari .ply (biner) atau .xyz (teks)."""
    low = str(path).lower()
    if low.endswith(".ply"):
        return mg.ply_xyz_minmax(path)
    if low.endswith(".xyz"):
        return xyz_minmax(path)
    raise SystemExit(f"[ERROR] Tidak bisa membaca batas dari: {path}")


def select_topic(topics: dict, override: str | None = None) -> tuple:
    """Pilih topik point cloud tanpa bertanya ke pengguna.

    Prioritas: /map_3d → PointCloud2 lain → LaserScan.
    Berbeda dari mcaptopc.pick_lidar_topic yang akan memanggil input().
    """
    def _listing():
        if not topics:
            return "        (file ini tidak punya topik sama sekali)"
        return "\n".join(f"        {t}  [{v['schema']}]" for t, v in topics.items())

    if override:
        if override not in topics:
            raise SystemExit(f"[ERROR] Topik '{override}' tidak ada di file ini.\n"
                             f"        Topik tersedia:\n{_listing()}")
        return override, topics[override]["schema"]

    pc2 = {t: v for t, v in topics.items() if v["schema"] in mp.POINTCLOUD2_SCHEMAS}
    laser = {t: v for t, v in topics.items() if v["schema"] in mp.LASER_SCAN_SCHEMAS}

    if mp.PREFERRED_PC2_TOPIC in pc2:
        t = mp.PREFERRED_PC2_TOPIC
        return t, pc2[t]["schema"]
    for group in (pc2, laser):
        if group:
            t, v = next(iter(group.items()))
            return t, v["schema"]

    raise SystemExit(f"[ERROR] Tidak ada topik PointCloud2 atau LaserScan di file ini.\n"
                     f"        Pakai -t untuk memilih manual. Topik tersedia:\n{_listing()}")


def _extract_points(real_mcap: str, topic: str, schema: str):
    """Ekstrak titik sesuai schema; mengikuti alur mcaptopc_cli.convert."""
    if schema in mp.POINTCLOUD2_SCHEMAS or "PointCloud2" in schema:
        arrs = mp.extract_pointcloud2_frames(real_mcap, topic)
        if not arrs:
            raise SystemExit("[ERROR] Tidak ada frame PointCloud2 valid.")
        return np.vstack(arrs)

    frames = mp.extract_laserscan_frames(real_mcap, topic)
    if not frames:
        raise SystemExit("[ERROR] Tidak ada frame yang bisa didekode.")
    strategy = "single" if len(frames) == 1 else "index"
    return mp.frames_to_pointcloud(frames, strategy=strategy)


def convert_to_ply(src: str, paths: dict, topic: str | None, want_png: bool) -> None:
    """Konversi MCAP → PLY di paths['ply']. PNG hanya bila want_png."""
    tmp = None
    real = src
    if mcli.looks_zstd(src):
        print(f"[zstd] Mendekompresi {os.path.basename(src)} …")
        real = mcli.decompress_zstd(src)
        tmp = real

    try:
        topics = mp.inspect_mcap(real)
        chosen, schema = select_topic(topics, topic)
        print(f"  ✔ Topik: {chosen}  [{schema}]")

        points = _extract_points(real, chosen, schema)
        if len(points) == 0:
            raise SystemExit("[ERROR] Nol titik valid diekstrak.")

        print(f"  Total titik : {len(points):,}")
        print(f"  X : {points[:, 0].min():.3f} .. {points[:, 0].max():.3f} m")
        print(f"  Y : {points[:, 1].min():.3f} .. {points[:, 1].max():.3f} m")
        print(f"  Z : {points[:, 2].min():.3f} .. {points[:, 2].max():.3f} m")

        mp.export_ply(points, str(paths["ply"]))

        if want_png:
            try:
                mp.visualise(points, f"{stem_of(src)} → {chosen}", str(paths["png"]))
            except Exception as e:  # noqa: BLE001
                print(f"  [WARN] Visualisasi PNG dilewati: {e}")
    finally:
        if tmp and os.path.exists(tmp):
            os.unlink(tmp)


def make_grid_file(cloud: str, grid_path, spacing: float, margin: float):
    """Buat grid yang menutupi `cloud` + margin. → path grid, atau None bila gagal.

    Kegagalan tidak fatal: tujuan perintah ini membuka CloudCompare, dan grid
    hanyalah pelengkap.
    """
    grid_path = Path(grid_path)
    try:
        mn, mx = bounds_of(cloud)
        xyz, rgb = mg.build_grid(
            float(mn[0]) - margin, float(mx[0]) + margin,
            float(mn[1]) - margin, float(mx[1]) + margin,
            0.0, spacing, GRID_STEP, GRID_MAJOR,
        )
        mg.write_ply(str(grid_path), xyz, rgb)
        print(f"  ✔ Grid → {grid_path}  ({len(xyz):,} titik, spasi {spacing} m)")
        return grid_path
    except Exception as e:  # noqa: BLE001
        print(f"  [WARN] Grid dilewati: {e}")
        return None


def launch_cloudcompare(files: list) -> None:
    """Buka CloudCompare terlepas dari terminal (seperti fungsi bash `clocom`)."""
    if not shutil.which("flatpak"):
        listing = "\n".join(f"        {f}" for f in files)
        raise SystemExit("[ERROR] 'flatpak' tidak ditemukan — CloudCompare tidak bisa "
                         "dibuka otomatis.\n"
                         f"        File sudah siap, buka manual:\n{listing}")

    cmd = ["setsid", "flatpak", "run", FLATPAK_APP] + [str(f) for f in files]
    subprocess.Popen(
        cmd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="clomcap",
        description="MCAP/PLY/XYZ → point cloud + grid → CloudCompare, satu perintah.")
    ap.add_argument("file", help="file .mcap, .mcap.zstd, .ply, atau .xyz")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d); default deteksi otomatis")
    ap.add_argument("--force", action="store_true",
                    help="konversi ulang walau hasil sebelumnya masih segar")
    ap.add_argument("--png", action="store_true",
                    help="buat juga PNG visualisasi (default: tidak, demi kecepatan)")
    ap.add_argument("--no-grid", action="store_true", dest="no_grid",
                    help="buka point cloud saja, tanpa grid")
    ap.add_argument("--spacing", type=float, default=1.0,
                    help="jarak garis grid dalam meter (default 1.0)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="margin grid di luar data dalam meter (default 1.0)")
    return ap


def run(args) -> None:
    src = args.file
    if not os.path.isfile(src):
        raise SystemExit(f"[ERROR] File tidak ditemukan: {src}")

    kind = classify_input(src)
    paths = out_paths(src, root=OUT_ROOT)
    paths["dir"].mkdir(parents=True, exist_ok=True)

    print("=" * 55)
    print(f"  Input  : {src}")
    print(f"  Output : {paths['dir']}")
    print("=" * 55)

    if kind == "cloud":
        cloud = os.path.abspath(src)
    else:
        cloud = str(paths["ply"])
        if not args.force and is_cache_fresh(src, paths["ply"]):
            print(f"  ✔ Memakai hasil sebelumnya: {paths['ply']}  (--force untuk ulang)")
        else:
            convert_to_ply(src, paths, args.topic, args.png)

    files = []
    if not args.no_grid:
        grid = make_grid_file(cloud, paths["grid"], args.spacing, args.margin)
        if grid is not None:
            files.append(str(grid))
    files.append(cloud)

    print(f"  Membuka CloudCompare … ({len(files)} berkas)")
    launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
