#!/usr/bin/env python3
"""clomcaps.py — banyak MCAP/PLY/XYZ → satu jendela CloudCompare.

Versi jamak dari clomcap: menerima beberapa file sekaligus, mengonversi yang
perlu dikonversi, membuat satu grid referensi yang menutupi gabungan semua
data, lalu membuka semuanya dalam satu CloudCompare.

Semua logika berat (konversi, cache, pembacaan batas, peluncuran CloudCompare)
dipakai ulang dari clomcap.py — file ini hanya mengatur banyak masukan.

Pemakaian:
    python clomcaps.py scan_0003_1sweep_0.mcap scan_0004_1sweep_0.mcap
    python clomcaps.py *.mcap --force
    python clomcaps.py a.mcap b.ply --no-grid
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np

# agar `import clomcap` dst. menemukan file di folder yang sama
sys.path.insert(0, str(Path(__file__).resolve().parent))

import clomcap
import make_grid as mg
from clomcap import launch_cloudcompare  # noqa: F401  (dipakai lewat global modul)

MULTI_DIRNAME = "_multi"


def dedupe_inputs(files: list) -> list:
    """Buang masukan yang menunjuk file sama, pertahankan urutan aslinya."""
    seen = set()
    out = []
    for f in files:
        key = os.path.normpath(os.path.abspath(f))
        if key in seen:
            print(f"  [SKIP] Duplikat, sudah ada di daftar: {f}")
            continue
        seen.add(key)
        out.append(f)
    return out


def combined_bounds(clouds: list) -> tuple:
    """Min/max XYZ gabungan dari semua cloud.

    Cloud yang gagal dibaca dilewati; grid tetap dibuat dari sisanya.
    """
    mins, maxs = [], []
    for c in clouds:
        try:
            mn, mx = clomcap.bounds_of(c)
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [WARN] Batas {os.path.basename(str(c))} dilewati: {e}")
            continue
        mins.append(mn)
        maxs.append(mx)

    if not mins:
        raise SystemExit("[ERROR] Tidak ada cloud yang bisa dibaca batasnya.")

    return np.min(np.asarray(mins), axis=0), np.max(np.asarray(maxs), axis=0)


def grid_path_for(srcs: list):
    """Lokasi grid: ikut layout clomcap bila satu file, out/_multi bila banyak."""
    if len(srcs) == 1:
        return clomcap.out_paths(srcs[0], root=clomcap.OUT_ROOT)["grid"]
    return Path(clomcap.OUT_ROOT) / MULTI_DIRNAME / "grid.ply"


def make_grid_file(clouds: list, grid_path, spacing: float, margin: float):
    """Buat satu grid yang menutupi semua cloud + margin. → path, atau None.

    Kegagalan tidak fatal: tujuan perintah ini membuka CloudCompare, dan grid
    hanyalah pelengkap.
    """
    grid_path = Path(grid_path)
    try:
        mn, mx = combined_bounds(clouds)
        xyz, rgb = mg.build_grid(
            float(mn[0]) - margin, float(mx[0]) + margin,
            float(mn[1]) - margin, float(mx[1]) + margin,
            0.0, spacing, clomcap.GRID_STEP, clomcap.GRID_MAJOR,
        )
        grid_path.parent.mkdir(parents=True, exist_ok=True)
        mg.write_ply(str(grid_path), xyz, rgb)
        print(f"  ✔ Grid gabungan → {grid_path}  "
              f"({len(xyz):,} titik, spasi {spacing} m)")
        return grid_path
    except (Exception, SystemExit) as e:  # noqa: BLE001
        print(f"  [WARN] Grid dilewati: {e}")
        return None


def prepare_cloud(src: str, args) -> str:
    """Siapkan satu masukan → path point cloud siap buka.

    .ply/.xyz dipakai apa adanya; .mcap dikonversi bila hasil lama sudah basi.
    """
    if not os.path.isfile(src):
        raise SystemExit(f"[ERROR] File tidak ditemukan: {src}")

    if clomcap.classify_input(src) == "cloud":
        return os.path.abspath(src)

    paths = clomcap.out_paths(src, root=clomcap.OUT_ROOT)
    paths["dir"].mkdir(parents=True, exist_ok=True)

    if not args.force and clomcap.is_cache_fresh(src, paths["ply"]):
        print(f"  ✔ Memakai hasil sebelumnya: {paths['ply']}  (--force untuk ulang)")
    else:
        clomcap.convert_to_ply(src, paths, args.topic, args.png)

    return str(paths["ply"])


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="clomcaps",
        description="Banyak MCAP/PLY/XYZ → satu CloudCompare, dengan satu grid gabungan.")
    ap.add_argument("files", nargs="+",
                    help="satu atau lebih file .mcap, .mcap.zstd, .ply, atau .xyz")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d) untuk semua file; "
                         "default deteksi otomatis")
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
    srcs = dedupe_inputs(args.files)

    print("=" * 55)
    print(f"  Masukan : {len(srcs)} berkas")
    print(f"  Output  : {clomcap.OUT_ROOT}")
    print("=" * 55)

    ok_srcs, clouds, gagal = [], [], []
    for i, src in enumerate(srcs, 1):
        print(f"\n[{i}/{len(srcs)}] {src}")
        try:
            clouds.append(prepare_cloud(src, args))
            ok_srcs.append(src)
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [GAGAL] Dilewati: {e}")
            gagal.append(src)

    if not clouds:
        raise SystemExit("\n[ERROR] Tidak ada satu pun berkas yang berhasil disiapkan.")

    print()
    files = []
    if not args.no_grid:
        grid = make_grid_file(clouds, grid_path_for(ok_srcs), args.spacing, args.margin)
        if grid is not None:
            files.append(str(grid))
    files.extend(clouds)

    if gagal:
        print(f"  [CATATAN] {len(gagal)} berkas dilewati: "
              + ", ".join(os.path.basename(g) for g in gagal))

    print(f"  Membuka CloudCompare … ({len(files)} berkas)")
    launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
