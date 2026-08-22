#!/usr/bin/env python3
"""mcaptopc_cli.py — konversi MCAP → point cloud LANGSUNG dari argumen.

Beda dengan mcaptopc.py (yang pakai menu interaktif), versi ini:
  • Menerima path MCAP langsung sebagai argumen — tanpa menu.
  • Otomatis mendekompresi file .zstd (hasil `ros2 bag record` dengan kompresi FILE).
  • Menaruh output (.ply/.xyz/.png) di folder yang sama dengan input (atau -o).
  • Memakai ulang seluruh fungsi konversi dari mcaptopc.py (file asli tak diubah).

Pemakaian:
    python mcaptopc_cli.py rekaman.mcap
    python mcaptopc_cli.py ~/bags/test_om_3/test_om_3_0.mcap.zstd
    python mcaptopc_cli.py rekaman.mcap.zstd -o /folder/output -t /map_3d
"""
import os
import sys
import argparse
import tempfile
import shutil
import subprocess
from pathlib import Path

# agar `import mcaptopc` menemukan file di folder yang sama
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import mcaptopc as mp   # memakai ulang fungsi konversi yang sudah ada


ZSTD_MAGIC = b"\x28\xb5\x2f\xfd"


def looks_zstd(path: str) -> bool:
    """True bila file terkompresi zstd (dari ekstensi atau magic byte)."""
    if path.lower().endswith(".zstd"):
        return True
    try:
        with open(path, "rb") as f:
            return f.read(4) == ZSTD_MAGIC
    except OSError:
        return False


def decompress_zstd(path: str) -> str:
    """Dekompresi ke file .mcap sementara; kembalikan path-nya."""
    tmp = tempfile.NamedTemporaryFile(prefix="mcap_", suffix=".mcap", delete=False)
    tmp_path = tmp.name
    tmp.close()
    # 1) coba pustaka python 'zstandard'
    try:
        import zstandard as zstd
        with open(path, "rb") as fi, open(tmp_path, "wb") as fo:
            zstd.ZstdDecompressor().copy_stream(fi, fo)
        return tmp_path
    except ImportError:
        pass
    except Exception:
        pass
    # 2) fallback: perintah CLI 'zstd'
    if shutil.which("zstd"):
        subprocess.run(["zstd", "-d", "-f", "-q", path, "-o", tmp_path], check=True)
        return tmp_path
    os.unlink(tmp_path)
    raise SystemExit("[ERROR] Perlu dekompresi zstd. Pasang salah satu:\n"
                     "        pip install zstandard    (pustaka python)\n"
                     "        atau tool sistem 'zstd'  (sudo apt install zstd)")


def clean_stem(path: str) -> str:
    """Nama dasar tanpa ekstensi .mcap / .mcap.zstd."""
    name = os.path.basename(path)
    for ext in (".zstd", ".mcap"):
        if name.lower().endswith(ext):
            name = name[: -len(ext)]
    return name


def convert(real_mcap: str, orig_path: str, out_dir: str, topic_override=None):
    """Jalankan konversi. real_mcap = file .mcap (mungkin hasil dekompresi),
    orig_path hanya untuk penamaan output."""
    topics = mp.inspect_mcap(real_mcap)

    if topic_override:
        if topic_override not in topics:
            raise SystemExit(f"[ERROR] Topik '{topic_override}' tidak ada.\n"
                             f"        Tersedia: {', '.join(topics)}")
        topic = topic_override
        schema = topics[topic]["schema"]
        print(f"  ✔ Memakai topik pilihan: {topic}  [{schema}]\n")
    else:
        topic, schema = mp.pick_lidar_topic(topics)

    # --- ekstraksi + konversi (mengikuti alur mcaptopc.main) ---
    if schema in mp.POINTCLOUD2_SCHEMAS or "PointCloud2" in schema:
        arrs = mp.extract_pointcloud2_frames(real_mcap, topic)
        if not arrs:
            raise SystemExit("[ERROR] Tidak ada frame PointCloud2 valid.")
        points = np.vstack(arrs)
        print(f"  Gabungan {len(arrs)} frame → {len(points):,} titik\n")
    elif schema in mp.LASER_SCAN_SCHEMAS or "LaserScan" in schema:
        frames = mp.extract_laserscan_frames(real_mcap, topic)
        if not frames:
            raise SystemExit("[ERROR] Tidak ada frame LaserScan valid.")
        strategy = "single" if len(frames) == 1 else "index"
        points = mp.frames_to_pointcloud(frames, strategy=strategy)
    else:
        frames = mp.extract_laserscan_frames(real_mcap, topic)
        if not frames:
            raise SystemExit("[ERROR] Tidak bisa mendekode pesan apa pun.")
        points = mp.frames_to_pointcloud(frames, strategy="index" if len(frames) > 1 else "single")

    if len(points) == 0:
        raise SystemExit("[ERROR] Nol titik valid diekstrak.")

    # --- ringkasan ---
    print("-" * 55)
    print(f"  Total titik : {len(points):,}")
    print(f"  X : {points[:,0].min():.3f} .. {points[:,0].max():.3f} m")
    print(f"  Y : {points[:,1].min():.3f} .. {points[:,1].max():.3f} m")
    print(f"  Z : {points[:,2].min():.3f} .. {points[:,2].max():.3f} m")
    print("-" * 55)

    # --- ekspor (nama mengikuti file asli, di folder out_dir) ---
    stem = clean_stem(orig_path)
    ply = os.path.join(out_dir, f"{stem}_pointcloud.ply")
    xyz = os.path.join(out_dir, f"{stem}_pointcloud.xyz")
    png = os.path.join(out_dir, f"{stem}_scan_viz.png")

    mp.export_ply(points, ply)
    mp.export_xyz(points, xyz)
    try:
        mp.visualise(points, title=f"{stem} → {topic}", output_png=png)
    except Exception as e:  # noqa: BLE001
        print(f"  [WARN] Visualisasi dilewati: {e}")

    print("\n" + "=" * 55)
    print("  Selesai! File keluaran:")
    print(f"    PLY → {ply}")
    print(f"    XYZ → {xyz}")
    print(f"    PNG → {png}")
    print("=" * 55)


def main():
    ap = argparse.ArgumentParser(
        description="Konversi MCAP → point cloud (PLY/XYZ/PNG), langsung dari argumen.")
    ap.add_argument("mcap", help="path file .mcap atau .mcap.zstd")
    ap.add_argument("-o", "--outdir", default=None,
                    help="folder output (default: folder file input)")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d). Default: deteksi otomatis")
    args = ap.parse_args()

    src = args.mcap
    if not os.path.isfile(src):
        raise SystemExit(f"[ERROR] File tidak ditemukan: {src}")

    out_dir = args.outdir or os.path.dirname(os.path.abspath(src))
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 55)
    print(f"  Input  : {src}")
    print(f"  Output : {out_dir}")
    print("=" * 55 + "\n")

    tmp = None
    real = src
    if looks_zstd(src):
        print(f"[zstd] Mendekompresi {os.path.basename(src)} …\n")
        real = decompress_zstd(src)
        tmp = real

    try:
        convert(real, src, out_dir, args.topic)
    finally:
        if tmp and os.path.exists(tmp):
            os.unlink(tmp)


if __name__ == "__main__":
    main()
