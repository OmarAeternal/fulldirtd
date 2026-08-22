#!/usr/bin/env python3
"""
cleanpc.py
──────────
Membersihkan noise dari point cloud hasil scan 3D (PLY / XYZ).

Dirancang untuk keluaran `mcaptopc.py`, tapi menerima file PLY/XYZ apa saja.

Tahapan filter:
  1. Range crop      – buang rangka alat sendiri (terlalu dekat) dan titik nyasar (terlalu jauh)
  2. Dedup           – buang titik duplikat persis
  3. SOR adaptif     – buang outlier statistik, ambang dihitung per-bin jarak
  4. Radius outlier  – buang titik yang tetangganya terlalu sedikit
  5. Voxel downsample – opsional, ratakan kerapatan

Kenapa "adaptif": kerapatan titik LiDAR turun drastis terhadap jarak (pada data uji,
jarak antar-titik 0,010 m di 0,5 m menjadi 0,609 m di 10 m). Filter outlier dengan
ambang global akan menganggap seluruh dinding terjauh sebagai noise dan menghapusnya.
Karena itu setiap titik hanya dibandingkan dengan titik lain yang jaraknya sepadan.

Pemakaian:
    python3 cleanpc.py                        # pilih file interaktif, tanya parameter
    python3 cleanpc.py scan.xyz               # file dari argumen, tanya parameter
    python3 cleanpc.py scan.xyz --yes         # pakai semua default, tanpa tanya
    python3 cleanpc.py scan.xyz --min-range 1.0 --k-mad 2.5
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


# ═══════════════════════════════════════════════════════════════════════════════
# KONSTANTA INTERNAL
#
# Ini sengaja tidak ditanyakan ke pengguna: nilainya tidak bergantung pada ruangan
# atau alat, dan menanyakan semuanya membuat tool melelahkan dipakai.
# ═══════════════════════════════════════════════════════════════════════════════

SOR_K = 16                  # jumlah tetangga untuk statistik SOR
SOR_N_BINS = 12             # jumlah bin jarak
SOR_MIN_BIN = 50            # titik minimum per bin; jumlah bin dikurangi bila data sedikit
MAD_FLOOR_FRAC = 0.15       # lantai MAD sebagai pecahan median (lihat sor_adaptive)
RADIUS_BASE = 0.15          # radius pencarian pada jarak r_ref (meter)
RADIUS_MIN_NEIGHBORS = 6    # tetangga minimum agar sebuah titik dipertahankan
RADIUS_R_REF = 2.0          # jarak acuan tempat radius = RADIUS_BASE (meter)
MAX_REMOVAL_FRAC = 0.5      # tahap yang membuang lebih dari ini dibatalkan

# Default parameter yang ditanyakan ke pengguna
DEFAULT_MIN_RANGE = 0.5
DEFAULT_MAX_RANGE = 15.0
DEFAULT_K_MAD = 3.0
DEFAULT_VOXEL = 0.0         # 0 = mati

STAGE_NAMES = ["Range crop", "Dedup", "SOR adaptif", "Radius outlier", "Voxel downsample"]


# ═══════════════════════════════════════════════════════════════════════════════
# BACA / TULIS FILE
# ═══════════════════════════════════════════════════════════════════════════════

# Pemetaan tipe properti PLY ke dtype numpy little-endian.
_PLY_DTYPES = {
    "float": "<f4", "float32": "<f4",
    "double": "<f8", "float64": "<f8",
    "uchar": "u1", "uint8": "u1",
    "char": "i1", "int8": "i1",
    "ushort": "<u2", "uint16": "<u2",
    "short": "<i2", "int16": "<i2",
    "uint": "<u4", "uint32": "<u4",
    "int": "<i4", "int32": "<i4",
}


def load_cloud(path):
    """Baca file PLY (ascii atau binary little-endian) atau XYZ.

    Mengembalikan (points Nx3 float64, colors Nx3 uint8 atau None).
    """
    ext = Path(path).suffix.lower()
    if ext == ".xyz":
        return _load_xyz(path), None
    if ext == ".ply":
        return _load_ply(path)
    raise ValueError(
        f"Unsupported format '{ext}'. Hanya .ply dan .xyz yang didukung."
    )


def _load_xyz(path):
    data = np.loadtxt(path)
    data = np.atleast_2d(data)
    if data.shape[1] < 3:
        raise ValueError(
            f"File XYZ '{path}' hanya punya {data.shape[1]} kolom, butuh minimal 3."
        )
    return np.ascontiguousarray(data[:, :3], dtype=np.float64)


def _parse_ply_header(f):
    """Baca header PLY dari file biner yang sudah terbuka.

    Mengembalikan (fmt, n_vertex, properties) dengan properties berupa daftar
    (nama, tipe) sesuai urutan kemunculannya.
    """
    magic = f.readline().strip()
    if magic != b"ply":
        raise ValueError("Bukan file PLY yang sah (baris pertama bukan 'ply').")

    fmt = None
    n_vertex = None
    properties = []
    in_vertex_element = False

    while True:
        raw = f.readline()
        if not raw:
            raise ValueError("Header PLY berakhir mendadak, 'end_header' tidak ditemukan.")
        line = raw.strip().decode("ascii", errors="replace")
        if line == "end_header":
            break
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            in_vertex_element = parts[1] == "vertex"
            if in_vertex_element:
                n_vertex = int(parts[2])
        elif parts[0] == "property" and in_vertex_element:
            if parts[1] == "list":
                raise ValueError("Properti list pada element vertex tidak didukung.")
            properties.append((parts[2], parts[1]))

    if fmt is None or n_vertex is None:
        raise ValueError("Header PLY tidak lengkap (format atau element vertex hilang).")
    return fmt, n_vertex, properties


def _load_ply(path):
    with open(path, "rb") as f:
        fmt, n_vertex, properties = _parse_ply_header(f)
        names = [p[0] for p in properties]

        if fmt == "binary_little_endian":
            dtype = []
            for name, typ in properties:
                if typ not in _PLY_DTYPES:
                    raise ValueError(f"Tipe properti PLY '{typ}' tidak dikenal.")
                dtype.append((name, _PLY_DTYPES[typ]))
            dtype = np.dtype(dtype)
            buf = f.read(n_vertex * dtype.itemsize)
            rec = np.frombuffer(buf, dtype=dtype, count=n_vertex)
            columns = {name: rec[name] for name in names}
        elif fmt == "ascii":
            data = np.loadtxt(f)
            data = np.atleast_2d(data)
            columns = {name: data[:, i] for i, name in enumerate(names)}
        elif fmt == "binary_big_endian":
            raise ValueError("PLY binary_big_endian belum didukung.")
        else:
            raise ValueError(f"Format PLY '{fmt}' tidak dikenal.")

    for axis in ("x", "y", "z"):
        if axis not in columns:
            raise ValueError(f"PLY tidak punya properti '{axis}'.")
    points = np.column_stack([columns["x"], columns["y"], columns["z"]]).astype(np.float64)

    # Properti selain x/y/z/rgb (intensity, normal, dsb.) diabaikan, bukan error.
    if all(c in columns for c in ("red", "green", "blue")):
        colors = np.column_stack(
            [columns["red"], columns["green"], columns["blue"]]
        ).astype(np.uint8)
    else:
        colors = None

    return points, colors


def save_xyz(path, points):
    """Tulis file XYZ ASCII, dipisah spasi."""
    np.savetxt(path, points, fmt="%.6f", delimiter=" ")


def save_ply(path, points, colors=None):
    """Tulis file PLY binary little-endian."""
    n = len(points)
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        "comment Generated by cleanpc.py",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if colors is not None:
        lines += ["property uchar red", "property uchar green", "property uchar blue"]
    lines.append("end_header")
    header = ("\n".join(lines) + "\n").encode("ascii")

    if colors is None:
        body = np.ascontiguousarray(points, dtype="<f4").tobytes()
    else:
        rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                 ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = points[:, 0], points[:, 1], points[:, 2]
        rec["r"], rec["g"], rec["b"] = colors[:, 0], colors[:, 1], colors[:, 2]
        body = rec.tobytes()

    with open(path, "wb") as f:
        f.write(header)
        f.write(body)


# ═══════════════════════════════════════════════════════════════════════════════
# FILTER — setiap fungsi mengembalikan boolean mask (True = dipertahankan)
# ═══════════════════════════════════════════════════════════════════════════════

def crop_range(points, min_range, max_range):
    """Pertahankan titik yang jaraknya dari titik asal ada di [min_range, max_range].

    Batas bawah membuang rangka/dudukan alat yang terpindai sendiri; batas atas
    membuang titik nyasar jauh di luar ruangan.
    """
    r = np.linalg.norm(points, axis=1)
    return (r >= min_range) & (r <= max_range)


def dedup(points):
    """Pertahankan kemunculan pertama dari setiap titik yang terduplikasi persis."""
    _, first_idx = np.unique(points, axis=0, return_index=True)
    mask = np.zeros(len(points), dtype=bool)
    mask[first_idx] = True
    return mask


def voxel_downsample(points, voxel):
    """Pertahankan satu titik per sel voxel. voxel <= 0 mematikan filter ini."""
    if voxel <= 0:
        return np.ones(len(points), dtype=bool)
    keys = np.floor(points / voxel).astype(np.int64)
    _, first_idx = np.unique(keys, axis=0, return_index=True)
    mask = np.zeros(len(points), dtype=bool)
    mask[first_idx] = True
    return mask


def sor_adaptive(points, k=SOR_K, k_mad=DEFAULT_K_MAD,
                 n_bins=SOR_N_BINS, min_bin=SOR_MIN_BIN):
    """Statistical outlier removal dengan ambang per-bin jarak.

    Untuk tiap titik, jarak rata-rata ke k tetangga terdekatnya dibandingkan dengan
    median + k_mad x MAD-terskala dari titik-titik yang jaraknya sepadan dari titik
    asal. Membandingkan di dalam bin jarak inilah yang mencegah filter menghapus
    dinding jauh, yang titiknya memang secara alami lebih renggang.

    Median dan MAD dipakai, bukan mean dan standar deviasi, karena mean/std tertarik
    ke atas oleh outlier yang justru sedang dicari.
    """
    n = len(points)
    if n <= k + 1:
        return np.ones(n, dtype=bool)

    tree = cKDTree(points)
    dists, _ = tree.query(points, k=k + 1, workers=-1)
    mean_knn = dists[:, 1:].mean(axis=1)      # kolom 0 adalah titik itu sendiri

    r = np.linalg.norm(points, axis=1)

    # Bin berbasis kuantil, bukan lebar tetap atau logaritmik. Bin kuantil membuat
    # tiap bin berisi jumlah titik yang sama, sehingga statistiknya selalu stabil
    # tak peduli bentuk sebaran jaraknya. Bin logaritmik gagal ketika ada titik
    # tepat di titik asal (r=0), yang membuat seluruh sebaran bin menjadi timpang.
    n_bins_eff = max(1, min(n_bins, n // min_bin))
    edges = np.quantile(r, np.linspace(0.0, 1.0, n_bins_eff + 1))
    edges[0] -= 1e-9
    edges[-1] += 1e-9
    bin_idx = np.clip(np.searchsorted(edges, r, side="right") - 1, 0, n_bins_eff - 1)

    mask = np.ones(n, dtype=bool)
    for b in np.unique(bin_idx):
        sel = bin_idx == b
        d = mean_knn[sel]
        med = np.median(d)
        mad = np.median(np.abs(d - med)) * 1.4826   # diskalakan agar setara std
        # Lantai untuk MAD. Pada permukaan yang kerapatannya nyaris seragam, MAD
        # jatuh mendekati nol dan ambangnya jadi teramat ketat — akibatnya titik
        # tepi permukaan (tepi dinding, kusen, batas oklusi) yang memang sedikit
        # lebih renggang ikut terbuang, padahal itu struktur asli. Lantai ini
        # berarti: jangan sebut sebuah titik outlier kecuali lingkungannya
        # setidaknya ~45% lebih renggang daripada lazimnya pada jarak itu.
        mad = max(mad, MAD_FLOOR_FRAC * med)
        if mad <= 0:
            mad = 1e-9
        mask[sel] = d <= med + k_mad * mad
    return mask


def radius_outlier(points, base_radius=RADIUS_BASE,
                   min_neighbors=RADIUS_MIN_NEIGHBORS, r_ref=RADIUS_R_REF):
    """Buang titik yang tetangganya terlalu sedikit dalam radius yang menskala jarak.

    Radius membesar sebanding jarak dari titik asal, karena radius tetap akan
    memusnahkan dinding jauh yang titiknya memang lebih berjauhan.
    """
    n = len(points)
    if n <= min_neighbors:
        return np.ones(n, dtype=bool)

    tree = cKDTree(points)
    r = np.linalg.norm(points, axis=1)
    radii = base_radius * np.maximum(1.0, r / r_ref)

    counts = np.array(tree.query_ball_point(points, radii, return_length=True))
    return (counts - 1) >= min_neighbors     # kurangi titik itu sendiri


# ═══════════════════════════════════════════════════════════════════════════════
# PIPELINE
# ═══════════════════════════════════════════════════════════════════════════════

def run_pipeline(points, colors, config):
    """Jalankan semua tahap filter berurutan sambil mengumpulkan statistik.

    Tahap yang akan membuang lebih dari MAX_REMOVAL_FRAC dari masukannya dibatalkan
    dan ditandai, supaya setelan yang terlalu agresif tidak diam-diam menghapus
    ruangannya. Range crop dikecualikan: membuang gumpalan besar di dekat sensor
    (rangka alat sendiri) memang justru tujuannya.
    """
    stages = [
        ("Range crop", lambda p: crop_range(p, config["min_range"], config["max_range"]), True),
        ("Dedup", dedup, True),
        ("SOR adaptif", lambda p: sor_adaptive(p, k=SOR_K, k_mad=config["k_mad"]), False),
        ("Radius outlier", radius_outlier, False),
        ("Voxel downsample", lambda p: voxel_downsample(p, config["voxel"]), True),
    ]

    stats = []
    for name, fn, exempt in stages:
        n_in = len(points)
        if n_in == 0:
            stats.append(dict(name=name, n_in=0, n_removed=0, pct=0.0, aborted=False))
            continue

        mask = fn(points)
        n_removed = int(n_in - mask.sum())
        aborted = False

        if not exempt and n_removed / n_in > MAX_REMOVAL_FRAC:
            aborted = True
            n_removed = 0
        else:
            points = points[mask]
            if colors is not None:
                colors = colors[mask]

        stats.append(dict(name=name, n_in=n_in, n_removed=n_removed,
                          pct=100.0 * n_removed / n_in, aborted=aborted))

    return points, colors, stats


# ═══════════════════════════════════════════════════════════════════════════════
# LAPORAN & VISUALISASI
# ═══════════════════════════════════════════════════════════════════════════════

def range_histogram(points, n_bins=10):
    """Cetak sebaran jumlah titik terhadap jarak dari sensor.

    Ditampilkan sebelum pengguna ditanya min_range/max_range, supaya batasnya
    dipilih berdasarkan data berkas itu sendiri, bukan tebakan.

    Mengembalikan daftar (lo, hi, count).
    """
    r = np.linalg.norm(points, axis=1)
    counts, edges = np.histogram(r, bins=n_bins)
    peak = counts.max() if len(counts) and counts.max() > 0 else 1

    print("  Sebaran titik terhadap jarak dari sensor:\n")
    print(f"    {'jarak (m)':<16} {'jumlah':>10}   {'':<40}")
    for i, c in enumerate(counts):
        lo, hi = edges[i], edges[i + 1]
        bar = "#" * int(round(40 * c / peak))
        print(f"    {lo:6.2f} – {hi:6.2f} {c:>10,}   {bar}")
    print()

    return [(float(edges[i]), float(edges[i + 1]), int(counts[i])) for i in range(n_bins)]


def plot_comparison(before, after, path):
    """Tulis PNG perbandingan sebelum/sesudah dalam tiga tampak."""
    import matplotlib
    matplotlib.use("Agg")           # jangan pernah mencoba membuka jendela
    import matplotlib.pyplot as plt

    views = [(0, 1, "Tampak atas (XY)"), (0, 2, "Tampak samping (XZ)"),
             (1, 2, "Tampak samping (YZ)")]
    rows = [("SEBELUM", before), ("SESUDAH", after)]

    fig, axes = plt.subplots(2, 3, figsize=(18, 11))
    for row, (label, pts) in enumerate(rows):
        for col, (a, b, title) in enumerate(views):
            ax = axes[row][col]
            if len(pts):
                ax.scatter(pts[:, a], pts[:, b], s=0.2, c=pts[:, 2],
                           cmap="viridis", linewidths=0)
            ax.set_title(f"{label} – {title}  ({len(pts):,} titik)", fontsize=10)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)

    fig.suptitle("cleanpc.py – perbandingan sebelum dan sesudah filter",
                 fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def print_stats(stats):
    print(f"    {'tahap':<20} {'masuk':>10} {'dibuang':>10} {'%':>7}")
    print(f"    {'-' * 20} {'-' * 10} {'-' * 10} {'-' * 7}")
    for s in stats:
        flag = "  ⚠ DIBATALKAN (>50%)" if s["aborted"] else ""
        print(f"    {s['name']:<20} {s['n_in']:>10,} {s['n_removed']:>10,} "
              f"{s['pct']:>6.1f}%{flag}")
    print()


# ═══════════════════════════════════════════════════════════════════════════════
# INPUT PENGGUNA
# ═══════════════════════════════════════════════════════════════════════════════

def ask_float(prompt, default, min_value=0.0):
    """Tanya sebuah angka. Enter = pakai default. Input salah ditanya ulang."""
    while True:
        try:
            raw = input(f"    {prompt} [{default}]: ").strip()
        except EOFError:
            return default
        if raw == "":
            return default
        try:
            value = float(raw)
        except ValueError:
            print("      Bukan angka. Coba lagi.")
            continue
        if value < min_value:
            print(f"      Nilai tidak boleh kurang dari {min_value}. Coba lagi.")
            continue
        return value


def select_file():
    """Tampilkan daftar file PLY/XYZ di direktori kerja lalu minta pengguna memilih."""
    files = sorted(
        f for f in os.listdir(".")
        if f.lower().endswith((".ply", ".xyz"))
        and not f.lower().endswith(("_clean.ply", "_clean.xyz"))
    )
    if not files:
        print("[ERROR] Tidak ada file .ply atau .xyz di direktori ini.")
        sys.exit(1)

    print(f"Ditemukan {len(files)} file point cloud:\n")
    for i, f in enumerate(files):
        size_mb = os.path.getsize(f) / 1_048_576
        print(f"  [{i + 1}] {f}  ({size_mb:.2f} MB)")

    while True:
        try:
            idx = int(input(f"\nPilih nomor file [1–{len(files)}]: ").strip()) - 1
            if 0 <= idx < len(files):
                print(f"\n✔ Dipilih: {files[idx]}\n")
                return files[idx]
        except (ValueError, EOFError):
            pass
        print("  Pilihan tidak sah.")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def _build_parser():
    p = argparse.ArgumentParser(
        description="Bersihkan noise dari point cloud PLY/XYZ.")
    p.add_argument("file", nargs="?", help="File .ply atau .xyz. Kosong = pilih interaktif.")
    p.add_argument("--min-range", type=float,
                   help=f"Buang titik lebih dekat dari ini, meter (default {DEFAULT_MIN_RANGE})")
    p.add_argument("--max-range", type=float,
                   help=f"Buang titik lebih jauh dari ini, meter (default {DEFAULT_MAX_RANGE})")
    p.add_argument("--k-mad", type=float,
                   help=f"Keagresifan SOR, makin kecil makin agresif (default {DEFAULT_K_MAD})")
    p.add_argument("--voxel", type=float,
                   help=f"Ukuran voxel downsample, meter. 0 = mati (default {DEFAULT_VOXEL})")
    p.add_argument("--yes", action="store_true",
                   help="Pakai semua default tanpa bertanya.")
    return p


def _resolve_config(args):
    """Gabungkan flag CLI, prompt interaktif, dan default menjadi satu config."""
    if args.yes:
        return dict(
            min_range=DEFAULT_MIN_RANGE if args.min_range is None else args.min_range,
            max_range=DEFAULT_MAX_RANGE if args.max_range is None else args.max_range,
            k_mad=DEFAULT_K_MAD if args.k_mad is None else args.k_mad,
            voxel=DEFAULT_VOXEL if args.voxel is None else args.voxel,
        )

    print("─" * 60)
    print("Parameter filter — tekan Enter untuk pakai nilai default")
    print("─" * 60)
    cfg = {}
    cfg["min_range"] = (args.min_range if args.min_range is not None else ask_float(
        "min_range  (m) – buang yang lebih dekat, mis. rangka alat", DEFAULT_MIN_RANGE))
    while True:
        cfg["max_range"] = (args.max_range if args.max_range is not None else ask_float(
            "max_range  (m) – buang yang lebih jauh dari ini", DEFAULT_MAX_RANGE))
        if cfg["max_range"] > cfg["min_range"]:
            break
        print(f"      max_range harus lebih besar dari min_range ({cfg['min_range']}).")
        if args.max_range is not None:
            sys.exit(1)
    cfg["k_mad"] = (args.k_mad if args.k_mad is not None else ask_float(
        "k_mad          – keagresifan SOR, kecil = agresif", DEFAULT_K_MAD))
    cfg["voxel"] = (args.voxel if args.voxel is not None else ask_float(
        "voxel      (m) – ratakan kerapatan, 0 = mati", DEFAULT_VOXEL))
    print()
    return cfg


def main():
    args = _build_parser().parse_args()

    print("\n╔══════════════════════════════════════════════════╗")
    print("║   cleanpc.py – Pembersih Noise Point Cloud       ║")
    print("╚══════════════════════════════════════════════════╝\n")

    path = args.file if args.file else select_file()
    if not os.path.exists(path):
        print(f"[ERROR] File tidak ditemukan: {path}")
        sys.exit(1)

    # ── Muat ──────────────────────────────────────────────────────────────────
    print("─" * 60)
    print("LANGKAH 1 – Membaca file")
    print("─" * 60)
    try:
        points, colors = load_cloud(path)
    except ValueError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    if len(points) == 0:
        print("[ERROR] File tidak berisi titik apa pun.")
        sys.exit(1)

    print(f"\n  File   : {path}")
    print(f"  Titik  : {len(points):,}")
    print(f"  Warna  : {'ada' if colors is not None else 'tidak ada'}")
    print(f"  X      : {points[:, 0].min():.2f} – {points[:, 0].max():.2f} m")
    print(f"  Y      : {points[:, 1].min():.2f} – {points[:, 1].max():.2f} m")
    print(f"  Z      : {points[:, 2].min():.2f} – {points[:, 2].max():.2f} m\n")

    # ── Histogram jarak, lalu parameter ───────────────────────────────────────
    print("─" * 60)
    print("LANGKAH 2 – Sebaran jarak")
    print("─" * 60 + "\n")
    range_histogram(points)

    config = _resolve_config(args)

    # ── Filter ────────────────────────────────────────────────────────────────
    print("─" * 60)
    print("LANGKAH 3 – Menjalankan filter")
    print("─" * 60 + "\n")
    before = points
    filtered, filtered_colors, stats = run_pipeline(points, colors, config)
    print_stats(stats)

    if len(filtered) == 0:
        print("[ERROR] Semua titik terbuang. Longgarkan parameternya "
              "(min_range lebih kecil, k_mad lebih besar).")
        sys.exit(1)

    kept = 100.0 * len(filtered) / len(before)
    print(f"  Tersisa: {len(filtered):,} dari {len(before):,} titik ({kept:.1f}%)\n")

    # ── Simpan ────────────────────────────────────────────────────────────────
    print("─" * 60)
    print("LANGKAH 4 – Menyimpan hasil")
    print("─" * 60 + "\n")
    stem = Path(path).with_suffix("").name
    ply_out = f"{stem}_clean.ply"
    xyz_out = f"{stem}_clean.xyz"
    png_out = f"{stem}_clean_compare.png"

    save_ply(ply_out, filtered, filtered_colors)
    print(f"  ✔ {ply_out}  ({os.path.getsize(ply_out) / 1024:.1f} KB)")
    save_xyz(xyz_out, filtered)
    print(f"  ✔ {xyz_out}  ({os.path.getsize(xyz_out) / 1024:.1f} KB)")
    plot_comparison(before, filtered, png_out)
    print(f"  ✔ {png_out}")

    print("\n" + "═" * 60)
    print("  Selesai. Buka file _clean.ply di CloudCompare.")
    print(f"  File asli '{path}' tidak diubah.")
    print("═" * 60 + "\n")


if __name__ == "__main__":
    main()
