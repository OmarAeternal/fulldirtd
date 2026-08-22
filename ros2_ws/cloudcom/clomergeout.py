#!/usr/bin/env python3
"""clomergeout.py — versi outdoor clomerge: scan melingkar searah, berurutan.

Beda mendasar dari clomerge:

  clomerge     : semua scan dicocokkan ke scan pertama (topologi bintang),
                 tanpa tebakan awal — FPFH+RANSAC mencari dari nol.
  clomergeout  : scan dianggap diambil melingkar searah. Tiap pasang berurutan
                 diberi TEBAKAN AWAL rotasi 360/N derajat, lalu ICP merapikan.
                 Lingkaran ditutup (terakhir → pertama) dan error dibagi rata.

Kenapa beda: di luar ruangan irisan antar scan sering tipis dan scan ke-5 bisa
tidak melihat apa pun dari scan ke-1, sehingga topologi bintang patah. Rantai
berurutan hanya menuntut irisan dengan tetangganya. Dan karena sudut putarnya
kira-kira sudah diketahui, registrasi tidak perlu menebak dari nol — jauh lebih
kokoh saat cirinya sedikit.

Sudutnya TIDAK harus tepat 360/N. Angka itu hanya titik awal; beberapa sudut di
sekitarnya ikut dicoba dan ICP menarik sisanya ke posisi benar. Bila tetap
gagal, pasangan itu jatuh ke FPFH+RANSAC seperti clomerge biasa.

Urutan putaran diambil dari INDEKS pada nama berkas, terkecil dulu — bukan
urutan pengetikan.

Pemakaian:
    python clomergeout.py scan_0003_1sweep_0.mcap scan_0004_1sweep_0.mcap \\
                          scan_0005_1sweep_0.mcap scan_0006_1sweep_0.mcap
    python clomergeout.py *.mcap --step-deg 45 --spread 20
"""
import argparse
import os
import re
import sys
from pathlib import Path

# agar `import clomerge` dst. menemukan file di folder yang sama
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

import clomcap
import clomcaps
import clomerge
import make_grid as mg
from clomcap import launch_cloudcompare  # noqa: F401  (dipakai lewat global modul)
from clomerge import BAIK, RAGU, GAGAL, SCAN_COLORS

MERGE_DIRNAME = "_merge_out"

# Sudut di sekitar tebakan yang ikut dicoba (derajat, ± dari tebakan) dan
# berapa banyak titik yang dicicip di rentang itu. Ini yang membuat program
# tahan terhadap sudut yang tidak persis 360/N.
DEFAULT_SPREAD = 25.0
DEFAULT_SEEDS = 5

# Di bawah fitness ini, hasil berpandu sudut dianggap tidak meyakinkan dan
# pasangan itu diulang dengan FPFH+RANSAC.
FALLBACK_FITNESS = clomerge.FITNESS_BAIK


# ═══════════════════════════════════════════════════════════════════════════════
# Urutan berdasarkan indeks pada nama berkas
# ═══════════════════════════════════════════════════════════════════════════════

def scan_index(path: str):
    """Angka pertama pada nama berkas → int. None bila tidak ada angka.

    'scan_0003_1sweep_0.mcap' → 3.  Kelompok angka PERTAMA yang dipakai, karena
    di situlah nomor scan berada; angka setelahnya menyatakan jumlah sapuan dan
    nomor potongan bag.
    """
    m = re.search(r"\d+", os.path.basename(str(path)))
    return int(m.group()) if m else None


def order_by_index(files: list) -> list:
    """Urutkan dari indeks terkecil. Yang tanpa angka ditaruh di belakang,
    urutan aslinya dipertahankan agar hasilnya bisa diprediksi."""
    berangka = [f for f in files if scan_index(f) is not None]
    tanpa = [f for f in files if scan_index(f) is None]
    return sorted(berangka, key=scan_index) + tanpa


# ═══════════════════════════════════════════════════════════════════════════════
# Geometri tebakan awal
# ═══════════════════════════════════════════════════════════════════════════════

def yaw_matrix(deg: float) -> np.ndarray:
    """Matriks 4x4 rotasi `deg` derajat terhadap sumbu Z (tegak)."""
    a = np.radians(float(deg))
    T = np.eye(4)
    T[:3, :3] = np.array([[np.cos(a), -np.sin(a), 0.0],
                          [np.sin(a), np.cos(a), 0.0],
                          [0.0, 0.0, 1.0]])
    return T


def seed_transform(src_xyz: np.ndarray, ref_xyz: np.ndarray,
                   yaw_deg: float) -> np.ndarray:
    """Tebakan awal: putar `yaw_deg`, lalu himpitkan titik tengah kedua cloud.

    Translasinya kasar, tapi cukup dekat untuk ICP. Titik tengah dipakai karena
    dua scan yang beririsan memandang wilayah yang sebagian besar sama, jadi
    pusat massanya berdekatan setelah rotasi diperbaiki.
    """
    T = yaw_matrix(yaw_deg)
    src_rot = clomerge.apply_transform(np.asarray(src_xyz), T)
    T[:3, 3] = np.asarray(ref_xyz).mean(axis=0) - src_rot.mean(axis=0)
    return T


def seed_angles(yaw_deg: float, spread: float, seeds: int) -> list:
    """Sudut-sudut yang dicoba di sekitar tebakan, termasuk tebakan itu sendiri."""
    seeds = max(1, int(seeds))
    if seeds == 1 or spread <= 0:
        return [float(yaw_deg)]
    return list(np.linspace(yaw_deg - spread, yaw_deg + spread, seeds))


# ═══════════════════════════════════════════════════════════════════════════════
# Aljabar transformasi untuk loop closure
# ═══════════════════════════════════════════════════════════════════════════════

def _skew(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def se3_log(T: np.ndarray) -> tuple:
    """Transformasi rigid → (ω, v), "kecepatan" yang bila dijalankan satu
    satuan waktu menghasilkan T. ω memuat sumbu×sudut."""
    T = np.asarray(T, dtype=np.float64)
    R, t = T[:3, :3], T[:3, 3]

    kosinus = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    th = float(np.arccos(kosinus))
    if th < 1e-9:
        return np.zeros(3), t.copy()

    w = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]]) * (th / (2.0 * np.sin(th)))
    W = _skew(w)
    Vinv = (np.eye(3) - 0.5 * W
            + (1.0 - (th / 2.0) / np.tan(th / 2.0)) / (th ** 2) * (W @ W))
    return w, Vinv @ t


def se3_exp(w: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Kebalikan se3_log: (ω, v) → matriks rigid 4x4."""
    th = float(np.linalg.norm(w))
    T = np.eye(4)
    if th < 1e-9:
        T[:3, 3] = v
        return T

    W = _skew(np.asarray(w, dtype=np.float64))
    W2 = W @ W
    T[:3, :3] = (np.eye(3) + (np.sin(th) / th) * W
                 + ((1.0 - np.cos(th)) / th ** 2) * W2)
    V = (np.eye(3) + ((1.0 - np.cos(th)) / th ** 2) * W
         + ((th - np.sin(th)) / th ** 3) * W2)
    T[:3, 3] = V @ np.asarray(v, dtype=np.float64)
    return T


def se3_scale(T: np.ndarray, f: float) -> np.ndarray:
    """Ambil sebagian `f` dari transformasi rigid T (0 → identitas, 1 → T).

    Lewat logaritma-eksponensial SE(3), bukan sekadar mengalikan sudut dan
    translasi dengan f. Bedanya nyata: dengan cara ini menyusun hasilnya N kali
    mengembalikan T PERSIS, sehingga membagi error lingkaran ke N sambungan
    benar-benar berjumlah kembali menjadi error penuh. Cara linear yang
    lebih sederhana meleset begitu rotasi dan translasinya sama-sama besar —
    dan di luar ruangan, hanyutan sebesar itu justru yang biasa terjadi.
    """
    w, v = se3_log(T)
    return se3_exp(float(f) * w, float(f) * v)


def chain_poses(rel: list) -> list:
    """Susun pose global dari transformasi antar-tetangga.

    `rel[i]` memetakan scan i+1 ke kerangka scan i. Hasilnya `A[i]` memetakan
    scan i ke kerangka scan 0, dengan A[0] = identitas.
    """
    poses = [np.eye(4)]
    for R in rel:
        poses.append(poses[-1] @ np.asarray(R, dtype=np.float64))
    return poses


def loop_error(poses: list, closing: np.ndarray) -> np.ndarray:
    """Selisih saat lingkaran menutup. Idealnya identitas.

    `closing` memetakan scan 0 ke kerangka scan terakhir. Menempuh seluruh
    rantai lalu sambungan penutup seharusnya kembali ke titik awal.
    """
    return np.asarray(poses[-1]) @ np.asarray(closing, dtype=np.float64)


def loop_correction(E: np.ndarray, n_sambungan: int) -> np.ndarray:
    """Koreksi yang disisipkan di tiap sambungan: 1/N dari kebalikan error."""
    return se3_scale(np.linalg.inv(np.asarray(E, dtype=np.float64)),
                     1.0 / max(1, int(n_sambungan)))


def distribute_loop_error(poses: list, E: np.ndarray) -> list:
    """Bagi rata error lingkaran ke seluruh sambungan.

    Tanpa ini, seluruh hanyutan menumpuk di scan terakhir. Koreksi sebesar
    1/N dari kebalikan error disisipkan di tiap sambungan, sehingga selisihnya
    tersebar merata sepanjang lingkaran alih-alih terkumpul di satu ujung.

    Scan 0 tetap identitas — ia acuannya.
    """
    n_sambungan = len(poses)          # rantai + satu sambungan penutup
    if n_sambungan < 2:
        return [np.asarray(p) for p in poses]

    koreksi = loop_correction(E, n_sambungan)

    keluar = [np.eye(4)]
    for i in range(1, len(poses)):
        # sambungan ke-i: pose sebelumnya × langkah relatif × koreksi
        langkah = np.linalg.inv(np.asarray(poses[i - 1])) @ np.asarray(poses[i])
        keluar.append(keluar[-1] @ langkah @ koreksi)
    return keluar


# ═══════════════════════════════════════════════════════════════════════════════
# Registrasi berpandu sudut
# ═══════════════════════════════════════════════════════════════════════════════

def register_pair_seeded(src_xyz: np.ndarray, ref_xyz: np.ndarray, yaw_deg: float,
                         voxel: float, spread: float = DEFAULT_SPREAD,
                         seeds: int = DEFAULT_SEEDS) -> tuple:
    """Cocokkan src → ref dengan tebakan awal sudut. → (T 4x4, fitness, rmse).

    Beberapa sudut di sekitar `yaw_deg` dicoba; tiap tebakan dihaluskan ICP
    bertingkat kasar-ke-halus, lalu yang skor dindingnya terbaik dipakai.
    """
    src = clomerge.to_o3d(src_xyz)
    ref = clomerge.to_o3d(ref_xyz)

    skala = (voxel * 2, voxel, voxel * 0.5)
    tingkat = [(clomerge.preprocess(src, s)[0], clomerge.preprocess(ref, s)[0])
               for s in skala]
    src_wall, ref_wall = clomerge.wall_subset(src), clomerge.wall_subset(ref)

    terbaik = None
    for sudut in seed_angles(yaw_deg, spread, seeds):
        T = seed_transform(src_xyz, ref_xyz, sudut)
        for (a, b), s in zip(tingkat, skala):
            T = clomerge.refine(a, b, T, s).transformation
        fitness, rmse = clomerge.evaluate(src_wall, ref_wall, T)
        if terbaik is None or fitness > terbaik[1]:
            terbaik = (np.asarray(T), fitness, rmse)
    return terbaik


def register_pair_outdoor(src_xyz: np.ndarray, ref_xyz: np.ndarray, yaw_deg: float,
                          voxel: float, spread: float, seeds: int,
                          tries: int) -> tuple:
    """Berpandu sudut dulu; jatuh ke FPFH+RANSAC bila hasilnya belum meyakinkan.

    → (T, fitness, rmse, metode)
    """
    T, fitness, rmse = register_pair_seeded(src_xyz, ref_xyz, yaw_deg,
                                            voxel, spread, seeds)
    if fitness >= FALLBACK_FITNESS:
        return T, fitness, rmse, "sudut"

    print(f"      fitness {fitness:.3f} belum meyakinkan → coba FPFH+RANSAC …")
    T2, f2, r2 = clomerge.register_pair(src_xyz, ref_xyz, voxel, tries)
    if f2 > fitness:
        return np.asarray(T2), f2, r2, "ransac"
    return T, fitness, rmse, "sudut"


# ═══════════════════════════════════════════════════════════════════════════════
# Alur cincin
# ═══════════════════════════════════════════════════════════════════════════════

def register_ring(clouds: list, names: list, args) -> dict:
    """Registrasi melingkar: rantai berurutan + sambungan penutup.

    → dict berisi pose global tiap scan, skor tiap sambungan, dan besar error
    lingkaran sebelum & sesudah dibagi rata.
    """
    n = len(clouds)
    langkah = args.step_deg if args.step_deg is not None else 360.0 / n
    print(f"  Sudut per langkah : {langkah:.1f}°  "
          f"(dicoba ±{args.spread:.0f}°, {args.seeds} titik)")

    xyz = [clomerge.read_cloud_xyz(c) for c in clouds]

    rel, skor = [], []
    for i in range(n - 1):
        print(f"\n  [{i + 1}/{n}] {names[i + 1]} → {names[i]}")
        T, f, r, metode = register_pair_outdoor(
            xyz[i + 1], xyz[i], langkah, args.voxel,
            args.spread, args.seeds, args.tries)
        print(f"      fitness {f:.3f}   rmse {r:.4f} m   "
              f"[{metode}]   → {clomerge.quality_verdict(f, r)}")
        rel.append(T)
        skor.append({"dari": names[i + 1], "ke": names[i],
                     "fitness": f, "rmse": r, "metode": metode,
                     "verdict": clomerge.quality_verdict(f, r)})

    poses = chain_poses(rel)

    hasil = {"poses": poses, "skor": skor, "closing": None,
             "err_sebelum": None, "err_sesudah": None}

    if args.no_loop or n < 3:
        return hasil

    print(f"\n  [tutup] {names[0]} → {names[-1]}")
    C, f, r, metode = register_pair_outdoor(
        xyz[0], xyz[-1], langkah, args.voxel,
        args.spread, args.seeds, args.tries)
    verdict = clomerge.quality_verdict(f, r)
    print(f"      fitness {f:.3f}   rmse {r:.4f} m   [{metode}]   → {verdict}")
    hasil["closing"] = {"dari": names[0], "ke": names[-1], "fitness": f,
                        "rmse": r, "metode": metode, "verdict": verdict}

    if verdict == GAGAL:
        print("      penutup GAGAL — error lingkaran tidak dibagi rata")
        return hasil

    E = loop_error(poses, C)
    hasil["err_sebelum"] = residual_size(E)

    poses_baru = distribute_loop_error(poses, E)
    hasil["poses"] = poses_baru

    # sambungan penutup ikut menerima satu bagian koreksi, sama seperti
    # sambungan rantai lainnya
    koreksi = loop_correction(E, len(poses))
    hasil["err_sesudah"] = residual_size(loop_error(poses_baru, C @ koreksi))

    g0, s0 = hasil["err_sebelum"]
    g1, s1 = hasil["err_sesudah"]
    print(f"\n  Error lingkaran : {g0:.3f} m / {s0:.2f}°  →  "
          f"{g1:.3f} m / {s1:.2f}°  setelah dibagi rata")
    return hasil


def residual_size(E: np.ndarray) -> tuple:
    """Besar sisa error sebuah transformasi. → (meter, derajat)."""
    E = np.asarray(E, dtype=np.float64)
    geser = float(np.linalg.norm(E[:3, 3]))
    kosinus = float(np.clip((np.trace(E[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    return geser, float(np.degrees(np.arccos(kosinus)))


def write_report(path, names: list, hasil: dict, langkah: float, L=None) -> None:
    """Catatan lengkap: urutan, skor tiap sambungan, error lingkaran, matriks."""
    b = ["# clomergeout — registrasi melingkar berurutan",
         f"# Sudut per langkah (tebakan awal): {langkah:.2f}°",
         "# Urutan diambil dari indeks nama berkas, terkecil dulu.",
         ""]
    b.append("# Urutan putaran:")
    for i, nm in enumerate(names):
        b.append(f"#   {i}. {nm}")
    b.append("")

    b.append("# Sambungan antar-tetangga:")
    for s in hasil["skor"]:
        b.append(f"#   {s['dari']} → {s['ke']}   fitness {s['fitness']:.4f}   "
                 f"rmse {s['rmse']:.4f} m   [{s['metode']}]   {s['verdict']}")
    if hasil["closing"]:
        c = hasil["closing"]
        b.append(f"#   PENUTUP {c['dari']} → {c['ke']}   fitness {c['fitness']:.4f}   "
                 f"rmse {c['rmse']:.4f} m   [{c['metode']}]   {c['verdict']}")
    b.append("")

    if hasil["err_sebelum"] is not None:
        g0, s0 = hasil["err_sebelum"]
        g1, s1 = hasil["err_sesudah"]
        b.append(f"# Error lingkaran sebelum dibagi : {g0:.4f} m / {s0:.3f}°")
        b.append(f"# Error lingkaran setelah dibagi : {g1:.4f} m / {s1:.3f}°")
        b.append("")

    if L is not None:
        b.append(f"# Perataan lantai: {clomerge.level_angle_deg(L):.2f}° dikoreksi, "
                 "lantai digeser ke Z=0.")
        b.append("# Sudah termasuk di dalam tiap matriks di bawah.")
        b.append("")

    b.append("# Matriks tiap scan ke kerangka acuan (scan indeks terkecil):")
    for nm, P in zip(names, hasil["poses"]):
        b.append(f"{nm}")
        for row in np.asarray(P):
            b.append("    " + "  ".join(f"{v: .6f}" for v in row))
        b.append("")
    Path(path).write_text("\n".join(b))


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="clomergeout",
        description="Versi outdoor clomerge: scan melingkar searah, berurutan "
                    "menurut indeks nama berkas, dengan tebakan awal sudut.")
    ap.add_argument("files", nargs="+",
                    help="tiga atau lebih file .mcap/.mcap.zstd/.ply/.xyz; "
                         "urutan putaran diambil dari indeks pada namanya")
    ap.add_argument("--step-deg", type=float, default=None, dest="step_deg",
                    help="sudut per langkah dalam derajat (default 360/jumlah file)")
    ap.add_argument("--spread", type=float, default=DEFAULT_SPREAD,
                    help=f"seberapa jauh sudut di sekitar tebakan ikut dicoba, "
                         f"± derajat (default {DEFAULT_SPREAD:.0f})")
    ap.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                    help=f"berapa sudut dicicip dalam rentang itu "
                         f"(default {DEFAULT_SEEDS})")
    ap.add_argument("--no-loop", action="store_true", dest="no_loop",
                    help="jangan tutup lingkaran; rantai saja")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d)")
    ap.add_argument("--force", action="store_true",
                    help="konversi ulang MCAP walau hasil sebelumnya masih segar")
    ap.add_argument("--png", action="store_true",
                    help="buat juga PNG visualisasi saat konversi")
    ap.add_argument("--voxel", type=float, default=clomerge.DEFAULT_VOXEL,
                    help=f"ukuran voxel registrasi dalam meter "
                         f"(default {clomerge.DEFAULT_VOXEL})")
    ap.add_argument("--tries", type=int, default=clomerge.DEFAULT_TRIES,
                    help=f"percobaan RANSAC saat jatuh ke cadangan "
                         f"(default {clomerge.DEFAULT_TRIES})")
    ap.add_argument("--no-level", action="store_true", dest="no_level",
                    help="jangan ratakan lantai/tanah")
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
    srcs = order_by_index(clomcaps.dedupe_inputs(args.files))
    if len(srcs) < 3:
        raise SystemExit("[ERROR] Butuh minimal tiga berkas untuk registrasi "
                         "melingkar. Untuk dua berkas, pakai clomerge.")

    langkah = args.step_deg if args.step_deg is not None else 360.0 / len(srcs)

    print("=" * 55)
    print(f"  Masukan : {len(srcs)} berkas")
    print("  Urutan putaran (menurut indeks nama):")
    for i, s in enumerate(srcs):
        print(f"    {i}. {os.path.basename(s)}   (indeks {scan_index(s)})")
    print(f"  Voxel   : {args.voxel} m")
    print("=" * 55)

    clouds, names = [], []
    for i, src in enumerate(srcs, 1):
        print(f"\n[{i}/{len(srcs)}] {src}")
        try:
            clouds.append(clomcaps.prepare_cloud(src, args))
            names.append(os.path.basename(src))
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [GAGAL] Dilewati: {e}")

    if len(clouds) < 3:
        raise SystemExit("\n[ERROR] Kurang dari tiga berkas yang berhasil disiapkan.")

    print("\n" + "-" * 55)
    print("  Registrasi melingkar")
    print("-" * 55)
    hasil = register_ring(clouds, names, args)

    xyz_list = [clomerge.apply_transform(clomerge.read_cloud_xyz(c), P)
                for c, P in zip(clouds, hasil["poses"])]
    xyz = np.vstack(xyz_list)
    counts = [len(a) for a in xyz_list]

    L = None
    if not args.no_level:
        L = clomerge.level_transform(xyz)
        if L is None:
            print("\n  [WARN] Bidang tanah tidak ditemukan — hasil dibiarkan "
                  "apa adanya.")
        else:
            print(f"\n  ✔ Tanah diratakan: {clomerge.level_angle_deg(L):.2f}° "
                  "dikoreksi, ditaruh di Z=0")
            xyz = clomerge.apply_transform(xyz, L)
            hasil["poses"] = [L @ np.asarray(P) for P in hasil["poses"]]

    d = clomerge.next_merge_slot(Path(clomcap.OUT_ROOT) / MERGE_DIRNAME)
    d.mkdir(parents=True, exist_ok=True)
    merged, check = d / "merged.ply", d / "merged_check.ply"

    mg.write_ply(str(merged), xyz.astype(np.float32), clomerge.color_by_height(xyz))
    mg.write_ply(str(check), xyz.astype(np.float32), clomerge.color_by_scan(counts))
    write_report(d / "transforms.txt", names, hasil, langkah, L)

    print("\n" + "=" * 55)
    print(f"  Total titik : {len(xyz):,}")
    for i, (nm, n) in enumerate(zip(names, counts)):
        print(f"    {i}. {nm}  ({n:,} titik, RGB {SCAN_COLORS[i % len(SCAN_COLORS)]})")

    files = []
    if not args.no_grid:
        grid = clomcaps.make_grid_file([str(merged)], d / "grid.ply",
                                       args.spacing, args.margin)
        if grid is not None:
            files.append(str(grid))
    files.extend([str(check), str(merged)])

    print(f"\n  Hasil run ini → {d}")
    print("    merged.ply  merged_check.ply  grid.ply  transforms.txt")

    semua = list(hasil["skor"]) + ([hasil["closing"]] if hasil["closing"] else [])
    ragu = [f"{s['dari']}→{s['ke']}" for s in semua if s["verdict"] != BAIK]
    if ragu:
        print(f"\n  [PERIKSA] Sambungan belum meyakinkan: {', '.join(ragu)}")
        print("            Buka merged_check.ply — kalau permukaan terlihat dobel")
        print("            beda warna, sambungan itu meleset. Coba --spread lebih")
        print("            lebar, atau --step-deg yang lebih sesuai kenyataan.")
    print("=" * 55)

    if not args.no_open:
        print(f"  Membuka CloudCompare … ({len(files)} berkas)")
        launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
