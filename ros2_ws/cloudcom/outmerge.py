#!/usr/bin/env python3
"""outmerge.py — penggabungan scan luar ruangan dengan peta yang tumbuh.

Kenapa ada perintah ketiga
──────────────────────────
    clomerge     : semua scan dicocokkan ke scan pertama, FPFH+RANSAC dari nol.
    clomergeout  : rantai melingkar, tiap sambungan diberi tebakan sudut 360/N.
    outmerge     : tanah diratakan dulu supaya pencarian tinggal 3 derajat
                   kebebasan, sudut disapu habis lewat citra tampak-atas, dan
                   scan berikutnya dicocokkan ke PETA GABUNGAN — bukan ke satu
                   scan saja.

Di luar ruangan dua scan yang bersebelahan sering hanya beririsan tipis, dan
tidak ada dinding panjang yang memaksa jawaban. Akibatnya pencocokan pasangan
demi pasangan gampang mendarat di jawaban yang salah tapi skornya lumayan.

Tiga hal yang membuat outmerge berbeda:

1. RATAKAN TANAH DULU, PER SCAN.
   Bidang tanah selalu terlihat dan mudah dicari. Setelah tiap scan diputar
   sehingga normal tanahnya menunjuk ke atas dan tanahnya ditaruh di z=0,
   guling dan angguk sudah terkunci dan tinggi pun sudah sepadan. Yang tersisa
   antara dua scan hanya putaran tegak dan geseran mendatar — enam derajat
   kebebasan menyusut jadi tiga. Ruang cari yang jauh lebih kecil inilah yang
   membuat sisanya mungkin.

2. SAPU SUDUT LEWAT CITRA TAMPAK-ATAS.
   Titik di atas tanah dijatuhkan ke citra tampak-atas. Untuk tiap sudut putar
   yang dicoba, geseran mendatar terbaik dicari sekaligus lewat korelasi fase
   (FFT) — satu kali hitung memberi jawaban untuk semua geseran. Jadi seluruh
   lingkaran 360° bisa disapu rapat, bukan ditebak. Puncak-puncak teratasnya
   dipakai sebagai titik awal ICP.

3. PETA YANG TUMBUH.
   Setelah dua scan pertama menyatu, keduanya digabung menjadi satu peta, dan
   scan berikutnya dicocokkan ke peta itu. Peta punya cakupan lebih luas dan
   ciri lebih banyak daripada scan mana pun sendirian, sehingga jawaban yang
   benar makin menonjol dan yang palsu makin tersaring. Scan yang irisannya
   terlalu tipis dengan tetangga mana pun tetap bisa dapat tempat karena ia
   dicocokkan ke keseluruhan.

Setelah semua terpasang, tiap scan dicocokkan ulang ke peta dari SEMUA scan
lain, beberapa putaran. Ini merapikan hanyutan yang menumpuk selama peta
dibangun berurutan.

Pemakaian:
    outmerge scan_0011*.ply scan_0012*.ply scan_0013*.ply scan_0014*.ply
    outmerge *.mcap --range 6
    outmerge a.ply b.ply c.ply --step-deg 1.5 --seeds 20
"""
import argparse
import os
import sys
import time
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

MERGE_DIRNAME = "_outmerge"

# ── pemotongan jarak ───────────────────────────────────────────────────────────
# Titik jauh di luar ruangan hampir selalu jarang, miring, dan tidak dilihat
# scan tetangga. Ia menambah beban hitung tanpa menambah ciri yang bisa
# dicocokkan, dan ikut menyeret ICP. Dipotong bola, bukan kotak, supaya
# kerapatannya seragam ke segala arah.
DEFAULT_RANGE = 6.0

# ── perataan tanah ────────────────────────────────────────────────────────────
GROUND_DIST = 0.06      # toleransi RANSAC bidang tanah (m)
GROUND_FRAC = 0.40      # bidang dicari di sekian bagian titik terbawah
GROUND_MIN_INLIER = 0.25  # di bawah ini, bidangnya tidak dipercaya

# ── citra tampak-atas ─────────────────────────────────────────────────────────
BEV_CELL = 0.10         # sisi sel citra (m)
BEV_ZMIN = 0.30         # di bawah ini dianggap tanah — datar, tak berciri
BEV_ZMAX = 5.00         # di atas ini biasanya dahan/langit, jarang dan berubah
BEV_SUB = 40_000        # titik yang dicicip untuk membuat citra
DEFAULT_STEP_DEG = 2.0  # kerapatan sapuan sudut
DEFAULT_SEEDS = 14      # berapa puncak teratas dilanjutkan ke ICP
SEED_NMS_DEG = 8.0      # puncak yang lebih dekat dari ini dianggap sama

# ── penilaian ─────────────────────────────────────────────────────────────────
# Dinilai hanya pada permukaan tegak di atas tanah. Tanah cocok pada putaran
# berapa pun sekaligus mendominasi jumlah titik; kalau ikut dinilai, skornya
# nyaris tak bisa membedakan jawaban benar dari yang melenceng.
EVAL_DIST = 0.10        # jarak titik dianggap sepadan (m)
EVAL_VOXEL = 0.05
EVAL_ZMIN = 0.25
EVAL_NZ = 0.60          # |nz| di bawah ini dianggap permukaan tegak
EVAL_MIN_POINTS = 500

ICP_SCALES = (0.60, 0.30, 0.15, 0.08)
ICP_ITER = 60

# Ambang mutu. Nilainya lebih rendah dari clomerge karena diukur dengan
# takaran yang lebih ketat: hanya permukaan tegak, sesudah dijarangkan 5 cm.
FITNESS_BAIK = 0.30
FITNESS_RAGU = 0.15
RMSE_BAIK = 0.06

DEFAULT_ROUNDS = 2      # putaran perapian setelah semua terpasang


def _o3d():
    return clomerge._o3d()


# ═══════════════════════════════════════════════════════════════════════════════
# Dasar
# ═══════════════════════════════════════════════════════════════════════════════

def to_o3d(xyz):
    o3d = _o3d()
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    return p


def apply_transform(xyz, T):
    T = np.asarray(T, dtype=np.float64)
    return np.asarray(xyz, dtype=np.float64) @ T[:3, :3].T + T[:3, 3]


def yaw_matrix(deg: float) -> np.ndarray:
    a = np.radians(float(deg))
    T = np.eye(4)
    T[:3, :3] = np.array([[np.cos(a), -np.sin(a), 0.0],
                          [np.sin(a), np.cos(a), 0.0],
                          [0.0, 0.0, 1.0]])
    return T


def yaw_of(T) -> float:
    """Sudut putar tegak sebuah matriks, dalam derajat 0-360."""
    T = np.asarray(T)
    return float(np.degrees(np.arctan2(T[1, 0], T[0, 0])) % 360.0)


def crop_range(xyz: np.ndarray, radius: float) -> np.ndarray:
    """Sisakan titik dalam bola berjari-jari `radius` dari asal sensor."""
    if radius is None or radius <= 0:
        return xyz
    return xyz[np.linalg.norm(xyz, axis=1) <= float(radius)]


# ═══════════════════════════════════════════════════════════════════════════════
# Perataan tanah
# ═══════════════════════════════════════════════════════════════════════════════

def ground_plane(xyz: np.ndarray):
    """Cari bidang tanah. → (normal menghadap atas, jarak, bagian inlier).

    Dicari hanya di antara titik terbawah. Di luar ruangan, atap mobil, meja,
    atau dahan mendatar bisa menang melawan tanah kalau seluruh cloud ikut
    dipertimbangkan — dan bidang yang salah menghasilkan tinggi yang salah
    untuk seluruh scan.
    """
    if len(xyz) < 100:
        return None
    lo = xyz[xyz[:, 2] < np.percentile(xyz[:, 2], GROUND_FRAC * 100.0)]
    if len(lo) < 50:
        return None

    model, inlier = to_o3d(lo).segment_plane(GROUND_DIST, 3, 1000)
    n = np.array(model[:3], dtype=np.float64)
    d = float(model[3])
    if n[2] < 0:
        n, d = -n, -d
    norma = float(np.linalg.norm(n))
    if norma < 1e-9:
        return None
    return n / norma, d / norma, len(inlier) / len(lo)


def level_transform(xyz: np.ndarray):
    """4x4 yang menegakkan scan: normal tanah → +Z, tanah → z=0. None bila gagal.

    Sudut putar tegaknya sengaja dibiarkan apa adanya — itulah yang nanti dicari
    saat mencocokkan.
    """
    hasil = ground_plane(xyz)
    if hasil is None:
        return None
    n, d, frac = hasil
    if frac < GROUND_MIN_INLIER or abs(n[2]) < 0.8:
        return None

    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = float(np.linalg.norm(v))
    c = float(np.dot(n, z))

    T = np.eye(4)
    if s > 1e-9:
        K = np.array([[0.0, -v[2], v[1]],
                      [v[2], 0.0, -v[0]],
                      [-v[1], v[0], 0.0]])
        T[:3, :3] = np.eye(3) + K + K @ K * ((1.0 - c) / s ** 2)
    T[2, 3] = d          # setelah diputar, tanah ada di z = -d → digeser ke 0
    return T


def tilt_deg(T) -> float:
    """Seberapa miring scan aslinya, dalam derajat."""
    if T is None:
        return 0.0
    return float(np.degrees(np.arccos(np.clip(np.asarray(T)[2, 2], -1.0, 1.0))))


# ═══════════════════════════════════════════════════════════════════════════════
# Citra tampak-atas + korelasi fase
# ═══════════════════════════════════════════════════════════════════════════════

def bev_size(half: float, cell: float) -> int:
    """Sisi citra, dibulatkan ke pangkat dua supaya FFT-nya cepat."""
    n = int(np.ceil(2.0 * half / cell))
    return 1 << int(np.ceil(np.log2(max(8, n))))


def bev_image(xyz: np.ndarray, cell: float, n: int) -> np.ndarray:
    """Titik di atas tanah → citra tampak-atas, sudah dipusatkan ke rata-rata nol.

    Isinya log(1+jumlah titik) per sel, bukan jumlah mentah: satu batang pohon
    yang kebetulan dekat sensor bisa mengisi ribuan titik dalam satu sel dan
    menenggelamkan seluruh sisa citra. Rata-ratanya dikurangkan supaya latar
    yang kosong tidak menyumbang apa pun ke korelasi.
    """
    m = (xyz[:, 2] > BEV_ZMIN) & (xyz[:, 2] < BEV_ZMAX)
    p = xyz[m]
    img = np.zeros((n, n), dtype=np.float64)
    if len(p) == 0:
        return img

    idx = np.floor(p[:, :2] / cell).astype(np.int64) + n // 2
    ok = ((idx[:, 0] >= 0) & (idx[:, 0] < n)
          & (idx[:, 1] >= 0) & (idx[:, 1] < n))
    idx = idx[ok]
    np.add.at(img, (idx[:, 1], idx[:, 0]), 1.0)

    img = np.log1p(img)
    return img - img.mean()


def phase_correlate(a: np.ndarray, b: np.ndarray) -> tuple:
    """Geseran yang paling menyelaraskan b ke a. → (dy sel, dx sel, tinggi puncak).

    Korelasi fase: besaran spektrum dibuang, hanya bedanya fase yang dipakai.
    Karena itu ia tidak peduli satu citra jauh lebih padat dari yang lain —
    yang selalu terjadi ketika satu sisi adalah peta gabungan dan sisi lain
    hanya satu scan.
    """
    A = np.fft.rfft2(a)
    B = np.fft.rfft2(b)
    R = A * np.conj(B)
    R /= np.abs(R) + 1e-12
    c = np.fft.irfft2(R, a.shape)

    k = np.unravel_index(int(np.argmax(c)), c.shape)
    n = a.shape[0]
    dy = k[0] - n if k[0] > n // 2 else k[0]
    dx = k[1] - n if k[1] > n // 2 else k[1]
    return int(dy), int(dx), float(c[k])


def subsample(xyz: np.ndarray, n: int, seed: int) -> np.ndarray:
    if len(xyz) <= n:
        return xyz
    rng = np.random.default_rng(seed)
    return xyz[rng.choice(len(xyz), n, replace=False)]


def bev_seeds(src: np.ndarray, ref: np.ndarray, step_deg: float, seeds: int,
              nms_deg: float = SEED_NMS_DEG) -> list:
    """Sapu seluruh 360° → daftar tebakan awal terbaik. → [(puncak, deg, tx, ty)].

    Tiap sudut hanya butuh satu FFT untuk mendapat geseran mendatar terbaiknya,
    jadi menyapu rapat justru lebih murah daripada mencoba beberapa tebakan
    dengan pencarian geseran yang biasa.
    """
    half = float(max(np.abs(src[:, :2]).max(), np.abs(ref[:, :2]).max())) + 1.0
    n = bev_size(half * 2.0, BEV_CELL)   # dilebihkan: korelasi fase melingkar,
                                         # ruang kosong mencegah jawaban melipat
    a = bev_image(subsample(ref, BEV_SUB, 1), BEV_CELL, n)
    s = subsample(src, BEV_SUB, 0)

    puncak = []
    for deg in np.arange(0.0, 360.0, float(step_deg)):
        b = bev_image(apply_transform(s, yaw_matrix(deg)), BEV_CELL, n)
        dy, dx, pk = phase_correlate(a, b)
        puncak.append((pk, float(deg), dx * BEV_CELL, dy * BEV_CELL))

    puncak.sort(reverse=True)
    simpan = []
    for pk, deg, tx, ty in puncak:
        if all(min(abs(deg - d), 360.0 - abs(deg - d)) > nms_deg
               for _, d, _, _ in simpan):
            simpan.append((pk, deg, tx, ty))
        if len(simpan) >= int(seeds):
            break
    return simpan


def seed_matrix(deg: float, tx: float, ty: float) -> np.ndarray:
    T = yaw_matrix(deg)
    T[0, 3], T[1, 3] = float(tx), float(ty)
    return T


# ═══════════════════════════════════════════════════════════════════════════════
# ICP + penilaian
# ═══════════════════════════════════════════════════════════════════════════════

def pyramid(xyz: np.ndarray, scales=ICP_SCALES) -> list:
    """Satu cloud yang sudah dijarangkan dan dinormalkan di tiap tingkat ICP.

    Dihitung sekali lalu dipakai ulang untuk semua tebakan awal. Peta gabungan
    bisa setengah juta titik; menjarangkannya ulang untuk tiap tebakan jauh
    lebih mahal daripada ICP-nya sendiri.
    """
    o3d = _o3d()
    keluar = []
    p = to_o3d(xyz)
    for s in scales:
        d = p.voxel_down_sample(s)
        d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=s * 3.0, max_nn=30))
        keluar.append(d)
    return keluar


def icp_multi(src_pyr: list, ref_pyr: list, T) -> np.ndarray:
    """ICP point-to-plane dari kasar ke halus; tiap tingkat mengawali berikutnya."""
    o3d = _o3d()
    T = np.asarray(T, dtype=np.float64)
    for a, b, s in zip(src_pyr, ref_pyr, ICP_SCALES):
        T = o3d.pipelines.registration.registration_icp(
            a, b, s * 2.0, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_ITER),
        ).transformation
    return np.asarray(T)


def eval_subset(xyz: np.ndarray):
    """Permukaan tegak di atas tanah, dijarangkan — bahan penilaian."""
    o3d = _o3d()
    atas = xyz[xyz[:, 2] > EVAL_ZMIN]
    if len(atas) < EVAL_MIN_POINTS:
        return to_o3d(xyz).voxel_down_sample(EVAL_VOXEL)

    p = to_o3d(atas)
    p.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=EVAL_DIST * 3, max_nn=30))
    tegak = np.abs(np.asarray(p.normals)[:, 2]) < EVAL_NZ
    if int(tegak.sum()) < EVAL_MIN_POINTS:
        return p.voxel_down_sample(EVAL_VOXEL)
    return to_o3d(np.asarray(p.points)[tegak]).voxel_down_sample(EVAL_VOXEL)


def score(src_ev, ref_ev, T) -> tuple:
    o3d = _o3d()
    r = o3d.pipelines.registration.evaluate_registration(
        src_ev, ref_ev, EVAL_DIST, np.asarray(T, dtype=np.float64))
    return float(r.fitness), float(r.inlier_rmse)


def verdict(fitness: float, rmse: float) -> str:
    if fitness >= FITNESS_BAIK and rmse <= RMSE_BAIK:
        return BAIK
    if fitness >= FITNESS_RAGU:
        return RAGU
    return GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# Pencocokan satu scan ke satu acuan
# ═══════════════════════════════════════════════════════════════════════════════

def register(src_xyz, ref_xyz, src_ev, ref_ev, args,
             src_pyr=None, ref_pyr=None, verbose=True) -> dict:
    """Cocokkan src ke ref: sapu sudut → ICP tiap tebakan → ambil skor terbaik.

    Selisih skor antara jawaban terbaik dan jawaban terbaik berikutnya yang
    BEDA ikut dilaporkan sebagai `margin`. Fitness tinggi dengan margin nyaris
    nol berarti ada dua jawaban yang sama-sama masuk akal — di luar ruangan itu
    tanda kuat bahwa salah satunya keliru, dan angka itulah yang membedakan
    pasangan yang benar-benar terkunci dari yang cuma kebetulan.
    """
    src_pyr = pyramid(src_xyz) if src_pyr is None else src_pyr
    ref_pyr = pyramid(ref_xyz) if ref_pyr is None else ref_pyr

    seeds = bev_seeds(src_xyz, ref_xyz, args.step_deg, args.seeds)
    hasil = []
    for pk, deg, tx, ty in seeds:
        T = icp_multi(src_pyr, ref_pyr, seed_matrix(deg, tx, ty))
        f, r = score(src_ev, ref_ev, T)
        hasil.append({"T": T, "fitness": f, "rmse": r, "seed": deg, "peak": pk})

    hasil.sort(key=lambda h: -h["fitness"])
    terbaik = hasil[0]

    # jawaban dianggap "beda" bila putarannya menyimpang lebih dari 5°
    # atau titik asalnya bergeser lebih dari setengah meter
    margin = terbaik["fitness"]
    y0 = yaw_of(terbaik["T"])
    t0 = terbaik["T"][:3, 3]
    for h in hasil[1:]:
        dy = min(abs(yaw_of(h["T"]) - y0), 360.0 - abs(yaw_of(h["T"]) - y0))
        if dy > 5.0 or np.linalg.norm(h["T"][:3, 3] - t0) > 0.5:
            margin = terbaik["fitness"] - h["fitness"]
            break

    terbaik["margin"] = float(margin)
    terbaik["verdict"] = verdict(terbaik["fitness"], terbaik["rmse"])

    if verbose:
        print(f"      yaw {y0:6.1f}°  geser ({t0[0]:5.2f}, {t0[1]:5.2f}, {t0[2]:5.2f}) m"
              f"   fitness {terbaik['fitness']:.4f}  rmse {terbaik['rmse']:.4f} m"
              f"  margin {margin:+.4f}   → {terbaik['verdict']}")
    return terbaik


# ═══════════════════════════════════════════════════════════════════════════════
# Peta yang tumbuh
# ═══════════════════════════════════════════════════════════════════════════════

def pick_seed_pair(clouds: dict, evals: dict, names: dict, args) -> tuple:
    """Dua scan yang paling meyakinkan satu sama lain → titik mula peta.

    Peta dimulai dari sambungan terkuat, bukan dari scan pertama yang diketik.
    Kalau fondasinya keliru, semua yang menempel di atasnya ikut keliru, dan
    tidak ada apa pun nanti yang bisa memperbaikinya.
    """
    kunci = sorted(clouds)
    pyr = {i: pyramid(clouds[i]) for i in kunci}

    terbaik = None
    for a in kunci:
        for b in kunci:
            if a >= b:
                continue
            print(f"    {names[a]} → {names[b]}")
            h = register(clouds[a], clouds[b], evals[a], evals[b], args,
                         src_pyr=pyr[a], ref_pyr=pyr[b])
            nilai = (h["fitness"], h["margin"])
            if terbaik is None or nilai > terbaik[0]:
                terbaik = (nilai, a, b, h)
    _, a, b, h = terbaik
    return a, b, h, pyr


def grow_map(clouds: dict, evals: dict, names: dict, args) -> dict:
    """Bangun peta: mulai dari pasangan terkuat, lalu tambah satu per satu.

    Tiap putaran, SEMUA scan yang belum terpasang dicoba ke peta yang ada dan
    hanya yang paling meyakinkan yang jadi ditambahkan. Urutannya karena itu
    ditentukan oleh datanya, bukan oleh nomor berkas: scan yang irisannya
    tipis ditunda sampai peta cukup lebar untuk menampungnya.
    """
    kunci = sorted(clouds)
    print("\n  Mencari pasangan pembuka:")
    a, b, h, pyr = pick_seed_pair(clouds, evals, names, args)

    poses = {b: np.eye(4), a: np.asarray(h["T"])}
    catatan = {b: {"acuan": True, "verdict": BAIK, "fitness": None, "rmse": None,
                   "margin": None, "lawan": "—"},
               a: dict(h, lawan=names[b])}
    print(f"\n  Pasangan pembuka : {names[a]} → {names[b]}  "
          f"(fitness {h['fitness']:.4f}, margin {h['margin']:+.4f})")

    peta = np.vstack([clouds[b], apply_transform(clouds[a], poses[a])])
    sisa = [i for i in kunci if i not in poses]

    while sisa:
        peta_pyr = pyramid(peta)
        peta_ev = eval_subset(peta)
        print(f"\n  Peta sekarang {len(poses)} scan, {len(peta):,} titik. "
              f"Mencoba {len(sisa)} sisanya:")

        calon = []
        for i in sisa:
            print(f"    {names[i]} → peta")
            h = register(clouds[i], peta, evals[i], peta_ev, args,
                         src_pyr=pyr[i], ref_pyr=peta_pyr)
            calon.append((h["fitness"], h["margin"], i, h))

        calon.sort(key=lambda c: (-c[0], -c[1]))
        _, _, i, h = calon[0]
        poses[i] = np.asarray(h["T"])
        catatan[i] = dict(h, lawan=f"peta {len(poses) - 1} scan")
        print(f"    ✔ dipasang: {names[i]}  (fitness {h['fitness']:.4f}, "
              f"{h['verdict']})")

        peta = np.vstack([peta, apply_transform(clouds[i], poses[i])])
        sisa.remove(i)

    return {"poses": poses, "catatan": catatan, "pyr": pyr}


def refine_all(clouds: dict, evals: dict, names: dict, poses: dict,
               catatan: dict, pyr: dict, rounds: int) -> None:
    """Cocokkan ulang tiap scan ke peta dari SEMUA scan lain, beberapa putaran.

    Selama peta dibangun berurutan, scan yang dipasang lebih awal tidak pernah
    melihat yang datang belakangan. Putaran ini memberi tiap scan kesempatan
    memakai seluruh sisanya sebagai acuan. Hanya ICP dari pose sekarang — sudut
    tidak disapu ulang — jadi murah, dan hasilnya hanya dipakai kalau skornya
    naik.
    """
    o3d = _o3d()
    kunci = sorted(poses)
    if len(kunci) < 3:
        return

    for putaran in range(1, int(rounds) + 1):
        print(f"\n  Perapian putaran {putaran}/{rounds}:")
        naik = 0
        for i in kunci:
            lain = np.vstack([apply_transform(clouds[j], poses[j])
                              for j in kunci if j != i])
            lain_pyr = pyramid(lain)
            lain_ev = eval_subset(lain)

            f0, r0 = score(evals[i], lain_ev, poses[i])
            T = icp_multi(pyr[i], lain_pyr, poses[i])
            f1, r1 = score(evals[i], lain_ev, T)

            # skor yang dilaporkan selalu diukur terhadap semua scan lain,
            # entah pose barunya diterima atau tidak — supaya angka di laporan
            # akhir sebanding antar scan
            catatan[i]["lawan"] = f"peta {len(kunci) - 1} scan lain"
            catatan[i]["acuan"] = False

            tanda = " "
            if f1 > f0:
                poses[i] = T
                catatan[i].update(fitness=f1, rmse=r1, verdict=verdict(f1, r1))
                naik += 1
                tanda = "✔"
            else:
                catatan[i].update(fitness=f0, rmse=r0, verdict=verdict(f0, r0))
            print(f"    {tanda} {names[i]:<40} fitness {f0:.4f} → {f1:.4f}"
                  f"   rmse {r1:.4f} m")
        if naik == 0:
            print("      tidak ada yang membaik — berhenti lebih awal")
            break

    # setelah dirapikan, acuannya tidak lagi istimewa: semua pose sudah
    # dinilai dengan takaran yang sama, jadi laporannya pun disamakan
    for i in kunci:
        catatan[i].pop("acuan", None)


# ═══════════════════════════════════════════════════════════════════════════════
# Laporan
# ═══════════════════════════════════════════════════════════════════════════════

def write_report(path, urut: list, names: dict, poses: dict, catatan: dict,
                 levels: dict, args) -> None:
    b = ["# outmerge — peta tumbuh, tanah diratakan, sudut disapu",
         f"# Jangkauan dipotong : {args.range if args.range else 'tidak'} m",
         f"# Sapuan sudut       : tiap {args.step_deg}°, "
         f"{args.seeds} puncak teratas dilanjutkan ke ICP",
         "#",
         "# fitness = bagian titik permukaan tegak yang menemukan pasangan",
         f"#           dalam {EVAL_DIST} m (makin tinggi makin baik)",
         "# margin  = selisih fitness terhadap jawaban BEDA terbaik berikutnya;",
         "#           kecil berarti ada tafsir lain yang sama masuk akalnya",
         "",
         "# Ringkasan:"]
    for i in urut:
        c = catatan[i]
        f = "—" if c.get("fitness") is None else f"{c['fitness']:.4f}"
        r = "—" if c.get("rmse") is None else f"{c['rmse']:.4f}"
        m = "—" if c.get("margin") is None else f"{c['margin']:+.4f}"
        b.append(f"#   {names[i]:<44} fitness {f}  rmse {r} m  "
                 f"margin {m}  {c['verdict']}   [dicocokkan ke {c['lawan']}]")
    b.append("")

    b.append("# Perataan tanah tiap scan (sudah termasuk di matriks gabungan):")
    for i in urut:
        L = levels.get(i)
        b.append(f"#   {names[i]:<44} "
                 + ("tanah tidak ditemukan — dibiarkan apa adanya"
                    if L is None else f"{tilt_deg(L):.2f}° dikoreksi"))
    b.append("")

    b.append("# Matriks gabungan tiap scan ke kerangka acuan.")
    b.append("# Sudah termasuk perataan tanah; pakai langsung pada scan asli.")
    b.append("")
    for i in urut:
        L = np.eye(4) if levels.get(i) is None else np.asarray(levels[i])
        M = np.asarray(poses[i]) @ L
        b.append(f"{names[i]}")
        b.append(f"  status  : {catatan[i]['verdict']}")
        b.append(f"  yaw     : {yaw_of(M):.3f}°")
        b.append("  matriks :")
        for row in M:
            b.append("    " + "  ".join(f"{v: .6f}" for v in row))
        b.append("")
    Path(path).write_text("\n".join(b))


# ═══════════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="outmerge",
        description="Gabung scan luar ruangan: tanah diratakan, sudut disapu "
                    "lewat citra tampak-atas, tiap scan berikutnya dicocokkan "
                    "ke peta gabungan.")
    ap.add_argument("files", nargs="+",
                    help="dua atau lebih file .mcap/.mcap.zstd/.ply/.xyz")
    ap.add_argument("--range", type=float, default=DEFAULT_RANGE,
                    help=f"potong tiap scan pada jari-jari ini, meter "
                         f"(default {DEFAULT_RANGE}; 0 = jangan potong)")
    ap.add_argument("--step-deg", type=float, default=DEFAULT_STEP_DEG,
                    dest="step_deg",
                    help=f"kerapatan sapuan sudut, derajat "
                         f"(default {DEFAULT_STEP_DEG})")
    ap.add_argument("--seeds", type=int, default=DEFAULT_SEEDS,
                    help=f"berapa puncak teratas dilanjutkan ke ICP "
                         f"(default {DEFAULT_SEEDS})")
    ap.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS,
                    help=f"putaran perapian setelah semua terpasang "
                         f"(default {DEFAULT_ROUNDS}; 0 = lewati)")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik tertentu (mis. /map_3d)")
    ap.add_argument("--force", action="store_true",
                    help="konversi ulang MCAP walau hasil sebelumnya masih segar")
    ap.add_argument("--png", action="store_true",
                    help="buat juga PNG visualisasi saat konversi")
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
    t0 = time.time()
    srcs = clomcaps.dedupe_inputs(args.files)
    if len(srcs) < 2:
        raise SystemExit("[ERROR] Butuh minimal dua berkas.")

    print("=" * 60)
    print(f"  Masukan   : {len(srcs)} berkas")
    print(f"  Jangkauan : {args.range if args.range else 'utuh'} m")
    print(f"  Sapuan    : tiap {args.step_deg}°, {args.seeds} puncak teratas")
    print("=" * 60)

    clouds, evals, levels, names, paths = {}, {}, {}, {}, {}
    for k, src in enumerate(srcs):
        print(f"\n[{k + 1}/{len(srcs)}] {src}")
        try:
            p = clomcaps.prepare_cloud(src, args)
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [GAGAL] Dilewati: {e}")
            continue

        xyz = clomerge.read_cloud_xyz(p)
        n0 = len(xyz)
        xyz = crop_range(xyz, args.range)
        if len(xyz) < 1000:
            print(f"  [GAGAL] Dilewati: tinggal {len(xyz)} titik setelah dipotong")
            continue

        L = level_transform(xyz)
        if L is None:
            print("  [WARN] Tanah tidak ditemukan — scan tidak diratakan. "
                  "Kemungkinan besar tidak akan terpasang.")
        else:
            xyz = apply_transform(xyz, L)
            print(f"  ✔ tanah diratakan {tilt_deg(L):.2f}°, "
                  f"{n0:,} → {len(xyz):,} titik")

        i = len(clouds)
        clouds[i], levels[i], names[i] = xyz, L, os.path.basename(p)
        paths[i] = p
        evals[i] = eval_subset(xyz)

    if len(clouds) < 2:
        raise SystemExit("\n[ERROR] Kurang dari dua berkas yang berhasil disiapkan.")

    print("\n" + "-" * 60)
    print("  Membangun peta")
    print("-" * 60)
    hasil = grow_map(clouds, evals, names, args)
    poses, catatan, pyr = hasil["poses"], hasil["catatan"], hasil["pyr"]

    if args.rounds > 0:
        refine_all(clouds, evals, names, poses, catatan, pyr, args.rounds)

    urut = sorted(poses)
    bagian = [apply_transform(clouds[i], poses[i]) for i in urut]
    xyz = np.vstack(bagian)
    counts = [len(a) for a in bagian]

    # peta gabungan diturunkan lagi supaya tanahnya persis di z=0
    L = level_transform(xyz)
    if L is not None:
        xyz = apply_transform(xyz, L)
        for i in urut:
            poses[i] = L @ np.asarray(poses[i])

    d = clomerge.next_merge_slot(Path(clomcap.OUT_ROOT) / MERGE_DIRNAME)
    d.mkdir(parents=True, exist_ok=True)
    merged, check = d / "merged.ply", d / "merged_check.ply"

    mg.write_ply(str(merged), xyz.astype(np.float32), clomerge.color_by_height(xyz))
    mg.write_ply(str(check), xyz.astype(np.float32), clomerge.color_by_scan(counts))
    write_report(d / "transforms.txt", urut, names, poses, catatan, levels, args)

    print("\n" + "=" * 60)
    print(f"  Total titik : {len(xyz):,}")
    for k, i in enumerate(urut):
        c = catatan[i]
        f = "acuan" if c.get("fitness") is None else f"fitness {c['fitness']:.4f}"
        print(f"    {names[i]}  ({counts[k]:,} titik, "
              f"RGB {SCAN_COLORS[k % len(SCAN_COLORS)]})  {f}  {c['verdict']}")

    files = []
    if not args.no_grid:
        grid = clomcaps.make_grid_file([str(merged)], d / "grid.ply",
                                       args.spacing, args.margin)
        if grid is not None:
            files.append(str(grid))
    files.extend([str(check), str(merged)])

    print(f"\n  Hasil run ini → {d}")
    print("    merged.ply  merged_check.ply  grid.ply  transforms.txt")
    print(f"  Waktu : {time.time() - t0:.0f} s")

    ragu = [names[i] for i in urut if catatan[i]["verdict"] != BAIK]
    if ragu:
        print(f"\n  [PERIKSA] Belum meyakinkan: {', '.join(ragu)}")
        print("            Buka merged_check.ply — kalau permukaan terlihat")
        print("            dobel beda warna, scan itu meleset. Coba --range")
        print("            lebih besar, atau --step-deg lebih rapat.")
    print("=" * 60)

    if not args.no_open:
        print(f"  Membuka CloudCompare … ({len(files)} berkas)")
        launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
