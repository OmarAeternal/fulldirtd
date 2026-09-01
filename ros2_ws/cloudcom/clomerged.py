#!/usr/bin/env python3
"""clomerged.py — penggabungan yang SADAR FITUR, untuk tempat bertembok polos.

Kenapa ada perintah keempat
───────────────────────────
    clomerge     : semua scan → scan pertama, FPFH+RANSAC dari nol.
    clomergeout  : rantai melingkar, tiap sambungan ditebak 360/N.
    outmerge     : tanah diratakan, sudut disapu lewat citra tampak-atas,
                   scan berikutnya dicocokkan ke peta gabungan.
    clomerged    : outmerge + tiga koreksi supaya tembok polos tidak menang
                   melawan ciri kecil yang justru menentukan.

Penyakit yang diobati
─────────────────────
Di tempat semi-outdoor dengan satu tembok panjang, hasil gabungan sering
"menempel rapi" ke temboknya tapi salah tempat menyusuri tembok itu: tulisan
di dinding jadi dobel dan bergeser — FILKOM terbaca "FILFILKOM".

Sebuah bidang hanya mengunci 3 dari 6 derajat kebebasan (geseran tegak lurus
bidang, dan dua kemiringan). Tembok + tanah mengunci 5. Yang tersisa satu:
GESERAN MENYUSURI TEMBOK. Yang bisa mengunci sisa satu itu cuma ciri kecil
— huruf timbul, tiang, kusen — dan justru itulah yang kalah suara.

Diukur pada data scan_0072-0075 (tempat ber-tulisan FILKOM):

  - Satu scan dinilai melawan DIRINYA SENDIRI, digeser menyusuri tembok:
        geser 0,5 m → fitness masih 0,54
        geser 1,0 m → fitness masih 0,48
    Padahal ambang "BAIK" cuma 0,30. Skornya memang tidak bisa membedakan.

  - Sebabnya: toleransi penilaian 10 cm, sedangkan jarak antar titik cuma
    3-4 cm. Pada permukaan serapat itu tiap titik pasti menemukan pasangan
    di mana pun ia ditaruh. Metriknya jenuh — yang diukur bukan "apakah aku
    di tempat yang benar", tapi cuma "apakah aku menempel di suatu permukaan".

Tiga koreksinya
───────────────
1. PENILAIAN YANG TAJAM.
   Dinilai hanya pada titik MENONJOL — titik yang tidak terletak di salah satu
   bidang besar — dan dengan toleransi 3 cm, bukan 10 cm. Huruf timbul 5-15 cm
   dari tembok: begitu digeser, mukanya mendarat di tembok polos dan langsung
   jadi pencilan. Tembok polos sendiri tidak ikut menyumbang suara.

2. PENCUPLIKAN BERIMBANG MENURUT ARAH NORMAL (normal-space sampling).
   ICP point-to-plane sama sekali tidak punya gradien menyusuri sebuah bidang.
   Yang punya gradien ke arah itu justru sisi-samping huruf, kusen, dan tiang —
   permukaan yang normalnya menghadap MENYUSURI tembok. Jumlahnya sedikit, jadi
   kalau titik diambil apa adanya, tembok menenggelamkan mereka. Di sini titik
   sumber dicuplik berimbang per arah normal, sehingga permukaan langka itu
   punya suara sebesar tembok.

3. PELEPASAN GESERAN (unslide).
   Setelah ICP mengendap, arah paling lemah dihitung dari matriks Hessian
   point-to-plane. Kalau memang ada arah yang nyaris tak terkunci, sepanjang
   arah itu disapu satu dimensi dengan penilaian tajam butir 1, lalu puncak
   terbaiknya dipakai sebagai titik awal ICP ulang. Inilah yang membetulkan
   "FILFILKOM" secara langsung.

Laporan akhir menyertakan kolom `tajam` (fitness penilaian tajam) dan `lemah`
(seberapa tidak terkuncinya arah terlemah). Fitness longgar yang tinggi tapi
tajam yang rendah = menempel di tembok, salah tempat.

Pemakaian:
    clomerged scan_0072*.mcap scan_0073*.mcap scan_0074*.mcap scan_0075*.mcap
    clomerged *.ply --sharp 0.02 --slide-range 2.0
    clomerged *.mcap --no-unslide          # matikan koreksi geseran
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
import outmerge
from clomcap import launch_cloudcompare  # noqa: F401  (dipakai lewat global modul)
from clomerge import BAIK, RAGU, GAGAL, SCAN_COLORS

MERGE_DIRNAME = "_merged"

# ── peta bidang ───────────────────────────────────────────────────────────────
# Bidang besar dikupas berulang. Yang dianggap "besar" harus mencakup minimal
# sekian bagian titik — kalau ambangnya terlalu kecil, tiap kepingan kecil ikut
# terhitung bidang dan tidak ada lagi yang tersisa sebagai ciri.
PLANE_TOL = 0.05        # tebal bidang saat dikupas (m)
PLANE_MIN_FRAC = 0.015  # pengupasan berhenti bila bidangnya sudah sekecil ini
PLANE_MAX = 14          # berapa kali dikupas sebelum berhenti

# Sebuah bidang baru dianggap LATAR bila luas terisinya sebesar ini. Ambang luas
# ini penting dan bukan hiasan: muka semua huruf timbul terletak pada satu bidang
# yang sama (sejajar tembok, maju belasan sentimeter), dan jumlah titiknya bisa
# mudah melewati PLANE_MIN_FRAC. Kalau yang dipakai hanya jumlah titik, bidang
# huruf ikut terkupas sebagai latar — dan justru cirinya yang hilang, kebalikan
# dari yang diinginkan. Luasnya jauh berbeda: tembok berpuluh m², tulisan
# beberapa m² saja.
PLANE_MIN_AREA = 8.0    # m² terisi
PLANE_AREA_CELL = 0.20  # sisi sel saat menghitung luas terisi (m)

# Titik dianggap MENONJOL bila jaraknya ke bidang terdekat melebihi ini.
# 4 cm: cukup di atas ketebalan bidang (5 cm) supaya derau tembok tidak lolos,
# cukup di bawah tinggi huruf (5-15 cm) supaya hurufnya lolos.
SALIENT_TOL = 0.04
SALIENT_VOXEL = 0.03

# ── penilaian tajam ───────────────────────────────────────────────────────────
# Toleransi ini yang membedakan clomerged dari outmerge. 3 cm masih di atas
# derau sensor, tapi sudah jauh di bawah tinggi huruf — jadi huruf yang meleset
# terhukum, sementara permukaan yang benar-benar bertumpuk tetap lolos.
SHARP_DIST = 0.03
REF_VOXEL = 0.015       # acuan dijarangkan lebih halus dari toleransi
SHARP_MIN_POINTS = 300  # di bawah ini, penilaian tajam tidak dipercaya

# Ambang mutu untuk fitness TAJAM. Jauh lebih rendah dari ambang longgar karena
# takarannya jauh lebih ketat: hanya titik menonjol, hanya dalam 3 cm.
TAJAM_BAIK = 0.25
TAJAM_RAGU = 0.12

# ── pencuplikan berimbang ─────────────────────────────────────────────────────
NORMAL_BINS = 8         # petak arah normal: 8 azimut x 8 elevasi
BALANCED_MAX = 12_000   # titik sumber per tingkat ICP
SALIENT_SHARE = 0.5     # separuh jatah titik sumber dijamin untuk titik menonjol

# ── pelepasan geseran ─────────────────────────────────────────────────────────
WEAK_RATIO = 0.08       # λmin/λmaks di bawah ini → arah itu dianggap tak terkunci
SLIDE_RANGE = 1.5       # sapuan ±sekian meter sepanjang arah lemah
SLIDE_STEP = 0.02
SLIDE_PEAKS = 3         # berapa puncak dicoba ulang lewat ICP
SLIDE_NMS = 0.25        # puncak lebih dekat dari ini dianggap sama (m)

ICP_SCALES = (0.60, 0.30, 0.15, 0.08)
ICP_ITER = 60
DEFAULT_ROUNDS = 2


def _o3d():
    return clomerge._o3d()


def to_o3d(xyz):
    return outmerge.to_o3d(xyz)


def apply_transform(xyz, T):
    return outmerge.apply_transform(xyz, T)


# ═══════════════════════════════════════════════════════════════════════════════
# Peta bidang dan titik menonjol
# ═══════════════════════════════════════════════════════════════════════════════

def plane_area(pts: np.ndarray, normal: np.ndarray,
               cell: float = PLANE_AREA_CELL) -> float:
    """Luas yang benar-benar TERISI oleh titik-titik ini di bidangnya, m².

    Dihitung dari sel yang terisi, bukan dari kotak pembatas: tembok berlubang
    pintu tetap terhitung luas, sedangkan enam huruf yang berjauhan hanya
    terhitung seluas hurufnya sendiri — bukan seluas rentangnya.
    """
    pts = np.asarray(pts, dtype=np.float64)
    if len(pts) < 3:
        return 0.0
    n = np.asarray(normal, dtype=np.float64)
    n = n / max(float(np.linalg.norm(n)), 1e-12)

    bantu = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(n, bantu)
    u /= max(float(np.linalg.norm(u)), 1e-12)
    v = np.cross(n, u)

    sel = np.floor(np.column_stack([pts @ u, pts @ v]) / cell).astype(np.int64)
    return float(len(np.unique(sel, axis=0)) * cell * cell)


def plane_atlas(xyz: np.ndarray, tol: float = PLANE_TOL,
                min_frac: float = PLANE_MIN_FRAC, maks: int = PLANE_MAX,
                min_area: float = PLANE_MIN_AREA) -> list:
    """Kupas bidang besar satu per satu. → daftar (a,b,c,d) LATAR, dinormalkan.

    Dikupas berulang, bukan sekali: tanah dan tembok sama-sama besar, dan yang
    dicari adalah SEMUANYA, supaya apa pun yang tersisa memang bukan latar.

    Bidang yang lolos jumlah titik tapi luasnya kecil tetap DIKUPAS — supaya
    pengupasan bisa lanjut ke bidang di bawahnya — tapi tidak dicatat sebagai
    latar. Itulah nasib bidang muka huruf, dan memang begitu seharusnya.

    Ambang berhentinya sengaja rendah dan jumlah kupasannya banyak. Tembok tidak
    selalu keluar lebih dulu: lantai sering pecah jadi beberapa bidang hampir
    sejajar (permukaan tidak rata sempurna), dan kalau pengupasan berhenti pada
    kupasan kecil yang pertama, temboknya tidak pernah sempat ditemukan — lalu
    seluruh tembok salah dianggap ciri.
    """
    o3d = _o3d()
    n0 = len(xyz)
    if n0 < 500:
        return []
    sisa = to_o3d(xyz)
    hasil = []
    for _ in range(int(maks)):
        if len(sisa.points) < max(500, min_frac * n0):
            break
        try:
            model, idx = sisa.segment_plane(tol, 3, 800)
        except (RuntimeError, ValueError):
            break
        if len(idx) < min_frac * n0:
            break
        m = np.asarray(model, dtype=np.float64)
        panjang = float(np.linalg.norm(m[:3]))
        if panjang < 1e-9:
            break
        m = m / panjang

        if plane_area(np.asarray(sisa.points)[idx], m[:3]) >= min_area:
            hasil.append(m)
        sisa = sisa.select_by_index(idx, invert=True)
    return hasil


def plane_distance(xyz: np.ndarray, atlas: list) -> np.ndarray:
    """Jarak tiap titik ke bidang TERDEKAT dalam atlas. Tanpa atlas → tak hingga."""
    xyz = np.asarray(xyz, dtype=np.float64)
    if not atlas:
        return np.full(len(xyz), np.inf)
    d = np.abs(np.column_stack([xyz @ m[:3] + m[3] for m in atlas]))
    return d.min(axis=1)


def salient_mask(xyz: np.ndarray, atlas: list,
                 tol: float = SALIENT_TOL) -> np.ndarray:
    """Titik yang tidak terletak di bidang besar mana pun — inilah cirinya."""
    return plane_distance(xyz, atlas) > tol


def salient_cloud(xyz: np.ndarray, atlas: list, tol: float = SALIENT_TOL,
                  voxel: float = SALIENT_VOXEL):
    """Awan titik menonjol saja, dijarangkan. Untuk penilaian tajam."""
    m = salient_mask(xyz, atlas, tol)
    if int(m.sum()) < SHARP_MIN_POINTS:
        return None
    return to_o3d(xyz[m]).voxel_down_sample(voxel)


# ═══════════════════════════════════════════════════════════════════════════════
# Pencuplikan berimbang menurut arah normal
# ═══════════════════════════════════════════════════════════════════════════════

def fold_normals(normals: np.ndarray, eps: float = 0.15) -> np.ndarray:
    """Satukan n dan -n jadi satu arah baku.

    Open3D tidak menjamin arah hadap normal konsisten — untuk permukaan yang
    sama, sebagian titik bisa menghadap ke satu sisi dan sebagian ke sisi lain.
    Kalau tandanya dibiarkan, satu tembok terpecah jadi dua petak dan
    pengimbangan malah memberinya jatah ganda.

    Tandanya ditentukan bertingkat: pakai komponen Z bila cukup besar, kalau
    tidak pakai Y, kalau tidak pakai X. Memakai Z saja tidak cukup — untuk
    tembok tegak Z-nya nyaris nol dan yang menentukan tinggal derau.
    """
    n = np.asarray(normals, dtype=np.float64)
    n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)

    tanda = np.where(np.abs(n[:, 2]) > eps, np.sign(n[:, 2]),
                     np.where(np.abs(n[:, 1]) > eps, np.sign(n[:, 1]),
                              np.sign(n[:, 0])))
    tanda[tanda == 0] = 1.0
    n = n * tanda[:, None]

    # Buang nol-negatif. Setelah dibalik tanda, komponen yang tadinya 0.0 jadi
    # -0.0, dan arctan2(-0.0, -0.0) bernilai -π sedangkan arctan2(0.0, 0.0)
    # bernilai 0 — normal tegak sempurna ke atas dan ke bawah akan jatuh ke
    # petak berbeda hanya gara-gara itu.
    return np.where(n == 0.0, 0.0, n)


def _azimuth_counts(bins: int) -> np.ndarray:
    """Berapa petak azimut untuk tiap pita elevasi, supaya petaknya setara luas.

    Pita dibagi rata menurut Z — pada bola, pita ber-ΔZ sama punya luas sama.
    Tapi pita di dekat puncak itu lingkaran kecil: kalau ia tetap dibelah 8
    azimut, petaknya jadi jauh lebih sempit daripada petak di khatulistiwa, dan
    permukaan MENDATAR (tanah — normalnya di puncak) menerima jatah berlipat
    hanya karena deraunya menyebar ke banyak petak. Karena itu jumlah azimut
    dibuat sebanding dengan keliling pitanya.
    """
    z = (np.arange(bins) + 0.5) / bins
    return np.maximum(1, np.rint(bins * np.sqrt(1.0 - z ** 2)).astype(int))


def normal_bin(normals: np.ndarray, bins: int = NORMAL_BINS) -> np.ndarray:
    """Nomor petak arah untuk tiap normal, petak-petaknya setara luas."""
    n = fold_normals(normals)

    jml = _azimuth_counts(bins)
    awal = np.concatenate([[0], np.cumsum(jml)[:-1]])

    iz = np.clip((np.abs(n[:, 2]) * bins).astype(int), 0, bins - 1)
    az = np.arctan2(n[:, 1], n[:, 0]) % (2 * np.pi)
    ia = (az / (2 * np.pi) * jml[iz]).astype(int) % jml[iz]
    return awal[iz] + ia


def balanced_sample(pcd, salien: np.ndarray, maks: int = BALANCED_MAX,
                    bins: int = NORMAL_BINS, share: float = SALIENT_SHARE):
    """Cuplik titik sumber berimbang per arah normal, titik menonjol diistimewakan.

    Inti perbaikannya ada di sini. Tembok bisa menyumbang 40% titik, semuanya
    dengan satu arah normal yang sama; sisi-samping huruf mungkin cuma 1%, tapi
    merekalah satu-satunya yang punya gradien MENYUSURI tembok. Kalau titik
    diambil apa adanya, Hessian ICP dikuasai tembok dan arah menyusuri tembok
    praktis tak terkunci.

    Dengan mengambil jatah yang sama dari tiap petak arah, permukaan langka itu
    naik pangkat sampai setara. Separuh jatah lagi dikhususkan untuk titik
    menonjol, karena sebuah huruf bisa saja arah normalnya sama dengan tembok
    (mukanya) — yang membedakannya bukan arah, tapi letaknya yang di depan.
    """
    o3d = _o3d()
    if not pcd.has_normals():
        return pcd
    total = len(pcd.points)
    if total <= maks:
        return pcd

    rng = np.random.default_rng(0)
    petak = normal_bin(np.asarray(pcd.normals), bins)
    salien = np.asarray(salien, dtype=bool)

    terpilih = []

    # jatah pertama: titik menonjol, sendiri sudah diimbangi per arah
    kuota_s = int(maks * share)
    idx_s = np.flatnonzero(salien)
    if len(idx_s):
        terpilih.append(_round_robin(idx_s, petak[idx_s], kuota_s, rng))

    # jatah kedua: seluruh titik, diimbangi per arah
    sudah = np.concatenate(terpilih) if terpilih else np.empty(0, dtype=int)
    sisa_mask = np.ones(total, dtype=bool)
    sisa_mask[sudah] = False
    idx_r = np.flatnonzero(sisa_mask)
    if len(idx_r):
        terpilih.append(_round_robin(idx_r, petak[idx_r], maks - len(sudah), rng))

    pilih = np.unique(np.concatenate(terpilih)) if terpilih else np.arange(total)
    return pcd.select_by_index(pilih.tolist())


def _round_robin(idx: np.ndarray, petak: np.ndarray, kuota: int,
                 rng) -> np.ndarray:
    """Ambil `kuota` indeks bergiliran dari tiap petak, sampai petaknya habis.

    Bergiliran, bukan kuota tetap per petak: petak yang isinya sedikit habis
    lebih dulu dan sisa jatahnya mengalir ke petak lain, jadi tidak ada jatah
    yang terbuang dan totalnya tetap tercapai.
    """
    kuota = int(max(0, kuota))
    if kuota == 0 or len(idx) == 0:
        return np.empty(0, dtype=int)
    if len(idx) <= kuota:
        return idx

    urut = rng.permutation(len(idx))
    idx, petak = idx[urut], petak[urut]

    ember = {}
    for i, p in zip(idx, petak):
        ember.setdefault(int(p), []).append(int(i))

    keluar = []
    daftar = list(ember.values())
    k = 0
    while len(keluar) < kuota and daftar:
        masih = []
        for e in daftar:
            if k < len(e):
                keluar.append(e[k])
                masih.append(e)
                if len(keluar) >= kuota:
                    break
        daftar = masih
        k += 1
    return np.asarray(keluar, dtype=int)


# ═══════════════════════════════════════════════════════════════════════════════
# Piramida ICP
# ═══════════════════════════════════════════════════════════════════════════════

def pyramid(xyz: np.ndarray, atlas: list = None, scales=ICP_SCALES) -> list:
    """Untuk tiap tingkat ICP: awan acuan lengkap + cuplikan sumber berimbang.

    Sebuah awan bisa berperan sebagai sumber maupun acuan, jadi keduanya
    disiapkan sekaligus dan dipakai ulang untuk semua tebakan awal — peta
    gabungan bisa ratusan ribu titik dan menjarangkannya berulang jauh lebih
    mahal daripada ICP-nya sendiri.
    """
    o3d = _o3d()
    if atlas is None:
        atlas = plane_atlas(xyz)
    p = to_o3d(xyz)
    tingkat = []
    for s in scales:
        d = p.voxel_down_sample(s)
        d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=s * 3.0, max_nn=30))
        salien = salient_mask(np.asarray(d.points), atlas)
        tingkat.append({"acuan": d, "sumber": balanced_sample(d, salien)})
    return tingkat


def icp_multi(src_pyr: list, ref_pyr: list, T) -> np.ndarray:
    """ICP point-to-plane kasar → halus; sumbernya cuplikan berimbang."""
    o3d = _o3d()
    T = np.asarray(T, dtype=np.float64)
    for a, b, s in zip(src_pyr, ref_pyr, ICP_SCALES):
        T = o3d.pipelines.registration.registration_icp(
            a["sumber"], b["acuan"], s * 2.0, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_ITER),
        ).transformation
    return np.asarray(T)


# ═══════════════════════════════════════════════════════════════════════════════
# Penilaian
# ═══════════════════════════════════════════════════════════════════════════════

def ref_cloud(xyz: np.ndarray, voxel: float = REF_VOXEL):
    """Acuan penilaian tajam: seluruh permukaan, dijarangkan halus.

    Sengaja SELURUH titik, bukan yang menonjol saja. Pertanyaan yang diajukan
    adalah "apakah ciri sumber mendarat tepat di permukaan acuan" — huruf yang
    meleset akan mendarat di tembok polos, dan tembok itu harus ada di acuan
    supaya jaraknya terukur sebagai setinggi hurufnya, bukan sebagai kosong.
    """
    return to_o3d(xyz).voxel_down_sample(voxel)


def sharp_score(src_sal, ref_pc, T, dist: float = SHARP_DIST) -> tuple:
    """Fitness+rmse titik menonjol sumber terhadap permukaan acuan, toleransi ketat."""
    o3d = _o3d()
    if src_sal is None or ref_pc is None:
        return 0.0, 0.0
    r = o3d.pipelines.registration.evaluate_registration(
        src_sal, ref_pc, float(dist), np.asarray(T, dtype=np.float64))
    return float(r.fitness), float(r.inlier_rmse)


def verdict(tajam: float, rmse: float) -> str:
    """Vonis berdasar fitness TAJAM — bukan yang longgar.

    Fitness longgar tinggi hanya berarti "menempel di suatu permukaan"; ia bisa
    0,5 sekalipun hasilnya meleset semeter menyusuri tembok. Yang dipakai
    memutuskan karena itu yang tajam.
    """
    if tajam >= TAJAM_BAIK and rmse <= SHARP_DIST * 0.7:
        return BAIK
    if tajam >= TAJAM_RAGU:
        return RAGU
    return GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# Arah lemah (analisis Hessian point-to-plane)
# ═══════════════════════════════════════════════════════════════════════════════

def weak_direction(src_pyr: list, ref_pyr: list, T, tingkat: int = -1) -> dict:
    """Arah gerak yang PALING TIDAK terkunci oleh geometri, pada pose T.

    Untuk tiap pasangan titik, ICP point-to-plane menghukum simpangan sepanjang
    normal acuan saja. Barisan Jacobian-nya [p×n, n]; H = ΣJᵀJ. Nilai eigen
    terkecil H menunjukkan arah gerak yang nyaris tidak menaikkan galat sama
    sekali — persis arah yang bisa menggelincirkan hasil tanpa ketahuan.

    Bagian putaran diskalakan dengan jari-jari khas awan supaya sebanding
    satuannya dengan bagian geseran; tanpa itu nilai eigennya tidak bisa
    dibandingkan dan "arah terlemah" jadi tak bermakna.
    """
    o3d = _o3d()
    a, b = src_pyr[tingkat], ref_pyr[tingkat]
    skala = ICP_SCALES[tingkat]

    src = o3d.geometry.PointCloud(a["sumber"]).transform(np.asarray(T, dtype=np.float64))
    P = np.asarray(src.points)
    if len(P) < 50 or not b["acuan"].has_normals():
        return {"ratio": 1.0, "arah": np.zeros(3), "putar": np.zeros(3), "n": 0}

    Q = np.asarray(b["acuan"].points)
    N = np.asarray(b["acuan"].normals)
    pohon = o3d.geometry.KDTreeFlann(b["acuan"])

    jr, jn = [], []
    batas = skala * 2.0
    for p in P:
        ok, idx, d2 = pohon.search_knn_vector_3d(p, 1)
        if not ok or d2[0] > batas ** 2:
            continue
        n = N[idx[0]]
        jr.append(np.cross(p, n))
        jn.append(n)
    if len(jn) < 50:
        return {"ratio": 1.0, "arah": np.zeros(3), "putar": np.zeros(3),
                "n": len(jn)}

    jari = float(np.sqrt(np.mean(np.sum((P - P.mean(axis=0)) ** 2, axis=1))))
    jari = max(jari, 1e-3)
    J = np.hstack([np.asarray(jr) / jari, np.asarray(jn)])
    H = J.T @ J / len(J)

    nilai, vektor = np.linalg.eigh(H)
    v = vektor[:, 0]
    return {
        "ratio": float(nilai[0] / max(nilai[-1], 1e-12)),
        "putar": v[:3] / jari,
        "arah": v[3:],
        "n": len(J),
    }


def describe_weak(w: dict) -> str:
    """Terjemahkan arah lemah jadi kalimat yang bisa dibaca."""
    if w["n"] == 0:
        return "tak terhitung"
    a, r = np.asarray(w["arah"]), np.asarray(w["putar"])
    if np.linalg.norm(a) < np.linalg.norm(r):
        return f"putaran (rasio {w['ratio']:.4f})"
    a = a / max(np.linalg.norm(a), 1e-12)
    jenis = "mendatar" if abs(a[2]) < 0.4 else "tegak"
    return (f"geser {jenis} ({a[0]:+.2f}, {a[1]:+.2f}, {a[2]:+.2f}) "
            f"rasio {w['ratio']:.4f}")


# ═══════════════════════════════════════════════════════════════════════════════
# Pelepasan geseran
# ═══════════════════════════════════════════════════════════════════════════════

def slide_profile(src_sal, ref_pc, T, arah: np.ndarray, jangkauan: float,
                  langkah: float, dist: float) -> tuple:
    """Sapu satu dimensi sepanjang `arah`. → (offset, fitness tajam tiap offset)."""
    arah = np.asarray(arah, dtype=np.float64)
    panjang = float(np.linalg.norm(arah))
    if panjang < 1e-9:
        return np.zeros(0), np.zeros(0)
    arah = arah / panjang

    offs = np.arange(-jangkauan, jangkauan + 1e-9, langkah)
    nilai = np.empty(len(offs))
    T = np.asarray(T, dtype=np.float64)
    for i, d in enumerate(offs):
        Td = T.copy()
        Td[:3, 3] = Td[:3, 3] + arah * d
        nilai[i], _ = sharp_score(src_sal, ref_pc, Td, dist)
    return offs, nilai


def pick_peaks(offs: np.ndarray, nilai: np.ndarray, berapa: int,
               nms: float) -> list:
    """Puncak-puncak teratas profil, yang berdekatan dianggap satu."""
    if len(offs) == 0:
        return []
    urut = np.argsort(-nilai)
    simpan = []
    for i in urut:
        if all(abs(offs[i] - o) > nms for o in simpan):
            simpan.append(float(offs[i]))
        if len(simpan) >= int(berapa):
            break
    return simpan


def slide_directions(atlas: list, w: dict, weak_ratio: float) -> list:
    """Arah-arah mendatar yang layak disapu. → daftar vektor satuan.

    Arah utamanya diambil dari TANGEN TEMBOK, bukan dari Hessian. Ini penting,
    dan dipelajari dari kegagalan: setelah titik dicuplik berimbang, tanah tidak
    lagi mendominasi, sehingga arah "terlemah" menurut Hessian sering menunjuk
    ke atas — padahal tinggi sudah terkunci oleh perataan tanah, dan penyakit
    yang dicari justru geseran MENDATAR menyusuri tembok. Tangen tembok
    menunjuk tepat ke sana, tanpa perlu ditebak.

    Arah lemah menurut Hessian tetap ikut sebagai calon tambahan bila memang
    mendatar dan memang lemah — kadang yang menggelincirkan bukan tembok yang
    terbesar.
    """
    calon = []

    def tambah(v):
        v = np.asarray(v, dtype=np.float64)
        v[2] = 0.0                                  # mendatar saja
        p = float(np.linalg.norm(v))
        if p < 1e-6:
            return
        v = v / p
        for u in calon:
            if abs(float(np.dot(u, v))) > 0.98:     # sudah ada yang sejajar
                return
        calon.append(v)

    for m in atlas:
        if abs(m[2]) < 0.4:                         # bidang TEGAK = tembok
            tambah([-m[1], m[0], 0.0])

    if w.get("n", 0) > 0 and w.get("ratio", 1.0) <= weak_ratio:
        tambah(w["arah"])

    return calon


def unslide(src_pyr, ref_pyr, src_sal, ref_pc, T, args, atlas=None) -> tuple:
    """Betulkan geseran menyusuri tembok. → (T terbaik, catatan).

    Tiap arah calon disapu satu dimensi dengan penilaian tajam, puncak-puncak
    teratasnya dijadikan titik awal ICP, lalu yang skornya paling tinggi yang
    menang. Pose semula SELALU ikut dilombakan, jadi jawaban yang sudah benar
    tidak mungkin dirusak — ia hanya kalah kalau ada yang benar-benar lebih
    baik menurut takaran yang sama.
    """
    w = weak_direction(src_pyr, ref_pyr, T)
    catatan = {"lemah": w, "geser": 0.0, "disapu": False, "arah": None}

    arah_calon = slide_directions(atlas or [], w, args.weak_ratio)
    if not arah_calon:
        return np.asarray(T), catatan

    T = np.asarray(T, dtype=np.float64)
    calon = [(T, 0.0, None)]
    for arah in arah_calon:
        offs, nilai = slide_profile(src_sal, ref_pc, T, arah, args.slide_range,
                                    SLIDE_STEP, args.sharp)
        if len(offs) == 0:
            continue
        catatan["disapu"] = True
        for d in pick_peaks(offs, nilai, SLIDE_PEAKS, SLIDE_NMS):
            Td = T.copy()
            Td[:3, 3] = Td[:3, 3] + arah * d
            calon.append((Td, d, arah))

    terbaik = None
    for Td, d, arah in calon:
        Tf = icp_multi(src_pyr, ref_pyr, Td)
        f, r = sharp_score(src_sal, ref_pc, Tf, args.sharp)
        if terbaik is None or f > terbaik[0]:
            terbaik = (f, Tf, d, arah)
    catatan["geser"] = float(terbaik[2])
    catatan["arah"] = None if terbaik[3] is None else np.asarray(terbaik[3])
    return terbaik[1], catatan


# ═══════════════════════════════════════════════════════════════════════════════
# Pencocokan satu scan ke satu acuan
# ═══════════════════════════════════════════════════════════════════════════════

def register(src_xyz, ref_xyz, src_sal, ref_pc, args,
             src_pyr, ref_pyr, ref_atlas=None, verbose=True) -> dict:
    """Sapu sudut (citra tampak-atas) → ICP berimbang → lepaskan geseran → nilai tajam.

    Sapuan sudutnya meminjam outmerge apa adanya: korelasi fase pada citra
    tampak-atas sudah bekerja baik untuk menemukan PUTARAN, dan putaran bukan
    derajat kebebasan yang bermasalah di sini.
    """
    seeds = outmerge.bev_seeds(src_xyz, ref_xyz, args.step_deg, args.seeds)

    hasil = []
    for pk, deg, tx, ty in seeds:
        T = icp_multi(src_pyr, ref_pyr, outmerge.seed_matrix(deg, tx, ty))
        f, r = sharp_score(src_sal, ref_pc, T, args.sharp)
        hasil.append({"T": T, "tajam": f, "rmse": r, "seed": deg, "peak": pk})

    hasil.sort(key=lambda h: -h["tajam"])
    terbaik = dict(hasil[0])

    if not args.no_unslide:
        T2, cat = unslide(src_pyr, ref_pyr, src_sal, ref_pc, terbaik["T"], args,
                          ref_atlas)
        f2, r2 = sharp_score(src_sal, ref_pc, T2, args.sharp)
        if f2 >= terbaik["tajam"]:
            terbaik.update(T=T2, tajam=f2, rmse=r2)
        terbaik["unslide"] = cat
    else:
        terbaik["unslide"] = {"lemah": weak_direction(src_pyr, ref_pyr, terbaik["T"]),
                              "geser": 0.0, "disapu": False, "arah": None}

    # margin: selisih terhadap jawaban BEDA terbaik berikutnya. Kecil berarti
    # ada tafsir lain yang sama masuk akalnya.
    margin = terbaik["tajam"]
    y0 = outmerge.yaw_of(terbaik["T"])
    t0 = np.asarray(terbaik["T"])[:3, 3]
    for h in hasil[1:]:
        dy = abs(outmerge.yaw_of(h["T"]) - y0)
        dy = min(dy, 360.0 - dy)
        if dy > 5.0 or np.linalg.norm(np.asarray(h["T"])[:3, 3] - t0) > 0.5:
            margin = terbaik["tajam"] - h["tajam"]
            break
    terbaik["margin"] = float(margin)

    # Titik yang SAMA, dinilai dengan toleransi lama 10 cm. Bukan untuk
    # memutuskan apa pun — gunanya memperlihatkan jenuhnya takaran lama: kalau
    # longgar tinggi sementara tajam rendah, hasilnya memang cuma menempel di
    # permukaan, bukan berada di tempat yang benar.
    terbaik["longgar"], _ = sharp_score(src_sal, ref_pc, terbaik["T"], 0.10)
    terbaik["verdict"] = verdict(terbaik["tajam"], terbaik["rmse"])

    if verbose:
        u = terbaik["unslide"]
        geser = (f"  unslide {u['geser']:+.2f} m" if u["disapu"] else "")
        if u.get("arah") is not None:
            a = u["arah"]
            geser += f" arah ({a[0]:+.2f},{a[1]:+.2f})"
        print(f"      yaw {y0:6.1f}°  geser ({t0[0]:5.2f}, {t0[1]:5.2f}, {t0[2]:5.2f}) m"
              f"   tajam {terbaik['tajam']:.4f}  rmse {terbaik['rmse']:.4f} m"
              f"  margin {terbaik['margin']:+.4f}{geser}   → {terbaik['verdict']}")
        print(f"      arah terlemah: {describe_weak(u['lemah'])}")
    return terbaik


# ═══════════════════════════════════════════════════════════════════════════════
# Peta yang tumbuh
# ═══════════════════════════════════════════════════════════════════════════════

def pick_seed_pair(clouds, sal, refs, names, pyr, atlases, args) -> tuple:
    """Dua scan yang paling meyakinkan satu sama lain → titik mula peta."""
    kunci = sorted(clouds)
    terbaik = None
    for a in kunci:
        for b in kunci:
            if a >= b:
                continue
            print(f"    {names[a]} → {names[b]}")
            h = register(clouds[a], clouds[b], sal[a], refs[b], args,
                         pyr[a], pyr[b], atlases[b])
            nilai = (h["tajam"], h["margin"])
            if terbaik is None or nilai > terbaik[0]:
                terbaik = (nilai, a, b, h)
    _, a, b, h = terbaik
    return a, b, h


def grow_map(clouds, sal, refs, names, pyr, atlases, args) -> dict:
    """Mulai dari sambungan terkuat, lalu tambah satu per satu yang paling yakin."""
    kunci = sorted(clouds)
    print("\n  Mencari pasangan pembuka:")
    a, b, h = pick_seed_pair(clouds, sal, refs, names, pyr, atlases, args)

    poses = {b: np.eye(4), a: np.asarray(h["T"])}
    catatan = {b: {"acuan": True, "verdict": BAIK, "tajam": None, "longgar": None,
                   "rmse": None, "margin": None, "lawan": "—",
                   "unslide": {"lemah": {"n": 0, "ratio": 1.0}, "geser": 0.0,
                               "disapu": False}},
               a: dict(h, lawan=names[b])}
    catatan[b]["unslide"] = {"lemah": {"n": 0, "ratio": 1.0}, "geser": 0.0,
                             "disapu": False, "arah": None}
    print(f"\n  Pasangan pembuka : {names[a]} → {names[b]}  "
          f"(tajam {h['tajam']:.4f}, margin {h['margin']:+.4f})")

    peta = np.vstack([clouds[b], apply_transform(clouds[a], poses[a])])
    sisa = [i for i in kunci if i not in poses]

    while sisa:
        atlas = plane_atlas(peta)
        peta_pyr = pyramid(peta, atlas)
        peta_ref = ref_cloud(peta)
        print(f"\n  Peta sekarang {len(poses)} scan, {len(peta):,} titik. "
              f"Mencoba {len(sisa)} sisanya:")

        calon = []
        for i in sisa:
            print(f"    {names[i]} → peta")
            h = register(clouds[i], peta, sal[i], peta_ref, args,
                         pyr[i], peta_pyr, atlas)
            calon.append((h["tajam"], h["margin"], i, h))

        calon.sort(key=lambda c: (-c[0], -c[1]))
        _, _, i, h = calon[0]
        poses[i] = np.asarray(h["T"])
        catatan[i] = dict(h, lawan=f"peta {len(poses) - 1} scan")
        print(f"    ✔ dipasang: {names[i]}  (tajam {h['tajam']:.4f}, {h['verdict']})")

        peta = np.vstack([peta, apply_transform(clouds[i], poses[i])])
        sisa.remove(i)

    return {"poses": poses, "catatan": catatan}


def refine_all(clouds, sal, names, poses, catatan, pyr, args) -> None:
    """Cocokkan ulang tiap scan ke peta dari SEMUA scan lain, beberapa putaran.

    Termasuk pelepasan geseran: scan yang dipasang lebih awal belum pernah
    melihat yang datang belakangan, dan huruf yang tadinya tak terjangkau bisa
    baru sekarang muncul di acuan.
    """
    kunci = sorted(poses)
    if len(kunci) < 3:
        return

    for putaran in range(1, int(args.rounds) + 1):
        print(f"\n  Perapian putaran {putaran}/{args.rounds}:")
        naik = 0
        for i in kunci:
            lain = np.vstack([apply_transform(clouds[j], poses[j])
                              for j in kunci if j != i])
            lain_atlas = plane_atlas(lain)
            lain_pyr = pyramid(lain, lain_atlas)
            lain_ref = ref_cloud(lain)

            f0, r0 = sharp_score(sal[i], lain_ref, poses[i], args.sharp)
            T = icp_multi(pyr[i], lain_pyr, poses[i])
            if not args.no_unslide:
                T, cat = unslide(pyr[i], lain_pyr, sal[i], lain_ref, T, args,
                                 lain_atlas)
                catatan[i]["unslide"] = cat
            f1, r1 = sharp_score(sal[i], lain_ref, T, args.sharp)

            catatan[i]["lawan"] = f"peta {len(kunci) - 1} scan lain"
            catatan[i].pop("acuan", None)

            tanda = " "
            if f1 > f0:
                poses[i], f0, r0 = T, f1, r1
                naik += 1
                tanda = "✔"
            catatan[i].update(tajam=f0, rmse=r0, verdict=verdict(f0, r0))
            catatan[i]["longgar"] = sharp_score(sal[i], lain_ref, poses[i], 0.10)[0]
            print(f"    {tanda} {names[i]:<40} tajam {f0:.4f} → {f1:.4f}"
                  f"   rmse {r0:.4f} m")
        if naik == 0:
            print("      tidak ada yang membaik — berhenti lebih awal")
            break


# ═══════════════════════════════════════════════════════════════════════════════
# Laporan
# ═══════════════════════════════════════════════════════════════════════════════

def write_report(path, urut, names, poses, catatan, levels, args) -> None:
    b = ["# clomerged — penggabungan sadar fitur",
         f"# Jangkauan dipotong : {args.range if args.range else 'tidak'} m",
         f"# Sapuan sudut       : tiap {args.step_deg}°, {args.seeds} puncak teratas",
         f"# Penilaian tajam    : titik menonjol (>{SALIENT_TOL} m dari bidang besar), "
         f"toleransi {args.sharp} m",
         f"# Pelepasan geseran  : "
         + ("dimatikan" if args.no_unslide
            else f"±{args.slide_range} m bila rasio arah lemah < {args.weak_ratio}"),
         "#",
         "# tajam   = bagian titik MENONJOL yang mendarat tepat di permukaan acuan.",
         "#           Inilah yang menentukan vonis. Huruf/tiang yang meleset langsung",
         f"#           terhukum karena toleransinya cuma {args.sharp} m.",
         "# longgar = takaran lama (10 cm, semua permukaan). Dicantumkan hanya supaya",
         "#           sebanding dengan run outmerge terdahulu — TIDAK memutuskan apa pun.",
         "#           longgar tinggi + tajam rendah = menempel di tembok, salah tempat.",
         "# lemah   = arah gerak yang paling tidak terkunci geometri, dan rasionya.",
         "#           Rasio kecil berarti hasil bisa tergelincir ke arah itu tanpa",
         "#           menaikkan galat — di situlah 'FILFILKOM' lahir.",
         "# unslide = berapa meter hasil digeser sepanjang arah lemah itu untuk",
         "#           membetulkannya.",
         "",
         "# Ringkasan:"]
    for i in urut:
        c = catatan[i]
        f = "—" if c.get("tajam") is None else f"{c['tajam']:.4f}"
        lg = "—" if c.get("longgar") is None else f"{c['longgar']:.4f}"
        r = "—" if c.get("rmse") is None else f"{c['rmse']:.4f}"
        m = "—" if c.get("margin") is None else f"{c['margin']:+.4f}"
        b.append(f"#   {names[i]:<40} tajam {f}  longgar {lg}  rmse {r} m  "
                 f"margin {m}  {c['verdict']}   [ke {c['lawan']}]")
    b.append("")

    b.append("# Arah terlemah menurut Hessian, dan koreksi geseran yang dilakukan.")
    b.append("# Catatan: arah yang DISAPU diambil dari tangen tembok, bukan dari")
    b.append("# kolom 'terlemah' ini — kolom itu hanya laporan kondisi geometri.")
    for i in urut:
        u = catatan[i].get("unslide") or {}
        if u.get("disapu"):
            a = u.get("arah")
            arah = ("" if a is None
                    else f" sepanjang ({a[0]:+.2f}, {a[1]:+.2f})")
            g = f"digeser {u['geser']:+.3f} m{arah}"
        else:
            g = "tidak disapu (tak ada tembok tegak sebagai acuan arah)"
        b.append(f"#   {names[i]:<40} terlemah: "
                 f"{describe_weak(u.get('lemah', {'n': 0}))}   → {g}")
    b.append("")

    b.append("# Perataan tanah tiap scan (sudah termasuk di matriks gabungan):")
    for i in urut:
        L = levels.get(i)
        b.append(f"#   {names[i]:<40} "
                 + ("tanah tidak ditemukan — dibiarkan apa adanya"
                    if L is None else f"{outmerge.tilt_deg(L):.2f}° dikoreksi"))
    b.append("")

    b.append("# Matriks gabungan tiap scan ke kerangka acuan.")
    b.append("# Sudah termasuk perataan tanah; pakai langsung pada scan asli.")
    b.append("")
    for i in urut:
        L = np.eye(4) if levels.get(i) is None else np.asarray(levels[i])
        M = np.asarray(poses[i]) @ L
        b.append(f"{names[i]}")
        b.append(f"  status  : {catatan[i]['verdict']}")
        b.append(f"  yaw     : {outmerge.yaw_of(M):.3f}°")
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
        prog="clomerged",
        description="Gabung scan di tempat bertembok polos: penilaian tajam pada "
                    "titik menonjol, pencuplikan berimbang per arah normal, dan "
                    "pelepasan geseran di arah yang tidak terkunci.")
    ap.add_argument("files", nargs="+",
                    help="dua atau lebih file .mcap/.mcap.zstd/.ply/.xyz")
    ap.add_argument("--range", type=float, default=outmerge.DEFAULT_RANGE,
                    help=f"potong tiap scan pada jari-jari ini, meter "
                         f"(default {outmerge.DEFAULT_RANGE}; 0 = jangan potong)")
    ap.add_argument("--step-deg", type=float, default=outmerge.DEFAULT_STEP_DEG,
                    dest="step_deg",
                    help=f"kerapatan sapuan sudut, derajat "
                         f"(default {outmerge.DEFAULT_STEP_DEG})")
    ap.add_argument("--seeds", type=int, default=outmerge.DEFAULT_SEEDS,
                    help=f"berapa puncak sudut teratas dilanjutkan ke ICP "
                         f"(default {outmerge.DEFAULT_SEEDS})")
    ap.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS,
                    help=f"putaran perapian setelah semua terpasang "
                         f"(default {DEFAULT_ROUNDS}; 0 = lewati)")
    ap.add_argument("--sharp", type=float, default=SHARP_DIST,
                    help=f"toleransi penilaian tajam, meter (default {SHARP_DIST}); "
                         "perkecil bila tulisan masih dobel, perbesar bila "
                         "semuanya divonis GAGAL padahal terlihat benar")
    ap.add_argument("--salient-tol", type=float, default=SALIENT_TOL,
                    dest="salient_tol",
                    help=f"titik dianggap menonjol bila sejauh ini dari bidang "
                         f"besar terdekat, meter (default {SALIENT_TOL}); "
                         "perkecil bila cirinya tipis (tulisan cat/ukiran dangkal)")
    ap.add_argument("--slide-range", type=float, default=SLIDE_RANGE,
                    dest="slide_range",
                    help=f"sapuan pelepasan geseran ±sekian meter "
                         f"(default {SLIDE_RANGE})")
    ap.add_argument("--weak-ratio", type=float, default=WEAK_RATIO,
                    dest="weak_ratio",
                    help=f"arah disapu hanya bila rasio kekuatannya di bawah ini "
                         f"(default {WEAK_RATIO})")
    ap.add_argument("--no-unslide", action="store_true", dest="no_unslide",
                    help="jangan lepaskan geseran; pakai hasil ICP apa adanya")
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

    print("=" * 62)
    print(f"  Masukan       : {len(srcs)} berkas")
    print(f"  Jangkauan     : {args.range if args.range else 'utuh'} m")
    print(f"  Sapuan sudut  : tiap {args.step_deg}°, {args.seeds} puncak teratas")
    print(f"  Penilaian     : titik menonjol >{args.salient_tol} m dari bidang, "
          f"toleransi {args.sharp} m")
    print(f"  Lepas geseran : "
          + ("dimatikan" if args.no_unslide else f"±{args.slide_range} m"))
    print("=" * 62)

    clouds, sal, refs, levels, names, pyr, atlases = {}, {}, {}, {}, {}, {}, {}
    for k, src in enumerate(srcs):
        print(f"\n[{k + 1}/{len(srcs)}] {src}")
        try:
            p = clomcaps.prepare_cloud(src, args)
        except (Exception, SystemExit) as e:  # noqa: BLE001
            print(f"  [GAGAL] Dilewati: {e}")
            continue

        xyz = clomerge.read_cloud_xyz(p)
        n0 = len(xyz)
        xyz = outmerge.crop_range(xyz, args.range)
        if len(xyz) < 1000:
            print(f"  [GAGAL] Dilewati: tinggal {len(xyz)} titik setelah dipotong")
            continue

        L = outmerge.level_transform(xyz)
        if L is None:
            print("  [WARN] Tanah tidak ditemukan — scan tidak diratakan. "
                  "Kemungkinan besar tidak akan terpasang.")
        else:
            xyz = apply_transform(xyz, L)

        atlas = plane_atlas(xyz)
        s = salient_cloud(xyz, atlas, args.salient_tol)
        if s is None:
            print("  [WARN] Nyaris tak ada titik menonjol — tempat ini terlalu "
                  "polos.\n         Penilaian tajam tidak akan bermakna di sini.")
        n_menonjol = 0 if s is None else len(s.points)
        print(f"  ✔ tanah diratakan {outmerge.tilt_deg(L):.2f}°, "
              f"{n0:,} → {len(xyz):,} titik, {len(atlas)} bidang besar, "
              f"{n_menonjol:,} titik menonjol")

        i = len(clouds)
        clouds[i], levels[i], names[i] = xyz, L, os.path.basename(p)
        sal[i], refs[i], atlases[i] = s, ref_cloud(xyz), atlas
        pyr[i] = pyramid(xyz, atlas)

    if len(clouds) < 2:
        raise SystemExit("\n[ERROR] Kurang dari dua berkas yang berhasil disiapkan.")

    print("\n" + "-" * 62)
    print("  Membangun peta")
    print("-" * 62)
    hasil = grow_map(clouds, sal, refs, names, pyr, atlases, args)
    poses, catatan = hasil["poses"], hasil["catatan"]

    if args.rounds > 0:
        refine_all(clouds, sal, names, poses, catatan, pyr, args)

    urut = sorted(poses)
    bagian = [apply_transform(clouds[i], poses[i]) for i in urut]
    xyz = np.vstack(bagian)
    counts = [len(a) for a in bagian]

    L = outmerge.level_transform(xyz)
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

    print("\n" + "=" * 62)
    print(f"  Total titik : {len(xyz):,}")
    for k, i in enumerate(urut):
        c = catatan[i]
        f = "acuan" if c.get("tajam") is None else f"tajam {c['tajam']:.4f}"
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
        print("            Buka merged_check.ply dan lihat tulisan/tiangnya.")
        print("            Masih dobel? Coba --sharp 0.02 atau --slide-range 2.5.")
        print("            Semua GAGAL padahal terlihat benar? --sharp 0.05.")
    print("=" * 62)

    if not args.no_open:
        print(f"  Membuka CloudCompare … ({len(files)} berkas)")
        launch_cloudcompare(files)


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
