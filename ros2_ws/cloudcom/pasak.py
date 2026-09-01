#!/usr/bin/env python3
"""pasak — registrasi berjangkar untuk tempat minim fitur.

Perintah kelima. `clomerge`, `outmerge`, dan `clomerged` semuanya MENCARI pose
terbaik; di tempat bertembok polos pencarian itu selalu bisa tergelincir
menyusuri tembok tanpa nilainya turun. `pasak` tidak mencari — ia MENYELESAIKAN.

Pembagian derajat kebebasannya:

    roll, pitch, z  ← bidang tanah
    yaw, x, y       ← dua benda yang sama, ditunjuk manusia

Dua jangkar memberi empat batasan untuk tiga anu: tertentu penuh, dengan satu
sisa yang bisa diperiksa. Tanpa pencarian tidak ada minimum lokal dan tidak ada
alias periodik — dua jebakan yang tercatat menjatuhkan pendahulunya.

Manusia hanya menentukan BENDA MANA berpasangan dengan benda mana. Posisi
tepatnya tidak diambil dari kliknya: klik memilih lembah, ICP yang meratakan
dasarnya.

Rancangan lengkap beserta angka diagnosisnya:
    docs/superpowers/specs/2026-08-23-pasak-design.md
"""

import math
import numpy as np


# ═══════════════════════════════════════════════════════════════════════════════
# Jangkar — yaw, x, y dari pasangan benda
# ═══════════════════════════════════════════════════════════════════════════════

def yaw_derajat(T) -> float:
    """Sudut putar tegak sebuah matriks 4x4, dalam derajat."""
    T = np.asarray(T, dtype=np.float64)
    return float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))


def kabsch2d(P, Q) -> np.ndarray:
    """4x4 yang membawa jangkar Q ke jangkar P. Putaran tegak + geser mendatar.

    Sengaja hanya mendatar. Tanah sudah mengunci roll, pitch, dan z sebelum ini;
    membiarkan Kabsch ikut mengurusnya berarti membiarkan galat titik-pusat
    jangkar — yang bias ke arah sensor sebesar separuh tebal benda — merusak
    tiga derajat kebebasan yang sudah benar.
    """
    P = np.asarray(P, dtype=np.float64)
    Q = np.asarray(Q, dtype=np.float64)
    if P.ndim != 2 or Q.ndim != 2 or len(P) != len(Q):
        raise ValueError("jangkar P dan Q harus sama banyak")
    if len(P) < 2:
        raise ValueError("perlu minimal 2 jangkar; dengan 1 pakai jalur tembok")

    p, q = P[:, :2], Q[:, :2]
    pc, qc = p.mean(axis=0), q.mean(axis=0)

    H = (q - qc).T @ (p - pc)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    d = float(np.sign(np.linalg.det(V @ U.T))) or 1.0
    R = V @ np.diag([1.0, d]) @ U.T
    t = pc - R @ qc

    T = np.eye(4)
    T[:2, :2] = R
    T[:2, 3] = t
    return T


def sisa_jangkar(P, Q, rinci: bool = False):
    """Seberapa tidak sepadan pasangan jangkarnya. Meter.

    Dengan DUA jangkar sisanya cuma satu angka dan bentuknya khusus: selisih
    jarak antar-jangkar. Empat batasan untuk tiga anu menyisakan tepat satu
    derajat, dan itulah derajatnya. Murah, dan langsung menangkap salah tunjuk.

    Dengan TIGA atau lebih, tiap jangkar punya sisanya sendiri, jadi penunjuk
    yang salah bisa disebut namanya.
    """
    P = np.asarray(P, dtype=np.float64)
    Q = np.asarray(Q, dtype=np.float64)

    if len(P) < 2:
        # Satu jangkar tidak menyisakan apa pun untuk diperiksa. Nol di sini
        # berarti "tak ada bukti", bukan "terbukti benar" — yang menjaga kasus
        # ini adalah peringatan wajib bahwa yaw-nya dipinjam dari tembok.
        return (0.0, np.zeros(len(P))) if rinci else 0.0

    if len(P) == 2:
        beda = abs(float(np.linalg.norm(P[0, :2] - P[1, :2]))
                   - float(np.linalg.norm(Q[0, :2] - Q[1, :2])))
        return (beda, np.array([beda / 2.0, beda / 2.0])) if rinci else beda

    T = kabsch2d(P, Q)
    Qt = (T[:2, :2] @ Q[:, :2].T).T + T[:2, 3]
    per = np.linalg.norm(Qt - P[:, :2], axis=1)
    sisa = float(per.max())
    return (sisa, per) if rinci else sisa


# ═══════════════════════════════════════════════════════════════════════════════
# Tetapan
# ═══════════════════════════════════════════════════════════════════════════════

EGO_RADIUS = 0.7        # jari-jari silinder buang-rig (m)
GROUND_CROP = 6.0       # tanah SELALU dicari dari potongan ini, apa pun --range

BENDA_VOXEL = 0.03
BENDA_EPS = 0.12        # jarak sambung DBSCAN (m)
BENDA_MIN_TETANGGA = 10
BENDA_MIN_TITIK = 60
BENDA_MAX_TAPAK = 1.5   # tapak mendatar lebih besar dari ini = sisa tembok
BENDA_MAX_TINGGI = 2.5


def _o3d():
    import open3d as o3d
    return o3d


BENIH_TETAP = 0         # benih yang dipasang ulang sebelum tiap pemasangan bidang


def seed(n: int = 0) -> None:
    """Satu tuas untuk semua sumber acak. Tanpa ini angkanya berubah tiap jalan."""
    _o3d().utility.random.seed(int(n))
    np.random.seed(int(n))


def _benih_tetap() -> None:
    """Pasang ulang benih sebelum tiap pemasangan bidang.

    RANSAC di Open3D menarik jumlah undian yang BERUBAH-UBAH walau jawabannya
    sama, jadi keadaan RNG global sesudahnya tidak bisa diramalkan — dan
    panggilan berikutnya ikut bergeser. Terukur: dua eksekusi berturut-turut
    dengan benih awal yang sama memberi titik-pusat benda yang berbeda, yang
    merambat jadi selisih 4e-5 m pada matriks akhir.

    Memasang ulang benih di sini membuat "masukan sama → keluaran sama" berlaku
    tanpa peduli urutan pemanggilan. Itu yang sebenarnya dibutuhkan; benih milik
    pemanggil tidak menjaminnya.
    """
    _o3d().utility.random.seed(BENIH_TETAP)


# ═══════════════════════════════════════════════════════════════════════════════
# Bantuan geometri
# ═══════════════════════════════════════════════════════════════════════════════

def terapkan(xyz: np.ndarray, T) -> np.ndarray:
    """Kenakan 4x4 pada Nx3."""
    if T is None:
        return np.asarray(xyz, dtype=np.float64)
    T = np.asarray(T, dtype=np.float64)
    xyz = np.asarray(xyz, dtype=np.float64)
    return xyz @ T[:3, :3].T + T[:3, 3]


def potong(xyz: np.ndarray, radius: float) -> np.ndarray:
    """Sisakan titik dalam bola berjari-jari `radius` dari asal sensor."""
    if radius is None or radius <= 0:
        return xyz
    return xyz[np.linalg.norm(xyz, axis=1) <= float(radius)]


def putar_x(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    T = np.eye(4)
    T[1:3, 1:3] = [[c, -s], [s, c]]
    return T


def derajat_miring(T) -> float:
    """Seberapa miring scan aslinya, dari matriks penegaknya."""
    if T is None:
        return 0.0
    return float(np.degrees(np.arccos(np.clip(np.asarray(T)[2, 2], -1.0, 1.0))))


PLANE_TOL = 0.05        # tebal bidang (m)
PLANE_MIN_FRAC = 0.015  # bidang lebih kecil dari ini tidak dianggap latar
PLANE_MIN_AREA = 8.0    # m² TERISI — huruf timbul tak akan lolos, tembok lolos
PLANE_MAKS = 14
ARAH_BIN = 30           # kisi arah normal: 30 azimut x 30 elevasi
ARAH_TOL = np.cos(np.radians(15.0))
ATLAS_VOXEL = 0.05


def _pas_bidang_svd(pts):
    """Bidang paling pas lewat SVD. → (normal satuan, d). Tanpa keacakan.

    Total least squares: normalnya vektor eigen terkecil kovarians. Berbeda dari
    RANSAC, jawabannya fungsi murni dari masukannya.
    """
    c = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - c, full_matrices=False)
    n = Vt[-1]
    return n, float(-np.dot(n, c))


def _arah_dominan(normal, berapa):
    """Arah normal yang paling sering muncul, dari kisi bola tetap.

    Kisi tetap, bukan pengelompokan serakah: kisi bisa diulang, pengelompokan
    serakah bergantung urutan masukan.
    """
    az = np.arctan2(normal[:, 1], normal[:, 0])
    el = np.arcsin(np.clip(normal[:, 2], -1.0, 1.0))
    ia = np.clip(((az + np.pi) / (2 * np.pi) * ARAH_BIN).astype(int), 0, ARAH_BIN - 1)
    ie = np.clip(((el + np.pi / 2) / np.pi * ARAH_BIN).astype(int), 0, ARAH_BIN - 1)
    kunci = ia * ARAH_BIN + ie
    cacah = np.bincount(kunci, minlength=ARAH_BIN * ARAH_BIN)

    arah = []
    for k in np.argsort(-cacah)[:berapa * 3]:
        if cacah[k] == 0:
            break
        n = normal[kunci == k].mean(axis=0)
        norma = float(np.linalg.norm(n))
        if norma < 1e-9:
            continue
        n = n / norma
        if any(abs(float(np.dot(n, a))) > ARAH_TOL for a in arah):
            continue
        arah.append(n)
        if len(arah) >= berapa:
            break
    return arah


def atlas_bidang(xyz: np.ndarray, tol: float = PLANE_TOL,
                 min_frac: float = PLANE_MIN_FRAC,
                 min_area: float = PLANE_MIN_AREA,
                 maks: int = PLANE_MAKS) -> list:
    """Bidang latar besar: tanah, tembok, kanopi. → daftar (a,b,c,d) dinormalkan.

    DETERMINISTIK, dan itu bukan kemewahan. `clomerged.plane_atlas` memakai
    `segment_plane`, yang diparalelkan Open3D: hasilnya bisa berubah antar
    eksekusi walau benihnya dipasang, karena tiap utas menarik undiannya sendiri
    menurut jadwal yang tak bisa diramalkan. Terukur di proyek ini: dua eksekusi
    berturut-turut memberi daftar benda yang berbeda, yang merambat sampai ke
    matriks akhir. Catatan proyek sudah mencatat pelajaran yang sama sebelumnya.

    Cara kerjanya tanpa keacakan sama sekali:
      1. normal ditaksir, lalu n dan -n disatukan
      2. arah normal yang sering muncul dicari dari kisi bola TETAP
      3. untuk tiap arah, jarak d = p·n dihistogramkan; puncaknya = bidang
      4. tiap puncak dipasang ulang lewat SVD dan diuji LUAS TERISI-nya

    Uji luas terisi, bukan jumlah titik: tembok berlubang pintu tetap terhitung
    luas, sedangkan enam huruf yang berjauhan hanya seluas hurufnya sendiri.
    """
    import clomerged
    o3d = _o3d()
    xyz = np.asarray(xyz, dtype=np.float64)
    if len(xyz) < 500:
        return []

    pc = to_o3d(xyz).voxel_down_sample(ATLAS_VOXEL)
    if len(pc.points) < 200:
        return []
    pc.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=ATLAS_VOXEL * 4.0, max_nn=40))
    P = np.asarray(pc.points)
    N = clomerged.fold_normals(np.asarray(pc.normals))
    ambang = max(int(min_frac * len(P)), 50)

    hasil = []
    for n in _arah_dominan(N, maks):
        sejajar = np.abs(N @ n) > ARAH_TOL
        if int(sejajar.sum()) < ambang:
            continue
        jarak = P[sejajar] @ n
        lo, hi = float(jarak.min()), float(jarak.max())
        if hi - lo < 1e-6:
            tepi = np.array([lo - tol, lo + tol])
        else:
            tepi = np.arange(lo, hi + tol, tol)
        cacah, tepi = np.histogram(jarak, bins=tepi)

        for k in np.argsort(-cacah):
            if cacah[k] < ambang:
                break
            d0 = -0.5 * (tepi[k] + tepi[k + 1])
            if any(abs(float(np.dot(n, m[:3]))) > ARAH_TOL
                   and abs(d0 - m[3] * np.sign(np.dot(n, m[:3]))) < tol
                   for m in hasil):
                continue
            inlier = np.abs(P @ n + d0) < tol
            if int(inlier.sum()) < ambang:
                continue
            nn, dd = _pas_bidang_svd(P[inlier])
            if float(np.dot(nn, n)) < 0:
                nn, dd = -nn, -dd
            inlier = np.abs(P @ nn + dd) < tol
            if int(inlier.sum()) < ambang:
                continue
            if clomerged.plane_area(P[inlier], nn) < min_area:
                continue
            hasil.append(np.array([nn[0], nn[1], nn[2], dd]))
            if len(hasil) >= maks:
                return hasil
    return hasil


def _topeng_menonjol(xyz: np.ndarray, atlas: list) -> np.ndarray:
    import clomerged
    return clomerged.salient_mask(xyz, atlas)


# ═══════════════════════════════════════════════════════════════════════════════
# buang_rig — tripod, mount, dan operator ikut ter-scan
# ═══════════════════════════════════════════════════════════════════════════════

def buang_rig(xyz: np.ndarray, atlas: list, radius: float = EGO_RADIUS):
    """Buang titik MENONJOL di dalam silinder `radius` dari sumbu sensor.

    → (awan bersih, berapa titik dibuang)

    Rig bergerak BERSAMA sensor, jadi ia cocok sempurna pada pose apa pun dan
    menarik ICP ke arah menumpuk-tripod. Di scan_0080-0083 ia 311-431 titik di
    tiap scan, tidak pernah di-mask oleh perintah mana pun sebelum ini.

    Syaratnya "menonjol DAN di dalam radius", bukan sekadar di dalam radius:
    scan_0081 punya bidang tembok di x = +0,49 — di dalam radius — dan tembok
    itu harus tetap hidup. Hanya yang bukan bagian dari bidang latar yang dibuang.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    dekat = np.hypot(xyz[:, 0], xyz[:, 1]) <= float(radius)
    buang = dekat & _topeng_menonjol(xyz, atlas)
    return xyz[~buang], int(buang.sum())


# ═══════════════════════════════════════════════════════════════════════════════
# Kerangka tanah — mengunci roll, pitch, z
# ═══════════════════════════════════════════════════════════════════════════════

def kerangka_tanah(xyz: np.ndarray, crop: float = GROUND_CROP):
    """4x4 yang menegakkan scan. None bila tanahnya tak bisa dipercaya.

    Bidang tanah SELALU dicari dari potongan `crop`, tak peduli seberapa luas
    awan yang diberikan. Tanpa itu jawabannya ikut berubah mengikuti --range:
    scan_0081 diratakan 7,35° pada --range 6 tapi 9,38° pada --range 15, dan
    selisih 2° berarti 17 cm meleset di jarak 5 m — bocor ke semua langkah
    sesudahnya.

    Pemasangan bidangnya lewat pemangkasan berulang + SVD, bukan RANSAC —
    lihat `atlas_bidang` untuk alasannya.

    Sudut putar tegaknya dibiarkan apa adanya; itu urusan jangkar.
    """
    xyz = potong(np.asarray(xyz, dtype=np.float64), crop)
    if len(xyz) < 200:
        return None

    lo = xyz[xyz[:, 2] < np.percentile(xyz[:, 2], 40.0)]
    if len(lo) < 50:
        return None

    # Taksiran awal dari irisan terbawah. Irisan itu MIRING kalau scan-nya
    # miring — pada 9° dan jangkauan 6 m ia sebuah baji setebal 1,9 m — jadi
    # taksiran pertama pasti condong dan tidak boleh dipercaya apa adanya.
    n, d = _pas_bidang_svd(lo)

    # Lalu bidangnya merekrut ulang dari SELURUH awan: tiap putaran memungut
    # titik yang dekat ke bidang saat ini, bukan titik yang kebetulan rendah.
    # Begitu ia mulai menempel ke tanah yang sebenarnya, bajinya lepas sendiri.
    for _ in range(8):
        dekat = np.abs(xyz @ n + d) < 0.08
        if int(dekat.sum()) < 50:
            break
        n_baru, d_baru = _pas_bidang_svd(xyz[dekat])
        if float(np.dot(n_baru, n)) < 0:
            n_baru, d_baru = -n_baru, -d_baru
        tenang = float(np.dot(n_baru, n)) > 1.0 - 1e-12
        n, d = n_baru, d_baru
        if tenang:
            break
    if n[2] < 0:
        n, d = -n, -d
    if abs(n[2]) < 0.8 or float((np.abs(xyz @ n + d) < 0.08).mean()) < 0.10:
        return None

    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    sn = float(np.linalg.norm(v))
    cs = float(np.dot(n, z))

    T = np.eye(4)
    if sn > 1e-9:
        K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        T[:3, :3] = np.eye(3) + K + K @ K * ((1.0 - cs) / sn ** 2)
    T[2, 3] = d
    return T


# ═══════════════════════════════════════════════════════════════════════════════
# Benda — calon jangkar
# ═══════════════════════════════════════════════════════════════════════════════

class Benda:
    """Satu gugus menonjol yang berdiri sendiri. Calon jangkar."""

    __slots__ = ("id", "pusat", "jumlah_titik", "ukuran",
                 "tinggi_dari_tanah", "jarak_ke_tembok", "titik")

    def __init__(self, id, titik, atlas=None):
        self.id = int(id)
        self.titik = np.asarray(titik, dtype=np.float64)
        self.pusat = self.titik.mean(axis=0)
        self.jumlah_titik = len(self.titik)
        self.ukuran = self.titik.max(axis=0) - self.titik.min(axis=0)
        self.tinggi_dari_tanah = float(self.titik[:, 2].min())
        self.jarak_ke_tembok = _jarak_ke_tembok(self.pusat, atlas)

    def __repr__(self):
        return (f"Benda#{self.id}(n={self.jumlah_titik}, "
                f"pusat=({self.pusat[0]:+.2f},{self.pusat[1]:+.2f},"
                f"{self.pusat[2]:+.2f}), "
                f"ukuran=({self.ukuran[0]:.2f},{self.ukuran[1]:.2f},"
                f"{self.ukuran[2]:.2f}))")

    def sifat(self) -> dict:
        """Sifat yang dipakai mengusulkan pasangan. Semua tak tergantung pose."""
        return {
            "tinggi": float(self.ukuran[2]),
            "tapak": float(max(self.ukuran[0], self.ukuran[1])),
            "tipis": float(min(self.ukuran[0], self.ukuran[1])),
            "titik": int(self.jumlah_titik),
            "dari_tanah": float(self.tinggi_dari_tanah),
            "ke_tembok": float(self.jarak_ke_tembok),
        }


def _jarak_ke_tembok(titik, atlas) -> float:
    """Jarak ke bidang TEGAK terdekat. inf bila tak ada tembok di atlas."""
    if not atlas:
        return float("inf")
    tegak = [m for m in atlas if abs(m[2]) < 0.5]
    if not tegak:
        return float("inf")
    return float(min(abs(float(np.dot(m[:3], titik) + m[3])) for m in tegak))


def daftar_benda(xyz: np.ndarray, atlas: list = None,
                 max_tapak: float = BENDA_MAX_TAPAK) -> list:
    """Gugus menonjol yang berdiri sendiri, sudah disaring. Rig sudah dibuang.

    Saringannya bukan hiasan. Di data asli titik menonjol mencapai 34-42% dari
    seluruh titik — tembok bengkok dan kanopi bocor lewat pengupasan sebagai
    "ciri". Saringan tapak/tinggi membuangnya dan menyisakan 6-21 benda per scan.

    Tapi ia juga bisa membuang benda sungguhan tanpa bersuara: pada scan_0083
    huruf M punya 1.949 titik — terpadat kedua di scan itu — dan tetap ditolak
    karena tapaknya 1,65 m. `max_tapak` ada supaya adegan berhuruf lebar bisa
    melonggarkannya tanpa mengubah bawaan yang menjaga adegan lain.
    """
    o3d = _o3d()
    xyz = np.asarray(xyz, dtype=np.float64)
    if atlas is None:
        atlas = atlas_bidang(xyz)

    bersih, _ = buang_rig(xyz, atlas)
    menonjol = bersih[_topeng_menonjol(bersih, atlas)]
    if len(menonjol) < BENDA_MIN_TITIK:
        return []

    pc = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(menonjol)).voxel_down_sample(BENDA_VOXEL)
    titik = np.asarray(pc.points)
    label = np.asarray(pc.cluster_dbscan(eps=BENDA_EPS,
                                         min_points=BENDA_MIN_TETANGGA))
    if len(label) == 0 or label.max() < 0:
        return []

    hasil = []
    for k in range(int(label.max()) + 1):
        q = titik[label == k]
        if len(q) < BENDA_MIN_TITIK:
            continue
        ukuran = q.max(axis=0) - q.min(axis=0)
        if max(ukuran[0], ukuran[1]) > max_tapak:
            continue
        if ukuran[2] > BENDA_MAX_TINGGI:
            continue
        hasil.append(Benda(len(hasil), q, atlas))
    return hasil


# ═══════════════════════════════════════════════════════════════════════════════
# Penilaian — berpasangan, dan kebal kepadatan
# ═══════════════════════════════════════════════════════════════════════════════

NILAI_VOXEL = 0.03      # acuan SELALU dijarangkan ke kisi ini sebelum dinilai
NILAI_KASAR = 0.05
FITNESS_DIST = 0.10
TAJAM_DIST = 0.03

REDAM = 0.1             # sisa gerak yang diizinkan sepanjang arah lemah
ICP_SCALES = (0.60, 0.30, 0.15, 0.08)
ICP_ITER = 60


def to_o3d(xyz):
    o3d = _o3d()
    return o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64)))


def _jarangkan(xyz, voxel):
    return np.asarray(to_o3d(xyz).voxel_down_sample(voxel).points)


def nilai(sumber: np.ndarray, acuan: np.ndarray, T) -> dict:
    """Nilai satu pose. SELALU berpasangan — jangan pernah melawan peta gabungan.

    Itu bukan pilihan gaya. `outmerge` menilai tiap scan melawan peta yang sedang
    tumbuh — tiga scan lain sekaligus, kira-kira tiga kali lebih padat, jadi tiap
    titik punya kira-kira tiga kali lebih banyak kesempatan menemukan pasangan
    dalam 10 cm. Ia melapor 0,61-0,85 BAIK untuk pose yang diukur berpasangan
    cuma 0,04-0,22. Angkanya mengikuti kepadatan peta, bukan kebenaran.

    Penawarnya dua lapis: hanya menilai berpasangan, DAN menjarangkan acuan ke
    kisi tetap lebih dulu sehingga kepadatan tidak lagi bisa membeli nilai.

    Dua angka dilaporkan karena satu pun tidak cukup:
      fitness10  longgar, jenuh — 0,4 m dan 1,2 m tak terbedakan olehnya
      tajam3     ketat, hanya pada ciri bukan-bidang — ini yang menolak geseran
      n_tampalan tanpa ini, dua angka di atas tak bisa ditafsirkan
    """
    from scipy.spatial import cKDTree

    src = terapkan(sumber, T)
    acuan = np.asarray(acuan, dtype=np.float64)

    a_kasar = _jarangkan(acuan, NILAI_KASAR)
    s_kasar = _jarangkan(src, NILAI_KASAR)
    d, _ = cKDTree(a_kasar).query(s_kasar, k=1)
    fitness10 = float((d < FITNESS_DIST).mean()) if len(d) else 0.0

    atlas_s = atlas_bidang(sumber)
    sal = src[_topeng_menonjol(sumber, atlas_s)]
    a_halus = _jarangkan(acuan, NILAI_VOXEL)
    if len(sal) < BENDA_MIN_TITIK or len(a_halus) == 0:
        tajam3, n_tampalan = 0.0, 0
    else:
        s_halus = _jarangkan(sal, NILAI_VOXEL)
        dt, _ = cKDTree(a_halus).query(s_halus, k=1)
        tajam3 = float((dt < TAJAM_DIST).mean())
        n_tampalan = int((dt < TAJAM_DIST).sum())

    return {"fitness10": fitness10, "tajam3": tajam3,
            "n_tampalan": n_tampalan, "n_menonjol": int(len(sal))}


# ═══════════════════════════════════════════════════════════════════════════════
# Arah lemah — dan perapian yang tidak menyerah padanya
# ═══════════════════════════════════════════════════════════════════════════════

def arah_lemah_mendatar(sumber, acuan, T, voxel: float = 0.15) -> np.ndarray:
    """Arah mendatar yang PALING TIDAK terkunci geometri. Vektor satuan 2-D.

    ICP point-to-plane hanya menghukum simpangan sepanjang normal acuan, jadi
    Hessian geserannya ΣnnT. Dibatasi ke mendatar, ia jadi 2x2 dan vektor eigen
    terkecilnya persis arah yang bisa menggelincirkan hasil tanpa menaikkan
    galat: menyusuri tembok.

    Sengaja dibatasi ke mendatar, tidak memakai penguraian 6x6 seperti
    `clomerged.weak_direction`. Tanah sudah mengunci tiga derajat sisanya, dan
    penguraian penuh kadang menjawab "putaran" — benar tapi tak bisa dipakai
    untuk apa yang harus dilakukan di sini.
    """
    o3d = _o3d()
    a = to_o3d(terapkan(sumber, T)).voxel_down_sample(voxel)
    b = to_o3d(acuan).voxel_down_sample(voxel)
    if len(a.points) < 20 or len(b.points) < 20:
        return np.array([1.0, 0.0])
    b.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3.0, max_nn=30))
    pohon = o3d.geometry.KDTreeFlann(b)
    N = np.asarray(b.normals)

    H = np.zeros((2, 2))
    batas = (voxel * 2.0) ** 2
    for p in np.asarray(a.points):
        ok, idx, d2 = pohon.search_knn_vector_3d(p, 1)
        if not ok or d2[0] > batas:
            continue
        n = N[idx[0]][:2]
        H += np.outer(n, n)

    if np.trace(H) < 1e-9:
        return np.array([1.0, 0.0])
    _, V = np.linalg.eigh(H)
    v = V[:, 0]
    return v / max(float(np.linalg.norm(v)), 1e-12)


def _piramida(xyz):
    o3d = _o3d()
    p = to_o3d(xyz)
    tingkat = []
    for s in ICP_SCALES:
        d = p.voxel_down_sample(s)
        d.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=s * 3.0, max_nn=30))
        tingkat.append(d)
    return tingkat


def rapikan(sumber, acuan, T0, redam: float = REDAM) -> np.ndarray:
    """ICP bertingkat yang boleh memoles tapi tidak boleh menggelincir.

    Setelah tiap tingkat, perpindahan diuraikan ke arah lemah dan sisanya.
    Komponen sepanjang arah lemah dikalikan `redam`; sisanya dibiarkan penuh.

    `redam=0` membekukan arah lemah pada jawaban jangkar. `redam=1` sama dengan
    ICP biasa. Baku 0,1 memberi ICP beberapa sentimeter untuk memperbaiki galat
    titik-pusat jangkar tanpa memberinya semeteran untuk tergelincir.

    Bedanya dengan `clomerged.unslide`: unslide MENYAPU arah lemah mencari
    puncak, dan karena itu masih bisa mendarat di puncak alias yang salah —
    tercatat menggeser scan 0074 dari -0,06 m ke -1,90 m. Di sini arah itu tidak
    dicari sama sekali; jangkar sudah menjawabnya.
    """
    o3d = _o3d()
    T = np.asarray(T0, dtype=np.float64).copy()
    lemah = arah_lemah_mendatar(sumber, acuan, T0)

    src_pyr, ref_pyr = _piramida(sumber), _piramida(acuan)
    for a, b, s in zip(src_pyr, ref_pyr, ICP_SCALES):
        baru = np.array(o3d.pipelines.registration.registration_icp(
            a, b, s * 2.0, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=ICP_ITER)).transformation, dtype=np.float64)

        d = baru[:2, 3] - T[:2, 3]
        sejajar = float(np.dot(d, lemah))
        baru[:2, 3] = T[:2, 3] + (d - sejajar * lemah) + redam * sejajar * lemah
        T = baru
    return T


TEGAK_MIN_TITIK = 4000   # bidang tegak lebih kecil dari ini bukan tembok
TEGAK_MAKS = 15.0        # koreksi lebih besar dari ini: curigai, jangan pakai


def tegakkan_tembok(xyz: np.ndarray, atlas: list, maks: float = TEGAK_MAKS):
    """Koreksi roll/pitch memakai tembok sebagai acuan tegak, bukan tanah.

    → 4x4, atau None bila tak ada tembok yang cukup besar.

    `kerangka_tanah` meratakan ke tanah, dan itu benar HANYA bila tanahnya
    tegak lurus gravitasi. Di FILKOM tidak: terukur pada scan_0080-0083,
    sesudah perataan tanah tembok yang seharusnya tegak condong 4,2-8,7 deg,
    dan keempat scan jadi saling miring sampai 9,6 deg. Bangunan dibangun
    tegak lurus gravitasi; tanah punya kemiringan buangan air. Temboknya acuan
    yang lebih baik.

    BATASNYA, dan ini bukan cacat yang bisa ditambal parameter: tembok hanya
    mengunci kemiringan pada sumbu TANGENnya. Memiringkan awan pada sumbu
    normal tembok memetakan bidang tembok ke dirinya sendiri, jadi ketegakannya
    tak berubah. Perlu dua tembok dengan arah normal berbeda untuk mengunci
    keduanya; di scan_0080-0083 hanya ada satu arah (sebaran azimut 0,1-0,5
    deg), jadi satu derajat tetap memakai jawaban tanah. Koreksi yang diambil
    di sini yang TERKECIL — arah tegak baru dipilih yang paling dekat dengan
    tegak lama — supaya derajat yang tak terkunci tidak ikut tergeser.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    tegak = []
    for m in atlas:
        if abs(m[2]) >= 0.5:
            continue
        n = int((np.abs(xyz @ m[:3] + m[3]) < PLANE_TOL) .sum())
        if n >= TEGAK_MIN_TITIK:
            tegak.append((n, m[:3]))
    if not tegak:
        return None

    # Normal rata-rata berbobot jumlah titik. Tanda disamakan dulu supaya
    # dua sisi tembok yang sama tidak saling meniadakan.
    acuan = max(tegak, key=lambda e: e[0])[1]
    N = np.zeros(3)
    for n, v in tegak:
        N += n * (v if float(np.dot(v, acuan)) >= 0 else -v)
    N /= np.linalg.norm(N)

    z = np.array([0.0, 0.0, 1.0])
    tegak_baru = z - float(np.dot(z, N)) * N        # komponen z yang tegak lurus N
    norm = float(np.linalg.norm(tegak_baru))
    if norm < 1e-9:
        return None
    tegak_baru /= norm

    sudut = math.acos(float(np.clip(np.dot(tegak_baru, z), -1.0, 1.0)))
    if math.degrees(sudut) > maks:
        return None
    if sudut < 1e-9:
        return np.eye(4)

    sumbu = np.cross(tegak_baru, z)
    sumbu /= np.linalg.norm(sumbu)
    K = np.array([[0, -sumbu[2], sumbu[1]],
                  [sumbu[2], 0, -sumbu[0]],
                  [-sumbu[1], sumbu[0], 0]], dtype=np.float64)
    R = np.eye(3) + math.sin(sudut) * K + (1 - math.cos(sudut)) * (K @ K)

    T = np.eye(4)
    T[:3, :3] = R
    # Tanah tetap di z = 0: putaran di atas dilakukan terhadap titik asal.
    dekat = xyz[np.abs(xyz[:, 2]) < 0.30]
    if len(dekat) > 200:
        T[2, 3] = -float(np.median((dekat @ R.T)[:, 2]))
    return T


# ═══════════════════════════════════════════════════════════════════════════════
# Tembok utama — cadangan yaw saat jangkarnya cuma satu
# ═══════════════════════════════════════════════════════════════════════════════

def tembok_utama(xyz: np.ndarray, atlas: list):
    """Normal bidang TEGAK dengan titik terbanyak. None bila tak ada tembok."""
    xyz = np.asarray(xyz, dtype=np.float64)
    terbaik, paling = None, 0
    for m in atlas:
        if abs(m[2]) >= 0.5:
            continue
        n = int((np.abs(xyz @ m[:3] + m[3]) < 0.05).sum())
        if n > paling:
            terbaik, paling = m, n
    return None if terbaik is None else np.asarray(terbaik[:3], dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════════════════
# pasang — jangkar menyelesaikan, perapian memoles
# ═══════════════════════════════════════════════════════════════════════════════

def _matriks_yaw(rad):
    c, s = np.cos(rad), np.sin(rad)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def pose_jangkar(benda_s, benda_a, pasangan,
                 tembok_s=None, tembok_a=None) -> tuple:
    """→ (daftar kandidat 4x4, asal_yaw). Tanpa pencarian sama sekali.

    Dua jangkar atau lebih: satu jawaban, tertentu penuh.
    Satu jangkar: yaw dipinjam dari normal tembok, yang menyisakan ambiguitas
    180° — jadi dua kandidat dikembalikan, dan yang memilih adalah nilai akhir.
    """
    if not pasangan:
        raise ValueError("tidak ada pasangan jangkar")
    Q = np.array([benda_s[i].pusat for i, _ in pasangan], dtype=np.float64)
    P = np.array([benda_a[j].pusat for _, j in pasangan], dtype=np.float64)

    if len(pasangan) >= 2:
        return [kabsch2d(P, Q)], "jangkar"

    if tembok_s is None or tembok_a is None:
        raise ValueError(
            "satu jangkar tanpa tembok tidak menentukan yaw — jangan dikarang")

    beda = (np.arctan2(tembok_a[1], tembok_a[0])
            - np.arctan2(tembok_s[1], tembok_s[0]))
    kandidat = []
    for tambah in (0.0, np.pi):
        T = _matriks_yaw(beda + tambah)
        T[:2, 3] = P[0, :2] - T[:2, :2] @ Q[0, :2]
        kandidat.append(T)
    return kandidat, "tembok"


def pasang(sumber, acuan, benda_s, benda_a, pasangan,
           redam: float = REDAM, icp: bool = True) -> dict:
    """Selesaikan pose sumber di kerangka acuan dari jangkar yang ditunjuk.

    → dict berisi T, asal_yaw, sisa_jangkar, geser_perapian, nilai, peringatan.

    Peringatannya sengaja banyak. Sudah ada tiga sumber kepercayaan diri palsu
    yang terukur di proyek ini; jangan tambah yang keempat.
    """
    sumber = np.asarray(sumber, dtype=np.float64)
    acuan = np.asarray(acuan, dtype=np.float64)
    if not pasangan:
        raise ValueError("tidak ada pasangan jangkar — tak ada yang bisa dipasang")

    tembok_s = tembok_a = None
    if len(pasangan) == 1:
        tembok_s = tembok_utama(sumber, atlas_bidang(sumber))
        tembok_a = tembok_utama(acuan, atlas_bidang(acuan))

    kandidat, asal_yaw = pose_jangkar(benda_s, benda_a, pasangan,
                                      tembok_s, tembok_a)

    if len(kandidat) == 1:
        T_jangkar = kandidat[0]
    else:
        skor = [nilai(sumber, acuan, T)["tajam3"] for T in kandidat]
        T_jangkar = kandidat[int(np.argmax(skor))]

    # `icp=False` menyerahkan pose apa adanya dari jangkar. Dipakai bila
    # tampalannya terlalu tipis untuk dipoles: terukur pada tepi 0082-0083,
    # jawaban jangkar tepat 1,1 cm lalu ICP menyeretnya 1,62 m. `redam` tak
    # menolong di situ — ia hanya meredam arah lemah, seretannya tegak lurus.
    T = rapikan(sumber, acuan, T_jangkar, redam=redam) if icp else T_jangkar.copy()

    Q = np.array([benda_s[i].pusat for i, _ in pasangan], dtype=np.float64)
    P = np.array([benda_a[j].pusat for _, j in pasangan], dtype=np.float64)
    sisa, per_jangkar = sisa_jangkar(P, Q, rinci=True)
    geser = float(np.linalg.norm(T[:2, 3] - T_jangkar[:2, 3]))

    peringatan = []
    if len(pasangan) == 1:
        peringatan.append(
            "yaw dipinjam dari normal tembok, bukan dari jangkar — "
            "satu jangkar tidak menentukannya")
    elif len(pasangan) == 2 and sisa > 0.3:
        peringatan.append(
            f"selisih jarak antar-jangkar {sisa:.2f} m — hampir pasti salah "
            f"tunjuk; dua jangkar hanya punya sisa ini")
    elif len(pasangan) >= 3 and sisa > 0.5:
        peringatan.append(
            f"sisa jangkar {sisa:.2f} m, paling menyimpang jangkar ke-"
            f"{int(np.argmax(per_jangkar))}")
    if icp and geser > 0.5:
        peringatan.append(
            f"perapian menggeser {geser:.2f} m dari jawaban jangkar — "
            f"redamannya bocor, atau jangkarnya salah")

    return {"T": T, "T_jangkar": T_jangkar, "asal_yaw": asal_yaw,
            "sisa_jangkar": float(sisa), "per_jangkar": per_jangkar,
            "geser_perapian": geser, "nilai": nilai(sumber, acuan, T),
            "peringatan": peringatan}


# ═══════════════════════════════════════════════════════════════════════════════
# Penyerahan ke pcs — dua fasa dengan berkas di antaranya
# ═══════════════════════════════════════════════════════════════════════════════

MERGE_DIRNAME = "_pasak"

# Warna yang terbedakan mata, bukan sandi ID. ID-nya ada di benda.json; pcs
# mengembalikan koordinat klik dan yang memetakannya ke ID adalah `benda_di`.
PALET = [
    (230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200),
    (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230),
    (210, 245, 60), (250, 190, 212), (0, 128, 128), (220, 190, 255),
    (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195),
    (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128),
]


def benda_di(daftar, titik, batas: float = 0.6):
    """Benda mana yang dimaksud oleh sebuah klik. → id, atau None.

    Kotak pembatas dulu, baru pusat terdekat. Kliknya boleh kasar — DBSCAN
    memisahkan benda paling rapat 0,12 m, jadi salah tunjuk hampir mustahil
    kecuali kliknya memang jatuh di antara dua benda.
    """
    titik = np.asarray(titik, dtype=np.float64)[:3]
    for b in daftar:
        lo = b.titik.min(axis=0) - 0.05
        hi = b.titik.max(axis=0) + 0.05
        if np.all(titik >= lo) and np.all(titik <= hi):
            return b.id
    jarak = [float(np.linalg.norm(b.pusat - titik)) for b in daftar]
    if not jarak:
        return None
    k = int(np.argmin(jarak))
    return daftar[k].id if jarak[k] < batas else None


def usulkan_pasangan(benda_s, benda_a, berapa: int = 8) -> list:
    """Peringkat calon pasangan menurut kemiripan sifat. → [(i, j, nilai), ...]

    Ini USULAN, bukan jawaban. Pencocokan otomatis lewat konstelasi sudah diuji
    pada data asli dan gagal — klik maksimum cuma 2-3 pasang, sisa sampai 3 m —
    karena titik-pusat gugus tidak terulang antar sudut pandang. Yang dikerjakan
    di sini cuma memperpendek daftar yang harus dilihat manusia.
    """
    calon = []
    for i, bs in enumerate(benda_s):
        for j, ba in enumerate(benda_a):
            s, a = bs.sifat(), ba.sifat()
            beda = (abs(s["tinggi"] - a["tinggi"]) / 0.30
                    + abs(s["tapak"] - a["tapak"]) / 0.25
                    + abs(s["tipis"] - a["tipis"]) / 0.25
                    + abs(s["dari_tanah"] - a["dari_tanah"]) / 0.30
                    + abs(np.log(max(s["titik"], 1) / max(a["titik"], 1))) / 0.8)
            calon.append((i, j, float(beda)))
    calon.sort(key=lambda t: t[2])
    return calon[:berapa]


def tulis_benda_ply(path, daftar) -> None:
    """Awan benda saja, tiap benda satu warna. Untuk dilihat manusia di pcs."""
    o3d = _o3d()
    if not daftar:
        return
    titik = np.vstack([b.titik for b in daftar])
    warna = np.vstack([np.tile(np.array(PALET[b.id % len(PALET)]) / 255.0,
                               (len(b.titik), 1)) for b in daftar])
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(titik))
    pc.colors = o3d.utility.Vector3dVector(warna)
    o3d.io.write_point_cloud(str(path), pc)


def kotak_benda(daftar) -> list:
    """Ringkasan tiap benda yang cukup untuk menjawab klik, tanpa awan titiknya."""
    return [{"id": b.id,
             "pusat": [float(v) for v in b.pusat],
             "lo": [float(v) for v in b.titik.min(axis=0)],
             "hi": [float(v) for v in b.titik.max(axis=0)],
             "sifat": b.sifat()} for b in daftar]


def benda_di_ringkasan(ringkas: list, titik, batas: float = 0.6):
    """Versi `benda_di` yang bekerja dari benda.json — dipakai oleh pcs."""
    t = np.asarray(titik, dtype=np.float64)[:3]
    for e in ringkas:
        if np.all(t >= np.array(e["lo"]) - 0.05) and np.all(t <= np.array(e["hi"]) + 0.05):
            return int(e["id"])
    if not ringkas:
        return None
    jarak = [float(np.linalg.norm(np.array(e["pusat"]) - t)) for e in ringkas]
    k = int(np.argmin(jarak))
    return int(ringkas[k]["id"]) if jarak[k] < batas else None


# ═══════════════════════════════════════════════════════════════════════════════
# Perintah
# ═══════════════════════════════════════════════════════════════════════════════

def siapkan(args) -> None:
    """Fasa satu: ekstrak benda dan serahkan ke mata manusia."""
    import json
    from pathlib import Path
    import clomcap, clomcaps, clomerge

    srcs = clomcaps.dedupe_inputs(args.files)
    if len(srcs) < 2:
        raise SystemExit("[ERROR] Butuh minimal dua berkas.")

    d = clomerge.next_merge_slot(Path(clomcap.OUT_ROOT) / MERGE_DIRNAME)
    (d / "awan").mkdir(parents=True, exist_ok=True)

    print("=" * 66)
    print(f"  pasak siapkan — {len(srcs)} berkas, jangkauan {args.range} m")
    print("=" * 66)

    katalog, semua = {}, {}
    for src in srcs:
        p = clomcaps.prepare_cloud(src, args)
        nama = Path(p).stem
        xyz = potong(clomerge.read_cloud_xyz(p), args.range)
        T = kerangka_tanah(xyz)
        if T is None:
            print(f"\n[{nama}] tanah tidak ditemukan — DILEWATI")
            continue
        xyz = terapkan(xyz, T)
        atlas = atlas_bidang(xyz)

        if getattr(args, "tegakkan", False):
            T2 = tegakkan_tembok(xyz, atlas)
            if T2 is None:
                print(f"[{nama}] tidak ada tembok cukup besar — tetap pakai tanah")
            else:
                sudut = math.degrees(math.acos(float(np.clip(T2[2, 2], -1, 1))))
                print(f"[{nama}] ditegakkan ke tembok: koreksi {sudut:.2f} derajat")
                xyz = terapkan(xyz, T2)
                atlas = atlas_bidang(xyz)
                T = T2 @ T

        bersih, rig = buang_rig(xyz, atlas)
        daftar = daftar_benda(xyz, atlas,
                              max_tapak=getattr(args, 'max_tapak',
                                                BENDA_MAX_TAPAK))

        _tulis_ply(d / "awan" / f"{nama}.ply", bersih)
        tulis_benda_ply(d / f"{nama}_benda.ply", daftar)
        katalog[nama] = kotak_benda(daftar)
        semua[nama] = daftar

        print(f"\n[{nama}]")
        print(f"  tanah diratakan {derajat_miring(T):.2f}°, "
              f"{len(xyz):,} titik, {len(atlas)} bidang latar")
        print(f"  rig dibuang     {rig:,} titik")
        print(f"  benda ditemukan {len(daftar)}")
        for b in daftar:
            s = b.sifat()
            print(f"     #{b.id:<3} pusat=({b.pusat[0]:+6.2f},{b.pusat[1]:+6.2f},"
                  f"{b.pusat[2]:+5.2f})  tinggi={s['tinggi']:.2f} "
                  f"tapak={s['tapak']:.2f} n={s['titik']}")

    if len(katalog) < 2:
        raise SystemExit("\n[ERROR] Kurang dari dua scan berhasil disiapkan.")

    nama_urut = list(katalog)
    usulan = {}
    for i in range(len(nama_urut)):
        for j in range(i + 1, len(nama_urut)):
            a, b = nama_urut[i], nama_urut[j]
            usulan[f"{a}|{b}"] = [
                {"a": int(x), "b": int(y), "beda": round(v, 3)}
                for x, y, v in usulkan_pasangan(semua[a], semua[b])]

    (d / "benda.json").write_text(json.dumps(katalog, indent=1))
    (d / "usulan.json").write_text(json.dumps(usulan, indent=1))
    if not (d / "pasangan.json").exists():
        (d / "pasangan.json").write_text(json.dumps({"pasangan": []}, indent=1))

    print("\n" + "=" * 66)
    print(f"  Ditulis ke {d}")
    print("=" * 66)
    print("\nLangkah berikutnya — tunjuk jangkarnya:")
    print(f"    pcs {d}/*_benda.ply")
    print("\nKlik satu benda di scan A lalu pasangannya di scan B. Dua pasang")
    print("cukup untuk mengunci penuh; satu pasang bisa, tapi yaw-nya lalu")
    print("bersandar pada tembok dan laporannya akan bilang begitu.")
    print(f"\nUsulan otomatis ada di {d}/usulan.json — periksa, jangan percaya.")
    print(f"Sesudah itu:\n    pasak selesaikan {d}")


def _tulis_ply(path, xyz) -> None:
    _o3d().io.write_point_cloud(str(path), to_o3d(xyz))


def selesaikan(args) -> None:
    """Fasa dua: selesaikan pose dari jangkar yang sudah ditunjuk."""
    import json
    from pathlib import Path
    import clomcap, clomerge

    d = Path(args.dir) if args.dir else _slot_terakhir()
    katalog = json.loads((d / "benda.json").read_text())
    pasangan_berkas = json.loads((d / "pasangan.json").read_text())["pasangan"]
    if not pasangan_berkas:
        raise SystemExit(
            f"[ERROR] {d}/pasangan.json masih kosong — belum ada jangkar yang "
            f"ditunjuk. Buka dulu:  pcs {d}/*_benda.ply")

    awan = {nm: clomerge.read_cloud_xyz(str(d / "awan" / f"{nm}.ply"))
            for nm in katalog}
    benda = {nm: [_benda_dari_json(e) for e in katalog[nm]] for nm in katalog}

    tepi = {}
    for e in pasangan_berkas:
        a, b = e["a"], e["b"]
        if a not in katalog or b not in katalog:
            raise SystemExit(f"[ERROR] pasangan menyebut scan yang tidak ada: {a} / {b}")
        jangkar = [(int(i), int(j)) for i, j in e["jangkar"]]
        for i, j in jangkar:
            if i >= len(benda[a]) or j >= len(benda[b]):
                raise SystemExit(
                    f"[ERROR] pasangan {a}#{i} = {b}#{j} menyebut benda yang "
                    f"tidak ada — jangan dilewati diam-diam")
        tepi[(a, b)] = (jangkar, bool(e.get("icp", True)))

    komponen = _komponen(list(katalog), tepi)
    if len(komponen) > 1:
        print(f"[WARN] Graf jangkar terpisah jadi {len(komponen)} kelompok. "
              f"Tiap kelompok jadi petanya sendiri.")

    pose, catatan = {}, {}
    for kel in komponen:
        akar = max(kel, key=lambda n: sum(1 for (a, b) in tepi if n in (a, b)))
        pose[akar] = np.eye(4)
        antre = [akar]
        while antre:
            u = antre.pop(0)
            for (a, b), (jangkar, pakai_icp) in tepi.items():
                for src, dst, jk in ((a, b, jangkar),
                                     (b, a, [(j, i) for i, j in jangkar])):
                    if dst != u or src in pose:
                        continue
                    print(f"\n[{src}] → [{dst}]  ({len(jk)} jangkar"
                          f"{'' if pakai_icp else ', tanpa ICP'})")
                    h = pasang(awan[src], awan[dst], benda[src], benda[dst],
                               jk, redam=args.redam, icp=pakai_icp)
                    pose[src] = pose[dst] @ h["T"]
                    catatan[src] = h
                    n = h["nilai"]
                    print(f"  fitness@10cm {n['fitness10']:.3f} | "
                          f"tajam@3cm {n['tajam3']:.3f} | "
                          f"tampalan {n['n_tampalan']:,} titik")
                    for w in h["peringatan"]:
                        print(f"  [WARN] {w}")
                    antre.append(src)

    _tulis_hasil(d, awan, pose, catatan, tepi, args)


def _benda_dari_json(e):
    b = Benda.__new__(Benda)
    b.id = int(e["id"])
    b.pusat = np.array(e["pusat"], dtype=np.float64)
    b.titik = np.array([e["lo"], e["hi"]], dtype=np.float64)
    b.jumlah_titik = int(e["sifat"]["titik"])
    b.ukuran = np.array(e["hi"], dtype=np.float64) - np.array(e["lo"], dtype=np.float64)
    b.tinggi_dari_tanah = float(e["sifat"]["dari_tanah"])
    b.jarak_ke_tembok = float(e["sifat"]["ke_tembok"])
    return b


def _komponen(simpul, tepi) -> list:
    sisa, keluar = set(simpul), []
    while sisa:
        akar = sorted(sisa)[0]
        kel, antre = {akar}, [akar]
        while antre:
            u = antre.pop()
            for (a, b) in tepi:
                for x, y in ((a, b), (b, a)):
                    if x == u and y not in kel:
                        kel.add(y)
                        antre.append(y)
        keluar.append(sorted(kel))
        sisa -= kel
    return keluar


def _slot_terakhir():
    from pathlib import Path
    import clomcap
    induk = Path(clomcap.OUT_ROOT) / MERGE_DIRNAME
    slot = sorted(p for p in induk.iterdir() if p.is_dir()) if induk.exists() else []
    if not slot:
        raise SystemExit(f"[ERROR] Belum ada hasil `pasak siapkan` di {induk}")
    return slot[-1]


def _tulis_hasil(d, awan, pose, catatan, tepi, args) -> None:
    """Tulis peta dan laporan. Nilainya BERPASANGAN, tidak pernah melawan peta."""
    o3d = _o3d()
    from pathlib import Path
    d = Path(d)

    nama = list(pose)
    gabung, periksa = [], []
    for k, nm in enumerate(nama):
        p = terapkan(awan[nm], pose[nm])
        gabung.append(p)
        w = np.tile(np.array(PALET[k % len(PALET)]) / 255.0, (len(p), 1))
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        pc.colors = o3d.utility.Vector3dVector(w)
        periksa.append(pc)

    _tulis_ply(d / "merged.ply", np.vstack(gabung))
    cek = periksa[0]
    for q in periksa[1:]:
        cek += q
    o3d.io.write_point_cloud(str(d / "merged_check.ply"), cek)

    baris = []
    baris.append("# pasak — jangkar ditunjuk manusia, geometri diselesaikan mesin")
    baris.append("#")
    baris.append("# Nilai di bawah SELALU berpasangan, tidak pernah melawan peta")
    baris.append("# gabungan. Menilai melawan peta yang tumbuh adalah cara outmerge")
    baris.append("# melaporkan 0,61-0,85 BAIK untuk pose yang berpasangan cuma")
    baris.append("# 0,04-0,22: peta tiga scan kira-kira tiga kali lebih padat, jadi")
    baris.append("# tiap titik punya tiga kali lebih banyak kesempatan menemukan")
    baris.append("# pasangan dalam 10 cm. Angkanya mengikuti kepadatan, bukan kebenaran.")
    baris.append("#")
    baris.append("# fitness@10cm  longgar dan JENUH — 0,4 m dan 1,2 m tak terbedakan")
    baris.append("# tajam@3cm     ketat, hanya pada ciri bukan-bidang — ini yang menolak geseran")
    baris.append("# tampalan      tanpa ini dua angka di atas tak bisa ditafsirkan")
    baris.append("")

    baris.append("# Asal-usul tiap pose:")
    for nm in nama:
        h = catatan.get(nm)
        if h is None:
            baris.append(f"#   {nm:<34} acuan")
            continue
        baris.append(f"#   {nm:<34} yaw dari {h['asal_yaw']}, "
                     f"sisa jangkar {h['sisa_jangkar']:.3f} m, "
                     f"perapian menggeser {h['geser_perapian']:.3f} m")
        for w in h["peringatan"]:
            baris.append(f"#     [WARN] {w}")
    baris.append("")

    baris.append("# Nilai SEMUA pasangan — termasuk yang tidak dijangkari,")
    baris.append("# karena pasangan yang tak dijangkari justru yang paling")
    baris.append("# mungkin diam-diam salah:")
    for i in range(len(nama)):
        for j in range(i + 1, len(nama)):
            a, b = nama[i], nama[j]
            T = np.linalg.inv(pose[b]) @ pose[a]
            n = nilai(awan[a], awan[b], T)
            tanda = "jangkar" if (a, b) in tepi or (b, a) in tepi else "—"
            baris.append(f"#   {a[5:9]}→{b[5:9]}  fitness {n['fitness10']:.3f}  "
                         f"tajam {n['tajam3']:.3f}  tampalan {n['n_tampalan']:>6,}  "
                         f"[{tanda}]")
    baris.append("")

    baris.append("# Matriks tiap scan ke kerangka acuan.")
    baris.append("# Sudah termasuk perataan tanah; pakai langsung pada scan asli.")
    for nm in nama:
        baris.append("")
        baris.append(nm)
        for r in pose[nm]:
            baris.append("    " + "".join(f"{v:11.6f}" for v in r))

    (d / "laporan.txt").write_text("\n".join(baris) + "\n")
    print("\n" + "=" * 66)
    print(f"  Peta   : {d}/merged.ply")
    print(f"  Periksa: {d}/merged_check.ply   (tiap scan satu warna)")
    print(f"  Laporan: {d}/laporan.txt")
    print("=" * 66)
    print(f"\nLihat sendiri — mata manusia yang memutuskan:\n    pcs {d}/merged_check.ply")


def build_parser():
    import argparse
    ap = argparse.ArgumentParser(
        prog="pasak",
        description="Registrasi berjangkar: tanah mengunci roll/pitch/z, "
                    "dua benda yang ditunjuk manusia mengunci yaw/x/y. "
                    "Tanpa pencarian, jadi tanpa minimum lokal dan tanpa alias.")
    sub = ap.add_subparsers(dest="perintah", required=True)

    s = sub.add_parser("siapkan", help="ekstrak benda, serahkan ke pcs")
    s.add_argument("files", nargs="+", help=".mcap atau .ply")
    s.add_argument("--range", type=float, default=6.0,
                   help="potong sejauh ini dari sensor (m). Terukur: --range 10 "
                        "memburukkan penilaian tajam 0,25-0,46 → 0,08-0,12")
    s.add_argument("-t", "--topic", default=None, help="topik PointCloud2")
    s.add_argument("--force", action="store_true", help="paksa konversi ulang mcap")
    s.add_argument("--tegakkan", action="store_true",
                   help="pakai tembok sebagai acuan tegak, bukan tanah; "
                        "membetulkan roll/pitch bila tanahnya miring")
    s.add_argument("--max-tapak", type=float, default=BENDA_MAX_TAPAK,
                   dest="max_tapak",
                   help=f"tapak mendatar terbesar yang masih dianggap benda, "
                        f"meter (baku {BENDA_MAX_TAPAK}); naikkan bila ada "
                        f"huruf atau benda lebar yang tak terpetik")
    s.set_defaults(fungsi=siapkan)

    k = sub.add_parser("selesaikan", help="selesaikan pose dari jangkar yang ditunjuk")
    k.add_argument("dir", nargs="?", default=None,
                   help="folder hasil siapkan; kosong = yang terakhir")
    k.add_argument("--redam", type=float, default=REDAM,
                   help="sisa gerak yang diizinkan sepanjang arah lemah. "
                        "0 membekukan, 1 sama dengan ICP biasa")
    k.set_defaults(fungsi=selesaikan)
    return ap


def main() -> None:
    args = build_parser().parse_args()
    args.fungsi(args)


if __name__ == "__main__":
    main()
