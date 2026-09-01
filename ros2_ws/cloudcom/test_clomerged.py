"""Tests untuk clomerged.py — penggabungan sadar fitur.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_clomerged.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import numpy as np
import pytest

import clomerged as cm
import outmerge as om
from clomerge import BAIK, RAGU, GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# Helper — tempat sintetis yang mengidap penyakitnya
# ═══════════════════════════════════════════════════════════════════════════════

def wall_site(seed=0, huruf=True) -> np.ndarray:
    """Tanah + satu tembok panjang polos + (opsional) tulisan timbul.

    Ini tiruan tempat FILKOM: temboknya mendominasi jumlah titik dan sama saja
    digeser menyusuri dirinya sendiri. Yang membedakan satu posisi dari posisi
    lain hanya hurufnya, dan hurufnya sedikit — persis perbandingan yang membuat
    algoritma lama tergelincir.
    """
    rng = np.random.default_rng(seed)

    tanah = np.column_stack([rng.uniform(-8, 8, 12000),
                             rng.uniform(-6, 2, 12000),
                             rng.normal(0, 0.004, 12000)])
    tembok = np.column_stack([rng.uniform(-7, 7, 14000),
                              np.random.default_rng(seed + 1).normal(0, 0.004, 14000),
                              rng.uniform(0.05, 3.2, 14000)])
    bagian = [tanah, tembok]

    if huruf:
        # letak sengaja tidak berjarak sama, supaya tidak ada jawaban geser yang
        # palsu-tapi-cocok — kebalikan dari "FILFILKOM"
        for x0, lebar in ((-3.1, 0.35), (-2.2, 0.22), (-1.4, 0.40),
                          (0.3, 0.30), (1.9, 0.45), (3.4, 0.25)):
            n = 900
            bagian.append(np.column_stack([
                rng.uniform(x0, x0 + lebar, n),
                rng.uniform(-0.14, -0.10, n),      # timbul 10-14 cm dari tembok
                rng.uniform(0.9, 1.7, n)]))
            # sisi samping huruf: normalnya menyusuri tembok — inilah satu-satunya
            # permukaan yang mengunci geseran
            for xs in (x0, x0 + lebar):
                m = 400
                bagian.append(np.column_stack([
                    np.full(m, xs) + rng.normal(0, 0.004, m),
                    rng.uniform(-0.14, 0.0, m),
                    rng.uniform(0.9, 1.7, m)]))
    return np.vstack(bagian)


class Args:
    """Pengganti hasil argparse untuk dipakai langsung di tes."""
    def __init__(self, **kw):
        self.range = 0.0
        self.step_deg = 5.0
        self.seeds = 6
        self.rounds = 0
        self.sharp = cm.SHARP_DIST
        self.salient_tol = cm.SALIENT_TOL
        self.slide_range = cm.SLIDE_RANGE
        self.weak_ratio = cm.WEAK_RATIO
        self.no_unslide = False
        self.__dict__.update(kw)


def geser(xyz, dx=0.0, dy=0.0, dz=0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (dx, dy, dz)
    return cm.apply_transform(xyz, T)


# ═══════════════════════════════════════════════════════════════════════════════
# Peta bidang dan titik menonjol
# ═══════════════════════════════════════════════════════════════════════════════

def test_plane_atlas_menemukan_tanah_dan_tembok():
    atlas = cm.plane_atlas(wall_site())
    assert len(atlas) >= 2

    normal = [np.abs(m[:3]) for m in atlas]
    assert any(n[2] > 0.9 for n in normal), "tanah (normal tegak) tidak ketemu"
    assert any(n[1] > 0.9 for n in normal), "tembok (normal mendatar) tidak ketemu"


def test_titik_menonjol_menangkap_huruf_bukan_tembok():
    xyz = wall_site()
    m = cm.salient_mask(xyz, cm.plane_atlas(xyz))

    # huruf ada di pita y antara -0.14 dan -0.10; tembok di sekitar y=0
    di_tembok = np.abs(xyz[:, 1]) < 0.02
    di_huruf = (xyz[:, 1] < -0.09) & (xyz[:, 2] > 0.85)

    assert m[di_huruf].mean() > 0.8, "huruf timbul seharusnya dianggap menonjol"
    assert m[di_tembok].mean() < 0.05, "tembok polos seharusnya TIDAK menonjol"


def test_tempat_tanpa_ciri_tidak_punya_titik_menonjol():
    xyz = wall_site(huruf=False)
    assert cm.salient_cloud(xyz, cm.plane_atlas(xyz)) is None


# ═══════════════════════════════════════════════════════════════════════════════
# Penilaian tajam — inti perbaikannya
# ═══════════════════════════════════════════════════════════════════════════════

def test_penilaian_tajam_menghukum_geseran_menyusuri_tembok():
    """Inilah bug aslinya: takaran lama memberi nilai tinggi untuk hasil yang
    tergelincir semeter menyusuri tembok. Takaran tajam harus menolaknya."""
    xyz = wall_site()
    atlas = cm.plane_atlas(xyz)
    sal = cm.salient_cloud(xyz, atlas)
    ref = cm.ref_cloud(xyz)

    benar, _ = cm.sharp_score(sal, ref, np.eye(4))
    T = np.eye(4)
    T[0, 3] = 1.0                                   # geser 1 m menyusuri tembok
    meleset, _ = cm.sharp_score(sal, ref, T)

    assert benar > 0.9
    assert meleset < 0.25, f"geseran 1 m masih dinilai {meleset:.3f}"
    assert benar > meleset * 3


def test_penilaian_tajam_tidak_terganggu_geseran_kecil_sekali():
    """Toleransinya harus longgar terhadap derau sensor, bukan cuma tajam."""
    xyz = wall_site()
    atlas = cm.plane_atlas(xyz)
    f, _ = cm.sharp_score(cm.salient_cloud(xyz, atlas), cm.ref_cloud(xyz),
                          np.eye(4))
    T = np.eye(4)
    T[0, 3] = 0.005                                 # 5 mm
    f2, _ = cm.sharp_score(cm.salient_cloud(xyz, atlas), cm.ref_cloud(xyz), T)
    assert f2 > f * 0.9


def test_vonis_dipimpin_fitness_tajam():
    assert cm.verdict(0.40, 0.015) == BAIK
    assert cm.verdict(0.18, 0.015) == RAGU
    assert cm.verdict(0.05, 0.015) == GAGAL
    # tajam tinggi tapi longgar — "cocok tapi tidak rapat" bukan BAIK
    assert cm.verdict(0.40, 0.028) != BAIK


# ═══════════════════════════════════════════════════════════════════════════════
# Pencuplikan berimbang menurut arah normal
# ═══════════════════════════════════════════════════════════════════════════════

def test_petak_normal_mengabaikan_tanda():
    n = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0],
                  [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]])
    p = cm.normal_bin(n)
    assert p[0] == p[1], "normal ke atas dan ke bawah harus satu petak"
    assert p[2] == p[3], "tembok menghadap X ± harus satu petak"
    assert p[4] == p[5], "tembok menghadap Y ± harus satu petak"


def test_permukaan_mendatar_tidak_terpecah_banyak_petak():
    """Normal tanah selalu ber-derau sedikit. Kalau petaknya tidak setara luas,
    derau itu menyebarkannya ke banyak petak dan tanah menerima jatah berlipat —
    persis yang harus dicegah."""
    rng = np.random.default_rng(0)
    n = np.column_stack([rng.normal(0, 0.03, 4000),
                         rng.normal(0, 0.03, 4000),
                         np.ones(4000)])
    banyak_tanah = len(np.unique(cm.normal_bin(n)))

    # bandingkan dengan tembok tegak yang deraunya sama besar
    t = np.column_stack([rng.normal(0, 0.03, 4000),
                         np.ones(4000),
                         rng.normal(0, 0.03, 4000)])
    banyak_tembok = len(np.unique(cm.normal_bin(t)))

    assert banyak_tanah <= 4, f"tanah tersebar ke {banyak_tanah} petak"
    assert banyak_tanah <= banyak_tembok + 2


def test_cuplikan_berimbang_mengangkat_arah_langka():
    """Yang mengunci geseran menyusuri tembok (sumbu X) cuma permukaan yang
    normalnya menghadap ±X — sisi samping huruf. Jumlahnya sedikit sekali
    dibanding muka tembok. Setelah diimbangi, perbandingannya harus naik tajam;
    itulah yang memberi ICP gradien ke arah yang tadinya buta."""
    import open3d as o3d

    xyz = wall_site()
    atlas = cm.plane_atlas(xyz)
    p = cm.to_o3d(xyz).voxel_down_sample(0.05)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))

    def bagi(pc):
        n = np.abs(np.asarray(pc.normals))
        tembok = int((n[:, 1] > 0.9).sum())        # muka tembok & muka huruf
        sisi = int((n[:, 0] > 0.9).sum())          # sisi samping huruf
        return sisi / max(tembok, 1)

    sebelum = bagi(p)
    sal = cm.salient_mask(np.asarray(p.points), atlas)
    q = cm.balanced_sample(p, sal, maks=4000)
    sesudah = bagi(q)

    assert len(q.points) <= 4000
    assert sesudah > sebelum * 2.5, (
        f"perbandingan sisi:tembok cuma naik {sebelum:.3f} → {sesudah:.3f}")


def test_cuplikan_berimbang_membiarkan_awan_kecil_utuh():
    import open3d as o3d
    p = cm.to_o3d(wall_site()).voxel_down_sample(0.5)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
    n = len(p.points)
    q = cm.balanced_sample(p, np.zeros(n, bool), maks=n + 10)
    assert len(q.points) == n


def test_round_robin_memenuhi_kuota_walau_petak_timpang():
    rng = np.random.default_rng(0)
    idx = np.arange(1000)
    petak = np.zeros(1000, dtype=int)
    petak[:990] = 0          # satu petak raksasa
    petak[990:] = np.arange(1, 11)   # sepuluh petak isi satu
    out = cm._round_robin(idx, petak, 50, rng)

    assert len(out) == 50
    # sepuluh petak kecil harus semuanya terwakili, bukan tenggelam
    assert (petak[out] > 0).sum() == 10


# ═══════════════════════════════════════════════════════════════════════════════
# Arah lemah
# ═══════════════════════════════════════════════════════════════════════════════

def test_arah_lemah_menunjuk_menyusuri_tembok():
    """Tembok membentang di sumbu X, jadi arah tak terkunci harus X."""
    xyz = wall_site(huruf=False)          # tanpa huruf: benar-benar tak terkunci
    pyr = cm.pyramid(xyz)
    w = cm.weak_direction(pyr, pyr, np.eye(4))

    assert w["n"] > 50
    a = np.abs(w["arah"] / np.linalg.norm(w["arah"]))
    assert a[0] > 0.8, f"arah lemah {w['arah'].round(2)} seharusnya menyusuri X"
    assert w["ratio"] < cm.WEAK_RATIO


def test_huruf_mengunci_arah_yang_tadinya_lemah():
    """Dengan huruf, arah X harus jauh lebih terkunci daripada tanpa huruf."""
    tanpa = cm.weak_direction(*(2 * [cm.pyramid(wall_site(huruf=False))]), np.eye(4))
    dengan = cm.weak_direction(*(2 * [cm.pyramid(wall_site(huruf=True))]), np.eye(4))
    assert dengan["ratio"] > tanpa["ratio"] * 2


def test_describe_weak_bisa_dibaca():
    w = {"n": 100, "ratio": 0.01, "arah": np.array([1.0, 0, 0]),
         "putar": np.array([0.0, 0, 0])}
    s = cm.describe_weak(w)
    assert "geser mendatar" in s and "0.01" in s
    assert cm.describe_weak({"n": 0}) == "tak terhitung"


# ═══════════════════════════════════════════════════════════════════════════════
# Pelepasan geseran — pembetulan "FILFILKOM"
# ═══════════════════════════════════════════════════════════════════════════════

def test_arah_sapuan_diambil_dari_tangen_tembok():
    """Tembok membentang di X, normalnya Y → arah sapuan harus X, dan MENDATAR.

    Ini pelajaran dari kegagalan di data asli: dulu arahnya diambil dari
    Hessian, dan Hessian menunjuk ke ATAS — padahal tinggi sudah terkunci oleh
    perataan tanah dan penyakitnya ada di bidang mendatar.
    """
    atlas = cm.plane_atlas(wall_site())
    lemah = {"n": 500, "ratio": 0.01, "arah": np.array([0.0, 0.0, 1.0]),
             "putar": np.zeros(3)}
    arah = cm.slide_directions(atlas, lemah, cm.WEAK_RATIO)

    assert arah, "tembok ada, tapi tak satu pun arah sapuan diusulkan"
    assert all(abs(a[2]) < 1e-9 for a in arah), "arah sapuan harus mendatar"
    assert any(abs(a[0]) > 0.9 for a in arah), (
        f"tak ada yang menyusuri X: {[a.round(2) for a in arah]}")


def test_arah_sapuan_tidak_kembar():
    """Dua tembok sejajar tidak boleh menghasilkan dua arah yang sama."""
    atlas = [np.array([0.0, 1.0, 0.0, 0.0]),
             np.array([0.0, -1.0, 0.0, 5.0]),      # sejajar, hadap sebaliknya
             np.array([1.0, 0.0, 0.0, 2.0])]       # tegak lurus
    arah = cm.slide_directions(atlas, {"n": 0}, cm.WEAK_RATIO)
    assert len(arah) == 2


def test_tanpa_tembok_tak_ada_yang_disapu():
    """Tempat tanpa bidang tegak tidak mengidap penyakit ini — jangan buang waktu."""
    atlas = [np.array([0.0, 0.0, 1.0, 0.0])]       # tanah saja
    assert cm.slide_directions(atlas, {"n": 0}, cm.WEAK_RATIO) == []


@pytest.mark.parametrize("salah", [0.35, -0.5, 0.8])
def test_unslide_mengembalikan_geseran_yang_meleset(salah):
    """Beri pose yang sengaja tergelincir menyusuri tembok; harus dikembalikan."""
    ref = wall_site(seed=0)
    src = wall_site(seed=7)               # titik dicuplik beda, tempat sama

    atlas = cm.plane_atlas(ref)
    ref_pyr, src_pyr = cm.pyramid(ref, atlas), cm.pyramid(src)
    sal, rc = cm.salient_cloud(src, cm.plane_atlas(src)), cm.ref_cloud(ref)

    T = np.eye(4)
    T[0, 3] = salah
    T2, catatan = cm.unslide(src_pyr, ref_pyr, sal, rc, T, Args(), atlas)

    assert catatan["disapu"], "arah menyusuri tembok seharusnya disapu"
    assert abs(T2[0, 3]) < 0.06, (
        f"masih meleset {T2[0, 3]:.3f} m dari {salah:+.2f} m")

    f_sebelum, _ = cm.sharp_score(sal, rc, T)
    f_sesudah, _ = cm.sharp_score(sal, rc, T2)
    assert f_sesudah > f_sebelum


def test_unslide_tidak_merusak_pose_yang_sudah_benar():
    ref, src = wall_site(seed=0), wall_site(seed=7)
    atlas = cm.plane_atlas(ref)
    ref_pyr, src_pyr = cm.pyramid(ref, atlas), cm.pyramid(src)
    sal, rc = cm.salient_cloud(src, cm.plane_atlas(src)), cm.ref_cloud(ref)

    T2, _ = cm.unslide(src_pyr, ref_pyr, sal, rc, np.eye(4), Args(), atlas)
    assert np.linalg.norm(T2[:3, 3]) < 0.05


def test_unslide_dilewati_bila_tak_ada_tembok():
    """Tanpa bidang tegak dan tanpa arah lemah, jangan buang waktu menyapu."""
    ref = wall_site(seed=0)
    pyr = cm.pyramid(ref)
    sal, rc = cm.salient_cloud(ref, cm.plane_atlas(ref)), cm.ref_cloud(ref)

    hanya_tanah = [np.array([0.0, 0.0, 1.0, 0.0])]
    args = Args(weak_ratio=0.0)           # arah Hessian pun tak pernah dipakai
    T2, catatan = cm.unslide(pyr, pyr, sal, rc, np.eye(4), args, hanya_tanah)
    assert not catatan["disapu"]
    assert np.allclose(T2, np.eye(4))


def test_pick_peaks_menyatukan_puncak_berdekatan():
    offs = np.array([-1.0, -0.98, -0.5, 0.0, 0.02, 0.9])
    nilai = np.array([0.9, 0.89, 0.2, 0.95, 0.94, 0.3])
    p = cm.pick_peaks(offs, nilai, 3, nms=0.25)
    assert len(p) == 3
    assert pytest.approx(0.0, abs=1e-9) == p[0]
    assert -1.0 in p                      # puncak -0.98 tertelan oleh -1.0
    assert 0.02 not in p


# ═══════════════════════════════════════════════════════════════════════════════
# Registrasi utuh
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.slow
def test_registrasi_menemukan_putaran_dan_letak_yang_benar():
    ref = wall_site(seed=0)
    T_asli = om.yaw_matrix(25.0)
    T_asli[:3, 3] = (1.2, -0.7, 0.0)
    src = cm.apply_transform(wall_site(seed=7), np.linalg.inv(T_asli))

    args = Args(step_deg=2.0, seeds=10)
    atlas = cm.plane_atlas(ref)
    h = cm.register(src, ref,
                    cm.salient_cloud(src, cm.plane_atlas(src)),
                    cm.ref_cloud(ref), args,
                    cm.pyramid(src), cm.pyramid(ref, atlas), atlas,
                    verbose=False)

    beda = np.linalg.inv(T_asli) @ np.asarray(h["T"])
    assert np.linalg.norm(beda[:3, 3]) < 0.10, (
        f"letak meleset {np.linalg.norm(beda[:3, 3]):.3f} m")
    assert h["verdict"] == BAIK
