"""Tests untuk outmerge.py — peta tumbuh untuk scan luar ruangan.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_outmerge.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import numpy as np
import pytest

import outmerge as om
from clomerge import BAIK, RAGU, GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# Helper
# ═══════════════════════════════════════════════════════════════════════════════

def site_cloud(seed=0, n_tanah=6000) -> np.ndarray:
    """Lokasi luar ruangan sintetis: tanah, dua tembok tegak, satu tiang.

    Sengaja tidak simetris supaya pencocokan punya cukup ciri untuk mengunci
    satu-satunya sudut yang benar.
    """
    rng = np.random.default_rng(seed)
    tanah = np.column_stack([rng.uniform(-8, 8, n_tanah),
                             rng.uniform(-8, 8, n_tanah),
                             rng.normal(0, 0.005, n_tanah)])
    tembok_a = np.column_stack([np.full(3000, -6.0), rng.uniform(-7, 2, 3000),
                                rng.uniform(0.1, 3.5, 3000)])
    tembok_b = np.column_stack([rng.uniform(-6, 4, 3000), np.full(3000, 5.0),
                                rng.uniform(0.1, 2.8, 3000)])
    tiang = np.column_stack([rng.uniform(2.4, 2.8, 1500),
                             rng.uniform(-3.2, -2.8, 1500),
                             rng.uniform(0.1, 4.0, 1500)])
    return np.vstack([tanah, tembok_a, tembok_b, tiang])


def rigid(rz_deg=0.0, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = om.yaw_matrix(rz_deg)
    T[:3, 3] = t
    return T


def tilt_matrix(deg, axis="x") -> np.ndarray:
    """Miringkan cloud terhadap sumbu mendatar — tiruan tripod tidak rata."""
    a = np.radians(deg)
    T = np.eye(4)
    if axis == "x":
        T[:3, :3] = [[1, 0, 0],
                     [0, np.cos(a), -np.sin(a)],
                     [0, np.sin(a), np.cos(a)]]
    else:
        T[:3, :3] = [[np.cos(a), 0, np.sin(a)],
                     [0, 1, 0],
                     [-np.sin(a), 0, np.cos(a)]]
    return T


class Args:
    """Pengganti hasil argparse untuk memanggil fungsi tingkat tinggi."""
    def __init__(self, **kw):
        self.step_deg = om.DEFAULT_STEP_DEG
        self.seeds = om.DEFAULT_SEEDS
        self.range = om.DEFAULT_RANGE
        self.rounds = 1
        self.__dict__.update(kw)


# ═══════════════════════════════════════════════════════════════════════════════
# Dasar geometri
# ═══════════════════════════════════════════════════════════════════════════════

def test_yaw_matrix_rotates_about_z_only():
    got = om.apply_transform(np.array([[1.0, 0.0, 5.0]]), om.yaw_matrix(90))
    assert np.allclose(got, [[0.0, 1.0, 5.0]], atol=1e-9)


def test_yaw_matrix_zero_is_identity():
    assert np.allclose(om.yaw_matrix(0), np.eye(4))


@pytest.mark.parametrize("deg", [0.0, 37.5, 180.0, 271.25, 359.0])
def test_yaw_of_recovers_angle(deg):
    assert om.yaw_of(om.yaw_matrix(deg)) == pytest.approx(deg, abs=1e-6)


def test_yaw_of_wraps_into_0_360():
    assert om.yaw_of(om.yaw_matrix(-90)) == pytest.approx(270.0, abs=1e-6)


def test_apply_transform_composes_like_matrix_product():
    x = site_cloud(seed=3)[:200]
    A, B = rigid(30, (1, 2, 3)), rigid(-70, (-4, 0.5, 1))
    assert np.allclose(om.apply_transform(om.apply_transform(x, A), B),
                       om.apply_transform(x, B @ A), atol=1e-9)


# ═══════════════════════════════════════════════════════════════════════════════
# crop_range
# ═══════════════════════════════════════════════════════════════════════════════

def test_crop_range_keeps_only_inside_sphere():
    x = np.array([[0, 0, 0], [3, 4, 0], [3, 4, 0.1], [10, 0, 0]], float)
    keluar = om.crop_range(x, 5.0)
    # (3,4,0) tepat 5.0 m ikut; (3,4,0.1) sedikit di luar; (10,0,0) jelas di luar
    assert len(keluar) == 2
    assert np.allclose(keluar[1], [3, 4, 0])


def test_crop_range_is_spherical_not_box():
    """Sudut kotak harus terbuang: (5,5,0) berjarak 7.07 m, bukan 5."""
    x = np.array([[5.0, 5.0, 0.0]])
    assert len(om.crop_range(x, 6.0)) == 0


@pytest.mark.parametrize("radius", [0, 0.0, None])
def test_crop_range_disabled_returns_everything(radius):
    x = site_cloud(seed=1)
    assert len(om.crop_range(x, radius)) == len(x)


# ═══════════════════════════════════════════════════════════════════════════════
# Perataan tanah
# ═══════════════════════════════════════════════════════════════════════════════

def test_ground_plane_finds_upward_normal():
    n, d, frac = om.ground_plane(site_cloud(seed=0))
    assert n[2] > 0.99
    assert abs(d) < 0.05
    assert frac > 0.5


def test_ground_plane_normal_points_up_even_for_scan_below_ground():
    """Cloud dengan tanah di atas sensor tetap memberi normal menghadap atas."""
    x = site_cloud(seed=0)
    x[:, 2] += 2.0
    n, _, _ = om.ground_plane(x)
    assert n[2] > 0.99


def test_ground_plane_returns_none_when_too_few_points():
    assert om.ground_plane(np.zeros((10, 3))) is None


@pytest.mark.parametrize("deg,axis", [(9.0, "x"), (5.5, "y"), (12.0, "x")])
def test_level_transform_removes_tilt(deg, axis):
    miring = om.apply_transform(site_cloud(seed=2), tilt_matrix(deg, axis))
    L = om.level_transform(miring)
    assert L is not None
    assert om.tilt_deg(L) == pytest.approx(deg, abs=0.5)


def test_level_transform_puts_ground_at_zero():
    x = om.apply_transform(site_cloud(seed=2), tilt_matrix(7.0))
    x[:, 2] -= 1.3                       # sensor 1,3 m di atas tanah
    rata = om.apply_transform(x, om.level_transform(x))
    tanah = rata[rata[:, 2] < 0.2]
    assert abs(np.median(tanah[:, 2])) < 0.02


def test_level_transform_leaves_yaw_alone():
    """Perataan tidak boleh ikut memutar tegak — itu tugas pencocokan.

    Dibandingkan melingkar: 359,99° sama saja dengan 0,01°.
    """
    L = om.level_transform(om.apply_transform(site_cloud(seed=2), tilt_matrix(8.0)))
    y = om.yaw_of(L)
    assert min(y, 360.0 - y) < 2.0


def test_level_transform_none_without_ground():
    """Hanya tembok tegak, tanpa tanah → tidak ada bidang mendatar yang sah."""
    rng = np.random.default_rng(0)
    tembok = np.column_stack([np.zeros(3000), rng.uniform(-5, 5, 3000),
                              rng.uniform(0, 4, 3000)])
    assert om.level_transform(tembok) is None


def test_tilt_deg_of_none_is_zero():
    assert om.tilt_deg(None) == 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# Citra tampak-atas & korelasi fase
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("half,cell,expected", [(6.0, 0.1, 128), (12.0, 0.1, 256),
                                                (3.0, 0.1, 64)])
def test_bev_size_is_power_of_two(half, cell, expected):
    n = om.bev_size(half, cell)
    assert n == expected
    assert n & (n - 1) == 0


def test_bev_image_ignores_ground():
    """Titik di bawah BEV_ZMIN tidak boleh menyumbang apa pun ke citra."""
    rng = np.random.default_rng(0)
    tanah = np.column_stack([rng.uniform(-3, 3, 2000), rng.uniform(-3, 3, 2000),
                             np.zeros(2000)])
    assert np.allclose(om.bev_image(tanah, 0.1, 64), 0.0)


def test_bev_image_is_zero_mean():
    img = om.bev_image(site_cloud(seed=1), 0.1, 128)
    assert abs(img.mean()) < 1e-9


def test_bev_image_drops_points_outside_canvas():
    """Titik di luar bingkai dibuang, bukan melipat ke sisi seberang."""
    jauh = np.array([[100.0, 100.0, 1.0]])
    assert np.allclose(om.bev_image(jauh, 0.1, 64), 0.0)


@pytest.mark.parametrize("dx,dy", [(0, 0), (7, -4), (-11, 3)])
def test_phase_correlate_recovers_shift(dx, dy):
    rng = np.random.default_rng(0)
    a = rng.normal(size=(64, 64))
    a -= a.mean()
    b = np.roll(np.roll(a, -dy, axis=0), -dx, axis=1)
    got_dy, got_dx, pk = om.phase_correlate(a, b)
    assert (got_dy, got_dx) == (dy, dx)
    assert pk > 0.5


def test_phase_correlate_is_insensitive_to_density():
    """Sisi yang jauh lebih padat tidak boleh menggeser jawabannya.

    Inilah alasan korelasi fase dipakai: satu sisi selalu peta gabungan yang
    padat, sisi lain hanya satu scan.
    """
    rng = np.random.default_rng(1)
    a = rng.normal(size=(64, 64))
    a -= a.mean()
    b = np.roll(a, -5, axis=1) * 0.01
    _, got_dx, _ = om.phase_correlate(a, b)
    assert got_dx == 5


def test_bev_seeds_finds_true_rotation():
    ref = site_cloud(seed=5)
    src = om.apply_transform(ref, rigid(rz_deg=-40.0, t=(1.5, -0.5, 0.0)))
    seeds = om.bev_seeds(src, ref, step_deg=2.0, seeds=6)
    sudut = [s[1] for s in seeds]
    assert any(min(abs(a - 40.0), 360 - abs(a - 40.0)) < 4.0 for a in sudut)


def test_bev_seeds_top_peak_leads_the_list():
    ref = site_cloud(seed=5)
    src = om.apply_transform(ref, rigid(rz_deg=25.0, t=(0.0, 0.0, 0.0)))
    seeds = om.bev_seeds(src, ref, step_deg=2.0, seeds=5)
    assert seeds == sorted(seeds, reverse=True)


def test_bev_seeds_respects_nms_spacing():
    ref = site_cloud(seed=5)
    src = om.apply_transform(ref, rigid(rz_deg=25.0))
    seeds = om.bev_seeds(src, ref, step_deg=2.0, seeds=6, nms_deg=15.0)
    sudut = [s[1] for s in seeds]
    for i, a in enumerate(sudut):
        for b in sudut[i + 1:]:
            assert min(abs(a - b), 360 - abs(a - b)) > 15.0


def test_bev_seeds_returns_at_most_requested():
    ref = site_cloud(seed=5)
    assert len(om.bev_seeds(ref, ref, step_deg=5.0, seeds=3)) <= 3


def test_seed_matrix_carries_yaw_and_planar_shift():
    T = om.seed_matrix(90.0, 2.0, -1.0)
    assert om.yaw_of(T) == pytest.approx(90.0, abs=1e-6)
    assert np.allclose(T[:3, 3], [2.0, -1.0, 0.0])


# ═══════════════════════════════════════════════════════════════════════════════
# Penilaian
# ═══════════════════════════════════════════════════════════════════════════════

def test_eval_subset_drops_ground():
    sub = np.asarray(om.eval_subset(site_cloud(seed=0)).points)
    assert len(sub) > 0
    assert (sub[:, 2] > om.EVAL_ZMIN).all()


def test_eval_subset_keeps_vertical_surfaces():
    """Tembok dan tiang harus lolos — merekalah yang menentukan sudut."""
    sub = np.asarray(om.eval_subset(site_cloud(seed=0)).points)
    assert (np.abs(sub[:, 0] + 6.0) < 0.2).sum() > 100      # tembok x = -6
    assert (np.abs(sub[:, 1] - 5.0) < 0.2).sum() > 100      # tembok y = +5


def test_eval_subset_falls_back_when_nothing_vertical():
    """Cloud tanpa permukaan tegak tetap memberi sesuatu untuk dinilai."""
    rng = np.random.default_rng(0)
    datar = np.column_stack([rng.uniform(-3, 3, 3000), rng.uniform(-3, 3, 3000),
                             np.full(3000, 2.0)])
    assert len(np.asarray(om.eval_subset(datar).points)) > 0


def test_score_perfect_when_identical():
    ev = om.eval_subset(site_cloud(seed=0))
    f, r = om.score(ev, ev, np.eye(4))
    assert f == pytest.approx(1.0)
    assert r < 1e-6


def test_score_drops_when_shifted_far():
    ev = om.eval_subset(site_cloud(seed=0))
    f, _ = om.score(ev, ev, rigid(t=(5.0, 5.0, 0.0)))
    assert f < 0.2


@pytest.mark.parametrize("f,r,expected", [
    (0.55, 0.04, BAIK),
    (om.FITNESS_BAIK, om.RMSE_BAIK, BAIK),
    (0.55, 0.09, RAGU),          # fitness cukup tapi simpangannya lebar
    (0.20, 0.04, RAGU),
    (0.10, 0.03, GAGAL),
])
def test_verdict(f, r, expected):
    assert om.verdict(f, r) == expected


# ═══════════════════════════════════════════════════════════════════════════════
# ICP & register
# ═══════════════════════════════════════════════════════════════════════════════

def test_pyramid_gets_coarser_at_each_level():
    pyr = om.pyramid(site_cloud(seed=0))
    jumlah = [len(p.points) for p in pyr]
    assert len(pyr) == len(om.ICP_SCALES)
    assert jumlah == sorted(jumlah)      # skala mengecil → titik bertambah
    assert all(p.has_normals() for p in pyr)


def test_icp_multi_polishes_a_near_guess():
    ref = site_cloud(seed=7)
    src = om.apply_transform(ref, rigid(rz_deg=12.0, t=(0.6, -0.4, 0.0)))
    benar = np.linalg.inv(rigid(rz_deg=12.0, t=(0.6, -0.4, 0.0)))
    T = om.icp_multi(om.pyramid(src), om.pyramid(ref),
                     rigid(rz_deg=-9.0, t=(-0.4, 0.3, 0.0)))
    assert np.allclose(T, benar, atol=0.05)


def test_register_recovers_known_transform():
    ref = site_cloud(seed=11)
    gerak = rigid(rz_deg=65.0, t=(2.0, -1.5, 0.0))
    src = om.apply_transform(ref, np.linalg.inv(gerak))

    h = om.register(src, ref, om.eval_subset(src), om.eval_subset(ref),
                    Args(), verbose=False)
    assert h["verdict"] == BAIK
    assert om.yaw_of(h["T"]) == pytest.approx(65.0, abs=2.0)
    assert np.allclose(h["T"][:2, 3], [2.0, -1.5], atol=0.15)


def test_register_margin_is_large_when_answer_is_unique():
    ref = site_cloud(seed=11)
    src = om.apply_transform(ref, rigid(rz_deg=-33.0, t=(1.0, 1.0, 0.0)))
    h = om.register(src, ref, om.eval_subset(src), om.eval_subset(ref),
                    Args(seeds=8), verbose=False)
    assert h["margin"] > 0.2


def test_register_identity_for_same_cloud():
    x = site_cloud(seed=4)
    ev = om.eval_subset(x)
    h = om.register(x, x, ev, ev, Args(seeds=6), verbose=False)
    assert np.allclose(h["T"], np.eye(4), atol=0.05)
    assert h["fitness"] > 0.9


# ═══════════════════════════════════════════════════════════════════════════════
# Peta yang tumbuh
# ═══════════════════════════════════════════════════════════════════════════════

def three_scan_site():
    """Tiga pandangan atas lokasi yang sama, tiap-tiap dari posisi berbeda."""
    dasar = site_cloud(seed=21)
    gerak = {0: rigid(0.0, (0.0, 0.0, 0.0)),
             1: rigid(50.0, (2.5, -1.0, 0.0)),
             2: rigid(-75.0, (-2.0, 2.0, 0.0))}
    clouds = {k: om.apply_transform(dasar, np.linalg.inv(T))
              for k, T in gerak.items()}
    return clouds, gerak


def test_grow_map_places_every_scan():
    clouds, gerak = three_scan_site()
    evals = {k: om.eval_subset(v) for k, v in clouds.items()}
    names = {k: f"scan_{k}.ply" for k in clouds}

    hasil = om.grow_map(clouds, evals, names, Args(seeds=8))
    assert set(hasil["poses"]) == set(clouds)
    assert all(c["verdict"] == BAIK for c in hasil["catatan"].values())


def test_grow_map_recovers_relative_geometry():
    """Pose yang dihasilkan boleh berbeda acuan, tapi hubungan antar-scan sama."""
    clouds, gerak = three_scan_site()
    evals = {k: om.eval_subset(v) for k, v in clouds.items()}
    names = {k: f"scan_{k}.ply" for k in clouds}

    poses = om.grow_map(clouds, evals, names, Args(seeds=8))["poses"]
    for a in clouds:
        for b in clouds:
            benar = np.linalg.inv(gerak[b]) @ gerak[a]
            dapat = np.linalg.inv(poses[b]) @ poses[a]
            assert np.allclose(dapat, benar, atol=0.15)


def test_grow_map_first_pose_is_identity_for_the_anchor():
    clouds, _ = three_scan_site()
    evals = {k: om.eval_subset(v) for k, v in clouds.items()}
    names = {k: f"scan_{k}.ply" for k in clouds}
    poses = om.grow_map(clouds, evals, names, Args(seeds=8))["poses"]
    assert sum(np.allclose(P, np.eye(4)) for P in poses.values()) == 1


def test_refine_all_never_lowers_fitness():
    """Perapian hanya boleh menerima pose yang skornya naik."""
    clouds, _ = three_scan_site()
    evals = {k: om.eval_subset(v) for k, v in clouds.items()}
    names = {k: f"scan_{k}.ply" for k in clouds}

    hasil = om.grow_map(clouds, evals, names, Args(seeds=8))
    poses, catatan, pyr = hasil["poses"], hasil["catatan"], hasil["pyr"]

    sebelum = {}
    for i in poses:
        lain = np.vstack([om.apply_transform(clouds[j], poses[j])
                          for j in poses if j != i])
        sebelum[i] = om.score(evals[i], om.eval_subset(lain), poses[i])[0]

    om.refine_all(clouds, evals, names, poses, catatan, pyr, rounds=1)

    for i in poses:
        lain = np.vstack([om.apply_transform(clouds[j], poses[j])
                          for j in poses if j != i])
        sesudah = om.score(evals[i], om.eval_subset(lain), poses[i])[0]
        assert sesudah >= sebelum[i] - 1e-9


def test_refine_all_is_noop_for_two_scans():
    """Dengan dua scan, "semua yang lain" hanyalah pasangannya — tak ada yang
    bisa ditambahkan, jadi pose dibiarkan."""
    clouds, _ = three_scan_site()
    clouds.pop(2)
    evals = {k: om.eval_subset(v) for k, v in clouds.items()}
    names = {k: f"scan_{k}.ply" for k in clouds}
    poses = {0: np.eye(4), 1: rigid(10.0, (1.0, 0.0, 0.0))}
    catatan = {k: {"verdict": BAIK, "lawan": "—"} for k in poses}
    salinan = {k: v.copy() for k, v in poses.items()}

    om.refine_all(clouds, evals, names, poses, catatan,
                  {k: om.pyramid(v) for k, v in clouds.items()}, rounds=1)
    for k in poses:
        assert np.allclose(poses[k], salinan[k])


# ═══════════════════════════════════════════════════════════════════════════════
# Laporan & CLI
# ═══════════════════════════════════════════════════════════════════════════════

def test_write_report_folds_level_into_the_matrix(tmp_path):
    """Matriks yang dicetak harus langsung berlaku pada scan ASLI yang miring."""
    L = om.tilt_matrix(0) if hasattr(om, "tilt_matrix") else np.eye(4)
    L = tilt_matrix(6.0)
    P = rigid(45.0, (1.0, 2.0, 0.0))
    p = tmp_path / "transforms.txt"
    om.write_report(p, [0], {0: "a.ply"},
                    {0: P}, {0: {"fitness": .5, "rmse": .04, "margin": .3,
                                 "verdict": BAIK, "lawan": "peta"}},
                    {0: L}, Args())

    isi = p.read_text()
    assert "a.ply" in isi and BAIK in isi
    gabungan = P @ L
    assert f"{gabungan[0, 0]: .6f}" in isi


def test_write_report_marks_scan_without_ground(tmp_path):
    p = tmp_path / "t.txt"
    om.write_report(p, [0], {0: "a.ply"}, {0: np.eye(4)},
                    {0: {"fitness": None, "rmse": None, "margin": None,
                         "verdict": RAGU, "lawan": "—"}},
                    {0: None}, Args())
    assert "tanah tidak ditemukan" in p.read_text()


def test_parser_defaults():
    a = om.build_parser().parse_args(["a.ply", "b.ply"])
    assert a.files == ["a.ply", "b.ply"]
    assert a.range == om.DEFAULT_RANGE
    assert a.step_deg == om.DEFAULT_STEP_DEG
    assert a.seeds == om.DEFAULT_SEEDS
    assert a.rounds == om.DEFAULT_ROUNDS


def test_parser_accepts_overrides():
    a = om.build_parser().parse_args(
        ["x.mcap", "y.mcap", "--range", "0", "--step-deg", "1", "--seeds", "30",
         "--rounds", "0", "--no-open"])
    assert (a.range, a.step_deg, a.seeds, a.rounds) == (0.0, 1.0, 30, 0)
    assert a.no_open


def test_run_rejects_single_file():
    with pytest.raises(SystemExit):
        om.run(om.build_parser().parse_args(["a.ply"]))
