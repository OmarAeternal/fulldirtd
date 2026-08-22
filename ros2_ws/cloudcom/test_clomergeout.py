"""Tests untuk clomergeout.py — registrasi melingkar searah untuk scan outdoor.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_clomergeout.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import numpy as np
import pytest

import clomcap
import clomerge
import clomergeout as cmo
import make_grid


# ═══════════════════════════════════════════════════════════════════════════════
# Helper
# ═══════════════════════════════════════════════════════════════════════════════

def site_cloud(seed=0) -> np.ndarray:
    """Lokasi luar ruangan sintetis: tanah, dua tembok tegak, satu tiang.

    Sengaja tidak simetris supaya registrasi punya cukup ciri untuk mengunci
    satu-satunya orientasi yang benar.
    """
    rng = np.random.default_rng(seed)
    tanah = np.column_stack([rng.uniform(-10, 10, 4000), rng.uniform(-10, 10, 4000),
                             np.zeros(4000)])
    tembok_a = np.column_stack([np.full(2500, -8.0), rng.uniform(-10, 4, 2500),
                                rng.uniform(0, 4, 2500)])
    tembok_b = np.column_stack([rng.uniform(-8, 6, 2500), np.full(2500, 7.0),
                                rng.uniform(0, 3, 2500)])
    tiang = np.column_stack([rng.uniform(3, 3.6, 1200), rng.uniform(-4, -3.4, 1200),
                             rng.uniform(0, 6, 1200)])
    return np.vstack([tanah, tembok_a, tembok_b, tiang])


def write_ply_cloud(path, xyz) -> str:
    xyz = np.asarray(xyz, dtype=np.float32)
    make_grid.write_ply(str(path), xyz, np.zeros((len(xyz), 3), dtype=np.uint8))
    return str(path)


def rigid(rz_deg=0.0, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = cmo.yaw_matrix(rz_deg)
    T[:3, 3] = t
    return T


# ═══════════════════════════════════════════════════════════════════════════════
# scan_index / order_by_index
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("name,expected", [
    ("scan_0003_1sweep_0.mcap", 3),
    ("/x/y/scan_0010_1sweep_0.mcap", 10),
    ("016_0.ply", 16),
    ("scan_0004_5sweep_0.mcap.zstd", 4),
])
def test_scan_index(name, expected):
    assert cmo.scan_index(name) == expected


def test_scan_index_none_when_no_digit():
    assert cmo.scan_index("depan.ply") is None


def test_scan_index_ignores_digits_in_folder():
    """Angka pada folder induk tidak boleh terbaca sebagai indeks scan."""
    assert cmo.scan_index("/data/2026/sesi_9/scan_0007_1sweep_0.mcap") == 7


def test_order_by_index_sorts_ascending():
    files = ["scan_0006_1sweep_0.mcap", "scan_0003_1sweep_0.mcap",
             "scan_0010_1sweep_0.mcap", "scan_0004_1sweep_0.mcap"]
    urut = cmo.order_by_index(files)
    assert [cmo.scan_index(f) for f in urut] == [3, 4, 6, 10]


def test_order_by_index_is_numeric_not_lexicographic():
    """scan_10 harus setelah scan_9, bukan sebelum seperti urutan teks."""
    urut = cmo.order_by_index(["scan_0010.ply", "scan_0009.ply"])
    assert [cmo.scan_index(f) for f in urut] == [9, 10]


def test_order_by_index_puts_unnumbered_last_in_original_order():
    urut = cmo.order_by_index(["b.ply", "scan_0002.ply", "a.ply"])
    assert urut == ["scan_0002.ply", "b.ply", "a.ply"]


def test_order_by_index_empty():
    assert cmo.order_by_index([]) == []


# ═══════════════════════════════════════════════════════════════════════════════
# yaw_matrix / seed_transform / seed_angles
# ═══════════════════════════════════════════════════════════════════════════════

def test_yaw_matrix_rotates_about_z_only():
    T = cmo.yaw_matrix(90)
    got = clomerge.apply_transform(np.array([[1.0, 0.0, 5.0]]), T)
    assert np.allclose(got, [[0.0, 1.0, 5.0]], atol=1e-9)


def test_yaw_matrix_zero_is_identity():
    assert np.allclose(cmo.yaw_matrix(0), np.eye(4))


def test_yaw_matrix_360_is_identity():
    assert np.allclose(cmo.yaw_matrix(360), np.eye(4), atol=1e-9)


def test_seed_transform_aligns_centroids():
    ref = site_cloud(seed=1)
    src = clomerge.apply_transform(ref, rigid(rz_deg=90, t=(3.0, -2.0, 0.0)))
    T = cmo.seed_transform(src, ref, -90.0)
    kembali = clomerge.apply_transform(src, T)
    assert np.allclose(kembali.mean(axis=0), ref.mean(axis=0), atol=1e-6)


def test_seed_angles_single_seed_returns_guess_only():
    assert cmo.seed_angles(90.0, 25.0, 1) == [90.0]


def test_seed_angles_spans_spread_symmetrically():
    a = cmo.seed_angles(90.0, 20.0, 5)
    assert len(a) == 5
    assert a[0] == pytest.approx(70.0)
    assert a[-1] == pytest.approx(110.0)
    assert a[2] == pytest.approx(90.0)


def test_seed_angles_zero_spread_returns_guess_only():
    assert cmo.seed_angles(90.0, 0.0, 5) == [90.0]


# ═══════════════════════════════════════════════════════════════════════════════
# se3_scale
# ═══════════════════════════════════════════════════════════════════════════════

def test_se3_scale_zero_is_identity():
    T = rigid(rz_deg=40, t=(2.0, 3.0, 1.0))
    assert np.allclose(cmo.se3_scale(T, 0.0), np.eye(4), atol=1e-9)


def test_se3_scale_one_returns_original():
    T = rigid(rz_deg=40, t=(2.0, 3.0, 1.0))
    assert np.allclose(cmo.se3_scale(T, 1.0), T, atol=1e-9)


def test_se3_scale_half_halves_the_angle():
    """Sudutnya separuh. Translasinya TIDAK separuh secara lurus — rotasi ikut
    membawa translasi, jadi yang dijamin adalah H∘H = T (diuji terpisah)."""
    T = rigid(rz_deg=40, t=(2.0, 4.0, 6.0))
    H = cmo.se3_scale(T, 0.5)
    assert cmo.residual_size(H)[1] == pytest.approx(20.0, abs=1e-6)
    assert H[2, 3] == pytest.approx(3.0)          # sumbu putar → murni separuh


def test_se3_scale_composes_back_to_whole():
    T = rigid(rz_deg=30, t=(1.0, 0.0, 0.0))
    H = cmo.se3_scale(T, 0.5)
    assert np.allclose(H @ H, T, atol=1e-9)


def test_se3_scale_identity_input_stays_identity():
    assert np.allclose(cmo.se3_scale(np.eye(4), 0.4), np.eye(4), atol=1e-12)


# ═══════════════════════════════════════════════════════════════════════════════
# chain_poses / loop_error / distribute_loop_error
# ═══════════════════════════════════════════════════════════════════════════════

def test_chain_poses_first_is_identity():
    poses = cmo.chain_poses([rigid(rz_deg=90), rigid(rz_deg=90)])
    assert np.allclose(poses[0], np.eye(4))
    assert len(poses) == 3


def test_chain_poses_composes_in_order():
    poses = cmo.chain_poses([rigid(rz_deg=90), rigid(rz_deg=90)])
    assert np.allclose(poses[2], cmo.yaw_matrix(180), atol=1e-9)


def test_loop_error_is_identity_for_perfect_ring():
    """Empat langkah 90° yang sempurna harus kembali persis ke titik awal."""
    rel = [rigid(rz_deg=90)] * 3
    poses = cmo.chain_poses(rel)
    E = cmo.loop_error(poses, rigid(rz_deg=90))
    assert np.allclose(E, np.eye(4), atol=1e-9)


def test_loop_error_detects_drift():
    rel = [rigid(rz_deg=92)] * 3          # tiap langkah meleset 2°
    poses = cmo.chain_poses(rel)
    E = cmo.loop_error(poses, rigid(rz_deg=92))
    geser, sudut = cmo.residual_size(E)
    assert sudut == pytest.approx(8.0, abs=0.1)


def test_distribute_loop_error_shrinks_residual():
    """Inti loop closure: sisa error harus mengecil setelah dibagi rata."""
    rel = [rigid(rz_deg=92, t=(0.05, 0.0, 0.0))] * 3
    closing = rigid(rz_deg=92, t=(0.05, 0.0, 0.0))
    poses = cmo.chain_poses(rel)

    E = cmo.loop_error(poses, closing)
    poses2 = cmo.distribute_loop_error(poses, E)
    koreksi = cmo.loop_correction(E, len(poses))
    E2 = cmo.loop_error(poses2, closing @ koreksi)

    g0, s0 = cmo.residual_size(E)
    g1, s1 = cmo.residual_size(E2)
    assert s1 < s0
    assert g1 < g0


def test_distribute_loop_error_keeps_first_pose_identity():
    rel = [rigid(rz_deg=95)] * 3
    poses = cmo.chain_poses(rel)
    E = cmo.loop_error(poses, rigid(rz_deg=95))
    assert np.allclose(cmo.distribute_loop_error(poses, E)[0], np.eye(4))


def test_distribute_loop_error_noop_on_perfect_ring():
    rel = [rigid(rz_deg=90)] * 3
    poses = cmo.chain_poses(rel)
    E = cmo.loop_error(poses, rigid(rz_deg=90))
    poses2 = cmo.distribute_loop_error(poses, E)
    for a, b in zip(poses, poses2):
        assert np.allclose(a, b, atol=1e-8)


def test_residual_size_identity_is_zero():
    g, s = cmo.residual_size(np.eye(4))
    assert g == pytest.approx(0.0)
    assert s == pytest.approx(0.0)


def test_residual_size_reports_translation_and_angle():
    g, s = cmo.residual_size(rigid(rz_deg=30, t=(3.0, 4.0, 0.0)))
    assert g == pytest.approx(5.0)
    assert s == pytest.approx(30.0, abs=1e-6)


# ═══════════════════════════════════════════════════════════════════════════════
# register_pair_seeded — inti "sudutnya boleh tidak pas"
# ═══════════════════════════════════════════════════════════════════════════════

def test_register_pair_seeded_recovers_exact_angle():
    ref = site_cloud(seed=2)
    src = clomerge.apply_transform(ref, rigid(rz_deg=90, t=(2.0, 1.0, 0.0)))
    T, fitness, rmse = cmo.register_pair_seeded(src, ref, -90.0, voxel=0.3)
    assert clomerge.quality_verdict(fitness, rmse) == clomerge.BAIK
    kembali = clomerge.apply_transform(src, T)
    assert np.allclose(kembali.mean(axis=0), ref.mean(axis=0), atol=0.2)


def test_register_pair_seeded_tolerates_off_angle():
    """Sudut sebenarnya 104°, tebakan 90° — harus tetap terkunci benar."""
    ref = site_cloud(seed=3)
    src = clomerge.apply_transform(ref, rigid(rz_deg=104, t=(1.5, -1.0, 0.0)))
    T, fitness, rmse = cmo.register_pair_seeded(src, ref, -90.0, voxel=0.3)
    assert clomerge.quality_verdict(fitness, rmse) == clomerge.BAIK
    kembali = clomerge.apply_transform(src, T)
    assert np.allclose(kembali.mean(axis=0), ref.mean(axis=0), atol=0.3)


def test_register_pair_seeded_identical_clouds():
    ref = site_cloud(seed=4)
    T, fitness, _ = cmo.register_pair_seeded(ref.copy(), ref, 0.0, voxel=0.3)
    assert fitness > 0.9
    assert np.allclose(T, np.eye(4), atol=0.1)


# ═══════════════════════════════════════════════════════════════════════════════
# run()
# ═══════════════════════════════════════════════════════════════════════════════

def _ring(tmp_path, seed, langkah=90.0, n=4):
    """n scan mengelilingi lokasi yang sama, tiap langkah `langkah` derajat."""
    dasar = site_cloud(seed=seed)
    paths = []
    for i in range(n):
        # tiap scan direkam dari kerangkanya sendiri → dasar diputar balik
        T = rigid(rz_deg=-langkah * i, t=(0.0, 0.0, 0.0))
        paths.append(write_ply_cloud(tmp_path / f"scan_{i:04d}.ply",
                                     clomerge.apply_transform(dasar, T)))
    return paths


def test_run_rejects_fewer_than_three(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 5, n=4)[:2]
    with pytest.raises(SystemExit):
        cmo.run(cmo.build_parser().parse_args(p))


def test_run_writes_outputs_into_numbered_slot(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 6)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3"]))

    slot = tmp_path / "out" / cmo.MERGE_DIRNAME / "001"
    assert (slot / "merged.ply").is_file()
    assert (slot / "merged_check.ply").is_file()
    assert (slot / "grid.ply").is_file()
    assert (slot / "transforms.txt").is_file()


def test_run_uses_separate_dir_from_clomerge(tmp_path, monkeypatch):
    """Hasil outdoor tidak boleh tercampur dengan hasil clomerge biasa."""
    assert cmo.MERGE_DIRNAME != clomerge.MERGE_DIRNAME


def test_run_orders_by_index_not_argv(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 7)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    terbalik = list(reversed(p))
    cmo.run(cmo.build_parser().parse_args(terbalik + ["--voxel", "0.3"]))

    teks = (tmp_path / "out" / cmo.MERGE_DIRNAME / "001" / "transforms.txt").read_text()
    urut = [b.split(". ")[1] for b in teks.splitlines()
            if b.startswith("#   ") and ". scan_" in b]
    assert urut == ["scan_0000.ply", "scan_0001.ply", "scan_0002.ply", "scan_0003.ply"]


def test_run_merged_has_all_points(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 8)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3"]))

    merged = tmp_path / "out" / cmo.MERGE_DIRNAME / "001" / "merged.ply"
    n = len(clomerge.read_cloud_xyz(str(merged)))
    assert n == 4 * len(site_cloud(seed=8))


def test_run_report_records_loop_closure(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 9)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3"]))

    teks = (tmp_path / "out" / cmo.MERGE_DIRNAME / "001" / "transforms.txt").read_text()
    assert "PENUTUP" in teks
    assert "Error lingkaran" in teks


def test_run_no_loop_skips_closing_edge(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 10)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3", "--no-loop"]))

    teks = (tmp_path / "out" / cmo.MERGE_DIRNAME / "001" / "transforms.txt").read_text()
    assert "PENUTUP" not in teks


def test_run_opens_grid_first(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 11)

    opened = {}
    monkeypatch.setattr(cmo, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3"]))

    assert opened["files"][0].endswith("grid.ply")
    assert len(opened["files"]) == 3


def test_run_no_open_skips_cloudcompare(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 12)
    monkeypatch.setattr(cmo, "launch_cloudcompare",
                        lambda files: pytest.fail("tidak boleh membuka CloudCompare"))
    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3", "--no-open"]))


def test_run_levels_ground_to_z_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    p = _ring(tmp_path, 13)
    monkeypatch.setattr(cmo, "launch_cloudcompare", lambda files: None)

    cmo.run(cmo.build_parser().parse_args(p + ["--voxel", "0.3"]))

    merged = tmp_path / "out" / cmo.MERGE_DIRNAME / "001" / "merged.ply"
    xyz = clomerge.read_cloud_xyz(str(merged))
    tanah = xyz[np.abs(xyz[:, 2]) < 0.05]
    assert len(tanah) > 1000


def test_parser_defaults():
    a = cmo.build_parser().parse_args(["a.ply", "b.ply", "c.ply"])
    assert a.step_deg is None
    assert a.spread == cmo.DEFAULT_SPREAD
    assert a.seeds == cmo.DEFAULT_SEEDS
    assert a.no_loop is False
    assert a.no_level is False
    assert a.no_grid is False


def test_se3_scale_quarter_composes_four_times():
    """Sifat yang menjamin pembagian error benar: 4 × seperempat = utuh."""
    T = rigid(rz_deg=37, t=(2.5, -1.5, 0.7))
    Q = cmo.se3_scale(T, 0.25)
    assert np.allclose(Q @ Q @ Q @ Q, T, atol=1e-9)


def test_se3_log_exp_roundtrip():
    T = rigid(rz_deg=140, t=(3.0, -2.0, 1.0))
    w, v = cmo.se3_log(T)
    assert np.allclose(cmo.se3_exp(w, v), T, atol=1e-9)


def test_se3_log_pure_translation():
    T = rigid(rz_deg=0, t=(1.0, 2.0, 3.0))
    w, v = cmo.se3_log(T)
    assert np.allclose(w, 0.0)
    assert np.allclose(v, [1.0, 2.0, 3.0])
