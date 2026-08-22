"""Tests untuk clomerge.py — registrasi otomatis banyak scan → satu PLY.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_clomerge.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import numpy as np
import pytest

import clomcap
import clomerge
import make_grid


def rigid(rz_deg=0.0, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Matriks 4x4: rotasi rz_deg terhadap sumbu Z, lalu translasi t."""
    a = np.radians(rz_deg)
    T = np.eye(4)
    T[:3, :3] = np.array([[np.cos(a), -np.sin(a), 0],
                          [np.sin(a), np.cos(a), 0],
                          [0, 0, 1]])
    T[:3, 3] = t
    return T


def room_cloud(seed=0) -> np.ndarray:
    """Ruangan sintetis: lantai, dua dinding, dan satu kotak di tengah.

    Bentuknya sengaja tidak simetris supaya registrasi punya cukup ciri untuk
    mengunci satu-satunya jawaban yang benar.
    """
    rng = np.random.default_rng(seed)
    floor = np.column_stack([rng.uniform(0, 8, 3000), rng.uniform(0, 6, 3000),
                             np.zeros(3000)])
    wall_x = np.column_stack([np.zeros(2000), rng.uniform(0, 6, 2000),
                              rng.uniform(0, 3, 2000)])
    wall_y = np.column_stack([rng.uniform(0, 8, 2000), np.zeros(2000),
                              rng.uniform(0, 3, 2000)])
    box = np.column_stack([rng.uniform(5, 6, 1500), rng.uniform(4, 5, 1500),
                           rng.uniform(0, 1.5, 1500)])
    return np.vstack([floor, wall_x, wall_y, box])


def slot1(tmp_path):
    """Folder hasil merge run pertama: out/_merge/001."""
    return tmp_path / "out" / clomerge.MERGE_DIRNAME / "001"


# ═══════════════════════════════════════════════════════════════════════════════
# apply_transform
# ═══════════════════════════════════════════════════════════════════════════════

def test_apply_transform_identity_is_noop():
    xyz = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, 6.0]])
    assert np.allclose(clomerge.apply_transform(xyz, np.eye(4)), xyz)


def test_apply_transform_translation():
    xyz = np.array([[0.0, 0.0, 0.0]])
    got = clomerge.apply_transform(xyz, rigid(t=(1.0, 2.0, 3.0)))
    assert np.allclose(got, [[1.0, 2.0, 3.0]])


def test_apply_transform_rotation_90_deg():
    got = clomerge.apply_transform(np.array([[1.0, 0.0, 0.0]]), rigid(rz_deg=90))
    assert np.allclose(got, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_apply_transform_empty_input():
    got = clomerge.apply_transform(np.zeros((0, 3)), rigid(t=(1, 1, 1)))
    assert got.shape == (0, 3)


# ═══════════════════════════════════════════════════════════════════════════════
# quality_verdict
# ═══════════════════════════════════════════════════════════════════════════════

def test_quality_verdict_good():
    assert clomerge.quality_verdict(0.8, 0.02) == clomerge.BAIK


def test_quality_verdict_high_fitness_but_loose_rmse_is_not_good():
    assert clomerge.quality_verdict(0.8, 0.5) == clomerge.RAGU


def test_quality_verdict_middling_fitness():
    assert clomerge.quality_verdict(0.2, 0.02) == clomerge.RAGU


def test_quality_verdict_failure():
    assert clomerge.quality_verdict(0.05, 0.02) == clomerge.GAGAL


# ═══════════════════════════════════════════════════════════════════════════════
# pewarnaan
# ═══════════════════════════════════════════════════════════════════════════════

def test_color_by_scan_gives_each_scan_one_colour():
    rgb = clomerge.color_by_scan([2, 3])
    assert len(rgb) == 5
    assert len(set(map(tuple, rgb[:2]))) == 1
    assert len(set(map(tuple, rgb[2:]))) == 1
    assert tuple(rgb[0]) != tuple(rgb[2])


def test_color_by_scan_wraps_palette():
    counts = [1] * (len(clomerge.SCAN_COLORS) + 1)
    rgb = clomerge.color_by_scan(counts)
    assert tuple(rgb[0]) == tuple(rgb[-1])


def test_color_by_scan_empty():
    assert clomerge.color_by_scan([]).shape == (0, 3)


def test_color_by_height_spans_full_range():
    xyz = np.column_stack([np.zeros(10), np.zeros(10), np.linspace(0, 5, 10)])
    rgb = clomerge.color_by_height(xyz)
    assert rgb.shape == (10, 3)
    assert rgb.dtype == np.uint8
    assert tuple(rgb[0]) != tuple(rgb[-1])


def test_color_by_height_uses_combined_scale():
    """Titik z yang sama harus berwarna sama, walau berasal dari scan berbeda."""
    low = np.column_stack([np.zeros(3), np.zeros(3), np.array([0.0, 1.0, 2.0])])
    high = np.column_stack([np.zeros(3), np.zeros(3), np.array([2.0, 3.0, 4.0])])
    rgb = clomerge.color_by_height(np.vstack([low, high]))
    assert tuple(rgb[2]) == tuple(rgb[3])   # dua-duanya z = 2.0


def test_color_by_height_flat_cloud_does_not_crash():
    xyz = np.zeros((5, 3))
    assert clomerge.color_by_height(xyz).shape == (5, 3)


def test_color_by_height_empty():
    assert clomerge.color_by_height(np.zeros((0, 3))).shape == (0, 3)


# ═══════════════════════════════════════════════════════════════════════════════
# registrasi
# ═══════════════════════════════════════════════════════════════════════════════

def test_register_pair_recovers_known_transform():
    """Cloud yang digeser-putar harus dikembalikan ke posisi semula."""
    ref = room_cloud(seed=1)
    truth = rigid(rz_deg=25, t=(1.5, -0.8, 0.0))
    moved = clomerge.apply_transform(ref, truth)

    T, fitness, rmse = clomerge.register_pair(moved, ref, voxel=0.05)

    assert clomerge.quality_verdict(fitness, rmse) == clomerge.BAIK
    kembali = clomerge.apply_transform(moved, T)
    assert np.allclose(kembali.mean(axis=0), ref.mean(axis=0), atol=0.05)
    assert float(np.abs(T @ truth - np.eye(4)).max()) < 0.1


def test_register_pair_identical_clouds_gives_identity():
    ref = room_cloud(seed=2)
    T, fitness, rmse = clomerge.register_pair(ref.copy(), ref, voxel=0.05)
    assert fitness > 0.9
    assert np.allclose(T, np.eye(4), atol=0.05)


def test_wall_subset_drops_floor_and_ceiling():
    """Lantai & plafon harus tersingkir; dinding harus bertahan."""
    rng = np.random.default_rng(0)
    lantai = np.column_stack([rng.uniform(0, 8, 8000), rng.uniform(0, 6, 8000),
                              np.zeros(8000)])
    dinding = np.column_stack([np.zeros(4000), rng.uniform(0, 6, 4000),
                               rng.uniform(0, 3, 4000)])
    keluar = np.asarray(clomerge.wall_subset(
        clomerge.to_o3d(np.vstack([lantai, dinding]))).points)

    assert len(keluar) > 0
    # yang tersisa harus berada di bidang dinding (x ≈ 0), bukan di lantai
    assert np.abs(keluar[:, 0]).max() < 0.2
    assert keluar[:, 2].max() > 1.0


def test_wall_subset_falls_back_when_no_vertical_surface():
    """Cloud datar tanpa dinding tetap boleh dinilai, jangan jadi kosong."""
    rng = np.random.default_rng(1)
    datar = np.column_stack([rng.uniform(0, 8, 5000), rng.uniform(0, 6, 5000),
                             np.zeros(5000)])
    keluar = clomerge.wall_subset(clomerge.to_o3d(datar))
    assert len(keluar.points) > 0


def test_wall_subset_does_not_mutate_input():
    pcd = clomerge.to_o3d(room_cloud(seed=20))
    clomerge.wall_subset(pcd)
    assert not pcd.has_normals()


def test_register_pair_scores_on_walls_not_floor():
    """Skor harus peka terhadap yaw.

    Solusi yang melenceng 90° pada ruangan tak simetris harus jatuh skornya,
    bukan tertutupi oleh lantai & plafon yang cocok pada rotasi berapa pun.
    """
    ref = room_cloud(seed=21)
    T_salah = rigid(rz_deg=90, t=(0.0, 0.0, 0.0))

    pcd = clomerge.to_o3d(ref)
    ref_wall = clomerge.wall_subset(pcd)
    src_wall = clomerge.wall_subset(pcd)

    benar, _ = clomerge.evaluate(src_wall, ref_wall, np.eye(4))
    salah, _ = clomerge.evaluate(src_wall, ref_wall, T_salah)
    assert benar > salah + 0.3


def test_register_pair_honours_tries_argument(monkeypatch):
    """tries=N harus benar-benar menjalankan RANSAC N kali."""
    hitung = {"n": 0}
    asli = clomerge.global_register

    def dihitung(*a, **k):
        hitung["n"] += 1
        return asli(*a, **k)

    monkeypatch.setattr(clomerge, "global_register", dihitung)
    ref = room_cloud(seed=22)
    clomerge.register_pair(ref.copy(), ref, voxel=0.2, tries=3)
    assert hitung["n"] == 3


def test_register_pair_keeps_best_scoring_attempt(monkeypatch):
    """Dari beberapa percobaan, yang dikembalikan harus yang skornya tertinggi.

    Ini inti perbaikannya: RANSAC mendarat di jawaban berbeda tiap dijalankan,
    jadi satu percobaan saja bisa apes dapat solusi yang melenceng 90°.
    """
    ref = room_cloud(seed=23)
    jawaban = [rigid(t=(9, 9, 9)), rigid(rz_deg=30), rigid(t=(5, 5, 5))]
    skor = iter([0.2, 0.9, 0.4])

    hasil = iter(jawaban)
    monkeypatch.setattr(clomerge, "global_register",
                        lambda *a, **k: type("R", (), {"transformation": next(hasil)})())
    # ICP dibuat meneruskan tebakan apa adanya, supaya yang diuji murni pemilihan
    monkeypatch.setattr(clomerge, "refine",
                        lambda a, b, init, s: type("R", (), {"transformation": init})())
    monkeypatch.setattr(clomerge, "evaluate", lambda a, b, T, **k: (next(skor), 0.01))

    T, fitness, _ = clomerge.register_pair(ref.copy(), ref, voxel=0.2, tries=3)

    assert fitness == 0.9
    assert np.allclose(T, jawaban[1])


def test_evaluate_perfect_match_scores_one():
    pcd = clomerge.to_o3d(room_cloud(seed=10))
    fitness, rmse = clomerge.evaluate(pcd, pcd, np.eye(4))
    assert fitness == pytest.approx(1.0)
    assert rmse == pytest.approx(0.0, abs=1e-9)


def test_evaluate_far_apart_scores_zero():
    pcd = clomerge.to_o3d(room_cloud(seed=11))
    fitness, _ = clomerge.evaluate(pcd, pcd, rigid(t=(500.0, 0.0, 0.0)))
    assert fitness == 0.0


def test_register_pair_score_does_not_inflate_with_bigger_voxel():
    """Skor dinilai pada jarak tetap, jadi tidak boleh melar mengikuti voxel.

    Ini yang membuat angka BAIK/RAGU/GAGAL bisa dibandingkan antar-run walau
    --voxel-nya berbeda.
    """
    ref = room_cloud(seed=12)
    moved = clomerge.apply_transform(ref, rigid(rz_deg=20, t=(1.0, -0.5, 0.0)))

    _, fit_halus, _ = clomerge.register_pair(moved, ref, voxel=0.05)
    _, fit_kasar, _ = clomerge.register_pair(moved, ref, voxel=0.20)

    assert fit_kasar == pytest.approx(fit_halus, abs=0.15)


# ═══════════════════════════════════════════════════════════════════════════════
# transforms.txt
# ═══════════════════════════════════════════════════════════════════════════════

def test_write_transforms_records_every_scan(tmp_path):
    results = [
        {"name": "a.mcap", "is_ref": True, "T": np.eye(4),
         "fitness": 1.0, "rmse": 0.0, "verdict": clomerge.BAIK},
        {"name": "b.mcap", "is_ref": False, "T": rigid(rz_deg=10, t=(1, 2, 3)),
         "fitness": 0.62, "rmse": 0.013, "verdict": clomerge.BAIK},
    ]
    out = tmp_path / "transforms.txt"
    clomerge.write_transforms(out, ["a.mcap", "b.mcap"], results)

    text = out.read_text()
    assert "a.mcap" in text and "b.mcap" in text
    assert "acuan" in text
    assert "0.6200" in text
    assert "0.0130" in text


# ═══════════════════════════════════════════════════════════════════════════════
# parser
# ═══════════════════════════════════════════════════════════════════════════════

def test_parser_defaults():
    args = clomerge.build_parser().parse_args(["a.mcap", "b.mcap"])
    assert args.files == ["a.mcap", "b.mcap"]
    assert args.voxel == clomerge.DEFAULT_VOXEL
    assert args.drop_failed is False
    assert args.no_open is False
    assert args.topic is None
    assert args.force is False


def test_parser_requires_a_file():
    with pytest.raises(SystemExit):
        clomerge.build_parser().parse_args([])


def test_parser_accepts_voxel_and_flags():
    args = clomerge.build_parser().parse_args(
        ["a.ply", "b.ply", "--voxel", "0.1", "--drop-failed", "--no-open"])
    assert args.voxel == 0.1
    assert args.drop_failed is True
    assert args.no_open is True


# ═══════════════════════════════════════════════════════════════════════════════
# run
# ═══════════════════════════════════════════════════════════════════════════════

def write_ply_cloud(path, xyz):
    make_grid.write_ply(str(path), xyz.astype(np.float32),
                        np.zeros((len(xyz), 3), dtype=np.uint8))
    return str(path)


def test_run_rejects_single_file(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_ply_cloud(tmp_path / "a.ply", room_cloud(seed=3))
    with pytest.raises(SystemExit):
        clomerge.run(clomerge.build_parser().parse_args([a]))


def test_run_rejects_duplicate_of_same_file(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_ply_cloud(tmp_path / "a.ply", room_cloud(seed=4))
    with pytest.raises(SystemExit):
        clomerge.run(clomerge.build_parser().parse_args([a, a]))


def test_run_writes_all_three_outputs_and_opens_them(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    ref = room_cloud(seed=5)
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply",
                        clomerge.apply_transform(ref, rigid(rz_deg=15, t=(1.0, 0.5, 0))))

    opened = {}
    monkeypatch.setattr(clomerge, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    d = slot1(tmp_path)
    assert (d / "merged.ply").is_file()
    assert (d / "merged_check.ply").is_file()
    assert (d / "transforms.txt").is_file()
    assert opened["files"] == [str(d / "grid.ply"),
                               str(d / "merged_check.ply"),
                               str(d / "merged.ply")]


def test_run_no_open_skips_cloudcompare(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    ref = room_cloud(seed=6)
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply",
                        clomerge.apply_transform(ref, rigid(t=(0.5, 0.5, 0))))

    monkeypatch.setattr(clomerge, "launch_cloudcompare",
                        lambda files: pytest.fail("tidak boleh membuka CloudCompare"))

    clomerge.run(clomerge.build_parser().parse_args(
        [a, b, "--voxel", "0.08", "--no-open"]))


def test_run_merged_contains_points_from_both_scans(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    ref = room_cloud(seed=7)
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply",
                        clomerge.apply_transform(ref, rigid(rz_deg=20, t=(1.0, 0, 0))))

    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)
    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    merged = slot1(tmp_path) / "merged.ply"
    mn, mx = make_grid.ply_xyz_minmax(str(merged))
    assert len(clomerge.read_cloud_xyz(str(merged))) == 2 * len(ref)
    # setelah registrasi benar, gabungan tidak boleh jauh lebih luas dari acuan
    assert mx[0] - mn[0] < (ref[:, 0].max() - ref[:, 0].min()) + 1.0


def test_run_drop_failed_removes_bad_scan(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    ref = room_cloud(seed=8)
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply", ref.copy())

    # paksa scan kedua dinilai gagal
    monkeypatch.setattr(clomerge, "register_pair",
                        lambda src, dst, voxel, tries=None: (np.eye(4), 0.01, 9.9))
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    with pytest.raises(SystemExit):
        clomerge.run(clomerge.build_parser().parse_args(
            [a, b, "--drop-failed", "--no-open"]))


def test_run_keeps_bad_scan_without_drop_failed(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    ref = room_cloud(seed=9)
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply", ref.copy())

    monkeypatch.setattr(clomerge, "register_pair",
                        lambda src, dst, voxel, tries=None: (np.eye(4), 0.01, 9.9))
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--no-open"]))

    merged = slot1(tmp_path) / "merged.ply"
    assert len(clomerge.read_cloud_xyz(str(merged))) == 2 * len(ref)


# ═══════════════════════════════════════════════════════════════════════════════
# Perataan lantai (level_transform)
# ═══════════════════════════════════════════════════════════════════════════════

def tilt(rx_deg=0.0, ry_deg=0.0, tz=0.0) -> np.ndarray:
    """Matriks 4x4: miringkan terhadap sumbu X lalu Y, lalu geser Z."""
    ax, ay = np.radians(rx_deg), np.radians(ry_deg)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(ax), -np.sin(ax)],
                   [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)],
                   [0, 1, 0],
                   [-np.sin(ay), 0, np.cos(ay)]])
    T = np.eye(4)
    T[:3, :3] = Ry @ Rx
    T[2, 3] = tz
    return T


def room_with_ceiling(seed=0) -> np.ndarray:
    """room_cloud ditambah plafon di z=3, supaya bisa diuji floor-vs-plafon."""
    rng = np.random.default_rng(seed)
    ceil = np.column_stack([rng.uniform(0, 8, 3000), rng.uniform(0, 6, 3000),
                            np.full(3000, 3.0)])
    return np.vstack([room_cloud(seed), ceil])


def floor_normal_after(xyz, L):
    """Normal bidang lantai setelah L diterapkan, dinormalkan menghadap atas."""
    out = clomerge.apply_transform(xyz, L)
    plane, _ = clomerge.to_o3d(out).segment_plane(0.02, 3, 2000)
    n = np.array(plane[:3], dtype=float)
    n /= np.linalg.norm(n)
    return n if n[2] > 0 else -n


def test_level_transform_already_level_returns_near_identity_rotation():
    L = clomerge.level_transform(room_cloud(seed=30))
    assert L is not None
    assert np.allclose(L[:3, :3], np.eye(3), atol=1e-2)


def test_level_transform_corrects_tilt():
    miring = clomerge.apply_transform(room_cloud(seed=31), tilt(rx_deg=7, ry_deg=-5))
    L = clomerge.level_transform(miring)
    assert L is not None
    n = floor_normal_after(miring, L)
    assert np.allclose(n, [0.0, 0.0, 1.0], atol=2e-2)


def test_level_transform_puts_floor_at_z_zero():
    miring = clomerge.apply_transform(room_cloud(seed=32), tilt(rx_deg=6, tz=4.0))
    L = clomerge.level_transform(miring)
    assert L is not None
    out = clomerge.apply_transform(miring, L)
    # lantai adalah kumpulan titik terendah; mediannya harus jatuh di sekitar 0
    terendah = out[out[:, 2] < np.percentile(out[:, 2], 20)]
    assert abs(float(np.median(terendah[:, 2]))) < 0.05


def test_level_transform_does_not_introduce_yaw():
    """Rotasi harus minimal: arah hadap dinding terhadap sumbu Z tidak berubah."""
    datar = room_cloud(seed=33)
    L = clomerge.level_transform(datar)
    assert L is not None
    R = L[:3, :3]
    # sumbu Z dunia diputar ke dirinya sendiri → komponen yaw harus nol
    assert abs(np.arctan2(R[1, 0], R[0, 0])) < np.radians(0.5)


def test_level_transform_picks_floor_not_ceiling():
    miring = clomerge.apply_transform(room_with_ceiling(seed=34), tilt(rx_deg=8))
    L = clomerge.level_transform(miring)
    assert L is not None
    out = clomerge.apply_transform(miring, L)
    # lantai di 0, plafon tetap di +3 — bukan tertukar
    lantai = out[np.abs(out[:, 2]) < 0.1]
    plafon = out[out[:, 2] > 2.5]
    assert len(lantai) > 2000 and abs(float(np.median(lantai[:, 2]))) < 0.02
    assert len(plafon) > 2000 and abs(float(np.median(plafon[:, 2])) - 3.0) < 0.1


def test_level_transform_returns_none_without_flat_surface():
    rng = np.random.default_rng(35)
    blob = rng.uniform(-1, 1, (6000, 3))
    assert clomerge.level_transform(blob) is None


def test_level_angle_deg_reports_tilt():
    L = tilt(rx_deg=10)
    assert abs(clomerge.level_angle_deg(L) - 10.0) < 0.5


# ═══════════════════════════════════════════════════════════════════════════════
# run(): grid + perataan
# ═══════════════════════════════════════════════════════════════════════════════

def _two_tilted_scans(tmp_path, seed):
    ref = clomerge.apply_transform(room_cloud(seed=seed), tilt(rx_deg=6, ry_deg=4))
    a = write_ply_cloud(tmp_path / "a.ply", ref)
    b = write_ply_cloud(tmp_path / "b.ply",
                        clomerge.apply_transform(ref, rigid(rz_deg=12, t=(0.8, 0.4, 0))))
    return a, b


def test_run_opens_grid_before_clouds(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 40)

    opened = {}
    monkeypatch.setattr(clomerge, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    d = slot1(tmp_path)
    assert (d / "grid.ply").is_file()
    assert opened["files"] == [str(d / "grid.ply"),
                               str(d / "merged_check.ply"),
                               str(d / "merged.ply")]


def test_run_no_grid_opens_clouds_only(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 41)

    opened = {}
    monkeypatch.setattr(clomerge, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomerge.run(clomerge.build_parser().parse_args(
        [a, b, "--voxel", "0.08", "--no-grid"]))

    assert len(opened["files"]) == 2
    assert not (slot1(tmp_path) / "grid.ply").exists()


def test_run_levels_merged_output(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 42)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    merged = slot1(tmp_path) / "merged.ply"
    xyz = clomerge.read_cloud_xyz(str(merged))
    n = floor_normal_after(xyz, np.eye(4))
    assert np.allclose(n, [0.0, 0.0, 1.0], atol=3e-2)
    terendah = xyz[xyz[:, 2] < np.percentile(xyz[:, 2], 20)]
    assert abs(float(np.median(terendah[:, 2]))) < 0.05


def test_run_no_level_leaves_tilt(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 43)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args(
        [a, b, "--voxel", "0.08", "--no-level"]))

    merged = slot1(tmp_path) / "merged.ply"
    n = floor_normal_after(clomerge.read_cloud_xyz(str(merged)), np.eye(4))
    assert not np.allclose(n, [0.0, 0.0, 1.0], atol=3e-2)


def test_run_bakes_level_into_transforms_txt(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 44)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    teks = (slot1(tmp_path) / "transforms.txt").read_text()
    assert "Perataan lantai" in teks
    # acuan tidak lagi identitas: perataan sudah dipanggang ke dalamnya
    assert "tidak ditransformasi" not in teks


def test_run_grid_stays_axis_aligned(tmp_path, monkeypatch):
    """Grid tidak boleh ikut diputar — Z-nya harus rata di 0."""
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 45)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    grid = slot1(tmp_path) / "grid.ply"
    gmn, gmx = make_grid.ply_xyz_minmax(str(grid))
    assert abs(float(gmn[2])) < 1e-6 and abs(float(gmx[2])) < 1e-6


# ═══════════════════════════════════════════════════════════════════════════════
# Slot bernomor: hasil merge tidak saling menimpa
# ═══════════════════════════════════════════════════════════════════════════════

def test_next_merge_slot_starts_at_001(tmp_path):
    assert clomerge.next_merge_slot(tmp_path).name == "001"


def test_next_merge_slot_increments(tmp_path):
    (tmp_path / "001").mkdir()
    assert clomerge.next_merge_slot(tmp_path).name == "002"


def test_next_merge_slot_takes_max_not_gap(tmp_path):
    """Nomor selalu naik; lubang di tengah tidak diisi ulang supaya urutan
    kronologisnya tetap terbaca dari nomornya."""
    for n in ("001", "002", "005"):
        (tmp_path / n).mkdir()
    assert clomerge.next_merge_slot(tmp_path).name == "006"


def test_next_merge_slot_ignores_non_numeric(tmp_path):
    (tmp_path / "001").mkdir()
    (tmp_path / "catatan").mkdir()
    (tmp_path / "003.txt").write_text("x")
    assert clomerge.next_merge_slot(tmp_path).name == "002"


def test_next_merge_slot_when_parent_missing(tmp_path):
    assert clomerge.next_merge_slot(tmp_path / "belum_ada").name == "001"


def test_run_writes_into_numbered_slot(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 50)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    slot = tmp_path / "out" / clomerge.MERGE_DIRNAME / "001"
    assert (slot / "merged.ply").is_file()
    assert (slot / "merged_check.ply").is_file()
    assert (slot / "grid.ply").is_file()
    assert (slot / "transforms.txt").is_file()


def test_run_twice_keeps_both_results(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 51)
    monkeypatch.setattr(clomerge, "launch_cloudcompare", lambda files: None)

    argv = [a, b, "--voxel", "0.08"]
    clomerge.run(clomerge.build_parser().parse_args(argv))
    clomerge.run(clomerge.build_parser().parse_args(argv))

    d = tmp_path / "out" / clomerge.MERGE_DIRNAME
    assert (d / "001" / "merged.ply").is_file()
    assert (d / "002" / "merged.ply").is_file()


def test_run_opens_files_from_new_slot(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a, b = _two_tilted_scans(tmp_path, 52)

    panggilan = []
    monkeypatch.setattr(clomerge, "launch_cloudcompare", panggilan.append)

    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))
    clomerge.run(clomerge.build_parser().parse_args([a, b, "--voxel", "0.08"]))

    assert len(panggilan) == 2
    assert all("/001/" in f for f in panggilan[0])
    assert all("/002/" in f for f in panggilan[1])
