"""Tests for cleanpc.py — the point cloud noise filter.

Run with:  .venv/bin/python -m pytest test_cleanpc.py -v
"""

import os

import numpy as np
import pytest

import cleanpc


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _plane_grid(n_side, spacing, z=0.0):
    """A flat square grid of points in the XY plane."""
    a = np.arange(n_side) * spacing
    xx, yy = np.meshgrid(a, a)
    return np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)])


# ═══════════════════════════════════════════════════════════════════════════════
# Task 1 — I/O
# ═══════════════════════════════════════════════════════════════════════════════

def test_save_load_xyz_roundtrip(tmp_path):
    pts = np.array([[0.0, 1.0, 2.0], [3.5, -4.25, 5.125]])
    p = tmp_path / "a.xyz"
    cleanpc.save_xyz(str(p), pts)
    loaded, colors = cleanpc.load_cloud(str(p))
    assert colors is None
    np.testing.assert_allclose(loaded, pts, atol=1e-6)


def test_save_load_ply_roundtrip_no_color(tmp_path):
    pts = np.array([[0.0, 1.0, 2.0], [3.5, -4.25, 5.125]])
    p = tmp_path / "a.ply"
    cleanpc.save_ply(str(p), pts, None)
    loaded, colors = cleanpc.load_cloud(str(p))
    assert colors is None
    np.testing.assert_allclose(loaded, pts, atol=1e-5)


def test_save_load_ply_roundtrip_with_color(tmp_path):
    pts = np.array([[0.0, 1.0, 2.0], [3.5, -4.25, 5.125]])
    cols = np.array([[10, 20, 30], [200, 210, 220]], dtype=np.uint8)
    p = tmp_path / "a.ply"
    cleanpc.save_ply(str(p), pts, cols)
    loaded, colors = cleanpc.load_cloud(str(p))
    np.testing.assert_allclose(loaded, pts, atol=1e-5)
    np.testing.assert_array_equal(colors, cols)


def test_load_ascii_ply(tmp_path):
    p = tmp_path / "ascii.ply"
    p.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "end_header\n1 2 3\n4 5 6\n"
    )
    pts, colors = cleanpc.load_cloud(str(p))
    assert colors is None
    np.testing.assert_allclose(pts, [[1, 2, 3], [4, 5, 6]])


def test_load_ply_ignores_extra_properties(tmp_path):
    """A PLY carrying intensity/normals must still load, with extras dropped."""
    p = tmp_path / "extra.ply"
    p.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float intensity\n"
        "end_header\n1 2 3 0.9\n4 5 6 0.1\n"
    )
    pts, colors = cleanpc.load_cloud(str(p))
    assert colors is None
    np.testing.assert_allclose(pts, [[1, 2, 3], [4, 5, 6]])


def test_load_rejects_unknown_extension(tmp_path):
    p = tmp_path / "a.pcd"
    p.write_text("nonsense")
    with pytest.raises(ValueError, match="format"):
        cleanpc.load_cloud(str(p))


# ═══════════════════════════════════════════════════════════════════════════════
# Task 2 — geometric filters
# ═══════════════════════════════════════════════════════════════════════════════

def test_crop_range_keeps_shell():
    pts = np.array([[0.1, 0, 0], [1.0, 0, 0], [20.0, 0, 0]])
    mask = cleanpc.crop_range(pts, 0.5, 15.0)
    np.testing.assert_array_equal(mask, [False, True, False])


def test_crop_range_boundaries_inclusive():
    pts = np.array([[0.5, 0, 0], [15.0, 0, 0]])
    mask = cleanpc.crop_range(pts, 0.5, 15.0)
    np.testing.assert_array_equal(mask, [True, True])


def test_dedup_keeps_first_occurrence():
    pts = np.array([[1.0, 1, 1], [2.0, 2, 2], [1.0, 1, 1]])
    mask = cleanpc.dedup(pts)
    np.testing.assert_array_equal(mask, [True, True, False])


def test_voxel_downsample_one_point_per_cell():
    pts = np.array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [5.5, 5.5, 5.5]])
    mask = cleanpc.voxel_downsample(pts, 1.0)
    assert mask.sum() == 2


def test_voxel_downsample_disabled_keeps_everything():
    pts = np.random.default_rng(0).normal(size=(50, 3))
    mask = cleanpc.voxel_downsample(pts, 0.0)
    assert mask.all()


# ═══════════════════════════════════════════════════════════════════════════════
# Task 3 — range-adaptive statistical outlier removal
# ═══════════════════════════════════════════════════════════════════════════════

def test_sor_removes_isolated_strays():
    dense = _plane_grid(30, 0.05)
    strays = np.array([[10.0, 10.0, 10.0], [-8.0, 3.0, 6.0], [7.0, -7.0, 4.0]])
    pts = np.vstack([dense, strays])
    mask = cleanpc.sor_adaptive(pts, k=8, k_mad=3.0)
    assert not mask[-3:].any()
    assert mask[:len(dense)].mean() > 0.95


def test_sor_keeps_legitimate_far_points_despite_lower_density():
    """Density falls off with range, as in real LiDAR. Far points are sparser but
    real, and must NOT be deleted. A global-threshold SOR fails this test."""
    chunks = []
    for radius, spacing in [(1.0, 0.02), (3.0, 0.06), (6.0, 0.12), (10.0, 0.20)]:
        n = 40
        u = (np.arange(n) - n / 2) * spacing
        vv, ww = np.meshgrid(u, u)
        patch = np.column_stack([np.full(vv.size, radius), vv.ravel(), ww.ravel()])
        chunks.append(patch)
    pts = np.vstack(chunks)
    mask = cleanpc.sor_adaptive(pts, k=8, k_mad=3.0)
    start = 0
    for i, c in enumerate(chunks):
        band = mask[start:start + len(c)]
        assert band.mean() > 0.90, f"band {i} retained only {band.mean():.2%}"
        start += len(c)


def test_sor_handles_tiny_input():
    pts = np.array([[0.0, 0, 0], [1.0, 0, 0]])
    mask = cleanpc.sor_adaptive(pts, k=8)
    assert mask.all()


# ═══════════════════════════════════════════════════════════════════════════════
# Task 4 — range-scaled radius outlier removal
# ═══════════════════════════════════════════════════════════════════════════════

def test_radius_outlier_removes_lone_points():
    cluster = np.random.default_rng(1).normal(scale=0.02, size=(200, 3))
    lone = np.array([[5.0, 5.0, 5.0]])
    pts = np.vstack([cluster, lone])
    mask = cleanpc.radius_outlier(pts, base_radius=0.15, min_neighbors=6)
    assert not mask[-1]
    assert mask[:-1].mean() > 0.95


def test_radius_outlier_scales_with_range():
    """A sparse-but-real patch far from the origin must survive, because the search
    radius grows with range."""
    near = _plane_grid(20, 0.03)
    far = _plane_grid(20, 0.15) + np.array([8.0, 0, 0])
    pts = np.vstack([near, far])
    mask = cleanpc.radius_outlier(pts, base_radius=0.15, min_neighbors=6, r_ref=2.0)
    assert mask[len(near):].mean() > 0.90


# ═══════════════════════════════════════════════════════════════════════════════
# Task 5 — pipeline
# ═══════════════════════════════════════════════════════════════════════════════

STAGE_NAMES = ["Range crop", "Dedup", "SOR adaptif", "Radius outlier", "Voxel downsample"]


def test_pipeline_reports_stats_per_stage():
    pts = np.vstack([_plane_grid(30, 0.05) + np.array([2.0, 0, 0]),
                     np.array([[0.1, 0, 0], [40.0, 0, 0]])])
    cfg = dict(min_range=0.5, max_range=15.0, k_mad=3.0, voxel=0.0)
    out, colors, stats = cleanpc.run_pipeline(pts, None, cfg)
    assert [s["name"] for s in stats] == STAGE_NAMES
    assert stats[0]["n_removed"] == 2
    assert colors is None
    assert len(out) < len(pts)


def test_pipeline_keeps_colors_aligned():
    """Colours must be dropped in lockstep with the points they belong to.

    The doomed point gets a colour value no other point has, so its survival is
    unambiguous. (Using a running index as the colour does not work: uint8 wraps
    at 256 and the marker collides with a legitimately retained point.)
    """
    pts = np.vstack([_plane_grid(20, 0.05) + np.array([2.0, 0, 0]),
                     np.array([[0.1, 0, 0]])])          # last point is too near
    colors = np.zeros((len(pts), 3), dtype=np.uint8)
    colors[-1] = [199, 199, 199]                        # unique marker
    cfg = dict(min_range=0.5, max_range=15.0, k_mad=3.0, voxel=0.0)
    out, out_colors, _ = cleanpc.run_pipeline(pts, colors, cfg)
    assert len(out) == len(out_colors)
    assert 199 not in out_colors[:, 0]


def test_pipeline_safety_brake_aborts_greedy_stage(monkeypatch):
    """A stage that would delete more than half its input is rolled back.

    The brake is exercised directly with a stub filter rather than by coaxing SOR
    into over-deleting, so this test covers the brake logic and nothing else.
    """
    pts = _plane_grid(20, 0.05) + np.array([2.0, 0, 0])

    def greedy(points, **kwargs):
        mask = np.zeros(len(points), dtype=bool)
        mask[: len(points) // 10] = True        # would delete 90%
        return mask

    monkeypatch.setattr(cleanpc, "sor_adaptive", greedy)
    cfg = dict(min_range=0.5, max_range=15.0, k_mad=3.0, voxel=0.0)
    out, _, stats = cleanpc.run_pipeline(pts, None, cfg)
    sor = next(s for s in stats if s["name"] == "SOR adaptif")
    assert sor["aborted"] is True
    assert sor["n_removed"] == 0
    # And the points really did survive the aborted stage.
    assert len(out) > len(pts) * 0.5


def test_pipeline_brake_does_not_protect_range_crop():
    """Range crop is exempt: discarding a huge near-sensor blob is its whole job."""
    blob = np.random.default_rng(5).normal(scale=0.05, size=(800, 3))   # all within 0.5 m
    room = _plane_grid(20, 0.05) + np.array([3.0, 0, 0])
    pts = np.vstack([blob, room])
    cfg = dict(min_range=0.5, max_range=15.0, k_mad=3.0, voxel=0.0)
    _, _, stats = cleanpc.run_pipeline(pts, None, cfg)
    crop = stats[0]
    assert crop["aborted"] is False
    assert crop["n_removed"] >= 800


# ═══════════════════════════════════════════════════════════════════════════════
# Task 6 — prompts, histogram, plotting
# ═══════════════════════════════════════════════════════════════════════════════

def test_ask_float_accepts_default_on_empty(monkeypatch):
    monkeypatch.setattr("builtins.input", lambda _: "")
    assert cleanpc.ask_float("x", 0.5) == 0.5


def test_ask_float_reasks_on_garbage(monkeypatch):
    answers = iter(["abc", "-3", "2.5"])
    monkeypatch.setattr("builtins.input", lambda _: next(answers))
    assert cleanpc.ask_float("x", 0.5) == 2.5


def test_ask_float_returns_default_on_eof(monkeypatch):
    def boom(_):
        raise EOFError

    monkeypatch.setattr("builtins.input", boom)
    assert cleanpc.ask_float("x", 1.25) == 1.25


def test_range_histogram_counts_all_points():
    pts = np.random.default_rng(3).normal(size=(500, 3)) * 3
    bins = cleanpc.range_histogram(pts, n_bins=5)
    assert sum(b[2] for b in bins) == len(pts)


def test_plot_comparison_writes_file(tmp_path):
    before = np.random.default_rng(4).normal(size=(200, 3))
    after = before[:100]
    out = tmp_path / "cmp.png"
    cleanpc.plot_comparison(before, after, str(out))
    assert out.exists() and out.stat().st_size > 0


# ═══════════════════════════════════════════════════════════════════════════════
# Task 7 — integration on the real scan
# ═══════════════════════════════════════════════════════════════════════════════

REAL = os.path.join(os.path.dirname(__file__), "test_lab_01_0_pointcloud.xyz")


@pytest.mark.skipif(not os.path.exists(REAL), reason="real dataset not present")
def test_end_to_end_on_real_scan(tmp_path):
    pts, colors = cleanpc.load_cloud(REAL)
    assert len(pts) > 100000
    cfg = dict(min_range=0.5, max_range=15.0, k_mad=3.0, voxel=0.0)
    out, out_colors, stats = cleanpc.run_pipeline(pts, colors, cfg)
    # The near-sensor blob is ~42% of this file and must go.
    assert stats[0]["n_removed"] > 70000
    # But a usable room must remain.
    assert len(out) > 50000
    dest = str(tmp_path / "lab.ply")
    cleanpc.save_ply(dest, out, out_colors)
    reloaded, _ = cleanpc.load_cloud(dest)
    assert len(reloaded) == len(out)
