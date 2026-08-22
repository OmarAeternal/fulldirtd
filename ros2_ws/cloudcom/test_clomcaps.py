"""Tests untuk clomcaps.py — banyak MCAP/PLY/XYZ → satu CloudCompare.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_clomcaps.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import os
from pathlib import Path

import numpy as np
import pytest

import clomcap
import clomcaps
import make_grid


def write_xyz(path, pts):
    """Tulis file .xyz sederhana berisi `pts`."""
    Path(path).write_text("\n".join(f"{x} {y} {z}" for x, y, z in pts) + "\n")
    return str(path)


# ═══════════════════════════════════════════════════════════════════════════════
# parser
# ═══════════════════════════════════════════════════════════════════════════════

def test_parser_accepts_many_files():
    args = clomcaps.build_parser().parse_args(["a.mcap", "b.mcap", "c.ply"])
    assert args.files == ["a.mcap", "b.mcap", "c.ply"]


def test_parser_accepts_single_file():
    args = clomcaps.build_parser().parse_args(["a.mcap"])
    assert args.files == ["a.mcap"]


def test_parser_requires_at_least_one_file():
    with pytest.raises(SystemExit):
        clomcaps.build_parser().parse_args([])


def test_parser_defaults_match_clomcap():
    args = clomcaps.build_parser().parse_args(["a.mcap"])
    assert args.topic is None
    assert args.force is False
    assert args.png is False
    assert args.no_grid is False
    assert args.spacing == 1.0
    assert args.margin == 1.0


# ═══════════════════════════════════════════════════════════════════════════════
# dedupe_inputs
# ═══════════════════════════════════════════════════════════════════════════════

def test_dedupe_inputs_keeps_order():
    assert clomcaps.dedupe_inputs(["a.ply", "b.ply", "c.ply"]) == ["a.ply", "b.ply", "c.ply"]


def test_dedupe_inputs_drops_repeats(tmp_path):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0)])
    assert clomcaps.dedupe_inputs([a, a]) == [a]


def test_dedupe_inputs_drops_same_file_via_different_paths(tmp_path):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0)])
    dotted = str(tmp_path / "." / "a.xyz")
    assert clomcaps.dedupe_inputs([a, dotted]) == [a]


# ═══════════════════════════════════════════════════════════════════════════════
# combined_bounds
# ═══════════════════════════════════════════════════════════════════════════════

def test_combined_bounds_single_cloud(tmp_path):
    c = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 3, 4)])
    mn, mx = clomcaps.combined_bounds([c])
    assert list(mn) == [0, 0, 0]
    assert list(mx) == [2, 3, 4]


def test_combined_bounds_unions_all_clouds(tmp_path):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 2)])
    b = write_xyz(tmp_path / "b.xyz", [(-5, 1, 1), (1, 9, 1)])
    mn, mx = clomcaps.combined_bounds([a, b])
    assert list(mn) == [-5, 0, 0]
    assert list(mx) == [2, 9, 2]


def test_combined_bounds_skips_unreadable_cloud(tmp_path):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 2)])
    bad = tmp_path / "bad.xyz"
    bad.write_text("bukan angka\n")
    mn, mx = clomcaps.combined_bounds([str(bad), a])
    assert list(mn) == [0, 0, 0]
    assert list(mx) == [2, 2, 2]


def test_combined_bounds_all_unreadable_raises(tmp_path):
    bad = tmp_path / "bad.xyz"
    bad.write_text("bukan angka\n")
    with pytest.raises(SystemExit):
        clomcaps.combined_bounds([str(bad)])


# ═══════════════════════════════════════════════════════════════════════════════
# make_grid_file
# ═══════════════════════════════════════════════════════════════════════════════

def test_make_grid_file_covers_every_cloud_with_margin(tmp_path):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])
    b = write_xyz(tmp_path / "b.xyz", [(-4, 0, 0), (0, 6, 0)])
    grid = tmp_path / "grid.ply"

    assert clomcaps.make_grid_file([a, b], grid, spacing=1.0, margin=1.0) == grid

    gmn, gmx = make_grid.ply_xyz_minmax(str(grid))
    assert gmn[0] <= -5.0 + 1e-6 and gmx[0] >= 3.0 - 1e-6
    assert gmn[1] <= -1.0 + 1e-6 and gmx[1] >= 7.0 - 1e-6


def test_make_grid_file_failure_returns_none(tmp_path, monkeypatch):
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])
    monkeypatch.setattr(clomcaps, "combined_bounds",
                        lambda clouds: (_ for _ in ()).throw(RuntimeError("boom")))
    assert clomcaps.make_grid_file([a], tmp_path / "grid.ply", 1.0, 1.0) is None


# ═══════════════════════════════════════════════════════════════════════════════
# grid_path_for
# ═══════════════════════════════════════════════════════════════════════════════

def test_grid_path_for_single_input_reuses_clomcap_layout(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    src = str(tmp_path / "scan_0003_1sweep_0.mcap")
    assert clomcaps.grid_path_for([src]) == \
        tmp_path / "out" / "scan_0003_1sweep_0" / "grid.ply"


def test_grid_path_for_many_inputs_uses_multi_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    got = clomcaps.grid_path_for([str(tmp_path / "a.mcap"), str(tmp_path / "b.mcap")])
    assert got == tmp_path / "out" / clomcaps.MULTI_DIRNAME / "grid.ply"


# ═══════════════════════════════════════════════════════════════════════════════
# prepare_cloud
# ═══════════════════════════════════════════════════════════════════════════════

def test_prepare_cloud_returns_cloud_input_as_is(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    monkeypatch.setattr(clomcap, "convert_to_ply",
                        lambda *a, **k: pytest.fail("tidak boleh konversi"))
    c = write_xyz(tmp_path / "c.xyz", [(0, 0, 0)])
    args = clomcaps.build_parser().parse_args([c])
    assert clomcaps.prepare_cloud(c, args) == os.path.abspath(c)


def test_prepare_cloud_missing_file_raises(tmp_path):
    src = str(tmp_path / "tidak_ada.mcap")
    args = clomcaps.build_parser().parse_args([src])
    with pytest.raises(SystemExit):
        clomcaps.prepare_cloud(src, args)


def test_prepare_cloud_uses_cache_when_fresh(tmp_path, monkeypatch):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")

    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)
    xyz = np.array([[0.0, 0.0, 0.0], [2.0, 2.0, 2.0]], dtype=np.float32)
    make_grid.write_ply(str(paths["ply"]), xyz, np.zeros((2, 3), dtype=np.uint8))
    os.utime(src, (1000, 1000))
    os.utime(paths["ply"], (2000, 2000))

    monkeypatch.setattr(clomcap, "convert_to_ply",
                        lambda *a, **k: pytest.fail("cache segar, tidak boleh konversi"))
    args = clomcaps.build_parser().parse_args([str(src)])
    assert clomcaps.prepare_cloud(str(src), args) == str(paths["ply"])


def test_prepare_cloud_force_reconverts(tmp_path, monkeypatch):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")

    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)
    xyz = np.array([[0.0, 0.0, 0.0], [2.0, 2.0, 2.0]], dtype=np.float32)
    make_grid.write_ply(str(paths["ply"]), xyz, np.zeros((2, 3), dtype=np.uint8))
    os.utime(src, (1000, 1000))
    os.utime(paths["ply"], (2000, 2000))

    called = {}
    monkeypatch.setattr(clomcap, "convert_to_ply",
                        lambda *a, **k: called.setdefault("yes", True))
    args = clomcaps.build_parser().parse_args([str(src), "--force"])
    clomcaps.prepare_cloud(str(src), args)
    assert called.get("yes") is True


# ═══════════════════════════════════════════════════════════════════════════════
# run
# ═══════════════════════════════════════════════════════════════════════════════

def test_run_opens_grid_then_every_cloud_in_order(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])
    b = write_xyz(tmp_path / "b.xyz", [(-4, 0, 0), (0, 6, 0)])

    opened = {}
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcaps.run(clomcaps.build_parser().parse_args([a, b]))

    assert len(opened["files"]) == 3
    assert opened["files"][0].endswith("grid.ply")
    assert clomcaps.MULTI_DIRNAME in opened["files"][0]
    assert opened["files"][1:] == [os.path.abspath(a), os.path.abspath(b)]


def test_run_no_grid_opens_clouds_only(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])
    b = write_xyz(tmp_path / "b.xyz", [(-4, 0, 0), (0, 6, 0)])

    opened = {}
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcaps.run(clomcaps.build_parser().parse_args([a, b, "--no-grid"]))
    assert opened["files"] == [os.path.abspath(a), os.path.abspath(b)]


def test_run_skips_failing_file_and_opens_the_rest(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    good = write_xyz(tmp_path / "good.xyz", [(0, 0, 0), (2, 2, 0)])
    bad = str(tmp_path / "tidak_ada.mcap")

    opened = {}
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcaps.run(clomcaps.build_parser().parse_args([bad, good, "--no-grid"]))
    assert opened["files"] == [os.path.abspath(good)]


def test_run_all_files_failing_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: pytest.fail("tidak boleh membuka CloudCompare"))

    args = clomcaps.build_parser().parse_args([
        str(tmp_path / "x.mcap"), str(tmp_path / "y.mcap"), "--no-grid"])
    with pytest.raises(SystemExit):
        clomcaps.run(args)


def test_run_deduplicates_repeated_inputs(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])

    opened = {}
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcaps.run(clomcaps.build_parser().parse_args([a, a, "--no-grid"]))
    assert opened["files"] == [os.path.abspath(a)]


def test_run_single_file_still_works(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    a = write_xyz(tmp_path / "a.xyz", [(0, 0, 0), (2, 2, 0)])

    opened = {}
    monkeypatch.setattr(clomcaps, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcaps.run(clomcaps.build_parser().parse_args([a]))
    assert len(opened["files"]) == 2
    assert opened["files"][0].endswith("grid.ply")
    assert opened["files"][1] == os.path.abspath(a)
