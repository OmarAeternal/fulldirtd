"""Tests untuk clomcap.py — driver MCAP → point cloud + grid → CloudCompare.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_clomcap.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import os
from pathlib import Path

import pytest

import clomcap


# ═══════════════════════════════════════════════════════════════════════════════
# classify_input
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("name", [
    "a.mcap", "a.mcap.zstd", "A.MCAP", "scan_0003_3sweep_0.mcap",
])
def test_classify_input_mcap(name):
    assert clomcap.classify_input(name) == "mcap"


@pytest.mark.parametrize("name", [
    "a.ply", "a.xyz", "A.PLY", "016_0_pointcloud.xyz",
])
def test_classify_input_cloud(name):
    assert clomcap.classify_input(name) == "cloud"


@pytest.mark.parametrize("name", ["a.bag", "a.db3", "a.txt", "noextension"])
def test_classify_input_rejects_unknown(name):
    with pytest.raises(SystemExit):
        clomcap.classify_input(name)


# ═══════════════════════════════════════════════════════════════════════════════
# stem_of
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("name,expected", [
    ("016_0.mcap", "016_0"),
    ("/x/y/scan_0003_3sweep_0.mcap.zstd", "scan_0003_3sweep_0"),
    ("016_0_pointcloud.xyz", "016_0_pointcloud"),
    ("/x/y/016_0_pointcloud.ply", "016_0_pointcloud"),
])
def test_stem_of(name, expected):
    assert clomcap.stem_of(name) == expected


# ═══════════════════════════════════════════════════════════════════════════════
# out_paths
# ═══════════════════════════════════════════════════════════════════════════════

def test_out_paths_layout(tmp_path):
    p = clomcap.out_paths("/anywhere/016_0.mcap", root=tmp_path)
    assert p["dir"] == tmp_path / "016_0"
    assert p["ply"] == tmp_path / "016_0" / "016_0.ply"
    assert p["png"] == tmp_path / "016_0" / "016_0_viz.png"
    assert p["grid"] == tmp_path / "016_0" / "grid.ply"


def test_out_paths_defaults_to_out_root():
    p = clomcap.out_paths("/anywhere/016_0.mcap")
    assert p["dir"].parent == clomcap.OUT_ROOT


# ═══════════════════════════════════════════════════════════════════════════════
# is_cache_fresh
# ═══════════════════════════════════════════════════════════════════════════════

def test_cache_not_fresh_when_ply_missing(tmp_path):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    assert clomcap.is_cache_fresh(str(src), tmp_path / "a.ply") is False


def test_cache_not_fresh_when_ply_older(tmp_path):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    ply = tmp_path / "a.ply"
    ply.write_bytes(b"y")
    os.utime(ply, (1000, 1000))
    os.utime(src, (2000, 2000))
    assert clomcap.is_cache_fresh(str(src), ply) is False


def test_cache_fresh_when_ply_newer(tmp_path):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    ply = tmp_path / "a.ply"
    ply.write_bytes(b"y")
    os.utime(src, (1000, 1000))
    os.utime(ply, (2000, 2000))
    assert clomcap.is_cache_fresh(str(src), ply) is True


def test_cache_fresh_when_equal_mtime(tmp_path):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    ply = tmp_path / "a.ply"
    ply.write_bytes(b"y")
    os.utime(src, (1500, 1500))
    os.utime(ply, (1500, 1500))
    assert clomcap.is_cache_fresh(str(src), ply) is True


def test_cache_not_fresh_when_ply_empty(tmp_path):
    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    ply = tmp_path / "a.ply"
    ply.write_bytes(b"")
    os.utime(src, (1000, 1000))
    os.utime(ply, (2000, 2000))
    assert clomcap.is_cache_fresh(str(src), ply) is False


# ═══════════════════════════════════════════════════════════════════════════════
# xyz_minmax / bounds_of
# ═══════════════════════════════════════════════════════════════════════════════

import numpy as np

import make_grid


def test_xyz_minmax_reads_bounds(tmp_path):
    f = tmp_path / "c.xyz"
    f.write_text("0.0 1.0 2.0\n-3.0 4.0 -5.0\n1.5 -2.5 0.5\n")
    mn, mx = clomcap.xyz_minmax(str(f))
    assert np.allclose(mn, [-3.0, -2.5, -5.0])
    assert np.allclose(mx, [1.5, 4.0, 2.0])


def test_xyz_minmax_skips_blank_and_comment_lines(tmp_path):
    f = tmp_path / "c.xyz"
    f.write_text("# header\n\n0.0 0.0 0.0\n\n2.0 2.0 2.0\n   \n")
    mn, mx = clomcap.xyz_minmax(str(f))
    assert np.allclose(mn, [0.0, 0.0, 0.0])
    assert np.allclose(mx, [2.0, 2.0, 2.0])


def test_xyz_minmax_ignores_extra_columns(tmp_path):
    f = tmp_path / "c.xyz"
    f.write_text("0.0 0.0 0.0 255 0 0\n2.0 2.0 2.0 0 255 0\n")
    mn, mx = clomcap.xyz_minmax(str(f))
    assert np.allclose(mn, [0.0, 0.0, 0.0])
    assert np.allclose(mx, [2.0, 2.0, 2.0])


def test_xyz_minmax_empty_file_raises(tmp_path):
    f = tmp_path / "c.xyz"
    f.write_text("")
    with pytest.raises(SystemExit):
        clomcap.xyz_minmax(str(f))


def test_bounds_of_dispatches_to_ply(tmp_path):
    xyz = np.array([[0.0, 0.0, 0.0], [3.0, 4.0, 5.0]], dtype=np.float32)
    rgb = np.zeros((2, 3), dtype=np.uint8)
    f = tmp_path / "c.ply"
    make_grid.write_ply(str(f), xyz, rgb)
    mn, mx = clomcap.bounds_of(str(f))
    assert np.allclose(mn, [0.0, 0.0, 0.0])
    assert np.allclose(mx, [3.0, 4.0, 5.0])


def test_bounds_of_dispatches_to_xyz(tmp_path):
    f = tmp_path / "c.xyz"
    f.write_text("1.0 1.0 1.0\n4.0 5.0 6.0\n")
    mn, mx = clomcap.bounds_of(str(f))
    assert np.allclose(mx, [4.0, 5.0, 6.0])


def test_bounds_of_rejects_other_extension(tmp_path):
    f = tmp_path / "c.mcap"
    f.write_bytes(b"x")
    with pytest.raises(SystemExit):
        clomcap.bounds_of(str(f))


# ═══════════════════════════════════════════════════════════════════════════════
# select_topic
# ═══════════════════════════════════════════════════════════════════════════════

PC2 = "sensor_msgs/msg/PointCloud2"
LASER = "sensor_msgs/msg/LaserScan"


def _topics(*pairs):
    return {t: {"schema": s, "count": 1} for t, s in pairs}


def test_select_topic_prefers_map_3d():
    topics = _topics(("/other_cloud", PC2), ("/map_3d", PC2))
    assert clomcap.select_topic(topics) == ("/map_3d", PC2)


def test_select_topic_falls_back_to_any_pointcloud2():
    topics = _topics(("/scan", LASER), ("/other_cloud", PC2))
    assert clomcap.select_topic(topics) == ("/other_cloud", PC2)


def test_select_topic_falls_back_to_laserscan():
    topics = _topics(("/scan", LASER))
    assert clomcap.select_topic(topics) == ("/scan", LASER)


def test_select_topic_honours_override():
    topics = _topics(("/map_3d", PC2), ("/other_cloud", PC2))
    assert clomcap.select_topic(topics, "/other_cloud") == ("/other_cloud", PC2)


def test_select_topic_override_missing_raises():
    topics = _topics(("/map_3d", PC2))
    with pytest.raises(SystemExit):
        clomcap.select_topic(topics, "/tidak_ada")


def test_select_topic_no_lidar_raises_instead_of_prompting():
    """Regresi: mcaptopc.pick_lidar_topic akan menunggu input() di sini."""
    topics = _topics(("/tf", "tf2_msgs/msg/TFMessage"))
    with pytest.raises(SystemExit):
        clomcap.select_topic(topics)


def test_select_topic_empty_raises():
    with pytest.raises(SystemExit):
        clomcap.select_topic({})


# ═══════════════════════════════════════════════════════════════════════════════
# convert_to_ply
# ═══════════════════════════════════════════════════════════════════════════════

def test_convert_to_ply_writes_ply(tmp_path, monkeypatch):
    """Semua I/O MCAP di-stub; yang diuji adalah alur convert_to_ply."""
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]], dtype=np.float64)
    monkeypatch.setattr(clomcap.mcli, "looks_zstd", lambda p: False)
    monkeypatch.setattr(clomcap.mp, "inspect_mcap",
                        lambda p: {"/map_3d": {"schema": PC2, "count": 1}})
    monkeypatch.setattr(clomcap.mp, "extract_pointcloud2_frames",
                        lambda p, t: [pts])

    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)

    clomcap.convert_to_ply(str(src), paths, topic=None, want_png=False)

    assert paths["ply"].is_file()
    mn, mx = clomcap.bounds_of(str(paths["ply"]))
    assert np.allclose(mx, [1.0, 2.0, 3.0])
    assert not paths["png"].exists()


def test_convert_to_ply_no_frames_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(clomcap.mcli, "looks_zstd", lambda p: False)
    monkeypatch.setattr(clomcap.mp, "inspect_mcap",
                        lambda p: {"/map_3d": {"schema": PC2, "count": 1}})
    monkeypatch.setattr(clomcap.mp, "extract_pointcloud2_frames", lambda p, t: [])

    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)

    with pytest.raises(SystemExit):
        clomcap.convert_to_ply(str(src), paths, topic=None, want_png=False)


def test_convert_to_ply_png_failure_is_not_fatal(tmp_path, monkeypatch):
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]], dtype=np.float64)
    monkeypatch.setattr(clomcap.mcli, "looks_zstd", lambda p: False)
    monkeypatch.setattr(clomcap.mp, "inspect_mcap",
                        lambda p: {"/map_3d": {"schema": PC2, "count": 1}})
    monkeypatch.setattr(clomcap.mp, "extract_pointcloud2_frames", lambda p, t: [pts])

    def _boom(points, title, output_png):
        raise RuntimeError("no display")

    monkeypatch.setattr(clomcap.mp, "visualise", _boom)

    src = tmp_path / "a.mcap"
    src.write_bytes(b"x")
    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)

    clomcap.convert_to_ply(str(src), paths, topic=None, want_png=True)
    assert paths["ply"].is_file()


def test_convert_to_ply_cleans_up_decompressed_temp(tmp_path, monkeypatch):
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]], dtype=np.float64)
    tmp_mcap = tmp_path / "decompressed.mcap"
    tmp_mcap.write_bytes(b"x")

    monkeypatch.setattr(clomcap.mcli, "looks_zstd", lambda p: True)
    monkeypatch.setattr(clomcap.mcli, "decompress_zstd", lambda p: str(tmp_mcap))
    monkeypatch.setattr(clomcap.mp, "inspect_mcap",
                        lambda p: {"/map_3d": {"schema": PC2, "count": 1}})
    monkeypatch.setattr(clomcap.mp, "extract_pointcloud2_frames", lambda p, t: [pts])

    src = tmp_path / "a.mcap.zstd"
    src.write_bytes(b"x")
    paths = clomcap.out_paths(str(src), root=tmp_path / "out")
    paths["dir"].mkdir(parents=True)

    clomcap.convert_to_ply(str(src), paths, topic=None, want_png=False)
    assert not tmp_mcap.exists()


# ═══════════════════════════════════════════════════════════════════════════════
# make_grid_file
# ═══════════════════════════════════════════════════════════════════════════════

def test_make_grid_file_covers_cloud_with_margin(tmp_path):
    cloud = tmp_path / "c.xyz"
    cloud.write_text("0.0 0.0 0.0\n4.0 6.0 1.0\n")
    grid = tmp_path / "grid.ply"

    result = clomcap.make_grid_file(str(cloud), grid, spacing=1.0, margin=1.0)

    assert result == grid
    assert grid.is_file()
    gmn, gmx = clomcap.bounds_of(str(grid))
    assert gmn[0] <= -1.0 and gmn[1] <= -1.0
    assert gmx[0] >= 5.0 and gmx[1] >= 7.0


def test_make_grid_file_failure_returns_none(tmp_path, monkeypatch):
    cloud = tmp_path / "c.xyz"
    cloud.write_text("0.0 0.0 0.0\n1.0 1.0 1.0\n")

    def _boom(*a, **k):
        raise RuntimeError("grid rusak")

    monkeypatch.setattr(clomcap.mg, "build_grid", _boom)
    assert clomcap.make_grid_file(str(cloud), tmp_path / "g.ply", 1.0, 1.0) is None


# ═══════════════════════════════════════════════════════════════════════════════
# launch_cloudcompare
# ═══════════════════════════════════════════════════════════════════════════════

def test_launch_cloudcompare_passes_files_in_order(monkeypatch):
    seen = {}

    monkeypatch.setattr(clomcap.shutil, "which", lambda n: "/usr/bin/flatpak")

    def _fake_popen(cmd, **kwargs):
        seen["cmd"] = cmd
        seen["kwargs"] = kwargs
        return object()

    monkeypatch.setattr(clomcap.subprocess, "Popen", _fake_popen)

    clomcap.launch_cloudcompare(["/o/grid.ply", "/o/cloud.ply"])

    assert seen["cmd"][:4] == ["setsid", "flatpak", "run", clomcap.FLATPAK_APP]
    assert seen["cmd"][4:] == ["/o/grid.ply", "/o/cloud.ply"]
    assert seen["kwargs"]["start_new_session"] is True


def test_launch_cloudcompare_without_flatpak_raises(monkeypatch):
    monkeypatch.setattr(clomcap.shutil, "which", lambda n: None)
    with pytest.raises(SystemExit):
        clomcap.launch_cloudcompare(["/o/cloud.ply"])


# ═══════════════════════════════════════════════════════════════════════════════
# CLI / run
# ═══════════════════════════════════════════════════════════════════════════════

def test_parser_defaults():
    args = clomcap.build_parser().parse_args(["a.mcap"])
    assert args.topic is None
    assert args.force is False
    assert args.png is False
    assert args.no_grid is False
    assert args.spacing == 1.0
    assert args.margin == 1.0


def test_run_missing_file_raises(tmp_path):
    args = clomcap.build_parser().parse_args([str(tmp_path / "tidak_ada.mcap")])
    with pytest.raises(SystemExit):
        clomcap.run(args)


def test_run_cloud_input_skips_conversion_and_opens_both(tmp_path, monkeypatch):
    cloud = tmp_path / "c.xyz"
    cloud.write_text("0.0 0.0 0.0\n2.0 2.0 2.0\n")
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")
    monkeypatch.setattr(clomcap, "convert_to_ply",
                        lambda *a, **k: pytest.fail("tidak boleh konversi"))

    opened = {}
    monkeypatch.setattr(clomcap, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcap.run(clomcap.build_parser().parse_args([str(cloud)]))

    assert len(opened["files"]) == 2
    assert opened["files"][0].endswith("grid.ply")
    assert opened["files"][1] == str(cloud)


def test_run_uses_cache_when_fresh(tmp_path, monkeypatch):
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
    monkeypatch.setattr(clomcap, "launch_cloudcompare", lambda files: None)

    clomcap.run(clomcap.build_parser().parse_args([str(src)]))


def test_run_force_reconverts_even_when_cache_fresh(tmp_path, monkeypatch):
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
    monkeypatch.setattr(clomcap, "launch_cloudcompare", lambda files: None)

    clomcap.run(clomcap.build_parser().parse_args([str(src), "--force"]))
    assert called.get("yes") is True


def test_run_no_grid_opens_cloud_only(tmp_path, monkeypatch):
    cloud = tmp_path / "c.xyz"
    cloud.write_text("0.0 0.0 0.0\n2.0 2.0 2.0\n")
    monkeypatch.setattr(clomcap, "OUT_ROOT", tmp_path / "out")

    opened = {}
    monkeypatch.setattr(clomcap, "launch_cloudcompare",
                        lambda files: opened.setdefault("files", files))

    clomcap.run(clomcap.build_parser().parse_args([str(cloud), "--no-grid"]))
    assert opened["files"] == [str(cloud)]


# ═══════════════════════════════════════════════════════════════════════════════
# default_out_root
# ═══════════════════════════════════════════════════════════════════════════════

def test_default_out_root_is_outside_ros2_ws(monkeypatch):
    monkeypatch.delenv("CLOUDCOM_OUT", raising=False)
    root = clomcap.default_out_root()
    assert root.name == "out"
    assert "ros2_ws" not in root.parts


def test_default_out_root_honours_env(monkeypatch, tmp_path):
    monkeypatch.setenv("CLOUDCOM_OUT", str(tmp_path / "lain"))
    assert clomcap.default_out_root() == tmp_path / "lain"


def test_default_out_root_expands_tilde(monkeypatch):
    monkeypatch.setenv("CLOUDCOM_OUT", "~/hasil_scan")
    assert clomcap.default_out_root() == Path.home() / "hasil_scan"
