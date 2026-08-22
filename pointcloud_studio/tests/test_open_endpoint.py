"""Tes endpoint GET /open — memuat berkas dari path di disk."""
import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

import server


@pytest.fixture
def client():
    return TestClient(server.app)


def tulis_xyz(dirpath, nama="awan.xyz", n=2000, sisi=1.0):
    """Berkas XYZ ascii berisi n titik acak dalam kubus `sisi` meter."""
    rng = np.random.default_rng(1)
    xyz = rng.random((n, 3)) * sisi
    rgb = np.full((n, 3), 128.0)
    baris = ["%.4f %.4f %.4f %d %d %d" % (*p, *c) for p, c in zip(xyz, rgb)]
    f = dirpath / nama
    f.write_text("\n".join(baris))
    return f


def test_path_relatif_ditolak(client):
    r = client.get("/open", params={"path": "awan.ply"})
    assert r.status_code == 400


def test_berkas_tidak_ada_ditolak(client, tmp_path):
    r = client.get("/open", params={"path": str(tmp_path / "hantu.ply")})
    assert r.status_code == 400


def test_direktori_ditolak(client, tmp_path):
    r = client.get("/open", params={"path": str(tmp_path)})
    assert r.status_code == 400


def test_ekstensi_asing_ditolak(client, tmp_path):
    f = tmp_path / "rahasia.env"
    f.write_text("TOKEN=jangan-dibaca")
    r = client.get("/open", params={"path": str(f)})
    assert r.status_code == 400


def test_berkas_sah_dibalas_biner_dan_statistik(client, tmp_path):
    f = tulis_xyz(tmp_path)
    r = client.get("/open", params={"path": str(f), "full": "1"})

    assert r.status_code == 200
    assert r.headers["content-type"] == "application/octet-stream"

    titik = np.frombuffer(r.content, dtype="<f4")
    assert titik.size % 6 == 0
    assert titik.size // 6 == 2000

    stats = json.loads(r.headers["X-Stats"])
    assert stats["n"] == 2000
    assert len(stats["size"]) == 3


def test_voxel_diterapkan_dan_dilaporkan(client, tmp_path):
    f = tulis_xyz(tmp_path, n=20_000, sisi=1.0)
    r = client.get("/open", params={"path": str(f), "voxel": "0.1"})

    assert r.status_code == 200
    info = json.loads(r.headers["X-Downsample"])
    assert info["n_asli"] == 20_000
    assert info["n_kirim"] < 20_000
    assert info["voxel"] == 0.1

    titik = np.frombuffer(r.content, dtype="<f4")
    assert titik.size // 6 == info["n_kirim"]


def test_full_melewati_optimasi(client, tmp_path):
    f = tulis_xyz(tmp_path, n=20_000, sisi=1.0)
    r = client.get("/open", params={"path": str(f), "voxel": "0.1", "full": "1"})

    info = json.loads(r.headers["X-Downsample"])
    assert info["n_kirim"] == 20_000
    assert info["voxel"] is None
