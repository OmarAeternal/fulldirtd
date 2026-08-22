"""Tes endpoint POST /load — unggahan berkas.

/load dulu mengirim resolusi penuh sementara /open sudah di-downsample.
Dengan multi-berkas, tiga unggahan besar mencekik browser — jadi keduanya
sekarang lewat jalur yang sama.
"""
import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

import server


@pytest.fixture
def client():
    return TestClient(server.app)


def isi_xyz(n=4000, sisi=1.0):
    """Berkas XYZ ascii berisi n titik acak dalam kubus `sisi` meter."""
    rng = np.random.default_rng(7)
    xyz = rng.random((n, 3)) * sisi
    baris = ["%.4f %.4f %.4f 128 128 128" % tuple(p) for p in xyz]
    return "\n".join(baris).encode()


def unggah(client, isi, nama="awan.xyz", **kueri):
    return client.post("/load", files={"file": (nama, isi, "text/plain")},
                       params=kueri)


def test_mengirim_header_downsample(client):
    r = unggah(client, isi_xyz())
    assert r.status_code == 200
    info = json.loads(r.headers["X-Downsample"])
    assert set(info) == {"voxel", "n_asli", "n_kirim", "melebihi_batas"}
    assert info["n_asli"] == 4000


def test_voxel_besar_mengurangi_titik(client):
    r = unggah(client, isi_xyz(), voxel=0.5)
    info = json.loads(r.headers["X-Downsample"])
    assert info["n_kirim"] < info["n_asli"]
    assert json.loads(r.headers["X-Stats"])["n"] == info["n_kirim"]


def test_full_mengirim_semua_titik(client):
    r = unggah(client, isi_xyz(), full=1)
    info = json.loads(r.headers["X-Downsample"])
    assert info["voxel"] is None
    assert info["n_kirim"] == info["n_asli"] == 4000


def test_body_biner_cocok_dengan_jumlah_titik(client):
    r = unggah(client, isi_xyz(), full=1)
    arr = np.frombuffer(r.content, dtype="<f4")
    assert arr.size == 4000 * 6


def test_berkas_tanpa_titik_ditolak(client):
    r = unggah(client, b"", nama="kosong.xyz")
    assert r.status_code == 400


def test_ekstensi_tak_terbaca_ditolak(client):
    r = unggah(client, b"bukan point cloud", nama="catatan.pdf")
    assert r.status_code == 400
