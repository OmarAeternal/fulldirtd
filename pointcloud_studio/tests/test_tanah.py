"""Tes perataan tanah — backend/tanah.py dan endpoint POST /tanah.

Semua adegan sintetis meniru lapangan terbuka yang diuji sekarang: satu tanah
miring, beberapa benda di atasnya, tanpa tembok.
"""
import numpy as np
import pytest
from fastapi.testclient import TestClient

import server
import tanah


def putar(sumbu, deg):
    """Matriks rotasi Rodrigues di sekitar `sumbu`."""
    a = np.asarray(sumbu, float)
    a = a / np.linalg.norm(a)
    t = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def lapangan(miring=8.0, sumbu=(1, 0.4, 0), tinggi=1.2, seed=0, n=40000,
             bising=0.01):
    """Tanah 24x24 m di z = -tinggi, beberapa benda tegak, lalu dimiringkan."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-12, 12, (n, 2))
    tanah_ = np.column_stack([xy, np.full(n, -tinggi) + rng.normal(0, bising, n)])
    benda = []
    for cx, cy in [(3, 2), (-4, 5), (6, -6)]:          # tiang / orang
        m = 1500
        benda.append(np.column_stack([
            cx + rng.normal(0, 0.15, m), cy + rng.normal(0, 0.15, m),
            rng.uniform(-tinggi, 0.8, m)]))
    xyz = np.vstack([tanah_] + benda)
    return xyz @ putar(sumbu, miring).T


def normal_tanah_sejati(miring, sumbu):
    return putar(sumbu, miring) @ np.array([0.0, 0.0, 1.0])


def sudut(a, b):
    return np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), -1, 1)))


def terapkan(M, xyz):
    M = np.asarray(M)
    return xyz @ M[:3, :3].T + M[:3, 3]


# ---------------------------------------------------------------------------

def test_menemukan_normal_tanah_miring():
    xyz = lapangan(miring=8.0)
    h = tanah.cari_tanah(xyz)
    assert h is not None
    assert sudut(h["normal"], normal_tanah_sejati(8.0, (1, 0.4, 0))) < 0.3
    assert h["miring_deg"] == pytest.approx(8.0, abs=0.3)


def test_matriks_membuat_tanah_datar_di_z_nol():
    xyz = lapangan(miring=10.0, tinggi=3.3)
    h = tanah.cari_tanah(xyz)
    rata = terapkan(h["matriks"], xyz)
    # titik tanah = 40.000 baris pertama
    z = rata[:40000, 2]
    assert abs(np.median(z)) < 0.01
    assert np.std(z) < 0.02            # bising 1 cm, tanpa kemiringan tersisa


def test_matriks_kaku_dan_tidak_memutar_arah_hadap():
    """Putaran terkecil: sumbu X tetap menunjuk ke arah yang sama dari atas."""
    xyz = lapangan(miring=9.0, sumbu=(0.3, 1, 0))
    M = np.asarray(tanah.cari_tanah(xyz)["matriks"])
    R = M[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)
    x_baru = R @ np.array([1.0, 0, 0])
    assert np.degrees(np.arctan2(x_baru[1], x_baru[0])) == pytest.approx(0, abs=1.0)


def test_tanah_datar_tidak_diputar():
    xyz = lapangan(miring=0.0)
    h = tanah.cari_tanah(xyz)
    assert h["miring_deg"] < 0.2


def test_gumpalan_dekat_sensor_tidak_mengalahkan_tanah():
    """Scan 0186/0192: sebagian besar titik jatuh dalam 30 cm dari sensor.

    Dihitung per titik, bidang kecil yang padat itu menang. Dihitung per luas
    (voxel), tanah yang lebar tetap menang.
    """
    rng = np.random.default_rng(3)
    xyz = lapangan(miring=0.0, n=15000)
    m = 120000                         # 8x titik tanah, miring 40°
    uv = rng.uniform(-0.25, 0.25, (m, 2))
    gumpal = np.column_stack([uv, np.zeros(m)]) @ putar((1, 0, 0), 40).T
    h = tanah.cari_tanah(np.vstack([xyz, gumpal]))
    assert h["miring_deg"] < 0.5


def test_scan_miring_jauh_tetap_ditemukan():
    """Scan 0186 terukur miring ~36°: batasnya harus cukup longgar."""
    h = tanah.cari_tanah(lapangan(miring=36.0))
    assert h["miring_deg"] == pytest.approx(36.0, abs=0.5)


def test_deterministik():
    xyz = lapangan(miring=7.0, seed=5)
    a = tanah.cari_tanah(xyz)
    b = tanah.cari_tanah(xyz)
    assert np.array_equal(np.asarray(a["matriks"]), np.asarray(b["matriks"]))


def test_normal_menghadap_atas_walau_scan_terbalik_sedikit():
    h = tanah.cari_tanah(lapangan(miring=20.0, sumbu=(0, -1, 0)))
    assert h["normal"][2] > 0


def test_terlalu_sedikit_titik_none():
    assert tanah.cari_tanah(np.zeros((10, 3))) is None


def test_awan_tanpa_bidang_none():
    rng = np.random.default_rng(1)
    bola = rng.normal(0, 1, (20000, 3))
    bola /= np.linalg.norm(bola, axis=1, keepdims=True) / 5
    assert tanah.cari_tanah(bola) is None


# ---------------------------------------------------------------------------

@pytest.fixture
def client():
    return TestClient(server.app)


def test_endpoint_tanah(client):
    xyz = lapangan(miring=6.0).astype("<f4")
    r = client.post("/tanah", content=xyz.tobytes())
    assert r.status_code == 200
    d = r.json()
    assert d["ditemukan"] is True
    assert d["miring_deg"] == pytest.approx(6.0, abs=0.3)
    assert len(d["matriks"]) == 4 and len(d["matriks"][0]) == 4


def test_endpoint_tanpa_tanah(client):
    r = client.post("/tanah", content=np.zeros((5, 3), "<f4").tobytes())
    assert r.status_code == 200
    assert r.json()["ditemukan"] is False


def test_endpoint_menolak_bukan_kelipatan_3(client):
    r = client.post("/tanah", content=b"\x00" * 16)
    assert r.status_code == 400
