"""Tests untuk mergeway.py — scan berurutan sepanjang lintasan drone.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_mergeway.py -v
"""

import math

import numpy as np
import pytest

import mergeway as mw
import test_areascan as ta


# ═══════════════════════════════════════════════════════════════════════════════
# Adegan: lapangan panjang, tiang di sepanjang lintasan
# ═══════════════════════════════════════════════════════════════════════════════

def _tiang_acak(rng, n=40):
    return np.c_[rng.uniform(-12, 45, n), rng.uniform(-12, 12, n)]


def scan_di(posisi, yaw_deg, rng, tiang, miring=8.0, jangkau=15.0, sektor=110.0):
    """Scan di kerangka sensor, sensor di atas `posisi`, drone ikut terekam."""
    r = np.sqrt(rng.uniform(0.3 ** 2, jangkau ** 2, 40000))
    az = rng.uniform(-math.pi, math.pi, len(r))
    tanah = np.c_[posisi[0] + r * np.cos(az), posisi[1] + r * np.sin(az),
                  rng.normal(0, 0.01, len(r))]
    batang = []
    for x, y in tiang:
        a = rng.uniform(0, 2 * math.pi, 60)
        batang.append(np.c_[x + 0.15 * np.cos(a), y + 0.15 * np.sin(a),
                            rng.uniform(0.05, 1.0, 60)])
    batang = np.vstack(batang) if batang else np.zeros((0, 3))
    sensor = np.array([posisi[0], posisi[1], ta.TINGGI_DRONE])
    keluar = []
    for p in (tanah, batang):
        lok = (p - sensor) @ ta._rot_z(yaw_deg)
        az = np.degrees(np.arctan2(lok[:, 1], lok[:, 0]))
        keluar.append(lok[(np.abs(az) < sektor) & (np.linalg.norm(lok[:, :2], axis=1) < jangkau)])
    drone = rng.uniform([-0.2, -0.3, -0.2], [0.2, 0.3, 0.2], (3000, 3))
    keluar.append(drone)
    return np.vstack(keluar) @ ta._rot_x(miring).T


def lintasan(posisi, tiang_n=40, seed=0, yaw=0.0):
    rng = np.random.default_rng(seed)
    tiang = _tiang_acak(rng, tiang_n) if tiang_n else np.zeros((0, 2))
    return {f"s{k}": scan_di(p, yaw, rng, tiang) for k, p in enumerate(posisi)}


def _diam(*a, **k):
    pass


# ═══════════════════════════════════════════════════════════════════════════════
# Pencocokan
# ═══════════════════════════════════════════════════════════════════════════════

def test_cocok_diterima_bila_tunggal():
    rng = np.random.default_rng(1)
    P = rng.uniform(-10, 10, (12, 2))
    B = P - [10.8, 0.6]
    h = mw.cocokkan_lintasan(P, B, yaw0=0.0, t0=[10.0, 0.0])
    assert h["diterima"]
    assert np.allclose(h["geser"], [10.8, 0.6], atol=0.05)


def test_cocok_ditolak_bila_kurang():
    P = np.array([[1.0, 2.0], [5.0, -3.0]])
    B = P - [10.0, 0.0]
    h = mw.cocokkan_lintasan(P, B, yaw0=0.0, t0=[10.0, 0.0])
    assert not h["diterima"]
    assert "< 3" in h["alasan"]


def test_cocok_ditolak_bila_berulang():
    """Deret tiang berjarak sama sejajar lintasan: geseran sebesar jaraknya sama baiknya."""
    P = np.c_[np.arange(0, 30, 2.0), np.zeros(15)]
    B = P[3:10] - [10.0, 0.0]
    h = mw.cocokkan_lintasan(P, B, yaw0=0.0, t0=[10.0, 0.0])
    assert not h["diterima"]
    assert "tunggal" in h["alasan"]


def test_tanpa_penanda_ditolak():
    h = mw.cocokkan_lintasan(np.zeros((3, 2)), np.zeros((0, 4)), 0.0, [10, 0])
    assert not h["diterima"]


# ═══════════════════════════════════════════════════════════════════════════════
# Rangkaian
# ═══════════════════════════════════════════════════════════════════════════════

def test_posisi_dari_penanda():
    benar = [(0, 0), (10.7, 0.8), (21.0, 1.2), (30.5, 0.9)]
    h = mw.susun(lintasan(benar, tiang_n=120), log=_diam)
    for k, nm in enumerate(h["nama"]):
        c = h["catatan"][nm]
        if k:
            assert c["asal"] == "penanda"
        assert np.allclose(c["geser"], benar[k], atol=0.2)


def test_tanpa_benda_dilanjutkan_jarak():
    h = mw.susun(lintasan([(0, 0), (11, 0), (22, 0)], tiang_n=0), log=_diam)
    for k, nm in enumerate(h["nama"]):
        assert np.allclose(h["catatan"][nm]["geser"], [10.0 * k, 0.0], atol=1e-9)
        if k:
            assert h["catatan"][nm]["asal"] == "lanjutan"


def test_arah_maju_dipakai():
    h = mw.susun(lintasan([(0, 0), (0, 10)], tiang_n=0), maju=90.0, jarak=7.0,
                 log=_diam)
    assert np.allclose(h["catatan"]["s1"]["geser"], [0.0, 7.0], atol=1e-9)


def test_tanpa_cocok_murni_lanjutan():
    benar = [(0, 0), (10.7, 0.8)]
    h = mw.susun(lintasan(benar), cocokkan=False, log=_diam)
    assert h["catatan"]["s1"]["asal"] == "lanjutan"
    assert np.allclose(h["catatan"]["s1"]["geser"], [10.0, 0.0])


def test_drone_dibuang_dan_tanah_di_nol():
    h = mw.susun(lintasan([(0, 0), (10, 0)], tiang_n=0), log=_diam)
    for nm in h["nama"]:
        assert nm in h["drone"]
        assert (h["peta"][nm][:, 2] < 1.5).all()
        assert abs(np.median(h["peta"][nm][:, 2])) < 0.05


def test_jumlah_berkas_dijaga():
    with pytest.raises(ValueError):
        mw.susun({"s0": np.zeros((10, 3))})
