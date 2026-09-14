"""Tests untuk areascan.py — satu posisi, beberapa arah hadap.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_areascan.py -v

Adegannya sintetis tapi meniru scan_0187-0190: drone 3 m di atas lapangan,
rig miring 10-19 derajat, tiap scan hanya melihat sektor ~220 derajat, titik
jarang dan tiap titik terekam dua kali.
"""

import math

import numpy as np
import pytest

import areascan as ar


# ═══════════════════════════════════════════════════════════════════════════════
# Adegan sintetis
# ═══════════════════════════════════════════════════════════════════════════════

TINGGI_DRONE = 3.0
BENDA = np.array([[1.2, 3.4], [-3.2, 0.9], [4.8, 4.3], [-2.6, 6.6], [4.5, -5.7],
                  [-2.4, -5.5], [7.5, 1.0], [-6.0, -4.0], [1.0, -7.5]])


def _rot_z(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _rot_x(deg):
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)],
                     [0.0, math.sin(a), math.cos(a)]])


def dunia(rng, lengkung_uv=0.0):
    """Titik dunia: tanah, tiang-tiang kecil, dan drone. Tumpu di (0, 0)."""
    r = np.sqrt(rng.uniform(0.3 ** 2, 12.0 ** 2, 60000))
    az = rng.uniform(-math.pi, math.pi, len(r))
    tanah = np.c_[r * np.cos(az), r * np.sin(az), rng.normal(0, 0.01, len(r))]
    tiang = []
    for x, y in BENDA:
        n = 60
        a = rng.uniform(0, 2 * math.pi, n)
        tiang.append(np.c_[x + 0.15 * np.cos(a), y + 0.15 * np.sin(a),
                           rng.uniform(0.05, 1.0, n)])
    drone = rng.uniform([-0.2, -0.3, TINGGI_DRONE - 0.2],
                        [0.2, 0.3, TINGGI_DRONE + 0.2], (3000, 3))
    return tanah, np.vstack(tiang), drone


def scan(yaw_deg, miring_deg, rng, lengkung_uv=0.0, sektor=110.0):
    """Satu scan di kerangka sensor. Sensor 6 cm di samping pusat drone."""
    tanah, tiang, drone = dunia(rng)
    sensor = np.array([0.06, 0.0, TINGGI_DRONE])
    keluar = []
    for p in (tanah, tiang):
        lok = (p - sensor) @ _rot_z(yaw_deg)          # dunia → hadap sensor
        az = np.degrees(np.arctan2(lok[:, 1], lok[:, 0]))
        lok = lok[np.abs(az) < sektor]
        if lengkung_uv:
            lok[:, 2] += lengkung_uv * lok[:, 0] * lok[:, 1]
        keluar.append(lok)
    keluar.append((drone - sensor) @ _rot_z(yaw_deg))
    xyz = np.vstack(keluar) @ _rot_x(miring_deg).T
    return np.vstack([xyz, xyz])                       # tiap titik dua kali


def empat_scan(arah="cw", langkah=(0.0, 84.0, 93.0, 88.0), lengkung_uv=0.0,
               seed=0):
    rng = np.random.default_rng(seed)
    tanda = -1.0 if arah == "cw" else 1.0
    yaw = np.cumsum(langkah) * tanda
    miring = [10.5, 10.8, 14.9, 19.3]
    awan = {f"s{k}": scan(y, m, rng, lengkung_uv) for k, (y, m)
            in enumerate(zip(yaw, miring))}
    return awan, yaw


def _diam(*a, **k):
    pass


# ═══════════════════════════════════════════════════════════════════════════════
# Dasar
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize("a,b,hasil", [(10, 350, 20), (-170, 170, 20),
                                       (180, 0, 180), (0, 180, 180), (90, 90, 0)])
def test_beda_sudut_dibungkus(a, b, hasil):
    assert ar.beda_sudut(a, b) == pytest.approx(hasil)


def test_buang_kembar():
    p = np.array([[1.0, 2, 3], [1.0, 2, 3], [0.0, 0, 0]])
    assert len(ar.buang_kembar(p)) == 2


def test_tanah_setempat_mengikuti_tanah_miring():
    """Tanah miring 5% sepanjang 10 m: tetap terbaca dekat nol, tiang tetap tinggi."""
    x, y = np.meshgrid(np.arange(0, 10, 0.1), np.arange(0, 3, 0.1))
    tanah = np.c_[x.ravel(), y.ravel(), 0.05 * x.ravel()]
    tiang = np.c_[np.full(10, 5.0), np.full(10, 1.5), 0.25 + np.linspace(0.3, 1.0, 10)]
    h = ar.tanah_setempat(np.vstack([tanah, tiang]))
    assert np.abs(h[:len(tanah)]).max() < 0.1
    assert h[len(tanah):].min() > 0.25


# ═══════════════════════════════════════════════════════════════════════════════
# Pencocokan penanda
# ═══════════════════════════════════════════════════════════════════════════════

def test_satu_penanda_cukup_dengan_tumpu():
    """Dengan pusat putar diketahui, satu benda yang sama menentukan yaw."""
    A = np.array([[3.0, 4.0, 50, 0.5]])
    B = A.copy()
    B[:, :2] = ar._putar2(A[:, :2], 70.0)   # sumber terputar +70
    h = ar.cocokkan(A, B, yaw_tebak=-90.0)
    assert h["yaw"] == pytest.approx(-70.0, abs=1e-6)
    assert h["cocok"] == 1


def test_cocokkan_menolak_di_luar_jendela():
    A = np.array([[3.0, 4.0, 50, 0.5]])
    B = A.copy()
    B[:, :2] = ar._putar2(A[:, :2], 70.0)
    assert ar.cocokkan(A, B, yaw_tebak=+90.0) is None


def test_cocokkan_memilih_rasi_bukan_benda_kebetulan():
    """Cincin penanda simetris: yang benar adalah yaw yang menjatuhkan BANYAK penanda."""
    A = np.c_[BENDA, np.full(len(BENDA), 50), np.full(len(BENDA), 0.5)]
    B = A.copy()
    B[:, :2] = ar._putar2(A[:, :2], 83.0) + [0.1, -0.05]
    h = ar.cocokkan(A, B, yaw_tebak=-90.0)
    assert h["yaw"] == pytest.approx(-83.0, abs=1.0)
    assert h["cocok"] == len(BENDA)


# ═══════════════════════════════════════════════════════════════════════════════
# Drone
# ═══════════════════════════════════════════════════════════════════════════════

def test_drone_melayang_ditemukan_orang_tidak():
    rng = np.random.default_rng(1)
    tanah = np.c_[rng.uniform(-8, 8, (20000, 2)), rng.normal(0, 0.01, 20000)]
    orang = np.c_[rng.normal(4, 0.1, (400, 2)), rng.uniform(0, 1.8, 400)]
    drone = rng.uniform([-0.2, -0.3, 2.8], [0.2, 0.3, 3.2], (2000, 3))
    xyz = np.vstack([tanah, orang, drone])
    d = ar.cari_drone(xyz, np.eye(4))
    assert d is not None
    assert np.allclose(d["pusat"][:2], [0, 0], atol=0.05)
    assert d["topeng"][-2000:].all()
    assert not d["topeng"][len(tanah):len(tanah) + 400].any()


def test_tanpa_drone_mengembalikan_none():
    rng = np.random.default_rng(2)
    tanah = np.c_[rng.uniform(-8, 8, (5000, 2)), rng.normal(0, 0.01, 5000)]
    assert ar.cari_drone(tanah, np.eye(4)) is None


# ═══════════════════════════════════════════════════════════════════════════════
# Rangkaian penuh
# ═══════════════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="module")
def hasil_cw():
    awan, yaw = empat_scan("cw")
    return ar.susun(awan, arah="cw", log=_diam), yaw


def test_yaw_kembali(hasil_cw):
    h, yaw = hasil_cw
    for k, nm in enumerate(h["nama"]):
        assert ar.beda_sudut(h["catatan"][nm]["yaw"], yaw[k]) == pytest.approx(0, abs=1.0)
        assert h["catatan"][nm]["peringatan"] == []


def test_drone_jadi_tumpu_dan_dibuang(hasil_cw):
    h, _ = hasil_cw
    for nm in h["nama"]:
        assert nm in h["drone"]
        assert h["drone"][nm]["tinggi"] == pytest.approx(TINGGI_DRONE, abs=0.1)
        assert (h["peta"][nm][:, 2] < 1.5).all()


def test_titik_kembar_dibuang(hasil_cw):
    h, _ = hasil_cw
    for c in h["catatan"].values():
        assert c["titik_unik"] == c["titik_awal"] // 2


def test_penanda_menumpuk_di_peta(hasil_cw):
    """Benda yang sama dari scan berbeda harus jatuh di tempat yang sama."""
    h, _ = hasil_cw
    for nm in h["nama"]:
        p = h["peta"][nm]
        for x, y in BENDA:
            dekat = np.linalg.norm(p[:, :2] - [x - 0.06, y], axis=1) < 0.4
            if dekat.sum() > 20:
                assert p[dekat, 2].max() > 0.7


def test_arah_terbalik_diperingatkan():
    awan, _ = empat_scan("cw")
    h = ar.susun(awan, arah="ccw", log=_diam)
    peringatan = " ".join(w for c in h["catatan"].values() for w in c["peringatan"])
    assert "terbalik" in peringatan


def test_ccw_bekerja_sama_baiknya():
    awan, yaw = empat_scan("ccw", seed=3)
    h = ar.susun(awan, arah="ccw", log=_diam)
    for k, nm in enumerate(h["nama"]):
        assert ar.beda_sudut(h["catatan"][nm]["yaw"], yaw[k]) == pytest.approx(0, abs=1.0)


def test_dua_scan_cukup():
    awan, yaw = empat_scan("cw")
    dua = {k: awan[k] for k in ("s0", "s1")}
    h = ar.susun(dua, arah="cw", log=_diam)
    assert ar.beda_sudut(h["catatan"]["s1"]["yaw"], yaw[1]) == pytest.approx(0, abs=1.0)


def test_sambung_tanah_menyerap_lengkung_rig():
    """Lengkung pelana yang sama di kerangka sensor harus hilang dari sambungan."""
    awan, _ = empat_scan("cw", lengkung_uv=-0.004, seed=4)
    h = ar.susun(awan, arah="cw", log=_diam)
    s = h["sambung"]
    assert h["koreksi"] is not None
    assert s["uji_sebelum"]["median"] > 0.05
    assert s["uji_sesudah"]["median"] < 0.02


def test_deterministik():
    awan, _ = empat_scan("cw", seed=5)
    a = ar.susun(awan, log=_diam)
    b = ar.susun(awan, log=_diam)
    for nm in a["nama"]:
        assert np.array_equal(a["pose"][nm], b["pose"][nm])
        assert np.array_equal(a["peta"][nm], b["peta"][nm])


def test_jumlah_berkas_dijaga():
    with pytest.raises(ValueError):
        ar.susun({"s0": np.zeros((10, 3))})
