"""Tes optimasi kerapatan titik."""
import numpy as np

import downsample


def rapat(n: int, sisi: float = 1.0) -> np.ndarray:
    """n titik acak dalam kubus `sisi` meter, Nx6 (xyz + rgb 0..1)."""
    rng = np.random.default_rng(0)
    xyz = rng.random((n, 3), dtype=np.float32) * np.float32(sisi)
    rgb = np.full((n, 3), 0.5, dtype=np.float32)
    return np.concatenate([xyz, rgb], axis=1)


def test_full_mengembalikan_titik_apa_adanya():
    pts = rapat(5000)
    keluar, info = downsample.optimize(pts, voxel=0.01, full=True)
    assert len(keluar) == len(pts)
    assert info["n_kirim"] == info["n_asli"] == 5000
    assert info["voxel"] is None


def test_voxel_mengurangi_titik_yang_menumpuk():
    # 50.000 titik dalam kubus 1 m. Voxel 10 cm membagi tiap sumbu jadi paling
    # banyak 11 sel (kisinya diselaraskan ke batas minimum data, jadi ada satu
    # sel tambahan di ujung), sehingga hasilnya tidak mungkin melebihi 11^3.
    pts = rapat(50_000, sisi=1.0)
    keluar, info = downsample.optimize(pts, voxel=0.1)
    assert len(keluar) <= 11 ** 3
    assert info["n_asli"] == 50_000
    assert info["n_kirim"] == len(keluar)
    assert info["voxel"] == 0.1


def test_hasil_tetap_nx6_dan_warna_terbawa():
    pts = rapat(50_000, sisi=1.0)
    keluar, _ = downsample.optimize(pts, voxel=0.1)
    assert keluar.shape[1] == 6
    assert keluar.dtype == np.float32
    assert np.allclose(keluar[:, 3:], 0.5, atol=1e-6)


def test_batas_bawaan_tiga_juta():
    assert downsample.BATAS_TITIK == 3_000_000


def test_voxel_digandakan_sampai_muat_batas():
    pts = rapat(50_000, sisi=1.0)
    keluar, info = downsample.optimize(pts, voxel=0.01, batas=5_000)
    assert len(keluar) <= 5_000
    # voxel akhir harus kelipatan dua dari yang diminta, bukan angka sembarang
    assert info["voxel"] > 0.01
    assert round(info["voxel"] / 0.01) in (2, 4, 8, 16, 32, 64)
    assert info["melebihi_batas"] is False


def test_voxel_tidak_diubah_bila_sudah_di_bawah_batas():
    pts = rapat(50_000, sisi=1.0)
    _, info = downsample.optimize(pts, voxel=0.1, batas=5_000)
    assert info["voxel"] == 0.1


def test_menyerah_setelah_enam_penggandaan_tanpa_gagal():
    # batas 1 titik mustahil dipenuhi; voxel mentok di 64x lalu data dikirim
    # apa adanya dengan penanda, bukan melempar galat.
    pts = rapat(50_000, sisi=1.0)
    keluar, info = downsample.optimize(pts, voxel=0.01, batas=1)
    assert len(keluar) > 1
    assert info["voxel"] == 0.01 * 64
    assert info["melebihi_batas"] is True


def test_cloud_kosong_tidak_menggagalkan():
    kosong = np.zeros((0, 6), dtype=np.float32)
    keluar, info = downsample.optimize(kosong, voxel=0.01)
    assert len(keluar) == 0
    assert info["n_asli"] == 0
    assert info["n_kirim"] == 0
