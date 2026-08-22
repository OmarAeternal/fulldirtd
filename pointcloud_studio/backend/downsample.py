"""Optimasi kerapatan titik sebelum dikirim ke browser.

Downsample berbasis voxel, bukan pembuangan acak: ruang dibagi kubus bersisi
`voxel` dan tiap kubus menyisakan satu titik. Bentuk geometri terjaga; yang
hilang hanya titik yang menumpuk di tempat yang sama.
"""
from __future__ import annotations

import numpy as np

# Jaring pengaman untuk berkas yang jauh lebih besar dari yang biasa dipakai.
# Bila setelah voxel jumlahnya masih di atas ini, voxel digandakan lalu diukur
# lagi. Bukan jalur yang biasa terpakai: pada data sekarang voxel 1 cm sudah
# menghasilkan ~810 ribu titik.
BATAS_TITIK = 3_000_000

# Paling banyak enam penggandaan, jadi voxel akhir maksimum 64x yang diminta
# (untuk bawaan 1 cm berarti mentok di 64 cm). Lebih dari itu, gambarnya sudah
# tidak berguna dan lebih baik jujur bahwa batas tak tercapai.
MAKS_PENGGANDAAN = 6


def _voxel_down(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Nx6 → Nx6, satu titik per kubus bersisi `voxel` meter."""
    import open3d as o3d

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts[:, :3].astype(np.float64))
    pc.colors = o3d.utility.Vector3dVector(pts[:, 3:6].astype(np.float64))
    d = pc.voxel_down_sample(float(voxel))
    return np.concatenate(
        [np.asarray(d.points), np.asarray(d.colors)], axis=1).astype(np.float32)


def optimize(pts: np.ndarray, voxel: float = 0.01, full: bool = False,
             batas: int = BATAS_TITIK) -> tuple:
    """→ (titik, info). `full` mengirim apa adanya, tanpa menyentuh voxel."""
    n = int(len(pts))
    if full:
        return pts, {"voxel": None, "n_asli": n, "n_kirim": n,
                     "melebihi_batas": False}

    v = float(voxel)
    keluar = _voxel_down(pts, v)
    for _ in range(MAKS_PENGGANDAAN):
        if len(keluar) <= batas:
            break
        v *= 2
        keluar = _voxel_down(pts, v)

    return keluar, {"voxel": v, "n_asli": n, "n_kirim": int(len(keluar)),
                    "melebihi_batas": bool(len(keluar) > batas)}
