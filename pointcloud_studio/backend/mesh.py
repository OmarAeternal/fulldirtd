"""Meshing berbasis-sudut (depth-map triangulation) untuk scan dari satu titik pandang.

Karena scanner diam di satu titik, tiap titik dapat dinyatakan sebagai arah (azimut,
elevasi) + jarak dari origin sensor — mirip peta kedalaman panorama. Titik bertetangga
dalam ruang sudut disambung menjadi segitiga. Kotak dengan lompatan kedalaman besar atau
sel kosong dilewati, sehingga celah dibiarkan bolong alih-alih menutup permukaan palsu.
"""
from __future__ import annotations
import numpy as np


def build_mesh(points: np.ndarray, angular_res_deg: float = 0.5,
               depth_jump: float = 0.25):
    """Bangun mesh dari titik.

    points: Nx3 (atau Nx6, kolom rgb diabaikan) dalam frame sensor (origin = 0,0,0).
    angular_res_deg: ukuran sel grid sudut (derajat). Lebih kecil = lebih rapat/detail.
    depth_jump: ambang selisih jarak relatif antar titik dalam satu kotak; di atas ini
                kotak dilewati (menghindari segitiga melar di tepi objek/celah).

    Kembalikan (vertices Nx3 float32, faces Mx3 int32). Faces mengindeks vertices.
    """
    xyz = points[:, :3].astype(np.float64)
    n = len(xyz)
    if n < 100:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.int32)

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.sqrt(x * x + y * y + z * z)
    valid = r > 1e-6
    xyz, r = xyz[valid], r[valid]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

    azimuth = np.arctan2(y, x)              # -pi..pi
    elevation = np.arcsin(np.clip(z / r, -1, 1))  # -pi/2..pi/2

    res = np.radians(angular_res_deg)
    ai = np.floor((azimuth + np.pi) / res).astype(np.int64)
    ei = np.floor((elevation + np.pi / 2) / res).astype(np.int64)

    n_az = int(np.ceil(2 * np.pi / res)) + 1
    n_el = int(np.ceil(np.pi / res)) + 1

    # Untuk tiap sel grid, simpan indeks titik TERDEKAT (r terkecil) — permukaan terlihat.
    cell = ai * n_el + ei
    order = np.argsort(r)  # terdekat dulu
    cell_sorted = cell[order]
    first_unique, first_idx = np.unique(cell_sorted, return_index=True)
    point_of_cell = order[first_idx]  # indeks titik pemenang per sel

    # grid[a, e] = indeks titik (atau -1)
    grid = np.full(n_az * n_el, -1, dtype=np.int64)
    grid[first_unique] = point_of_cell
    grid = grid.reshape(n_az, n_el)

    faces = []
    # Iterasi tiap kotak (a,e)-(a+1,e)-(a,e+1)-(a+1,e+1)
    g00 = grid[:-1, :-1]
    g10 = grid[1:, :-1]
    g01 = grid[:-1, 1:]
    g11 = grid[1:, 1:]
    filled = (g00 >= 0) & (g10 >= 0) & (g01 >= 0) & (g11 >= 0)

    aa, ee = np.where(filled)
    if len(aa) == 0:
        return xyz.astype(np.float32), np.empty((0, 3), np.int32)

    i00 = grid[aa, ee]
    i10 = grid[aa + 1, ee]
    i01 = grid[aa, ee + 1]
    i11 = grid[aa + 1, ee + 1]

    r00, r10, r01, r11 = r[i00], r[i10], r[i01], r[i11]
    rmax = np.maximum.reduce([r00, r10, r01, r11])
    rmin = np.minimum.reduce([r00, r10, r01, r11])
    # ambang adaptif: lewati kotak bila selisih jarak > depth_jump * rata-rata jarak
    ravg = (r00 + r10 + r01 + r11) * 0.25
    keep = (rmax - rmin) <= (depth_jump * ravg + 0.02)

    i00, i10, i01, i11 = i00[keep], i10[keep], i01[keep], i11[keep]
    # dua segitiga per kotak
    tri1 = np.stack([i00, i10, i11], axis=1)
    tri2 = np.stack([i00, i11, i01], axis=1)
    faces = np.concatenate([tri1, tri2], axis=0).astype(np.int32)

    return xyz.astype(np.float32), faces
