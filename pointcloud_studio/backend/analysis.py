"""Analisis dimensi ruangan dengan RANSAC (Open3D).

Mendeteksi bidang-bidang dominan (lantai, plafon, dinding), lalu menurunkan:
- dimensi ruangan (panjang, lebar, tinggi),
- RMSE planaritas tiap bidang (ukuran presisi/noise),
- sudut antar dinding (uji ortogonalitas terhadap 90°).

Hasil terhubung ke tabel validasi Kategori A & B pada PANDUAN_VALIDASI_DATA.
"""
from __future__ import annotations
import numpy as np
import open3d as o3d


def _o3d_cloud(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return pc


def analyze(points: np.ndarray, distance_threshold: float = 0.03,
            max_planes: int = 8, min_plane_points: int = 500) -> dict:
    """Jalankan analisis. points: Nx3 atau Nx6. Kembalikan dict siap-JSON."""
    xyz = np.asarray(points[:, :3], dtype=np.float64)
    n_total = len(xyz)

    # --- Bounding box (acuan kasar dimensi) ---
    mn = xyz.min(axis=0)
    mx = xyz.max(axis=0)
    bbox = (mx - mn)

    result = {
        "n_points": int(n_total),
        "bbox": {"x": float(bbox[0]), "y": float(bbox[1]), "z": float(bbox[2])},
        "range": {
            "x": [float(mn[0]), float(mx[0])],
            "y": [float(mn[1]), float(mx[1])],
            "z": [float(mn[2]), float(mx[2])],
        },
        "planes": [],
        "walls": {},
        "notes": [],
    }

    if n_total < min_plane_points:
        result["notes"].append("Titik terlalu sedikit untuk deteksi bidang RANSAC.")
        return result

    pc = _o3d_cloud(xyz)
    remaining = pc
    planes = []

    for _ in range(max_planes):
        if len(remaining.points) < min_plane_points:
            break
        model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)
        if len(inliers) < min_plane_points:
            break
        a, b, c, d = model
        inlier_cloud = remaining.select_by_index(inliers)
        pts = np.asarray(inlier_cloud.points)
        normal = np.array([a, b, c], dtype=np.float64)
        nnorm = normal / (np.linalg.norm(normal) + 1e-12)
        # RMSE planaritas = akar rata-rata kuadrat jarak inlier ke bidang
        dist = np.abs(pts @ normal + d) / (np.linalg.norm(normal) + 1e-12)
        rmse = float(np.sqrt(np.mean(dist ** 2)))

        # klasifikasi arah: |nz| tinggi = horizontal (lantai/plafon), rendah = dinding
        nz = abs(nnorm[2])
        if nz > 0.85:
            centroid_z = float(pts[:, 2].mean())
            kind = "lantai/plafon"
        elif nz < 0.35:
            kind = "dinding"
        else:
            kind = "miring"

        planes.append({
            "kind": kind,
            "normal": [float(v) for v in nnorm],
            "d": float(d),
            "n_inliers": int(len(inliers)),
            "rmse_m": rmse,
            "centroid": [float(v) for v in pts.mean(axis=0)],
        })
        remaining = remaining.select_by_index(inliers, invert=True)

    result["planes"] = planes

    # --- Dimensi dari bidang bila memungkinkan ---
    horiz = [p for p in planes if p["kind"] == "lantai/plafon"]
    walls = [p for p in planes if p["kind"] == "dinding"]

    # Tinggi = jarak antara dua bidang horizontal terjauh (lantai vs plafon)
    if len(horiz) >= 2:
        zc = sorted(p["centroid"][2] for p in horiz)
        result["walls"]["tinggi_m"] = float(zc[-1] - zc[0])
        result["walls"]["metode_tinggi"] = "lantai↔plafon (RANSAC)"
    else:
        result["walls"]["tinggi_m"] = float(bbox[2])
        result["walls"]["metode_tinggi"] = "bounding box (bidang horizontal <2)"

    # Ortogonalitas: sudut antar pasangan normal dinding
    ortho = []
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            n1 = np.array(walls[i]["normal"])
            n2 = np.array(walls[j]["normal"])
            cosang = abs(float(np.dot(n1, n2)))
            cosang = min(1.0, max(0.0, cosang))
            ang = np.degrees(np.arccos(cosang))
            ortho.append({
                "pasangan": [i + 1, j + 1],
                "sudut_deg": round(ang, 2),
                "deviasi_dari_90": round(abs(ang - 90.0), 2),
            })
    result["walls"]["ortogonalitas"] = ortho
    result["walls"]["panjang_bbox_m"] = float(max(bbox[0], bbox[1]))
    result["walls"]["lebar_bbox_m"] = float(min(bbox[0], bbox[1]))

    # Ringkasan planaritas
    if planes:
        rmses = [p["rmse_m"] for p in planes]
        result["walls"]["rmse_planaritas_rata_m"] = float(np.mean(rmses))
        result["walls"]["rmse_planaritas_terbaik_m"] = float(np.min(rmses))

    return result
