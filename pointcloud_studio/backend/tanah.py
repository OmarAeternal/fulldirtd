"""Perataan tanah — cari bidang tanah sebuah scan, lalu tegakkan.

Dibuat untuk pengujian di lapangan terbuka yang rata: yang dilihat hanya
tanah, tidak ada tembok yang dijadikan acuan.

Tiga keputusan yang tidak kelihatan dari kodenya:

* **Dihitung per luas, bukan per titik.** Pada scan 0186 dan 0192 separuh
  lebih titik jatuh dalam 30 cm dari sensor. Dihitung per titik, gumpalan kecil
  itu mengalahkan tanah (terukur memberi bidang miring 36° yang salah). Maka
  titik dikelompokkan dulu per kubus 10 cm; tiap kubus satu suara.
* **RANSAC buatan sendiri dengan benih tetap, dihaluskan lewat SVD.**
  `segment_plane` milik Open3D diparalelkan dan tidak bisa diulang walau
  di-seed — hasil perataan yang berubah tiap kali membuka berkas yang sama
  tidak bisa dipercaya.
* **Putaran terkecil.** Normal tanah diputar ke +Z lewat sumbu yang tegak lurus
  keduanya, jadi arah hadap scan dilihat dari atas tidak ikut berubah. Tanah
  lalu digeser ke z = 0 supaya tinggi benda terbaca langsung.
"""
from __future__ import annotations

import numpy as np

# Sisi kubus pengelompokan. Cukup kasar untuk meredam kepadatan dekat sensor,
# cukup halus supaya tanah 1 m² masih bernilai ~100 suara.
VOXEL_LUAS = 0.10

# Batas jumlah kubus yang diperiksa RANSAC; lebih dari ini dicuplik acak
# (benih tetap). Scan lapangan sekarang ~28 ribu kubus.
MAKS_SAMPEL = 80_000

ITERASI = 1000
BENIH = 0

# Tanah berumput dan bising jarak jauh: 8 cm untuk menemukan, lalu 4 cm untuk
# menghaluskan.
AMBANG_CARI = 0.08
AMBANG_HALUS = 0.04

# Scan 0186 terukur miring ~36°. Di lapangan tanpa tembok, bidang yang lebih
# tegak dari ini hampir pasti bukan tanah.
MAKS_MIRING_DEG = 50.0

# Di bawah bagian kubus ini, "bidang terbesar" belum tentu tanah — lebih jujur
# tidak meratakan sama sekali.
MIN_FRAKSI = 0.12

MIN_TITIK = 100


def _kubus(xyz: np.ndarray, sisi: float) -> np.ndarray:
    """Satu titik (yang pertama muncul) per kubus bersisi `sisi`."""
    k = np.floor(xyz / sisi).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return xyz[np.sort(i)]


def _bidang_svd(P: np.ndarray):
    """→ (normal menghadap atas, d) dengan n·p + d = 0."""
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    n = vt[2]
    if n[2] < 0:
        n = -n
    return n, float(-n @ c)


def _ransac(P: np.ndarray, rng) -> tuple:
    """→ (normal, d) bidang dengan kubus terbanyak dalam AMBANG_CARI."""
    idx = rng.integers(0, len(P), (ITERASI, 3))
    a, b, c = P[idx[:, 0]], P[idx[:, 1]], P[idx[:, 2]]
    N = np.cross(b - a, c - a)
    s = np.linalg.norm(N, axis=1)
    ok = s > 1e-9
    N = N[ok] / s[ok, None]
    a = a[ok]
    N[N[:, 2] < 0] *= -1
    tegak = N[:, 2] >= np.cos(np.radians(MAKS_MIRING_DEG))
    N, a = N[tegak], a[tegak]
    if not len(N):
        return None
    D = -np.einsum("ij,ij->i", N, a)

    skor = np.empty(len(N), dtype=np.int64)
    for j in range(0, len(N), 64):          # 64 hipotesis x 80 ribu ≈ 41 MB
        h = P @ N[j:j + 64].T + D[j:j + 64]
        skor[j:j + 64] = (np.abs(h) < AMBANG_CARI).sum(axis=0)
    t = int(np.argmax(skor))                 # seri → indeks terkecil, tetap
    return N[t], float(D[t])


def matriks_rata(n: np.ndarray, d: float) -> np.ndarray:
    """4x4 yang memutar normal `n` ke +Z (putaran terkecil) dan tanah ke z = 0."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = float(np.linalg.norm(v))
    c = float(n @ z)
    T = np.eye(4)
    if s > 1e-12:
        K = np.array([[0.0, -v[2], v[1]],
                      [v[2], 0.0, -v[0]],
                      [-v[1], v[0], 0.0]])
        T[:3, :3] = np.eye(3) + K + K @ K * ((1.0 - c) / s ** 2)
    T[2, 3] = d          # sesudah diputar, tanah ada di z = -d
    return T


def cari_tanah(xyz: np.ndarray):
    """Nx3 → dict hasil perataan, atau None bila tanah tidak meyakinkan.

    Kunci: normal, d, miring_deg, fraksi (bagian kubus yang menempel di
    tanah), n_kubus, matriks (4x4, dipakai p' = R p + t).
    """
    xyz = np.asarray(xyz, dtype=np.float64)[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < MIN_TITIK:
        return None

    P = _kubus(xyz, VOXEL_LUAS)
    if len(P) < MIN_TITIK:
        return None
    rng = np.random.default_rng(BENIH)
    if len(P) > MAKS_SAMPEL:
        P = P[np.sort(rng.choice(len(P), MAKS_SAMPEL, replace=False))]

    awal = _ransac(P, rng)
    if awal is None:
        return None
    n, d = awal
    for ambang in (AMBANG_CARI, AMBANG_HALUS):
        inl = np.abs(P @ n + d) < ambang
        if inl.sum() < 3:
            return None
        n, d = _bidang_svd(P[inl])

    miring = float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))))
    fraksi = float((np.abs(P @ n + d) < AMBANG_HALUS).mean())
    if fraksi < MIN_FRAKSI or miring > MAKS_MIRING_DEG:
        return None

    return {
        "normal": [float(v) for v in n],
        "d": d,
        "miring_deg": miring,
        "fraksi": fraksi,
        "n_kubus": int(len(P)),
        "matriks": matriks_rata(n, d).tolist(),
    }
