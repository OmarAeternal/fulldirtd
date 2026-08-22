"""Parsing file point cloud (PLY biner/ascii, XYZ) menjadi array numpy.

Mengembalikan array Nx6 float32: kolom [x, y, z, r, g, b] dengan r,g,b di rentang 0..1.
Bila file tidak punya warna, warna diisi abu-abu netral.
"""
from __future__ import annotations
import io
import numpy as np


def _parse_ply(data: bytes) -> np.ndarray:
    """Parse PLY (ascii atau binary_little_endian) berisi vertex x,y,z (+ opsional rgb)."""
    # Pisahkan header (teks) dari body
    end_tag = b"end_header\n"
    idx = data.find(end_tag)
    if idx == -1:
        raise ValueError("PLY tidak valid: 'end_header' tidak ditemukan.")
    header = data[:idx].decode("ascii", errors="replace")
    body = data[idx + len(end_tag):]

    fmt = None
    n_vertex = 0
    props = []  # list of (name, type)
    in_vertex = False
    for line in header.splitlines():
        line = line.strip()
        if line.startswith("format"):
            fmt = line.split()[1]  # ascii | binary_little_endian | binary_big_endian
        elif line.startswith("element"):
            parts = line.split()
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                n_vertex = int(parts[2])
        elif line.startswith("property") and in_vertex:
            parts = line.split()
            # property <type> <name>   (list tidak didukung untuk vertex)
            props.append((parts[2], parts[1]))

    names = [p[0] for p in props]
    if not all(k in names for k in ("x", "y", "z")):
        raise ValueError("PLY tidak memuat properti x/y/z.")

    color_keys = ("red", "green", "blue")
    has_color = all(k in names for k in color_keys)

    np_type = {
        "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
        "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
        "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
        "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
    }

    if fmt == "ascii":
        arr = np.fromstring  # placeholder to avoid lint; use loadtxt below
        rows = np.loadtxt(io.BytesIO(body), dtype=np.float64, ndmin=2)
        col = {name: i for i, name in enumerate(names)}
        xyz = rows[:, [col["x"], col["y"], col["z"]]].astype(np.float32)
        if has_color:
            rgb = rows[:, [col["red"], col["green"], col["blue"]]].astype(np.float32) / 255.0
        else:
            rgb = None
    else:
        endian = "<" if "little" in fmt else ">"
        dtype = np.dtype([(n, endian + np_type[t]) for n, t in props])
        rows = np.frombuffer(body, dtype=dtype, count=n_vertex)
        xyz = np.stack([rows["x"], rows["y"], rows["z"]], axis=1).astype(np.float32)
        if has_color:
            rgb = np.stack([rows["red"], rows["green"], rows["blue"]], axis=1).astype(np.float32)
            # normalisasi bila uchar (0..255)
            if rgb.max() > 1.0:
                rgb = rgb / 255.0
        else:
            rgb = None

    return _combine(xyz, rgb)


def _parse_xyz(data: bytes) -> np.ndarray:
    """Parse file XYZ ascii (dipisah spasi): x y z [r g b]."""
    rows = np.loadtxt(io.BytesIO(data), dtype=np.float64, ndmin=2)
    if rows.shape[1] < 3:
        raise ValueError("File XYZ harus punya minimal 3 kolom (x y z).")
    xyz = rows[:, :3].astype(np.float32)
    rgb = None
    if rows.shape[1] >= 6:
        rgb = rows[:, 3:6].astype(np.float32)
        if rgb.max() > 1.0:
            rgb = rgb / 255.0
    return _combine(xyz, rgb)


def _combine(xyz: np.ndarray, rgb) -> np.ndarray:
    """Gabungkan xyz + rgb → Nx6, buang titik non-finite."""
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if rgb is None:
        rgb = np.full_like(xyz, 0.7)
    else:
        rgb = rgb[finite]
        rgb = np.clip(rgb, 0.0, 1.0)
    return np.concatenate([xyz, rgb], axis=1).astype(np.float32)


def parse(filename: str, data: bytes) -> np.ndarray:
    """Titik masuk: pilih parser berdasarkan ekstensi. Kembalikan Nx6 float32."""
    name = filename.lower()
    if name.endswith(".ply"):
        return _parse_ply(data)
    if name.endswith(".xyz") or name.endswith(".txt") or name.endswith(".asc"):
        return _parse_xyz(data)
    # coba tebak: kalau diawali 'ply' → PLY, selain itu XYZ
    if data[:3] == b"ply":
        return _parse_ply(data)
    return _parse_xyz(data)
