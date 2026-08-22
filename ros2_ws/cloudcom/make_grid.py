#!/usr/bin/env python3
"""make_grid.py — buat grid referensi (.ply) untuk dibuka bersama point cloud di CloudCompare.

Grid dibuat sebagai titik-titik rapat sepanjang garis grid pada bidang XY (horizontal),
agar tampil seperti garis grid di CloudCompare. Warna:
  • garis biasa  : abu-abu
  • garis mayor  : setiap `major` meter → abu-abu terang
  • sumbu X (y=0) : merah   ·  sumbu Y (x=0) : hijau

Pemakaian:
    python make_grid.py -o grid_1m.ply --spacing 1.0 --z 0 ref1.ply ref2.ply ...
      ref*.ply  : file point cloud untuk mengukur luas grid (opsional; boleh banyak)
    python make_grid.py -o grid.ply --xmin -5 --xmax 6 --ymin -12 --ymax 6 --spacing 1
"""
import argparse
import struct
import numpy as np


def ply_xyz_minmax(path):
    """Baca min/max XYZ dari PLY biner format (float x,y,z + uchar r,g,b)."""
    with open(path, "rb") as f:
        header = b""
        while b"end_header\n" not in header:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: header PLY tak lengkap")
            header += line
        n = 0
        for ln in header.decode("ascii", "replace").splitlines():
            if ln.startswith("element vertex"):
                n = int(ln.split()[2])
        body = f.read()
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("r", "u1"), ("g", "u1"), ("b", "u1")])
    a = np.frombuffer(body, dtype=dt, count=n)
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=1)
    return xyz.min(axis=0), xyz.max(axis=0)


def line_color(v, major):
    """Warna untuk satu garis grid pada koordinat v."""
    if abs(v) < 1e-6:
        return None  # ditangani khusus (sumbu)
    if abs(v % major) < 1e-6 or abs(v % major - major) < 1e-6:
        return (170, 170, 170)   # garis mayor
    return (90, 90, 90)          # garis minor


def build_grid(xmin, xmax, ymin, ymax, z, spacing, step, major):
    x0 = np.floor(xmin / spacing) * spacing
    x1 = np.ceil(xmax / spacing) * spacing
    y0 = np.floor(ymin / spacing) * spacing
    y1 = np.ceil(ymax / spacing) * spacing

    xs = np.round(np.arange(x0, x1 + 1e-6, spacing), 4)
    ys = np.round(np.arange(y0, y1 + 1e-6, spacing), 4)
    dense_x = np.arange(x0, x1 + 1e-6, step)
    dense_y = np.arange(y0, y1 + 1e-6, step)

    pts, cols = [], []

    def add(p, c):
        pts.append(p)
        cols.append(np.tile(np.array(c, np.uint8), (len(p), 1)))

    # Garis sejajar sumbu X (y tetap)
    for y in ys:
        p = np.column_stack([dense_x, np.full_like(dense_x, y), np.full_like(dense_x, z)])
        c = line_color(y, major)
        add(p, (220, 60, 60) if c is None else c)   # y=0 → sumbu X (merah)
    # Garis sejajar sumbu Y (x tetap)
    for x in xs:
        p = np.column_stack([np.full_like(dense_y, x), dense_y, np.full_like(dense_y, z)])
        c = line_color(x, major)
        add(p, (60, 200, 90) if c is None else c)   # x=0 → sumbu Y (hijau)

    return np.vstack(pts).astype(np.float32), np.vstack(cols)


def write_ply(path, xyz, rgb):
    n = len(xyz)
    header = ("ply\nformat binary_little_endian 1.0\ncomment grid referensi\n"
              f"element vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        buf = bytearray()
        for i in range(n):
            buf += struct.pack("<fffBBB", xyz[i, 0], xyz[i, 1], xyz[i, 2],
                               int(rgb[i, 0]), int(rgb[i, 1]), int(rgb[i, 2]))
        f.write(buf)


def main():
    ap = argparse.ArgumentParser(description="Buat grid referensi PLY untuk CloudCompare.")
    ap.add_argument("refs", nargs="*", help="PLY acuan untuk ukuran grid (opsional, boleh banyak)")
    ap.add_argument("-o", "--out", default="grid_1m.ply")
    ap.add_argument("--spacing", type=float, default=1.0, help="jarak antar garis (m), default 1.0")
    ap.add_argument("--step", type=float, default=0.03, help="kerapatan titik sepanjang garis (m)")
    ap.add_argument("--major", type=float, default=5.0, help="garis mayor tiap N meter")
    ap.add_argument("--z", type=float, default=0.0, help="ketinggian bidang grid (m)")
    ap.add_argument("--xmin", type=float); ap.add_argument("--xmax", type=float)
    ap.add_argument("--ymin", type=float); ap.add_argument("--ymax", type=float)
    ap.add_argument("--margin", type=float, default=1.0, help="margin di luar data (m)")
    args = ap.parse_args()

    if args.xmin is not None:
        xmin, xmax, ymin, ymax = args.xmin, args.xmax, args.ymin, args.ymax
    elif args.refs:
        mns, mxs = [], []
        for r in args.refs:
            mn, mx = ply_xyz_minmax(r); mns.append(mn); mxs.append(mx)
        mn = np.min(mns, axis=0); mx = np.max(mxs, axis=0)
        xmin, ymin = mn[0] - args.margin, mn[1] - args.margin
        xmax, ymax = mx[0] + args.margin, mx[1] + args.margin
    else:
        xmin, xmax, ymin, ymax = -10, 10, -10, 10

    xyz, rgb = build_grid(xmin, xmax, ymin, ymax, args.z, args.spacing, args.step, args.major)
    write_ply(args.out, xyz, rgb)
    print(f"✔ Grid dibuat: {args.out}")
    print(f"  bidang XY di Z={args.z}, spasi {args.spacing} m (mayor tiap {args.major} m)")
    print(f"  cakupan X [{xmin:.1f}..{xmax:.1f}]  Y [{ymin:.1f}..{ymax:.1f}]  → {len(xyz):,} titik")
    print(f"  sumbu X = merah, sumbu Y = hijau")


if __name__ == "__main__":
    main()
