"""Server FastAPI — PointCloud Studio.

Menyajikan frontend statis dan tiga endpoint komputasi (load, mesh, analyze).
Titik dipertukarkan sebagai biner Float32 little-endian agar efisien untuk
ratusan ribu titik.

Format biner titik (respons /load): header JSON kecil di query/headers tidak dipakai;
sebagai gantinya /load mengembalikan JSON {n, stats} + body biner terpisah lewat
multipart? -> Untuk kesederhanaan, /load mengembalikan JSON berisi statistik dan
titik dikirim sebagai Float32 base64. Untuk /mesh & /analyze, frontend meng-upload
titik sebagai body biner (Float32 [x,y,z] * N).
"""
from __future__ import annotations
import io
import json
import os
import sys
import pathlib
import numpy as np
from fastapi import FastAPI, UploadFile, File, Request, HTTPException
from fastapi.responses import Response, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import loader
import mesh as meshmod
import analysis as analysismod
import downsample

FRONTEND = pathlib.Path(__file__).parent.parent / "frontend"

# Ekstensi yang boleh dibaca lewat /open. Daftar putih, bukan daftar hitam:
# endpoint ini menerima path sembarang dari query, jadi yang tidak dikenal
# ditolak alih-alih dicoba.
EKSTENSI_CLOUD = (".ply", ".xyz", ".txt", ".asc")

app = FastAPI(title="PointCloud Studio")


@app.middleware("http")
async def _selalu_revalidasi(request: Request, call_next):
    """Paksa browser menanyakan kesegaran aset frontend di tiap muat.

    Starlette mengirim ETag dan Last-Modified tapi tidak Cache-Control. Tanpa
    info kesegaran eksplisit, browser boleh memakai caching heuristik — dan
    menyajikan app.js lama setelah kode diperbarui, tanpa tanda apa pun bahwa
    yang dilihat sudah basi.

    `no-cache` bukan berarti "jangan simpan": salinannya tetap dipakai, hanya
    saja harus divalidasi dulu. ETag membuat validasi itu murah (304, tanpa
    badan respons).
    """
    resp = await call_next(request)
    if request.url.path == "/" or request.url.path.startswith("/static/"):
        resp.headers["Cache-Control"] = "no-cache"
    return resp


# Dipakai `pcs` untuk tahu apakah server yang sedang jalan sudah memuat kode
# terbaru. Dihitung sekali saat impor — justru itu gunanya: angkanya mewakili
# kode yang benar-benar ada di memori proses ini, bukan yang ada di disk.
_MTIME_SUMBER = max(p.stat().st_mtime
                    for p in pathlib.Path(__file__).parent.glob("*.py"))


@app.get("/versi")
async def versi():
    """PID dan umur kode yang sedang dijalankan proses ini."""
    return JSONResponse({"pid": os.getpid(), "sumber_mtime": _MTIME_SUMBER})


def _stats(pts: np.ndarray) -> dict:
    xyz = pts[:, :3]
    mn = xyz.min(axis=0)
    mx = xyz.max(axis=0)
    return {
        "n": int(len(xyz)),
        "min": [float(v) for v in mn],
        "max": [float(v) for v in mx],
        "size": [float(v) for v in (mx - mn)],
    }


def _respons_titik(nama: str, data: bytes, voxel: float, full: bool) -> Response:
    """Bytes berkas → Response titik biner + header statistik.

    Jalur yang sama untuk /load (unggahan) dan /open (path di disk): parse,
    tolak yang kosong, optimasi kerapatan, lalu kirim Float32 murni dengan
    statistik di header supaya body tidak tercampur metadata.
    """
    try:
        pts = loader.parse(nama, data)
    except Exception as e:  # noqa: BLE001
        raise HTTPException(status_code=400, detail=f"Gagal membaca file: {e}")
    if len(pts) == 0:
        raise HTTPException(status_code=400, detail="File tidak memuat titik valid.")

    pts, info = downsample.optimize(pts, voxel=voxel, full=full)
    return Response(
        content=pts.astype("<f4").tobytes(),
        media_type="application/octet-stream",
        headers={"X-Stats": json.dumps(_stats(pts)),
                 "X-Downsample": json.dumps(info),
                 "Access-Control-Expose-Headers": "X-Stats, X-Downsample"})


def _param_optimasi(request: Request) -> tuple:
    """→ (voxel, full) dari query, dengan bawaan yang sama di kedua endpoint."""
    full = request.query_params.get("full", "0") in ("1", "true", "yes")
    return float(request.query_params.get("voxel", 0.01)), full


@app.post("/load")
async def load(request: Request, file: UploadFile = File(...)):
    """Terima file PLY/XYZ → parse → kembalikan titik biner + statistik.

    Respons: body = Float32 little-endian [x,y,z,r,g,b] * N.
    Statistik dikirim di header 'X-Stats' (JSON) agar body tetap biner murni.
    """
    voxel, full = _param_optimasi(request)
    return _respons_titik(file.filename or "upload", await file.read(), voxel, full)


@app.get("/open")
async def open_path(request: Request):
    """Muat berkas dari path di disk → titik biner, format sama dengan /load.

    Dipakai oleh perintah `pcs`, yang membuka browser ke `/?file=<path>`.
    Server hanya mendengar di 127.0.0.1, tapi path tetap divalidasi: yang
    ditolak semuanya dibalas 400 (bukan 404) supaya balasan ini tidak bisa
    dipakai menebak berkas mana yang ada di disk.
    """
    mentah = request.query_params.get("path", "")
    p = pathlib.Path(mentah)
    if (not mentah or not p.is_absolute()
            or p.suffix.lower() not in EKSTENSI_CLOUD or not p.is_file()):
        raise HTTPException(
            status_code=400,
            detail="Path tidak sah: harus absolut, menunjuk berkas yang ada, "
                   f"dan berekstensi {'/'.join(EKSTENSI_CLOUD)}.")

    voxel, full = _param_optimasi(request)
    return _respons_titik(p.name, p.read_bytes(), voxel, full)


async def _read_points(request: Request) -> np.ndarray:
    """Baca body biner Float32 [x,y,z]*N dari request."""
    raw = await request.body()
    arr = np.frombuffer(raw, dtype="<f4")
    if arr.size % 3 != 0:
        raise HTTPException(status_code=400, detail="Data titik tidak kelipatan 3.")
    return arr.reshape(-1, 3)


@app.post("/mesh")
async def make_mesh(request: Request):
    """Body: Float32 [x,y,z]*N. Query: res (derajat), jump (relatif).
    Respons JSON: {n_vertices, n_faces} + biner terpisah? -> kirim biner gabungan.
    Format biner respons: [int32 nV][int32 nF][float32 verts*3*nV][int32 faces*3*nF].
    """
    pts = await _read_points(request)
    res = float(request.query_params.get("res", 0.5))
    jump = float(request.query_params.get("jump", 0.25))
    verts, faces = meshmod.build_mesh(pts, angular_res_deg=res, depth_jump=jump)
    header = np.array([len(verts), len(faces)], dtype="<i4").tobytes()
    body = header + verts.astype("<f4").tobytes() + faces.astype("<i4").tobytes()
    return Response(content=body, media_type="application/octet-stream")


@app.post("/analyze")
async def analyze(request: Request):
    """Body: Float32 [x,y,z]*N. Respons JSON hasil analisis dimensi/RANSAC."""
    pts = await _read_points(request)
    dt = float(request.query_params.get("dt", 0.03))
    try:
        result = analysismod.analyze(pts, distance_threshold=dt)
    except Exception as e:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=f"Analisis gagal: {e}")
    return JSONResponse(result)


@app.get("/")
async def index():
    return FileResponse(FRONTEND / "index.html")


# Sajikan aset statis (app.js, vendor/)
app.mount("/static", StaticFiles(directory=str(FRONTEND)), name="static")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="warning")
