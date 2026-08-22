#!/usr/bin/env python3
"""pcs — buka point cloud di PointCloud Studio dengan satu perintah.

    pcs merged.ply
    pcs sweep_1.ply sweep_2.ply        # tiap berkas jadi satu layer
    pcs scan_0007_1sweep_0.mcap

Berkas diselesaikan jadi PLY/XYZ (MCAP dikonversi lewat proyek `ros2_ws/cloudcom`,
memakai cache bila hasil lama masih segar), server dipastikan hidup, lalu browser
dibuka ke halaman yang langsung memuat berkas-berkas itu.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
import webbrowser

HERE = pathlib.Path(__file__).resolve().parent

# Konversi MCAP tetap milik proyek sebelah: prepare_cloud() di sana sudah
# menangani deteksi topik, konversi, dan cache. Dipanggil lewat interpreternya
# sendiri supaya kedua venv tetap terpisah — PointCloud Studio tidak perlu
# pustaka MCAP, dan cloudcom tidak perlu FastAPI.
CLOUDCOM = pathlib.Path.home() / "riset td" / "ros2_ws" / "cloudcom"

EKSTENSI_CLOUD = (".ply", ".xyz")
EKSTENSI_MCAP = (".mcap", ".mcap.zstd")

VOXEL_BAWAAN = 0.01
PORT_BAWAAN = 8000

# Berapa lama menunggu server siap sebelum menyerah.
TUNGGU_SERVER = 40


def jenis_berkas(path) -> str:
    """→ 'mcap' bila perlu konversi, 'cloud' bila sudah siap dibuka."""
    low = str(path).lower()
    if low.endswith(EKSTENSI_MCAP):
        return "mcap"
    if low.endswith(EKSTENSI_CLOUD):
        return "cloud"
    raise SystemExit(
        f"[ERROR] Ekstensi tidak didukung: {pathlib.Path(path).name}\n"
        f"        Didukung: .ply, .xyz, .mcap, .mcap.zstd")


def bangun_url(port: int, berkas, voxel: float, full: bool) -> str:
    """URL halaman yang langsung memuat semua berkas di `berkas`.

    Tiap berkas jadi satu parameter `file` (halaman membacanya dengan
    getAll); `voxel`/`full` disebut sekali dan berlaku untuk semuanya.

    Path dikodekan dengan quote, bukan quote_plus: path di sini mengandung
    spasi ("riset td"), dan '+' akan terbaca sebagai spasi oleh sebagian
    pembaca query sementara '%20' selalu benar.
    """
    if not berkas:
        return f"http://127.0.0.1:{port}/"

    q = [("file", str(b)) for b in berkas]
    q.append(("voxel", voxel))
    if full:
        q.append(("full", "1"))
    kueri = urllib.parse.urlencode(q, quote_via=urllib.parse.quote)
    return f"http://127.0.0.1:{port}/?{kueri}"


# ═══════════════════════════════════════════════════════════════════════════════
# Menyiapkan berkas
# ═══════════════════════════════════════════════════════════════════════════════

def konversi_mcap(src: pathlib.Path, topic, force: bool) -> pathlib.Path:
    """MCAP → PLY lewat clomcaps.prepare_cloud di venv cloudcom."""
    py = CLOUDCOM / ".venv" / "bin" / "python"
    if not py.exists():
        raise SystemExit(
            f"[ERROR] Berkas MCAP perlu dikonversi, tapi venv cloudcom tidak ada:\n"
            f"        {py}\n"
            f"        Konversi dulu dengan `clomcap`, lalu buka hasil .ply-nya.")

    kode = (
        "import sys, argparse\n"
        f"sys.path.insert(0, {str(CLOUDCOM)!r})\n"
        "import clomcaps\n"
        "src, keluaran, force, topic = sys.argv[1:5]\n"
        "a = argparse.Namespace(force=force == '1', topic=topic or None, png=False)\n"
        "open(keluaran, 'w').write(clomcaps.prepare_cloud(src, a))\n"
    )

    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as tf:
        jalur_hasil = tf.name

    print(f"  Mengonversi {src.name} …")
    hasil = subprocess.run(
        [str(py), "-c", kode, str(src), jalur_hasil,
         "1" if force else "0", topic or ""])
    if hasil.returncode != 0:
        raise SystemExit("[ERROR] Konversi MCAP gagal.")

    return pathlib.Path(pathlib.Path(jalur_hasil).read_text().strip())


def siapkan_berkas(args):
    """Argumen berkas → daftar path PLY/XYZ absolut yang siap dibuka.

    → [] bila tidak ada berkas yang diminta; aplikasinya dibuka kosong dan
    pemakainya memilih sendiri lewat tombol "Buka" atau seret-lepas.

    Semua berkas divalidasi lebih dulu, sebelum konversi MCAP mana pun
    dijalankan: salah ketik di berkas ketiga tidak boleh membuang menit-menit
    mengonversi yang pertama.
    """
    sumber = []
    for mentah in args.file:
        src = pathlib.Path(mentah).expanduser()
        if not src.is_file():
            raise SystemExit(f"[ERROR] File tidak ditemukan: {src}")
        src = src.resolve()
        sumber.append((src, jenis_berkas(src)))   # menolak ekstensi asing di sini

    return [src if jenis == "cloud" else konversi_mcap(src, args.topic, args.force)
            for src, jenis in sumber]


# ═══════════════════════════════════════════════════════════════════════════════
# Server
# ═══════════════════════════════════════════════════════════════════════════════

def server_hidup(port: int) -> bool:
    """True bila PointCloud Studio sudah melayani di port itu."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1) as r:
            return r.status == 200
    except Exception:  # noqa: BLE001  — apa pun yang gagal berarti belum siap
        return False


def info_server(port: int):
    """→ dict dari /versi, atau None bila server tak bisa/tak mau melapor."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/versi", timeout=1) as r:
            return json.loads(r.read())
    except Exception:  # noqa: BLE001
        return None


def mtime_sumber() -> float:
    """Waktu ubah terbaru di antara berkas backend di disk."""
    return max(p.stat().st_mtime for p in (HERE / "backend").glob("*.py"))


def server_basi(info, mtime_disk: float) -> bool:
    """True bila server yang jalan memuat kode yang lebih tua dari yang di disk.

    Python memuat modul sekali saat proses mulai, jadi menyunting backend tidak
    berpengaruh pada server yang sudah menyala — endpoint-nya diam-diam terus
    berperilaku versi lama. Karena `pcs` sengaja memakai ulang server yang
    sudah jalan, tanpa pemeriksaan ini perubahan backend bisa tak terlihat
    berjam-jam.

    Server tanpa /versi berarti menyala sebelum endpoint itu ada — sudah pasti
    basi. Selisih sub-detik diabaikan: itu presisi filesystem, bukan suntingan.
    """
    if not info or "sumber_mtime" not in info:
        return True
    return mtime_disk - float(info["sumber_mtime"]) > 1.0


def port_terpakai(port: int) -> bool:
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def matikan_server(pid: int, port: int) -> None:
    """Hentikan server kita sendiri dan tunggu portnya benar-benar lepas."""
    try:
        os.kill(pid, signal.SIGTERM)
    except OSError:
        return
    batas = time.time() + 10
    while time.time() < batas:
        if not port_terpakai(port):
            return
        time.sleep(0.2)


def pastikan_server(port: int) -> None:
    """Pakai ulang server yang sudah jalan, atau nyalakan yang baru.

    Server yang memuat kode lebih tua dari yang di disk dinyalakan ulang:
    memakainya ulang berarti menjalankan versi lama tanpa tanda apa pun.
    """
    if server_hidup(port):
        info = info_server(port)
        if not server_basi(info, mtime_sumber()):
            print(f"  Memakai server yang sudah jalan di port {port}")
            return
        if info and info.get("pid"):
            print(f"  Server di port {port} memuat kode lama — dinyalakan ulang")
            matikan_server(int(info["pid"]), port)
        else:
            raise SystemExit(
                f"[ERROR] Server di port {port} memuat kode lama tapi tidak bisa\n"
                f"        dinyalakan ulang otomatis (versinya terlalu tua untuk\n"
                f"        melapor). Hentikan sendiri lalu jalankan `pcs` lagi.")

    if port_terpakai(port):
        raise SystemExit(
            f"[ERROR] Port {port} dipakai proses lain yang bukan PointCloud Studio.\n"
            f"        Pakai port lain: pcs --port {port + 1} …")

    log = pathlib.Path(tempfile.gettempdir()) / f"pointcloud_studio_{port}.log"
    print(f"  Menyalakan server di port {port} …")
    with open(log, "w") as f:
        subprocess.Popen(
            [sys.executable, "-m", "uvicorn", "backend.server:app",
             "--host", "127.0.0.1", "--port", str(port), "--log-level", "warning"],
            cwd=str(HERE), stdout=f, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL, start_new_session=True)

    batas = time.time() + TUNGGU_SERVER
    while time.time() < batas:
        if server_hidup(port):
            return
        time.sleep(0.3)

    raise SystemExit(f"[ERROR] Server tidak siap dalam {TUNGGU_SERVER} detik.\n"
                     f"        Log: {log}\n\n" + log.read_text())


# ═══════════════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="pcs",
        description="Buka point cloud (.ply/.xyz/.mcap) di PointCloud Studio.")
    ap.add_argument("file", nargs="*", default=[],
                    help="berkas .ply, .xyz, .mcap, atau .mcap.zstd; boleh lebih "
                         "dari satu (tiap berkas jadi satu layer), boleh juga "
                         "dikosongkan untuk membuka aplikasinya saja")
    ap.add_argument("--voxel", type=float, default=VOXEL_BAWAAN,
                    help=f"ukuran voxel dalam meter (default {VOXEL_BAWAAN}); "
                         "perkecil untuk detail lebih halus")
    ap.add_argument("--full", action="store_true",
                    help="kirim resolusi penuh, tanpa optimasi kerapatan")
    ap.add_argument("--port", type=int, default=PORT_BAWAAN,
                    help=f"port server (default {PORT_BAWAAN})")
    ap.add_argument("--force", action="store_true",
                    help="konversi ulang MCAP walau hasil lama masih segar")
    ap.add_argument("-t", "--topic", default=None,
                    help="paksa topik MCAP tertentu (mis. /map_3d)")
    return ap


def run(args) -> None:
    berkas = siapkan_berkas(args)
    pastikan_server(args.port)

    url = bangun_url(args.port, berkas, args.voxel, args.full)
    for b in berkas:
        print(f"  {b}")
    if not berkas:
        print(f"  {url}")
    if not webbrowser.open(url):
        print(f"  Browser tidak bisa dibuka otomatis. Buka sendiri:\n  {url}")


def main() -> None:
    run(build_parser().parse_args())


if __name__ == "__main__":
    main()
