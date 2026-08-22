"""Tes kesegaran: browser dan `pcs` harus selalu melihat kode terbaru.

Dua cara aplikasi ini pernah menyajikan barang basi tanpa memberi tahu siapa pun:

1. Aset statis dikirim hanya dengan ETag/Last-Modified, tanpa Cache-Control.
   Tanpa info kesegaran eksplisit, browser boleh memakai caching heuristik dan
   menyajikan JS lama tanpa bertanya ke server.
2. `pcs` memakai ulang server yang sudah jalan. Kalau backend berubah setelah
   server itu menyala, kode lama tetap di memori dan endpoint diam-diam
   berperilaku versi lama.
"""
import pathlib

import pytest
from fastapi.testclient import TestClient

import pcs
import server


@pytest.fixture
def client():
    return TestClient(server.app)


# ── Cache-Control ───────────────────────────────────────────────────────────

@pytest.mark.parametrize("path", ["/", "/static/app.js", "/static/index.html"])
def test_aset_frontend_wajib_divalidasi_ulang(client, path):
    r = client.get(path)
    assert r.status_code == 200
    assert "no-cache" in r.headers.get("cache-control", "")


def test_etag_tetap_dikirim_supaya_revalidasi_murah(client):
    """no-cache berarti 'tanya dulu', bukan 'unduh ulang'. ETag tetap perlu."""
    r = client.get("/static/app.js")
    assert r.headers.get("etag")


def test_revalidasi_membalas_304(client):
    r = client.get("/static/app.js")
    r2 = client.get("/static/app.js", headers={"If-None-Match": r.headers["etag"]})
    assert r2.status_code == 304


# ── /versi ──────────────────────────────────────────────────────────────────

def test_versi_melaporkan_pid_dan_umur_sumber(client):
    d = client.get("/versi").json()
    assert isinstance(d["pid"], int) and d["pid"] > 0
    assert isinstance(d["sumber_mtime"], float)


def test_versi_mencerminkan_berkas_backend_terbaru(client):
    d = client.get("/versi").json()
    terbaru = max(p.stat().st_mtime
                  for p in (pathlib.Path(server.__file__).parent).glob("*.py"))
    assert d["sumber_mtime"] == pytest.approx(terbaru)


# ── deteksi server basi di `pcs` ────────────────────────────────────────────

def test_server_basi_bila_sumber_lebih_baru():
    assert pcs.server_basi({"sumber_mtime": 100.0}, 200.0) is True


def test_server_segar_bila_sumber_sama():
    assert pcs.server_basi({"sumber_mtime": 200.0}, 200.0) is False


def test_server_tanpa_endpoint_versi_dianggap_basi():
    """Server yang menyala sebelum /versi ada tidak bisa melapor — anggap basi."""
    assert pcs.server_basi(None, 200.0) is True


def test_selisih_mtime_sangat_kecil_diabaikan():
    """Beda sub-detik datang dari presisi filesystem, bukan dari suntingan."""
    assert pcs.server_basi({"sumber_mtime": 200.0}, 200.0000001) is False


def test_mtime_sumber_membaca_seluruh_berkas_backend():
    t = pcs.mtime_sumber()
    assert t == pytest.approx(
        max(p.stat().st_mtime for p in (pcs.HERE / "backend").glob("*.py")))
