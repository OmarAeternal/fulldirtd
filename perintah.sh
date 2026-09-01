#!/usr/bin/env bash
# perintah.sh — mendefinisikan perintah cloudcom & PointCloud Studio.
#
# Sumberkan sekali dari ~/.bashrc, lalu perintahnya tersedia di terminal mana pun:
#
#     echo "source \"$(pwd)/perintah.sh\"" >> ~/.bashrc
#     source ~/.bashrc
#
# Lokasi repo dideteksi otomatis dari posisi berkas ini, jadi kamu boleh
# meng-clone ke folder bernama apa saja, di mana saja.

_RISET_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_CC="$_RISET_ROOT/ros2_ws/cloudcom"
_PCS="$_RISET_ROOT/pointcloud_studio"

# Penolong: jalankan skrip cloudcom memakai python di venv-nya.
_cc_run() {
    local skrip="$1"; shift
    if [ ! -x "$_CC/.venv/bin/python" ]; then
        echo "[ERROR] venv cloudcom belum dibuat. Lihat SETUP.md langkah 2." >&2
        return 1
    fi
    "$_CC/.venv/bin/python" "$_CC/$skrip" "$@"
}

# ── CloudCompare ──────────────────────────────────────────────────────────
# Buka CloudCompare lepas dari terminal (tidak ikut mati saat terminal ditutup).
clocom() {
    setsid flatpak run org.cloudcompare.CloudCompare "$@" >/dev/null 2>&1 </dev/null &
    disown
}

# ── Konversi & tampilan ───────────────────────────────────────────────────
# MCAP/PLY/XYZ → CloudCompare + grid, satu perintah.
clomcap()  { _cc_run clomcap.py  "$@"; }

# Banyak MCAP/PLY/XYZ → satu CloudCompare + satu grid gabungan.
clomcaps() { _cc_run clomcaps.py "$@"; }

# ── Registrasi / penggabungan scan ────────────────────────────────────────
# Banyak scan dari posisi berbeda → registrasi otomatis → satu PLY tergabung.
clomerge()    { _cc_run clomerge.py    "$@"; }

# Versi outdoor: scan melingkar searah, berurutan menurut indeks nama berkas.
clomergeout() { _cc_run clomergeout.py "$@"; }

# Outdoor, peta tumbuh: tanah diratakan, sudut disapu, tiap scan berikutnya
# dicocokkan ke gabungan scan yang sudah terpasang.
outmerge()    { _cc_run outmerge.py    "$@"; }

# Sadar fitur: penilaian tajam pada titik menonjol, cuplikan berimbang per arah
# normal, pelepasan geseran di arah yang tidak terkunci. Untuk tempat bertembok
# polos, di mana hasil "menempel rapi tapi salah tempat".
clomerged()   { _cc_run clomerged.py   "$@"; }

# Registrasi berjangkar: pasak lantai & tembok sebagai penambat.
pasak()       { _cc_run pasak.py       "$@"; }

# ── PointCloud Studio ─────────────────────────────────────────────────────
# Point cloud (PLY/XYZ/MCAP) → PointCloud Studio di browser.
pcs() {
    if [ ! -x "$_PCS/.venv/bin/python" ]; then
        echo "[ERROR] venv pointcloud_studio belum dibuat. Lihat SETUP.md langkah 3." >&2
        return 1
    fi
    "$_PCS/.venv/bin/python" "$_PCS/pcs.py" "$@"
}
