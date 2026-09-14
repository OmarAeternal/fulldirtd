#!/usr/bin/env python3
"""areascan — satu posisi, 2-4 arah hadap, satu peta.

Perintah keenam. Rig LiDAR dibawa drone yang MELAYANG DI TEMPAT dan hanya
berputar ~90 derajat di antara dua scan, jadi empat scan menutup lingkaran
penuh. Berbeda dari `clomerge`/`pasak` yang harus menemukan posisi tiap scan,
di sini posisinya sudah diketahui: semua scan berbagi satu titik tumpu.

Urutannya:

    1. buang titik kembar     rekaman 0187-0190 menyimpan tiap titik DUA kali
    2. cari drone             gumpalan melayang tinggi di atas tanah = titik tumpu
    3. ratakan tanah          roll, pitch, z   (pasak.kerangka_tanah)
    4. pose awal              yaw = k x 90 derajat searah --arah, putar di drone
    5. cocokkan tepi          benda kecil di tanah menyetel yaw (dan hanyutan)
    6. sambung tanah          petak tanah bersama harus setinggi yang sama;
                              menyerap kemiringan sisa DAN lengkung rig

Kenapa bukan ICP atau FPFH: titiknya sedikit. Separuh titik scan lapangan
menumpuk di badan drone dan di dalam 3 m dari sensor; di 10 m garis pindai
berjarak lebih dari satu meter. Korelasi citra benda terukur tergelincir ke
geseran 2 m (batas pencariannya) dengan yaw meleset 20-30 derajat. Yang
berhasil adalah pencocokan RASI benda: karena pusat putarnya sudah diketahui,
SATU benda yang sama sudah cukup menentukan yaw, jadi benda yang tertangkap
cuma belasan titik pun tetap berguna.

Diukur pada scan_0187-0190 (lapangan, 15 September 2026): drone di
(-0,06, -0,01) sensor pada semua scan, 3,0-3,2 m di atas tanah; langkah yaw
-77, -96, -85 derajat; hanyutan <= 0,4 m.
"""

import math
import numpy as np
from scipy import ndimage

import pasak


# ═══════════════════════════════════════════════════════════════════════════════
# Tetapan
# ═══════════════════════════════════════════════════════════════════════════════

LANGKAH_DEG = 90.0

# Arti arah, dilihat dari atas dengan +z ke langit (sesudah tanah diratakan):
# ccw = yaw bertambah, cw = yaw berkurang. Pada 0187-0190 benda di tanah
# bergeser azimutnya +80..+95 derajat per scan, artinya drone berputar cw.
ARAH_TANDA = {"ccw": +1.0, "cw": -1.0}
ARAH_BAKU = "cw"

# Drone: kubus 10 cm yang tersambung, melayang setidaknya segini di atas tanah.
# Orang dewasa setinggi 1,6-1,8 m menyambung ke tanah, jadi tidak lolos
# syarat "ada celah kosong di bawahnya".
DRONE_VOXEL = 0.10
DRONE_MIN_TINGGI = 1.0
DRONE_CELAH = 0.5          # celah kosong minimum di bawah gumpalan (m)
DRONE_MIN_TITIK = 50
DRONE_BUANG = 0.25         # buang titik sejauh ini di sekeliling kotak drone (m)

# Penanda (benda kecil di tanah). Tinggi diukur dari tanah SETEMPAT, bukan dari
# bidang tanah — lapangan 0187-0190 naik-turun +-0,5 m dalam 15 m, dan bidang
# tunggal membuat tanah jauh terbaca "benda" setinggi 0,3-0,6 m.
TANAH_SEL = 0.5
PENANDA_SEL = 0.10
PENANDA_TINGGI = (0.25, 2.5)
PENANDA_MIN_TITIK = 12      # sengaja rendah: titiknya memang sedikit
PENANDA_MIN_TEBAL = 0.20    # rentang tinggi; tonjolan tanah jauh cuma 5-13 cm
PENANDA_MAX_TAPAK = 1.2
PENANDA_JARAK = (0.8, 14.0)  # dari titik tumpu

COCOK_TOL = 0.40           # jarak dua penanda masih dianggap benda yang sama (m)
COCOK_JENDELA = 40.0       # yaw dicari sejauh ini dari tebakan (derajat)
MAKS_HANYUT = 1.0          # geseran drone terbesar yang diterima (m)
MIN_COCOK = 2              # di bawah ini pose awal dipakai apa adanya

SAMBUNG_SEL = 0.5          # petak pembanding tinggi tanah antar scan (m)
SAMBUNG_MIN_TITIK = 5      # titik tanah minimum per petak — sengaja rendah, titiknya jarang
SAMBUNG_MIN_PETAK = 50
SAMBUNG_HUBER = 0.04       # sisa di atas ini (m) diredam bobotnya: rumput, tepi benda
SAMBUNG_REDAM = 1.0        # tarikan lemah ke koreksi nol


# ═══════════════════════════════════════════════════════════════════════════════
# Dasar
# ═══════════════════════════════════════════════════════════════════════════════

def buang_kembar(xyz: np.ndarray) -> np.ndarray:
    """Titik yang persis sama disimpan sekali. Urutan tidak dipertahankan."""
    return np.unique(np.asarray(xyz, dtype=np.float64), axis=0)


def matriks_yaw(deg: float) -> np.ndarray:
    a = math.radians(deg)
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


def geser(x: float, y: float, z: float = 0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def beda_sudut(a: float, b: float) -> float:
    """a - b dibungkus ke (-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def tanah_setempat(xyz: np.ndarray, sel: float = TANAH_SEL) -> np.ndarray:
    """Tinggi tiap titik di atas titik terendah dalam 3x3 petak sekitarnya."""
    lo = xyz[:, :2].min(axis=0)
    ij = np.floor((xyz[:, :2] - lo) / sel).astype(np.int64)
    G = np.full(tuple(ij.max(axis=0) + 1), np.inf)
    np.minimum.at(G, (ij[:, 0], ij[:, 1]), xyz[:, 2])
    G = ndimage.minimum_filter(G, size=3, mode="constant", cval=np.inf)
    return xyz[:, 2] - G[ij[:, 0], ij[:, 1]]


def _komponen(ij: np.ndarray, dim: int):
    """Label komponen tersambung (tetangga penuh) untuk indeks kisi ij."""
    lo = ij.min(axis=0)
    k = ij - lo + 1
    grid = np.zeros(tuple(k.max(axis=0) + 2), dtype=bool)
    grid[tuple(k.T)] = True
    lab, n = ndimage.label(grid, structure=np.ones((3,) * dim))
    return lab[tuple(k.T)], n


# ═══════════════════════════════════════════════════════════════════════════════
# Drone — titik tumpu
# ═══════════════════════════════════════════════════════════════════════════════

def cari_drone(xyz: np.ndarray, T_tanah: np.ndarray):
    """→ dict {pusat (3, kerangka sensor), n, tinggi, topeng} atau None.

    Dicari di kerangka yang sudah diratakan: gumpalan tersambung terbesar yang
    dasarnya lebih dari DRONE_MIN_TINGGI di atas tanah dan tidak punya titik
    di bawahnya sampai DRONE_CELAH. Pusatnya dikembalikan di kerangka sensor
    supaya bisa dipakai sebelum maupun sesudah perataan.
    """
    g = pasak.terapkan(xyz, T_tanah)
    atas = g[:, 2] > DRONE_MIN_TINGGI
    if int(atas.sum()) < DRONE_MIN_TITIK:
        return None
    idx = np.flatnonzero(atas)
    lab, n = _komponen(np.floor(g[idx] / DRONE_VOXEL).astype(np.int64), 3)

    terbaik = None
    for l in range(1, n + 1):
        anggota = idx[lab == l]
        if len(anggota) < DRONE_MIN_TITIK:
            continue
        q = g[anggota]
        mn, mx = q.min(axis=0), q.max(axis=0)
        # celah: tidak boleh ada titik di bawah dasar gumpalan dalam tapaknya
        tapak = ((g[:, 0] >= mn[0]) & (g[:, 0] <= mx[0])
                 & (g[:, 1] >= mn[1]) & (g[:, 1] <= mx[1]))
        bawah = tapak & (g[:, 2] < mn[2]) & (g[:, 2] > mn[2] - DRONE_CELAH)
        if bawah.any():
            continue
        if terbaik is None or len(anggota) > len(terbaik):
            terbaik = anggota

    if terbaik is None:
        return None
    q = g[terbaik]
    mn, mx = q.min(axis=0) - DRONE_BUANG, q.max(axis=0) + DRONE_BUANG
    topeng = ((g >= mn) & (g <= mx)).all(axis=1)
    pusat_rata = np.median(q, axis=0)
    Ti = np.linalg.inv(T_tanah)
    return {
        "pusat": Ti[:3, :3] @ pusat_rata + Ti[:3, 3],
        "pusat_rata": pusat_rata,
        "n": int(len(terbaik)),
        "tinggi": float(pusat_rata[2]),
        "ukuran": (q.max(axis=0) - q.min(axis=0)),
        "topeng": topeng,
    }


# ═══════════════════════════════════════════════════════════════════════════════
# Penanda — benda kecil di tanah yang menyetel yaw
# ═══════════════════════════════════════════════════════════════════════════════

def daftar_penanda(g: np.ndarray, tumpu=(0.0, 0.0)) -> np.ndarray:
    """Awan yang sudah diratakan → (M, 4): x, y, jumlah titik, tebal.

    x, y relatif terhadap titik tumpu.
    """
    g = np.asarray(g, dtype=np.float64)
    if len(g) == 0:
        return np.zeros((0, 4))
    rel = g[:, :2] - np.asarray(tumpu)[:2]
    r = np.linalg.norm(rel, axis=1)
    h = tanah_setempat(g)
    m = ((h > PENANDA_TINGGI[0]) & (h < PENANDA_TINGGI[1])
         & (r > PENANDA_JARAK[0]) & (r < PENANDA_JARAK[1]))
    if int(m.sum()) < PENANDA_MIN_TITIK:
        return np.zeros((0, 4))
    p, hp = rel[m], h[m]
    # kisi 10 cm, tetangga 8 arah: setara jarak sambung ~0,15-0,28 m
    lab, n = _komponen(np.floor(p / PENANDA_SEL).astype(np.int64), 2)

    keluar = []
    for l in range(1, n + 1):
        s = lab == l
        if int(s.sum()) < PENANDA_MIN_TITIK:
            continue
        q, hq = p[s], hp[s]
        if float(np.ptp(q, axis=0).max()) > PENANDA_MAX_TAPAK:
            continue
        if float(np.ptp(hq)) < PENANDA_MIN_TEBAL:
            continue
        keluar.append((*q.mean(axis=0), float(s.sum()), float(np.ptp(hq))))
    return np.array(keluar) if keluar else np.zeros((0, 4))


def _putar2(xy: np.ndarray, deg: float) -> np.ndarray:
    a = math.radians(deg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    return xy @ R.T


def _skor(acuan: np.ndarray, sumber: np.ndarray, tol: float):
    """→ (jumlah sumber yang punya pasangan, rata sisa, indeks acuan tiap sumber)."""
    if len(acuan) == 0 or len(sumber) == 0:
        return 0, math.inf, np.full(len(sumber), -1)
    d = np.linalg.norm(sumber[:, None, :] - acuan[None, :, :], axis=2)
    j = d.argmin(axis=1)
    dm = d[np.arange(len(sumber)), j]
    ok = dm < tol
    j[~ok] = -1
    return int(ok.sum()), (float(dm[ok].mean()) if ok.any() else math.inf), j


def cocokkan(acuan: np.ndarray, sumber: np.ndarray, yaw_tebak: float,
             jendela: float = COCOK_JENDELA, tol: float = COCOK_TOL,
             maks_hanyut: float = MAKS_HANYUT):
    """Yaw (dan hanyutan) yang membawa penanda `sumber` ke `acuan`.

    Keduanya relatif terhadap titik tumpu masing-masing. → dict atau None.

    Hipotesis dibangkitkan dari SATU pasangan penanda: dengan pusat putar
    diketahui, penanda di jarak yang sama dari tumpu menentukan yaw sendirian.
    Tiap hipotesis dinilai dengan menghitung penanda lain yang ikut jatuh pada
    pasangannya. Hanyutan baru dilepas sesudahnya, dan hanya bila ada tiga
    pasangan atau lebih — dua pasangan tidak cukup untuk memisahkan hanyutan
    dari galat pusat penanda.
    """
    A, B = np.asarray(acuan)[:, :2], np.asarray(sumber)[:, :2]
    if len(A) == 0 or len(B) == 0:
        return None
    rA, rB = np.linalg.norm(A, axis=1), np.linalg.norm(B, axis=1)
    azA = np.degrees(np.arctan2(A[:, 1], A[:, 0]))
    azB = np.degrees(np.arctan2(B[:, 1], B[:, 0]))

    calon = []
    for i in range(len(A)):
        for k in range(len(B)):
            if abs(rA[i] - rB[k]) > tol:
                continue
            yaw = beda_sudut(azA[i], azB[k])
            if abs(beda_sudut(yaw, yaw_tebak)) > jendela:
                continue
            n, sisa, _ = _skor(A, _putar2(B, yaw), tol)
            # seri → yang lebih dekat ke tebakan; tetap dan bisa diulang
            calon.append((-n, sisa, abs(beda_sudut(yaw, yaw_tebak)), yaw))
    if not calon:
        return None
    calon.sort()
    n, _, _, yaw = calon[0]
    n = -n
    if n < 1:
        return None

    t = np.zeros(2)
    # Perhalus dengan pasangan yang sudah ditemukan. Putaran tetap di tumpu
    # kecuali cukup pasangan untuk menyimpulkan hanyutan.
    for _ in range(5):
        Bt = _putar2(B, yaw) + t
        _, _, j = _skor(A, Bt, tol)
        ok = j >= 0
        if ok.sum() < 1:
            break
        P, Q = A[j[ok]], B[ok]
        if ok.sum() >= 3:
            T = pasak.kabsch2d(P, Q)
            yaw_b = math.degrees(math.atan2(T[1, 0], T[0, 0]))
            t_b = T[:2, 3]
            if np.linalg.norm(t_b) > maks_hanyut:
                t_b = np.zeros(2)
                yaw_b = _yaw_tanpa_geser(P, Q)
        else:
            yaw_b, t_b = _yaw_tanpa_geser(P, Q), np.zeros(2)
        if abs(beda_sudut(yaw_b, yaw_tebak)) > jendela:
            break
        yaw, t = yaw_b, t_b

    n, sisa, j = _skor(A, _putar2(B, yaw) + t, tol)
    return {"yaw": float(yaw), "geser": t, "cocok": n, "sisa": sisa,
            "pasangan": [(int(a), int(b)) for b, a in enumerate(j) if a >= 0]}


def _yaw_tanpa_geser(P: np.ndarray, Q: np.ndarray) -> float:
    """Yaw kuadrat-terkecil yang memutar Q ke P di sekitar titik asal."""
    s = float(np.sum(Q[:, 0] * P[:, 1] - Q[:, 1] * P[:, 0]))
    c = float(np.sum(Q[:, 0] * P[:, 0] + Q[:, 1] * P[:, 1]))
    return math.degrees(math.atan2(s, c))


# ═══════════════════════════════════════════════════════════════════════════════
# Tanah bersama — sambungkan tanah antar scan
# ═══════════════════════════════════════════════════════════════════════════════

def ke_peta(xyz: np.ndarray, yaw: float, t) -> np.ndarray:
    """Kerangka rata-bertumpu → peta: putar di tumpu lalu geser hanyutan."""
    out = np.array(xyz, dtype=np.float64, copy=True)
    out[:, :2] = _putar2(out[:, :2], yaw) + np.asarray(t)[:2]
    return out


def ke_lokal(xy: np.ndarray, yaw: float, t) -> np.ndarray:
    return _putar2(np.asarray(xy)[:, :2] - np.asarray(t)[:2], -yaw)


def petak_tanah(g: np.ndarray, sel: float = SAMBUNG_SEL) -> dict:
    """{kunci petak: median xyz} dari titik tanah (< 15 cm di atas tanah setempat)."""
    g = g[tanah_setempat(g) < 0.15]
    ij = np.floor(g[:, :2] / sel).astype(np.int64)
    kunci = ij[:, 0] * 100003 + ij[:, 1]
    urut = np.argsort(kunci, kind="stable")
    kunci, g = kunci[urut], g[urut]
    u, awal, jumlah = np.unique(kunci, return_index=True, return_counts=True)
    return {int(k): np.median(g[a:a + c], axis=0)
            for k, a, c in zip(u, awal, jumlah) if c >= SAMBUNG_MIN_TITIK}


def pasangan_petak(peta: dict) -> tuple:
    """→ (xy tengah, dz = z_j - z_i, i, j, kunci) untuk tiap petak x pasangan scan."""
    nama = list(peta)
    petak = [petak_tanah(peta[nm]) for nm in nama]
    X, dz, I, J, kunci = [], [], [], [], []
    for i in range(len(nama)):
        for j in range(i + 1, len(nama)):
            for k in sorted(set(petak[i]) & set(petak[j])):
                p, q = petak[i][k], petak[j][k]
                X.append((p[:2] + q[:2]) / 2.0)
                dz.append(q[2] - p[2])
                I.append(i)
                J.append(j)
                kunci.append(k)
    return (np.array(X).reshape(-1, 2), np.array(dz), np.array(I, dtype=int),
            np.array(J, dtype=int), np.array(kunci, dtype=np.int64))


def ringkas_beda(dz: np.ndarray, r: np.ndarray = None) -> dict:
    if len(dz) == 0:
        return {"n_petak": 0, "median": math.nan, "p90": math.nan, "jauh": math.nan}
    a = np.abs(dz)
    jauh = a[r > 8.0] if r is not None else np.zeros(0)
    return {"n_petak": int(len(a)), "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90)),
            "jauh": float(np.median(jauh)) if len(jauh) else math.nan}


LENGKUNG = ("uu", "uv", "vv")


def koreksi_z(uv: np.ndarray, par: dict) -> np.ndarray:
    """Koreksi tinggi di kerangka rata-bertumpu SATU scan."""
    u, v = uv[:, 0], uv[:, 1]
    return (par["a"] * u + par["b"] * v + par["c"] + par["r"] * np.hypot(u, v)
            + par["uu"] * u * u + par["uv"] * u * v + par["vv"] * v * v)


def sambung_tanah(rata: dict, yaw: dict, geser_: dict):
    """Koreksi tinggi tiap scan supaya petak tanah bersama setinggi yang sama.

    → (par {nama: dict a,b,c,r,uu,uv,vv}, info) atau (None, info).

    Model koreksinya, di kerangka sensor-rata tiap scan (u, v dari tumpu):

        dz = a·u + b·v + c           per scan: sisa kemiringan dan tinggi
           + e·r                     per scan: kerucut, r = jarak dari tumpu
           + uu·u² + uv·u·v + vv·v²  SAMA untuk semua scan: lengkung rig

    Kenapa ada lengkung. Pada 0187-0190 tanah tiap scan naik sampai 0,6 m di
    tepi sepanjang SATU sumbu sensornya, dan selisih dua scan berbentuk pelana
    — tanda lengkung yang sama di kerangka sensor, ikut berputar 90 derajat
    bersama drone. Itu milik rig, bukan milik tanah, jadi satu lengkung untuk
    semua scan. Terukur pada petak yang TIDAK dipakai menyelesaikan (median |dz|):

        kaku                          11,8 cm   0-3 m 3,8   9-12 m 22,3
        + kemiringan per scan          9,7
        + lengkung bersama             6,5      0-3 m 5,8   9-12 m  8,7
        + kerucut per scan             5,6      0-3 m 4,6   9-12 m  5,6

    Kerucut setara galat sudut elevasi per scan; kemiringan rig memang beda
    10-19 derajat antar scan. Model yang lebih kaya lagi (kubik bersama,
    lengkung per scan) hanya 5,4-5,7 cm — sudah dekat derau rumput, dan
    lengkung per scan punya arah yang tak terkunci pasangan mana pun.
    Pembobotan per jarak diuji dan DITOLAK: ia hanya menukar galat dekat
    dengan galat jauh — tanda modelnya yang kurang, bukan bobotnya.

    Yang tidak bisa disimpulkan dari pasangan dikunci nol (gauge): kemiringan
    rata-rata di kerangka PETA — bukan rata-rata koefisien lokal, yang untuk
    empat scan berjarak 90 derajat memang selalu nol dan tak mengunci apa pun —
    serta mangkuk bundar uu + vv dan jumlah kerucut, yang tak berubah oleh
    putaran sehingga tak terlihat dari selisih antar scan.

    Tiap petak ke-5 disisihkan sebagai penguji. Bila di situ koreksinya tidak
    membaik, koreksi dibuang.
    """
    nama = list(rata)
    peta = {nm: ke_peta(rata[nm], yaw[nm], geser_[nm]) for nm in nama}
    X, dz, I, J, kunci = pasangan_petak(peta)
    info = {"sebelum": ringkas_beda(dz, np.linalg.norm(X, axis=1) if len(X) else None)}
    if len(dz) < SAMBUNG_MIN_PETAK:
        info["alasan"] = f"petak bersama {len(dz)} < {SAMBUNG_MIN_PETAK}"
        return None, info

    kolom = [f"{c}{k}" for k in range(len(nama)) for c in "abcr"] + list(LENGKUNG)
    ix = {c: n for n, c in enumerate(kolom)}
    M = np.zeros((len(dz), len(kolom)))
    for k, nm in enumerate(nama):
        w = (I == k).astype(float) - (J == k).astype(float)
        uv = ke_lokal(X, yaw[nm], geser_[nm])
        u, v = uv[:, 0], uv[:, 1]
        M[:, ix[f"a{k}"]] += w * u
        M[:, ix[f"b{k}"]] += w * v
        M[:, ix[f"c{k}"]] += w
        M[:, ix[f"r{k}"]] += w * np.hypot(u, v)
        M[:, ix["uu"]] += w * u * u
        M[:, ix["uv"]] += w * u * v
        M[:, ix["vv"]] += w * v * v

    gauge = np.zeros((5, len(kolom)))
    for k, nm in enumerate(nama):
        cy, sy = math.cos(math.radians(yaw[nm])), math.sin(math.radians(yaw[nm]))
        gauge[0, ix[f"a{k}"]], gauge[0, ix[f"b{k}"]] = cy, -sy
        gauge[1, ix[f"a{k}"]], gauge[1, ix[f"b{k}"]] = sy, cy
        gauge[2, ix[f"c{k}"]] = 1.0
        gauge[4, ix[f"r{k}"]] = 1.0
    gauge[3, ix["uu"]] = gauge[3, ix["vv"]] = 1.0
    gauge *= 1e3
    redam = SAMBUNG_REDAM * np.eye(len(kolom))

    uji = (kunci % 5) == 0
    latih = ~uji
    bobot = np.ones(int(latih.sum()))
    for _ in range(8):
        A = np.vstack([M[latih] * bobot[:, None], gauge, redam])
        b = np.concatenate([dz[latih] * bobot, np.zeros(len(gauge) + len(kolom))])
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        sisa = np.abs(M[latih] @ sol - dz[latih])
        bobot = 1.0 / np.sqrt(np.maximum(1.0, sisa / SAMBUNG_HUBER))

    r_uji = np.linalg.norm(X[uji], axis=1)
    info["uji_sebelum"] = ringkas_beda(dz[uji], r_uji)
    info["uji_sesudah"] = ringkas_beda(dz[uji] - M[uji] @ sol, r_uji)
    if not info["uji_sesudah"]["median"] < info["uji_sebelum"]["median"]:
        info["alasan"] = "petak penguji tidak membaik"
        return None, info

    par = {}
    for k, nm in enumerate(nama):
        par[nm] = {c: float(sol[ix[f"{c}{k}"]]) for c in "abcr"}
        par[nm].update({c: float(sol[ix[c]]) for c in LENGKUNG})
    return par, info


# ═══════════════════════════════════════════════════════════════════════════════
# Rangkaian
# ═══════════════════════════════════════════════════════════════════════════════

def siapkan_scan(xyz: np.ndarray, nama: str = "scan") -> dict:
    """Satu scan mentah → rata (kerangka tanah, titik asal di bawah drone).

    → dict: rata (Nx3 tanpa drone), dasar (4x4 sensor → rata), drone (dict
    atau None), penanda, catatan. Dipakai bersama oleh areascan dan mergeway.
    """
    xyz = np.asarray(xyz, dtype=np.float64)[:, :3]
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    n_awal = len(xyz)
    xyz = buang_kembar(xyz)
    c = {"titik_awal": n_awal, "titik_unik": len(xyz), "peringatan": []}

    # Tanah dicari tanpa sekitar sensor: badan drone ada di sana dan
    # jumlahnya bisa separuh scan.
    T = pasak.kerangka_tanah(xyz[np.linalg.norm(xyz, axis=1) > 1.0])
    if T is None:
        raise SystemExit(f"[ERROR] {nama}: tanah tidak ditemukan")
    d = cari_drone(xyz, T)
    drone = None
    if d is None:
        c["peringatan"].append("drone tidak ditemukan — titik tumpu = sensor")
        tumpu, bersih = np.zeros(3), xyz
    else:
        tumpu, bersih = d["pusat"], xyz[~d["topeng"]]
        drone = {k: v for k, v in d.items() if k != "topeng"}
        drone["titik"] = xyz[d["topeng"]]
    tumpu_rata = pasak.terapkan(tumpu[None, :], T)[0]
    dasar = geser(-tumpu_rata[0], -tumpu_rata[1]) @ T
    rata = pasak.terapkan(bersih, dasar)
    c.update(miring=pasak.derajat_miring(T), tumpu=tumpu_rata,
             titik_bersih=len(bersih))
    return {"rata": rata, "dasar": dasar, "drone": drone, "catatan": c,
            "penanda": daftar_penanda(rata)}


def susun(awan_mentah: dict, arah: str = ARAH_BAKU, langkah: float = LANGKAH_DEG,
          sambung: bool = True, log=print) -> dict:
    """Inti areascan, tanpa berkas. `awan_mentah` = {nama: Nx3} BERURUTAN.

    → dict berisi peta per scan (Nx3, sudah di kerangka peta), pose kaku
    (4x4, kerangka sensor → peta, TANPA koreksi tinggi), koreksi tinggi,
    drone, penanda, catatan per scan, dan ukuran kesambungan tanah.
    Kerangka peta: titik asal di tanah tepat di bawah drone scan pertama,
    +z ke langit, +x searah hadap scan pertama.
    """
    if arah not in ARAH_TANDA:
        raise ValueError(f"arah harus salah satu dari {sorted(ARAH_TANDA)}")
    nama = list(awan_mentah)
    if len(nama) < 2:
        raise ValueError("butuh minimal dua scan")
    tanda = ARAH_TANDA[arah]

    rata, dasar, drone, penanda, catatan = {}, {}, {}, {}, {}
    for nm in nama:
        s = siapkan_scan(awan_mentah[nm], nm)
        rata[nm], dasar[nm], catatan[nm], penanda[nm] = (
            s["rata"], s["dasar"], s["catatan"], s["penanda"])
        if s["drone"] is not None:
            drone[nm] = s["drone"]

    catatan[nama[0]].update(yaw=0.0, geser=np.zeros(2), asal="acuan")
    peta_penanda = penanda[nama[0]][:, :2].copy()

    for k, nm in enumerate(nama[1:], start=1):
        c = catatan[nm]
        yaw_lalu = catatan[nama[k - 1]]["yaw"]
        tebak = yaw_lalu + tanda * langkah
        h = cocokkan(peta_penanda, penanda[nm], tebak)
        n_h = 0 if h is None else h["cocok"]
        if n_h >= MIN_COCOK:
            yaw, t, asal = h["yaw"], h["geser"], "penanda"
            c.update(cocok=h["cocok"], sisa=h["sisa"])
        else:
            yaw, t, asal = tebak, np.zeros(2), "tebakan"
            c["peringatan"].append(
                f"penanda cocok {n_h} < {MIN_COCOK} — yaw dipakai apa adanya {tebak:+.1f}°")
        balik = cocokkan(peta_penanda, penanda[nm], yaw_lalu - tanda * langkah)
        if balik is not None and balik["cocok"] >= MIN_COCOK and balik["cocok"] > n_h:
            c["peringatan"].append(
                f"arah sebaliknya cocok lebih banyak ({balik['cocok']} lawan {n_h}) "
                f"— mungkin --arah terbalik")
        c.update(yaw=float(yaw), geser=np.asarray(t, dtype=float), asal=asal,
                 langkah=beda_sudut(yaw, yaw_lalu), koreksi=beda_sudut(yaw, tebak))
        baru = _putar2(penanda[nm][:, :2], yaw) + t
        peta_penanda = np.vstack([peta_penanda, baru]) if len(peta_penanda) else baru

    # Penutup lingkaran: scan terakhir melawan scan pertama saja. Tidak
    # dipakai untuk mengubah pose — hanya bukti apakah rantai yawnya menyimpang.
    if len(nama) >= 3:
        a, b = nama[0], nama[-1]
        tutup = cocokkan(penanda[a][:, :2], penanda[b], catatan[b]["yaw"], jendela=20.0)
        catatan[b]["tutup"] = None if tutup is None else {
            "cocok": tutup["cocok"],
            "selisih_yaw": beda_sudut(tutup["yaw"], catatan[b]["yaw"])}

    yaw = {nm: catatan[nm]["yaw"] for nm in nama}
    gsr = {nm: catatan[nm]["geser"] for nm in nama}
    pose = {nm: geser(gsr[nm][0], gsr[nm][1]) @ matriks_yaw(yaw[nm]) @ dasar[nm]
            for nm in nama}

    par, info = (sambung_tanah(rata, yaw, gsr) if sambung else (None, None))
    peta = {}
    for nm in nama:
        g = rata[nm].copy()
        if par is not None:
            g[:, 2] += koreksi_z(g, par[nm])
        peta[nm] = ke_peta(g, yaw[nm], gsr[nm])
    if info is not None:
        if par is None:
            log(f"  [WARN] tanah tidak disambung: {info['alasan']}")
        else:
            log(f"  tanah bersama (petak penguji): {_fmt_beda(info['uji_sebelum'])}"
                f"  →  {_fmt_beda(info['uji_sesudah'])}")

    return {"nama": nama, "peta": peta, "pose": pose, "koreksi": par,
            "sambung": info, "drone": drone, "penanda": penanda,
            "catatan": catatan, "arah": arah}


def _fmt_beda(b: dict) -> str:
    if not b["n_petak"]:
        return "tidak ada petak bersama"
    jauh = "" if math.isnan(b["jauh"]) else f", >8 m {b['jauh'] * 100:.1f} cm"
    return (f"median {b['median'] * 100:.1f} cm, p90 {b['p90'] * 100:.1f} cm{jauh} "
            f"({b['n_petak']:,} petak)")


# ═══════════════════════════════════════════════════════════════════════════════
# Berkas
# ═══════════════════════════════════════════════════════════════════════════════

MERGE_DIRNAME = "_areascan"


def tulis_hasil(d, hasil: dict) -> None:
    import json
    from pathlib import Path
    o3d = pasak._o3d()
    d = Path(d)
    nama, pose = hasil["nama"], hasil["pose"]

    gabung, warna = [], []
    for k, nm in enumerate(nama):
        p = hasil["peta"][nm]
        gabung.append(p)
        warna.append(np.tile(np.array(pasak.PALET[k % len(pasak.PALET)]) / 255.0,
                             (len(p), 1)))
        pasak._tulis_ply(d / f"{nm}.ply", p)

    # Drone cukup sekali, dari scan pertama yang menemukannya.
    for nm in nama:
        if nm in hasil["drone"]:
            dr = pasak.terapkan(hasil["drone"][nm]["titik"], pose[nm])
            pasak._tulis_ply(d / "drone.ply", dr)
            gabung.append(dr)
            warna.append(np.tile([0.1, 0.1, 0.1], (len(dr), 1)))
            break

    semua = np.vstack(gabung)
    pasak._tulis_ply(d / "merged.ply", semua)
    pc = pasak.to_o3d(semua)
    pc.colors = o3d.utility.Vector3dVector(np.vstack(warna))
    o3d.io.write_point_cloud(str(d / "merged_check.ply"), pc)

    tanda = []
    for k, nm in enumerate(nama):
        yaw, t = hasil["catatan"][nm]["yaw"], hasil["catatan"][nm]["geser"]
        for x, y, n, tebal in hasil["penanda"][nm]:
            px, py = _putar2(np.array([[x, y]]), yaw)[0] + t
            tanda.append({"scan": nm, "x": round(float(px), 3),
                          "y": round(float(py), 3), "titik": int(n),
                          "tebal": round(float(tebal), 2)})
    (d / "penanda.json").write_text(json.dumps(tanda, indent=1))

    (d / "laporan.txt").write_text(laporan(hasil) + "\n")


def laporan(hasil: dict) -> str:
    nama, cat = hasil["nama"], hasil["catatan"]
    b = ["# areascan — satu titik tumpu, arah " + hasil["arah"], "#",
         "# yaw      sudut hadap tiap scan di kerangka peta (ccw positif,",
         "#          dilihat dari atas)",
         "# langkah  yaw dikurangi yaw scan sebelumnya — idealnya +-90",
         "# koreksi  seberapa jauh penanda menggeser yaw dari tebakan",
         "# cocok    penanda scan ini yang menemukan pasangannya di peta",
         ""]
    for nm in nama:
        c = cat[nm]
        b.append(f"{nm}")
        b.append(f"  titik       {c['titik_awal']:,} → unik {c['titik_unik']:,} "
                 f"→ tanpa drone {c['titik_bersih']:,}")
        b.append(f"  tanah       diratakan {c['miring']:.2f}°")
        if nm in hasil["drone"]:
            dr = hasil["drone"][nm]
            b.append(f"  drone       {dr['n']:,} titik, {dr['tinggi']:.2f} m di atas "
                     f"tanah, di ({dr['pusat'][0]:+.3f}, {dr['pusat'][1]:+.3f}) sensor")
        b.append(f"  penanda     {len(hasil['penanda'][nm])}")
        b.append(f"  yaw         {c['yaw']:+8.2f}°   (dari {c['asal']})")
        if "langkah" in c:
            b.append(f"  langkah     {c['langkah']:+8.2f}°   koreksi {c['koreksi']:+.2f}°")
            b.append(f"  hanyut      ({c['geser'][0]:+.2f}, {c['geser'][1]:+.2f}) m")
        if "cocok" in c:
            b.append(f"  cocok       {c['cocok']}, sisa rata {c['sisa'] * 100:.1f} cm")
        if c.get("tutup") is not None:
            b.append(f"  penutup     melawan scan pertama langsung: {c['tutup']['cocok']} "
                     f"cocok, yaw berselisih {c['tutup']['selisih_yaw']:+.2f}°")
        for w in c["peringatan"]:
            b.append(f"  [WARN] {w}")
        b.append("")
    u = hasil["sambung"]
    b.append("# Beda tinggi tanah pada petak 50 cm yang dilihat >1 scan")
    if u is None:
        b.append("  tidak disambung (--tanpa-sambung)")
    else:
        b.append(f"  semua petak, sebelum   {_fmt_beda(u['sebelum'])}")
        if "uji_sebelum" in u:
            b.append(f"  petak penguji, sebelum {_fmt_beda(u['uji_sebelum'])}")
            b.append(f"  petak penguji, sesudah {_fmt_beda(u['uji_sesudah'])}")
            b.append("  (petak penguji = tiap petak ke-5, TIDAK dipakai menyelesaikan)")
        if hasil["koreksi"] is None:
            b.append(f"  [WARN] koreksi tidak dipakai: {u.get('alasan')}")
        else:
            q = next(iter(hasil["koreksi"].values()))
            b.append(f"  lengkung rig bersama  uu {q['uu']:+.5f}  uv {q['uv']:+.5f}  "
                     f"vv {q['vv']:+.5f}   (m per m²)")
            for nm, q in hasil["koreksi"].items():
                b.append(f"  {nm:<22} miring sisa ({math.degrees(math.atan(q['a'])):+.2f}°, "
                         f"{math.degrees(math.atan(q['b'])):+.2f}°)  tinggi {q['c'] * 100:+.1f} cm  "
                         f"kerucut {q['r'] * 1000:+.1f} mm/m")
    b.append("")
    b.append("# Matriks kaku tiap scan (kerangka sensor asli → peta).")
    b.append("# BELUM termasuk koreksi tinggi di atas — itu bukan transformasi kaku.")
    b.append("# Titik yang sudah terkoreksi ada di <scan>.ply dan merged.ply.")
    for nm in nama:
        b.append(nm)
        for r in hasil["pose"][nm]:
            b.append("    " + "".join(f"{v:11.6f}" for v in r))
    return "\n".join(b)


def jalankan(args) -> None:
    from pathlib import Path
    import clomcap, clomcaps, clomerge

    srcs = clomcaps.dedupe_inputs(args.files)
    if not 2 <= len(srcs) <= 4:
        raise SystemExit("[ERROR] areascan butuh 2-4 berkas, urut sesuai putaran.")

    print("=" * 66)
    print(f"  areascan — {len(srcs)} scan, arah {args.arah}, langkah {args.langkah:g}°")
    print("=" * 66)
    awan = {}
    for src in srcs:
        p = clomcaps.prepare_cloud(src, args)
        awan[Path(p).stem] = clomerge.read_cloud_xyz(p)

    hasil = susun(awan, arah=args.arah, langkah=args.langkah,
                  sambung=not args.tanpa_sambung)

    d = clomerge.next_merge_slot(Path(clomcap.OUT_ROOT) / MERGE_DIRNAME)
    d.mkdir(parents=True, exist_ok=True)
    tulis_hasil(d, hasil)
    print()
    print(laporan(hasil).split("# Matriks")[0].rstrip())
    print("\n" + "=" * 66)
    print(f"  Peta   : {d}/merged.ply")
    print(f"  Periksa: {d}/merged_check.ply   (tiap scan satu warna)")
    print(f"  Laporan: {d}/laporan.txt")
    print("=" * 66)


def build_parser():
    import argparse
    ap = argparse.ArgumentParser(
        prog="areascan",
        description="Gabungkan 2-4 scan dari SATU posisi yang tiap scannya "
                    "diputar ~90 derajat. Drone yang tertangkap di tiap scan "
                    "menjadi titik tumpu putaran.")
    ap.add_argument("files", nargs="+",
                    help=".mcap atau .ply, URUT sesuai putaran")
    ap.add_argument("--arah", choices=sorted(ARAH_TANDA), default=ARAH_BAKU,
                    help=f"arah putar antar scan dilihat dari atas "
                         f"(baku {ARAH_BAKU}; terukur cw pada 0187-0190)")
    ap.add_argument("--langkah", type=float, default=LANGKAH_DEG,
                    help="besar putaran tebakan antar scan, derajat (baku 90)")
    ap.add_argument("--tanpa-sambung", action="store_true", dest="tanpa_sambung",
                    help="lewati penyambungan tanah antar scan")
    ap.add_argument("-t", "--topic", default=None, help="topik PointCloud2")
    ap.add_argument("--force", action="store_true", help="paksa konversi ulang mcap")
    ap.set_defaults(png=False)
    return ap


def main() -> None:
    jalankan(build_parser().parse_args())


if __name__ == "__main__":
    main()
