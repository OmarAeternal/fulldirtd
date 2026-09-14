#!/usr/bin/env python3
"""mergeway — scan berurutan sepanjang lintasan drone, tiap ~10 m ke depan.

Perintah ketujuh, saudara `areascan`. Di sana drone diam dan berputar; di sini
drone terbang lurus dan berhenti tiap JARAK meter untuk satu scan. Persiapan
tiap scan sama persis dengan areascan (buang kembar, cari drone, ratakan ke
tanah sendiri) — lihat `areascan.siapkan_scan`.

Posisi tiap scan:

    tebakan   = posisi scan sebelumnya + JARAK ke arah --maju, yaw tetap
    cocokkan  = penanda di tanah melawan peta yang sudah terpasang
    diterima  hanya bila >= MIN_COCOK penanda cocok DAN jawabannya tunggal;
              selain itu scan dilanjutkan dari tebakan

Kenapa penerimaannya ketat. Pada scan_0193-0196 (15 September 2026):
0195 dan 0196 tidak punya satu penanda pun; 0194->0193 paling banter
2 penanda cocok, dan 161 kombinasi geser/yaw berbeda sama baiknya. Korelasi
citra benda tertarik ke geseran ~0 m karena artefak rig yang diam di kerangka
sensor (bayangan kaki drone, garis dekat sensor). Pencocokan yang "menang
tipis" di data seperti itu lebih buruk daripada tidak mencocokkan sama sekali:
tebakan 10 m setidaknya salahnya bisa diduga.

Tanah: tiap scan hanya diratakan ke bidang tanahnya sendiri (z = 0), tidak
dipaksa menyambung ke tetangganya. Beda tinggi pada petak bersama dilaporkan
supaya bisa dinilai.
"""

import math
import numpy as np

import areascan as ar
import pasak


# ═══════════════════════════════════════════════════════════════════════════════
# Tetapan
# ═══════════════════════════════════════════════════════════════════════════════

JARAK = 10.0            # jarak terbang antar scan (m)
MAJU_DEG = 0.0          # arah terbang di kerangka sensor-rata; 0 = +x, arah hadap kipas scan

MIN_COCOK = 3           # penanda cocok minimum supaya pencocokan dipercaya
COCOK_TOL = 0.40
MELESET_MAKS = 4.0      # geseran terjauh dari tebakan yang masih dicari (m)
YAW_JENDELA = 15.0      # yaw terjauh dari tebakan yang masih dicari (derajat)
YAW_LANGKAH = 1.0
BEDA_HIPOTESIS = 1.0    # dua jawaban lebih jauh dari ini (m) = jawaban BERBEDA


# ═══════════════════════════════════════════════════════════════════════════════
# Pencocokan
# ═══════════════════════════════════════════════════════════════════════════════

def arah(deg: float) -> np.ndarray:
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])


def cocokkan_lintasan(peta: np.ndarray, sumber: np.ndarray, yaw0: float, t0,
                      tol: float = COCOK_TOL, meleset: float = MELESET_MAKS,
                      jendela: float = YAW_JENDELA, min_cocok: int = MIN_COCOK):
    """Pose (yaw, t) yang membawa penanda `sumber` ke `peta`. → dict.

    Kunci `diterima` False berarti jangan dipakai; `alasan` menjelaskan.

    Hipotesis: tiap yaw dalam jendela x tiap pasangan (penanda peta, penanda
    sumber) memberi satu geseran. Tiap hipotesis dinilai dengan jumlah penanda
    sumber yang jatuh pada penanda peta. Jawaban tunggal berarti tidak ada
    hipotesis lain yang SAMA banyak cocoknya tapi posisinya lebih dari
    BEDA_HIPOTESIS dari jawaban terbaik.
    """
    t0 = np.asarray(t0, dtype=float)
    P, B = np.asarray(peta)[:, :2], np.asarray(sumber)[:, :2]
    hasil = {"diterima": False, "cocok": 0, "yaw": yaw0, "geser": t0}
    if len(P) == 0 or len(B) == 0:
        hasil["alasan"] = f"penanda peta {len(P)}, penanda scan {len(B)}"
        return hasil

    calon = []
    for yaw in np.arange(yaw0 - jendela, yaw0 + jendela + 1e-9, YAW_LANGKAH):
        Br = ar._putar2(B, yaw)
        t = (P[:, None, :] - Br[None, :, :]).reshape(-1, 2)
        t = t[np.linalg.norm(t - t0, axis=1) <= meleset]
        for tt in t:
            d = np.linalg.norm((Br + tt)[:, None, :] - P[None, :, :], axis=2)
            calon.append((int((d.min(axis=1) < tol).sum()), float(yaw), tt))
    if not calon:
        hasil["alasan"] = "tak ada pasangan penanda dalam jangkauan tebakan"
        return hasil

    n_max = max(c[0] for c in calon)
    terbaik = [c for c in calon if c[0] == n_max]
    # seri → yang paling dekat ke tebakan; tetap dan bisa diulang
    terbaik.sort(key=lambda c: (float(np.linalg.norm(c[2] - t0)), abs(c[1] - yaw0)))
    n, yaw, t = terbaik[0]
    berbeda = [c for c in terbaik if np.linalg.norm(c[2] - t) > BEDA_HIPOTESIS]
    hasil.update(cocok=n, kembar=len(berbeda))
    if n < min_cocok:
        hasil["alasan"] = f"penanda cocok {n} < {min_cocok}"
        return hasil
    if berbeda:
        hasil["alasan"] = (f"{len(berbeda)} jawaban lain sama baiknya "
                           f"({n} cocok) — tidak tunggal")
        return hasil

    # Perhalus lewat Kabsch pada pasangan yang sudah cocok.
    Bt = ar._putar2(B, yaw) + t
    d = np.linalg.norm(Bt[:, None, :] - P[None, :, :], axis=2)
    j = d.argmin(axis=1)
    ok = d[np.arange(len(B)), j] < tol
    T = pasak.kabsch2d(P[j[ok]], B[ok])
    yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    t = T[:2, 3]
    sisa = np.linalg.norm(ar._putar2(B[ok], yaw) + t - P[j[ok]], axis=1)
    hasil.update(diterima=True, yaw=float(yaw), geser=t, sisa=float(sisa.mean()))
    return hasil


# ═══════════════════════════════════════════════════════════════════════════════
# Rangkaian
# ═══════════════════════════════════════════════════════════════════════════════

def susun(awan_mentah: dict, jarak: float = JARAK, maju: float = MAJU_DEG,
          cocokkan: bool = True, log=print) -> dict:
    """Inti mergeway, tanpa berkas. `awan_mentah` = {nama: Nx3} URUT lintasan.

    Kerangka peta: titik asal di tanah di bawah drone scan pertama, +x searah
    hadap scan pertama, +z ke langit.
    """
    nama = list(awan_mentah)
    if len(nama) < 2:
        raise ValueError("butuh minimal dua scan")

    siap = {nm: ar.siapkan_scan(awan_mentah[nm], nm) for nm in nama}
    catatan = {nm: siap[nm]["catatan"] for nm in nama}
    penanda = {nm: siap[nm]["penanda"] for nm in nama}
    catatan[nama[0]].update(yaw=0.0, geser=np.zeros(2), asal="acuan")
    peta_penanda = penanda[nama[0]][:, :2].copy()

    for k, nm in enumerate(nama[1:], start=1):
        c = catatan[nm]
        lalu = catatan[nama[k - 1]]
        yaw0 = lalu["yaw"]
        t0 = lalu["geser"] + jarak * arah(maju + yaw0)
        if cocokkan:
            h = cocokkan_lintasan(peta_penanda, penanda[nm], yaw0, t0)
        else:
            h = {"diterima": False, "cocok": 0, "alasan": "--tanpa-cocok"}
        if h["diterima"]:
            yaw, t, asal = h["yaw"], h["geser"], "penanda"
            c.update(cocok=h["cocok"], sisa=h["sisa"])
        else:
            yaw, t, asal = yaw0, t0, "lanjutan"
            c["alasan_lanjut"] = h["alasan"]
        c.update(yaw=float(yaw), geser=np.asarray(t, dtype=float), asal=asal,
                 tebakan=t0, dari_tebakan=float(np.linalg.norm(np.asarray(t) - t0)),
                 langkah=float(np.linalg.norm(np.asarray(t) - lalu["geser"])))
        log(f"  {nm}: {asal:<8} posisi ({t[0]:+7.2f}, {t[1]:+7.2f}) m, yaw {yaw:+.1f}°"
            + ("" if h["diterima"] else f"   [{h['alasan']}]"))
        baru = ar._putar2(penanda[nm][:, :2], yaw) + t
        peta_penanda = np.vstack([peta_penanda, baru]) if len(peta_penanda) else baru

    peta, pose, drone = {}, {}, {}
    for nm in nama:
        yaw, t = catatan[nm]["yaw"], catatan[nm]["geser"]
        peta[nm] = ar.ke_peta(siap[nm]["rata"], yaw, t)
        pose[nm] = ar.geser(t[0], t[1]) @ ar.matriks_yaw(yaw) @ siap[nm]["dasar"]
        if siap[nm]["drone"] is not None:
            drone[nm] = siap[nm]["drone"]

    X, dz, I, J, _ = ar.pasangan_petak(peta)
    beda = ar.ringkas_beda(dz, np.linalg.norm(X, axis=1) if len(X) else None)
    return {"nama": nama, "peta": peta, "pose": pose, "drone": drone,
            "penanda": penanda, "catatan": catatan, "beda_tanah": beda,
            "jarak": jarak, "maju": maju}


# ═══════════════════════════════════════════════════════════════════════════════
# Berkas
# ═══════════════════════════════════════════════════════════════════════════════

MERGE_DIRNAME = "_mergeway"


def tulis_hasil(d, hasil: dict) -> None:
    from pathlib import Path
    o3d = pasak._o3d()
    d = Path(d)
    nama = hasil["nama"]

    gabung, warna = [], []
    for k, nm in enumerate(nama):
        p = hasil["peta"][nm]
        pasak._tulis_ply(d / f"{nm}.ply", p)
        gabung.append(p)
        warna.append(np.tile(np.array(pasak.PALET[k % len(pasak.PALET)]) / 255.0,
                             (len(p), 1)))

    # Drone di tiap posisi: jejak lintasannya.
    jejak = [pasak.terapkan(hasil["drone"][nm]["titik"], hasil["pose"][nm])
             for nm in nama if nm in hasil["drone"]]
    if jejak:
        jejak = np.vstack(jejak)
        pasak._tulis_ply(d / "drone.ply", jejak)
        gabung.append(jejak)
        warna.append(np.tile([0.1, 0.1, 0.1], (len(jejak), 1)))

    semua = np.vstack(gabung)
    pasak._tulis_ply(d / "merged.ply", semua)
    pc = pasak.to_o3d(semua)
    pc.colors = o3d.utility.Vector3dVector(np.vstack(warna))
    o3d.io.write_point_cloud(str(d / "merged_check.ply"), pc)
    (d / "laporan.txt").write_text(laporan(hasil) + "\n")


def laporan(hasil: dict) -> str:
    nama, cat = hasil["nama"], hasil["catatan"]
    b = [f"# mergeway — lintasan lurus, tebakan {hasil['jarak']:g} m ke arah "
         f"{hasil['maju']:+g}° (0 = +x sensor)", "#",
         "# asal  penanda  = posisi dari benda di tanah yang cocok",
         "#       lanjutan = pencocokan ditolak, posisi = scan sebelumnya + jarak",
         "#       Posisi 'lanjutan' adalah TEBAKAN, bukan hasil ukur.", ""]
    for nm in nama:
        c = cat[nm]
        b.append(nm)
        b.append(f"  titik       {c['titik_awal']:,} → unik {c['titik_unik']:,} "
                 f"→ tanpa drone {c['titik_bersih']:,}")
        b.append(f"  tanah       diratakan {c['miring']:.2f}°")
        if nm in hasil["drone"]:
            dr = hasil["drone"][nm]
            b.append(f"  drone       {dr['n']:,} titik, {dr['tinggi']:.2f} m di atas tanah")
        b.append(f"  penanda     {len(hasil['penanda'][nm])}")
        b.append(f"  posisi      ({c['geser'][0]:+.2f}, {c['geser'][1]:+.2f}) m, "
                 f"yaw {c['yaw']:+.2f}°   (dari {c['asal']})")
        if "langkah" in c:
            b.append(f"  langkah     {c['langkah']:.2f} m dari scan sebelumnya, "
                     f"{c['dari_tebakan']:.2f} m dari tebakan")
        if "cocok" in c:
            b.append(f"  cocok       {c['cocok']} penanda, sisa rata {c['sisa'] * 100:.1f} cm")
        if "alasan_lanjut" in c:
            b.append(f"  dilanjutkan karena: {c['alasan_lanjut']}")
        for w in c["peringatan"]:
            b.append(f"  [WARN] {w}")
        b.append("")
    b.append("# Beda tinggi tanah pada petak 50 cm yang dilihat >1 scan (tidak dipaksa sambung)")
    b.append(f"  {ar._fmt_beda(hasil['beda_tanah'])}")
    b.append("")
    b.append("# Matriks tiap scan (kerangka sensor asli → peta)")
    for nm in nama:
        b.append(nm)
        for r in hasil["pose"][nm]:
            b.append("    " + "".join(f"{v:11.6f}" for v in r))
    return "\n".join(b)


def jalankan(args) -> None:
    from pathlib import Path
    import clomcap, clomcaps, clomerge

    srcs = clomcaps.dedupe_inputs(args.files)
    if len(srcs) < 2:
        raise SystemExit("[ERROR] mergeway butuh minimal 2 berkas, urut sesuai lintasan.")

    print("=" * 66)
    print(f"  mergeway — {len(srcs)} scan, tiap {args.jarak:g} m ke arah {args.maju:+g}°")
    print("=" * 66)
    awan = {}
    for src in srcs:
        p = clomcaps.prepare_cloud(src, args)
        awan[Path(p).stem] = clomerge.read_cloud_xyz(p)

    hasil = susun(awan, jarak=args.jarak, maju=args.maju,
                  cocokkan=not args.tanpa_cocok)

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
        prog="mergeway",
        description="Gabungkan scan berurutan sepanjang lintasan lurus drone. "
                    "Dicocokkan lewat benda di tanah bila meyakinkan; bila "
                    "tidak, tiap scan dilanjutkan --jarak meter dari sebelumnya.")
    ap.add_argument("files", nargs="+", help=".mcap atau .ply, URUT sesuai lintasan")
    ap.add_argument("--jarak", type=float, default=JARAK,
                    help=f"jarak terbang antar scan, meter (baku {JARAK:g})")
    ap.add_argument("--maju", type=float, default=MAJU_DEG,
                    help="arah terbang dalam derajat di kerangka sensor, ccw dari "
                         "atas: 0 = +x (arah hadap kipas scan), 90 = +y, "
                         "180 = -x (baku 0)")
    ap.add_argument("--tanpa-cocok", action="store_true", dest="tanpa_cocok",
                    help="jangan cocokkan sama sekali, murni lanjutan jarak")
    ap.add_argument("-t", "--topic", default=None, help="topik PointCloud2")
    ap.add_argument("--force", action="store_true", help="paksa konversi ulang mcap")
    ap.set_defaults(png=False)
    return ap


def main() -> None:
    jalankan(build_parser().parse_args())


if __name__ == "__main__":
    main()
