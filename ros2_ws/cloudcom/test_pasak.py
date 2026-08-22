"""Tests untuk pasak.py — registrasi berjangkar.

Jalankan dengan:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest test_pasak.py -v

Env var itu perlu karena /opt/ros/jazzy ada di PYTHONPATH: pytest meng-autoload
plugin `launch` milik ROS, yang gagal impor dengan ModuleNotFoundError: yaml
sebelum tes sempat dikumpulkan.
"""

import numpy as np
import pytest

import pasak as pk


# ═══════════════════════════════════════════════════════════════════════════════
# Kabsch 2-D — yaw, x, y dari pasangan jangkar
# ═══════════════════════════════════════════════════════════════════════════════

def _putar2d(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def test_kabsch2d_menemukan_geseran_murni():
    """Dua jangkar yang digeser 1,2 m: geserannya harus kembali persis."""
    P = np.array([[0.0, 0.0, 1.0], [3.0, 0.5, 1.2]])
    Q = P + np.array([1.2, 0.0, 0.0])

    T = pk.kabsch2d(P, Q)

    assert np.allclose(T[:3, 3][:2], [-1.2, 0.0], atol=1e-9)
    assert np.allclose(T[:2, :2], np.eye(2), atol=1e-9)


def test_kabsch2d_menemukan_putaran():
    """Yaw 8° yang disuntikkan harus kembali sebagai −8° pada matriksnya."""
    P = np.array([[1.0, 0.0, 1.0], [4.0, 1.0, 1.2]])
    Q = np.asarray([(_putar2d(8.0) @ np.append(p, 1.0))[:3] for p in P])

    T = pk.kabsch2d(P, Q)

    assert pk.yaw_derajat(T) == pytest.approx(-8.0, abs=1e-6)
    assert np.allclose((T @ np.append(Q[0], 1.0))[:3], P[0], atol=1e-9)


def test_kabsch2d_tidak_memiringkan():
    """Jangkar boleh beda tinggi; jawabannya tetap putaran tegak saja."""
    P = np.array([[0.0, 0.0, 0.5], [2.0, 0.0, 2.4]])
    Q = np.array([[0.0, 0.0, 2.9], [0.0, 2.0, 0.1]])

    T = pk.kabsch2d(P, Q)

    assert np.allclose(T[2, :3], [0.0, 0.0, 1.0], atol=1e-12)
    assert np.allclose(T[:2, 2], [0.0, 0.0], atol=1e-12)


def test_kabsch2d_menolak_kurang_dari_dua_jangkar():
    with pytest.raises(ValueError):
        pk.kabsch2d(np.zeros((1, 3)), np.zeros((1, 3)))


# ═══════════════════════════════════════════════════════════════════════════════
# Sisa jangkar — pemeriksaan salah tunjuk
# ═══════════════════════════════════════════════════════════════════════════════

def test_sisa_dua_jangkar_nol_saat_benar():
    """Dua jangkar yang memang sepadan: selisih jaraknya nol."""
    P = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0]])
    Q = np.array([[5.0, 2.0, 1.0], [5.0, 5.0, 1.0]])      # sama-sama 3 m

    assert pk.sisa_jangkar(P, Q) == pytest.approx(0.0, abs=1e-9)


def test_sisa_dua_jangkar_menangkap_salah_tunjuk():
    """Jangkar kedua salah orang: jaraknya tidak cocok, dan itu ketahuan.

    Ini satu-satunya sisa yang tersedia pada dua jangkar — 4 batasan untuk 3
    anu — dan ia murah sekali dihitung.
    """
    P = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0]])      # jarak 3,0 m
    Q = np.array([[5.0, 2.0, 1.0], [5.0, 6.2, 1.0]])      # jarak 4,2 m

    assert pk.sisa_jangkar(P, Q) == pytest.approx(1.2, abs=1e-9)


def test_sisa_tiga_jangkar_menunjuk_yang_menyimpang():
    """Dengan ≥3 jangkar, yang salah harus bisa disebut namanya."""
    P = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0], [0.0, 4.0, 1.0]])
    Q = P.copy()
    Q[2] += [0.0, 0.9, 0.0]                                # jangkar #2 meleset

    sisa, per_jangkar = pk.sisa_jangkar(P, Q, rinci=True)

    assert int(np.argmax(per_jangkar)) == 2
    assert sisa > 0.2


# ═══════════════════════════════════════════════════════════════════════════════
# Helper — selasar sintetis yang mengidap penyakitnya
# ═══════════════════════════════════════════════════════════════════════════════

def selasar(seed=0, rig=True, tiang=True, tembok_dekat=False,
            tembok_lipat=1, gundukan=False) -> np.ndarray:
    """Tanah + tembok panjang polos + tiga tiang + rig di titik asal.

    Tiruan selasar FILKOM: temboknya mendominasi jumlah titik dan sama saja
    digeser menyusuri dirinya sendiri. Yang membedakan satu posisi dari posisi
    lain hanya tiangnya — dan tiangnya kalah banyak, persis perbandingan yang
    membuat algoritma lama tergelincir.

    `tembok_lipat` melipatgandakan titik tembok tanpa menambah tiang: dipakai
    untuk membuktikan perapian tidak menyerah pada mayoritas.
    """
    rng = np.random.default_rng(seed)
    bagian = []

    bagian.append(np.column_stack([rng.uniform(-8, 8, 14000),
                                   rng.uniform(-6, 2, 14000),
                                   rng.normal(0, 0.004, 14000)]))
    n_t = 16000 * int(tembok_lipat)
    bagian.append(np.column_stack([rng.uniform(-7, 7, n_t),
                                   1.20 + rng.normal(0, 0.004, n_t),
                                   rng.uniform(0.05, 3.2, n_t)]))

    if tiang:
        # jarak sengaja tidak seragam — kebalikan dari deret yang bisa
        # menghasilkan jawaban geser palsu-tapi-cocok
        for x0, lebar in ((-2.6, 0.28), (0.4, 0.22), (3.1, 0.34)):
            for dy in (0.0, -0.30):                      # dua muka tiang
                n = 1400
                bagian.append(np.column_stack([
                    rng.uniform(x0, x0 + lebar, n),
                    1.20 + dy + rng.normal(0, 0.004, n),
                    rng.uniform(0.10, 1.95, n)]))
            for xs in (x0, x0 + lebar):                  # sisi tiang
                n = 900
                bagian.append(np.column_stack([
                    xs + rng.normal(0, 0.004, n),
                    rng.uniform(0.90, 1.20, n),
                    rng.uniform(0.10, 1.95, n)]))

    if gundukan:
        # permukaan melengkung 2,6 m — terlalu bengkok untuk dikupas sebagai
        # bidang, jadi ia lolos sebagai "ciri" dan menjadi gugus raksasa.
        # Persis yang terjadi pada kanopi selasar di data asli.
        n = 5000
        t = rng.uniform(0, 1, n)
        bagian.append(np.column_stack([
            -6.0 + 2.6 * t,
            -3.0 + 0.9 * np.sin(np.pi * t) + rng.normal(0, 0.004, n),
            rng.uniform(0.2, 2.2, n)]))

    if tembok_dekat:
        # tembok di x = +0,49 — DI DALAM radius buang-rig, dan harus selamat
        n = 6000
        bagian.append(np.column_stack([0.49 + rng.normal(0, 0.004, n),
                                       rng.uniform(-4, 1.1, n),
                                       rng.uniform(0.05, 3.0, n)]))
    if rig:
        n = 400
        bagian.append(np.column_stack([rng.uniform(-0.30, 0.00, n),
                                       rng.uniform(-0.20, 0.20, n),
                                       rng.uniform(0.60, 1.50, n)]))
    return np.vstack(bagian)


def _dalam_silinder(xyz, r):
    return np.hypot(xyz[:, 0], xyz[:, 1]) <= r


# ═══════════════════════════════════════════════════════════════════════════════
# buang_rig — tripod dan operator ikut ter-scan dan bergerak bersama sensor
# ═══════════════════════════════════════════════════════════════════════════════

def test_buang_rig_membuang_gugus_di_asal():
    """Gugus di titik asal harus hilang; ia selalu cocok sempurna dan menipu ICP."""
    pk.seed(7)
    xyz = selasar(seed=1, rig=True, tembok_dekat=False)
    atlas = pk.atlas_bidang(xyz)

    bersih, dibuang = pk.buang_rig(xyz, atlas)

    tersisa = bersih[_dalam_silinder(bersih, pk.EGO_RADIUS)]
    tinggi = tersisa[tersisa[:, 2] > 0.30]
    assert len(tinggi) == 0, f"masih ada {len(tinggi)} titik rig tersisa"
    assert dibuang >= 350


def test_buang_rig_tidak_memakan_tembok_dekat():
    """Tembok di x=0,49 ada di dalam radius, tapi ia latar — harus selamat.

    Ini bukan kasus karangan: scan_0081 punya bidang tembok tepat di situ.
    """
    pk.seed(7)
    xyz = selasar(seed=2, rig=True, tembok_dekat=True)
    atlas = pk.atlas_bidang(xyz)

    def tembok_dekat(a):
        m = (np.abs(a[:, 0] - 0.49) < 0.05) & (a[:, 2] > 0.30)
        return int(m.sum())

    sebelum = tembok_dekat(xyz)
    bersih, _ = pk.buang_rig(xyz, atlas)

    assert sebelum > 4000
    assert tembok_dekat(bersih) > 0.95 * sebelum


def test_buang_rig_melaporkan_jumlah_yang_benar():
    pk.seed(7)
    xyz = selasar(seed=3)
    atlas = pk.atlas_bidang(xyz)

    bersih, dibuang = pk.buang_rig(xyz, atlas)

    assert dibuang == len(xyz) - len(bersih)


# ═══════════════════════════════════════════════════════════════════════════════
# kerangka_tanah — dan kestabilannya terhadap --range
# ═══════════════════════════════════════════════════════════════════════════════

def test_kerangka_tanah_menegakkan_scan_miring():
    pk.seed(7)
    xyz = selasar(seed=4, rig=False)
    miring = pk.putar_x(np.radians(9.0))
    condong = pk.terapkan(xyz, miring)

    T = pk.kerangka_tanah(condong)
    tegak = pk.terapkan(condong, T)

    tanah = tegak[tegak[:, 2] < 0.10]
    assert abs(float(np.median(tanah[:, 2]))) < 0.02
    assert pk.derajat_miring(T) == pytest.approx(9.0, abs=0.5)


def test_kerangka_tanah_tidak_bergantung_pada_jangkauan():
    """Jawabannya harus sama entah masukannya dipotong 6 m atau tidak.

    Sekarang tidak begitu: scan_0081 diratakan 7,35° pada --range 6 tapi 9,38°
    pada --range 15. Selisih 2° berarti 17 cm meleset di jarak 5 m, dan itu
    bocor ke seluruh langkah sesudahnya.
    """
    xyz = selasar(seed=5, rig=False)

    pk.seed(11); T_penuh = pk.kerangka_tanah(xyz)
    pk.seed(11); T_15 = pk.kerangka_tanah(pk.potong(xyz, 15.0))
    pk.seed(11); T_6 = pk.kerangka_tanah(pk.potong(xyz, 6.0))

    assert np.allclose(T_penuh, T_15, atol=1e-12)
    assert np.allclose(T_penuh, T_6, atol=1e-12)


# ═══════════════════════════════════════════════════════════════════════════════
# daftar_benda
# ═══════════════════════════════════════════════════════════════════════════════

def test_daftar_benda_menemukan_ketiga_tiang():
    pk.seed(7)
    xyz = selasar(seed=6, rig=True)
    T = pk.kerangka_tanah(xyz)
    xyz = pk.terapkan(xyz, T)

    daftar = pk.daftar_benda(xyz)

    x_tiang = sorted(b.pusat[0] for b in daftar)
    assert len(daftar) >= 3
    for diharap in (-2.46, 0.51, 3.27):
        assert min(abs(x - diharap) for x in x_tiang) < 0.30, \
            f"tiang di x≈{diharap} tidak ditemukan; yang ada {x_tiang}"


def test_daftar_benda_membuang_gugus_raksasa():
    """Permukaan melengkung lolos pengupasan jadi gugus 2,6 m; ia bukan benda.

    Di data asli inilah kanopi dan tembok bengkok: titik menonjol mencapai
    34-42% dari seluruh titik, jauh lebih banyak daripada ciri yang sebenarnya.
    """
    pk.seed(7)
    xyz = selasar(seed=7, rig=True, gundukan=True)
    xyz = pk.terapkan(xyz, pk.kerangka_tanah(xyz))

    daftar = pk.daftar_benda(xyz)

    for b in daftar:
        assert max(b.ukuran[0], b.ukuran[1]) <= pk.BENDA_MAX_TAPAK
        assert b.ukuran[2] <= pk.BENDA_MAX_TINGGI
    # dan gundukannya memang tidak ikut
    assert all(b.pusat[0] > -5.0 or b.ukuran[0] < 1.0 for b in daftar)


def test_daftar_benda_tidak_memasukkan_rig():
    """Rig sudah dibuang sebelum pencarian benda; ia tak boleh muncul di daftar."""
    pk.seed(7)
    xyz = selasar(seed=8, rig=True)
    xyz = pk.terapkan(xyz, pk.kerangka_tanah(xyz))

    daftar = pk.daftar_benda(xyz)

    for b in daftar:
        assert np.hypot(b.pusat[0], b.pusat[1]) > pk.EGO_RADIUS, \
            f"benda di {b.pusat} masih di dalam radius rig"


# ═══════════════════════════════════════════════════════════════════════════════
# Helper — dua scan atas geometri yang sama, dengan pose yang diketahui
# ═══════════════════════════════════════════════════════════════════════════════

def _geser(dx, dy=0.0, yaw=0.0):
    T = _putar2d(yaw)
    T[0, 3], T[1, 3] = dx, dy
    return T


def _dua_scan(T_benar, seed_a=20, seed_s=21, **kw):
    """Acuan dan sumber: geometri identik, CUPLIKAN berbeda, pose diketahui.

    Cuplikannya sengaja dibedakan. Kalau titiknya sama persis, ICP menang tanpa
    membuktikan apa pun.
    """
    acuan = selasar(seed=seed_a, **kw)
    sumber = pk.terapkan(selasar(seed=seed_s, **kw), T_benar)
    return sumber, acuan


def _jodohkan(benda_s, benda_a, T_benar, batas=0.35):
    """Peran manusia, dimainkan oleh kebenaran yang kita tahu.

    Membawa tiap benda sumber ke kerangka acuan lewat pose yang benar, lalu
    memasangkannya dengan benda acuan terdekat. Persis apa yang mata manusia
    lakukan di pcs, tanpa matanya.
    """
    inv = np.linalg.inv(T_benar)
    pasangan = []
    for i, bs in enumerate(benda_s):
        p = (inv @ np.append(bs.pusat, 1.0))[:3]
        j = int(np.argmin([np.linalg.norm(p[:2] - ba.pusat[:2]) for ba in benda_a]))
        if np.linalg.norm(p[:2] - benda_a[j].pusat[:2]) < batas:
            pasangan.append((i, j))
    return pasangan


# ═══════════════════════════════════════════════════════════════════════════════
# pasang — jangkar menyelesaikan, bukan mencari
# ═══════════════════════════════════════════════════════════════════════════════

def test_pasang_mengembalikan_geseran_satu_koma_dua_meter():
    """Geseran menyusuri tembok — kegagalan pokoknya — harus kembali.

    1,2 m adalah jarak yang tercatat masih menggeser tulisan FILKOM pada
    pasangan 0073+0075 di clomerge maupun outmerge.
    """
    pk.seed(3)
    T_benar = _geser(1.2)
    sumber, acuan = _dua_scan(T_benar)
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)
    pasangan = _jodohkan(bs, ba, T_benar)
    assert len(pasangan) >= 2

    hasil = pk.pasang(sumber, acuan, bs, ba, pasangan)

    galat = np.linalg.norm((hasil["T"] @ T_benar)[:2, 3])
    assert galat < 0.02, f"masih meleset {galat:.3f} m"


def test_pasang_mengembalikan_yaw_delapan_derajat():
    pk.seed(3)
    T_benar = _geser(0.4, -0.3, yaw=8.0)
    sumber, acuan = _dua_scan(T_benar)
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)
    pasangan = _jodohkan(bs, ba, T_benar)
    assert len(pasangan) >= 2

    hasil = pk.pasang(sumber, acuan, bs, ba, pasangan)

    assert pk.yaw_derajat(hasil["T"]) == pytest.approx(-8.0, abs=0.5)
    assert hasil["asal_yaw"] == "jangkar"


def test_pasang_satu_jangkar_bersandar_pada_tembok_dan_mengakuinya():
    """Satu jangkar saja: yaw datang dari tembok, dan laporannya harus bilang."""
    pk.seed(3)
    T_benar = _geser(0.9, 0.2)
    sumber, acuan = _dua_scan(T_benar)
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)
    pasangan = _jodohkan(bs, ba, T_benar)[:1]
    assert len(pasangan) == 1

    hasil = pk.pasang(sumber, acuan, bs, ba, pasangan)

    assert hasil["asal_yaw"] == "tembok"
    galat = np.linalg.norm((hasil["T"] @ T_benar)[:2, 3])
    assert galat < 0.05, f"masih meleset {galat:.3f} m"


def test_pasang_menolak_pasangan_kosong():
    pk.seed(3)
    sumber, acuan = _dua_scan(_geser(0.5))
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)

    with pytest.raises(ValueError):
        pk.pasang(sumber, acuan, bs, ba, [])


def test_pasang_melaporkan_sisa_jangkar_saat_salah_tunjuk():
    """Jangkar kedua ditunjuk ke tiang yang salah: jaraknya tak cocok, ketahuan."""
    pk.seed(3)
    T_benar = _geser(0.6)
    sumber, acuan = _dua_scan(T_benar)
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)
    benar = _jodohkan(bs, ba, T_benar)
    assert len(benar) >= 3
    salah = [benar[0], (benar[1][0], benar[2][1])]

    hasil = pk.pasang(sumber, acuan, bs, ba, salah)

    assert hasil["sisa_jangkar"] > 0.3
    assert hasil["peringatan"], "salah tunjuk sebesar ini harus diperingatkan"


def test_dua_jangkar_tertukar_TIDAK_terdeteksi_lewat_jarak():
    """Batas yang harus diketahui, bukan cacat yang harus disembunyikan.

    Menukar dua jangkar tidak mengubah jarak antar keduanya, jadi sisa jarak
    tetap nol. Yang menangkapnya cuma nilai akhir — dan itulah sebabnya laporan
    wajib menyertakan tajam@3cm, bukan sisa jangkar saja.
    """
    P = np.array([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0]])
    Q = np.array([[3.0, 0.0, 1.0], [0.0, 0.0, 1.0]])

    assert pk.sisa_jangkar(P, Q) == pytest.approx(0.0, abs=1e-9)


# ═══════════════════════════════════════════════════════════════════════════════
# rapikan — ICP boleh memoles, tidak boleh menggelincir
# ═══════════════════════════════════════════════════════════════════════════════

def test_rapikan_membekukan_arah_lemah_saat_redam_nol():
    """redam=0 berarti arah lemah dikunci pada jawaban jangkar. Persis nol."""
    pk.seed(3)
    T_benar = _geser(0.5)
    sumber, acuan = _dua_scan(T_benar)
    T0 = np.linalg.inv(T_benar) @ _geser(0.35)          # sengaja tergelincir

    T = pk.rapikan(sumber, acuan, T0, redam=0.0)

    lemah = pk.arah_lemah_mendatar(sumber, acuan, T0)
    pindah = float(abs(np.dot((T[:2, 3] - T0[:2, 3]), lemah)))
    assert pindah < 1e-6, f"arah lemah bergerak {pindah:.4f} m padahal dibekukan"


def test_rapikan_tetap_membetulkan_arah_yang_kuat():
    """Yang dibekukan hanya arah lemah. Tegak lurus tembok harus tetap dibetulkan."""
    pk.seed(3)
    T_benar = _geser(0.5)
    sumber, acuan = _dua_scan(T_benar)
    kuat = np.array([0.0, 1.0])                          # tegak lurus tembok
    T0 = np.linalg.inv(T_benar).copy()
    T0[:2, 3] += 0.12 * kuat

    T = pk.rapikan(sumber, acuan, T0, redam=0.0)

    sisa = float(abs(np.dot(T[:2, 3] - np.linalg.inv(T_benar)[:2, 3], kuat)))
    assert sisa < 0.04, f"arah kuat tidak dibetulkan, sisa {sisa:.3f} m"


def test_rapikan_tetap_dekat_jangkar_walau_tembok_sepuluh_kali():
    """Tembok diperbanyak 10x tanpa menambah tiang. Mayoritas tidak boleh menang."""
    pk.seed(3)
    T_benar = _geser(0.7)
    sumber, acuan = _dua_scan(T_benar, tembok_lipat=10)
    T0 = np.linalg.inv(T_benar)

    T = pk.rapikan(sumber, acuan, T0, redam=pk.REDAM)

    assert np.linalg.norm(T[:2, 3] - T0[:2, 3]) < 0.05


# ═══════════════════════════════════════════════════════════════════════════════
# Penilaian — jujur, berpasangan, dan tidak menggelembung
# ═══════════════════════════════════════════════════════════════════════════════

def test_tajam_satu_untuk_awan_melawan_dirinya():
    pk.seed(3)
    xyz = selasar(seed=40)

    n = pk.nilai(xyz, xyz, np.eye(4))

    assert n["tajam3"] == pytest.approx(1.0, abs=1e-9)
    assert n["n_tampalan"] > 0


def test_nilai_tidak_naik_saat_acuan_disampel_lebih_padat():
    """Kepadatan acuan naik, geometrinya sama: nilainya tidak boleh ikut naik.

    Inilah cacat yang membuat outmerge melapor 0,61-0,85 BAIK untuk pose yang
    diukur berpasangan cuma 0,04-0,22 — ia menilai melawan peta 3 scan yang
    jauh lebih padat, jadi tiap titik punya jauh lebih banyak kesempatan.
    """
    pk.seed(3)
    sumber = pk.terapkan(selasar(seed=41), _geser(0.5))
    acuan = selasar(seed=42)
    padat = np.vstack([acuan, selasar(seed=43), selasar(seed=44)])

    jarang = pk.nilai(sumber, acuan, np.eye(4))
    tebal = pk.nilai(sumber, padat, np.eye(4))

    assert tebal["fitness10"] <= jarang["fitness10"] + 0.05
    assert tebal["tajam3"] <= jarang["tajam3"] + 0.05


def test_nilai_menghukum_geseran_yang_fitness_lolos():
    """Jurangnya harus terlihat di laporan: fitness longgar, tajam ketat."""
    pk.seed(3)
    xyz = selasar(seed=45)

    n = pk.nilai(xyz, xyz, _geser(0.8))

    assert n["fitness10"] > 0.30, "fitness memang jenuh — itu sebabnya ia sendirian tak cukup"
    assert n["tajam3"] < 0.15, "tajam harus menolak geseran 0,8 m"


# ═══════════════════════════════════════════════════════════════════════════════
# Determinisme
# ═══════════════════════════════════════════════════════════════════════════════

def _jalankan_penuh(T_benar):
    pk.seed(99)
    sumber, acuan = _dua_scan(T_benar)
    bs, ba = pk.daftar_benda(sumber), pk.daftar_benda(acuan)
    return pk.pasang(sumber, acuan, bs, ba, _jodohkan(bs, ba, T_benar))


def test_jawaban_jangkar_persis_sama_tiap_eksekusi():
    """Bagian yang menjadi kontribusinya harus persis, bukan kira-kira.

    Jawaban jangkar bentuk tertutup: ekstraksi benda, atlas bidang, dan Kabsch
    2-D. Tidak ada satu pun yang boleh bergeser antar eksekusi — di sinilah
    kesimpulan diambil.
    """
    T_benar = _geser(0.6, 0.2, yaw=4.0)

    assert np.array_equal(_jalankan_penuh(T_benar)["T_jangkar"],
                          _jalankan_penuh(T_benar)["T_jangkar"])


def test_pose_akhir_terulang_jauh_di_bawah_ketelitian_sensor():
    """ICP menjumlah secara paralel, jadi urutan penjumlahannya bisa berubah.

    Simpangannya orde 1e-7 m — sepersepuluh mikron, terhadap derau sensor 3 cm.
    Bit-per-bit hanya bisa didapat dengan satu utas, dan harganya kecepatan.
    Yang dijanjikan di sini karena itu bukan "identik" melainkan "terulang jauh
    di bawah apa pun yang bisa diukur alatnya" — dan yang dijaga ketat adalah
    jawaban jangkarnya, di tes sebelah.
    """
    T_benar = _geser(0.6, 0.2, yaw=4.0)

    a = _jalankan_penuh(T_benar)["T"]
    b = _jalankan_penuh(T_benar)["T"]

    assert np.allclose(a, b, atol=1e-6)


def test_determinisme_tidak_bergantung_urutan_panggilan():
    """"Masukan sama → keluaran sama" harus berlaku tanpa peduli apa yang
    dipanggil sebelumnya.

    RANSAC Open3D menarik jumlah undian yang berubah-ubah walau jawabannya sama,
    jadi benih milik pemanggil saja tidak cukup: panggilan kedua mewarisi
    keadaan RNG yang tak bisa diramalkan dari yang pertama.
    """
    pk.seed(5)
    xyz = selasar(seed=50)
    sendirian = [b.pusat.copy() for b in pk.daftar_benda(xyz)]

    pk.seed(5)
    pk.daftar_benda(selasar(seed=51))          # pemanasan yang mengacak RNG
    pk.daftar_benda(selasar(seed=52))
    sesudah = [b.pusat.copy() for b in pk.daftar_benda(xyz)]

    assert len(sendirian) == len(sesudah)
    for a, b in zip(sendirian, sesudah):
        assert np.array_equal(a, b)
