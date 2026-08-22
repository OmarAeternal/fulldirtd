"""Tes bagian logika perintah `pcs`."""
import pytest

import pcs


def test_nilai_bawaan():
    a = pcs.build_parser().parse_args(["awan.ply"])
    assert a.voxel == 0.01
    assert a.port == 8000
    assert a.full is False
    assert a.force is False
    assert a.topic is None


def test_flag_diurai():
    a = pcs.build_parser().parse_args(
        ["awan.ply", "--voxel", "0.005", "--full", "--port", "8123",
         "--force", "-t", "/map_3d"])
    assert a.voxel == 0.005
    assert a.full is True
    assert a.port == 8123
    assert a.force is True
    assert a.topic == "/map_3d"


@pytest.mark.parametrize("nama", ["a.ply", "a.PLY", "a.xyz"])
def test_berkas_cloud_dikenali(nama):
    assert pcs.jenis_berkas(nama) == "cloud"


@pytest.mark.parametrize("nama", ["a.mcap", "a.mcap.zstd", "a.MCAP"])
def test_berkas_mcap_dikenali(nama):
    assert pcs.jenis_berkas(nama) == "mcap"


def test_ekstensi_asing_ditolak():
    with pytest.raises(SystemExit):
        pcs.jenis_berkas("catatan.pdf")


def test_url_mengkodekan_spasi_pada_path():
    url = pcs.bangun_url(8000, ["/home/bromarku/riset td/a.ply"], 0.01, False)
    assert "riset%20td" in url
    assert " " not in url


def test_url_memuat_port_voxel_dan_file():
    url = pcs.bangun_url(8123, ["/data/a.ply"], 0.005, False)
    assert url.startswith("http://127.0.0.1:8123/?")
    assert "file=%2Fdata%2Fa.ply" in url or "file=/data/a.ply" in url
    assert "voxel=0.005" in url
    assert "full=" not in url


def test_url_menyertakan_full_hanya_bila_diminta():
    assert "full=1" in pcs.bangun_url(8000, ["/data/a.ply"], 0.01, True)


# `pcs` tanpa argumen: buka aplikasinya saja, biarkan pemakainya memilih berkas
# lewat tombol "Buka" atau seret-lepas.

def test_berkas_boleh_kosong():
    a = pcs.build_parser().parse_args([])
    assert a.file == []


def test_url_tanpa_berkas_polos_tanpa_kueri():
    assert pcs.bangun_url(8000, [], 0.01, False) == "http://127.0.0.1:8000/"


def test_url_tanpa_berkas_mengabaikan_voxel_dan_full():
    url = pcs.bangun_url(8123, [], 0.005, True)
    assert url == "http://127.0.0.1:8123/"


def test_siapkan_berkas_kosong_mengembalikan_daftar_kosong():
    a = pcs.build_parser().parse_args([])
    assert pcs.siapkan_berkas(a) == []


# Banyak berkas: tiap berkas jadi satu layer di aplikasinya.

def test_banyak_berkas_diurai():
    a = pcs.build_parser().parse_args(["a.ply", "b.ply", "c.xyz"])
    assert a.file == ["a.ply", "b.ply", "c.xyz"]


def test_url_banyak_berkas_mengulang_parameter_file():
    url = pcs.bangun_url(8000, ["/data/a.ply", "/data/b.ply"], 0.01, False)
    assert url.count("file=") == 2
    assert url.index("a.ply") < url.index("b.ply")   # urutan terjaga
    assert url.count("voxel=") == 1                  # berlaku untuk semuanya


def test_url_mengkodekan_spasi_pada_semua_berkas():
    url = pcs.bangun_url(8000, ["/home/riset td/a.ply", "/home/riset td/b.ply"],
                         0.01, False)
    assert url.count("riset%20td") == 2
    assert " " not in url


def test_siapkan_berkas_banyak_menjaga_urutan(tmp_path):
    nama = ["b.ply", "a.ply", "c.xyz"]
    for n in nama:
        (tmp_path / n).write_text("")
    a = pcs.build_parser().parse_args([str(tmp_path / n) for n in nama])
    assert [p.name for p in pcs.siapkan_berkas(a)] == nama


def test_siapkan_berkas_hasilnya_absolut(tmp_path, monkeypatch):
    (tmp_path / "a.ply").write_text("")
    monkeypatch.chdir(tmp_path)
    a = pcs.build_parser().parse_args(["a.ply"])
    assert pcs.siapkan_berkas(a)[0].is_absolute()


def test_berkas_hilang_gagal_sebelum_konversi(tmp_path, monkeypatch):
    """Salah ketik di berkas ke-2 tidak boleh membuang menit mengonversi ke-1."""
    (tmp_path / "a.mcap").write_text("")
    dipanggil = []
    monkeypatch.setattr(pcs, "konversi_mcap", lambda *a, **k: dipanggil.append(a))
    a = pcs.build_parser().parse_args(
        [str(tmp_path / "a.mcap"), str(tmp_path / "tidak_ada.ply")])
    with pytest.raises(SystemExit):
        pcs.siapkan_berkas(a)
    assert dipanggil == []


def test_ekstensi_asing_gagal_sebelum_konversi(tmp_path, monkeypatch):
    (tmp_path / "a.mcap").write_text("")
    (tmp_path / "catatan.pdf").write_text("")
    dipanggil = []
    monkeypatch.setattr(pcs, "konversi_mcap", lambda *a, **k: dipanggil.append(a))
    a = pcs.build_parser().parse_args(
        [str(tmp_path / "a.mcap"), str(tmp_path / "catatan.pdf")])
    with pytest.raises(SystemExit):
        pcs.siapkan_berkas(a)
    assert dipanggil == []
