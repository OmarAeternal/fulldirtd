#!/usr/bin/env python3
"""
scan_launcher.py - pembungkus untuk sweep_mapping.launch.py

File ini TIDAK mengubah kode yang sudah ada. Dia memanggil launch file yang
sama seperti biasa, hanya dengan tiga tambahan:

  1. Resolusi port  - menentukan port LiDAR lewat /dev/serial/by-id, lalu
                      menimpanya dengan serial_port:= . Kebal terhadap
                      tertukarnya urutan /dev/ttyUSB*.
  2. Kecepatan rpm  - menerima --rpm, dikonversi ke parameter delay
                      (detik per step) yang dimengerti stepper_sweep_node.
  3. Watchdog       - membatalkan run kalau /scan tidak mengirim data, supaya
                      tidak lahir bag tanpa titik seperti scan_0011..0014
                      (stepper berputar 57 detik, LiDAR diam).

Default: 1 sweep, 1 rpm.
"""

import argparse
import fnmatch
import glob
import os
import signal
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Nilai-nilai ini dibaca dari sweep_mapping.launch.py dan sllidar_c1_launch.py
# ---------------------------------------------------------------------------
LAUNCH_PKG = "sweep_mapping"
LAUNCH_FILE = "sweep_mapping.launch.py"
PORT_ARG = "serial_port"          # dideklarasikan di sllidar_c1_launch.py
SCAN_TOPIC = "/scan"

LIDAR_BAUD = 460800               # serial_baudrate default C1
STEPS_PER_REV = 1600              # step motor per putaran motor
GEAR_RATIO = 0.5                  # pulley motor / pulley LiDAR (30T/60T)

STREAM_THRESHOLD = 200            # byte/detik minimum agar dianggap streaming


def delay_from_rpm(rpm, steps_per_rev, gear_ratio):
    """Ubah rpm LiDAR menjadi parameter `delay` (detik per step motor).

    Satu putaran LiDAR butuh steps_per_rev / gear_ratio step motor.
    Dengan default 1600 / 0.5 = 3200 step. Pada 1 rpm (60 detik per putaran)
    berarti 60 / 3200 = 0.01875 detik per step.

    Angka ini teoretis: overhead Python membuat rpm nyata sedikit di bawah
    target. Bag lama memakai delay 0.018 dan durasinya terukur ~57.6 detik.
    """
    if rpm <= 0:
        raise ValueError("rpm harus lebih besar dari 0")
    return 60.0 * gear_ratio / (steps_per_rev * rpm)


# ---------------------------------------------------------------------------
# Inventaris perangkat serial
# ---------------------------------------------------------------------------
def serial_inventory():
    """Kumpulkan kandidat port serial beserta nama stabilnya.

    Symlink by-id dan by-path sudah disediakan udev bawaan Ubuntu - tidak
    perlu menambah rule apa pun.
    """
    table = {}

    for raw in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        table[os.path.realpath(raw)] = {"dev": raw, "by_id": None, "by_path": None}

    for kind, pattern in (("by_id", "/dev/serial/by-id/*"),
                          ("by_path", "/dev/serial/by-path/*")):
        for link in sorted(glob.glob(pattern)):
            target = os.path.realpath(link)
            entry = table.setdefault(
                target, {"dev": target, "by_id": None, "by_path": None})
            if entry[kind] is None:
                entry[kind] = link

    return sorted(table.values(), key=lambda e: e["dev"])


def stable_name(entry):
    """by-id (berbasis serial number chip) > by-path (posisi fisik) > ttyUSB*."""
    return entry["by_id"] or entry["by_path"] or entry["dev"]


def probe(dev, baud, duration):
    """Hitung byte yang mengalir sendiri dari sebuah port.

    LiDAR C1 memuntahkan data terus-menerus begitu motornya berputar. Modul
    IMU WIT umumnya juga streaming, tapi jauh lebih pelan. Port yang diam
    sama sekali berarti tidak ada perangkat aktif di sana.

    DTR dan RTS ditahan rendah sebelum port dibuka supaya probing tidak
    me-reset board yang kebetulan tersentuh.
    """
    try:
        import serial
    except ImportError:
        return None, "pyserial tidak terpasang"

    port = serial.Serial()
    port.port = dev
    port.baudrate = baud
    port.timeout = 0.2
    port.dtr = False
    port.rts = False

    try:
        port.open()
    except Exception as exc:
        return None, str(exc)

    total = 0
    deadline = time.time() + duration
    try:
        while time.time() < deadline:
            total += len(port.read(4096))
    except Exception as exc:
        return None, str(exc)
    finally:
        port.close()

    return total, None


def resolve_lidar_port(args):
    """Tentukan port LiDAR. Mengembalikan (path_stabil, alasan)."""
    if args.port:
        return args.port, "ditentukan manual lewat --port"

    devices = serial_inventory()
    if not devices:
        die("Tidak ada perangkat serial di /dev/ttyUSB* maupun /dev/ttyACM*. "
            "LiDAR belum tercolok atau kabelnya bermasalah.")

    candidates = devices
    if args.lidar_pattern:
        candidates = [d for d in devices
                      if d["by_id"] and fnmatch.fnmatch(
                          os.path.basename(d["by_id"]), args.lidar_pattern)]
        if not candidates:
            die(f"Tidak ada perangkat yang cocok dengan pola "
                f"'{args.lidar_pattern}'. Jalankan --check-only untuk melihat "
                f"nama yang tersedia.")

    if args.exclude:
        candidates = [d for d in candidates
                      if not any(ex in stable_name(d) for ex in args.exclude)]
        if not candidates:
            die("Semua kandidat tersaring habis oleh --exclude.")

    if len(candidates) == 1:
        return stable_name(candidates[0]), "satu-satunya kandidat"

    if args.no_probe:
        die(f"Ada {len(candidates)} kandidat dan probing dimatikan. "
            f"Tentukan --port atau --lidar-pattern secara eksplisit.")

    print(f"  {len(candidates)} kandidat, probing masing-masing "
          f"{args.probe_time}s @ {args.baud} baud...")

    scored = []
    for entry in candidates:
        count, err = probe(entry["dev"], args.baud, args.probe_time)
        if err:
            print(f"    {entry['dev']}: dilewati ({err})")
            continue
        rate = count / args.probe_time
        print(f"    {entry['dev']}: {rate:.0f} byte/s")
        scored.append((rate, stable_name(entry)))

    if not scored:
        die("Tidak satu pun port bisa dibuka. Kalau pesannya 'Permission "
            "denied', user belum masuk grup dialout - itu satu-satunya bagian "
            "yang harus diperbaiki di level sistem:\n"
            "          sudo usermod -aG dialout $USER   (lalu logout-login)")

    scored.sort(reverse=True)
    best_rate, best = scored[0]

    if best_rate < STREAM_THRESHOLD:
        die(f"Tidak ada port yang mengalirkan data (tertinggi {best_rate:.0f} "
            f"byte/s). LiDAR tercolok tapi diam - periksa suplai daya motor "
            f"LiDAR, atau baudrate salah (coba --baud 115200).")

    return best, f"streaming {best_rate:.0f} byte/s, tertinggi di antara kandidat"


def warn_imu_conflict(port):
    """Peringatkan kalau LiDAR mendarat di port yang direbut node IMU.

    wit_ros2_imu/launch/rviz_and_imu.launch.py meng-hardcode {'port':
    '/dev/ttyUSB0'} sebagai parameter Node, bukan LaunchConfiguration, jadi
    nilainya tidak bisa ditimpa dari CLI. Kalau LiDAR kebetulan berada di
    ttyUSB0, node IMU akan ikut membuka port yang sama dan bisa membuat
    driver sllidar gagal.
    """
    if os.path.realpath(port) == "/dev/ttyUSB0":
        print("\n  [PERINGATAN] LiDAR terdeteksi di /dev/ttyUSB0, port yang "
              "juga dibuka node IMU\n"
              "               (hardcoded di wit_ros2_imu). Keduanya bisa "
              "berebut port yang sama.\n"
              "               Kalau /scan tetap kosong, pindahkan LiDAR ke "
              "lubang USB lain,\n"
              "               atau colok IMU-nya supaya ttyUSB0 terisi "
              "perangkat yang benar.\n")


# ---------------------------------------------------------------------------
# Pemeriksaan topik ROS
# ---------------------------------------------------------------------------
def topic_exists(topic, timeout=5):
    try:
        out = subprocess.run(["ros2", "topic", "list"],
                             capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return topic in out.stdout.split()


def topic_has_data(topic, timeout):
    """True kalau minimal satu pesan berhasil diterima dalam batas waktu."""
    try:
        result = subprocess.run(["ros2", "topic", "echo", "--once", topic],
                                capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0 and bool(result.stdout.strip())


# ---------------------------------------------------------------------------
# Bantuan kecil
# ---------------------------------------------------------------------------
def die(msg, code=2):
    print(f"\n[GAGAL] {msg}\n", file=sys.stderr)
    sys.exit(code)


def check_only(args):
    print("Perangkat serial yang terdeteksi:\n")
    devices = serial_inventory()
    if not devices:
        print("  (tidak ada - LiDAR dan IMU sama-sama tidak tercolok)")
        return

    for entry in devices:
        print(f"  {entry['dev']}")
        print(f"    by-id  : {entry['by_id'] or '- (chip tanpa serial number)'}")
        print(f"    by-path: {entry['by_path'] or '-'}")
        count, err = probe(entry["dev"], args.baud, args.probe_time)
        if err:
            print(f"    aliran : tidak terbaca ({err})")
        else:
            rate = count / args.probe_time
            verdict = ("STREAMING - kemungkinan besar LiDAR"
                       if rate >= STREAM_THRESHOLD else "diam")
            print(f"    aliran : {rate:.0f} byte/detik -> {verdict}")
        print()

    print("Catatan: stepper memakai GPIO (lgpio), bukan serial, jadi dia tidak "
          "pernah muncul\ndi daftar ini. Perangkat serial di rig ini hanya "
          "LiDAR C1 dan IMU WIT.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(
        description="Jalankan sweep_mapping dengan port LiDAR terdeteksi otomatis.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Contoh:\n"
               "  ./scan_launcher.py                       # 1 sweep, 1 rpm\n"
               "  ./scan_launcher.py --sweeps 3 --rpm 2\n"
               "  ./scan_launcher.py --check-only          # cuma lihat port\n"
               "  ./scan_launcher.py --sweeps 2 -- bag_prefix:=uji foxglove:=false\n")

    parser.add_argument("--sweeps", type=int, default=1,
                        help="jumlah sweep, 0 = terus-menerus (default: 1)")
    parser.add_argument("--rpm", type=float, default=1.0,
                        help="kecepatan putar LiDAR (default: 1.0)")
    parser.add_argument("--delay", type=float, default=None,
                        help="detik per step, menimpa --rpm kalau diisi")
    parser.add_argument("--steps-per-rev", type=int, default=STEPS_PER_REV,
                        help=f"step motor per putaran (default: {STEPS_PER_REV})")
    parser.add_argument("--gear-ratio", type=float, default=GEAR_RATIO,
                        help=f"pulley motor / pulley LiDAR (default: {GEAR_RATIO})")

    parser.add_argument("--port", default=None,
                        help="paksa port LiDAR, lewati deteksi otomatis")
    parser.add_argument("--lidar-pattern", default=None,
                        help="pola nama by-id untuk menyaring kandidat")
    parser.add_argument("--exclude", action="append", default=[],
                        help="potongan nama yang diabaikan, boleh berulang "
                             "(misal nama by-id modul IMU)")
    parser.add_argument("--baud", type=int, default=LIDAR_BAUD,
                        help=f"baudrate untuk probing (default: {LIDAR_BAUD})")
    parser.add_argument("--probe-time", type=float, default=1.0,
                        help="lama probing tiap port, detik (default: 1.0)")
    parser.add_argument("--no-probe", action="store_true",
                        help="jangan buka port apa pun untuk probing")
    parser.add_argument("--no-port-arg", action="store_true",
                        help="jangan kirim serial_port:= ke launch file")

    parser.add_argument("--pkg", default=LAUNCH_PKG,
                        help=f"package ROS 2 (default: {LAUNCH_PKG})")
    parser.add_argument("--launch-file", default=LAUNCH_FILE,
                        help=f"launch file (default: {LAUNCH_FILE})")

    parser.add_argument("--scan-timeout", type=float, default=20.0,
                        help="batas tunggu data /scan, detik (default: 20)")
    parser.add_argument("--no-watchdog", action="store_true",
                        help="jangan batalkan run walau /scan tidak muncul")

    parser.add_argument("--check-only", action="store_true",
                        help="hanya tampilkan inventaris port, tidak launch")
    parser.add_argument("--dry-run", action="store_true",
                        help="tampilkan perintah yang akan dijalankan lalu keluar")
    parser.add_argument("extra", nargs="*", metavar="key:=value",
                        help="argumen tambahan diteruskan apa adanya ke "
                             "sweep_mapping.launch.py")
    return parser


def main():
    args = build_parser().parse_args()

    if args.check_only:
        check_only(args)
        return 0

    if args.delay is not None:
        delay = args.delay
        speed_note = f"delay {delay} s/step (ditentukan manual)"
    else:
        try:
            delay = delay_from_rpm(args.rpm, args.steps_per_rev, args.gear_ratio)
        except ValueError as exc:
            die(str(exc))
        speed_note = f"{args.rpm} rpm -> delay {delay:.5f} s/step"

    print("== 1/3 Resolusi port LiDAR ==")
    port, reason = resolve_lidar_port(args)
    print(f"  -> {port}")
    print(f"     ({reason})")
    warn_imu_conflict(port)

    print("\n== 2/3 Menyusun perintah launch ==")
    print(f"  {speed_note}")
    steps_per_sweep = args.steps_per_rev / args.gear_ratio
    print(f"  perkiraan durasi: {steps_per_sweep * delay:.1f} detik per sweep\n")

    cmd = ["ros2", "launch", args.pkg, args.launch_file]
    if not args.no_port_arg:
        cmd.append(f"{PORT_ARG}:={port}")
    cmd.append(f"sweeps:={args.sweeps}")
    cmd.append(f"delay:={delay:.6f}")
    cmd.append(f"steps_per_rev:={args.steps_per_rev}")
    cmd.append(f"gear_ratio:={args.gear_ratio}")
    cmd.extend(args.extra)

    print("  " + " ".join(cmd) + "\n")

    if args.dry_run:
        return 0

    print(f"== 3/3 Menjalankan ({args.sweeps} sweep @ {args.rpm} rpm) ==\n")

    # start_new_session memberi anak process group sendiri, supaya seluruh
    # pohon node bisa dimatikan sekaligus oleh watchdog maupun Ctrl-C.
    proc = subprocess.Popen(cmd, start_new_session=True)

    def forward_sigint(_signum, _frame):
        print("\n  Ctrl-C diterima, menghentikan launch...")
        stop(proc)

    signal.signal(signal.SIGINT, forward_sigint)

    if not args.no_watchdog:
        if not wait_for_scan(proc, args):
            stop(proc)
            die(f"Topik {SCAN_TOPIC} tidak mengirim data dalam "
                f"{args.scan_timeout:.0f} detik. Run dibatalkan supaya tidak "
                f"menghasilkan bag kosong.\n"
                f"        Gulung ke atas dan cari baris [ERROR] dari "
                f"sllidar_node - pesannya menentukan langkah berikutnya.",
                code=3)

    return proc.wait()


def wait_for_scan(proc, args):
    """Tunggu sampai topik scan hidup dan benar-benar mengirim pesan."""
    print(f"  [watchdog] menunggu {SCAN_TOPIC} ...")
    deadline = time.time() + args.scan_timeout

    while time.time() < deadline:
        if proc.poll() is not None:
            print(f"  [watchdog] launch keluar duluan (kode {proc.returncode})")
            return False
        if topic_exists(SCAN_TOPIC):
            sisa = max(2.0, deadline - time.time())
            if topic_has_data(SCAN_TOPIC, timeout=sisa):
                print(f"  [watchdog] {SCAN_TOPIC} hidup dan mengirim data. "
                      f"Scan berjalan.\n")
                return True
        time.sleep(1.0)

    return False


def stop(proc):
    """Matikan seluruh process group anak dengan rapi."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


if __name__ == "__main__":
    sys.exit(main())
