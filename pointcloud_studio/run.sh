#!/usr/bin/env bash
# PointCloud Studio — launcher.
# Membuat venv + install dependensi saat pertama kali, lalu menyalakan server & membuka browser.
set -e
cd "$(dirname "$0")"

PORT="${PORT:-8000}"
VENV=".venv"

# 1. Setup venv + dependensi (sekali saja)
if [ ! -d "$VENV" ]; then
  echo "[setup] Membuat virtual environment…"
  python3 -m venv "$VENV"
fi
if ! "$VENV/bin/python" -c "import fastapi, open3d" 2>/dev/null; then
  echo "[setup] Memasang dependensi (mengunduh Open3D ~400MB, sekali saja)…"
  "$VENV/bin/pip" install --upgrade pip -q
  "$VENV/bin/pip" install -q -r requirements.txt
fi

# 2. Unduh Three.js bila belum ada (untuk mode offline)
if [ ! -f "frontend/vendor/three.module.js" ]; then
  echo "[setup] Mengunduh Three.js…"
  curl -sL "https://unpkg.com/three@0.160.0/build/three.module.js" -o frontend/vendor/three.module.js
  curl -sL "https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js" -o frontend/vendor/OrbitControls.js
fi

# 3. Jalankan server + buka browser
URL="http://127.0.0.1:${PORT}"
echo "[jalan] PointCloud Studio di ${URL}  (Ctrl+C untuk berhenti)"
( sleep 1.5; (xdg-open "$URL" || google-chrome "$URL" || true) >/dev/null 2>&1 ) &
exec "$VENV/bin/python" -m uvicorn backend.server:app --host 127.0.0.1 --port "$PORT" --log-level warning
