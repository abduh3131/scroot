#!/usr/bin/env bash

set -e

echo "=== [1] Current directory ==="
pwd

echo "=== [2] Removing old .venv (if any) ==="
rm -rf .venv

echo "=== [3] Creating Python 3.9 virtualenv at .venv ==="
python3.9 -m venv .venv

echo "=== [4] Activating .venv ==="
# shellcheck disable=SC1091
source .venv/bin/activate

echo "Python in venv: $(which python)"
python -V

echo "=== [5] Upgrading pip and installing numpy<2 ==="
pip install --upgrade pip
pip install "numpy<2"

echo "=== [6] Stripping OpenCV wheels from autonomy/requirements.txt (we use system OpenCV) ==="
if [ -f autonomy/requirements.txt ]; then
  cp autonomy/requirements.txt autonomy/requirements.txt.bak 2>/dev/null || true
  sed -i.bak '/opencv-python/d;/opencv-contrib-python/d;/opencv-python-headless/d' autonomy/requirements.txt
else
  echo "WARNING: autonomy/requirements.txt not found, skipping OpenCV strip step."
fi

echo "=== [7] Installing remaining Python dependencies ==="
if [ -f autonomy/requirements.txt ]; then
  pip install -r autonomy/requirements.txt
else
  echo "WARNING: autonomy/requirements.txt missing, you may need to install deps manually."
fi

echo "=== [8] Patching setup_scroot.py to require Python 3.9+ instead of 3.10+ (if needed) ==="
python - << 'EOF'
import re
from pathlib import Path

path = Path("setup_scroot.py")
if not path.exists():
    print("setup_scroot.py not found, skipping patch.")
else:
    text = path.read_text(encoding="utf-8")
    # Look for a check like: if sys.version_info < (3, 10):
    pattern = r"sys\.version_info\s*<\s*\(\s*3\s*,\s*10\s*\)"
    if re.search(pattern, text):
        patched = re.sub(pattern, "sys.version_info < (3, 9)", text, count=1)
        backup = path.with_suffix(".py.bak")
        backup.write_text(text, encoding="utf-8")
        path.write_text(patched, encoding="utf-8")
        print(f"Patched {path} (backed up to {backup}).")
    else:
        print("No Python 3.10 version check pattern found; leaving setup_scroot.py unchanged.")
EOF

echo "=== [9] Running setup_scroot.py (skip model downloads) ==="
if [ -f setup_scroot.py ]; then
  python setup_scroot.py --skip-models || echo "setup_scroot.py returned non-zero; it may already be configured."
else
  echo "WARNING: setup_scroot.py not found, skipping."
fi

echo "=== [10] Testing camera access from INSIDE the venv (Python 3.9) ==="
python - << 'EOF'
import cv2

print("cv2 version:", cv2.__version__)
cap = cv2.VideoCapture(0)
print("Opened:", cap.isOpened())
ret, frame = cap.read()
print("Got frame:", ret, "shape:", None if frame is None else frame.shape)
cap.release()
EOF

echo "=== [11] Launching scroot GUI (scooter_app.py) ==="
python scooter_app.py