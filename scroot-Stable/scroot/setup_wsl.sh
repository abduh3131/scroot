#!/usr/bin/env bash
set -euo pipefail

# WSL-aware setup for the minimal navigator.
# Installs system prerequisites (when apt is available), prepares a venv,
# and installs Python dependencies with the correct Torch build for CPU or CUDA.

is_wsl() {
  if grep -qiE "microsoft|wsl" /proc/version 2>/dev/null; then
    return 0
  fi
  return 1
}

maybe_install_apt_packages() {
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "apt-get not available; install ffmpeg and python3-venv manually if needed." >&2
    return
  fi

  local packages=(python3-venv python3-pip ffmpeg libgl1 libglib2.0-0)
  local missing=()
  for pkg in "${packages[@]}"; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
      missing+=("$pkg")
    fi
  done

  if [ ${#missing[@]} -gt 0 ]; then
    echo "Installing missing system packages: ${missing[*]}"
    sudo apt-get update -y
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}"
  else
    echo "All required system packages already installed."
  fi
}

select_python() {
  if command -v python3.8 >/dev/null 2>&1; then
    echo "python3.8"
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    echo "python3"
    return
  fi
  echo "Error: python3 is not installed." >&2
  exit 1
}

ensure_python_version() {
  local py_bin="$1"
  "$py_bin" - <<'PY'
import sys
if sys.version_info < (3, 8):
    raise SystemExit(f"Python 3.8+ is required; found {sys.version}")
print(f"Using Python {sys.version.split()[0]}")
PY
}

install_python_dependencies() {
  local py_bin="$1"
  local torch_index="https://download.pytorch.org/whl/cpu"
  if command -v nvidia-smi >/dev/null 2>&1; then
    torch_index="https://download.pytorch.org/whl/cu121"
    echo "Detected NVIDIA GPU; installing CUDA-enabled Torch from $torch_index"
  else
    echo "No NVIDIA GPU detected; installing CPU-only Torch from $torch_index"
  fi

  "$py_bin" -m pip install --upgrade pip setuptools wheel
  "$py_bin" -m pip install --upgrade \
    "torch==2.2.2" "torchvision==0.17.2" --index-url "$torch_index"
  "$py_bin" -m pip install --upgrade \
    ultralytics==8.1.0 \
    opencv-python-headless>=4.8.1.78 \
    "numpy>=1.24,<2" \
    pyserial>=3.5
}

main() {
  if is_wsl; then
    echo "WSL environment detected."
  else
    echo "WSL markers not found; continuing in generic Linux mode."
  fi

  maybe_install_apt_packages

  local py_bin
  py_bin=$(select_python)
  ensure_python_version "$py_bin"

  echo "Creating virtual environment at .venv"
  "$py_bin" -m venv .venv
  # shellcheck disable=SC1091
  source .venv/bin/activate

  install_python_dependencies "python"

  echo "Setup complete. Activate the environment with 'source .venv/bin/activate' and run:"
  echo "  python navigator.py --input /path/to/video.mp4 --output overlay.mp4"
}

main "$@"
