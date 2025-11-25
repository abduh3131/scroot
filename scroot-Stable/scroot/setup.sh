#!/usr/bin/env bash
set -euo pipefail

python3 -m pip install --upgrade pip
python3 -m pip install --upgrade \
  ultralytics==8.1.0 \
  opencv-python-headless>=4.8.1.78 \
  numpy>=1.24 \
  pyserial>=3.5

printf '\nDependencies installed. If running on Jetson without CUDA, leave --device=cpu.\n'
