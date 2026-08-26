#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

exec .venv/bin/python -u pico_teleop_piper_fixed.py \
  --hardware \
  --controller-hand left \
  --position-scale 0.8 \
  --no-speed-limit \
  --no-workspace-limit \
  --binary-gripper \
  --can-name can0 \
  --speed-percent 100 \
  --yaw-deg 0
