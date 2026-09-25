#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

exec .venv/bin/python -u pico_teleop_piper_fixed.py \
  --hardware \
  --safe-test \
  --no-gripper \
  --can-name can0 \
  --speed-percent 20 \
  --yaw-deg 0
