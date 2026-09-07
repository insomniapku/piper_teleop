#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${PIPER_PYTHON:-}" ]]; then
  python_bin="${PIPER_PYTHON}"
elif [[ -x "${repo_dir}/.venv/bin/python" ]]; then
  python_bin="${repo_dir}/.venv/bin/python"
else
  python_bin="$(command -v python3)"
fi

if [[ ! -x "${python_bin}" ]]; then
  echo "Python is not executable: ${python_bin}" >&2
  exit 1
fi

exec "${python_bin}" -u "${repo_dir}/pico_teleop_piper_bimanual_v2.py" \
  --hardware \
  --left-can-name can0 \
  --right-can-name can1 \
  --position-scale 0.8 \
  --rotation-scale 1.0 \
  --max-orientation-delta-deg 180 \
  --max-joint-step-deg 5 \
  --ik-position-tolerance-mm 2 \
  --no-speed-limit \
  --no-workspace-limit \
  --speed-percent 100 \
  --binary-gripper \
  --left-yaw-deg 180 \
  --right-yaw-deg 180 \
  --enable-recording \
  --recording-dir "${repo_dir}/recordings" \
  --camera-fps 30
