#!/usr/bin/env bash
# Quickchanger ArUco two-stage move（默认 DICT_4X4_50, id=1）
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FR10_SITE="/home/nvidia/miniconda3/envs/fr10-fairino/lib/python3.10/site-packages"
CONDA_SH="/home/nvidia/miniconda3/etc/profile.d/conda.sh"

if [[ -f "${CONDA_SH}" ]]; then
  source "${CONDA_SH}"
  conda activate rmc-yolo
  export PYTHONPATH="${FR10_SITE}:${PYTHONPATH:-}"
fi

if [[ -z "${DISPLAY:-}" ]]; then
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  elif [[ -S /tmp/.X11-unix/X1001 ]]; then
    export DISPLAY=:1001
  fi
fi

for arg in "$@"; do
  if [[ "${arg}" == "-h" || "${arg}" == "--help" ]]; then
    exec python "${SCRIPT_DIR}/quickchanger_aruco_hd.py" --profile hd "$@"
  fi
done

"${SCRIPT_DIR}/robot_sdk_preempt.sh" \
  python "${SCRIPT_DIR}/quickchanger_aruco_hd.py" --profile hd "$@"
