#!/usr/bin/env bash
# Quickchanger smooth demo sweep: one continuous scan MoveL while detecting ArUco.
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

PYTHON_BIN="python"
if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  PYTHON_BIN="python3"
fi

for arg in "$@"; do
  if [[ "${arg}" == "-h" || "${arg}" == "--help" ]]; then
    exec "${PYTHON_BIN}" "${SCRIPT_DIR}/quickchanger_smooth_sweep_hd.py" --profile hd "$@"
  fi
done

"${SCRIPT_DIR}/robot_sdk_preempt.sh" \
  "${PYTHON_BIN}" "${SCRIPT_DIR}/quickchanger_smooth_sweep_hd.py" --profile hd "$@"
