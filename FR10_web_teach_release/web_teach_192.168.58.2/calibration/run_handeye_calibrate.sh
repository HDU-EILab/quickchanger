#!/usr/bin/env bash
# FR10 棋盘格手眼标定启动脚本（rmc-yolo 视觉 + fr10-fairino Robot SDK）
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FR10_SITE="/home/nvidia/miniconda3/envs/fr10-fairino/lib/python3.10/site-packages"
CONDA_SH="/home/nvidia/miniconda3/etc/profile.d/conda.sh"

if [[ ! -f "${CONDA_SH}" ]]; then
  echo "[FAIL] 未找到 conda: ${CONDA_SH}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${CONDA_SH}"
conda activate rmc-yolo
export PYTHONPATH="${FR10_SITE}:${PYTHONPATH:-}"

exec python "${SCRIPT_DIR}/fr10_handeye_calibrate.py" "$@"
