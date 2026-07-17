#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FR10_SITE="/home/nvidia/miniconda3/envs/fr10-fairino/lib/python3.10/site-packages"
CONDA_SH="/home/nvidia/miniconda3/etc/profile.d/conda.sh"
source "${CONDA_SH}"
conda activate rmc-yolo
export PYTHONPATH="${FR10_SITE}:${PYTHONPATH:-}"
exec python "${SCRIPT_DIR}/pick_point_move.py" "$@"
