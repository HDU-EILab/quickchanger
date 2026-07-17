#!/usr/bin/env bash
# 在「本压缩包解压后的 FR10_web_teach_release 目录」下执行。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR/web_teach_192.168.58.2"
conda activate fr10-fairino
exec python fr10_web_teach.py --port 8890
