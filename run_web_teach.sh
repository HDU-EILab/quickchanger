#!/usr/bin/env bash
# 不依赖 conda：使用 PATH 中的 python3（需已安装 FAIRINO SDK）。
# 在「本压缩包解压后的 FR10_web_teach_release 目录」下执行。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "$SCRIPT_DIR/web_teach_192.168.58.2"
PORT="${PORT:-8890}"
exec python3 fr10_web_teach.py --port "$PORT"
