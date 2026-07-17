#!/usr/bin/env python3
"""检查当前 Python 能否导入 FAIRINO SDK（fairino 或顶层 Robot）。"""
from __future__ import annotations

import sys


def main() -> int:
    try:
        from fairino import Robot  # type: ignore

        print("OK: from fairino import Robot")
        return 0
    except Exception as e1:
        try:
            import Robot as _R  # type: ignore

            print("OK: import Robot")
            return 0
        except Exception as e2:
            print("无法导入 FAIRINO SDK。", file=sys.stderr)
            print("  fairino:", e1, file=sys.stderr)
            print("  Robot:", e2, file=sys.stderr)
            print("请按 README_RELEASE.md / requirements.txt 安装或配置 PYTHONPATH。", file=sys.stderr)
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
