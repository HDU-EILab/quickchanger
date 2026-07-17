#!/usr/bin/env python3
"""RH56 打开（与网页示教 rh56_open 一致）。默认串口 /dev/fr10-hand-serial。"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from rh56_hand import rh56_open_sequence  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description="RH56 打开")
    ap.add_argument("--port", default="/dev/fr10-hand-serial", help="串口设备路径，默认 /dev/fr10-hand-serial")
    ap.add_argument("--id", type=int, default=1, help="灵巧手 ID，默认 1")
    args = ap.parse_args()
    port = (args.port or "").strip() or "/dev/fr10-hand-serial"
    rh56_open_sequence(port, hand_id=args.id)
    print("OK: 打开指令已发送", port)


if __name__ == "__main__":
    main()
