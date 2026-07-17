#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""与主线 pick_point_move 一致的 CLI 参数（开发目录专用）。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_DEV_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DEV_DIR))

from _paths import pick_point_calib_dir  # noqa: E402

_CALIB_DIR = pick_point_calib_dir()
sys.path.insert(0, str(_CALIB_DIR))

import pick_point_move as ppm  # noqa: E402

from hd_realsense import add_profile_arguments  # noqa: E402
from pick_point_depth_filter import add_depth_stabilize_args  # noqa: E402


def add_pick_point_arguments(ap: argparse.ArgumentParser) -> None:
    add_profile_arguments(ap)
    ap.add_argument("--host", default=ppm.FR10_HOST)
    ap.add_argument("--pixel", nargs=2, type=int, metavar=("U", "V"))
    ap.add_argument(
        "--approach",
        choices=("tcp_above", "standoff"),
        default="tcp_above",
    )
    ap.add_argument(
        "--height-mm",
        type=float,
        default=0.0,
        help=(
            f"在默认高度 {ppm.BASE_HEIGHT_MM:.0f} mm 之上的增量 mm"
            f"（不传则 {ppm.BASE_HEIGHT_MM:.0f}）"
        ),
    )
    ap.add_argument(
        "--height-ref",
        choices=("camera", "tcp"),
        default="camera",
    )
    ap.add_argument("--height-bias-mm", type=float, default=0.0)
    ap.add_argument("--assume-depth-mm", type=float, default=None)
    ap.add_argument("--no-field-offset", action="store_true")
    ap.add_argument(
        "--deadzone-mm",
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DZ"),
        default=None,
        help="覆盖死区修正 mm，例: --deadzone-mm 0 0 8",
    )
    ap.add_argument(
        "--normal",
        choices=("camera", "base_z", "plane"),
        default="camera",
    )
    ap.add_argument(
        "--normal-fallback",
        choices=("camera", "base_z"),
        default="camera",
    )
    ap.add_argument("--vel", type=float, default=10.0)
    ap.add_argument("--tool", type=int, default=0)
    ap.add_argument("--user", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ppm.add_orient_arguments(ap)
    ap.add_argument("--auto-reduce-height", action="store_true", default=True)
    ap.add_argument(
        "--no-auto-reduce-height",
        action="store_false",
        dest="auto_reduce_height",
    )
    add_depth_stabilize_args(ap)
