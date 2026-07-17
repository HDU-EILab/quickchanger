#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高分辨率测试目录专用的 RealSense 打开与内参打印。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_DEV_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DEV_DIR))

from _paths import pick_point_calib_dir  # noqa: E402

_CALIB_DIR = pick_point_calib_dir()
sys.path.insert(0, str(_CALIB_DIR))

from fr10_handeye_calibrate import RealSenseGrabber  # noqa: E402

from hd_config import HD_PROFILE, LEGACY_PROFILE, PROFILES, StreamProfile  # noqa: E402


def add_profile_arguments(ap: argparse.ArgumentParser) -> None:
    ap.add_argument(
        "--profile",
        choices=tuple(PROFILES.keys()),
        default=HD_PROFILE.name,
        help="取流预设：hd=1920x1080+1280x720，legacy=1280x720+848x480",
    )
    ap.add_argument("--color-width", type=int, default=None)
    ap.add_argument("--color-height", type=int, default=None)
    ap.add_argument("--depth-width", type=int, default=None)
    ap.add_argument("--depth-height", type=int, default=None)
    ap.add_argument("--fps", type=int, default=None)


def resolve_profile(args: argparse.Namespace) -> StreamProfile:
    base = PROFILES[args.profile]
    return StreamProfile(
        name=base.name,
        color_width=int(args.color_width or base.color_width),
        color_height=int(args.color_height or base.color_height),
        depth_width=int(args.depth_width or base.depth_width),
        depth_height=int(args.depth_height or base.depth_height),
        fps=int(args.fps or base.fps),
    )


def open_profile(profile: StreamProfile, depth_filters: bool = True) -> RealSenseGrabber:
    cam = RealSenseGrabber(
        width=profile.color_width,
        height=profile.color_height,
        fps=profile.fps,
        depth_width=profile.depth_width,
        depth_height=profile.depth_height,
        with_depth=True,
        depth_filters=depth_filters,
    )
    print(
        f"[OK] 取流 {profile.name}: "
        f"彩色 {profile.color_width}x{profile.color_height} "
        f"深度 {profile.depth_width}x{profile.depth_height} @{profile.fps}fps"
    )
    if depth_filters:
        print("[OK] RealSense 深度滤波已开启（spatial + temporal）")
    return cam


def open_from_namespace(args: argparse.Namespace) -> tuple[RealSenseGrabber, StreamProfile]:
    profile = resolve_profile(args)
    depth_filters = not getattr(args, "no_rs_depth_filter", False)
    return open_profile(profile, depth_filters=depth_filters), profile


def intrinsics_dict(cam: RealSenseGrabber) -> dict[str, float]:
    k = cam.camera_matrix
    d = cam.dist_coeffs
    return {
        "fx": float(k[0, 0]),
        "fy": float(k[1, 1]),
        "cx": float(k[0, 2]),
        "cy": float(k[1, 2]),
        "k1": float(d[0]),
        "k2": float(d[1]),
        "p1": float(d[2]),
        "p2": float(d[3]),
        "k3": float(d[4]),
        "depth_scale": float(cam.depth_scale),
    }


def print_stream_info(cam: RealSenseGrabber, profile: StreamProfile) -> None:
    intr = intrinsics_dict(cam)
    print(
        f"[内参] fx={intr['fx']:.3f} fy={intr['fy']:.3f} "
        f"cx={intr['cx']:.3f} cy={intr['cy']:.3f}"
    )
    print(
        f"[畸变] k1={intr['k1']:.6f} k2={intr['k2']:.6f} "
        f"p1={intr['p1']:.6f} p2={intr['p2']:.6f} k3={intr['k3']:.6f}"
    )
    print(f"[深度] depth_scale={intr['depth_scale']:.6f}")
    u, v = profile.color_center
    print(f"[参考] 画面中心像素约 ({u}, {v})")
