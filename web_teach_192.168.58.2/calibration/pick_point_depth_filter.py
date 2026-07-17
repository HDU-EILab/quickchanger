#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选点深度稳定：空间鲁棒中值 + 时序平滑 + 移动前多帧采样。"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from fr10_handeye_calibrate import RealSenseGrabber


class DepthDisplaySmoother:
    """GUI 深度读数时序中值平滑。"""

    def __init__(self, window: int = 9) -> None:
        self.window = max(1, int(window))
        self._buf: list[float] = []
        self._last_key: tuple[float, float, float] | None = None

    def reset(self) -> None:
        self._buf.clear()
        self._last_key = None

    def update(self, u: int, v: int, z_m: float) -> float:
        return self.update_key((float(u), float(v), 0.0), z_m)

    def update_key(self, key: tuple[float, float, float], z_m: float) -> float:
        if self._last_key != key:
            self.reset()
            self._last_key = key
        if z_m <= 1e-6:
            return z_m
        self._buf.append(float(z_m))
        if len(self._buf) > self.window:
            self._buf.pop(0)
        return float(np.median(self._buf))


@dataclass
class CircleFaceDepthResult:
    z_m: float
    ok: bool
    spread_mm: float
    n_pixels: int
    area_frac: float
    peak_mm: float = 0.0
    mode_gap_mm: float = 0.0
    jitter_mm: float = 0.0
    inlier_mask: np.ndarray | None = None


class CircleFaceDepthSmoother:
    """圆面主区域深度：时序中值 + 抖动统计 + OK 迟滞。"""

    def __init__(
        self,
        window: int = 15,
        ok_frames: int = 4,
        bad_frames: int = 2,
        max_jitter_mm: float = 12.0,
    ) -> None:
        self.window = max(3, int(window))
        self.ok_frames = max(1, int(ok_frames))
        self.bad_frames = max(1, int(bad_frames))
        self.max_jitter_mm = float(max_jitter_mm)
        self._key: tuple[float, float, float] | None = None
        self._buf: list[float] = []
        self._ok_streak = 0
        self._bad_streak = 0
        self._stable = False

    def reset(self) -> None:
        self._key = None
        self._buf.clear()
        self._ok_streak = 0
        self._bad_streak = 0
        self._stable = False

    def update(self, key: tuple[float, float, float], result: CircleFaceDepthResult) -> CircleFaceDepthResult:
        if self._key != key:
            self.reset()
            self._key = key

        frame_ok = result.ok and result.z_m > 1e-6
        if frame_ok:
            self._buf.append(result.z_m)
            if len(self._buf) > self.window:
                self._buf.pop(0)
            self._ok_streak += 1
            self._bad_streak = 0
        else:
            self._bad_streak += 1
            self._ok_streak = 0

        if self._ok_streak >= self.ok_frames:
            self._stable = True
        elif self._bad_streak >= self.bad_frames:
            self._stable = False

        if self._buf:
            z_smooth = float(np.median(self._buf))
            jitter_mm = (
                float(np.std(self._buf) * 1000.0) if len(self._buf) >= 3 else 999.0
            )
        else:
            z_smooth = result.z_m
            jitter_mm = 999.0

        display_ok = (
            self._stable
            and len(self._buf) >= self.ok_frames
            and jitter_mm <= self.max_jitter_mm
        )

        return replace(
            result,
            z_m=z_smooth,
            jitter_mm=jitter_mm,
            ok=display_ok,
        )


def add_depth_stabilize_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument(
        "--depth-radius",
        type=int,
        default=5,
        help="空间邻域半径 px（中值+离群剔除，默认 5）",
    )
    ap.add_argument(
        "--depth-frames",
        type=int,
        default=7,
        help="按 m 移动前连续采样帧数，取时序中值（默认 7）",
    )
    ap.add_argument(
        "--depth-smooth-frames",
        type=int,
        default=15,
        help="GUI 圆面深度时序平滑窗口帧数（默认 15）",
    )
    ap.add_argument(
        "--no-rs-depth-filter",
        action="store_true",
        help="关闭 RealSense spatial/temporal 硬件滤波",
    )
    ap.add_argument(
        "--max-depth-spread-mm",
        type=float,
        default=45.0,
        help="圆内主区域/邻域深度采样最大允许极差 mm，超过则视为不可用（默认 45）",
    )
    ap.add_argument(
        "--depth-bin-mm",
        type=float,
        default=25.0,
        help="圆面深度直方图分箱宽度 mm（固定锚点，默认 25）",
    )
    ap.add_argument(
        "--depth-inner-frac",
        type=float,
        default=0.30,
        help="忽略圆心孔区域：仅在内半径=该比例×外半径 的环形内采样（默认 0.30）",
    )
    ap.add_argument(
        "--max-face-jitter-mm",
        type=float,
        default=12.0,
        help="GUI 判定稳定 OK 的时序抖动上限 mm（默认 12）",
    )


def analyze_circle_face_depth(
    depth_np: np.ndarray,
    cu: float,
    cv: float,
    radius_px: float,
    depth_scale: float,
    depth_min_reliable_m: float,
    bin_mm: float = 25.0,
    max_spread_mm: float = 45.0,
    min_pixels: int = 40,
    min_area_frac: float = 0.12,
    inner_frac: float = 0.30,
    return_mask: bool = False,
) -> CircleFaceDepthResult:
    """环形区域内固定锚点分箱，取占面积最大的深度簇。"""
    empty = CircleFaceDepthResult(0.0, False, 999.0, 0, 0.0)
    if radius_px < 8.0:
        return empty

    h, w = depth_np.shape
    r = int(max(4, round(radius_px)))
    cu_i, cv_i = int(round(cu)), int(round(cv))
    u0, u1 = max(0, cu_i - r), min(w, cu_i + r + 1)
    v0, v1 = max(0, cv_i - r), min(h, cv_i + r + 1)

    yy, xx = np.mgrid[v0:v1, u0:u1]
    dist = np.sqrt((xx.astype(np.float64) - cu) ** 2 + (yy.astype(np.float64) - cv) ** 2)
    outer_r = float(radius_px)
    inner_r = outer_r * max(0.0, min(0.85, float(inner_frac)))
    annulus_mask = (dist <= outer_r) & (dist >= inner_r)

    patch = depth_np[v0:v1, u0:u1].astype(np.float64) * depth_scale
    valid_mask = annulus_mask & (patch > 1e-6)
    valid = patch[valid_mask]
    annulus_area = int(annulus_mask.sum())

    if valid.size < min_pixels or annulus_area < 30:
        return replace(empty, n_pixels=int(valid.size))

    bin_w = max(0.008, float(bin_mm) * 1e-3)
    ref_m = float(np.median(valid))
    anchor = np.floor(ref_m / bin_w) * bin_w
    edges = anchor + np.arange(-4.0, 5.5, 1.0) * bin_w
    counts, edges = np.histogram(valid, bins=edges)
    if counts.size == 0 or counts.max() == 0:
        return replace(empty, n_pixels=int(valid.size))

    order = np.argsort(counts)[::-1]
    peak_idx = int(order[0])
    lo, hi = float(edges[peak_idx]), float(edges[peak_idx + 1])
    peak_mm = 0.5 * (lo + hi)

    in_peak_local = valid_mask & (patch >= lo) & (patch < hi)
    if peak_idx == counts.size - 1:
        in_peak_local = valid_mask & (patch >= lo) & (patch <= hi)
    in_peak_vals = patch[in_peak_local]
    if in_peak_vals.size == 0:
        in_peak_vals = valid[np.abs(valid - peak_mm) <= bin_w * 0.55]
        in_peak_local = valid_mask & (np.abs(patch - peak_mm) <= bin_w * 0.55)

    z = robust_median_patch(in_peak_vals) if in_peak_vals.size else 0.0
    spread_mm = (
        float((in_peak_vals.max() - in_peak_vals.min()) * 1000.0)
        if in_peak_vals.size
        else 999.0
    )
    area_frac = float(in_peak_vals.size / max(1, annulus_area))

    mode_gap_mm = 0.0
    if len(order) >= 2 and counts[order[1]] >= counts[order[0]] * 0.35:
        c0 = 0.5 * (edges[order[0]] + edges[order[0] + 1])
        c1 = 0.5 * (edges[order[1]] + edges[order[1] + 1])
        mode_gap_mm = abs(c0 - c1) * 1000.0

    ok = (
        z >= depth_min_reliable_m
        and spread_mm <= max_spread_mm
        and mode_gap_mm <= max_spread_mm
        and area_frac >= min_area_frac
        and in_peak_vals.size >= min_pixels
    )

    inlier_mask = None
    if return_mask:
        inlier_mask = np.zeros((h, w), dtype=bool)
        inlier_mask[v0:v1, u0:u1] = in_peak_local

    return CircleFaceDepthResult(
        z_m=z,
        ok=ok,
        spread_mm=spread_mm if ok else max(spread_mm, mode_gap_mm),
        n_pixels=int(in_peak_vals.size),
        area_frac=area_frac,
        peak_mm=peak_mm * 1000.0,
        mode_gap_mm=mode_gap_mm,
        inlier_mask=inlier_mask,
    )


def sample_circle_dominant_depth_m(
    depth_np: np.ndarray,
    cu: float,
    cv: float,
    radius_px: float,
    depth_scale: float,
    depth_min_reliable_m: float,
    bin_mm: float = 25.0,
    max_spread_mm: float = 45.0,
    min_pixels: int = 40,
    min_area_frac: float = 0.12,
    inner_frac: float = 0.30,
) -> tuple[float, bool, float, int, float]:
    """在拟合圆环形区域内取占面积最大深度簇的中值。

    返回 (深度 m, 是否可靠, 主区域极差 mm, 主区域像素数, 主区域占环形面积比例)。
    """
    r = analyze_circle_face_depth(
        depth_np,
        cu,
        cv,
        radius_px,
        depth_scale,
        depth_min_reliable_m,
        bin_mm=bin_mm,
        max_spread_mm=max_spread_mm,
        min_pixels=min_pixels,
        min_area_frac=min_area_frac,
        inner_frac=inner_frac,
    )
    return r.z_m, r.ok, r.spread_mm, r.n_pixels, r.area_frac


def acquire_stable_circle_dominant_depth(
    cam: RealSenseGrabber,
    cu: float,
    cv: float,
    radius_px: float,
    depth_scale: float,
    depth_min_reliable_m: float,
    depth_frames: int,
    max_spread_mm: float,
    assume_depth_mm: float | None,
    bin_mm: float = 25.0,
    inner_frac: float = 0.30,
) -> tuple[float, str, np.ndarray, float]:
    n = max(1, int(depth_frames))
    frame_samples: list[float] = []
    last_depth: np.ndarray | None = None
    last_spread = 999.0
    for i in range(n):
        _, last_depth = cam.read()
        r = analyze_circle_face_depth(
            last_depth,
            cu,
            cv,
            radius_px,
            depth_scale,
            depth_min_reliable_m,
            bin_mm=bin_mm,
            max_spread_mm=max_spread_mm,
            inner_frac=inner_frac,
        )
        last_spread = r.spread_mm
        if r.z_m > 1e-6 and r.ok:
            frame_samples.append(r.z_m)
        if n > 1 and i + 1 < n:
            time.sleep(0.02)
    if last_depth is None:
        raise RuntimeError("未获取到深度帧")
    if assume_depth_mm is not None and assume_depth_mm > 0 and not frame_samples:
        return assume_depth_mm * 1e-3, "assume", last_depth, 0.0
    if not frame_samples:
        return 0.0, "invalid", last_depth, last_spread
    z = float(np.median(frame_samples))
    tag = "dominant_stable" if n > 1 else "dominant"
    if z < depth_min_reliable_m:
        tag = "dominant_close"
    return z, tag, last_depth, last_spread


def robust_median_patch(valid_m: np.ndarray) -> float:
    if valid_m.size == 0:
        return 0.0
    if valid_m.size < 4:
        return float(np.median(valid_m))
    med = float(np.median(valid_m))
    mad = float(np.median(np.abs(valid_m - med)))
    tol = max(3.0 * mad, 0.002)
    kept = valid_m[np.abs(valid_m - med) <= tol]
    if kept.size == 0:
        kept = valid_m
    return float(np.median(kept))


def median_depth_robust_m(
    depth_np: np.ndarray,
    u: int,
    v: int,
    depth_scale: float,
    depth_min_reliable_m: float,
    radius: int = 5,
    max_radius: int = 25,
) -> tuple[float, bool]:
    h, w = depth_np.shape
    for r in range(radius, max_radius + 1, 2):
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = depth_np[v0:v1, u0:u1].astype(np.float64) * depth_scale
        valid = patch[patch > 1e-6]
        if valid.size == 0:
            continue
        z = robust_median_patch(valid)
        return z, z >= depth_min_reliable_m
    return 0.0, False


def acquire_stable_depth(
    cam: RealSenseGrabber,
    u: int,
    v: int,
    depth_scale: float,
    depth_min_reliable_m: float,
    depth_frames: int,
    depth_radius: int,
    assume_depth_mm: float | None,
) -> tuple[float, str, np.ndarray]:
    n = max(1, int(depth_frames))
    samples: list[float] = []
    last_depth: np.ndarray | None = None
    for i in range(n):
        _, last_depth = cam.read()
        z, _ = median_depth_robust_m(
            last_depth,
            u,
            v,
            depth_scale,
            depth_min_reliable_m,
            radius=depth_radius,
        )
        if z > 1e-6:
            samples.append(z)
        if n > 1 and i + 1 < n:
            time.sleep(0.02)
    if last_depth is None:
        raise RuntimeError("未获取到深度帧")

    if assume_depth_mm is not None and assume_depth_mm > 0 and not samples:
        return assume_depth_mm * 1e-3, "assume", last_depth

    if not samples:
        return 0.0, "invalid", last_depth

    z = float(np.median(samples))
    if n > 1:
        tag = "measured_stable" if z >= depth_min_reliable_m else "measured_close"
    else:
        tag = "measured" if z >= depth_min_reliable_m else "measured_close"
    return z, tag, last_depth
