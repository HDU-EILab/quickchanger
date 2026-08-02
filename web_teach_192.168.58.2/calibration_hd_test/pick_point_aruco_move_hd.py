#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HD ArUco center pick.

This is intentionally a thin variant of pick_point_move_hd.py:
ArUco detection only replaces the manually clicked pixel (u, v). All depth,
hand-eye, target pose, IK, MoveL, field offset, and GUI zoom behavior stays in
pick_point_move.run_once / the existing HD pick-point stack.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

_DEV_DIR = Path(__file__).resolve().parent
from _paths import pick_point_calib_dir  # noqa: E402

_CALIB_DIR = pick_point_calib_dir()
sys.path.insert(0, str(_CALIB_DIR))
sys.path.insert(0, str(_DEV_DIR))

import pick_point_move as ppm  # noqa: E402
from fr10_handeye_calibrate import _load_robot, gui_available  # noqa: E402
from hd_pick_args import add_pick_point_arguments  # noqa: E402
from hd_realsense import open_from_namespace, print_stream_info  # noqa: E402
from pick_point_gui_zoom import (  # noqa: E402
    ZoomPanView,
    handle_zoom_keys,
)

ppm.ACTIVE_DIR = _CALIB_DIR / "sessions" / "active"
ACTIVE_DIR = ppm.ACTIVE_DIR

ARUCO_GUI_HELP = "auto ArUco center -> m move | r refresh | q quit"


@dataclass
class ArucoResult:
    ok: bool
    message: str
    center_uv: tuple[float, float] | None = None
    corners: np.ndarray | None = None
    marker_id: int | None = None
    seen_ids: tuple[int, ...] = ()
    perimeter_px: float = 0.0
    rejected_count: int = 0


DICT_NAMES = {
    "4x4_50": "DICT_4X4_50",
    "4x4_100": "DICT_4X4_100",
    "4x4_250": "DICT_4X4_250",
    "4x4_1000": "DICT_4X4_1000",
}


def _require_aruco():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "当前 OpenCV 缺少 cv2.aruco；请安装 opencv-contrib-python 或使用含 aruco 的 OpenCV。"
        )
    return cv2.aruco


def get_dictionary(name: str):
    aruco = _require_aruco()
    const_name = DICT_NAMES.get(name, name)
    if not hasattr(aruco, const_name):
        raise ValueError(f"未知 ArUco dictionary: {name}")
    return aruco.getPredefinedDictionary(getattr(aruco, const_name))


def make_detector(dictionary):
    aruco = _require_aruco()
    params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 6
    params.minMarkerPerimeterRate = 0.015
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.08
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(dictionary, params), None
    return None, params


def detect_with_image(img: np.ndarray, dictionary, detector, params):
    aruco = _require_aruco()
    if detector is not None:
        return detector.detectMarkers(img)
    return aruco.detectMarkers(img, dictionary, parameters=params)


def preprocess_variants(bgr: np.ndarray) -> list[tuple[str, np.ndarray]]:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    variants: list[tuple[str, np.ndarray]] = [("gray", gray)]
    variants.append(("equalized", cv2.equalizeHist(gray)))
    variants.append(
        ("clahe", cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray))
    )
    variants.append(("inverted", cv2.bitwise_not(gray)))
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    adaptive = cv2.adaptiveThreshold(
        blur,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )
    variants.append(("adaptive", adaptive))
    variants.append(("adaptive_inverted", cv2.bitwise_not(adaptive)))
    return variants


def marker_perimeter(corners: np.ndarray) -> float:
    pts = corners.reshape(4, 2).astype(np.float64)
    return float(sum(np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)))


def detect_aruco_center(
    bgr: np.ndarray,
    *,
    marker_id: int,
    dictionary_name: str,
) -> ArucoResult:
    dictionary = get_dictionary(dictionary_name)
    detector, params = make_detector(dictionary)

    best: ArucoResult | None = None
    total_rejected = 0
    seen_ids: set[int] = set()
    for variant_name, img in preprocess_variants(bgr):
        corners_list, ids, rejected = detect_with_image(img, dictionary, detector, params)
        total_rejected += 0 if rejected is None else len(rejected)
        if ids is None or len(corners_list) == 0:
            continue
        for corners, mid in zip(corners_list, ids.reshape(-1).astype(int)):
            seen_ids.add(int(mid))
            if int(mid) != int(marker_id):
                continue
            pts = corners.reshape(4, 2).astype(np.float64)
            center = pts.mean(axis=0)
            perim = marker_perimeter(corners)
            cand = ArucoResult(
                ok=True,
                message=f"detected via {variant_name}",
                center_uv=(float(center[0]), float(center[1])),
                corners=pts,
                marker_id=int(mid),
                seen_ids=tuple(sorted(seen_ids)),
                perimeter_px=perim,
                rejected_count=total_rejected,
            )
            if best is None or cand.perimeter_px > best.perimeter_px:
                best = cand

    if best is not None:
        best.rejected_count = total_rejected
        best.seen_ids = tuple(sorted(seen_ids))
        return best
    seen = tuple(sorted(seen_ids))
    detail = f"；本帧仅看到其它 id={list(seen)}" if seen else ""
    return ArucoResult(
        ok=False,
        message=f"未检测到 dictionary={dictionary_name} id={marker_id}{detail}",
        seen_ids=seen,
        rejected_count=total_rejected,
    )


def draw_aruco_overlay(bgr: np.ndarray, result: ArucoResult) -> np.ndarray:
    out = bgr.copy()
    if not result.ok or result.corners is None or result.center_uv is None:
        cv2.putText(
            out,
            result.message,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )
        return out

    pts = result.corners.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, (0, 255, 255), 2)
    u, v = result.center_uv
    cv2.drawMarker(
        out,
        (int(round(u)), int(round(v))),
        (0, 255, 0),
        cv2.MARKER_CROSS,
        28,
        2,
    )
    cv2.putText(
        out,
        f"id={result.marker_id} center=({u:.1f},{v:.1f})",
        (int(round(u)) + 12, max(24, int(round(v)) - 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="HD ArUco(4x4 id=1) 中心识别 -> MoveL")
    add_pick_point_arguments(ap)
    ap.add_argument(
        "--dictionary",
        choices=tuple(DICT_NAMES.keys()),
        default="4x4_50",
        help="ArUco dictionary；常见 dictionary4x4 默认对应 4x4_50",
    )
    ap.add_argument("--aruco-id", type=int, default=1)
    ap.add_argument(
        "--test-image",
        type=Path,
        default=None,
        help="只对静态图片做检测；不连接相机、不移动机械臂",
    )
    return ap


def run_detection_only(args: argparse.Namespace) -> int:
    bgr = cv2.imread(str(args.test_image))
    if bgr is None:
        print(f"[FAIL] cannot read {args.test_image}")
        return 1
    result = detect_aruco_center(
        bgr,
        marker_id=args.aruco_id,
        dictionary_name=args.dictionary,
    )
    out = ACTIVE_DIR / "pick_point_aruco_hd_test.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), draw_aruco_overlay(bgr, result))
    if not result.ok or result.center_uv is None:
        print(f"[FAIL] {result.message}; rejected={result.rejected_count}")
        print(f"[INFO] 标注图 {out}")
        return 1
    u, v = result.center_uv
    print(
        f"[OK] ArUco id={result.marker_id} center=({u:.2f}, {v:.2f}) "
        f"perimeter={result.perimeter_px:.1f}px; {result.message}"
    )
    print(f"[INFO] 标注图 {out}")
    return 0


def aruco_to_run_once(
    args: argparse.Namespace,
    color: np.ndarray,
    depth: np.ndarray,
    cam,
    T_c2g: np.ndarray,
    rob,
) -> int:
    result = detect_aruco_center(
        color,
        marker_id=args.aruco_id,
        dictionary_name=args.dictionary,
    )
    out = ACTIVE_DIR / "pick_point_aruco_hd_last.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), draw_aruco_overlay(color, result))
    print(f"[INFO] 标注图 {out}")
    if not result.ok or result.center_uv is None:
        print(f"[FAIL] {result.message}; rejected={result.rejected_count}")
        return 1

    u, v = result.center_uv
    ui, vi = int(round(u)), int(round(v))
    print(
        f"[OK] ArUco id={result.marker_id} center=({u:.1f}, {v:.1f}) "
        f"perimeter={result.perimeter_px:.1f}px; {result.message}"
    )
    print(f"[INFO] 用 ArUco 中心替代手动点击像素 ({ui}, {vi})")
    return ppm.run_once(args, ui, vi, color, depth, cam, T_c2g, rob)


def run_gui(args: argparse.Namespace, T_c2g: np.ndarray, rob, cam) -> int:
    view: ZoomPanView | None = None
    win = f"ArUco pick | {ARUCO_GUI_HELP}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        color, depth = cam.read()
        h, w = color.shape[:2]
        if view is None or view.img_w != w or view.img_h != h:
            view = ZoomPanView(w, h)

        result = detect_aruco_center(
            color,
            marker_id=args.aruco_id,
            dictionary_name=args.dictionary,
        )
        vis = draw_aruco_overlay(color, result)
        display = view.render(vis)
        view.draw_hud(display)
        cv2.imshow(win, display)

        key = cv2.waitKey(1) & 0xFF
        if handle_zoom_keys(view, key):
            continue
        if key == ord("r"):
            continue
        if key == ord("m"):
            if not result.ok:
                print(f"[skip] {result.message}")
                continue
            ret = aruco_to_run_once(args, color, depth, cam, T_c2g, rob)
            if ret == 0:
                cv2.destroyAllWindows()
                return 0
            print("[INFO] 本次移动未成功，可重新对准后按 m 再试")
        elif key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return 0


def main() -> int:
    args = build_arg_parser().parse_args()

    if args.test_image is not None:
        return run_detection_only(args)

    T_c2g = ppm.load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper（来自 calibration/sessions/active/）")

    cam, profile = open_from_namespace(args)
    print_stream_info(cam, profile)
    rob = _load_robot(args.host)
    try:
        if args.pixel:
            u, v = int(args.pixel[0]), int(args.pixel[1])
            color, depth = cam.read()
            print(f"[INFO] 使用手动像素覆盖 ({u}, {v})，未运行 ArUco 检测")
            return ppm.run_once(args, u, v, color, depth, cam, T_c2g, rob)

        use_gui = (not args.headless) and gui_available()
        if use_gui:
            print(
                "GUI：自动检测 ArUco → m 移动 → r 重检 → q 退出 "
                f"（{profile.color_width}x{profile.color_height}）"
            )
            return run_gui(args, T_c2g, rob, cam)

        color, depth = cam.read()
        return aruco_to_run_once(args, color, depth, cam, T_c2g, rob)
    finally:
        cam.stop()


if __name__ == "__main__":
    raise SystemExit(main())
