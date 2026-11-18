#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HD ArUco quickchanger move.

Detect an ArUco marker center and move in two stages:
1. Move to the target with final deadzone Z + prelift, with fixed RPY.
2. Move to the final target with the configured final deadzone.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
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
from pick_point_aruco_move_hd import (  # noqa: E402
    DICT_NAMES,
    detect_aruco_center,
    draw_aruco_overlay,
)
from pick_point_gui_zoom import ZoomPanView, handle_zoom_keys  # noqa: E402

ppm.ACTIVE_DIR = _CALIB_DIR / "sessions" / "active"
ACTIVE_DIR = ppm.ACTIVE_DIR

GUI_HELP = "auto ArUco -> m two-stage move | r refresh | q quit"
DEFAULT_CONFIG_PATH = _DEV_DIR / "quickchanger_config.json"
QUICKCHANGER_BAD_DEPTH_CODE = 213


def load_quickchanger_config(path: Path | str | None):
    if path is None:
        return {}
    cfg_path = Path(path)
    if not cfg_path.is_file():
        print(f"[WARN] quickchanger config 不存在，跳过: {cfg_path}")
        return {}
    with cfg_path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{cfg_path}: config 须为 JSON object")
    print(f"[OK] 已加载 quickchanger config: {cfg_path}")
    return data


def config_vec(data: dict, key: str, n: int) -> list[float] | None:
    raw = data.get(key)
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != n:
        raise ValueError(f"quickchanger_config.json: {key} 须为长度 {n} 的数组")
    return [float(x) for x in raw]


def config_id_float_list_map(data: dict, key: str) -> dict[int, list[float]]:
    raw = data.get(key)
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"quickchanger_config.json: {key} 须为 object")
    out: dict[int, list[float]] = {}
    for id_key, values in raw.items():
        try:
            marker_id = int(id_key)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"quickchanger_config.json: {key} 的键须为 ArUco ID"
            ) from exc
        if not isinstance(values, (list, tuple)):
            raise ValueError(
                f"quickchanger_config.json: {key}.{id_key} 须为数字数组"
            )
        out[marker_id] = [float(x) for x in values]
    return out


def cli_has(argv: list[str], name: str) -> bool:
    return any(arg == name or arg.startswith(name + "=") for arg in argv)


def apply_quickchanger_config(args: argparse.Namespace, argv: list[str] | None = None):
    argv = [] if argv is None else list(argv)
    cfg = load_quickchanger_config(getattr(args, "config", DEFAULT_CONFIG_PATH))
    stage2 = config_vec(cfg, "stage2_deadzone_mm", 3)
    stage1 = config_vec(cfg, "stage1_deadzone_mm", 3)
    move_vel = float(cfg.get("quickchanger_move_vel", getattr(args, "vel", 10.0)))

    if stage2 is not None and not cli_has(argv, "--deadzone-mm"):
        args.deadzone_mm = stage2
    if stage1 is not None and not cli_has(argv, "--prelift-mm"):
        args.stage1_deadzone_mm = stage1
    else:
        args.stage1_deadzone_mm = None
    if not cli_has(argv, "--vel"):
        args.vel = move_vel
    if not cli_has(argv, "--stage1-vel"):
        args.stage1_vel = float(cfg.get("stage1_move_vel", move_vel))
    if not cli_has(argv, "--stage2-vel"):
        args.stage2_vel = float(cfg.get("stage2_move_vel", move_vel))
    args.stage1_retry_deadzone_z_by_id = config_id_float_list_map(
        cfg, "stage1_retry_deadzone_z_by_id"
    )
    args.quickchanger_depth_min_mm = float(
        cfg.get("quickchanger_depth_min_mm", 180.0)
    )
    args.quickchanger_depth_max_mm = float(
        cfg.get("quickchanger_depth_max_mm", 800.0)
    )
    args.quickchanger_depth_retry_frames = int(
        cfg.get("quickchanger_depth_retry_frames", 6)
    )
    return cfg


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Quickchanger ArUco two-stage move")
    add_pick_point_arguments(ap)
    ap.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"quickchanger 配置文件，默认 {DEFAULT_CONFIG_PATH.name}",
    )
    ap.add_argument(
        "--dictionary",
        choices=tuple(DICT_NAMES.keys()),
        default="4x4_50",
        help="ArUco dictionary；4x4 默认 4x4_50",
    )
    ap.add_argument("--aruco-id", type=int, default=1, help="目标 ArUco ID，默认 1")
    ap.add_argument(
        "--prelift-mm",
        type=float,
        default=40.0,
        help="未配置 stage1_deadzone_mm 时，第一段相对最终点的 Z 偏移，默认 40 mm",
    )
    ap.add_argument(
        "--skip-stage1",
        action="store_true",
        help="调试用：跳过阶段1点位移动，只先原地调整 RPY，再直接移动到阶段2最终点",
    )
    ap.add_argument(
        "--stage1-vel",
        type=float,
        default=None,
        help="阶段1 MoveL 速度；不传则读取 config stage1_move_vel",
    )
    ap.add_argument(
        "--stage2-vel",
        type=float,
        default=None,
        help="阶段2 MoveL 速度；不传则读取 config stage2_move_vel",
    )
    ap.add_argument(
        "--final-rpy",
        nargs=3,
        type=float,
        metavar=("RX", "RY", "RZ"),
        default=[-180.0, 0.0, 45.0],
        help="两段移动固定 TCP RPY，默认 -180 0 45",
    )
    ap.add_argument(
        "--test-image",
        type=Path,
        default=None,
        help="只对静态图片做检测；不连接相机、不移动机械臂",
    )
    return ap


def detect_or_report(args: argparse.Namespace, color: np.ndarray):
    result = detect_aruco_center(
        color,
        marker_id=args.aruco_id,
        dictionary_name=args.dictionary,
    )
    out = ACTIVE_DIR / "quickchanger_aruco_hd_last.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), draw_aruco_overlay(color, result))
    print(f"[INFO] 标注图 {out}")
    if not result.ok or result.center_uv is None:
        print(f"[FAIL] {result.message}; rejected={result.rejected_count}")
        return None
    u, v = result.center_uv
    print(
        f"[OK] ArUco id={result.marker_id} center=({u:.1f}, {v:.1f}) "
        f"perimeter={result.perimeter_px:.1f}px; {result.message}"
    )
    return result


def run_detection_only(args: argparse.Namespace) -> int:
    bgr = cv2.imread(str(args.test_image))
    if bgr is None:
        print(f"[FAIL] cannot read {args.test_image}")
        return 1
    result = detect_or_report(args, bgr)
    return 0 if result is not None else 1


def target_base_from_aruco(
    args: argparse.Namespace,
    result,
    depth_np: np.ndarray,
    cam,
    T_c2g: np.ndarray,
    rob,
) -> tuple[np.ndarray, float, str, list[float]]:
    if result.center_uv is None:
        raise RuntimeError("ArUco center missing")
    u, v = result.center_uv
    ui, vi = int(round(u)), int(round(v))

    fx, fy = cam.camera_matrix[0, 0], cam.camera_matrix[1, 1]
    cx, cy = cam.camera_matrix[0, 2], cam.camera_matrix[1, 2]
    _, tcp_pose = ppm.read_robot_state(rob, "tcp")
    T_g2b = ppm.pose_to_matrix(tcp_pose)
    assume_mm = args.assume_depth_mm
    depth_m, depth_tag, depth_np = marker_depth_m(args, result, depth_np, cam)
    if depth_m <= 1e-6 and assume_mm is not None and assume_mm > 0:
        depth_m = float(assume_mm) * 1e-3
        depth_tag = "assume"
    if depth_m <= 1e-6:
        raise ValueError("Quickchanger ArUco 标记区域无有效深度")

    p_cam = ppm.deproject_pixel(ui, vi, depth_m, fx, fy, cx, cy)
    T_c2b = T_g2b @ T_c2g
    p_base = (T_c2b @ np.append(p_cam, 1.0))[:3]
    depth_mm = depth_m * 1000.0
    min_mm = float(getattr(args, "quickchanger_depth_min_mm", 180.0))
    max_mm = float(getattr(args, "quickchanger_depth_max_mm", 800.0))
    if not (min_mm <= depth_mm <= max_mm):
        raise ValueError(
            f"Quickchanger 深度异常: {depth_mm:.1f} mm，"
            f"允许范围 [{min_mm:.1f}, {max_mm:.1f}] mm；"
            "拒绝生成机械臂目标"
        )
    return p_base, depth_m, depth_tag, tcp_pose


def marker_depth_points(result) -> list[tuple[str, int, int]]:
    if result.center_uv is None:
        return []
    c = np.array(result.center_uv, dtype=np.float64)
    pts: list[tuple[str, np.ndarray]] = [("center", c)]
    if result.corners is not None:
        corners = result.corners.reshape(4, 2).astype(np.float64)
        for i, p in enumerate(corners):
            pts.append((f"corner{i}_in", c + 0.65 * (p - c)))
        for i in range(4):
            p = 0.5 * (corners[i] + corners[(i + 1) % 4])
            pts.append((f"edge{i}_mid", p))
    out: list[tuple[str, int, int]] = []
    seen = set()
    for label, p in pts:
        uv = (int(round(float(p[0]))), int(round(float(p[1]))))
        if uv in seen:
            continue
        seen.add(uv)
        out.append((label, uv[0], uv[1]))
    return out


def marker_depth_m(args, result, depth_np: np.ndarray, cam) -> tuple[float, str, np.ndarray]:
    min_m = float(getattr(args, "quickchanger_depth_min_mm", 180.0)) * 1e-3
    max_m = float(getattr(args, "quickchanger_depth_max_mm", 800.0)) * 1e-3
    radius = int(getattr(args, "depth_radius", 5))
    frames = max(1, int(getattr(args, "depth_frames", 7)))
    points = marker_depth_points(result)
    samples: list[tuple[str, float]] = []
    rejected: list[tuple[str, float]] = []
    last_depth = depth_np

    for frame_idx in range(frames):
        if frame_idx > 0:
            _, last_depth = cam.read()
        for label, u, v in points:
            z, _ = ppm.median_depth_m(
                last_depth,
                u,
                v,
                cam.depth_scale,
                radius=radius,
                max_radius=max(radius, 11),
            )
            if z <= 1e-6:
                continue
            if min_m <= z <= max_m:
                samples.append((label, float(z)))
            else:
                rejected.append((label, float(z)))
        if frames > 1 and frame_idx + 1 < frames:
            time.sleep(0.02)

    if samples:
        values = np.array([z for _, z in samples], dtype=np.float64)
        depth_m = float(np.median(values))
        compact = [(label, round(z * 1000.0, 1)) for label, z in samples[:12]]
        print(
            f"[INFO] ArUco 标记多点深度样本 valid={len(samples)} "
            f"median={depth_m * 1000.0:.1f}mm samples={compact}"
        )
        if rejected:
            rej = [(label, round(z * 1000.0, 1)) for label, z in rejected[:8]]
            print(f"[INFO] ArUco 标记深度已过滤异常样本 n={len(rejected)} samples={rej}")
        return depth_m, "aruco_marker_median", last_depth

    if rejected:
        values = [round(z * 1000.0, 1) for _, z in rejected[:12]]
        print(
            f"[WARN] ArUco 标记深度全部超出允许范围 "
            f"[{min_m * 1000.0:.1f}, {max_m * 1000.0:.1f}]mm samples={values}"
        )
    return 0.0, "invalid", last_depth


def move_to_quickchanger_target(
    args: argparse.Namespace,
    result,
    color: np.ndarray,
    depth_np: np.ndarray,
    cam,
    T_c2g: np.ndarray,
    rob,
) -> int:
    try:
        p_base, depth_m, depth_tag, tcp_pose = target_base_from_aruco(
            args, result, depth_np, cam, T_c2g, rob
        )
    except ValueError as exc:
        print(f"[FAIL] {exc}")
        return QUICKCHANGER_BAD_DEPTH_CODE
    u, v = result.center_uv
    ui, vi = int(round(u)), int(round(v))

    # Re-read TCP just before planning, matching pick_point_move's behavior.
    _, tcp_pose = ppm.read_robot_state(rob, "tcp")
    p_base, _ = ppm.apply_field_offset(
        p_base, enabled=not getattr(args, "no_field_offset", False)
    )
    height_mm = ppm.effective_height_mm(args.height_mm)
    normal_mode = args.normal
    n_plane = None
    if args.approach == "standoff":
        fx, fy = cam.camera_matrix[0, 0], cam.camera_matrix[1, 1]
        cx, cy = cam.camera_matrix[0, 2], cam.camera_matrix[1, 2]
        T_g2b = ppm.pose_to_matrix(tcp_pose)
        T_c2b = T_g2b @ T_c2g
        normal_mode, n_plane = ppm.resolve_normal_mode(
            args,
            p_base,
            tcp_pose,
            depth_np,
            ui,
            vi,
            T_c2b,
            fx,
            fy,
            cx,
            cy,
            cam.depth_scale,
            depth_tag,
        )

    raw_pose, target_info = ppm.compute_tcp_target(
        p_base,
        tcp_pose,
        height_mm,
        args.approach,
        normal_mode,
        n_plane,
        T_c2g,
        args.height_ref,
        args.height_bias_mm,
        down_rpy=[float(x) for x in args.final_rpy],
    )
    raw_pose[3:6] = [float(x) for x in args.final_rpy]

    deadzone_mm = ppm.resolve_handeye_deadzone_mm(args)
    final_deadzone_mm = list(deadzone_mm)
    if getattr(args, "stage1_deadzone_mm", None) is not None:
        pre_deadzone_mm = [float(x) for x in args.stage1_deadzone_mm]
    else:
        pre_deadzone_mm = list(deadzone_mm)
        pre_deadzone_mm[2] += float(args.prelift_mm)

    final_pose = ppm.apply_deadzone_to_pose_mm(raw_pose, final_deadzone_mm)
    stage1_candidates: list[tuple[list[float], list[float], str]] = []
    pre_pose = ppm.apply_deadzone_to_pose_mm(raw_pose, pre_deadzone_mm)
    stage1_candidates.append((pre_pose, list(pre_deadzone_mm), "primary"))
    retry_by_id = getattr(args, "stage1_retry_deadzone_z_by_id", {}) or {}
    for z_mm in retry_by_id.get(int(args.aruco_id), []):
        retry_deadzone = list(pre_deadzone_mm)
        retry_deadzone[2] = float(z_mm)
        retry_pose = ppm.apply_deadzone_to_pose_mm(raw_pose, retry_deadzone)
        if any(
            ppm.tcp_move_delta_mm(retry_pose, pose) < 0.5
            for pose, _, _ in stage1_candidates
        ):
            continue
        stage1_candidates.append(
            (retry_pose, retry_deadzone, f"retry_z={float(z_mm):.1f}")
        )
    args.quickchanger_last_move_info = {
        "raw_pose": [float(x) for x in raw_pose[:6]],
        "stage1_pose": [float(x) for x in pre_pose[:6]],
        "stage1_deadzone_mm": [float(x) for x in pre_deadzone_mm],
        "final_pose": [float(x) for x in final_pose[:6]],
        "target_base_mm": [float(x) * 1000.0 for x in p_base[:3]],
        "depth_mm": float(depth_m) * 1000.0,
        "depth_tag": str(depth_tag),
    }

    print(f"[INFO] Quickchanger ArUco 像素 ({ui}, {vi})")
    print(
        f"[INFO] 深度 {depth_m * 1000.0:.1f} mm ({depth_tag})"
        + (" [多帧稳定]" if depth_tag == "measured_stable" else "")
        + (" [手动/回退]" if depth_tag == "assume" else "")
    )
    print("[INFO] 物体目标点 S(基座 mm):", [round(x * 1000.0, 2) for x in p_base])
    print("[INFO] 当前 TCP mm/rpy:", [round(x, 2) for x in tcp_pose[:6]])
    print("[INFO] 固定 RPY:", [round(float(x), 2) for x in args.final_rpy])
    print(
        f"[INFO] 目标生成复用 pick_point_move: approach={args.approach}, "
        f"height={height_mm:.1f} mm, height-ref={args.height_ref}, "
        f"height-bias={args.height_bias_mm:.1f} mm"
    )
    print("[INFO] deadzone 前目标 TCP mm/rpy:", [round(x, 2) for x in raw_pose[:6]])
    print("[INFO] 阶段1 deadzone_mm（基座 +X/+Y/+Z）:", pre_deadzone_mm)
    if len(stage1_candidates) > 1:
        retry_notes = [
            f"{label}:{[round(x, 2) for x in dz]}"
            for _, dz, label in stage1_candidates[1:]
        ]
        print("[INFO] 阶段1 112 重试候选 deadzone_mm:", retry_notes)
    print("[INFO] 阶段2 deadzone_mm（基座 +X/+Y/+Z）:", final_deadzone_mm)
    print(
        f"[INFO] 阶段1目标：原始目标 + deadzone {pre_deadzone_mm}:",
        [round(x, 2) for x in pre_pose[:6]],
    )
    print("[INFO] 阶段2目标：最终点:", [round(x, 2) for x in final_pose[:6]])
    print(f"[INFO] 阶段1预计位移 {ppm.tcp_move_delta_mm(tcp_pose, pre_pose):.1f} mm")
    print(f"[INFO] 阶段2 Z 位移 {abs(float(pre_pose[2]) - float(final_pose[2])):.1f} mm")
    stage1_vel = float(getattr(args, "stage1_vel", None) or args.vel)
    stage2_vel = float(getattr(args, "stage2_vel", None) or args.vel)
    print(f"[INFO] 阶段速度: stage1={stage1_vel:.1f}, stage2={stage2_vel:.1f}")
    if args.skip_stage1:
        orient_pose = list(tcp_pose[:6])
        orient_pose[3:6] = [float(x) for x in args.final_rpy]
        print("[DEBUG] --skip-stage1 已启用：跳过阶段1点位移动")
        print("[DEBUG] 姿态调整目标 TCP mm/rpy:", [round(x, 2) for x in orient_pose[:6]])

    vis = color.copy()
    cv2.drawMarker(vis, (ui, vi), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    out = ACTIVE_DIR / "quickchanger_pick_last.jpg"
    cv2.imwrite(str(out), vis)
    print(f"[INFO] 标注图已保存 {out}")

    if args.dry_run:
        print("[dry-run] 未下发 MoveL")
        return 0

    if args.skip_stage1:
        print("[INFO] 调试阶段：原地调整到固定 RPY，XYZ 不变")
        err = ppm.move_tcp(rob, orient_pose, stage1_vel, args.tool, args.user)
        if err != 0:
            print(f"[FAIL] 调试姿态调整 MoveL 错误码 {err}")
            return err
    else:
        err = 112
        selected_stage1_pose = None
        selected_stage1_deadzone = None
        for idx, (candidate_pose, candidate_deadzone, label) in enumerate(
            stage1_candidates, start=1
        ):
            print(
                f"[INFO] 阶段1/2 尝试 {idx}/{len(stage1_candidates)} "
                f"{label}: deadzone={candidate_deadzone}, "
                f"target={[round(x, 2) for x in candidate_pose[:6]]}"
            )
            err = ppm.move_tcp(rob, candidate_pose, stage1_vel, args.tool, args.user)
            if err == 0:
                selected_stage1_pose = candidate_pose
                selected_stage1_deadzone = candidate_deadzone
                break
            if err != 112:
                print(f"[FAIL] 阶段1 MoveL 错误码 {err}")
                return err
            print(f"[WARN] 阶段1候选 {label} 逆解无解/MoveL 112，尝试下一个候选")
        if selected_stage1_pose is None:
            print("[FAIL] 阶段1所有候选均 MoveL 112，未进入阶段2")
            return err
        pre_pose = selected_stage1_pose
        args.quickchanger_last_move_info["stage1_pose"] = [
            float(x) for x in selected_stage1_pose[:6]
        ]
        args.quickchanger_last_move_info["stage1_deadzone_mm"] = [
            float(x) for x in selected_stage1_deadzone
        ]

    print("[INFO] 阶段2/2 移动到最终点")
    err = ppm.move_tcp(rob, final_pose, stage2_vel, args.tool, args.user)
    if err != 0:
        print(f"[FAIL] 阶段2 MoveL 错误码 {err}")
        return err

    after = ppm._normalize_pose(rob.robot.GetActualTCPPose(1))
    args.quickchanger_last_move_info["actual_after_pose"] = [float(x) for x in after[:6]]
    print("[OK] 到位 TCP mm/rpy:", [round(x, 2) for x in after[:6]])
    return 0


def run_once_from_frame(
    args: argparse.Namespace,
    color: np.ndarray,
    depth: np.ndarray,
    cam,
    T_c2g: np.ndarray,
    rob,
) -> int:
    result = detect_or_report(args, color)
    if result is None:
        return 1
    return move_to_quickchanger_target(args, result, color, depth, cam, T_c2g, rob)


def run_gui(args: argparse.Namespace, T_c2g: np.ndarray, rob, cam) -> int:
    view: ZoomPanView | None = None
    win = f"Quickchanger | {GUI_HELP}"
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
            ret = move_to_quickchanger_target(args, result, color, depth, cam, T_c2g, rob)
            if ret == 0:
                cv2.destroyAllWindows()
                return 0
            print("[INFO] 本次移动未成功，可重新对准后按 m 再试")
        elif key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return 0


def main() -> int:
    argv = sys.argv[1:]
    args = build_arg_parser().parse_args(argv)
    apply_quickchanger_config(args, argv)

    if args.test_image is not None:
        return run_detection_only(args)

    T_c2g = ppm.load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper（来自 calibration/sessions/active/）")

    cam, profile = open_from_namespace(args)
    print_stream_info(cam, profile)
    rob = _load_robot(args.host)
    try:
        use_gui = (not args.headless) and gui_available()
        if use_gui:
            print(
                "GUI：自动检测 ArUco → m 两段移动 → r 重检 → q 退出 "
                f"（{profile.color_width}x{profile.color_height}）"
            )
            return run_gui(args, T_c2g, rob, cam)

        color, depth = cam.read()
        return run_once_from_frame(args, color, depth, cam, T_c2g, rob)
    finally:
        cam.stop()


if __name__ == "__main__":
    raise SystemExit(main())
