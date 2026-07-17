#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RGB 点击选点 → 深度反投影 → 手眼标定变换 → MoveL。

默认 --approach tcp_above：Tool#0 TCP 在点击点正上方（基座同 XY，Z+H mm）。
默认先原地转至法兰水平朝下，再平移到目标上方（--no-orient-down-first 可关闭）。

用法：
  ./run_pick_point_move.sh
  ./run_pick_point_move.sh --height-mm 50
  ./run_pick_point_move.sh --pixel 640 360 --height-mm 100 --assume-depth-mm 250
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

_CALIB_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_CALIB_DIR))
from fr10_handeye_calibrate import (  # noqa: E402
    RealSenseGrabber,
    _load_robot,
    _normalize_pose,
    gui_available,
    pose_to_matrix,
    read_robot_state,
)
from pick_point_gui_zoom import (  # noqa: E402
    GUI_HELP,
    ZoomPanView,
    handle_zoom_keys,
    make_mouse_callback,
)
from pick_point_depth_filter import (  # noqa: E402
    DepthDisplaySmoother,
    acquire_stable_depth,
    add_depth_stabilize_args,
    median_depth_robust_m,
)

ACTIVE_DIR = _CALIB_DIR / "sessions" / "active"
FR10_HOST = "192.168.58.2"
# D435 可靠深度一般 ≥ 20 cm，更近时常显示 0
DEPTH_MIN_RELIABLE_M = 0.20
MIN_MOVE_MM = 3.0
# 默认抬起高度；--height-mm 为在其之上的增量（例：默认 100，--height-mm 50 → 150）
BASE_HEIGHT_MM = 100.0
# Tool#0 法兰水平、TCP Z 指向基座 -Z（朝下）；rz 默认保持当前腕角
FLANGE_DOWN_RX_DEG = 180.0
FLANGE_DOWN_RY_DEG = 0.0
RPY_ORIENT_TOL_DEG = 1.0
IK_TIMEOUT_S = 5.0


def effective_height_mm(height_delta_mm: float) -> float:
    return BASE_HEIGHT_MM + float(height_delta_mm)


def resolve_flange_down_rpy(tcp_pose_mm_deg: list[float], args) -> list[float]:
    """法兰/TCP 水平面向下：rx/ry 固定，rz 默认保持当前。"""
    custom = getattr(args, "down_rpy", None)
    if custom and len(custom) == 3:
        return [float(custom[0]), float(custom[1]), float(custom[2])]
    return [FLANGE_DOWN_RX_DEG, FLANGE_DOWN_RY_DEG, float(tcp_pose_mm_deg[5])]


def rpy_close(a: list[float], b: list[float], tol_deg: float = RPY_ORIENT_TOL_DEG) -> bool:
    return all(abs(float(a[i]) - float(b[i])) <= tol_deg for i in range(3))


def add_orient_arguments(ap: argparse.ArgumentParser) -> None:
    ap.add_argument(
        "--orient-down-first",
        action="store_true",
        default=True,
        help="移动前先原地转至法兰水平朝下（默认开启）",
    )
    ap.add_argument(
        "--no-orient-down-first",
        action="store_false",
        dest="orient_down_first",
        help="跳过朝下姿态，仅平移到目标上方（旧行为）",
    )
    ap.add_argument(
        "--down-rpy",
        nargs=3,
        type=float,
        metavar=("RX", "RY", "RZ"),
        default=None,
        help=f"朝下姿态 deg，默认 ({FLANGE_DOWN_RX_DEG}, {FLANGE_DOWN_RY_DEG}, 当前rz)",
    )


def load_field_offset_mm() -> list[float]:
    """读取手眼现场死区修正 mm（基座 +X/+Y/+Z，加在最终 TCP 目标点上）。"""
    for p in (ACTIVE_DIR / "field_offset.json", _CALIB_DIR / "field_offset.json"):
        if p.is_file():
            data = json.loads(p.read_text(encoding="utf-8"))
            raw = data.get("deadzone_mm")
            if raw is None:
                raw = data.get("base_offset_mm", [0.0, 0.0, 0.0])
            if not isinstance(raw, (list, tuple)):
                raise ValueError(f"{p}: deadzone_mm/base_offset_mm 须为长度 3 的数组")
            return [float(raw[i]) if i < len(raw) else 0.0 for i in range(3)]
    return [0.0, 0.0, 0.0]


def resolve_handeye_deadzone_mm(args) -> list[float]:
    if getattr(args, "deadzone_mm", None) is not None and len(args.deadzone_mm) == 3:
        return [float(x) for x in args.deadzone_mm]
    if getattr(args, "no_field_offset", False):
        return [0.0, 0.0, 0.0]
    return load_field_offset_mm()


def apply_deadzone_to_pose_mm(pose6: list[float], off_mm: list[float]) -> list[float]:
    """在最终 TCP 目标位姿的 XYZ 上叠加基座系死区修正。"""
    out = list(pose6)
    for i in range(3):
        out[i] += float(off_mm[i])
    return out


def apply_field_offset(
    point_base_m: np.ndarray, enabled: bool = True
) -> tuple[np.ndarray, list[float]]:
    """兼容旧接口：死区已改到最终 TCP，此处不再改表面点。"""
    if not enabled:
        return point_base_m, [0.0, 0.0, 0.0]
    return point_base_m, [0.0, 0.0, 0.0]


def load_T_cam2gripper() -> np.ndarray:
    for p in (ACTIVE_DIR / "handeye_result.json", _CALIB_DIR / "handeye_result.json"):
        if p.is_file():
            data = json.loads(p.read_text(encoding="utf-8"))
            if data.get("mode") != "eye_in_hand":
                raise RuntimeError("需要 eye_in_hand 标定结果")
            return np.array(data["T_cam2gripper"], dtype=np.float64)
    raise FileNotFoundError("未找到 handeye_result.json")


def deproject_pixel(
    u: int, v: int, depth_m: float, fx: float, fy: float, cx: float, cy: float
) -> np.ndarray:
    if depth_m <= 1e-6:
        raise ValueError(f"像素 ({u},{v}) 无有效深度")
    return np.array(
        [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m],
        dtype=np.float64,
    )


def median_depth_m(
    depth_np: np.ndarray,
    u: int,
    v: int,
    depth_scale: float,
    radius: int = 5,
    max_radius: int = 25,
) -> tuple[float, bool]:
    """返回 (深度 m, 是否可靠)。过近时扩大邻域仍可能为 0。"""
    return median_depth_robust_m(
        depth_np, u, v, depth_scale, DEPTH_MIN_RELIABLE_M, radius, max_radius
    )


def resolve_depth_m(
    depth_np: np.ndarray,
    u: int,
    v: int,
    depth_scale: float,
    assume_depth_mm: float | None,
) -> tuple[float, str]:
    z, reliable = median_depth_m(depth_np, u, v, depth_scale)
    if z > 1e-6:
        tag = "measured" if reliable else "measured_close"
        return z, tag
    if assume_depth_mm is not None and assume_depth_mm > 0:
        return assume_depth_mm * 1e-3, "assume"
    raise ValueError(
        f"像素 ({u},{v}) 无有效深度（D435 通常需 ≥{DEPTH_MIN_RELIABLE_M*1000:.0f} mm）。"
        f"请拉远相机，或加 --assume-depth-mm <距离> 手动指定相机到物体距离"
    )


def estimate_plane_normal_base(
    depth_np: np.ndarray,
    u: int,
    v: int,
    T_cam2base: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float,
    window: int = 9,
) -> np.ndarray | None:
    """在点击点附近拟合局部平面，返回基座标系单位法向。"""
    h, w = depth_np.shape
    pts = []
    for dv in range(-window, window + 1, 2):
        for du in range(-window, window + 1, 2):
            uu, vv = u + du, v + dv
            if not (0 <= uu < w and 0 <= vv < h):
                continue
            z, _ = median_depth_m(depth_np, uu, vv, depth_scale, radius=1, max_radius=7)
            if z <= 1e-6:
                continue
            p_cam = deproject_pixel(uu, vv, z, fx, fy, cx, cy)
            p_b = (T_cam2base @ np.append(p_cam, 1.0))[:3]
            pts.append(p_b)
    if len(pts) < 6:
        return None
    P = np.array(pts)
    c = P.mean(axis=0)
    _, _, vh = np.linalg.svd(P - c)
    n = vh[-1]
    n = n / (np.linalg.norm(n) + 1e-12)
    return n


def cam_point_to_base(
    u: int,
    v: int,
    depth_np: np.ndarray,
    depth_scale: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    T_gripper2base: np.ndarray,
    T_cam2gripper: np.ndarray,
    assume_depth_mm: float | None = None,
) -> tuple[np.ndarray, float, str]:
    z, z_tag = resolve_depth_m(depth_np, u, v, depth_scale, assume_depth_mm)
    p_cam = deproject_pixel(u, v, z, fx, fy, cx, cy)
    T_c2b = T_gripper2base @ T_cam2gripper
    return (T_c2b @ np.append(p_cam, 1.0))[:3], z, z_tag


def cam_point_to_base_for_move(
    args: argparse.Namespace,
    u: int,
    v: int,
    depth_np: np.ndarray,
    cam: RealSenseGrabber,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    T_g2b: np.ndarray,
    T_c2g: np.ndarray,
    assume_depth_mm: float | None,
) -> tuple[np.ndarray, float, str, np.ndarray]:
    depth_radius = int(getattr(args, "depth_radius", 5))
    depth_frames = int(getattr(args, "depth_frames", 7))
    if depth_frames > 1 and assume_depth_mm is None:
        print(f"[INFO] 深度多帧采样 n={depth_frames} …")
        depth_m, depth_tag, depth_np = acquire_stable_depth(
            cam,
            u,
            v,
            cam.depth_scale,
            DEPTH_MIN_RELIABLE_M,
            depth_frames,
            depth_radius,
            assume_depth_mm,
        )
        if depth_m <= 1e-6:
            raise ValueError(
                f"像素 ({u},{v}) 无有效深度（D435 通常需 ≥{DEPTH_MIN_RELIABLE_M*1000:.0f} mm）。"
                f"请拉远相机，或加 --assume-depth-mm <距离>"
            )
        p_cam = deproject_pixel(u, v, depth_m, fx, fy, cx, cy)
        T_c2b = T_g2b @ T_c2g
        p_base = (T_c2b @ np.append(p_cam, 1.0))[:3]
        return p_base, depth_m, depth_tag, depth_np
    p_base, depth_m, depth_tag = cam_point_to_base(
        u, v, depth_np, cam.depth_scale, fx, fy, cx, cy, T_g2b, T_c2g, assume_depth_mm
    )
    return p_base, depth_m, depth_tag, depth_np


def _pick_normal(
    point_base_m: np.ndarray,
    tcp_pose_mm_deg: list[float],
    normal_mode: str,
    normal_base: np.ndarray | None,
) -> np.ndarray:
    tcp_cur_m = np.array(tcp_pose_mm_deg[:3], dtype=np.float64) * 1e-3
    if normal_mode == "base_z":
        n = np.array([0.0, 0.0, 1.0])
        sign = 1.0 if tcp_cur_m[2] >= point_base_m[2] else -1.0
        return sign * n
    if normal_mode == "camera":
        vec = tcp_cur_m - point_base_m
        return vec / (np.linalg.norm(vec) + 1e-12)
    if normal_mode == "plane":
        if normal_base is None:
            raise RuntimeError("plane normal missing")
        sign = 1.0 if np.dot(tcp_cur_m - point_base_m, normal_base) >= 0.0 else -1.0
        return sign * normal_base
    if normal_mode == "fixed":
        if normal_base is None:
            raise RuntimeError("fixed normal missing")
        n = np.array(normal_base, dtype=np.float64)
        return n / (np.linalg.norm(n) + 1e-12)
    raise ValueError(normal_mode)


def resolve_normal_mode(
    args: argparse.Namespace,
    point_base_m: np.ndarray,
    tcp_pose_mm_deg: list[float],
    depth_np: np.ndarray,
    u: int,
    v: int,
    T_c2b: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float,
    depth_tag: str,
) -> tuple[str, np.ndarray | None]:
    """plane 失败或深度不可靠时自动回退 camera。"""
    mode = args.normal
    n_plane = None
    if mode != "plane":
        return mode, n_plane

    n_plane = estimate_plane_normal_base(
        depth_np, u, v, T_c2b, fx, fy, cx, cy, depth_scale
    )
    if n_plane is not None and depth_tag not in ("assume", "measured_close"):
        return "plane", n_plane

    fallback = "camera" if args.normal_fallback == "camera" else "base_z"
    reason = "深度点不足" if n_plane is None else f"深度不可靠({depth_tag})"
    print(f"[WARN] 平面法向估计失败({reason})，回退 --normal {fallback}")
    return fallback, None


def compute_tcp_target(
    point_base_m: np.ndarray,
    tcp_pose_mm_deg: list[float],
    height_mm: float,
    approach: str,
    normal_mode: str,
    normal_base: np.ndarray | None,
    T_cam2gripper: np.ndarray,
    height_ref: str = "camera",
    height_bias_mm: float = 0.0,
    down_rpy: list[float] | None = None,
) -> tuple[list[float], dict]:
    eff_h_m = (height_mm - height_bias_mm) * 1e-3
    target = list(tcp_pose_mm_deg)

    if approach == "tcp_above":
        # TCP 在基座系中与点击点同 XY，沿 +Z 抬高 H（正上方）
        tcp_pos_m = np.array(
            [point_base_m[0], point_base_m[1], point_base_m[2] + eff_h_m],
            dtype=np.float64,
        )
        target[0:3] = [float(v * 1000.0) for v in tcp_pos_m]
        if down_rpy is not None:
            target[3:6] = [float(x) for x in down_rpy]
        info = {
            "approach": "tcp_above",
            "point_base_mm": (point_base_m * 1000.0).tolist(),
            "height_mm": height_mm,
            "height_bias_mm": height_bias_mm,
            "vertical_axis": "base_z",
            "target_tcp_mm": target[:3],
            "target_rpy_deg": target[3:6],
        }
        return target, info

    # standoff：沿法向抬起（旧逻辑，需 --approach standoff）
    n = _pick_normal(point_base_m, tcp_pose_mm_deg, normal_mode, normal_base)
    R_g2b = pose_to_matrix(tcp_pose_mm_deg)[:3, :3]
    t_c2g = T_cam2gripper[:3, 3]
    cam_offset_base = R_g2b @ t_c2g

    if height_ref == "camera":
        p_cam_target = point_base_m + (height_mm * 1e-3) * n
        tcp_pos_m = p_cam_target - cam_offset_base
    else:
        tcp_pos_m = point_base_m + (height_mm * 1e-3) * n

    if abs(height_bias_mm) > 1e-6:
        tcp_pos_m -= (height_bias_mm * 1e-3) * n

    target[0:3] = [float(tcp_pos_m[i] * 1000.0) for i in range(3)]
    info = {
        "approach": "standoff",
        "point_base_mm": (point_base_m * 1000.0).tolist(),
        "normal_base": n.tolist(),
        "height_mm": height_mm,
        "height_ref": height_ref,
        "height_bias_mm": height_bias_mm,
        "cam_offset_base_mm": (cam_offset_base * 1000.0).tolist(),
        "normal_mode": normal_mode,
    }
    return target, info


def _timeout_handler(_signum, _frame):
    raise TimeoutError


def ik_ok(rob, pose_mm_deg: list[float]) -> bool | None:
    """法奥错误码 112 = 逆运动学无解（位姿不可达）。"""
    pose = [round(float(x), 2) for x in pose_mm_deg[:6]]
    print(f"[ROBOT] GetInverseKin start pose={pose}", flush=True)
    old_handler = signal.getsignal(signal.SIGALRM)
    try:
        signal.signal(signal.SIGALRM, _timeout_handler)
        signal.setitimer(signal.ITIMER_REAL, IK_TIMEOUT_S)
        ret = rob.GetInverseKin(0, pose_mm_deg, -1)
    except TimeoutError:
        print(
            f"[FAIL] GetInverseKin 超时 {IK_TIMEOUT_S:.1f}s。"
            "通常是机器人 SDK/RPC 连接被占用或控制器未响应；"
            "请检查 20003/20004 连接和 fr10_web_teach/pick_point 残留进程。",
            flush=True,
        )
        return None
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, old_handler)
    print(f"[ROBOT] GetInverseKin raw={ret}", flush=True)
    return isinstance(ret, (list, tuple)) and len(ret) > 0 and int(ret[0]) == 0


def shrink_height_until_ik(
    args: argparse.Namespace,
    point_base_m: np.ndarray,
    tcp_pose_mm_deg: list[float],
    height_mm: float,
    rob,
    T_cam2gripper: np.ndarray,
    normal_mode: str,
    n_vec: np.ndarray | None,
    step_mm: float = 10.0,
    min_height_mm: float = 30.0,
) -> tuple[float, list[float]] | None:
    """逐步降低高度直到逆解有解。"""
    h = float(height_mm)
    while h >= min_height_mm:
        target, _ = compute_tcp_target(
            point_base_m,
            tcp_pose_mm_deg,
            h,
            args.approach,
            "fixed" if args.approach == "standoff" else normal_mode,
            n_vec if args.approach == "standoff" else None,
            T_cam2gripper,
            args.height_ref,
            args.height_bias_mm,
        )
        if ik_ok(rob, target) is True:
            return h, target
        h -= step_mm
    return None


def tcp_move_delta_mm(cur: list[float], target: list[float]) -> float:
    return float(
        np.linalg.norm(np.array(target[:3]) - np.array(cur[:3]))
    )


def move_tcp(rob, pose_mm_deg: list[float], vel: float, tool: int, user: int) -> int:
    ok = ik_ok(rob, pose_mm_deg)
    if ok is None:
        return 12003
    if not ok:
        return 112
    ret = rob.MoveL(desc_pos=pose_mm_deg, tool=tool, user=user, vel=vel, ovl=100.0)
    if isinstance(ret, (list, tuple)) and len(ret) > 0:
        return int(ret[0])
    return int(ret)


def run_once(
    args: argparse.Namespace,
    u: int,
    v: int,
    color: np.ndarray,
    depth_np: np.ndarray,
    cam: RealSenseGrabber,
    T_c2g: np.ndarray,
    rob,
) -> int:
    fx, fy = cam.camera_matrix[0, 0], cam.camera_matrix[1, 1]
    cx, cy = cam.camera_matrix[0, 2], cam.camera_matrix[1, 2]
    _, tcp_pose = read_robot_state(rob, "tcp")
    T_g2b = pose_to_matrix(tcp_pose)
    height_mm = effective_height_mm(args.height_mm)

    assume_mm = args.assume_depth_mm
    if (
        assume_mm is None
        and args.approach == "standoff"
        and args.height_ref == "camera"
    ):
        assume_mm = height_mm

    try:
        p_base, depth_m, depth_tag, depth_np = cam_point_to_base_for_move(
            args,
            u,
            v,
            depth_np,
            cam,
            fx,
            fy,
            cx,
            cy,
            T_g2b,
            T_c2g,
            assume_mm,
        )
    except ValueError as e:
        print(f"[FAIL] {e}")
        return 1

    p_base, _ = apply_field_offset(
        p_base, enabled=not getattr(args, "no_field_offset", False)
    )
    deadzone_mm = resolve_handeye_deadzone_mm(args)

    normal_mode = args.normal
    n_plane = None
    if args.approach == "standoff":
        T_c2b = T_g2b @ T_c2g
        normal_mode, n_plane = resolve_normal_mode(
            args,
            p_base,
            tcp_pose,
            depth_np,
            u,
            v,
            T_c2b,
            fx,
            fy,
            cx,
            cy,
            cam.depth_scale,
            depth_tag,
        )

    # 移动前再读一次姿态，减少选点与按键之间臂已移动带来的偏差
    _, tcp_pose = read_robot_state(rob, "tcp")
    down_rpy = None
    if args.approach == "tcp_above" and getattr(args, "orient_down_first", True):
        down_rpy = resolve_flange_down_rpy(tcp_pose, args)
    target, info = compute_tcp_target(
        p_base,
        tcp_pose,
        height_mm,
        args.approach,
        normal_mode,
        n_plane,
        T_c2g,
        args.height_ref,
        args.height_bias_mm,
        down_rpy=down_rpy,
    )
    info["height_mm_delta"] = args.height_mm
    info["height_mm_base"] = BASE_HEIGHT_MM
    if any(abs(x) > 1e-6 for x in deadzone_mm):
        target = apply_deadzone_to_pose_mm(target, deadzone_mm)
        info["handeye_deadzone_mm"] = deadzone_mm
        info["target_tcp_mm_after_deadzone"] = target[:3]
    else:
        info["handeye_deadzone_mm"] = [0.0, 0.0, 0.0]
    if args.approach == "standoff":
        info["normal_mode_requested"] = args.normal
    n_vec = (
        np.array(info["normal_base"], dtype=np.float64)
        if "normal_base" in info
        else None
    )

    ik_state = ik_ok(rob, target)
    if ik_state is None:
        print("[FAIL] 未下发运动：GetInverseKin 超时")
        return 12003
    if not ik_state:
        print(
            f"[WARN] 目标位姿逆解无解(112)，高度 {height_mm} mm 可能超出工作空间"
        )
        if args.auto_reduce_height:
            shrunk = shrink_height_until_ik(
                args,
                p_base,
                tcp_pose,
                height_mm,
                rob,
                T_c2g,
                normal_mode,
                n_vec,
            )
            if shrunk is None:
                print(
                    "[FAIL] 已尝试降低高度仍无解。请减小 --height-mm 增量、"
                    "换选点位置，或先用示教器移到更近处"
                )
                return 112
            new_h, target = shrunk
            print(
                f"[INFO] 已自动将高度从 {height_mm} mm 调整为 {new_h:.0f} mm"
            )
            info["height_mm"] = new_h
        else:
            print(
                "[FAIL] MoveL 错误码 112（逆解无解）。"
                "可加 --auto-reduce-height 自动降低高度，或减小 --height-mm"
            )
            return 112

    delta = tcp_move_delta_mm(tcp_pose, target)
    print(f"[INFO] 选中像素 ({u}, {v})")
    print(
        f"[INFO] 深度 {depth_m*1000:.1f} mm ({depth_tag})"
        + (" [多帧稳定]" if depth_tag == "measured_stable" else "")
        + (" [过近，读数可能不准]" if depth_tag == "measured_close" else "")
        + (" [手动/回退]" if depth_tag == "assume" else "")
    )
    if info.get("approach") == "tcp_above":
        orient_note = ""
        if down_rpy is not None:
            orient_note = f"；姿态朝下 RPY={[round(x, 2) for x in down_rpy]}"
        print(
            f"[INFO] 模式 tcp_above: TCP XY=点击点，Z=表面Z+{height_mm} mm"
            f"（基准 {BASE_HEIGHT_MM} + 增量 {args.height_mm}，"
            f"bias {args.height_bias_mm} mm，基座 +Z 正上方）{orient_note}"
        )
    else:
        print(
            f"[INFO] 模式 standoff: height-ref={args.height_ref}，"
            f"H={height_mm} mm（基准 {BASE_HEIGHT_MM} + 增量 {args.height_mm}），"
            f"bias={args.height_bias_mm} mm"
        )
        print(
            "[INFO] 相机相对TCP偏移(基座 mm):",
            [round(x, 1) for x in info.get("cam_offset_base_mm", [])],
        )
        if info.get("normal_mode_requested") != info.get("normal_mode"):
            print(
                f"[INFO] 法向: {info.get('normal_mode_requested')} → {info.get('normal_mode')}"
            )
        print("[INFO] 抬起法向:", [round(x, 4) for x in info.get("normal_base", [])])
    if any(abs(x) > 1e-6 for x in deadzone_mm):
        print("[INFO] 手眼死区修正 deadzone_mm（基座 +X/+Y/+Z）:", deadzone_mm)
    print("[INFO] 物体表面点 S(基座 mm):", [round(x, 2) for x in info["point_base_mm"]])
    print("[INFO] 当前 TCP mm:", [round(x, 2) for x in tcp_pose[:3]])
    print("[INFO] 目标 TCP mm:", [round(x, 2) for x in target[:3]])
    print(f"[INFO] 预计位移 {delta:.1f} mm")

    if delta < MIN_MOVE_MM:
        print(
            f"[WARN] 目标与当前 TCP 距离仅 {delta:.1f} mm < {MIN_MOVE_MM} mm，"
            "控制器可能几乎不动。请检查 depth 是否为 0、height-ref 是否与深度读数一致"
        )
        if not args.force_move:
            print("[FAIL] 未下发运动（加 --force-move 可强制执行）")
            return 1

    vis = color.copy()
    cv2.drawMarker(vis, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    out = ACTIVE_DIR / "pick_point_last.jpg"
    cv2.imwrite(str(out), vis)
    print(f"[INFO] 标注图已保存 {out}")

    if args.dry_run:
        print("[dry-run] 未下发 MoveL")
        if down_rpy is not None and getattr(args, "orient_down_first", True):
            orient_pose = list(tcp_pose[:3]) + down_rpy
            print(f"[dry-run] 阶段1 原地朝下 MoveL {orient_pose}")
            print(f"[dry-run] 阶段2 平移到上方 MoveL {target}")
        return 0

    if (
        down_rpy is not None
        and getattr(args, "orient_down_first", True)
        and not rpy_close(tcp_pose[3:6], down_rpy)
    ):
        orient_pose = list(tcp_pose[:3]) + down_rpy
        print(
            f"[INFO] 阶段1/2 原地调整朝下 RPY={[round(x, 2) for x in down_rpy]} "
            f"（当前 RPY={[round(x, 2) for x in tcp_pose[3:6]]}）"
        )
        err = move_tcp(rob, orient_pose, args.vel, args.tool, args.user)
        if err != 0:
            if err == 112:
                print(
                    "[FAIL] 阶段1 MoveL 112：朝下姿态逆解无解，"
                    "可试 --down-rpy 或 --no-orient-down-first"
                )
            else:
                print(f"[FAIL] 阶段1 MoveL 错误码 {err}")
            return err
        time.sleep(0.4)
        _, tcp_pose = read_robot_state(rob, "tcp")

    print(f"[INFO] 阶段2/2 平移到目标上方 {[round(x, 2) for x in target[:3]]}")
    err = move_tcp(rob, target, args.vel, args.tool, args.user)
    if err != 0:
        if err == 112:
            print("[FAIL] MoveL 错误码 112：逆运动学无解，目标位姿不可达")
        else:
            print(f"[FAIL] MoveL 错误码 {err}")
        return err
    time.sleep(0.5)
    after = _normalize_pose(rob.robot.GetActualTCPPose(1))
    print("[OK] 到位 TCP mm:", [round(x, 2) for x in after[:3]])
    return 0


def run_gui(args: argparse.Namespace, T_c2g: np.ndarray, rob, cam: RealSenseGrabber) -> int:
    state = {"uv": None, "color": None, "depth": None}
    view: ZoomPanView | None = None
    depth_radius = int(getattr(args, "depth_radius", 5))
    smoother = DepthDisplaySmoother(
        window=int(getattr(args, "depth_smooth_frames", 9))
    )

    def on_image_click(u: float, v: float) -> None:
        state["uv"] = (int(round(u)), int(round(v)))
        smoother.reset()

    win = f"Pick point | {GUI_HELP}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        color, depth = cam.read()
        state["color"] = color
        state["depth"] = depth
        h, w = color.shape[:2]
        if view is None or view.img_w != w or view.img_h != h:
            view = ZoomPanView(w, h)
            cv2.setMouseCallback(win, make_mouse_callback(view, on_image_click))

        vis = color.copy()
        if state["uv"]:
            u, v = state["uv"]
            cv2.drawMarker(vis, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
            z_raw, reliable = median_depth_m(
                depth, u, v, cam.depth_scale, radius=depth_radius
            )
            z = smoother.update(u, v, z_raw)
            if z <= 1e-6:
                label = f"({u},{v}) depth=无效(过近?≥{DEPTH_MIN_RELIABLE_M*1000:.0f}mm)"
                color_txt = (0, 0, 255)
            elif not reliable:
                label = f"({u},{v}) depth~{z*1000:.0f}mm 过近"
                color_txt = (0, 165, 255)
            else:
                label = f"({u},{v}) depth~{z*1000:.0f}mm"
                color_txt = (0, 255, 0)
            cv2.putText(
                vis,
                label,
                (u + 12, max(24, v - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color_txt,
                2,
            )
        display = view.render(vis)
        view.draw_hud(display)
        cv2.imshow(win, display)
        key = cv2.waitKey(1) & 0xFF
        if handle_zoom_keys(view, key):
            continue
        if key == ord("r"):
            state["uv"] = None
            smoother.reset()
        elif key == ord("m"):
            if state["uv"] is None:
                print("[skip] 请先左键点击选点")
                continue
            u, v = state["uv"]
            ret = run_once(args, u, v, color, depth, cam, T_c2g, rob)
            if ret == 0:
                cv2.destroyAllWindows()
                return 0
            print("[INFO] 本次移动未成功，可 r 刷新画面后重试")
        elif key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="RGB 点击选点并移到上方")
    ap.add_argument("--host", default=FR10_HOST)
    ap.add_argument("--pixel", nargs=2, type=int, metavar=("U", "V"))
    ap.add_argument(
        "--approach",
        choices=("tcp_above", "standoff"),
        default="tcp_above",
        help="tcp_above=TCP 在点击点正上方(基座Z) H mm；standoff=沿法向抬起(旧)",
    )
    ap.add_argument(
        "--height-mm",
        type=float,
        default=0.0,
        help=(
            f"在默认高度 {BASE_HEIGHT_MM:.0f} mm 之上的增量 mm"
            f"（不传则 {BASE_HEIGHT_MM:.0f}；例 --height-mm 50 → {BASE_HEIGHT_MM + 50:.0f}）"
        ),
    )
    ap.add_argument(
        "--height-ref",
        choices=("camera", "tcp"),
        default="camera",
        help="camera=相机光心到物体距离（默认，与深度读数一致）；tcp=Tool0 到物体距离",
    )
    ap.add_argument(
        "--height-bias-mm",
        type=float,
        default=0.0,
        help="沿法向额外降低 TCP 的距离 mm（现场微调，如 25）",
    )
    ap.add_argument(
        "--assume-depth-mm",
        type=float,
        default=None,
        help="深度为 0 时手动指定相机到物体距离 mm；camera 模式下默认用基准+增量总高度",
    )
    ap.add_argument(
        "--no-field-offset",
        action="store_true",
        help="不应用 field_offset.json 中的手眼死区修正",
    )
    ap.add_argument(
        "--deadzone-mm",
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DZ"),
        default=None,
        help="本次运行覆盖死区修正 mm（基座 +X/+Y/+Z），例: --deadzone-mm 0 0 8",
    )
    ap.add_argument(
        "--force-move",
        action="store_true",
        help="即使预计位移很小也下发 MoveL",
    )
    ap.add_argument(
        "--normal",
        choices=("plane", "base_z", "camera"),
        default="plane",
        help="抬起方向；plane 失败时见 --normal-fallback",
    )
    ap.add_argument(
        "--normal-fallback",
        choices=("camera", "base_z"),
        default="camera",
        help="plane 法向估计失败或深度不可靠时的回退方向",
    )
    ap.add_argument("--vel", type=float, default=10.0)
    ap.add_argument("--tool", type=int, default=0)
    ap.add_argument("--user", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--headless", action="store_true")
    add_orient_arguments(ap)
    ap.add_argument(
        "--auto-reduce-height",
        action="store_true",
        default=True,
        help="逆解无解时沿法向自动降低高度（默认开启）",
    )
    ap.add_argument(
        "--no-auto-reduce-height",
        action="store_false",
        dest="auto_reduce_height",
        help="禁止自动降高度",
    )
    add_depth_stabilize_args(ap)
    args = ap.parse_args()

    T_c2g = load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper")

    use_rs_filter = not args.no_rs_depth_filter
    cam = RealSenseGrabber(with_depth=True, depth_filters=use_rs_filter)
    if use_rs_filter:
        print("[OK] RealSense 深度滤波已开启（spatial + temporal）")
    rob = _load_robot(args.host)
    try:
        if args.pixel:
            u, v = int(args.pixel[0]), int(args.pixel[1])
            color, depth = cam.read()
            return run_once(args, u, v, color, depth, cam, T_c2g, rob)

        use_gui = (not args.headless) and gui_available()
        if use_gui:
            print(
                "GUI：左键选点 → 滚轮/双指捏合缩放 → m 移动 → r 刷新 → q 退出"
            )
            return run_gui(args, T_c2g, rob, cam)

        print("[FAIL] 无 GUI，请用 --pixel U V 或 --headless 下指定像素")
        print(
            f"  例: ./run_pick_point_move.sh --pixel 640 360 --height-mm 50"
            f"  # 总高度 {BASE_HEIGHT_MM + 50:.0f} mm"
        )
        return 1
    finally:
        cam.stop()


if __name__ == "__main__":
    raise SystemExit(main())
