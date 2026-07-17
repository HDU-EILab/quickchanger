#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR10 + RealSense D435 棋盘格手眼标定（交互采集 + OpenCV calibrateHandEye）

棋盘：12×9 格 → 内角点 11×8；单格边长默认 15 mm。
机械臂位姿 [x,y,z,rx,ry,rz]，单位 mm / °，外旋 xyz 欧拉角转 4×4 齐次矩阵。

依赖环境（Jetson 上建议 rmc-yolo + FAIRINO Robot 模块）：
  conda activate rmc-yolo
  export PYTHONPATH=/home/nvidia/miniconda3/envs/fr10-fairino/lib/python3.10/site-packages:$PYTHONPATH

用法：
  ./run_handeye_calibrate.sh --mode eye_in_hand --min-samples 12   # 默认 GUI
  ./run_handeye_calibrate.sh --headless ...   # 无显示器 / 终端模式

交互（GUI 窗口，或 --headless 时在终端输入）：
  c / 空格  采集一帧（须检测到棋盘）
  p         仅预览（headless 模式，保存 preview_latest.jpg）
  d         删除上一帧
  q         结束采集并计算（样本数 ≥ min-samples）
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

FR10_HOST_DEFAULT = "192.168.58.2"
# 12×9 格 → 内角点 11×8
INNER_CORNERS = (11, 8)
SQUARE_SIZE_M_DEFAULT = 0.015
CHESSBOARD_FLAGS = (
    cv2.CALIB_CB_ADAPTIVE_THRESH
    | cv2.CALIB_CB_NORMALIZE_IMAGE
    | cv2.CALIB_CB_FAST_CHECK
)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def _load_robot(host: str):
    try:
        from fairino import Robot  # type: ignore
    except Exception:
        import Robot as Robot  # type: ignore
    return Robot.RPC(host)


def _normalize_joint(raw) -> list[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) < 7:
        raise RuntimeError(f"GetActualJointPosDegree 返回异常: {raw}")
    if int(raw[0]) != 0:
        raise RuntimeError(f"GetActualJointPosDegree 错误码 {raw[0]}: {raw}")
    return [float(raw[i]) for i in range(1, 7)]


def _normalize_pose(raw) -> list[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) < 7:
        raise RuntimeError(f"位姿返回异常: {raw}")
    if int(raw[0]) != 0:
        raise RuntimeError(f"位姿错误码 {raw[0]}: {raw}")
    return [float(raw[i]) for i in range(1, 7)]


def pose_to_matrix(pose_mm_deg: list[float], euler_order: str = "xyz") -> np.ndarray:
    """Fairino [x,y,z,rx,ry,rz] → 4×4，平移 m，外旋欧拉角（度）。"""
    from scipy.spatial.transform import Rotation

    t = np.array(pose_mm_deg[:3], dtype=np.float64) * 1e-3
    R = Rotation.from_euler(euler_order, pose_mm_deg[3:6], degrees=True).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def make_object_points(inner: tuple[int, int], square_m: float) -> np.ndarray:
    cols, rows = inner
    objp = np.zeros((cols * rows, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    objp[:, :2] = grid * square_m
    return objp


def detect_chessboard(gray: np.ndarray, inner: tuple[int, int]):
    found, corners = cv2.findChessboardCorners(gray, inner, CHESSBOARD_FLAGS)
    if not found:
        return False, None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return True, corners


def solve_target_to_cam(
    corners: np.ndarray,
    objp: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    ok, rvec, tvec = cv2.solvePnP(
        objp,
        corners,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP 失败")
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), rvec, tvec


def read_robot_state(rob, frame: str):
    if frame == "flange":
        raw = rob.GetActualToolFlangePose(1)
    else:
        raw = rob.robot.GetActualTCPPose(1)
    pose = _normalize_pose(raw)
    joints = _normalize_joint(rob.robot.GetActualJointPosDegree(1))
    return joints, pose


class RealSenseGrabber:
    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        depth_width: int = 848,
        depth_height: int = 480,
        with_depth: bool = True,
        depth_filters: bool = False,
    ):
        self.with_depth = with_depth
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        if with_depth:
            cfg.enable_stream(
                rs.stream.depth, depth_width, depth_height, rs.format.z16, fps
            )
        profile = self.pipeline.start(cfg)
        time.sleep(1.5)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()
        self.camera_matrix = np.array(
            [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.array(intr.coeffs[:5], dtype=np.float64)
        self.depth_scale = 0.001
        if with_depth:
            self.depth_scale = (
                profile.get_device().first_depth_sensor().get_depth_scale()
            )
        self._align = rs.align(rs.stream.color) if with_depth else None
        self._depth_filters = None
        if with_depth and depth_filters:
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 2)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
            spatial.set_option(rs.option.filter_smooth_delta, 20)
            temporal = rs.temporal_filter()
            temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
            temporal.set_option(rs.option.filter_smooth_delta, 20)
            hole = rs.hole_filling_filter()
            self._depth_filters = (spatial, temporal, hole)
        self._color_ts0_ms = None
        self._color_mono0_s = None
        self.last_frame_time_s = None
        self.last_frame_arrival_s = None
        self.last_frame_latency_s = None
        self.last_frame_number = None
        self.last_frame_timestamp_ms = None

    def _record_color_frame_metadata(self, color):
        arrival_s = time.monotonic()
        ts_ms = float(color.get_timestamp())
        if self._color_ts0_ms is None:
            self._color_ts0_ms = ts_ms
            self._color_mono0_s = arrival_s
        frame_time_s = self._color_mono0_s + (ts_ms - self._color_ts0_ms) / 1000.0
        self.last_frame_time_s = frame_time_s
        self.last_frame_arrival_s = arrival_s
        self.last_frame_latency_s = max(0.0, arrival_s - frame_time_s)
        self.last_frame_number = int(color.get_frame_number())
        self.last_frame_timestamp_ms = ts_ms

    def _filter_depth_frame(self, depth_frame):
        if self._depth_filters is None:
            return depth_frame
        spatial, temporal, hole = self._depth_filters
        depth_frame = spatial.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        depth_frame = hole.process(depth_frame)
        return depth_frame

    def read(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=10000)
        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("未获取到彩色帧")
        self._record_color_frame_metadata(color)
        color_np = np.asanyarray(color.get_data())
        depth_np = None
        if self.with_depth:
            depth = frames.get_depth_frame()
            if not depth:
                raise RuntimeError("未获取到深度帧")
            depth = self._filter_depth_frame(depth)
            depth_np = np.asanyarray(depth.get_data())
        return color_np, depth_np

    def read_color_only(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=10000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("未获取到彩色帧")
        self._record_color_frame_metadata(color)
        return np.asanyarray(color.get_data())

    def stop(self):
        self.pipeline.stop()


class Sample:
    def __init__(
        self,
        idx: int,
        joints: list[float],
        pose: list[float],
        R_g2b: np.ndarray,
        t_g2b: np.ndarray,
        R_t2c: np.ndarray,
        t_t2c: np.ndarray,
        color_path: str,
        depth_path: str,
    ):
        self.idx = idx
        self.joints = joints
        self.pose = pose
        self.R_g2b = R_g2b
        self.t_g2b = t_g2b
        self.R_t2c = R_t2c
        self.t_t2c = t_t2c
        self.color_path = color_path
        self.depth_path = depth_path

    def to_dict(self) -> dict:
        return {
            "idx": self.idx,
            "joints_deg": self.joints,
            "pose_mm_deg": self.pose,
            "R_gripper2base": self.R_g2b.tolist(),
            "t_gripper2base": self.t_g2b.reshape(3).tolist(),
            "R_target2cam": self.R_t2c.tolist(),
            "t_target2cam": self.t_t2c.reshape(3).tolist(),
            "color": self.color_path,
            "depth": self.depth_path,
        }


def compute_handeye(samples: list[Sample], mode: str, method: int):
    R_g2b = [s.R_g2b for s in samples]
    t_g2b = [s.t_g2b.reshape(3, 1) for s in samples]
    R_t2c = [s.R_t2c for s in samples]
    t_t2c = [s.t_t2c.reshape(3, 1) for s in samples]

    if mode == "eye_in_hand":
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=method
        )
        return {
            "mode": mode,
            "R_cam2gripper": R_cam2gripper.tolist(),
            "t_cam2gripper": t_cam2gripper.reshape(3).tolist(),
            "T_cam2gripper": _T_from_Rt(R_cam2gripper, t_cam2gripper).tolist(),
        }
    if mode == "eye_to_hand":
        # 眼在手外：相机固定，棋盘随末端运动；输入 base→gripper 逆变换
        R_b2g = [R.T for R in R_g2b]
        t_b2g = [(-R.T @ t) for R, t in zip(R_g2b, t_g2b)]
        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_b2g, t_b2g, R_t2c, t_t2c, method=method
        )
        return {
            "mode": mode,
            "R_cam2base": R_cam2base.tolist(),
            "t_cam2base": t_cam2base.reshape(3).tolist(),
            "T_cam2base": _T_from_Rt(R_cam2base, t_cam2base).tolist(),
        }
    raise ValueError(f"未知 mode: {mode}")


def _T_from_Rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def handeye_reproj_error(samples: list[Sample], result: dict, mode: str) -> float:
    """标定一致性：眼在手上→棋盘原点在基座标系应重合；眼在手外→棋盘相对末端应恒定。单位 m。"""
    origins = []
    for s in samples:
        T_g2b = _T_from_Rt(s.R_g2b, s.t_g2b)
        T_t2c = _T_from_Rt(s.R_t2c, s.t_t2c)
        if mode == "eye_in_hand":
            T_c2g = np.array(result["T_cam2gripper"], dtype=np.float64)
            T_t2b = T_g2b @ T_c2g @ T_t2c
            origins.append(T_t2b[:3, 3])
        else:
            T_c2b = np.array(result["T_cam2base"], dtype=np.float64)
            T_t2b = T_c2b @ T_t2c
            T_t2g = np.linalg.inv(T_g2b) @ T_t2b
            origins.append(T_t2g[:3, 3])
    origins = np.array(origins)
    mean = origins.mean(axis=0)
    return float(np.linalg.norm(origins - mean, axis=1).mean())


def handeye_sample_residuals(
    samples: list[Sample], result: dict, mode: str
) -> list[tuple[int, float]]:
    """Return per-sample board-origin consistency residuals in meters."""
    origins = []
    for s in samples:
        T_g2b = _T_from_Rt(s.R_g2b, s.t_g2b)
        T_t2c = _T_from_Rt(s.R_t2c, s.t_t2c)
        if mode == "eye_in_hand":
            T_c2g = np.array(result["T_cam2gripper"], dtype=np.float64)
            T_t2b = T_g2b @ T_c2g @ T_t2c
            origins.append(T_t2b[:3, 3])
        else:
            T_c2b = np.array(result["T_cam2base"], dtype=np.float64)
            T_t2b = T_c2b @ T_t2c
            T_t2g = np.linalg.inv(T_g2b) @ T_t2b
            origins.append(T_t2g[:3, 3])
    origins_np = np.array(origins)
    mean = origins_np.mean(axis=0)
    residuals = np.linalg.norm(origins_np - mean, axis=1)
    return [(int(s.idx), float(r)) for s, r in zip(samples, residuals)]


def print_outlier_report(samples: list[Sample], result: dict, mode: str) -> None:
    rows = sorted(
        handeye_sample_residuals(samples, result, mode),
        key=lambda item: item[1],
        reverse=True,
    )
    print("\n========== 逐样本一致性残差 ==========")
    for idx, residual_m in rows:
        pose = next(s.pose for s in samples if int(s.idx) == idx)
        print(
            f"#{idx:02d}: {residual_m * 1000:7.2f} mm  "
            f"xyz=({pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f})  "
            f"rpy=({pose[3]:.1f},{pose[4]:.1f},{pose[5]:.1f})"
        )


def _compute_result_and_error(
    samples: list[Sample], mode: str, method: int
) -> tuple[dict, float]:
    result = compute_handeye(samples, mode, method)
    err = handeye_reproj_error(samples, result, mode)
    return result, err


def print_leave_one_out_report(
    samples: list[Sample], mode: str, method: int
) -> None:
    if len(samples) <= 3:
        return
    rows = []
    for i, sample in enumerate(samples):
        trial = samples[:i] + samples[i + 1 :]
        _, err = _compute_result_and_error(trial, mode, method)
        rows.append((int(sample.idx), err))
    print("\n========== 删除单个样本后重算 ==========")
    for idx, err in sorted(rows, key=lambda item: item[1]):
        print(f"drop #{idx:02d}: mean={err * 1000:7.3f} mm")


def greedy_prune_samples(
    samples: list[Sample],
    mode: str,
    method: int,
    *,
    min_samples: int,
    max_drop: int,
    min_improve_m: float,
) -> tuple[list[Sample], list[int], list[dict]]:
    keep = list(samples)
    removed: list[int] = []
    history: list[dict] = []
    _, current_err = _compute_result_and_error(keep, mode, method)

    while len(keep) > min_samples and len(removed) < max_drop:
        best_sample = None
        best_err = current_err
        best_trial: list[Sample] | None = None
        for sample in keep:
            trial = [s for s in keep if int(s.idx) != int(sample.idx)]
            _, err = _compute_result_and_error(trial, mode, method)
            if err < best_err:
                best_sample = sample
                best_err = err
                best_trial = trial
        if best_sample is None or best_trial is None:
            break
        if current_err - best_err < min_improve_m:
            break
        removed.append(int(best_sample.idx))
        keep = best_trial
        current_err = best_err
        history.append(
            {
                "dropped": int(best_sample.idx),
                "num_samples": len(keep),
                "mean_board_origin_error_m": current_err,
                "keep_indices": [int(s.idx) for s in keep],
            }
        )
    return keep, removed, history


def save_session(
    out_dir: Path,
    samples: list[Sample],
    meta: dict,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
):
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    np.savez(
        out_dir / "intrinsics.npz",
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
    )
    payload = [s.to_dict() for s in samples]
    (out_dir / "samples.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def load_session(session_dir: Path):
    meta = json.loads((session_dir / "meta.json").read_text(encoding="utf-8"))
    intr = np.load(session_dir / "intrinsics.npz")
    camera_matrix = intr["camera_matrix"]
    dist_coeffs = intr["dist_coeffs"]
    raw_samples = json.loads((session_dir / "samples.json").read_text(encoding="utf-8"))
    samples: list[Sample] = []
    for row in raw_samples:
        samples.append(
            Sample(
                idx=int(row["idx"]),
                joints=row["joints_deg"],
                pose=row["pose_mm_deg"],
                R_g2b=np.array(row["R_gripper2base"], dtype=np.float64),
                t_g2b=np.array(row["t_gripper2base"], dtype=np.float64),
                R_t2c=np.array(row["R_target2cam"], dtype=np.float64),
                t_t2c=np.array(row["t_target2cam"], dtype=np.float64),
                color_path=row.get("color", ""),
                depth_path=row.get("depth", ""),
            )
        )
    return meta, camera_matrix, dist_coeffs, samples


def gui_available() -> bool:
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        return False
    try:
        cv2.namedWindow("__gui_probe__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__gui_probe__")
        return True
    except cv2.error:
        return False


def _annotate_preview(
    color: np.ndarray,
    inner: tuple[int, int],
    found: bool,
    corners,
    samples: int,
    min_samples: int,
) -> np.ndarray:
    vis = color.copy()
    if found and corners is not None:
        cv2.drawChessboardCorners(vis, inner, corners, found)
    cv2.putText(
        vis,
        f"samples {samples}/{min_samples}  detected={found}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0) if found else (0, 0, 255),
        2,
    )
    return vis


def _try_capture_sample(
    *,
    found: bool,
    corners,
    color: np.ndarray,
    depth: np.ndarray | None,
    objp: np.ndarray,
    rob,
    args: argparse.Namespace,
    cam: RealSenseGrabber,
    out_dir: Path,
    samples: list[Sample],
) -> bool:
    if not found or corners is None:
        print("[skip] 未检测到棋盘，请调整位姿后重试")
        return False
    joints, pose = read_robot_state(rob, args.robot_frame)
    T_g2b = pose_to_matrix(pose, args.euler_order)
    R_t2c, t_t2c, _, _ = solve_target_to_cam(
        corners, objp, cam.camera_matrix, cam.dist_coeffs
    )
    idx = len(samples)
    color_name = f"{idx:03d}_color.png"
    depth_name = f"{idx:03d}_depth.png"
    cv2.imwrite(str(out_dir / color_name), color)
    if depth is not None:
        cv2.imwrite(str(out_dir / depth_name), depth)
    samples.append(
        Sample(
            idx=idx,
            joints=joints,
            pose=pose,
            R_g2b=T_g2b[:3, :3],
            t_g2b=T_g2b[:3, 3],
            R_t2c=R_t2c,
            t_t2c=t_t2c.reshape(3),
            color_path=color_name,
            depth_path=depth_name,
        )
    )
    print(
        f"[capture #{idx}] joints={[round(j, 3) for j in joints]} "
        f"pose={[round(p, 2) for p in pose]}"
    )
    return True


def _read_chessboard_frame(cam: RealSenseGrabber, inner: tuple[int, int]):
    color, depth = cam.read()
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    found, corners = detect_chessboard(gray, inner)
    return color, depth, found, corners


def run_interactive_headless(
    args: argparse.Namespace,
    inner: tuple[int, int],
    objp: np.ndarray,
    out_dir: Path,
    rob,
    cam: RealSenseGrabber,
    samples: list[Sample],
) -> None:
    print("=" * 56)
    print("终端标定模式（不占用 NoMachine 画面，在此输入命令）")
    print("  c = 采集一帧   p = 预览 preview_latest.jpg")
    print("  d = 删除上一帧   q = 结束并计算")
    print("=" * 56, flush=True)
    while True:
        cmd = input(f"[{len(samples)}/{args.min_samples}] > ").strip().lower()
        if cmd in ("q", "quit", "exit"):
            break
        if cmd in ("d", "del", "delete"):
            if samples:
                removed = samples.pop()
                print(f"[delete] 移除样本 #{removed.idx}")
            else:
                print("[delete] 无样本可删")
            continue
        if cmd in ("p", "preview", ""):
            color, depth, found, corners = _read_chessboard_frame(cam, inner)
            vis = _annotate_preview(
                color, inner, found, corners, len(samples), args.min_samples
            )
            preview = out_dir / "preview_latest.jpg"
            cv2.imwrite(str(preview), vis)
            print(f"[preview] detected={found} 已保存 {preview}")
            continue
        if cmd in ("c", "capture", " "):
            color, depth, found, corners = _read_chessboard_frame(cam, inner)
            _try_capture_sample(
                found=found,
                corners=corners,
                color=color,
                depth=depth,
                objp=objp,
                rob=rob,
                args=args,
                cam=cam,
                out_dir=out_dir,
                samples=samples,
            )
            continue
        print("未知命令，请输入 c / p / d / q")


def run_interactive_gui(
    args: argparse.Namespace,
    inner: tuple[int, int],
    objp: np.ndarray,
    out_dir: Path,
    rob,
    cam: RealSenseGrabber,
    samples: list[Sample],
) -> None:
    win = "FR10 hand-eye | c=capture d=del q=quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            color, depth, found, corners = _read_chessboard_frame(cam, inner)
            vis = _annotate_preview(
                color, inner, found, corners, len(samples), args.min_samples
            )
            cv2.imshow(win, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("c"), ord(" ")):
                _try_capture_sample(
                    found=found,
                    corners=corners,
                    color=color,
                    depth=depth,
                    objp=objp,
                    rob=rob,
                    args=args,
                    cam=cam,
                    out_dir=out_dir,
                    samples=samples,
                )
            elif key == ord("d"):
                if samples:
                    removed = samples.pop()
                    print(f"[delete] 移除样本 #{removed.idx}")
            elif key in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()


def run_interactive(args: argparse.Namespace) -> int:
    inner = (args.cols - 1, args.rows - 1)
    objp = make_object_points(inner, args.square_mm * 1e-3)
    out_dir = Path(args.output_dir) / f"handeye_session_{datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True, exist_ok=True)

    rob = _load_robot(args.host)
    cam = RealSenseGrabber(
        args.width,
        args.height,
        args.fps,
        args.depth_width,
        args.depth_height,
        with_depth=not args.no_depth,
    )
    samples: list[Sample] = []
    use_gui = (not args.headless) and gui_available()
    if not args.headless and not use_gui:
        print("[INFO] GUI 不可用，自动切换终端模式；可加 --headless 显式指定")

    print(f"会话目录: {out_dir}")
    print(f"棋盘 内角点 {inner[0]}×{inner[1]}，格长 {args.square_mm} mm，模式 {args.mode}")
    if use_gui:
        print("GUI 模式：在 OpenCV 窗口按 c / 空格 采集，d 删除，q 结束。")
    else:
        print("终端模式：输入 c 采集，p 预览，d 删除，q 结束。")

    try:
        if use_gui:
            run_interactive_gui(args, inner, objp, out_dir, rob, cam, samples)
        else:
            run_interactive_headless(args, inner, objp, out_dir, rob, cam, samples)
    finally:
        cam.stop()

    if len(samples) < args.min_samples:
        print(
            f"[FAIL] 样本数 {len(samples)} < {args.min_samples}，请重新运行并多采集几帧"
        )
        meta = _build_meta(args, inner)
        save_session(out_dir, samples, meta, cam.camera_matrix, cam.dist_coeffs)
        return 1

    return finalize_session(
        out_dir, samples, args, inner, cam.camera_matrix, cam.dist_coeffs
    )


def finalize_session(
    out_dir: Path,
    samples: list[Sample],
    args: argparse.Namespace,
    inner: tuple[int, int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> int:
    method = getattr(cv2, args.handeye_method)
    source_num_samples = len(samples)
    pruned_info = None
    if getattr(args, "auto_prune", False):
        keep, removed, history = greedy_prune_samples(
            samples,
            args.mode,
            method,
            min_samples=max(int(args.min_samples), int(args.prune_min_samples)),
            max_drop=int(args.prune_max_drop),
            min_improve_m=float(args.prune_min_improve_mm) * 1e-3,
        )
        if removed:
            samples = keep
            pruned_info = {
                "source_num_samples": source_num_samples,
                "removed_indices": removed,
                "keep_indices": [int(s.idx) for s in samples],
                "history": history,
            }
            print("\n========== 自动剔除样本 ==========")
            for row in history:
                print(
                    f"drop #{row['dropped']:02d}: n={row['num_samples']} "
                    f"mean={row['mean_board_origin_error_m'] * 1000:.3f} mm"
                )
        else:
            print("\n[INFO] 自动剔除未找到满足改善阈值的样本")

    result = compute_handeye(samples, args.mode, method)
    err = handeye_reproj_error(samples, result, args.mode)
    result["mean_board_origin_error_m"] = err
    result["num_samples"] = len(samples)
    if pruned_info is not None:
        result["pruned"] = pruned_info
    result["computed_at"] = datetime.now().isoformat(timespec="seconds")

    meta = _build_meta(args, inner)
    meta["handeye"] = result
    save_session(out_dir, samples, meta, camera_matrix, dist_coeffs)
    (out_dir / "handeye_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print("\n========== 手眼标定结果 ==========")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"平均棋盘原点一致性误差: {err * 1000:.2f} mm")
    print(f"已保存: {out_dir / 'handeye_result.json'}")
    if getattr(args, "analyze_outliers", False):
        print_outlier_report(samples, result, args.mode)
        print_leave_one_out_report(samples, args.mode, method)
    if err > 0.01:
        print("[WARN] 误差偏大，建议增加样本、扩大位姿变化范围或检查棋盘尺寸/模式(eye_in_hand/eye_to_hand)")
    return 0


def _build_meta(args: argparse.Namespace, inner: tuple[int, int]) -> dict:
    return {
        "host": args.host,
        "mode": args.mode,
        "robot_frame": args.robot_frame,
        "euler_order": args.euler_order,
        "board_squares": [args.cols, args.rows],
        "inner_corners": list(inner),
        "square_mm": args.square_mm,
        "handeye_method": args.handeye_method,
    }


def run_compute_only(session_dir: Path, args: argparse.Namespace) -> int:
    meta, camera_matrix, dist_coeffs, samples = load_session(session_dir)
    inner = tuple(meta["inner_corners"])
    if len(samples) < args.min_samples:
        print(f"[FAIL] 会话仅 {len(samples)} 帧，少于 --min-samples {args.min_samples}")
        return 1
    args.mode = meta.get("mode", args.mode)
    out_dir = session_dir
    if args.auto_prune:
        out_dir = Path(args.prune_output_dir) if args.prune_output_dir else Path(
            f"{session_dir}_pruned"
        )
        if out_dir.exists() and out_dir.resolve() != session_dir.resolve():
            raise FileExistsError(f"输出目录已存在: {out_dir}")
        if out_dir.resolve() != session_dir.resolve():
            shutil.copytree(session_dir, out_dir)
            print(f"[INFO] 自动剔除将在副本中计算，不覆盖原始会话: {out_dir}")
    return finalize_session(
        out_dir, samples, args, inner, camera_matrix, dist_coeffs
    )


def parse_args():
    p = argparse.ArgumentParser(description="FR10 + D435 棋盘格手眼标定")
    p.add_argument("--host", default=FR10_HOST_DEFAULT)
    p.add_argument("--mode", choices=("eye_in_hand", "eye_to_hand"), default="eye_in_hand")
    p.add_argument(
        "--robot-frame",
        choices=("tcp", "flange"),
        default="tcp",
        help="用于标定的机械臂坐标系：工具 TCP 或法兰",
    )
    p.add_argument("--euler-order", default="xyz", help="位姿欧拉角顺序（外旋）")
    p.add_argument("--cols", type=int, default=12, help="棋盘横向格数")
    p.add_argument("--rows", type=int, default=9, help="棋盘纵向格数")
    p.add_argument("--square-mm", type=float, default=15.0, help="单格边长 mm")
    p.add_argument("--min-samples", type=int, default=10, help="最少采集帧数")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--depth-width", type=int, default=848)
    p.add_argument("--depth-height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument(
        "--no-depth",
        action="store_true",
        help="仅彩色流（棋盘检测足够；深度可选保存）",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="强制终端模式（无 OpenCV 窗口）",
    )
    p.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "sessions"),
    )
    p.add_argument(
        "--handeye-method",
        default="CALIB_HAND_EYE_TSAI",
        choices=[
            "CALIB_HAND_EYE_TSAI",
            "CALIB_HAND_EYE_PARK",
            "CALIB_HAND_EYE_HORAUD",
            "CALIB_HAND_EYE_ANDREFF",
            "CALIB_HAND_EYE_DANIILIDIS",
        ],
    )
    p.add_argument(
        "--compute-only",
        type=str,
        default="",
        help="从已保存会话目录重新计算，不连接相机/机械臂",
    )
    p.add_argument(
        "--analyze-outliers",
        action="store_true",
        help="输出逐样本残差和 leave-one-out 重算结果，用于定位坏样本",
    )
    p.add_argument(
        "--auto-prune",
        action="store_true",
        help="按当前误差指标贪心剔除坏样本后重算；compute-only 时默认写入 *_pruned 副本",
    )
    p.add_argument(
        "--prune-output-dir",
        default="",
        help="--compute-only --auto-prune 的输出目录；默认 <session>_pruned",
    )
    p.add_argument(
        "--prune-max-drop",
        type=int,
        default=6,
        help="自动剔除最多删除的样本数",
    )
    p.add_argument(
        "--prune-min-samples",
        type=int,
        default=10,
        help="自动剔除后至少保留的样本数",
    )
    p.add_argument(
        "--prune-min-improve-mm",
        type=float,
        default=0.1,
        help="每删除一个样本至少改善多少 mm；低于该值停止",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.compute_only:
        return run_compute_only(Path(args.compute_only), args)
    return run_interactive(args)


if __name__ == "__main__":
    raise SystemExit(main())
