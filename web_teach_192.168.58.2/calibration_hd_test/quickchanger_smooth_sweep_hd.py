#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smooth demo sweep for quickchanger.

Unlike quickchanger_demo_sweep_hd.py, the scan phase sends one continuous MoveL
from demo_start to demo_end. The parent process keeps sampling RealSense frames
and robot TCP while that MoveL runs. By default, the first valid target ArUco
detection stops the scan, then the stopped view is rechecked and used for the
existing quickchanger two-stage move.
"""
from __future__ import annotations

import argparse
import json
import threading
import subprocess
import sys
import time
import xmlrpc.client
from collections import deque
from pathlib import Path

import cv2
import numpy as np

_DEV_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DEV_DIR))

import quickchanger_aruco_hd as qc  # noqa: E402
from quickchanger_demo_sweep_hd import (  # noqa: E402
    DEFAULT_END,
    DEFAULT_LOST_AFTER_VISIBLE,
    DEFAULT_START,
    aruco_score,
    capture_detection,
)

DEFAULT_SMOOTH_SCAN_VEL = 20.0
DEFAULT_SAMPLE_HZ = 8.0
DEFAULT_STOP_ON_LOST = True
DEFAULT_POSE_SYNC_MODE = "buffer"
DEFAULT_POSE_SAMPLE_HZ = 40.0
DEFAULT_FRAME_TIME_OFFSET_S = 0.12
DEFAULT_SCAN_DETECT_SCALE = 0.5
DEFAULT_MIN_VALID_BEFORE_STOP = 3
DEFAULT_STOP_ON_FIRST_VISIBLE = True
DEFAULT_EXPERIENCE_MOVE_VEL = 35.0
DEFAULT_EXPERIENCE_DISABLED_IDS = (2,)
DEFAULT_EXPERIENCE_PATH = _DEV_DIR / "quickchanger_experience_positions.json"
DEFAULT_REQUIRE_VALID_DEPTH_BEFORE_STOP = True
DEFAULT_STOP_DEPTH_CHECK_FRAMES = 2


def build_arg_parser() -> argparse.ArgumentParser:
    ap = qc.build_arg_parser()
    ap.description = (
        "Quickchanger smooth demo sweep: continuous MoveL scan for ArUco, "
        "then run quickchanger"
    )
    ap.add_argument(
        "aruco_id_pos",
        nargs="?",
        type=int,
        help="目标 ArUco ID；等价于 --aruco-id，默认 1",
    )
    ap.add_argument(
        "--demo-start",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        default=None,
        help="演示扫描起点 TCP mm/rpy；不传则读取 config",
    )
    ap.add_argument(
        "--demo-end",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        default=None,
        help="演示扫描终点 TCP mm/rpy；不传则读取 config",
    )
    ap.add_argument(
        "--smooth-scan-vel",
        type=float,
        default=None,
        help="连续扫描 MoveL 速度；不传则读取 config，默认 20",
    )
    ap.add_argument(
        "--smooth-experience-move-vel",
        type=float,
        default=None,
        help="移动到经验位姿的 MoveL 速度；不传则读取 config，默认 35",
    )
    ap.add_argument(
        "--smooth-sample-hz",
        type=float,
        default=None,
        help="连续扫描中相机检测采样频率；不传则读取 config，默认 8 Hz",
    )
    ap.add_argument(
        "--min-perimeter-px",
        type=float,
        default=120.0,
        help="低于该周长的 ArUco 检测不参与遮挡评分，默认 120px",
    )
    ap.add_argument(
        "--lost-after-visible",
        type=int,
        default=None,
        help="首次识别后连续多少帧未识别即结束扫描；不传则读取 config",
    )
    ap.add_argument(
        "--complete-scan",
        action="store_true",
        help="完整扫到终点后再回到最佳视角；不按连续丢失提前触发",
    )
    ap.add_argument(
        "--no-stop-on-lost",
        action="store_true",
        help="连续丢失触发后不发送 StopMotion，而是等待连续扫描 MoveL 自然结束",
    )
    ap.add_argument(
        "--pose-sync-mode",
        choices=("buffer", "bracket", "after"),
        default=None,
        help=(
            "扫描中图像与 TCP 同步方式；buffer=后台高频采 TCP 并按图像时间插值，"
            "bracket=取图前后读 TCP 并插值，after=旧逻辑取图后读当前 TCP；"
            "不传则读取 config"
        ),
    )
    ap.add_argument(
        "--pose-sample-hz",
        type=float,
        default=None,
        help="pose-sync-mode=buffer 时的后台 TCP 采样频率；不传则读取 config",
    )
    ap.add_argument(
        "--frame-time-offset-s",
        type=float,
        default=None,
        help=(
            "RealSense 图像帧时间校正，单位秒；正值表示图像比 read() 返回时刻更早，"
            "高速扫描错位时可调大；不传则读取 config"
        ),
    )
    ap.add_argument(
        "--scan-detect-scale",
        type=float,
        default=None,
        help=(
            "smooth 扫描阶段 ArUco 检测图像缩放比例；0.5 表示用半分辨率检测后映射回原图，"
            "不传则读取 config"
        ),
    )
    ap.add_argument(
        "--smooth-min-valid-before-stop",
        type=int,
        default=None,
        help="目标窗口至少累计多少帧有效匹配后，才允许连续丢失触发 StopMotion；不传则读取 config",
    )
    ap.add_argument(
        "--no-stop-on-first-visible",
        action="store_true",
        help="禁用首帧有效目标即停，回退到历史中间匹配视角逻辑",
    )
    ap.add_argument(
        "--experience-path",
        type=Path,
        default=None,
        help=f"按 ArUco ID 记录经验停止位姿的 JSON 文件，默认 {DEFAULT_EXPERIENCE_PATH.name}",
    )
    ap.add_argument(
        "--no-experience",
        action="store_true",
        help="禁用经验位姿复用，直接执行原扫描起点到终点搜索",
    )
    ap.add_argument(
        "--_smooth-move-worker",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    ap.add_argument(
        "--_worker-target-pose",
        nargs=6,
        type=float,
        default=None,
        help=argparse.SUPPRESS,
    )
    return ap


def cli_has(argv: list[str], name: str) -> bool:
    return any(arg == name or arg.startswith(name + "=") for arg in argv)


def apply_smooth_config(args: argparse.Namespace, argv: list[str]):
    cfg = qc.apply_quickchanger_config(args, argv)
    demo_start = qc.config_vec(cfg, "demo_start", 6)
    demo_end = qc.config_vec(cfg, "demo_end", 6)
    if args.demo_start is None:
        args.demo_start = demo_start if demo_start is not None else list(DEFAULT_START)
    if args.demo_end is None:
        args.demo_end = demo_end if demo_end is not None else list(DEFAULT_END)
    if args.lost_after_visible is None:
        args.lost_after_visible = int(
            cfg.get("lost_after_visible", DEFAULT_LOST_AFTER_VISIBLE)
        )
    if args.smooth_scan_vel is None:
        args.smooth_scan_vel = float(
            cfg.get("smooth_scan_vel", DEFAULT_SMOOTH_SCAN_VEL)
        )
    if args.smooth_experience_move_vel is None:
        args.smooth_experience_move_vel = float(
            cfg.get(
                "smooth_experience_move_vel",
                cfg.get("smooth_scan_vel", DEFAULT_EXPERIENCE_MOVE_VEL),
            )
        )
    if args.smooth_sample_hz is None:
        args.smooth_sample_hz = float(
            cfg.get("smooth_sample_hz", DEFAULT_SAMPLE_HZ)
        )
    cfg_stop = bool(cfg.get("smooth_stop_on_lost", DEFAULT_STOP_ON_LOST))
    args.smooth_stop_on_lost = cfg_stop and not args.no_stop_on_lost
    if args.pose_sync_mode is None:
        args.pose_sync_mode = str(
            cfg.get("smooth_pose_sync_mode", DEFAULT_POSE_SYNC_MODE)
        )
    if args.pose_sample_hz is None:
        args.pose_sample_hz = float(
            cfg.get("smooth_pose_sample_hz", DEFAULT_POSE_SAMPLE_HZ)
        )
    if args.frame_time_offset_s is None:
        args.frame_time_offset_s = float(
            cfg.get("smooth_frame_time_offset_s", DEFAULT_FRAME_TIME_OFFSET_S)
        )
    if args.scan_detect_scale is None:
        args.scan_detect_scale = float(
            cfg.get("smooth_scan_detect_scale", DEFAULT_SCAN_DETECT_SCALE)
        )
    if args.smooth_min_valid_before_stop is None:
        args.smooth_min_valid_before_stop = int(
            cfg.get("smooth_min_valid_before_stop", DEFAULT_MIN_VALID_BEFORE_STOP)
        )
    cfg_stop_first = bool(
        cfg.get("smooth_stop_on_first_visible", DEFAULT_STOP_ON_FIRST_VISIBLE)
    )
    args.smooth_stop_on_first_visible = (
        cfg_stop_first and not args.no_stop_on_first_visible
    )
    if args.experience_path is None:
        exp_path = Path(cfg.get("smooth_experience_path", str(DEFAULT_EXPERIENCE_PATH)))
        args.experience_path = exp_path if exp_path.is_absolute() else _DEV_DIR / exp_path
    disabled_ids = cfg.get("smooth_experience_disabled_ids", DEFAULT_EXPERIENCE_DISABLED_IDS)
    if not isinstance(disabled_ids, (list, tuple)):
        disabled_ids = DEFAULT_EXPERIENCE_DISABLED_IDS
    args.smooth_experience_disabled_ids = {
        int(x) for x in disabled_ids
        if isinstance(x, (int, float, str)) and str(x).strip()
    }
    args.smooth_require_valid_depth_before_stop = bool(
        cfg.get(
            "smooth_require_valid_depth_before_stop",
            DEFAULT_REQUIRE_VALID_DEPTH_BEFORE_STOP,
        )
    )
    args.smooth_stop_depth_check_frames = max(
        1,
        int(cfg.get("smooth_stop_depth_check_frames", DEFAULT_STOP_DEPTH_CHECK_FRAMES)),
    )
    return cfg


def marker_depth_ok_for_stop(
    args: argparse.Namespace,
    cam,
    label: str,
) -> tuple[bool, object | None, object | None, object | None, float]:
    color, depth, result, score = capture_detection(args, cam)
    if not result.ok or result.center_uv is None or score < float(args.min_perimeter_px):
        print(
            f"[SMOOTH] {label} 停止前深度确认：目标复查不稳定 "
            f"score={score:.1f}"
        )
        return False, color, depth, result, score

    old_frames = getattr(args, "depth_frames", 7)
    args.depth_frames = int(getattr(args, "smooth_stop_depth_check_frames", 2))
    try:
        depth_m, depth_tag, _ = qc.marker_depth_m(args, result, depth, cam)
    finally:
        args.depth_frames = old_frames

    if depth_m <= 1e-6:
        print("[SMOOTH] {label} 停止前深度确认：无有效深度，继续扫描".format(label=label))
        return False, color, depth, result, score
    print(
        f"[SMOOTH] {label} 停止前深度确认通过: "
        f"depth={depth_m * 1000.0:.1f}mm tag={depth_tag}"
    )
    return True, color, depth, result, score


def run_quickchanger_with_depth_retries(
    args: argparse.Namespace,
    cam,
    T_c2g: np.ndarray,
    rob,
    color: np.ndarray,
    depth: np.ndarray,
    result,
    score: float,
) -> int:
    attempts = max(1, int(getattr(args, "quickchanger_depth_retry_frames", 6)))
    for attempt in range(1, attempts + 1):
        if result.ok and score >= float(args.min_perimeter_px):
            print(
                f"[SMOOTH] quickchanger 深度/落位尝试 {attempt}/{attempts}, "
                f"score={score:.1f}"
            )
            ret = qc.move_to_quickchanger_target(
                args, result, color, depth, cam, T_c2g, rob
            )
            if ret != qc.QUICKCHANGER_BAD_DEPTH_CODE:
                return ret
            print("[WARN] quickchanger 深度异常，静止位置重新取帧后再试")
        else:
            print(
                f"[WARN] quickchanger 重试帧未稳定识别目标，score={score:.1f}，继续取帧"
            )

        if attempt >= attempts:
            break
        time.sleep(0.08)
        color, depth, result, score = capture_detection(args, cam)

    print("[FAIL] quickchanger 多次复查仍未获得可用深度，未下发阶段1/阶段2")
    return qc.QUICKCHANGER_BAD_DEPTH_CODE


def run_move_worker(args: argparse.Namespace) -> int:
    if args._worker_target_pose is None:
        print("[WORKER][FAIL] missing --_worker-target-pose")
        return 2
    pose = [float(x) for x in args._worker_target_pose]
    print(
        "[WORKER] continuous MoveL ->",
        [round(x, 2) for x in pose],
        f"vel={float(args.smooth_scan_vel):.1f}",
        flush=True,
    )
    rob = qc._load_robot(args.host)
    ret = qc.ppm.move_tcp(rob, pose, args.smooth_scan_vel, args.tool, args.user)
    print(f"[WORKER] MoveL finished ret={ret}", flush=True)
    return int(ret)


def close_robot_rpc(rob) -> None:
    if rob is None:
        return
    for obj in (rob, getattr(rob, "robot", None)):
        if obj is None or not hasattr(obj, "CloseRPC"):
            continue
        try:
            obj.CloseRPC()
            time.sleep(0.35)
            return
        except Exception:
            pass


def normalize_pose_result(raw) -> list[float]:
    if not isinstance(raw, (list, tuple)) or len(raw) == 0:
        raise RuntimeError(f"GetActualTCPPose 返回异常: {raw}")
    if int(raw[0]) != 0:
        raise RuntimeError(f"GetActualTCPPose 错误码 {raw[0]}: {raw}")
    if len(raw) >= 2 and isinstance(raw[1], (list, tuple)) and len(raw[1]) >= 6:
        return [float(raw[1][i]) for i in range(6)]
    if len(raw) >= 7:
        return [float(raw[i]) for i in range(1, 7)]
    raise RuntimeError(f"GetActualTCPPose 返回格式异常: {raw}")


def read_tcp_pose_light(host: str) -> list[float]:
    proxy = xmlrpc.client.ServerProxy(f"http://{host}:20003/RPC2", allow_none=True)
    return normalize_pose_result(proxy.GetActualTCPPose(1))


def lerp_pose(a: list[float], b: list[float], alpha: float) -> list[float]:
    alpha = max(0.0, min(1.0, float(alpha)))
    return [
        float(a[i]) + (float(b[i]) - float(a[i])) * alpha
        for i in range(6)
    ]


class PoseSampler:
    def __init__(self, host: str, hz: float, maxlen: int = 256):
        self.host = host
        self.period = 1.0 / max(1e-6, float(hz))
        self.samples = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.last_error = ""

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="tcp-pose-sampler", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def _run(self) -> None:
        while not self.stop_event.is_set():
            t0 = time.monotonic()
            try:
                pose = read_tcp_pose_light(self.host)
                with self.lock:
                    self.samples.append((time.monotonic(), pose))
            except Exception as exc:
                self.last_error = str(exc)
            elapsed = time.monotonic() - t0
            if elapsed < self.period:
                self.stop_event.wait(self.period - elapsed)

    def pose_at(self, t: float) -> tuple[list[float], float]:
        with self.lock:
            samples = list(self.samples)
        if len(samples) < 2:
            raise RuntimeError(f"TCP 位姿缓存不足: n={len(samples)}, last_error={self.last_error}")
        before = None
        after = None
        for item in samples:
            if item[0] <= t:
                before = item
            elif item[0] > t:
                after = item
                break
        if before is None:
            before = samples[0]
        if after is None:
            after = samples[-1]
        if before is after or abs(after[0] - before[0]) < 1e-6:
            return list(before[1]), abs(t - before[0]) * 1000.0
        alpha = (t - before[0]) / (after[0] - before[0])
        pose = lerp_pose(before[1], after[1], alpha)
        skew_ms = (after[0] - before[0]) * 1000.0
        return pose, skew_ms


def capture_detection_synced(args, cam, pose_sampler: PoseSampler | None = None):
    """Return frame plus a TCP pose matched to the image sampling time."""
    if getattr(args, "pose_sync_mode", DEFAULT_POSE_SYNC_MODE) == "buffer":
        color, depth, result, score = capture_detection_scan(args, cam)
        read_t = getattr(cam, "last_frame_arrival_s", time.monotonic())
        frame_t = getattr(cam, "last_frame_time_s", read_t)
        frame_t -= float(args.frame_time_offset_s)
        if pose_sampler is None:
            raise RuntimeError("pose-sync-mode=buffer 但 PoseSampler 未启动")
        tcp_pose, skew_ms = pose_sampler.pose_at(frame_t)
        latency_ms = max(0.0, (read_t - frame_t) * 1000.0)
        return color, depth, result, score, tcp_pose, frame_t, skew_ms, "buffer", latency_ms

    if getattr(args, "pose_sync_mode", DEFAULT_POSE_SYNC_MODE) == "after":
        color, depth, result, score = capture_detection_scan(args, cam)
        read_t = getattr(cam, "last_frame_arrival_s", time.monotonic())
        pose_t = getattr(cam, "last_frame_time_s", read_t) - float(args.frame_time_offset_s)
        tcp_pose = read_tcp_pose_light(args.host)
        latency_ms = max(0.0, (read_t - pose_t) * 1000.0)
        return color, depth, result, score, tcp_pose, pose_t, 0.0, "after", latency_ms

    pre_pose = read_tcp_pose_light(args.host)
    pre_t = time.monotonic()
    color, depth, result, score = capture_detection_scan(args, cam)
    read_t = getattr(cam, "last_frame_arrival_s", time.monotonic())
    frame_t = getattr(cam, "last_frame_time_s", read_t) - float(args.frame_time_offset_s)
    post_pose = read_tcp_pose_light(args.host)
    post_t = time.monotonic()
    span = max(1e-6, post_t - pre_t)
    alpha = (frame_t - pre_t) / span
    tcp_pose = lerp_pose(pre_pose, post_pose, alpha)
    skew_ms = (post_t - pre_t) * 1000.0
    latency_ms = max(0.0, (read_t - frame_t) * 1000.0)
    return color, depth, result, score, tcp_pose, frame_t, skew_ms, "bracket", latency_ms


def capture_detection_scan(args, cam):
    """Fast scan-time ArUco detection. Depth is not needed until final positioning."""
    if hasattr(cam, "read_color_only"):
        color = cam.read_color_only()
        depth = None
    else:
        color, depth = cam.read()
    scale = float(getattr(args, "scan_detect_scale", DEFAULT_SCAN_DETECT_SCALE))
    detect_img = color
    if 0.0 < scale < 1.0:
        detect_img = cv2.resize(
            color,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_AREA,
        )
    result = qc.detect_aruco_center(
        detect_img,
        marker_id=args.aruco_id,
        dictionary_name=args.dictionary,
    )
    if result.ok and result.center_uv is not None and 0.0 < scale < 1.0:
        inv = 1.0 / scale
        u, v = result.center_uv
        result.center_uv = (float(u) * inv, float(v) * inv)
        if result.corners is not None:
            result.corners = result.corners.astype(np.float64) * inv
        result.perimeter_px = float(result.perimeter_px) * inv
    score = aruco_score(result)
    return color, depth, result, score


def move_or_dry_run(args, rob, pose: list[float], label: str, vel: float) -> int:
    print(f"[SMOOTH] {label}: {[round(x, 2) for x in pose]}")
    if args.dry_run:
        return 0
    err = qc.ppm.move_tcp(rob, pose, vel, args.tool, args.user)
    if err != 0:
        print(f"[FAIL] {label} MoveL 错误码 {err}")
    return int(err)


def start_continuous_move_worker(args, target_pose: list[float]):
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--_smooth-move-worker",
        "--profile",
        str(args.profile),
        "--host",
        str(args.host),
        "--tool",
        str(args.tool),
        "--user",
        str(args.user),
        "--smooth-scan-vel",
        str(float(args.smooth_scan_vel)),
        "--_worker-target-pose",
        *[str(float(x)) for x in target_pose[:6]],
    ]
    if getattr(args, "config", None) is not None:
        cmd.extend(["--config", str(args.config)])
    return subprocess.Popen(
        cmd,
        cwd=str(_DEV_DIR),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def drain_worker_output(proc) -> None:
    if proc is None or proc.stdout is None:
        return
    try:
        out = proc.stdout.read()
    except Exception:
        out = ""
    if out:
        for line in out.splitlines():
            print(line)


def stop_motion_light(host: str) -> bool:
    try:
        proxy = xmlrpc.client.ServerProxy(f"http://{host}:20003/RPC2", allow_none=True)
        ret = proxy.StopMotion()
        print(f"[SMOOTH] XMLRPC StopMotion ret={ret}")
        return True
    except Exception as exc:
        print(f"[WARN] XMLRPC StopMotion 调用失败: {exc}；等待连续 MoveL 结束")
        return False


def validate_args(args: argparse.Namespace) -> int:
    if args.aruco_id_pos is not None:
        args.aruco_id = int(args.aruco_id_pos)
    if args.lost_after_visible < 1:
        print("[FAIL] --lost-after-visible 必须 >= 1")
        return 2
    if args.smooth_sample_hz <= 0:
        print("[FAIL] --smooth-sample-hz 必须 > 0")
        return 2
    if args.smooth_scan_vel <= 0:
        print("[FAIL] --smooth-scan-vel 必须 > 0")
        return 2
    if args.smooth_experience_move_vel <= 0:
        print("[FAIL] --smooth-experience-move-vel 必须 > 0")
        return 2
    if args.pose_sync_mode not in ("buffer", "bracket", "after"):
        print("[FAIL] --pose-sync-mode 必须是 buffer、bracket 或 after")
        return 2
    if args.pose_sample_hz <= 0:
        print("[FAIL] --pose-sample-hz 必须 > 0")
        return 2
    if args.frame_time_offset_s < 0:
        print("[FAIL] --frame-time-offset-s 必须 >= 0")
        return 2
    if not (0.0 < args.scan_detect_scale <= 1.0):
        print("[FAIL] --scan-detect-scale 必须在 (0, 1] 范围内")
        return 2
    if args.smooth_min_valid_before_stop < 1:
        print("[FAIL] --smooth-min-valid-before-stop 必须 >= 1")
        return 2
    return 0


def pose_distance_xyz(a: list[float], b: list[float]) -> float:
    return float(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)) ** 0.5)


def load_experience(args: argparse.Namespace) -> dict:
    if args.no_experience:
        return {}
    path = Path(args.experience_path)
    if not path.is_file():
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        print(f"[WARN] 经验位姿文件读取失败，跳过: {path}: {exc}")
        return {}


def experience_disabled_for_id(args: argparse.Namespace) -> bool:
    disabled = getattr(args, "smooth_experience_disabled_ids", set())
    return int(args.aruco_id) in disabled


def experience_pose_for(args: argparse.Namespace) -> list[float] | None:
    if experience_disabled_for_id(args):
        print(f"[SMOOTH] ArUco ID={args.aruco_id} 已禁用经验位复用，直接走扫描流程")
        return None
    data = load_experience(args)
    item = data.get(str(int(args.aruco_id)))
    if not isinstance(item, dict):
        return None
    source = str(item.get("source", ""))
    if not source.startswith("scan_path_y_"):
        print(
            f"[SMOOTH] 忽略旧版经验位姿 source={source!r}；"
            "仅复用按扫描轨迹 Y 生成的新经验位"
        )
        return None
    pose = item.get("stop_pose")
    if not isinstance(pose, (list, tuple)) or len(pose) < 6:
        return None
    try:
        pose6 = [float(pose[i]) for i in range(6)]
    except Exception:
        return None
    expected, _t, inside = scan_pose_for_target_y(
        pose6[1],
        list(args.demo_start),
        list(args.demo_end),
    )
    if not inside or len(expected) < 6:
        print("[SMOOTH] 忽略经验位姿：该 Y 不在当前扫描轨迹范围内")
        return None
    max_err = max(abs(float(pose6[i]) - float(expected[i])) for i in range(6))
    if max_err > 1e-3:
        print(
            f"[SMOOTH] 忽略经验位姿：坐标不在当前扫描轨迹上 "
            f"max_err={max_err:.3f}mm/deg"
        )
        return None
    return pose6


def save_experience_pose(
    args: argparse.Namespace,
    stop_pose: list[float],
    source: str,
    score: float,
) -> None:
    if args.no_experience:
        return
    if experience_disabled_for_id(args):
        print(f"[SMOOTH] ArUco ID={args.aruco_id} 已禁用经验位写入，本次不保存经验位置")
        return
    path = Path(args.experience_path)
    data = load_experience(args)
    data[str(int(args.aruco_id))] = {
        "aruco_id": int(args.aruco_id),
        "stop_pose": [round(float(x), 3) for x in stop_pose[:6]],
        "source": source,
        "score": round(float(score), 3),
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
        tmp.replace(path)
        print(
            f"[SMOOTH] 已更新 ArUco ID={args.aruco_id} 经验停止位姿: "
            f"{[round(x, 2) for x in stop_pose[:6]]} -> {path}"
        )
    except Exception as exc:
        print(f"[WARN] 经验位姿文件写入失败: {path}: {exc}")


def scan_pose_for_target_y(
    target_y: float,
    start_pose: list[float],
    end_pose: list[float],
) -> tuple[list[float], float, bool]:
    start = [float(x) for x in start_pose[:6]]
    end = [float(x) for x in end_pose[:6]]
    dy = end[1] - start[1]
    if abs(dy) < 1e-6:
        if abs(float(target_y) - start[1]) < 1e-6:
            return list(start), 0.0, True
        return [], 0.0, False
    t_raw = (float(target_y) - start[1]) / dy
    if t_raw < 0.0 or t_raw > 1.0:
        return [], t_raw, False
    pose = [start[i] + (end[i] - start[i]) * t_raw for i in range(6)]
    return pose, t_raw, True


def experience_pose_from_quickchanger_result(
    args: argparse.Namespace,
) -> tuple[list[float] | None, str, float | None]:
    info = getattr(args, "quickchanger_last_move_info", None)
    if not isinstance(info, dict):
        return None, "quickchanger_no_move_info", None

    actual_pose = info.get("actual_after_pose")
    final_pose = info.get("final_pose")
    use_actual = isinstance(actual_pose, list) and len(actual_pose) >= 2
    source_pose = actual_pose if use_actual else final_pose
    if not isinstance(source_pose, list) or len(source_pose) < 2:
        return None, "quickchanger_no_final_y", None

    final_y = float(source_pose[1])
    exp_pose, t, inside = scan_pose_for_target_y(
        final_y,
        list(args.demo_start),
        list(args.demo_end),
    )
    source = "scan_path_y_actual_after" if use_actual else "scan_path_y_final_target"
    if not inside:
        print(
            f"[WARN] 快换落位 Y={final_y:.2f} 超出扫描轨迹 Y 范围，"
            f"无法在当前扫描线段上生成 Y 相同的经验位姿 t={t:.3f}，本次不更新经验位置"
        )
        return None, source + "_outside_scan_path", final_y
    print(
        f"[SMOOTH] 经验位姿按快换落位 Y 生成: y={final_y:.2f}, "
        f"scan_t={t:.3f}, pose={[round(x, 2) for x in exp_pose[:6]]}"
    )
    return exp_pose, source, final_y


def make_match(
    args: argparse.Namespace,
    label: str,
    frame_idx: int,
    result,
    score: float,
    pose: list[float],
    color=None,
    depth=None,
    scan_t: float = 0.0,
    pose_skew_ms: float = 0.0,
    frame_latency_ms: float = 0.0,
):
    return {
        "label": label,
        "score": float(score),
        "pose": pose[:6],
        "frame": frame_idx,
        "color": color,
        "depth": depth,
        "result": result,
        "scan_t": scan_t,
        "pose_skew_ms": pose_skew_ms,
        "frame_latency_ms": frame_latency_ms,
        "rs_frame_no": None,
    }


def detect_at_current_pose(args: argparse.Namespace, cam, label: str):
    color, depth, result, score = capture_detection(args, cam)
    ok = bool(result.ok and result.center_uv is not None and score >= float(args.min_perimeter_px))
    print(
        f"[SMOOTH] {label} 定点复查: "
        f"ok={ok} score={score:.1f} threshold={float(args.min_perimeter_px):.1f}"
    )
    if not ok:
        return None
    if getattr(args, "smooth_require_valid_depth_before_stop", True):
        old_frames = getattr(args, "depth_frames", 7)
        args.depth_frames = int(getattr(args, "smooth_stop_depth_check_frames", 2))
        try:
            depth_m, depth_tag, _ = qc.marker_depth_m(args, result, depth, cam)
        finally:
            args.depth_frames = old_frames
        if depth_m <= 1e-6:
            print(f"[SMOOTH] {label} 定点复查深度无效，改走扫描流程")
            return None
        print(
            f"[SMOOTH] {label} 定点复查深度通过: "
            f"depth={depth_m * 1000.0:.1f}mm tag={depth_tag}"
        )
    pose = read_tcp_pose_light(args.host)
    return make_match(args, label, 0, result, score, pose, color, depth)


def scan_move_for_target(
    args: argparse.Namespace,
    cam,
    target_pose: list[float],
    label: str,
):
    print(f"[SMOOTH] {label} 连续检测 MoveL 目标: {[round(x, 2) for x in target_pose[:6]]}")
    if args.dry_run:
        return detect_at_current_pose(args, cam, label)

    proc = start_continuous_move_worker(args, list(target_pose))
    pose_sampler = None
    if args.pose_sync_mode == "buffer":
        pose_sampler = PoseSampler(args.host, args.pose_sample_hz)
        pose_sampler.start()
        time.sleep(0.05)

    frame_idx = 0
    detected_frames = 0
    valid_frames = 0
    scan_started = time.monotonic()
    stats_started = scan_started
    last_stats_log = scan_started
    last_miss_log = -999
    sample_period = 1.0 / float(args.smooth_sample_hz)
    found = None
    try:
        while True:
            if proc.poll() is not None:
                print(f"[SMOOTH] {label} MoveL 已到目标/结束，未命中目标")
                break

            loop_t0 = time.monotonic()
            frame_idx += 1
            try:
                (
                    color,
                    depth,
                    result,
                    score,
                    tcp_pose,
                    pose_t,
                    pose_skew_ms,
                    pose_sync_mode,
                    frame_latency_ms,
                ) = capture_detection_synced(args, cam, pose_sampler)
            except Exception as exc:
                print(f"[WARN] {label} 同步采样失败: {exc}")
                time.sleep(sample_period)
                continue

            detected = bool(result.ok and result.center_uv is not None)
            ok_for_score = detected and score >= float(args.min_perimeter_px)
            if detected:
                detected_frames += 1
            if ok_for_score:
                valid_frames += 1
                u, v = result.center_uv
                rs_frame_no = getattr(cam, "last_frame_number", None)
                print(
                    f"[SMOOTH] {label} 命中 frame={frame_idx} rs_frame={rs_frame_no} "
                    f"t={pose_t - scan_started:.2f}s id={result.marker_id} "
                    f"center=({u:.1f},{v:.1f}) perimeter={result.perimeter_px:.1f}px "
                    f"score={score:.1f} skew={pose_skew_ms:.1f}ms "
                    f"latency={frame_latency_ms:.1f}ms"
                )
                if getattr(args, "smooth_require_valid_depth_before_stop", True):
                    depth_ok, depth_color, depth_np, depth_result, depth_score = (
                        marker_depth_ok_for_stop(args, cam, label)
                    )
                    if not depth_ok:
                        continue
                    color = depth_color
                    depth = depth_np
                    result = depth_result
                    score = depth_score
                stop_motion_light(args.host)
                time.sleep(0.2)
                stop_pose = read_tcp_pose_light(args.host)
                found = make_match(
                    args,
                    label,
                    frame_idx,
                    result,
                    score,
                    stop_pose,
                    color,
                    depth,
                    pose_t - scan_started,
                    pose_skew_ms,
                    frame_latency_ms,
                )
                found["rs_frame_no"] = rs_frame_no
                print(
                    f"[SMOOTH] {label} 已停止在当前位置: "
                    f"{[round(x, 2) for x in stop_pose[:6]]}"
                )
                break

            if frame_idx == 1 or frame_idx - last_miss_log >= max(1, int(args.smooth_sample_hz)):
                seen_ids = getattr(result, "seen_ids", ()) or ()
                extra = f"，本帧看到其它 id={list(seen_ids)}" if seen_ids else ""
                print(f"[SMOOTH] {label} 尚未检测到目标 ArUco frame={frame_idx}{extra}")
                last_miss_log = frame_idx

            now = time.monotonic()
            if now - last_stats_log >= 1.0:
                duration = max(1e-6, now - stats_started)
                print(
                    f"[SMOOTH] {label} scan_stats frames={frame_idx} "
                    f"fps={frame_idx / duration:.1f} "
                    f"detected={detected_frames} valid={valid_frames}"
                )
                last_stats_log = now

            elapsed = time.monotonic() - loop_t0
            if elapsed < sample_period:
                time.sleep(sample_period - elapsed)
    finally:
        if pose_sampler is not None:
            pose_sampler.stop()
        if proc.poll() is None:
            if found is None:
                try:
                    stop_motion_light(args.host)
                except Exception:
                    pass
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        else:
            proc.wait(timeout=1.0)
        drain_worker_output(proc)
    return found


def choose_nearest_endpoint(current_pose: list[float], start_pose: list[float], end_pose: list[float]):
    d_start = pose_distance_xyz(current_pose, start_pose)
    d_end = pose_distance_xyz(current_pose, end_pose)
    if d_start <= d_end:
        return list(start_pose), list(end_pose), "扫描起点", "扫描终点", d_start, d_end
    return list(end_pose), list(start_pose), "扫描终点", "扫描起点", d_end, d_start


def run_smooth_demo(args: argparse.Namespace) -> int:
    bad = validate_args(args)
    if bad:
        return bad

    T_c2g = qc.ppm.load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper（来自 calibration/sessions/active/）")
    cam, profile = qc.open_from_namespace(args)
    qc.print_stream_info(cam, profile)
    rob = qc._load_robot(args.host)

    try:
        print(f"[SMOOTH] 目标 ArUco ID={args.aruco_id}, dictionary={args.dictionary}")
        print(
            "[SMOOTH] 连续扫描参数:",
            f"vel={float(args.smooth_scan_vel):.1f}",
            f"experience_vel={float(args.smooth_experience_move_vel):.1f}",
            f"sample_hz={float(args.smooth_sample_hz):.1f}",
            f"stop_on_lost={bool(args.smooth_stop_on_lost)}",
            f"pose_sync={args.pose_sync_mode}",
            f"pose_sample_hz={float(args.pose_sample_hz):.1f}",
            f"frame_offset={float(args.frame_time_offset_s):.3f}s",
            f"detect_scale={float(args.scan_detect_scale):.2f}",
            f"depth_before_stop={bool(args.smooth_require_valid_depth_before_stop)}",
            f"depth_check_frames={int(args.smooth_stop_depth_check_frames)}",
            f"experience={'off' if args.no_experience else str(args.experience_path)}",
        )
        best = None
        exp_pose = experience_pose_for(args)
        close_robot_rpc(rob)
        rob = None

        if exp_pose is not None:
            print(
                f"[SMOOTH] 找到 ArUco ID={args.aruco_id} 经验停止位姿，优先复用: "
                f"{[round(x, 2) for x in exp_pose[:6]]}"
            )
            rob = qc._load_robot(args.host)
            err = move_or_dry_run(
                args,
                rob,
                exp_pose,
                "移动到经验位姿（到位后再检测）",
                float(args.smooth_experience_move_vel),
            )
            close_robot_rpc(rob)
            rob = None
            if err != 0:
                print("[WARN] 移动到经验位姿失败，改走常规扫描端点")
            else:
                best = detect_at_current_pose(args, cam, "经验位姿")
            if best is not None:
                print("[SMOOTH] 经验位姿命中目标，跳过常规扫描")

        if best is None:
            current_pose = read_tcp_pose_light(args.host)
            near_pose, far_pose, near_label, far_label, d_near, d_far = choose_nearest_endpoint(
                current_pose,
                list(args.demo_start),
                list(args.demo_end),
            )
            print(
                f"[SMOOTH] 经验位姿不可用/未命中；当前更接近{near_label} "
                f"(near={d_near:.1f}mm, far={d_far:.1f}mm)"
            )
            best = scan_move_for_target(args, cam, near_pose, f"前往{near_label}")
            if best is None:
                best = detect_at_current_pose(args, cam, near_label)
            if best is None:
                best = scan_move_for_target(args, cam, far_pose, f"{near_label}到{far_label}扫描")

        if best is None:
            print("[FAIL] 经验位姿与双向扫描过程中均未检测到满足阈值的 ArUco")
            return 1

        print(
            f"[SMOOTH] 选定停止视角 source={best.get('label', '')} "
            f"frame={best.get('frame')} rs_frame={best.get('rs_frame_no')} "
            f"t={best.get('scan_t', 0.0):.2f}s score={best['score']:.1f} "
            f"pose={[round(x, 2) for x in best['pose']]}"
        )
        rob = qc._load_robot(args.host)
        print("[SMOOTH] 已停在选定位置；不回到历史中间视角")
        time.sleep(0.2)

        color, depth, result, score = capture_detection(args, cam)
        if not result.ok or score < float(args.min_perimeter_px):
            print(
                f"[FAIL] 停止视角复查 ArUco 不稳定，score={score:.1f}，"
                "未执行 quickchanger"
            )
            return 1

        out = qc.ACTIVE_DIR / "quickchanger_smooth_best.jpg"
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), qc.draw_aruco_overlay(color, result))
        print(f"[SMOOTH] 停止视角标注图 {out}")
        pre_stage1_pose = read_tcp_pose_light(args.host)
        print(
            "[SMOOTH] 进入 stage1 前 TCP 位姿: "
            f"{[round(x, 2) for x in pre_stage1_pose[:6]]}"
        )
        print("[SMOOTH] 调用 quickchanger 两阶段移动逻辑")
        ret = run_quickchanger_with_depth_retries(
            args, cam, T_c2g, rob, color, depth, result, score
        )
        if ret == 0:
            exp_save_pose, exp_source, exp_y = experience_pose_from_quickchanger_result(args)
            if exp_save_pose is None:
                print(
                    "[WARN] 无法按快换落位 Y 在扫描轨迹上生成经验位姿，"
                    "本次不更新经验位置"
                )
            else:
                save_experience_pose(args, exp_save_pose, exp_source, score)
                if exp_y is not None:
                    print(f"[SMOOTH] 经验位姿对应快换落位 Y={exp_y:.2f} mm")
        return ret
    finally:
        close_robot_rpc(rob)
        cam.stop()


def main() -> int:
    argv = sys.argv[1:]
    args = build_arg_parser().parse_args(argv)
    apply_smooth_config(args, argv)
    if args._smooth_move_worker:
        return run_move_worker(args)
    return run_smooth_demo(args)


if __name__ == "__main__":
    raise SystemExit(main())
