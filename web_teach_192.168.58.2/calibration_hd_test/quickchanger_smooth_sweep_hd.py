#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Smooth demo sweep for quickchanger.

Unlike quickchanger_demo_sweep_hd.py, the scan phase sends one continuous MoveL
from demo_start to demo_end. The parent process keeps sampling RealSense frames
and robot TCP while that MoveL runs, then returns to the best visible ArUco view
and reuses the existing quickchanger two-stage move.
"""
from __future__ import annotations

import argparse
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
    return cfg


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
    return 0


def run_smooth_demo(args: argparse.Namespace) -> int:
    bad = validate_args(args)
    if bad:
        return bad

    T_c2g = qc.ppm.load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper（来自 calibration/sessions/active/）")
    cam, profile = qc.open_from_namespace(args)
    qc.print_stream_info(cam, profile)
    rob = qc._load_robot(args.host)

    best = None
    valid_matches = []
    seen_once = False
    lost_after_seen = 0
    triggered = False
    proc = None
    pose_sampler = None
    frame_idx = 0
    scan_started = time.monotonic()
    stats_started = scan_started
    last_stats_log = scan_started
    detected_frames = 0
    valid_frames = 0
    sample_period = 1.0 / float(args.smooth_sample_hz)

    try:
        print(f"[SMOOTH] 目标 ArUco ID={args.aruco_id}, dictionary={args.dictionary}")
        print(
            "[SMOOTH] 连续扫描参数:",
            f"vel={float(args.smooth_scan_vel):.1f}",
            f"sample_hz={float(args.smooth_sample_hz):.1f}",
            f"lost_after_visible={int(args.lost_after_visible)}",
            f"stop_on_lost={bool(args.smooth_stop_on_lost)}",
            f"pose_sync={args.pose_sync_mode}",
            f"pose_sample_hz={float(args.pose_sample_hz):.1f}",
            f"frame_offset={float(args.frame_time_offset_s):.3f}s",
            f"detect_scale={float(args.scan_detect_scale):.2f}",
        )
        err = move_or_dry_run(
            args,
            rob,
            list(args.demo_start),
            "移动到演示扫描起点",
            float(args.smooth_scan_vel),
        )
        if err != 0:
            return err

        if args.dry_run:
            print("[dry-run] 不下发连续扫描 MoveL；仅采样当前画面约 3 秒")
            dry_end = time.monotonic() + 3.0
        else:
            print("[SMOOTH] 启动连续扫描 MoveL:", [round(x, 2) for x in args.demo_end])
            close_robot_rpc(rob)
            rob = None
            proc = start_continuous_move_worker(args, list(args.demo_end))
            if args.pose_sync_mode == "buffer":
                pose_sampler = PoseSampler(args.host, args.pose_sample_hz)
                pose_sampler.start()
                time.sleep(0.05)
            dry_end = None
        if args.dry_run and args.pose_sync_mode == "buffer":
            pose_sampler = PoseSampler(args.host, args.pose_sample_hz)
            pose_sampler.start()
            time.sleep(0.08)
        scan_started = time.monotonic()
        stats_started = scan_started
        last_stats_log = scan_started

        last_miss_log = -999
        while True:
            scan_move_finished = False
            if args.dry_run:
                if time.monotonic() >= dry_end:
                    scan_move_finished = True
            elif proc is not None and proc.poll() is not None:
                scan_move_finished = True

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
                print(f"[WARN] 同步采样失败: {exc}")
                time.sleep(sample_period)
                continue
            detected = bool(result.ok and result.center_uv is not None)
            ok_for_score = detected and score >= float(args.min_perimeter_px)
            seen_ids = getattr(result, "seen_ids", ()) or ()
            other_ids = [int(x) for x in seen_ids if int(x) != int(args.aruco_id)]
            other_id_ends_window = bool(best is not None and seen_once and other_ids)
            if detected:
                detected_frames += 1
            if ok_for_score and not other_id_ends_window:
                valid_frames += 1

            if other_id_ends_window:
                lost_after_seen = max(lost_after_seen, int(args.lost_after_visible))
                print(
                    "[SMOOTH] 已进入其它 ArUco 编号区域，立即结束目标窗口："
                    f"target_id={args.aruco_id}, other_ids={other_ids}, "
                    f"seen_ids={list(seen_ids)}"
                )
            elif detected:
                u, v = result.center_uv
                rs_frame_no = getattr(cam, "last_frame_number", None)
                print(
                    f"[SMOOTH] frame={frame_idx} rs_frame={rs_frame_no} "
                    f"t={pose_t - scan_started:.2f}s "
                    f"id={result.marker_id} center=({u:.1f},{v:.1f}) "
                    f"perimeter={result.perimeter_px:.1f}px score={score:.1f} "
                    f"pose_sync={pose_sync_mode} skew={pose_skew_ms:.1f}ms "
                    f"frame_latency={frame_latency_ms:.1f}ms"
                )
                if ok_for_score:
                    seen_once = True
                    lost_after_seen = 0
                    match = {
                        "score": score,
                        "pose": tcp_pose[:6],
                        "frame": frame_idx,
                        "color": color,
                        "depth": depth,
                        "result": result,
                        "scan_t": pose_t - scan_started,
                        "pose_skew_ms": pose_skew_ms,
                        "frame_latency_ms": frame_latency_ms,
                        "rs_frame_no": rs_frame_no,
                    }
                    valid_matches.append(match)
                    mid_idx = (len(valid_matches) - 1) // 2
                    best = valid_matches[mid_idx]
                    print(
                        f"[SMOOTH] 记录有效匹配 #{len(valid_matches)}："
                        f"frame={frame_idx}, score={score:.1f}, "
                        f"tcp={[round(x, 2) for x in tcp_pose[:6]]}"
                    )
                    print(
                        f"[SMOOTH] 当前中间匹配视角："
                        f"match_index={mid_idx + 1}/{len(valid_matches)}, "
                        f"frame={best['frame']}, "
                        f"tcp={[round(x, 2) for x in best['pose']]}"
                    )
                elif not ok_for_score:
                    print(
                        f"[SMOOTH] 已识别但低于评分阈值 "
                        f"({score:.1f} < {args.min_perimeter_px:.1f})"
                    )
                    if seen_once:
                        lost_after_seen += 1
                        print(
                            f"[SMOOTH] 目标未达有效阈值，按丢失计数 "
                            f"{lost_after_seen}/{args.lost_after_visible}"
                        )
            else:
                if seen_once:
                    lost_after_seen += 1
                    extra = f"，本帧看到其它 id={list(seen_ids)}" if seen_ids else ""
                    print(
                        f"[SMOOTH] 首次识别后未检测到目标 "
                        f"{lost_after_seen}/{args.lost_after_visible}{extra}"
                    )
                elif frame_idx == 1 or frame_idx - last_miss_log >= max(1, int(args.smooth_sample_hz)):
                    print(f"[SMOOTH] 尚未检测到目标 ArUco frame={frame_idx}")
                    last_miss_log = frame_idx

            if (
                best is not None
                and not args.complete_scan
                and seen_once
                and lost_after_seen >= int(args.lost_after_visible)
            ):
                print("[SMOOTH] 目标连续丢失，结束连续扫描并开始定位")
                triggered = True
                if not args.dry_run and bool(args.smooth_stop_on_lost):
                    stop_motion_light(args.host)
                break

            if scan_move_finished:
                print("[SMOOTH] 连续扫描 MoveL 已到终点，结束采样循环")
                break

            now = time.monotonic()
            if now - last_stats_log >= 1.0:
                duration = max(1e-6, now - stats_started)
                print(
                    f"[SMOOTH] scan_stats frames={frame_idx} "
                    f"fps={frame_idx / duration:.1f} "
                    f"detected={detected_frames} valid={valid_frames} "
                    f"lost={lost_after_seen}"
                )
                last_stats_log = now

            elapsed = time.monotonic() - loop_t0
            if elapsed < sample_period:
                time.sleep(sample_period - elapsed)

        if proc is not None:
            try:
                proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                print("[WARN] 连续扫描 worker 未及时退出，终止子进程")
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
            drain_worker_output(proc)
            if proc.returncode not in (0, None) and best is None:
                print(f"[FAIL] 连续扫描 MoveL 失败 return={proc.returncode}")
                return int(proc.returncode)

        if best is None:
            print("[FAIL] 连续扫描过程中未检测到满足阈值的 ArUco")
            return 1

        duration = max(1e-6, time.monotonic() - stats_started)
        print(
            f"[SMOOTH] 扫描统计 frames={frame_idx}, "
            f"fps={frame_idx / duration:.1f}, "
            f"detected={detected_frames}, valid={valid_frames}"
        )

        if not triggered:
            print("[SMOOTH] 连续扫描结束，使用历史中间匹配视角执行 quickchanger")

        print(
            f"[SMOOTH] 中间匹配视角 frame={best['frame']}, "
            f"rs_frame={best.get('rs_frame_no')}, "
            f"t={best['scan_t']:.2f}s, score={best['score']:.1f}, "
            f"pose_skew={best.get('pose_skew_ms', 0.0):.1f}ms, "
            f"frame_latency={best.get('frame_latency_ms', 0.0):.1f}ms, "
            f"pose={[round(x, 2) for x in best['pose']]}"
        )
        if rob is None:
            rob = qc._load_robot(args.host)
        err = move_or_dry_run(
            args,
            rob,
            best["pose"],
            "回到/保持最佳视角",
            float(args.smooth_scan_vel),
        )
        if err != 0:
            return err
        time.sleep(0.1)

        color, depth, result, score = capture_detection(args, cam)
        if not result.ok or score < float(args.min_perimeter_px):
            print(
                f"[FAIL] 最佳视角复查 ArUco 不稳定，score={score:.1f}，"
                "未执行 quickchanger"
            )
            return 1

        out = qc.ACTIVE_DIR / "quickchanger_smooth_best.jpg"
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), qc.draw_aruco_overlay(color, result))
        print(f"[SMOOTH] 最佳视角标注图 {out}")
        print("[SMOOTH] 调用 quickchanger 两阶段移动逻辑")
        return qc.move_to_quickchanger_target(args, result, color, depth, cam, T_c2g, rob)
    finally:
        if pose_sampler is not None:
            pose_sampler.stop()
        if proc is not None and proc.poll() is None:
            try:
                stop_motion_light(args.host)
            except Exception:
                pass
            proc.terminate()
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
