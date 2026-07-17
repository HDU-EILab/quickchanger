#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Demo sweep for quickchanger.

Move through a fixed presentation sweep, watch for the requested ArUco marker,
then run the existing quickchanger two-stage move from the best visible view.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_DEV_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DEV_DIR))

import quickchanger_aruco_hd as qc  # noqa: E402


DEFAULT_START = [-769.0, -82.0, 1118.0, -159.0, 16.0, 45.0]
DEFAULT_END = [-769.0, 487.0, 1118.0, -159.0, 16.0, 45.0]
DEFAULT_SCAN_STEPS = 56
DEFAULT_LOST_AFTER_VISIBLE = 4


def build_arg_parser() -> argparse.ArgumentParser:
    ap = qc.build_arg_parser()
    ap.description = "Quickchanger demo sweep: scan for ArUco, then run quickchanger"
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
        "--scan-steps",
        type=int,
        default=None,
        help="起点到终点的分段数；不传则读取 config",
    )
    ap.add_argument(
        "--scan-vel",
        type=float,
        default=20.0,
        help="演示扫描 MoveL 速度，默认 20",
    )
    ap.add_argument(
        "--settle-s",
        type=float,
        default=0.05,
        help="每个扫描点到位后等待相机稳定时间，默认 0.05s",
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
        help="第一次识别后，连续多少个扫描点未识别即结束扫描并开始定位；不传则读取 config",
    )
    ap.add_argument(
        "--complete-scan",
        action="store_true",
        help="不按连续丢失提前触发，完整扫到终点后回到最佳视角再执行 quickchanger",
    )
    return ap


def apply_demo_config(args: argparse.Namespace, argv: list[str]):
    cfg = qc.apply_quickchanger_config(args, argv)
    demo_start = qc.config_vec(cfg, "demo_start", 6)
    demo_end = qc.config_vec(cfg, "demo_end", 6)
    if args.demo_start is None:
        args.demo_start = demo_start if demo_start is not None else list(DEFAULT_START)
    if args.demo_end is None:
        args.demo_end = demo_end if demo_end is not None else list(DEFAULT_END)
    if args.scan_steps is None:
        args.scan_steps = int(cfg.get("scan_steps", DEFAULT_SCAN_STEPS))
    if args.lost_after_visible is None:
        args.lost_after_visible = int(
            cfg.get("lost_after_visible", DEFAULT_LOST_AFTER_VISIBLE)
        )
    return cfg


def interpolate_pose(start: list[float], end: list[float], t: float) -> list[float]:
    s = np.array(start, dtype=np.float64)
    e = np.array(end, dtype=np.float64)
    return (s + (e - s) * float(t)).tolist()


def aruco_score(result) -> float:
    if not result.ok or result.center_uv is None or result.corners is None:
        return 0.0
    pts = result.corners.reshape(4, 2).astype(np.float64)
    area = abs(float(cv2.contourArea(pts.astype(np.float32))))
    if area <= 1.0:
        return 0.0
    side_lengths = [
        float(np.linalg.norm(pts[(i + 1) % 4] - pts[i]))
        for i in range(4)
    ]
    mean_side = float(np.mean(side_lengths))
    side_cv = float(np.std(side_lengths) / (mean_side + 1e-9))
    square_quality = max(0.5, 1.0 - side_cv)
    return float(result.perimeter_px) * square_quality


def capture_detection(args, cam):
    color, depth = cam.read()
    result = qc.detect_aruco_center(
        color,
        marker_id=args.aruco_id,
        dictionary_name=args.dictionary,
    )
    score = aruco_score(result)
    return color, depth, result, score


def move_or_dry_run(args, rob, pose: list[float], label: str) -> int:
    print(f"[DEMO] {label}: {[round(x, 2) for x in pose]}")
    if args.dry_run:
        return 0
    err = qc.ppm.move_tcp(rob, pose, args.scan_vel, args.tool, args.user)
    if err != 0:
        print(f"[FAIL] {label} MoveL 错误码 {err}")
    return err


def run_demo(args: argparse.Namespace) -> int:
    if args.aruco_id_pos is not None:
        args.aruco_id = int(args.aruco_id_pos)
    if args.scan_steps < 1:
        print("[FAIL] --scan-steps 必须 >= 1")
        return 2
    if args.lost_after_visible < 1:
        print("[FAIL] --lost-after-visible 必须 >= 1")
        return 2

    T_c2g = qc.ppm.load_T_cam2gripper()
    print("[OK] 已加载 T_cam2gripper（来自 calibration/sessions/active/）")
    cam, profile = qc.open_from_namespace(args)
    qc.print_stream_info(cam, profile)
    rob = qc._load_robot(args.host)
    state_rob = qc._load_robot(args.host)

    best = None
    seen_once = False
    lost_after_seen = 0
    triggered = False

    try:
        print(f"[DEMO] 目标 ArUco ID={args.aruco_id}, dictionary={args.dictionary}")
        err = move_or_dry_run(args, rob, list(args.demo_start), "移动到演示扫描起点")
        if err != 0:
            return err
        time.sleep(max(0.0, float(args.settle_s)))

        frame_idx = 0
        last_miss_log = -999
        for step in range(1, args.scan_steps + 1):
            frame_idx = step
            t = step / float(args.scan_steps)
            pose = interpolate_pose(list(args.demo_start), list(args.demo_end), t)
            err = move_or_dry_run(args, rob, pose, f"扫描 {step}/{args.scan_steps}")
            if err != 0:
                return err
            time.sleep(max(0.0, float(args.settle_s)))

            color, depth, result, score = capture_detection(args, cam)
            detected = bool(result.ok and result.center_uv is not None)
            ok_for_score = detected and score >= float(args.min_perimeter_px)
            if detected:
                seen_once = True
                lost_after_seen = 0
                u, v = result.center_uv
                try:
                    _, tcp_pose = qc.ppm.read_robot_state(state_rob, "tcp")
                except Exception as exc:
                    print(f"[WARN] 读取扫描中 TCP 失败: {exc}")
                    tcp_pose = None
                print(
                    f"[DEMO] frame={frame_idx} 检测到 id={result.marker_id} "
                    f"center=({u:.1f},{v:.1f}) "
                    f"perimeter={result.perimeter_px:.1f}px score={score:.1f}"
                )
                if ok_for_score and tcp_pose is not None and (
                    best is None or score > best["score"]
                ):
                    best = {
                        "score": score,
                        "pose": tcp_pose[:6],
                        "frame": frame_idx,
                        "color": color,
                        "depth": depth,
                        "result": result,
                    }
                    print(
                        f"[DEMO] 更新最佳视角：frame={frame_idx}, "
                        f"score={score:.1f}, tcp={[round(x, 2) for x in tcp_pose[:6]]}"
                    )
                elif not ok_for_score:
                    print(
                        f"[DEMO] 已识别但低于最佳评分阈值 "
                        f"({score:.1f} < {args.min_perimeter_px:.1f})"
                    )
            else:
                if seen_once:
                    lost_after_seen += 1
                    print(
                        f"[DEMO] 首次识别后未检测到目标 "
                        f"{lost_after_seen}/{args.lost_after_visible}"
                    )
                else:
                    if step == 1 or step - last_miss_log >= 5:
                        print(f"[DEMO] 尚未检测到目标 ArUco ({step}/{args.scan_steps})")
                        last_miss_log = step

            if (
                best is not None
                and not args.complete_scan
                and seen_once
                and lost_after_seen >= int(args.lost_after_visible)
            ):
                print("[DEMO] 目标连续丢失，结束扫描并开始 quickchanger 定位")
                triggered = True
                break

        if best is None:
            print("[FAIL] 扫描过程中未检测到满足阈值的 ArUco")
            return 1

        if not triggered:
            print("[DEMO] 扫描结束，使用历史最佳视角执行 quickchanger")

        print(
            f"[DEMO] 最佳视角 frame={best['frame']}, score={best['score']:.1f}, "
            f"pose={[round(x, 2) for x in best['pose']]}"
        )
        err = move_or_dry_run(args, rob, best["pose"], "回到/保持最佳视角")
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

        out = qc.ACTIVE_DIR / "quickchanger_demo_best.jpg"
        out.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out), qc.draw_aruco_overlay(color, result))
        print(f"[DEMO] 最佳视角标注图 {out}")
        print("[DEMO] 调用 quickchanger 两阶段移动逻辑")
        return qc.move_to_quickchanger_target(args, result, color, depth, cam, T_c2g, rob)
    finally:
        cam.stop()


def main() -> int:
    argv = sys.argv[1:]
    args = build_arg_parser().parse_args(argv)
    apply_demo_config(args, argv)
    return run_demo(args)


if __name__ == "__main__":
    raise SystemExit(main())
