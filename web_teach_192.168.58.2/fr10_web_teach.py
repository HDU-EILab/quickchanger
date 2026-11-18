#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR10 网页示教器（TCP + 关节程序点）

风格参考 Realman downloads 下 web_teach 脚本：单文件 Python + 内嵌 HTML。
1) 读取当前 TCP 位姿；MoveL；页面内 nudge 位姿
2) 关节程序点（MoveJ）：与项目根目录 step 一致的 #home、#A/#B/#C 各 pre / 工作位 / out
3) 服务端硬性依赖：执行 #A 前须落在 #A pre（或从 #A out 经 confirm_no_fixture_at_target 进入 #A）；执行 #A 后须先成功执行 #A pre 或 #A out 才允许其它运动；#B、#C 同理
4) 下发 #A pre / #B pre / #C pre 时：若当前在任一 out，或在对应工作位去 pre，须在请求中带 confirm_gripper_clear（网页端弹窗「快换工具未装配」确认后重试）
5) 当前关节在 #A out / #B out / #C out 时，下发 #A / #B / #C 须在请求中带 confirm_no_fixture_at_target（目标位无快换装置确认）

6) RH56 灵巧手：同目录 rh56_hand.py + set.py（抓紧）+ reset.py（打开）；默认串口 /dev/fr10-hand-serial。电机串口默认 /dev/fr10-motor-serial
7) 位姿轮询仅走 XML-RPC(20003)；运动复用 SDK 长连接，通道异常时自动重连，避免反复占死 20004

规则全文见同目录 WEB_TEACH_RULES.md。

启动：
  conda activate fr10-fairino
  cd /Users/evntcn/FR10/web_teach_192.168.58.2
  python fr10_web_teach.py --port 8890

访问：
  http://<你的电脑IP>:8890
"""

from __future__ import annotations

import atexit
import json
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
import xmlrpc.client
from collections.abc import Sequence
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial  # type: ignore[reportMissingImports]
except Exception:
    serial = None  # pyserial 未安装时在运行期返回清晰错误


FR10_HOST = "192.168.58.2"
FR10_RPC_PORT = 20003
FR10_REALTIME_PORT = 20004
RPC_RECONNECT_PAUSE_S = 0.35
DEFAULT_TOOL = 0
DEFAULT_USER = 0
DEFAULT_VEL = 20.0
DEFAULT_OVL = 100.0
DEMO_START_POSE = [-505.0, 265.0, 934.0, 91.0, -13.0, -2.0]
DEMO_X_EXIT_MM = 240.0
POST_DEMO_MOTOR_FORWARD_PRE_Z_MM = 3.0
POST_DEMO_MOTOR_FORWARD_AFTER_X_MM = 3.0
QUICKCHANGER_DIR = "/home/nvidia/FR10_web_teach_release/web_teach_192.168.58.2/calibration_hd_test"
QUICKCHANGER_SCRIPT = "quickchanger_smooth_sweep_hd.py"
QUICKCHANGER_EXPERIENCE_FILE = "quickchanger_experience_positions.json"
QUICKCHANGER_CONDA_SH = "/home/nvidia/miniconda3/etc/profile.d/conda.sh"
QUICKCHANGER_CONDA_ENV = "rmc-yolo"
QUICKCHANGER_FR10_SITE = "/home/nvidia/miniconda3/envs/fr10-fairino/lib/python3.10/site-packages"
# 与项目根目录 step 文件 #home 一致（末轴 45°）
HOME_JOINT = [-0.001, -85.329, 59.850, -64.537, -90.000, 45.000]
A_PRE_JOINT = [-2.915, -64.967, 33.445, -60.132, -90.013, 41.870]
A_JOINT = [-2.920, -69.404, 48.962, -71.096, -90.015, 41.870]
A_OUT_JOINT = [-3.265, -78.911, 61.741, -74.390, -89.995, 41.498]
B_PRE_JOINT = [-15.490, -66.350, 35.740, -60.955, -89.633, 29.268]
B_JOINT = [-15.490, -70.441, 50.514, -71.632, -89.635, 29.268]
B_OUT_JOINT = [-17.402, -79.855, 62.963, -74.649, -89.581, 27.354]
C_PRE_JOINT = [-27.121, -60.920, 26.878, -57.417, -89.325, 17.642]
C_JOINT = [-27.121, -66.214, 44.247, -69.504, -89.325, 17.642]
C_OUT_JOINT = [-29.990, -75.074, 56.798, -73.106, -89.293, 14.774]

# 判定「已到达某关节预置位」的阈值（度，每轴最大误差）
JOINT_AT_PRE_TOL_DEG = 0.8

GRIPPER_CLEAR_CONFIRM_MSG = "请确定快换工具当前未装配！"
NO_FIXTURE_AT_TARGET_MSG = "请确定目标位置没有快换装置！"

_arm_lock = threading.Lock()
# 硬性依赖：执行 #A / #B / #C 后须先成功执行对应 pre 或 out，才允许其它运动类指令
_deferred_pre: str | None = None  # None | "A" | "B" | "C"
# 运动用长连接：避免 get_pose 轮询反复 RPC()/CloseRPC 占死 20004
_rpc_rob = None
_rpc_mod = None
_demo_return_pose: list[float] | None = None
_motor_forward_z_nudge_pending = False
_current_tool_hint: str | None = None
_quickchanger_jobs_lock = threading.Lock()
_quickchanger_jobs: dict[str, dict] = {}
_quickchanger_job_seq = 0
_QUICKCHANGER_JOB_KEEP = 20
_QUICKCHANGER_JOB_DIR = "/tmp/fr10_quickchanger_jobs"

# 电机串口控制（来源于 motor_control.py 的正反转/停止命令）
MOTOR_DEFAULT_PORT = "/dev/ttyUSB0"
MOTOR_DEFAULT_BAUD = 115200
MOTOR_DEFAULT_TIMEOUT = 0.5
MOTOR_COMMANDS = {
    "forward": "01 10 00 A0 00 07 0E 03 E8 03 E8 40 00 01 F4 03 E8 01 2C 03 E8 EA B8",
    "reverse": "01 10 00 A7 00 07 0E 03 E8 03 E8 40 00 01 F4 03 E8 01 2C 03 E8 5B CD",
    "stop": "01 10 00 04 00 01 02 00 00 A7 D4",
}
_motor_lock = threading.Lock()
_motor_mode = "idle"  # idle | forward | reverse | stop

# RH56 灵巧手（逻辑同目录下 set.py / reset.py + rh56_hand.py；默认稳定逻辑名，避免 ttyUSB 编号漂移）
RH56_DEFAULT_PORT = "/dev/fr10-hand-serial"
_rh56_lock = threading.Lock()


def _rh56_import():
    d = os.path.dirname(os.path.abspath(__file__))
    if d not in sys.path:
        sys.path.insert(0, d)
    from rh56_hand import rh56_grip_sequence, rh56_open_sequence  # noqa: PLC0415

    return rh56_grip_sequence, rh56_open_sequence


def _bus_import():
    d = os.path.dirname(os.path.abspath(__file__))
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        from motor_power_bus import BusActionHandler  # noqa: PLC0415
        return BusActionHandler
    except ImportError:
        from motor_power_bus import MotorPowerBus  # noqa: PLC0415
        return MotorPowerBus


def _normalize_joint_result(ret) -> tuple[int, list[float] | None]:
    if not isinstance(ret, Sequence) or isinstance(ret, (str, bytes)) or len(ret) == 0:
        return -1, None
    try:
        err = int(ret[0])
    except Exception:
        return -1, None
    if err != 0:
        return err, None
    if len(ret) >= 2 and isinstance(ret[1], Sequence) and not isinstance(ret[1], (str, bytes)):
        seq = list(ret[1])
        if len(seq) >= 6:
            return 0, [float(seq[i]) for i in range(6)]
    if len(ret) >= 7:
        return 0, [float(ret[i]) for i in range(1, 7)]
    return 0, None


def _max_joint_err_deg(cur: list[float], target: list[float]) -> float:
    return max(abs(cur[i] - target[i]) for i in range(6))


def _at_joint_target(cur: list[float], target: list[float], tol_deg: float = JOINT_AT_PRE_TOL_DEG) -> bool:
    return _max_joint_err_deg(cur, target) <= tol_deg


def _deferred_error_if_blocked(action: str) -> str | None:
    """若尚未完成必须的 pre/out，则禁止除对应 pre、out 与读位姿外的运动。"""
    if _deferred_pre is None:
        return None
    allowed = {"get_pose"}
    if _deferred_pre == "A":
        allowed.add("movej_a_pre")
        allowed.add("movej_a_out")
    elif _deferred_pre == "B":
        allowed.add("movej_b_pre")
        allowed.add("movej_b_out")
    elif _deferred_pre == "C":
        allowed.add("movej_c_pre")
        allowed.add("movej_c_out")
    if action in allowed:
        return None
    if _deferred_pre == "A":
        return "硬性依赖：已执行 #A，请先执行「#A pre」或「#A out」后再执行其它运动指令。"
    if _deferred_pre == "B":
        return "硬性依赖：已执行 #B，请先执行「#B pre」或「#B out」后再执行其它运动指令。"
    if _deferred_pre == "C":
        return "硬性依赖：已执行 #C，请先执行「#C pre」或「#C out」后再执行其它运动指令。"
    return None


def _movej(
    rob,
    joint: list[float],
    vel: float,
    ovl: float,
    tool: int,
    user: int,
):
    return rob.MoveJ(
        joint_pos=joint,
        tool=tool,
        user=user,
        vel=vel,
        ovl=ovl,
    )


def _movej_ok(ret) -> bool:
    return (ret == 0) or (isinstance(ret, Sequence) and len(ret) > 0 and int(ret[0]) == 0)


def _ret_code(ret) -> int | None:
    if isinstance(ret, (int, float)):
        return int(ret)
    if isinstance(ret, Sequence) and not isinstance(ret, (str, bytes)) and len(ret) > 0:
        try:
            return int(ret[0])
        except Exception:
            return None
    return None


def _is_channel_error(ret) -> bool:
    """SDK 通道未建立时常见 -4。"""
    return _ret_code(ret) == -4


def _load_robot_module():
    global _rpc_mod
    if _rpc_mod is not None:
        return _rpc_mod
    try:
        from fairino import Robot  # type: ignore[reportMissingImports]
        _rpc_mod = Robot
        return Robot
    except Exception:
        import Robot as RobotTop  # type: ignore
        _rpc_mod = RobotTop
        return RobotTop


def _tcp_port_open(host: str, port: int, timeout: float = 0.8) -> bool:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        return True
    except Exception:
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass


def _rpc_close_unlocked() -> None:
    global _rpc_rob
    rob = _rpc_rob
    _rpc_rob = None
    if rob is None:
        return
    try:
        rob.CloseRPC()
    except Exception:
        pass
    # 给控制器释放 20004 一点时间，避免立刻重连撞上半关闭套接字
    time.sleep(RPC_RECONNECT_PAUSE_S)


def _sdk_channel_ok(rob) -> bool:
    """GetSDKComState: (0,0)=正常，(0,1)=异常。勿另开 TCP 探测 20004（控制器通常只允许一路）。"""
    try:
        st = rob.GetSDKComState()
        if isinstance(st, Sequence) and len(st) >= 2 and int(st[0]) == 0:
            return int(st[1]) == 0
    except Exception:
        pass
    try:
        if getattr(rob, "SDK_state", None) is False:
            return False
        if getattr(rob, "sock_cli_state_state", None) is False:
            return False
    except Exception:
        pass
    return True


def _realtime_port_reachable() -> bool:
    """仅在尚无 RPC 长连接时，用于判断控制器实时口是否起来。"""
    return _tcp_port_open(FR10_HOST, FR10_REALTIME_PORT)


def _rpc_open_unlocked():
    global _rpc_rob
    _rpc_close_unlocked()
    if not _realtime_port_reachable():
        raise RuntimeError(
            f"控制器实时数据端口({FR10_REALTIME_PORT})不可达。"
            "请检查控制器上电/急停/报警；确认后可重启示教服务再试。"
        )
    robot_mod = _load_robot_module()
    rob = robot_mod.RPC(FR10_HOST)
    _rpc_rob = rob
    return rob


def _rpc_ensure(force: bool = False):
    """在 _arm_lock 内调用：返回可用的 RPC 对象。"""
    global _rpc_rob
    if force or _rpc_rob is None:
        return _rpc_open_unlocked()
    if not _sdk_channel_ok(_rpc_rob):
        return _rpc_open_unlocked()
    return _rpc_rob


def _get_pose_light() -> dict:
    """仅走 XML-RPC 20003，不占用 20004，供实时位姿轮询使用。"""
    link = f"http://{FR10_HOST}:{FR10_RPC_PORT}"
    try:
        proxy = xmlrpc.client.ServerProxy(link, allow_none=True)
        raw = proxy.GetActualTCPPose(1)
        err, pose = _normalize_pose_result(raw)
        if err == 0 and pose is not None:
            return {"ok": True, "msg": "pose ok", "pose": pose, "via": "xmlrpc"}
        return {"ok": False, "msg": f"GetActualTCPPose failed: {raw}", "via": "xmlrpc"}
    except Exception as e:
        return {"ok": False, "msg": f"xmlrpc get_pose: {e}", "via": "xmlrpc"}


def _atexit_close_rpc() -> None:
    with _arm_lock:
        _rpc_close_unlocked()


atexit.register(_atexit_close_rpc)


def _normalize_pose_result(ret) -> tuple[int, list[float] | None]:
    if not isinstance(ret, Sequence) or isinstance(ret, (str, bytes)) or len(ret) == 0:
        return -1, None
    try:
        err = int(ret[0])
    except Exception:
        return -1, None
    if err != 0:
        return err, None

    if len(ret) >= 2 and isinstance(ret[1], Sequence) and not isinstance(ret[1], (str, bytes)):
        seq = list(ret[1])
        if len(seq) >= 6:
            return 0, [float(seq[i]) for i in range(6)]

    if len(ret) >= 7:
        return 0, [float(ret[i]) for i in range(1, 7)]

    return 0, None


def _read_joint_deg(rob) -> tuple[int, list[float] | None]:
    raw = rob.robot.GetActualJointPosDegree(1)
    return _normalize_joint_result(raw)


def _parse_movej_params(payload: dict) -> tuple[dict | None, str | None]:
    """成功返回 (params_dict, None)，失败返回 (None, error_msg)。"""
    try:
        vel = float(payload.get("v", DEFAULT_VEL))
        ovl = float(payload.get("ovl", DEFAULT_OVL))
        tool = int(payload.get("tool", DEFAULT_TOOL))
        user = int(payload.get("user", DEFAULT_USER))
    except Exception as e:
        return None, f"invalid params: {e}"
    return {"vel": vel, "ovl": ovl, "tool": tool, "user": user}, None


def _parse_motion_params(payload: dict) -> tuple[dict | None, str | None]:
    try:
        vel = float(payload.get("v", DEFAULT_VEL))
        ovl = float(payload.get("ovl", DEFAULT_OVL))
        tool = int(payload.get("tool", DEFAULT_TOOL))
        user = int(payload.get("user", DEFAULT_USER))
    except Exception as e:
        return None, f"invalid params: {e}"
    return {"vel": vel, "ovl": ovl, "tool": tool, "user": user}, None


def _read_tcp_pose_rpc(rob) -> tuple[int, list[float] | None]:
    raw = rob.robot.GetActualTCPPose(1)
    return _normalize_pose_result(raw)


def _move_tcp_delta(rob, payload: dict, delta_xyz: tuple[float, float, float], label: str, sdk: str) -> dict:
    mp, err = _parse_motion_params(payload)
    if err:
        return {"ok": False, "msg": err, "sdk": sdk}
    assert mp is not None
    pe, pose = _read_tcp_pose_rpc(rob)
    if pe != 0 or pose is None:
        return {"ok": False, "msg": f"读取当前 TCP 失败: err={pe}", "sdk": sdk}
    target = list(pose)
    for i, d in enumerate(delta_xyz):
        target[i] += float(d)
    ret = rob.MoveL(
        desc_pos=target,
        tool=mp["tool"],
        user=mp["user"],
        vel=mp["vel"],
        ovl=mp["ovl"],
    )
    ok = _movej_ok(ret)
    return {
        "ok": bool(ok),
        "msg": f"{label} MoveL => {ret}",
        "pose_before": pose,
        "pose_target": target,
        "ret": ret if isinstance(ret, (int, float, str, list, dict)) else str(ret),
        "sdk": sdk,
    }


def _move_tcp_absolute(rob, payload: dict, pose: list[float], label: str, sdk: str) -> dict:
    mp, err = _parse_motion_params(payload)
    if err:
        return {"ok": False, "msg": err, "sdk": sdk}
    assert mp is not None
    target = [float(x) for x in pose[:6]]
    ret = rob.MoveL(
        desc_pos=target,
        tool=mp["tool"],
        user=mp["user"],
        vel=mp["vel"],
        ovl=mp["ovl"],
    )
    ok = _movej_ok(ret)
    return {
        "ok": bool(ok),
        "msg": f"{label} MoveL => {ret}",
        "pose_target": target,
        "ret": ret if isinstance(ret, (int, float, str, list, dict)) else str(ret),
        "sdk": sdk,
    }


def _demo_begin(rob, payload: dict, sdk: str) -> dict:
    global _demo_return_pose
    pe, pose = _read_tcp_pose_rpc(rob)
    if pe != 0 or pose is None:
        return {"ok": False, "msg": f"读取回归位置失败: err={pe}", "sdk": sdk}
    out = _move_tcp_absolute(rob, payload, DEMO_START_POSE, "演示开始", sdk)
    if out.get("ok"):
        _demo_return_pose = [float(x) for x in pose[:6]]
        out["return_pose"] = _demo_return_pose
        out["msg"] = (
            f"已记录回归位置 {[round(x, 2) for x in _demo_return_pose]}；"
            + str(out.get("msg", ""))
        )
    return out


def _demo_end(rob, payload: dict, sdk: str) -> dict:
    global _demo_return_pose
    if _demo_return_pose is None:
        return {"ok": True, "msg": "未记录回归位置，演示结束不执行移动", "sdk": sdk}
    target = list(_demo_return_pose)
    out = _move_tcp_absolute(rob, payload, target, "演示结束回归", sdk)
    if out.get("ok"):
        _demo_return_pose = None
        out["msg"] = "已回到回归位置，并已释放回归位置记录；" + str(out.get("msg", ""))
    return out


def _mark_demo_flow_finished() -> None:
    global _motor_forward_z_nudge_pending
    _motor_forward_z_nudge_pending = True


def _motor_forward_with_post_demo_nudge(payload: dict) -> dict:
    global _motor_forward_z_nudge_pending
    pre_nudge_result = None
    after_nudge_result = None
    with _arm_lock:
        if _motor_forward_z_nudge_pending:
            try:
                rob = _rpc_ensure(force=False)
                sdk = rob.GetSDKVersion()
                pre_nudge_result = _move_tcp_delta(
                    rob,
                    payload,
                    (0.0, 0.0, POST_DEMO_MOTOR_FORWARD_PRE_Z_MM),
                    f"演示后首次电机正转前 Z+={POST_DEMO_MOTOR_FORWARD_PRE_Z_MM:g}",
                    sdk,
                )
            except Exception as e:
                try:
                    _rpc_close_unlocked()
                except Exception:
                    pass
                return {
                    "ok": False,
                    "msg": f"电机正转前 Z+={POST_DEMO_MOTOR_FORWARD_PRE_Z_MM:g}mm 失败: {e}",
                }
            if not pre_nudge_result.get("ok"):
                return {
                    "ok": False,
                    "msg": f"电机正转前 Z+={POST_DEMO_MOTOR_FORWARD_PRE_Z_MM:g}mm 失败，未执行电机正转: "
                    + str(pre_nudge_result.get("msg", "")),
                    "pre_nudge_result": pre_nudge_result,
                }
            _motor_forward_z_nudge_pending = False

    try:
        out = _bus_import().handle_action("motor_forward", payload)
    except Exception as e:
        return {"ok": False, "msg": f"RS485 总线模块执行失败: {e}"}
    if out.get("ok") and pre_nudge_result is not None:
        with _arm_lock:
            try:
                rob = _rpc_ensure(force=False)
                sdk = rob.GetSDKVersion()
                after_nudge_result = _move_tcp_delta(
                    rob,
                    payload,
                    (POST_DEMO_MOTOR_FORWARD_AFTER_X_MM, 0.0, 0.0),
                    f"演示后首次电机正转完成后 X+={POST_DEMO_MOTOR_FORWARD_AFTER_X_MM:g}",
                    sdk,
                )
            except Exception as e:
                try:
                    _rpc_close_unlocked()
                except Exception:
                    pass
                after_nudge_result = {
                    "ok": False,
                    "msg": f"电机正转完成后 X+={POST_DEMO_MOTOR_FORWARD_AFTER_X_MM:g}mm 失败: {e}",
                }
        out["after_nudge_result"] = after_nudge_result
        if not after_nudge_result.get("ok"):
            out["ok"] = False
            out["msg"] = (
                "电机正转已执行成功，但随后 "
                f"X+={POST_DEMO_MOTOR_FORWARD_AFTER_X_MM:g}mm 失败: "
                + str(after_nudge_result.get("msg", ""))
            )
            return out
    if pre_nudge_result is not None:
        out["pre_nudge_result"] = pre_nudge_result
        out["msg"] = (
            f"已先执行演示后首次正转 Z+={POST_DEMO_MOTOR_FORWARD_PRE_Z_MM:g}mm；"
            + str(out.get("msg", ""))
        )
    if after_nudge_result is not None:
        out["msg"] = (
            str(out.get("msg", ""))
            + f"；正转成功后已执行 X+={POST_DEMO_MOTOR_FORWARD_AFTER_X_MM:g}mm"
        )
    return out


def _quickchanger_demo_cmd(aruco_id: int) -> str:
    return (
        f"cd {QUICKCHANGER_DIR!r} && "
        f"source {QUICKCHANGER_CONDA_SH!r} && "
        f"conda activate {QUICKCHANGER_CONDA_ENV!r} && "
        f"export PYTHONPATH={QUICKCHANGER_FR10_SITE!r}:${{PYTHONPATH:-}} && "
        f"python {QUICKCHANGER_SCRIPT!r} --profile hd --headless --aruco-id {int(aruco_id)}"
    )


QUICKCHANGER_TOOL_PRESETS = {
    1: ("灵巧手", "device_switch_hand"),
    2: ("夹爪", "device_switch_gripper"),
    3: ("电钻", "device_switch_drill"),
}


def _set_current_tool_hint(tool_name: str | None) -> None:
    global _current_tool_hint
    if tool_name in ("灵巧手", "夹爪", "电钻"):
        _current_tool_hint = tool_name
    elif tool_name is None:
        _current_tool_hint = None


def _quickchanger_auto_power(aruco_id: int, payload: dict) -> dict:
    if int(aruco_id) == 3:
        _set_current_tool_hint(None)
        return {
            "ok": True,
            "msg": "电钻 ID3 按配置跳过到位后的自动供电切换",
            "tool_name": "电钻",
            "skipped": True,
        }
    preset = QUICKCHANGER_TOOL_PRESETS.get(int(aruco_id))
    if preset is None:
        return {"ok": False, "msg": f"ArUco ID {aruco_id} 未配置自动供电档案"}
    tool_name, action = preset
    power_payload = dict(payload)
    power_payload["confirm_power_on"] = True
    try:
        out = _bus_import().handle_action("power_status", power_payload)
    except Exception as e:
        return {"ok": False, "msg": f"{tool_name}供电检查失败: {e}", "tool_name": tool_name}
    if not out.get("ok"):
        return {"ok": False, "msg": f"{tool_name}供电检查失败: {out.get('msg', '')}", "tool_name": tool_name, "power_status": out}
    try:
        switched = _bus_import().handle_action(action, power_payload)
    except Exception as e:
        return {"ok": False, "msg": f"{tool_name}供电切换失败: {e}", "tool_name": tool_name, "power_status": out}
    switched["tool_name"] = tool_name
    switched["power_checked"] = out
    switched["msg"] = f"已完成 Quickchanger，并已自动切换到{tool_name}供电。{switched.get('msg', '')}"
    if switched.get("ok"):
        _set_current_tool_hint(tool_name)
    return switched


def _quickchanger_prepare_power(aruco_id: int, payload: dict) -> dict | None:
    if int(aruco_id) != 3:
        return None
    _set_current_tool_hint(None)
    power_payload = dict(payload)
    try:
        off = _bus_import().handle_action("power_output_off", power_payload)
    except Exception as e:
        return {"ok": False, "msg": f"电钻 ID3 执行前关闭电源输出失败: {e}", "tool_name": "电钻"}
    if not off.get("ok"):
        return {
            "ok": False,
            "msg": f"电钻 ID3 执行前关闭电源输出失败: {off.get('msg', '')}",
            "tool_name": "电钻",
            "power_off_result": off,
        }
    time.sleep(0.5)
    return {
        "ok": True,
        "msg": f"电钻 ID3 执行前已关闭电源输出。{off.get('msg', '')}",
        "tool_name": "电钻",
        "power_off_result": off,
    }


def _clear_quickchanger_experience(payload: dict) -> dict:
    try:
        aruco_id = int(payload.get("aruco_id", 1))
    except Exception:
        return {"ok": False, "msg": "aruco_id 必须是整数"}
    if aruco_id < 0:
        return {"ok": False, "msg": "aruco_id 不能为负数"}

    path = os.path.join(QUICKCHANGER_DIR, QUICKCHANGER_EXPERIENCE_FILE)
    try:
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                data = {}
        else:
            data = {}
    except Exception as e:
        return {"ok": False, "msg": f"读取经验位文件失败: {e}", "path": path, "aruco_id": aruco_id}

    key = str(aruco_id)
    removed = data.pop(key, None)
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
            f.write("\n")
        os.replace(tmp, path)
    except Exception as e:
        return {"ok": False, "msg": f"写入经验位文件失败: {e}", "path": path, "aruco_id": aruco_id}

    if removed is None:
        msg = f"ArUco ID{aruco_id} 当前没有经验位置"
    else:
        msg = f"已清除 ArUco ID{aruco_id} 经验位置"
    return {
        "ok": True,
        "msg": msg,
        "aruco_id": aruco_id,
        "removed": removed,
        "path": path,
        "remaining_ids": sorted(
            data.keys(),
            key=lambda x: (0, int(x)) if str(x).isdigit() else (1, str(x)),
        ),
    }


def _run_quickchanger_demo(payload: dict) -> dict:
    try:
        aruco_id = int(payload.get("aruco_id", 1))
    except Exception:
        return {"ok": False, "msg": "aruco_id 必须是整数"}
    if aruco_id < 0:
        return {"ok": False, "msg": "aruco_id 不能为负数"}
    with _arm_lock:
        _rpc_close_unlocked()
        prepare_power_result = _quickchanger_prepare_power(aruco_id, payload)
        if prepare_power_result is not None and not prepare_power_result.get("ok"):
            return {
                "ok": False,
                "msg": prepare_power_result.get("msg", "quickchanger 执行前供电准备失败"),
                "aruco_id": aruco_id,
                "prepare_power_result": prepare_power_result,
            }
        cmd = _quickchanger_demo_cmd(aruco_id)
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                cwd=QUICKCHANGER_DIR,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=360,
            )
        except subprocess.TimeoutExpired as e:
            return {
                "ok": False,
                "msg": f"quickchanger demo 超时: {e}",
                "aruco_id": aruco_id,
            }
        except Exception as e:
            return {
                "ok": False,
                "msg": f"quickchanger demo 启动失败: {e}",
                "aruco_id": aruco_id,
            }
        out = proc.stdout or ""
        tail = "\n".join(out.splitlines()[-18:])
        if prepare_power_result is not None:
            tail = "\n".join(
                x
                for x in (
                    f"[PRE_POWER] {prepare_power_result.get('msg', '')}",
                    tail,
                )
                if x
            )
        auto_power = payload.get("auto_power") in (True, 1, "1", "true", "yes", "y")
        power_result = None
        if proc.returncode == 0 and auto_power:
            power_result = _quickchanger_auto_power(aruco_id, payload)
            tail = "\n".join(
                x for x in (tail, f"[AUTO_POWER] {power_result.get('msg', '')}") if x
            )
            if not power_result.get("ok"):
                return {
                    "ok": False,
                    "msg": f"quickchanger demo id={aruco_id} return=0，但自动供电失败\n{tail}",
                    "returncode": proc.returncode,
                    "aruco_id": aruco_id,
                    "power_result": power_result,
                }
        msg_head = f"quickchanger demo id={aruco_id} return={proc.returncode}"
        if proc.returncode == 0 and power_result and power_result.get("ok"):
            msg_head = power_result.get("msg", msg_head)
        _mark_demo_flow_finished()
        return {
            "ok": proc.returncode == 0,
            "msg": f"{msg_head}\n{tail}",
            "returncode": proc.returncode,
            "aruco_id": aruco_id,
            "power_result": power_result,
        }


def _quickchanger_job_snapshot(job_id: str) -> dict | None:
    with _quickchanger_jobs_lock:
        job = _quickchanger_jobs.get(job_id)
        return dict(job) if job else None


def _quickchanger_job_path(job_id: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in job_id)
    return os.path.join(_QUICKCHANGER_JOB_DIR, safe + ".json")


def _quickchanger_job_write(job: dict) -> None:
    job_id = str(job.get("job_id", "")).strip()
    if not job_id:
        return
    try:
        os.makedirs(_QUICKCHANGER_JOB_DIR, exist_ok=True)
        tmp = _quickchanger_job_path(job_id) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(job, f, ensure_ascii=False)
        os.replace(tmp, _quickchanger_job_path(job_id))
    except Exception as exc:
        print(f"[WARN] quickchanger job 状态写入失败: {exc}", flush=True)


def _quickchanger_job_read(job_id: str) -> dict | None:
    try:
        with open(_quickchanger_job_path(job_id), "r", encoding="utf-8") as f:
            job = json.load(f)
        return job if isinstance(job, dict) else None
    except FileNotFoundError:
        return None
    except Exception as exc:
        print(f"[WARN] quickchanger job 状态读取失败: {exc}", flush=True)
        return None


def _quickchanger_job_prune_unlocked() -> None:
    if len(_quickchanger_jobs) <= _QUICKCHANGER_JOB_KEEP:
        return
    done_items = [
        (job.get("finished_at", job.get("started_at", 0.0)), job_id)
        for job_id, job in _quickchanger_jobs.items()
        if job.get("done")
    ]
    for _, job_id in sorted(done_items)[: max(0, len(_quickchanger_jobs) - _QUICKCHANGER_JOB_KEEP)]:
        _quickchanger_jobs.pop(job_id, None)


def _quickchanger_job_worker(job_id: str, payload: dict) -> None:
    with _quickchanger_jobs_lock:
        job = _quickchanger_jobs.get(job_id)
        if job:
            job["status"] = "running"
            job["msg"] = "quickchanger smooth 扫描执行中..."
            _quickchanger_job_write(dict(job))
    try:
        result = _run_quickchanger_demo(payload)
    except Exception as e:
        result = {"ok": False, "msg": f"quickchanger 后台任务异常: {e}"}
    with _quickchanger_jobs_lock:
        job = _quickchanger_jobs.get(job_id)
        if job:
            job["status"] = "done"
            job["done"] = True
            job["finished_at"] = time.time()
            job["result"] = result
            job["ok"] = bool(result.get("ok"))
            job["msg"] = result.get("msg", "")
            _quickchanger_job_write(dict(job))
            _quickchanger_job_prune_unlocked()


def _start_quickchanger_job(payload: dict) -> dict:
    global _quickchanger_job_seq
    try:
        aruco_id = int(payload.get("aruco_id", 1))
    except Exception:
        return {"ok": False, "msg": "aruco_id 必须是整数"}
    if aruco_id < 0:
        return {"ok": False, "msg": "aruco_id 不能为负数"}
    job_payload = dict(payload)
    job_payload["aruco_id"] = aruco_id
    requested_job_id = str(payload.get("job_id", "")).strip()
    with _quickchanger_jobs_lock:
        if requested_job_id and requested_job_id in _quickchanger_jobs:
            job = _quickchanger_jobs[requested_job_id]
            return {
                "ok": True,
                "job_id": requested_job_id,
                "aruco_id": job.get("aruco_id", aruco_id),
                "done": bool(job.get("done")),
                "status": job.get("status", ""),
                "msg": "quickchanger 后台任务已存在",
            }
        _quickchanger_job_seq += 1
        job_id = requested_job_id or f"qc-{int(time.time() * 1000)}-{_quickchanger_job_seq}"
        _quickchanger_jobs[job_id] = {
            "ok": True,
            "job_id": job_id,
            "aruco_id": aruco_id,
            "status": "pending",
            "done": False,
            "started_at": time.time(),
            "msg": "quickchanger 后台任务已创建",
        }
        _quickchanger_job_write(dict(_quickchanger_jobs[job_id]))
    th = threading.Thread(
        target=_quickchanger_job_worker,
        args=(job_id, job_payload),
        daemon=True,
    )
    th.start()
    return {
        "ok": True,
        "job_id": job_id,
        "aruco_id": aruco_id,
        "done": False,
        "msg": "quickchanger 后台任务已启动",
    }


def _get_quickchanger_job(payload: dict) -> dict:
    job_id = str(payload.get("job_id", "")).strip()
    if not job_id:
        return {"ok": False, "msg": "缺少 job_id"}
    job = _quickchanger_job_snapshot(job_id)
    if not job:
        job = _quickchanger_job_read(job_id)
    if not job:
        return {"ok": False, "msg": f"找不到 quickchanger 任务: {job_id}", "job_id": job_id}
    job["ok"] = True
    return job


def _truthy_confirm_gripper(payload: dict) -> bool:
    v = payload.get("confirm_gripper_clear")
    if v is True:
        return True
    if isinstance(v, (int, float)) and int(v) == 1:
        return True
    if isinstance(v, str) and v.strip().lower() in ("1", "true", "yes", "y"):
        return True
    return False


def _truthy_confirm_no_fixture_at_target(payload: dict) -> bool:
    v = payload.get("confirm_no_fixture_at_target")
    if v is True:
        return True
    if isinstance(v, (int, float)) and int(v) == 1:
        return True
    if isinstance(v, str) and v.strip().lower() in ("1", "true", "yes", "y"):
        return True
    return False


def _motor_cmd(action: str, payload: dict) -> dict:
    global _motor_mode
    if serial is None:
        return {"ok": False, "msg": "pyserial 未安装，无法执行电机控制。请先 pip install pyserial。"}
    if action not in ("forward", "reverse", "stop"):
        return {"ok": False, "msg": f"unknown motor action: {action}"}

    mode_map = {"forward": "forward", "reverse": "reverse", "stop": "stop"}
    target_mode = mode_map[action]
    if _motor_mode == target_mode:
        return {"ok": True, "msg": f"电机已处于 {target_mode} 模式（忽略重复指令）。", "mode": _motor_mode}

    try:
        port = str(payload.get("motor_port", MOTOR_DEFAULT_PORT)).strip()
        baud = int(payload.get("motor_baud", MOTOR_DEFAULT_BAUD))
        timeout = float(payload.get("motor_timeout", MOTOR_DEFAULT_TIMEOUT))
    except Exception as e:
        return {"ok": False, "msg": f"invalid motor params: {e}"}

    if not port:
        return {"ok": False, "msg": "请先填写电机串口端口（默认 /dev/ttyUSB0，可在页面修改）。"}

    cmd_hex = MOTOR_COMMANDS[action]
    with _motor_lock:
        try:
            with serial.Serial(port, baud, timeout=timeout) as ser:
                ser.reset_input_buffer()
                ser.write(bytes.fromhex(cmd_hex))
                time.sleep(0.1)
                resp = ser.read(ser.in_waiting or 20)
            _motor_mode = target_mode
            return {
                "ok": True,
                "msg": f"电机 {action} 指令已发送",
                "mode": _motor_mode,
                "motor_port": port,
                "resp_hex": resp.hex().upper() if resp else "",
            }
        except Exception as e:
            return {"ok": False, "msg": f"电机串口下发失败: {e}", "motor_port": port}


def _rh56_cmd(mode: str, payload: dict) -> dict:
    """mode: grip（抓紧）| open（打开）"""
    if serial is None:
        return {"ok": False, "msg": "pyserial 未安装，无法控制 RH56。请先 pip install pyserial。"}
    if mode not in ("grip", "open"):
        return {"ok": False, "msg": f"unknown rh56 mode: {mode}"}
    try:
        raw = payload.get("rh56_port")
        port = (
            RH56_DEFAULT_PORT
            if raw is None or str(raw).strip() == ""
            else str(raw).strip()
        )
        hand_id = int(payload.get("rh56_hand_id", 1))
    except Exception as e:
        return {"ok": False, "msg": f"invalid rh56 params: {e}"}
    try:
        grip, open_ = _rh56_import()
    except Exception as e:
        return {"ok": False, "msg": f"无法加载 rh56_hand 模块: {e}"}
    with _rh56_lock:
        try:
            if mode == "grip":
                grip(port, hand_id=hand_id)
                return {"ok": True, "msg": "RH56 抓紧指令已发送", "rh56_port": port}
            open_(port, hand_id=hand_id)
            return {"ok": True, "msg": "RH56 打开指令已发送", "rh56_port": port}
        except Exception as e:
            return {"ok": False, "msg": f"RH56 串口执行失败: {e}", "rh56_port": port}


def _infer_current_tool(payload: dict) -> dict:
    hinted = str(payload.get("tool_hint") or payload.get("current_tool") or "").strip()
    if hinted in ("灵巧手", "夹爪", "电钻"):
        return {"ok": True, "tool": hinted, "source": "payload_hint"}
    if _current_tool_hint in ("灵巧手", "夹爪", "电钻"):
        return {"ok": True, "tool": _current_tool_hint, "source": "server_hint"}

    try:
        out = _bus_import().handle_action("power_status", payload)
    except Exception as e:
        return {"ok": False, "msg": f"读取末端供电状态失败: {e}"}
    if not out.get("ok"):
        return {"ok": False, "msg": f"读取末端供电状态失败: {out.get('msg', '')}", "power_status": out}

    current_device = str(out.get("current_device") or "").strip()
    if current_device in ("灵巧手", "夹爪", "电钻"):
        return {"ok": True, "tool": current_device, "source": "device_state", "power_status": out}

    ps = out.get("power_status") if isinstance(out.get("power_status"), dict) else {}
    output_on = ps.get("output_setting") == 1
    voltage = ps.get("realtime_voltage_v")
    try:
        voltage_f = float(voltage)
    except Exception:
        voltage_f = None
    if not output_on or voltage_f is None:
        return {
            "ok": False,
            "msg": "无法判断当前末端设备：供电输出未开启或电压未知，请先执行对应设备切换。",
            "power_status": out,
        }
    if 7.0 <= voltage_f <= 10.5:
        return {"ok": True, "tool": "灵巧手", "source": "voltage", "power_status": out}
    if 20.5 <= voltage_f <= 26.5:
        return {"ok": True, "tool": "夹爪", "source": "voltage", "power_status": out}
    if 17.0 <= voltage_f <= 20.5:
        return {
            "ok": False,
            "msg": f"当前疑似电钻供电 {voltage_f:.2f}V，不执行抓紧/打开动作。",
            "power_status": out,
        }
    return {
        "ok": False,
        "msg": f"无法根据当前电压 {voltage_f:.2f}V 判断末端设备，请先切换设备。",
        "power_status": out,
    }


def _tool_open_close_auto(mode: str, payload: dict) -> dict:
    if mode not in ("open", "close"):
        return {"ok": False, "msg": f"unknown tool mode: {mode}"}
    inferred = _infer_current_tool(payload)
    if not inferred.get("ok"):
        return inferred

    tool = inferred.get("tool")
    if tool == "灵巧手":
        out = _rh56_cmd("open" if mode == "open" else "grip", payload)
        out["current_tool"] = tool
        out["detect_source"] = inferred.get("source")
        return out
    if tool == "夹爪":
        action = "gripper_open" if mode == "open" else "gripper_close"
        out = _bus_import().handle_action(action, {**payload, "auto_switch_device": True})
        out["current_tool"] = tool
        out["detect_source"] = inferred.get("source")
        return out
    return {
        "ok": False,
        "msg": f"当前设备是{tool}，不支持抓紧/打开自动动作。",
        "current_tool": tool,
    }


def _fr10_request(action: str, payload: dict) -> dict:
    """位姿走轻量 XML-RPC；运动复用长连接，通道异常时自动重连并重试一次。"""
    global _deferred_pre
    bus_actions = {
        "motor_reverse",
        "motor_stop",
        "motor_status",
        "motor_lock_diagnosis",
        "motor_reset_diagnosis",
        "bus_monitor",
        "power_set_voltage",
        "power_set_current",
        "power_output_on",
        "power_output_off",
        "power_status",
        "device_switch_hand",
        "device_switch_gripper",
        "device_switch_drill",
        "drill_start",
        "drill_pulse",
        "gripper_open",
        "gripper_close",
        "gripper_demo",
    }
    if action == "motor_forward":
        return _motor_forward_with_post_demo_nudge(payload)
    if action in bus_actions:
        try:
            out = _bus_import().handle_action(action, payload)
            if out.get("ok"):
                if action == "device_switch_hand":
                    _set_current_tool_hint("灵巧手")
                elif action == "device_switch_gripper":
                    _set_current_tool_hint("夹爪")
                elif action == "device_switch_drill":
                    _set_current_tool_hint("电钻")
                elif action in ("power_output_off", "drill_pulse"):
                    _set_current_tool_hint(None)
            return out
        except Exception as e:
            return {"ok": False, "msg": f"RS485 总线模块执行失败: {e}"}
    if action == "rh56_grip":
        return _rh56_cmd("grip", payload)
    if action == "rh56_open":
        return _rh56_cmd("open", payload)
    if action == "tool_close_auto":
        return _tool_open_close_auto("close", payload)
    if action == "tool_open_auto":
        return _tool_open_close_auto("open", payload)
    if action == "quickchanger_demo_start":
        return _start_quickchanger_job(payload)
    if action == "quickchanger_demo_status":
        return _get_quickchanger_job(payload)
    if action == "quickchanger_clear_experience":
        return _clear_quickchanger_experience(payload)
    if action == "quickchanger_demo":
        return _run_quickchanger_demo(payload)
    if action == "get_pose":
        return _get_pose_light()

    with _arm_lock:
        try:
            rob = _rpc_ensure(force=False)
            sdk = rob.GetSDKVersion()

            motion_actions = {
                "movel_pose",
                "move_tcp_z_plus_80",
                "move_tcp_x_plus_80",
                "move_tcp_x_minus_80",
                "movej_home",
                "movej_a_pre",
                "movej_a",
                "movej_a_out",
                "movej_b_pre",
                "movej_b",
                "movej_b_out",
                "movej_c_pre",
                "movej_c",
                "movej_c_out",
            }
            if action in motion_actions:
                block = _deferred_error_if_blocked(action)
                if block:
                    return {"ok": False, "msg": block, "sdk": sdk}

            def _run_motion(rob_cur, sdk_cur):
                global _deferred_pre
                if action == "movel_pose":
                    pose = payload.get("pose", [])
                    if not isinstance(pose, list) or len(pose) != 6:
                        return {"ok": False, "msg": "need pose[6]"}
                    try:
                        pose6 = [float(v) for v in pose]
                        vel = float(payload.get("v", DEFAULT_VEL))
                        ovl = float(payload.get("ovl", DEFAULT_OVL))
                        tool = int(payload.get("tool", DEFAULT_TOOL))
                        user = int(payload.get("user", DEFAULT_USER))
                    except Exception as e:
                        return {"ok": False, "msg": f"invalid params: {e}"}

                    ret = rob_cur.MoveL(desc_pos=pose6, tool=tool, user=user, vel=vel, ovl=ovl)
                    ok = _movej_ok(ret)
                    return {
                        "ok": bool(ok),
                        "msg": f"MoveL => {ret}",
                        "ret": ret if isinstance(ret, (int, float, str, list, dict)) else str(ret),
                        "sdk": sdk_cur,
                    }

                if action == "move_tcp_z_plus_80":
                    return _move_tcp_delta(rob_cur, payload, (0.0, 0.0, 80.0), "向上移出 Z+=80", sdk_cur)

                if action == "move_tcp_x_plus_80":
                    return _move_tcp_delta(rob_cur, payload, (DEMO_X_EXIT_MM, 0.0, 0.0), f"向后移出 X+={DEMO_X_EXIT_MM:g}", sdk_cur)

                if action == "move_tcp_x_minus_80":
                    return _move_tcp_delta(rob_cur, payload, (-DEMO_X_EXIT_MM, 0.0, 0.0), f"向前移入 X-={DEMO_X_EXIT_MM:g}", sdk_cur)

                if action == "demo_begin":
                    return _demo_begin(rob_cur, payload, sdk_cur)

                if action == "demo_end":
                    return _demo_end(rob_cur, payload, sdk_cur)

                if action in (
                    "movej_home",
                    "movej_a_pre",
                    "movej_a",
                    "movej_a_out",
                    "movej_b_pre",
                    "movej_b",
                    "movej_b_out",
                    "movej_c_pre",
                    "movej_c",
                    "movej_c_out",
                ):
                    mp, err = _parse_movej_params(payload)
                    if err:
                        return {"ok": False, "msg": err, "sdk": sdk_cur}
                    assert mp is not None
                    vel, ovl, tool, user = mp["vel"], mp["ovl"], mp["tool"], mp["user"]

                    joint_map = {
                        "movej_home": ("HOME", HOME_JOINT),
                        "movej_a_pre": ("#A pre", A_PRE_JOINT),
                        "movej_a": ("#A", A_JOINT),
                        "movej_a_out": ("#A out", A_OUT_JOINT),
                        "movej_b_pre": ("#B pre", B_PRE_JOINT),
                        "movej_b": ("#B", B_JOINT),
                        "movej_b_out": ("#B out", B_OUT_JOINT),
                        "movej_c_pre": ("#C pre", C_PRE_JOINT),
                        "movej_c": ("#C", C_JOINT),
                        "movej_c_out": ("#C out", C_OUT_JOINT),
                    }
                    label, joint = joint_map[action]

                    if action in ("movej_a_pre", "movej_b_pre", "movej_c_pre"):
                        je, jcur = _read_joint_deg(rob_cur)
                        if je != 0 or jcur is None:
                            return {"ok": False, "msg": f"读取关节失败，无法校验 →pre: err={je}", "sdk": sdk_cur}
                        at_any_out = (
                            _at_joint_target(jcur, A_OUT_JOINT)
                            or _at_joint_target(jcur, B_OUT_JOINT)
                            or _at_joint_target(jcur, C_OUT_JOINT)
                        )
                        need_gripper_clear = at_any_out
                        if action == "movej_a_pre" and _at_joint_target(jcur, A_JOINT):
                            need_gripper_clear = True
                        if action == "movej_b_pre" and _at_joint_target(jcur, B_JOINT):
                            need_gripper_clear = True
                        if action == "movej_c_pre" and _at_joint_target(jcur, C_JOINT):
                            need_gripper_clear = True
                        if need_gripper_clear and not _truthy_confirm_gripper(payload):
                            return {
                                "ok": False,
                                "needs_confirm": True,
                                "confirm_kind": "gripper_clear",
                                "msg": GRIPPER_CLEAR_CONFIRM_MSG,
                                "sdk": sdk_cur,
                            }

                    if action == "movej_a":
                        je, jcur = _read_joint_deg(rob_cur)
                        if je != 0 or jcur is None:
                            return {"ok": False, "msg": f"读取关节失败，无法校验 #A 入口: err={je}", "sdk": sdk_cur}
                        at_a_pre = _at_joint_target(jcur, A_PRE_JOINT)
                        at_a_out = _at_joint_target(jcur, A_OUT_JOINT)
                        if at_a_pre:
                            pass
                        elif at_a_out:
                            if not _truthy_confirm_no_fixture_at_target(payload):
                                return {
                                    "ok": False,
                                    "needs_confirm": True,
                                    "confirm_kind": "no_fixture_at_target",
                                    "msg": NO_FIXTURE_AT_TARGET_MSG,
                                    "sdk": sdk_cur,
                                }
                        else:
                            d = _max_joint_err_deg(jcur, A_PRE_JOINT)
                            return {
                                "ok": False,
                                "msg": (
                                    f"硬性依赖：执行 #A 前机械臂须处在 #A pre，或从 #A out 经确认后进入 #A "
                                    f"（相对 #A pre 最大轴误差约 {d:.2f}°，阈值 {JOINT_AT_PRE_TOL_DEG}°）。"
                                ),
                                "sdk": sdk_cur,
                                "joint_actual": jcur,
                            }

                    if action == "movej_b":
                        je, jcur = _read_joint_deg(rob_cur)
                        if je != 0 or jcur is None:
                            return {"ok": False, "msg": f"读取关节失败，无法校验 #B 入口: err={je}", "sdk": sdk_cur}
                        at_b_pre = _at_joint_target(jcur, B_PRE_JOINT)
                        at_b_out = _at_joint_target(jcur, B_OUT_JOINT)
                        if at_b_pre:
                            pass
                        elif at_b_out:
                            if not _truthy_confirm_no_fixture_at_target(payload):
                                return {
                                    "ok": False,
                                    "needs_confirm": True,
                                    "confirm_kind": "no_fixture_at_target",
                                    "msg": NO_FIXTURE_AT_TARGET_MSG,
                                    "sdk": sdk_cur,
                                }
                        else:
                            d = _max_joint_err_deg(jcur, B_PRE_JOINT)
                            return {
                                "ok": False,
                                "msg": (
                                    f"硬性依赖：执行 #B 前机械臂须处在 #B pre，或从 #B out 经确认后进入 #B "
                                    f"（相对 #B pre 最大轴误差约 {d:.2f}°，阈值 {JOINT_AT_PRE_TOL_DEG}°）。"
                                ),
                                "sdk": sdk_cur,
                                "joint_actual": jcur,
                            }

                    if action == "movej_c":
                        je, jcur = _read_joint_deg(rob_cur)
                        if je != 0 or jcur is None:
                            return {"ok": False, "msg": f"读取关节失败，无法校验 #C 入口: err={je}", "sdk": sdk_cur}
                        at_c_pre = _at_joint_target(jcur, C_PRE_JOINT)
                        at_c_out = _at_joint_target(jcur, C_OUT_JOINT)
                        if at_c_pre:
                            pass
                        elif at_c_out:
                            if not _truthy_confirm_no_fixture_at_target(payload):
                                return {
                                    "ok": False,
                                    "needs_confirm": True,
                                    "confirm_kind": "no_fixture_at_target",
                                    "msg": NO_FIXTURE_AT_TARGET_MSG,
                                    "sdk": sdk_cur,
                                }
                        else:
                            d = _max_joint_err_deg(jcur, C_PRE_JOINT)
                            return {
                                "ok": False,
                                "msg": (
                                    f"硬性依赖：执行 #C 前机械臂须处在 #C pre，或从 #C out 经确认后进入 #C "
                                    f"（相对 #C pre 最大轴误差约 {d:.2f}°，阈值 {JOINT_AT_PRE_TOL_DEG}°）。"
                                ),
                                "sdk": sdk_cur,
                                "joint_actual": jcur,
                            }

                    ret = _movej(rob_cur, joint, vel, ovl, tool, user)
                    ok = _movej_ok(ret)
                    if ok:
                        if action == "movej_a":
                            _deferred_pre = "A"
                        elif action == "movej_b":
                            _deferred_pre = "B"
                        elif action == "movej_c":
                            _deferred_pre = "C"
                        elif action == "movej_a_pre" and _deferred_pre == "A":
                            _deferred_pre = None
                        elif action == "movej_a_out" and _deferred_pre == "A":
                            _deferred_pre = None
                        elif action == "movej_b_pre" and _deferred_pre == "B":
                            _deferred_pre = None
                        elif action == "movej_b_out" and _deferred_pre == "B":
                            _deferred_pre = None
                        elif action == "movej_c_pre" and _deferred_pre == "C":
                            _deferred_pre = None
                        elif action == "movej_c_out" and _deferred_pre == "C":
                            _deferred_pre = None
                    return {
                        "ok": bool(ok),
                        "msg": f"MoveJ {label} => {ret}",
                        "joint": joint,
                        "ret": ret if isinstance(ret, (int, float, str, list, dict)) else str(ret),
                        "sdk": sdk_cur,
                        "deferred_pre": _deferred_pre,
                    }

                return {"ok": False, "msg": f"unknown action: {action}", "sdk": sdk_cur}

            out = _run_motion(rob, sdk)
            # 通道类失败：断开重建后重试一次（不重试 needs_confirm / 硬性依赖）
            if (
                not out.get("ok")
                and not out.get("needs_confirm")
                and (
                    _is_channel_error(out.get("ret"))
                    or (isinstance(out.get("msg"), str) and ("-4" in out["msg"] or "实时" in out["msg"]))
                    or not _sdk_channel_ok(rob)
                )
                and action in motion_actions
            ):
                rob = _rpc_ensure(force=True)
                sdk = rob.GetSDKVersion()
                out = _run_motion(rob, sdk)
                if out.get("ok"):
                    out["msg"] = (out.get("msg") or "") + "（已自动重连 SDK）"
                else:
                    out["reconnected"] = True
            return out
        except Exception as e:
            try:
                _rpc_close_unlocked()
            except Exception:
                pass
            return {"ok": False, "msg": str(e)}


_HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>FR10 网页示教器（TCP + step）</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#e0e0e0;display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:12px 10px}
h1{font-size:1.2rem;margin-bottom:4px;color:#00d4ff}
.sub{font-size:.76rem;color:#888;margin-bottom:8px}
.topbar{width:100%;max-width:420px;display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:4px}
.topbar h1{margin-bottom:0}
.box{width:100%;max-width:420px;background:#152238;border:1px solid #2d3f5f;border-radius:10px;padding:10px;margin-bottom:8px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-bottom:6px}
label{font-size:.7rem;color:#aaa}
input{width:100%;padding:7px;border:1px solid #444;border-radius:8px;background:#0f3460;color:#fff;font-size:.9rem;text-align:center}
.row2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:6px}
.btn{padding:10px 0;border:none;border-radius:10px;font-size:.85rem;font-weight:700;color:#fff;cursor:pointer;touch-action:manipulation}
.btn:active{transform:scale(.95)}
.btn-query{background:#1e8449}
.btn-movel{background:#117864}
.btn-home{background:#8e44ad}
.btn-step-a{background:#2874a6}
.btn-step-b{background:#b9770e}
.btn-step-c{background:#117a65}
.btn-qc{background:#0e6655}
.btn-demo{background:#d35400}
.btn-motor-f{background:#7d3c98}
.btn-motor-r{background:#2e86c1}
.btn-motor-s{background:#c0392b}
.btn-rh56-g{background:#1a5276}
.btn-rh56-o{background:#1e8449}
.btn-mode{background:#34495e;padding:8px 12px;white-space:nowrap}
.simple-mode .full-only{display:none!important}
.nudge{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.sp{visibility:hidden}
.btn-n{background:#5b2c6f;padding:8px 0;font-size:.78rem}
.status{width:100%;max-width:420px;margin-top:6px;padding:8px 10px;background:#0f3460;border-radius:10px;font-size:.78rem;word-break:break-all;text-align:center}
.pose-hdr{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:6px;flex-wrap:wrap}
.pose-live{font-size:.72rem;color:#aed6f1;display:flex;align-items:center;gap:6px;white-space:nowrap}
.pose-live input{width:auto;accent-color:#00d4ff;min-width:1.1rem;min-height:1.1rem}
</style>
</head>
<body class="simple-mode">
<div class="topbar">
  <h1>FR10 网页示教器</h1>
  <button class="btn btn-mode" id="mode_toggle" onclick="toggleUiMode()">完整模式</button>
</div>
<div class="sub">TCP MoveL + 关节 step 点（MoveJ），A/B/C 依赖由服务端校验</div>

<div class="box">
  <div style="font-size:.82rem;color:#aed6f1;margin-bottom:6px">基础移出/移入</div>
  <div class="row2">
    <button class="btn btn-home" onclick="moveStep('home')">#home</button>
    <button class="btn btn-qc" onclick="moveTcpRelative('z_plus')">向上移出</button>
  </div>
  <div class="row2">
    <button class="btn btn-qc" onclick="moveTcpRelative('x_plus')">向后移出 X+240</button>
    <button class="btn btn-qc" onclick="moveTcpRelative('x_minus')">向前移入 X-240</button>
  </div>
</div>

<div class="box">
  <div style="font-size:.82rem;color:#f5cba7;margin-bottom:6px">Quickchanger 演示流程</div>
  <div class="row2">
    <button class="btn btn-demo" onclick="demoSessionAction('begin')">演示开始</button>
    <button class="btn btn-demo" onclick="demoSessionAction('end')">演示结束</button>
  </div>
  <div class="row2 full-only">
    <div><label>ArUco ID</label><input id="qc_aruco_id" type="number" step="1" value="1"></div>
    <button class="btn btn-demo" onclick="runQuickchangerDemo()">执行演示流程</button>
  </div>
  <div class="row2">
    <button class="btn btn-demo" onclick="runQuickchangerPreset(1,'灵巧手')">灵巧手 ID1</button>
    <button class="btn btn-demo" onclick="runQuickchangerPreset(2,'夹爪')">夹爪 ID2</button>
  </div>
  <div class="row2">
    <button class="btn btn-demo" onclick="runQuickchangerPreset(3,'电钻')">电钻 ID3</button>
    <div></div>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-qc" onclick="clearQuickchangerExperience(1,'灵巧手')">清除灵巧手经验位置</button>
    <button class="btn btn-qc" onclick="clearQuickchangerExperience(2,'夹爪')">清除夹爪经验位置</button>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-qc" onclick="clearQuickchangerExperience(3,'电钻')">清除电钻经验位置</button>
    <div></div>
  </div>
</div>

<div class="box">
  <div style="font-size:.82rem;color:#f9e79f;margin-bottom:6px">电机控制（串口）</div>
  <div class="grid full-only">
    <div><label>电机端口</label><input id="motor_port" type="text" value="/dev/fr10-motor-serial" placeholder="/dev/fr10-motor-serial"></div>
    <div><label>波特率</label><input id="motor_baud" type="number" step="1" value="115200"></div>
    <div><label>超时(s)</label><input id="motor_timeout" type="number" step="0.1" value="0.8"></div>
  </div>
  <div class="row2">
    <button class="btn btn-motor-f" onclick="motorAction('forward')">电机正转</button>
    <button class="btn btn-motor-r" onclick="motorAction('reverse')">电机反转</button>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-motor-s" onclick="motorAction('stop')">电机停止</button>
    <button class="btn btn-motor-f" onclick="motorAction('status')">电机状态</button>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-motor-f" onclick="motorAction('lock_diagnosis')">锁紧诊断</button>
    <button class="btn btn-motor-r" onclick="motorAction('reset_diagnosis')">复位诊断</button>
  </div>
</div>

<div class="box">
  <div style="font-size:.82rem;color:#f9e79f;margin-bottom:6px">末端供电 / 设备切换</div>
  <div class="grid full-only">
    <div><label>末端端口</label><input id="ee_port" type="text" value="/dev/fr10-hand-serial" placeholder="/dev/fr10-hand-serial"></div>
    <div><label>波特率</label><input id="ee_baud" type="number" step="1" value="115200"></div>
    <div><label>超时(s)</label><input id="ee_timeout" type="number" step="0.1" value="0.8"></div>
    <div></div>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-motor-f" onclick="deviceSwitch('hand')">切换灵巧手 8.4V</button>
    <button class="btn btn-motor-r" onclick="deviceSwitch('gripper')">切换夹爪 24V</button>
  </div>
  <div class="row2">
    <button class="btn btn-motor-r full-only" onclick="deviceSwitch('drill')">切换电钻 20V</button>
    <button class="btn btn-motor-f" onclick="drillPulse()">电钻点动</button>
  </div>
  <div class="row2">
    <button class="btn btn-motor-f full-only" onclick="powerAction('status')">电源状态</button>
    <button class="btn btn-motor-s" onclick="powerAction('off')">关闭输出</button>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-motor-f" onclick="busMonitor()">末端监测 10s</button>
    <div></div>
  </div>
  <div style="font-size:.82rem;color:#d6eaf8;margin-bottom:6px">工具开合（自动判断设备）</div>
  <div class="row2">
    <button class="btn btn-rh56-o" onclick="toolAction('open')">打开</button>
    <button class="btn btn-rh56-g" onclick="toolAction('close')">抓紧/闭合</button>
  </div>
  <div class="row2 full-only">
    <button class="btn btn-rh56-o" onclick="gripperAction('open')">夹爪打开</button>
    <button class="btn btn-rh56-g" onclick="gripperAction('close')">夹爪闭合</button>
  </div>
</div>

<div class="box full-only">
  <div style="font-size:.82rem;color:#d6eaf8;margin-bottom:6px">RH56 灵巧手（串口）</div>
  <div class="grid">
    <div><label>RH56 端口</label><input id="rh56_port" type="text" value="/dev/fr10-hand-serial" placeholder="/dev/fr10-hand-serial"></div>
    <div><label>手 ID</label><input id="rh56_hand_id" type="number" step="1" value="1"></div>
    <div></div>
  </div>
  <div class="row2">
    <button class="btn btn-rh56-g" onclick="rh56Grip()">抓紧（set）</button>
    <button class="btn btn-rh56-o" onclick="rh56Open()">打开（reset）</button>
  </div>
</div>

<div class="box full-only">
  <div class="pose-hdr">
    <div style="font-size:.82rem;color:#8dd3ff">位姿（x/y/z=mm，rx/ry/rz=deg）</div>
    <label class="pose-live"><input type="checkbox" id="pose_live_sync" checked>实时同步机械臂</label>
  </div>
  <div class="grid">
    <div><label>X</label><input id="px" type="number" step="0.1" value="0"></div>
    <div><label>Y</label><input id="py" type="number" step="0.1" value="0"></div>
    <div><label>Z</label><input id="pz" type="number" step="0.1" value="0"></div>
    <div><label>Rx</label><input id="prx" type="number" step="0.1" value="0"></div>
    <div><label>Ry</label><input id="pry" type="number" step="0.1" value="0"></div>
    <div><label>Rz</label><input id="prz" type="number" step="0.1" value="0"></div>
  </div>
  <div class="grid">
    <div><label>vel(%)</label><input id="mv" type="number" step="1" value="20"></div>
    <div><label>ovl(%)</label><input id="ovl" type="number" step="1" value="100"></div>
    <div><label>tool</label><input id="tool" type="number" step="1" value="0"></div>
    <div><label>user</label><input id="user" type="number" step="1" value="0"></div>
    <div><label>位移步长 mm</label><input id="step_pos_mm" type="number" step="0.1" value="10"></div>
    <div><label>角度步长 deg</label><input id="step_rot_deg" type="number" step="0.1" value="2"></div>
  </div>
  <div class="row2">
    <button class="btn btn-query" onclick="queryPose()">读取当前 TCP 位姿</button>
    <button class="btn btn-movel" onclick="moveLPose()">执行 MoveL</button>
  </div>
  <div class="row2">
    <button class="btn btn-home" onclick="moveStep('home')">回 #home（同上）</button>
    <div></div>
  </div>
  <div class="nudge">
    <div class="sp"></div><button class="btn btn-n" onclick="nudge('x',1)">X+</button><div class="sp"></div>
    <button class="btn btn-n" onclick="nudge('y',1)">Y+</button><button class="btn btn-n" onclick="nudge('z',1)">Z+</button><button class="btn btn-n" onclick="nudge('x',-1)">X-</button>
    <div class="sp"></div><button class="btn btn-n" onclick="nudge('y',-1)">Y-</button><button class="btn btn-n" onclick="nudge('z',-1)">Z-</button><div class="sp"></div>
    <button class="btn btn-n" onclick="nudge('rx',1)">Rx+</button><div class="sp"></div>
    <button class="btn btn-n" onclick="nudge('ry',1)">Ry+</button><button class="btn btn-n" onclick="nudge('rz',1)">Rz+</button><button class="btn btn-n" onclick="nudge('rx',-1)">Rx-</button>
    <div class="sp"></div><button class="btn btn-n" onclick="nudge('ry',-1)">Ry-</button><button class="btn btn-n" onclick="nudge('rz',-1)">Rz-</button><div class="sp"></div>
  </div>
</div>

<div class="status" id="st">就绪</div>

<script>
const st=document.getElementById('st');
let currentToolHint=null;
function toggleUiMode(){
  const simple=document.body.classList.toggle('simple-mode');
  const btn=document.getElementById('mode_toggle');
  if(btn) btn.textContent=simple?'完整模式':'简约模式';
}
function setCurrentToolHint(tool){
  currentToolHint=(tool==='灵巧手'||tool==='夹爪'||tool==='电钻')?tool:null;
}
function updateToolHintFromAction(action,d){
  if(!d||!d.ok)return;
  if(action==='device_switch_hand') setCurrentToolHint('灵巧手');
  else if(action==='device_switch_gripper') setCurrentToolHint('夹爪');
  else if(action==='device_switch_drill') setCurrentToolHint('电钻');
  else if(action==='power_output_off'||action==='drill_pulse') setCurrentToolHint(null);
  else if(d.current_device) setCurrentToolHint(d.current_device);
}
function P(url,o){return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o),cache:'no-store'}).then(r=>r.json());}
function G(url){return fetch(url,{method:'GET',cache:'no-store'}).then(r=>r.json());}
function fields(){return ['px','py','pz','prx','pry','prz'];}
function readPose(){return fields().map(id=>parseFloat(document.getElementById(id).value)||0);}
function fillPose(p){if(!Array.isArray(p)||p.length<6)return;fields().forEach((id,i)=>document.getElementById(id).value=String(Number(p[i]).toFixed(3)));}

const POSE_POLL_MS=800;
let posePollTimer=null;
let posePollBusy=false;

function queryPose(opts){
  opts=opts||{};
  const silent=!!opts.silent;
  if(!silent) st.textContent='⏳ 读取中...';
  const t0=performance.now();
  return P('/api/fr10',{action:'get_pose'}).then(d=>{
    if(d.ok && Array.isArray(d.pose)) fillPose(d.pose);
    if(!silent) st.textContent=(d.ok?'✅ ':'❌ ')+'pose ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
    else if(!d.ok) st.textContent='❌ 实时同步: '+(d.msg||'');
    return d;
  }).catch(e=>{
    if(!silent) st.textContent='❌ '+e;
    else st.textContent='❌ 实时同步: '+e;
    throw e;
  });
}

function posePollTick(){
  const el=document.getElementById('pose_live_sync');
  if(!el||!el.checked||document.hidden)return;
  if(posePollBusy)return;
  posePollBusy=true;
  P('/api/fr10',{action:'get_pose'}).then(d=>{
    if(d.ok && Array.isArray(d.pose)) fillPose(d.pose);
    else if(!d.ok) st.textContent='❌ 实时同步: '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ 实时同步: '+e;}).finally(()=>{posePollBusy=false;});
}

function startPosePoll(){
  stopPosePoll();
  const el=document.getElementById('pose_live_sync');
  if(!el||!el.checked)return;
  posePollTimer=setInterval(posePollTick,POSE_POLL_MS);
}

function stopPosePoll(){
  if(posePollTimer){clearInterval(posePollTimer);posePollTimer=null;}
}

function moveLPose(){
  const pose=readPose();
  const v=parseFloat(document.getElementById('mv').value)||10;
  const ovl=parseFloat(document.getElementById('ovl').value)||100;
  const tool=parseInt(document.getElementById('tool').value||'0',10);
  const user=parseInt(document.getElementById('user').value||'0',10);
  st.textContent='⏳ MoveL...';
  const t0=performance.now();
  P('/api/fr10',{action:'movel_pose',pose,v,ovl,tool,user}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+'movel ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function motionPayload(action){
  const v=parseFloat(document.getElementById('mv').value)||10;
  const ovl=parseFloat(document.getElementById('ovl').value)||100;
  const tool=parseInt(document.getElementById('tool').value||'0',10);
  const user=parseInt(document.getElementById('user').value||'0',10);
  return {action,v,ovl,tool,user};
}

function moveTcpRelative(kind){
  const actionMap={
    z_plus:'move_tcp_z_plus_80',
    x_plus:'move_tcp_x_plus_80',
    x_minus:'move_tcp_x_minus_80'
  };
  const labelMap={
    z_plus:'向上移出 Z+=80',
    x_plus:'向后移出 X+=240',
    x_minus:'向前移入 X-=240'
  };
  const action=actionMap[kind]; if(!action)return;
  st.textContent='⏳ '+labelMap[kind]+'...';
  const t0=performance.now();
  P('/api/fr10',motionPayload(action)).then(d=>{
    if(d.ok && Array.isArray(d.pose_target)) fillPose(d.pose_target);
    st.textContent=(d.ok?'✅ ':'❌ ')+labelMap[kind]+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function demoSessionAction(kind){
  const action=kind==='begin'?'demo_begin':'demo_end';
  const label=kind==='begin'?'演示开始':'演示结束';
  if(kind==='begin'){
    const q='执行演示开始？\\n\\n将记录当前位置为回归位置，然后移动到固定演示起始位。';
    if(!window.confirm(q)){
      st.textContent='❌ 已取消（未执行演示开始）';
      return;
    }
  }
  st.textContent='⏳ '+label+'...';
  const t0=performance.now();
  P('/api/fr10',motionPayload(action)).then(d=>{
    if(d.ok && Array.isArray(d.pose_target)) fillPose(d.pose_target);
    st.textContent=(d.ok?'✅ ':'❌ ')+label+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
    queryPose({silent:true}).catch(()=>{});
  }).catch(e=>{st.textContent='❌ '+label+' 请求失败: '+e;});
}

function runQuickchangerDemo(){
  const aruco_id=parseInt(document.getElementById('qc_aruco_id').value||'1',10)||1;
  if(!window.confirm('执行 Quickchanger 演示流程？\\nArUco ID = '+aruco_id)){
    st.textContent='❌ 已取消（未执行演示流程）';
    return;
  }
  st.textContent='⏳ Quickchanger 演示流程执行中...';
  const t0=performance.now();
  P('/api/fr10',{action:'quickchanger_demo',aruco_id}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+'quickchanger demo ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
    queryPose({silent:true}).catch(()=>{});
  }).catch(e=>{st.textContent='❌ '+e;});
}

function runQuickchangerPreset(aruco_id,label){
  let msg='执行 '+label+' Quickchanger 流程？\\nArUco ID = '+aruco_id+'\\n\\n流程成功后将自动检查并切换到目标供电。';
  if(aruco_id===3) msg+='\\n\\n注意：电钻切换供电后会立即转动。';
  if(!window.confirm(msg)){
    st.textContent='❌ 已取消（未执行 '+label+' 流程）';
    return;
  }
  st.textContent='⏳ '+label+' Quickchanger 后台任务启动中...';
  const t0=performance.now();
  const job_id='qc-client-'+Date.now()+'-'+Math.floor(Math.random()*100000);
  P('/api/fr10',eePayload('quickchanger_demo_start',{aruco_id,auto_power:true,job_id})).then(d=>{
    if(!d.ok||!d.job_id){
      st.textContent='❌ '+label+' quickchanger 启动失败: '+(d.msg||'');
      return;
    }
    pollQuickchangerPreset(d.job_id,label,t0,0,0,aruco_id);
  }).catch(e=>{
    st.textContent='⏳ '+label+' 启动响应中断，继续用本地任务号查询... '+e;
    pollQuickchangerPreset(job_id,label,t0,1,0,aruco_id);
  });
}

function pollQuickchangerPreset(job_id,label,t0,failCount,missingCount,aruco_id){
  setTimeout(()=>{
    G('/api/fr10_job?job_id='+encodeURIComponent(job_id)+'&aruco_id='+encodeURIComponent(aruco_id||'')).then(d=>{
      if(!d.ok){
        const nextMissing=(missingCount||0)+1;
        if(nextMissing<120){
          st.textContent='⏳ '+label+' 等待后台任务/返回结果... '+nextMissing+'/120 '+(d.msg||'');
          pollQuickchangerPreset(job_id,label,t0,0,nextMissing,aruco_id);
          return;
        }
        st.textContent='⚠ '+label+' 暂无法确认返回结果，请查看机械臂和供电状态。'+(d.msg||'');
        window.alert(label+'暂无法确认返回结果。实际动作可能已经完成，请查看供电状态。');
        return;
      }
      if(!d.done){
        st.textContent='⏳ '+label+' Quickchanger 执行中... '+(d.msg||d.status||'')+' ('+Math.round((performance.now()-t0)/1000)+'s)';
        pollQuickchangerPreset(job_id,label,t0,0,0,aruco_id);
        return;
      }
      const r=d.result||{};
      const powerMsg=r.power_result&&r.power_result.msg?r.power_result.msg:'';
      const shownMsg=powerMsg||r.msg||d.msg||'';
      if(r.ok){
        if(aruco_id===1) setCurrentToolHint('灵巧手');
        else if(aruco_id===2) setCurrentToolHint('夹爪');
        else if(aruco_id===3) setCurrentToolHint(null);
      }
      st.textContent=(r.ok?'✅ ':'❌ ')+label+' quickchanger ('+Math.round(performance.now()-t0)+'ms) '+shownMsg;
      if(r.ok){
        window.alert(label+'切换完成\\n\\n'+(powerMsg||'Quickchanger 已完成，目标供电已处理。'));
      }else if(powerMsg){
        window.alert(label+'供电切换异常\\n\\n'+powerMsg);
      }
      queryPose({silent:true}).catch(()=>{});
    }).catch(e=>{
      const nextFail=failCount+1;
      st.textContent='⏳ '+label+' Quickchanger 可能仍在执行；状态查询中断 '+nextFail+' 次: '+e;
      if(nextFail<60){
        pollQuickchangerPreset(job_id,label,t0,nextFail,missingCount||0,aruco_id);
      }else{
        window.alert(label+'状态查询持续中断。动作可能仍在后台执行或已经完成，请查看供电状态确认。');
      }
    });
  },1000);
}

function clearQuickchangerExperience(aruco_id,label){
  const q='清除 '+label+' ID'+aruco_id+' 的 quickchanger 经验位置？\\n\\n'
    +'只会删除该编号记录，不影响其它编号。';
  if(!window.confirm(q)){
    st.textContent='❌ 已取消（未清除 '+label+' 经验位置）';
    return;
  }
  st.textContent='⏳ 清除 '+label+' 经验位置...';
  const t0=performance.now();
  P('/api/fr10',{action:'quickchanger_clear_experience',aruco_id}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+label+' 清除经验位置 ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+label+' 清除经验位置请求失败: '+e;});
}

function motorAction(kind){
  const motor_port=(document.getElementById('motor_port').value||'').trim()||'/dev/fr10-motor-serial';
  const motor_baud=parseInt(document.getElementById('motor_baud').value||'115200',10) || 115200;
  const motor_timeout=parseFloat(document.getElementById('motor_timeout').value||'0.8');
  const actionMap={
    forward:'motor_forward',
    reverse:'motor_reverse',
    stop:'motor_stop',
    status:'motor_status',
    lock_diagnosis:'motor_lock_diagnosis',
    reset_diagnosis:'motor_reset_diagnosis'
  };
  const labelMap={
    forward:'正转',
    reverse:'反转',
    stop:'停止',
    status:'状态',
    lock_diagnosis:'锁紧诊断',
    reset_diagnosis:'复位诊断'
  };
  const action=actionMap[kind]; if(!action) return;
  st.textContent='⏳ 电机'+labelMap[kind]+'...';
  const t0=performance.now();
  P('/api/fr10',{action,motor_port,motor_baud,motor_timeout}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+'motor '+labelMap[kind]+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function eePayload(action,extra){
  const ee_port=(document.getElementById('ee_port').value||'').trim()||'/dev/fr10-hand-serial';
  const ee_baud=parseInt(document.getElementById('ee_baud').value||'115200',10)||115200;
  const ee_timeout=parseFloat(document.getElementById('ee_timeout').value||'0.8');
  return Object.assign({action,ee_port,ee_baud,ee_timeout},extra||{});
}

function sendEeAction(action,label,extra,opt){
  opt=opt||{};
  st.textContent='⏳ '+label+'...';
  const t0=performance.now();
  P('/api/fr10',eePayload(action,extra)).then(d=>{
    if(!d.ok && d.needs_confirm && d.confirm_kind==='power_on' && !opt.confirm_power_on){
      const q=(d.msg||'确认打开电源输出？')+'\\n\\n点击「确定」后将继续执行。';
      if(window.confirm(q)){
        sendEeAction(action,label,Object.assign({},extra||{},{confirm_power_on:true}),{confirm_power_on:true});
        return;
      }
      st.textContent='❌ 已取消（未打开电源输出）';
      return;
    }
    updateToolHintFromAction(action,d);
    st.textContent=(d.ok?'✅ ':'❌ ')+label+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function deviceSwitch(kind){
  if(kind==='hand') sendEeAction('device_switch_hand','切换灵巧手 8.4V',{});
  else if(kind==='gripper') sendEeAction('device_switch_gripper','切换夹爪 24V',{});
  else if(kind==='drill') sendEeAction('device_switch_drill','切换电钻 20V',{});
}

function powerAction(kind){
  if(kind==='status') sendEeAction('power_status','电源状态',{});
  else if(kind==='off') sendEeAction('power_output_off','关闭电源输出',{});
}

function drillPulse(){
  const label='电钻点动';
  const q='电钻点动将先关闭电源输出，然后执行「切换电钻 20V」上电动作。\\n\\n'
    +'请确认电钻已固定、周围无人。';
  if(!window.confirm(q)){
    st.textContent='❌ 已取消（未打开电钻输出）';
    return;
  }
  st.textContent='⏳ '+label+' 关闭输出...';
  const t0=performance.now();
  P('/api/fr10',eePayload('power_output_off',{})).then(off=>{
    if(!off.ok){
      st.textContent='❌ '+label+' 关闭输出失败 ('+Math.round(performance.now()-t0)+'ms) '+(off.msg||'');
      return;
    }
    st.textContent='⏳ '+label+' 已关闭输出，等待 0.5s 后切换电钻 20V...';
    return new Promise(resolve=>{
      window.setTimeout(()=>{
        resolve(P('/api/fr10',eePayload('device_switch_drill',{confirm_power_on:true})));
      },500);
    });
  }).then(d=>{
    if(!d) return;
    if(!d.ok){
      st.textContent='❌ '+label+' 切换电钻 20V 失败 ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
      return;
    }
    setCurrentToolHint('电钻');
    st.textContent='✅ '+label+' 已完成：关闭输出 -> 切换电钻 20V ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+label+' 请求失败: '+e;});
}

function busMonitor(){
  sendEeAction('bus_monitor','末端电源监测',{duration_s:10});
}

function gripperAction(kind){
  if(kind==='open') sendEeAction('gripper_open','夹爪打开',{});
  else if(kind==='close') sendEeAction('gripper_close','夹爪闭合',{});
}

function toolPayload(action,extra){
  const rh56_port=(document.getElementById('rh56_port').value||'').trim()||'/dev/fr10-hand-serial';
  const rh56_hand_id=parseInt(document.getElementById('rh56_hand_id').value||'1',10)||1;
  const hint=currentToolHint?{tool_hint:currentToolHint}:{};
  return eePayload(action,Object.assign({rh56_port,rh56_hand_id},hint,extra||{}));
}

function toolAction(kind){
  const action=kind==='open'?'tool_open_auto':'tool_close_auto';
  const label=kind==='open'?'打开':'抓紧/闭合';
  st.textContent='⏳ '+label+'（自动判断设备）...';
  const t0=performance.now();
  P('/api/fr10',toolPayload(action,{})).then(d=>{
    if(d.current_tool) setCurrentToolHint(d.current_tool);
    const tool=d.current_tool?('['+d.current_tool+'] '):'';
    st.textContent=(d.ok?'✅ ':'❌ ')+tool+label+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+label+' 请求失败: '+e;});
}

function rh56Grip(){
  const rh56_port=(document.getElementById('rh56_port').value||'').trim()||'/dev/fr10-hand-serial';
  const rh56_hand_id=parseInt(document.getElementById('rh56_hand_id').value||'1',10)||1;
  st.textContent='⏳ RH56 抓紧...';
  const t0=performance.now();
  P('/api/fr10',{action:'rh56_grip',rh56_port,rh56_hand_id}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+'RH56抓紧 ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function rh56Open(){
  const rh56_port=(document.getElementById('rh56_port').value||'').trim()||'/dev/fr10-hand-serial';
  const rh56_hand_id=parseInt(document.getElementById('rh56_hand_id').value||'1',10)||1;
  st.textContent='⏳ RH56 打开...';
  const t0=performance.now();
  P('/api/fr10',{action:'rh56_open',rh56_port,rh56_hand_id}).then(d=>{
    st.textContent=(d.ok?'✅ ':'❌ ')+'RH56打开 ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

const STEP_ACTIONS={home:'movej_home',a_pre:'movej_a_pre',a:'movej_a',a_out:'movej_a_out',b_pre:'movej_b_pre',b:'movej_b',b_out:'movej_b_out',c_pre:'movej_c_pre',c:'movej_c',c_out:'movej_c_out'};
const STEP_LABELS={home:'#home',a_pre:'#A pre',a:'#A',a_out:'#A out',b_pre:'#B pre',b:'#B',b_out:'#B out',c_pre:'#C pre',c:'#C',c_out:'#C out'};
function moveStep(which,opt){
  opt=opt||{};
  const act=STEP_ACTIONS[which]; if(!act) return;
  const v=parseFloat(document.getElementById('mv').value)||10;
  const ovl=parseFloat(document.getElementById('ovl').value)||100;
  const tool=parseInt(document.getElementById('tool').value||'0',10);
  const user=parseInt(document.getElementById('user').value||'0',10);
  const body={action:act,v,ovl,tool,user};
  if(opt.confirm_gripper_clear) body.confirm_gripper_clear=true;
  if(opt.confirm_no_fixture_at_target) body.confirm_no_fixture_at_target=true;
  st.textContent='⏳ MoveJ '+STEP_LABELS[which]+'...';
  const t0=performance.now();
  P('/api/fr10',body).then(d=>{
    if(!d.ok && d.needs_confirm && d.confirm_kind==='gripper_clear' && !opt.confirm_gripper_clear){
      const q='请确定快换工具当前未装配！\n\n点击「确定」后将执行「'+STEP_LABELS[which]+'」。';
      if(window.confirm(q)){
        moveStep(which,Object.assign({},opt,{confirm_gripper_clear:true}));
        return;
      }
      st.textContent='❌ 已取消（未执行移动）';
      return;
    }
    if(!d.ok && d.needs_confirm && d.confirm_kind==='no_fixture_at_target' && !opt.confirm_no_fixture_at_target){
      const q='请确定目标位置没有快换装置！\n\n点击「确定」后将执行「'+STEP_LABELS[which]+'」。';
      if(window.confirm(q)){
        moveStep(which,Object.assign({},opt,{confirm_no_fixture_at_target:true}));
        return;
      }
      st.textContent='❌ 已取消（未执行移动）';
      return;
    }
    st.textContent=(d.ok?'✅ ':'❌ ')+STEP_LABELS[which]+' ('+Math.round(performance.now()-t0)+'ms) '+(d.msg||'');
  }).catch(e=>{st.textContent='❌ '+e;});
}

function nudge(axis,dir){
  const posStep=Math.max(0.1,parseFloat(document.getElementById('step_pos_mm').value)||10);
  const rotStep=Math.max(0.1,parseFloat(document.getElementById('step_rot_deg').value)||2);
  const m={x:'px',y:'py',z:'pz',rx:'prx',ry:'pry',rz:'prz'};
  const id=m[axis]; if(!id) return;
  const step=(axis==='x'||axis==='y'||axis==='z')?posStep:rotStep;
  const el=document.getElementById(id);
  const cur=parseFloat(el.value)||0;
  el.value=String((cur+dir*step).toFixed(3));
}

function onPoseLiveToggle(){
  const el=document.getElementById('pose_live_sync');
  if(el&&el.checked)startPosePoll();
  else stopPosePoll();
}

document.addEventListener('visibilitychange',()=>{
  if(document.hidden)stopPosePoll();
  else{const el=document.getElementById('pose_live_sync');if(el&&el.checked)startPosePoll();}
});

window.addEventListener('load',()=>{
  const liveEl=document.getElementById('pose_live_sync');
  if(liveEl) liveEl.addEventListener('change',onPoseLiveToggle);
  setTimeout(()=>{
    queryPose().finally(()=>{
      if(liveEl&&liveEl.checked)startPosePoll();
    });
  },120);
});
</script>
</body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")

    def _json_resp(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self._cors()
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _read_body(self):
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length)) if length else {}

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/fr10_job":
            qs = urllib.parse.parse_qs(parsed.query)
            job_id = (qs.get("job_id") or [""])[0]
            self._json_resp(200, _get_quickchanger_job({"job_id": job_id}))
            return
        if parsed.path != "/":
            self.send_response(404)
            self.end_headers()
            return
        body = _HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/api/fr10":
            self.send_response(404)
            self.end_headers()
            return
        body = self._read_body()
        action = str(body.get("action", "")).strip()
        out = _fr10_request(action, body)
        self._json_resp(200, out)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()


def main():
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8890)
    args = ap.parse_args()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[WEB] FR10 host: {FR10_HOST}")
    print(f"[WEB] Listening on http://0.0.0.0:{args.port}")
    print(f"[WEB] Open from phone: http://<你的电脑IP>:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        print("\n[WEB] stopped")


if __name__ == "__main__":
    main()
