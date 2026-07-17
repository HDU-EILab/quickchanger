#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双 RS485 总线（现场 Jetson 布局，与 motor_control_with_power_debug.py 单 COM 实验室接线不同）：

- 快换电机：/dev/fr10-motor-serial（FTDI），仅 Modbus 电机
- 末端：/dev/fr10-hand-serial（CH340），CL-200W 电源 + HEPG 夹爪 + RH56 灵巧手
"""
from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from typing import Any, Iterator, Optional

try:
    import serial  # type: ignore[reportMissingImports]
except Exception:
    serial = None

MOTOR_BUS_DEFAULT_PORT = "/dev/fr10-motor-serial"
EE_BUS_DEFAULT_PORT = "/dev/fr10-hand-serial"
# 兼容旧参数名 bus_port
BUS_DEFAULT_PORT = MOTOR_BUS_DEFAULT_PORT
BUS_DEFAULT_BAUD = 115200
BUS_DEFAULT_TIMEOUT = 0.8

POWER_SLAVE_ADDR = 1
MOTOR_SLAVE_ADDR = 1
HAND_DEVICE_ID = 1
GRIPPER_SLAVE_ADDR = 9

MODBUS_SILENT_INTERVAL = 0.01
READ_RESPONSE_WAIT = 0.03
POWER_RESPONSE_WAIT = 0.12
POWER_RETRY_COUNT = 2
POWER_COMMAND_GAP = 0.25
ACTION_MONITOR_SECONDS = 3.5
ACTION_MONITOR_INTERVAL = 0.2

POWER_MAX_VOLTAGE_MV = 30000
POWER_MAX_CURRENT_MA = 7000

REG_SET_VOLTAGE = 0x0500
REG_SET_CURRENT = 0x0501
REG_OUTPUT_ONOFF = 0x0502
REG_REAL_VOLTAGE = 0x0400
REG_REAL_CURRENT = 0x0401
REG_CC_CV = 0x0402

DEVICE_PROFILES = {
    "1": {"name": "灵巧手", "rack_id": 1, "voltage_v": 8.4, "current_a": 2.0},
    "2": {"name": "夹爪", "rack_id": 2, "voltage_v": 24.0, "current_a": 2.0},
}

MOTOR_COMMANDS = {
    "status_run": "01 03 00 00 00 01",
    "status_lock": "01 03 00 73 00 01",
    "status_orig": "01 03 00 74 00 01",
    "stop": "01 10 00 04 00 01 02 00 00 A7 D4",
    "forward": "01 10 00 A0 00 07 0E 03 E8 03 E8 40 00 01 F4 03 E8 01 2C 03 E8 EA B8",
    "reverse": "01 10 00 A7 00 07 0E 03 E8 03 E8 40 00 01 F4 03 E8 01 2C 03 E8 5B CD",
}

_motor_bus_lock = threading.Lock()
_ee_bus_lock = threading.Lock()
_device_manager_state: dict[str, Any] = {"current_device": None, "current_profile_key": None}


def calculate_modbus_crc16(data_or_hex) -> bytes:
    if isinstance(data_or_hex, str):
        data = bytes.fromhex(data_or_hex)
    else:
        data = bytes(data_or_hex)
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def bytes_to_hex(data: bytes) -> str:
    return data.hex(" ").upper()


def build_motor_frame(cmd_key: str, with_existing_crc: bool, slave_addr: int = MOTOR_SLAVE_ADDR) -> bytes:
    raw = bytearray(bytes.fromhex(MOTOR_COMMANDS[cmd_key]))
    if with_existing_crc:
        raw = raw[:-2]
    raw[0] = slave_addr & 0xFF
    return bytes(raw) + calculate_modbus_crc16(raw)


def _parse_modbus_write_ack(resp: bytes, slave_addr: int) -> tuple[bool, str]:
    if not resp:
        return False, ""
    hx = bytes_to_hex(resp)
    if len(resp) >= 5 and resp[0] == slave_addr and (resp[1] & 0x80):
        return False, f"Modbus异常 func=0x{resp[1]:02X} code=0x{resp[2]:02X} ({hx})"
    if len(resp) >= 8 and resp[0] == slave_addr and resp[1] == 0x10:
        return True, hx
    return False, hx


def _ok(msg: str, **data) -> dict:
    out: dict = {"ok": True, "msg": msg}
    if data:
        out.update(data)
    return out


def _fail(msg: str, **data) -> dict:
    out: dict = {"ok": False, "msg": msg}
    if data:
        out.update(data)
    return out


def _serial_params(payload: dict, port_keys: tuple[str, ...], default_port: str) -> tuple[str, int, float]:
    port = ""
    for key in port_keys:
        v = payload.get(key)
        if v is not None and str(v).strip():
            port = str(v).strip()
            break
    if not port:
        port = default_port
    baud = int(payload.get("bus_baud") or payload.get("motor_baud") or payload.get("ee_baud") or BUS_DEFAULT_BAUD)
    timeout = float(payload.get("bus_timeout") or payload.get("motor_timeout") or payload.get("ee_timeout") or BUS_DEFAULT_TIMEOUT)
    return port, baud, timeout


def parse_motor_params(payload: dict) -> tuple[str, int, float, int]:
    port, baud, timeout = _serial_params(
        payload, ("motor_port", "bus_port"), MOTOR_BUS_DEFAULT_PORT,
    )
    motor_slave = int(payload.get("motor_slave_addr", MOTOR_SLAVE_ADDR))
    return port, baud, timeout, motor_slave


def parse_ee_params(payload: dict) -> tuple[str, int, float, int, int]:
    port, baud, timeout = _serial_params(
        payload, ("ee_port", "hand_port", "bus_port"), EE_BUS_DEFAULT_PORT,
    )
    gripper_slave = int(payload.get("gripper_slave_addr", GRIPPER_SLAVE_ADDR))
    power_slave = int(payload.get("power_slave_addr", POWER_SLAVE_ADDR))
    return port, baud, timeout, gripper_slave, power_slave


def parse_bus_params(payload: dict) -> tuple[str, int, float, int, int, int]:
    """兼容旧单总线调用：motor 与 ee 各用各自默认口。"""
    m_port, baud, timeout, motor_slave = parse_motor_params(payload)
    e_port, _, _, gripper_slave, power_slave = parse_ee_params(payload)
    return m_port, baud, timeout, motor_slave, gripper_slave, power_slave


class CL200WPowerSupply:
    def __init__(self, ser: "serial.Serial", slave_addr: int = POWER_SLAVE_ADDR):
        self.ser = ser
        self.slave_addr = slave_addr
        self.last_command_time = 0.0
        self.cached_set_voltage_mv: Optional[int] = None
        self.cached_set_current_ma: Optional[int] = None
        self.cached_output_onoff: Optional[int] = None

    def _read_response(self, expect_len: Optional[int]) -> bytes:
        if expect_len is None:
            time.sleep(POWER_RESPONSE_WAIT)
            return self.ser.read(self.ser.in_waiting or 256)
        deadline = time.time() + max(BUS_DEFAULT_TIMEOUT, POWER_RESPONSE_WAIT)
        chunks = bytearray()
        while len(chunks) < expect_len and time.time() < deadline:
            part = self.ser.read(expect_len - len(chunks))
            if part:
                chunks.extend(part)
            else:
                time.sleep(0.01)
        return bytes(chunks)

    def _send_and_read_once(self, frame_without_crc: bytes, expect_len: Optional[int] = None) -> bytes:
        frame = frame_without_crc + calculate_modbus_crc16(frame_without_crc)
        elapsed = time.time() - self.last_command_time
        if elapsed < POWER_COMMAND_GAP:
            time.sleep(POWER_COMMAND_GAP - elapsed)
        time.sleep(MODBUS_SILENT_INTERVAL)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(POWER_RESPONSE_WAIT)
        self.last_command_time = time.time()
        resp = self._read_response(expect_len)
        if not resp:
            raise TimeoutError("电源没有响应")
        if len(resp) < 5:
            raise ValueError(f"电源响应太短：{bytes_to_hex(resp)}")
        body, recv_crc = resp[:-2], resp[-2:]
        if recv_crc != calculate_modbus_crc16(body):
            raise ValueError(f"电源响应 CRC 错误：{bytes_to_hex(resp)}")
        return resp

    def _send_and_read(self, frame_without_crc: bytes, expect_len: Optional[int] = None) -> bytes:
        last_error: Optional[Exception] = None
        for attempt in range(1, POWER_RETRY_COUNT + 1):
            try:
                if attempt > 1:
                    time.sleep(0.15)
                return self._send_and_read_once(frame_without_crc, expect_len)
            except TimeoutError as e:
                last_error = e
        raise last_error or TimeoutError("电源无响应")

    def write_register(self, reg_addr: int, value: int) -> bytes:
        frame = bytes([
            self.slave_addr, 0x06,
            (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF,
        ])
        return self._send_and_read(frame, expect_len=8)

    def read_register(self, reg_addr: int) -> int:
        frame = bytes([
            self.slave_addr, 0x03,
            (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
            0x00, 0x01,
        ])
        resp = self._send_and_read(frame, expect_len=7)
        if resp[0] != self.slave_addr or resp[1] != 0x03 or resp[2] != 0x02:
            raise ValueError(f"电源读寄存器异常：{bytes_to_hex(resp)}")
        return (resp[3] << 8) | resp[4]

    def set_voltage_v(self, voltage_v: float) -> None:
        mv = int(round(voltage_v * 1000))
        if not (0 <= mv <= POWER_MAX_VOLTAGE_MV):
            raise ValueError(f"电压超限 0-{POWER_MAX_VOLTAGE_MV / 1000} V")
        self.write_register(REG_SET_VOLTAGE, mv)
        self.cached_set_voltage_mv = mv

    def set_current_a(self, current_a: float) -> None:
        ma = int(round(current_a * 1000))
        if not (0 <= ma <= POWER_MAX_CURRENT_MA):
            raise ValueError(f"电流超限 0-{POWER_MAX_CURRENT_MA / 1000} A")
        self.write_register(REG_SET_CURRENT, ma)
        self.cached_set_current_ma = ma

    def output_on(self) -> None:
        self.write_register(REG_OUTPUT_ONOFF, 1)
        self.cached_output_onoff = 1

    def output_off(self) -> None:
        self.write_register(REG_OUTPUT_ONOFF, 0)
        self.cached_output_onoff = 0

    def read_basic_status(self) -> dict:
        voltage_mv = self.read_register(REG_REAL_VOLTAGE)
        current_ma = self.read_register(REG_REAL_CURRENT)
        cc_cv = self.read_register(REG_CC_CV)
        return {
            "realtime_voltage_v": voltage_mv / 1000.0,
            "realtime_current_a": current_ma / 1000.0,
            "cc_cv_raw": cc_cv,
            "cc_cv": "CC" if cc_cv == 1 else "CV",
        }

    def read_status(self) -> dict:
        basic = self.read_basic_status()
        output_setting = self.read_output_setting()
        return {
            **basic,
            "cc_cv_label": "恒流 CC" if basic["cc_cv_raw"] == 1 else "恒压 CV",
            "output_setting": output_setting,
            "cached_set_voltage_v": None if self.cached_set_voltage_mv is None else self.cached_set_voltage_mv / 1000.0,
            "cached_set_current_a": None if self.cached_set_current_ma is None else self.cached_set_current_ma / 1000.0,
            "cached_output_onoff": self.cached_output_onoff,
        }

    def read_output_setting(self) -> Optional[int]:
        try:
            return self.read_register(REG_OUTPUT_ONOFF)
        except Exception:
            return None


class MotorController:
    def __init__(self, ser: "serial.Serial", slave_addr: int = MOTOR_SLAVE_ADDR):
        self.ser = ser
        self.slave_addr = slave_addr
        self.current_mode = "初始待机模式"

    def _read_reg(self, cmd_key: str) -> Optional[str]:
        full_cmd = build_motor_frame(cmd_key, with_existing_crc=False, slave_addr=self.slave_addr)
        time.sleep(MODBUS_SILENT_INTERVAL)
        self.ser.reset_input_buffer()
        self.ser.write(full_cmd)
        self.ser.flush()
        time.sleep(max(READ_RESPONSE_WAIT, 0.08))
        resp = self.ser.read(7)
        if not resp or len(resp) != 7:
            return None
        body, recv_crc = resp[:-2], resp[-2:]
        if recv_crc != calculate_modbus_crc16(body):
            return None
        if resp[0] != self.slave_addr or resp[1] != 0x03 or resp[2] != 0x02:
            return None
        return f"{resp[3]:02X}{resp[4]:02X}"

    def read_status_values(self) -> dict:
        return {
            "run": self._read_reg("status_run"),
            "lock": self._read_reg("status_lock"),
            "orig": self._read_reg("status_orig"),
            "motor_slave_addr": self.slave_addr,
        }

    def parse_status_report(self, values: dict) -> dict:
        run_val, lock_val, orig_val = values.get("run"), values.get("lock"), values.get("orig")
        motion = "未知"
        if run_val == "0001":
            motion = "正在运行"
        elif run_val == "0000":
            motion = "停止/待机"
        elif run_val is None:
            motion = "未读到（检查电机地址/接线/供电）"
        position = "未知"
        if lock_val == "0001":
            position = "锁紧位"
        elif orig_val == "0001":
            position = "原点位"
        elif run_val == "0001":
            position = "位移中"
        elif run_val == "0000":
            position = "停止且不在锁紧/原点"
        elif lock_val is None and orig_val is None:
            position = "未读到"
        return {"motion": motion, "position": position, **values}

    def execute_action(self, action: str) -> tuple[bool, str]:
        mode_map = {"forward": "电机正转模式", "reverse": "电机反转模式", "stop": "停止模式"}
        if action not in mode_map:
            raise ValueError(f"unknown motor action: {action}")
        cmd = build_motor_frame(action, with_existing_crc=True, slave_addr=self.slave_addr)
        time.sleep(MODBUS_SILENT_INTERVAL)
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        self.ser.flush()
        time.sleep(max(READ_RESPONSE_WAIT, 0.12))
        resp = self.ser.read(16)
        ok, hx = _parse_modbus_write_ack(resp, self.slave_addr)
        if ok:
            self.current_mode = mode_map[action]
        return ok, hx


class GripperController:
    def __init__(self, ser: "serial.Serial", slave_addr: int = GRIPPER_SLAVE_ADDR):
        self.ser = ser
        self.slave_addr = slave_addr
        self.last_command_time = 0.0

    def _send_and_read(self, frame_without_crc: bytes, expect_len: int) -> bytes:
        frame = frame_without_crc + calculate_modbus_crc16(frame_without_crc)
        elapsed = time.time() - self.last_command_time
        if elapsed < POWER_COMMAND_GAP:
            time.sleep(POWER_COMMAND_GAP - elapsed)
        time.sleep(MODBUS_SILENT_INTERVAL)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(frame)
        self.ser.flush()
        self.last_command_time = time.time()
        deadline = time.time() + max(BUS_DEFAULT_TIMEOUT, 0.15)
        buf = bytearray()
        while len(buf) < expect_len and time.time() < deadline:
            part = self.ser.read(expect_len - len(buf))
            if part:
                buf.extend(part)
            else:
                time.sleep(0.005)
        resp = bytes(buf)
        if not resp:
            raise TimeoutError(
                f"夹爪 Modbus 地址 {self.slave_addr} 无响应；请确认已切换 24V、夹爪已接在 RS485 总线上，或调整「夹爪地址」"
            )
        if len(resp) < expect_len:
            raise TimeoutError(
                f"夹爪响应过短({len(resp)}B)：{bytes_to_hex(resp)}；地址={self.slave_addr}"
            )
        chunk = resp[:expect_len]
        body, recv_crc = chunk[:-2], chunk[-2:]
        if recv_crc != calculate_modbus_crc16(body):
            raise ValueError(f"夹爪 CRC 错误：{bytes_to_hex(chunk)}")
        return chunk

    def write_registers(self, start_addr: int, values) -> bytes:
        data = bytearray()
        for value in values:
            data.append((value >> 8) & 0xFF)
            data.append(value & 0xFF)
        frame = bytes([
            self.slave_addr, 0x10,
            (start_addr >> 8) & 0xFF, start_addr & 0xFF,
            0x00, len(values), len(data),
        ]) + bytes(data)
        resp = self._send_and_read(frame, expect_len=8)
        if resp[0] != self.slave_addr or resp[1] != 0x10:
            raise ValueError(f"夹爪写寄存器异常：{bytes_to_hex(resp)}")
        return resp

    def read_status(self) -> Optional[dict]:
        frame = bytes([self.slave_addr, 0x04, 0x07, 0xD0, 0x00, 0x03])
        try:
            resp = self._send_and_read(frame, expect_len=11)
        except Exception:
            return None
        if resp[0] != self.slave_addr or resp[1] != 0x04 or resp[2] != 0x06:
            return None
        reg_07d0 = (resp[3] << 8) | resp[4]
        reg_07d1 = (resp[5] << 8) | resp[6]
        reg_07d2 = (resp[7] << 8) | resp[8]
        return {
            "status": reg_07d0 & 0xFF,
            "fault": reg_07d1 & 0xFF,
            "position": (reg_07d1 >> 8) & 0xFF,
            "speed": reg_07d2 & 0xFF,
            "force": (reg_07d2 >> 8) & 0xFF,
        }

    def move_to(self, position: int, speed: int = 0xFF, force: int = 0xFF) -> bool:
        position = max(0, min(0xFF, int(position)))
        control = 0x0009
        position_reg = position << 8
        force_speed_reg = (force << 8) | speed
        self.write_registers(0x03E8, [control, position_reg, force_speed_reg])
        return True

    def close_once(self) -> bool:
        self.move_to(0xFF)
        time.sleep(1.2)
        return True

    def open_once(self) -> bool:
        self.move_to(0x00)
        time.sleep(1.2)
        return True

    def demo_close_then_open(self) -> bool:
        self.close_once()
        time.sleep(0.5)
        self.open_once()
        return True


class DexterousHandController:
    def __init__(self, ser: "serial.Serial", hand_id: int = HAND_DEVICE_ID):
        self.ser = ser
        self.hand_id = hand_id

    @staticmethod
    def _pack_i16_le(value: int) -> bytes:
        value &= 0xFFFF
        return bytes([value & 0xFF, (value >> 8) & 0xFF])

    def _pack_frame(self, rw: int, addr: int, data: bytes = b"") -> bytes:
        addr_l, addr_h = addr & 0xFF, (addr >> 8) & 0xFF
        if rw == 0x11:
            payload = bytes([self.hand_id, 0x04, rw, addr_l, addr_h]) + data
        else:
            payload = bytes([self.hand_id, len(data) + 3, rw, addr_l, addr_h]) + data
        checksum = sum(payload) & 0xFF
        return b"\xEB\x90" + payload + bytes([checksum])

    def _write_registers(self, addr: int, values) -> bool:
        data = b"".join(self._pack_i16_le(int(v)) for v in values)
        frame = self._pack_frame(0x12, addr, data)
        time.sleep(MODBUS_SILENT_INTERVAL)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(0.1)
        resp = self.ser.read(9)
        return len(resp) == 9 and resp[:2] == b"\x90\xEB" and resp[4] == 0x12

    def set_speed(self, speeds) -> bool:
        return self._write_registers(0x05F2, [max(0, min(1000, int(v))) for v in speeds])

    def set_force(self, forces) -> bool:
        return self._write_registers(0x05DA, [max(0, min(1000, int(v))) for v in forces])

    def move_to_angle(self, angles) -> bool:
        values = []
        for angle in angles:
            if angle == -1:
                values.append(0xFFFF)
            else:
                values.append(max(0, min(1000, int(angle))))
        return self._write_registers(0x05CE, values)

    def open_hand(self) -> bool:
        return self.set_speed([300] * 6) and self.set_force([1000] * 6) and self.move_to_angle([1000] * 6)

    def grip_hand(self) -> bool:
        """原网页示教「抓紧」：四指与拇指指尖收紧。"""
        return self.set_speed([300] * 6) and self.set_force([1000] * 6) and self.move_to_angle([0, 0, 0, 0, 0, 1000])

    def close_four_fingers_keep_thumb(self) -> bool:
        return self.set_speed([300] * 6) and self.set_force([500] * 6) and self.move_to_angle([0, 0, 0, 0, -1, -1])

    def v_sign(self) -> bool:
        return (
            self.set_speed([300] * 6)
            and self.set_force([500] * 6)
            and self.move_to_angle([0, 0, 1000, 1000, 0, 1000])
        )

    def demo_v_sign(self) -> bool:
        if not self.open_hand():
            return False
        time.sleep(1.5)
        return self.v_sign()


def _diagnose_motor_action(action_cn: str, command_responded: bool, samples: list) -> list[str]:
    lines: list[str] = []
    if not command_responded:
        lines.append("电机控制器没有返回写指令响应，请检查快换电机 RS485、Modbus 地址与独立供电。")
        return lines
    if not samples:
        lines.append("没有采集到电机状态样本。")
        return lines
    lock_values = [s["lock"] for s in samples]
    orig_values = [s["orig"] for s in samples]
    if "0001" in lock_values:
        lines.append("锁紧成功：lock 状态已变成 0001。")
    if "0001" in orig_values:
        lines.append("回原点成功：orig 状态已变成 0001。")
    known_run = [v for v in (s["run"] for s in samples) if v is not None]
    if known_run and all(v == "0000" for v in known_run):
        lines.append("run 一直为停止，但写动作有响应。")
    elif known_run and any(v == "0001" for v in known_run):
        lines.append("电机已进入运行状态。")
    if not any(v == "0001" for v in lock_values + orig_values) and known_run and all(v == "0000" for v in known_run):
        lines.append(f"{action_cn}命令已下发，但 lock/orig 未变化，请检查机械与使能。")
    return lines


def _diagnose_action(action_cn: str, command_responded: bool, samples: list) -> list[str]:
    lines: list[str] = []
    if not command_responded:
        lines.append("电机控制器没有返回写指令响应，请检查 RS485、地址与供电。")
        return lines
    if not samples:
        lines.append("没有采集到监测样本。")
        return lines
    currents = [s["current"] for s in samples]
    current_delta = max(currents) - min(currents)
    lock_values = [s["lock"] for s in samples]
    orig_values = [s["orig"] for s in samples]
    cc_seen = any(s["cc_cv"] == "CC" for s in samples)
    if "0001" in lock_values:
        lines.append("锁紧成功：lock 状态已变成 0001。")
    if "0001" in orig_values:
        lines.append("回原点成功：orig 状态已变成 0001。")
    if cc_seen and max(currents) >= 1.8:
        lines.append("电机尝试动作但可能被限流或机械卡住。")
    if current_delta < 0.1:
        lines.append("控制器接受命令但电机未进入动力动作，疑似使能/刹车/互锁问题。")
    known_run = [v for v in (s["run"] for s in samples) if v is not None]
    if known_run and all(v == "0000" for v in known_run):
        lines.append("run 一直为停止，但写动作有响应。")
    lines.append(f"电流范围 {min(currents):.3f}~{max(currents):.3f} A，Δ={current_delta:.3f} A")
    return lines


class MotorPowerBus:
    """快换电机与末端各用独立串口，在各自锁内打开会话。"""

    @staticmethod
    @contextmanager
    def with_motor_session(
        port: str,
        baud: int,
        timeout: float,
        motor_slave: int = MOTOR_SLAVE_ADDR,
    ) -> Iterator[tuple]:
        if serial is None:
            raise RuntimeError("pyserial 未安装")
        with _motor_bus_lock:
            ser = serial.Serial(
                port, baud, bytesize=8, parity=serial.PARITY_NONE, stopbits=1,
                timeout=timeout, write_timeout=1.0, rtscts=False, dsrdtr=False,
            )
            try:
                yield ser, MotorController(ser, motor_slave)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

    @staticmethod
    @contextmanager
    def with_ee_session(
        port: str,
        baud: int,
        timeout: float,
        gripper_slave: int = GRIPPER_SLAVE_ADDR,
        power_slave: int = POWER_SLAVE_ADDR,
    ) -> Iterator[tuple]:
        if serial is None:
            raise RuntimeError("pyserial 未安装")
        with _ee_bus_lock:
            ser = serial.Serial(
                port, baud, bytesize=8, parity=serial.PARITY_NONE, stopbits=1,
                timeout=timeout, write_timeout=1.0, rtscts=False, dsrdtr=False,
            )
            try:
                power = CL200WPowerSupply(ser, power_slave)
                gripper = GripperController(ser, gripper_slave)
                yield ser, power, gripper
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

    @staticmethod
    def _motor_monitor(motor: MotorController, duration_s: float, interval_s: float) -> list:
        samples = []
        start = time.time()
        while True:
            loop_start = time.time()
            elapsed = loop_start - start
            sample = {"t": round(elapsed, 2), **motor.read_status_values()}
            samples.append(sample)
            if elapsed >= duration_s:
                break
            sleep_s = interval_s - (time.time() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
        return samples

    @staticmethod
    def _ee_power_monitor(power: CL200WPowerSupply, duration_s: float, interval_s: float) -> list:
        samples = []
        start = time.time()
        while True:
            loop_start = time.time()
            elapsed = loop_start - start
            power_status = power.read_basic_status()
            sample = {
                "t": round(elapsed, 2),
                "voltage": power_status["realtime_voltage_v"],
                "current": power_status["realtime_current_a"],
                "cc_cv": power_status["cc_cv"],
            }
            samples.append(sample)
            if elapsed >= duration_s:
                break
            sleep_s = interval_s - (time.time() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
        return samples

    @staticmethod
    def _monitor(power: CL200WPowerSupply, motor: MotorController, duration_s: float, interval_s: float) -> list:
        """实验室单总线联调用：电源 + 电机联合采样（现场双总线请分开调用）。"""
        samples = []
        start = time.time()
        while True:
            loop_start = time.time()
            elapsed = loop_start - start
            power_status = power.read_basic_status()
            motor_status = motor.read_status_values()
            sample = {
                "t": round(elapsed, 2),
                "voltage": power_status["realtime_voltage_v"],
                "current": power_status["realtime_current_a"],
                "cc_cv": power_status["cc_cv"],
                **motor_status,
            }
            samples.append(sample)
            if elapsed >= duration_s:
                break
            sleep_s = interval_s - (time.time() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
        return samples

    @classmethod
    def _switch_device(cls, power: CL200WPowerSupply, profile_key: str, confirm_power_on: bool) -> dict:
        profile = DEVICE_PROFILES[profile_key]
        power.output_off()
        time.sleep(0.1)
        power.set_voltage_v(profile["voltage_v"])
        time.sleep(0.05)
        power.set_current_a(profile["current_a"])
        time.sleep(0.05)
        if not confirm_power_on:
            return _fail(
                f"已设置 {profile['name']} 电源参数但未开输出，请确认接线后重试并带 confirm_power_on",
                needs_confirm=True,
                confirm_kind="power_on",
                profile=profile,
            )
        power.output_on()
        time.sleep(0.2)
        _device_manager_state["current_profile_key"] = profile_key
        _device_manager_state["current_device"] = profile["name"]
        status = power.read_status()
        return _ok(
            f"已切换到{profile['name']}：{profile['voltage_v']}V / {profile['current_a']}A，输出已开",
            current_device=profile["name"],
            power_status=status,
        )

    @classmethod
    def _ensure_device(cls, power: CL200WPowerSupply, device_name: str, auto_switch: bool) -> Optional[dict]:
        if _device_manager_state.get("current_device") == device_name:
            return None
        if not auto_switch:
            return _fail(f"当前设备不是{device_name}，请先切换设备档案")
        key = "1" if device_name == "灵巧手" else "2"
        return cls._switch_device(power, key, confirm_power_on=True)

    _MOTOR_ACTIONS = frozenset({
        "motor_forward", "motor_reverse", "motor_stop", "motor_status",
        "motor_lock_diagnosis", "motor_reset_diagnosis",
    })
    _EE_ACTIONS = frozenset({
        "bus_monitor",
        "power_set_voltage", "power_set_current", "power_output_on", "power_output_off", "power_status",
        "device_switch_hand", "device_switch_gripper",
        "gripper_open", "gripper_close", "gripper_demo",
    })

    @classmethod
    def _handle_motor_action(cls, action: str, payload: dict) -> dict:
        try:
            port, baud, timeout, motor_slave = parse_motor_params(payload)
        except Exception as e:
            return _fail(f"invalid motor params: {e}")
        with_monitor = payload.get("with_monitor") in (True, 1, "1", "true", "yes", "y")
        try:
            with cls.with_motor_session(port, baud, timeout, motor_slave) as (_ser, motor):
                if action in ("motor_forward", "motor_reverse", "motor_stop"):
                    key = action.replace("motor_", "")
                    labels = {"forward": "正转锁紧", "reverse": "反转复位", "stop": "停止"}
                    responded, resp_hex = motor.execute_action(key)
                    data: dict[str, Any] = {
                        "command_responded": responded,
                        "resp_hex": resp_hex,
                        "motor_slave_addr": motor.slave_addr,
                        "motor_port": port,
                    }
                    if with_monitor and responded:
                        data["samples"] = cls._motor_monitor(motor, ACTION_MONITOR_SECONDS, ACTION_MONITOR_INTERVAL)
                        data["diagnosis"] = _diagnose_motor_action(labels[key], responded, data["samples"])
                    if not responded:
                        return _fail(
                            f"快换电机地址 {motor.slave_addr} 在 {port} 无有效 Modbus 应答：{resp_hex or '无响应'}。"
                            "请确认快换电机接在 fr10-motor-serial 且独立供电，默认地址为 1。",
                            **data,
                        )
                    return _ok(f"电机{labels[key]}已下发", **data)

                if action == "motor_status":
                    values = motor.read_status_values()
                    report = motor.parse_status_report(values)
                    hint = None
                    if values.get("run") is None:
                        hint = f"未读到电机状态，请检查 {port} 上 Modbus 地址 {motor.slave_addr}"
                    return _ok("电机状态", status=report, motor_port=port, hint=hint)

                if action == "motor_lock_diagnosis":
                    responded, resp_hex = motor.execute_action("forward")
                    samples = cls._motor_monitor(motor, ACTION_MONITOR_SECONDS, ACTION_MONITOR_INTERVAL)
                    diagnosis = _diagnose_motor_action("正转锁紧", responded, samples)
                    return _ok(
                        "一键锁紧诊断完成",
                        samples=samples,
                        diagnosis=diagnosis,
                        command_responded=responded,
                        resp_hex=resp_hex,
                        motor_port=port,
                    )

                if action == "motor_reset_diagnosis":
                    responded, resp_hex = motor.execute_action("reverse")
                    samples = cls._motor_monitor(motor, ACTION_MONITOR_SECONDS, ACTION_MONITOR_INTERVAL)
                    diagnosis = _diagnose_motor_action("反转复位", responded, samples)
                    return _ok(
                        "一键复位诊断完成",
                        samples=samples,
                        diagnosis=diagnosis,
                        command_responded=responded,
                        resp_hex=resp_hex,
                        motor_port=port,
                    )

                return _fail(f"unknown motor action: {action}")
        except Exception as e:
            return _fail(str(e), motor_port=port)

    @classmethod
    def _handle_ee_action(cls, action: str, payload: dict) -> dict:
        try:
            port, baud, timeout, gripper_slave, power_slave = parse_ee_params(payload)
        except Exception as e:
            return _fail(f"invalid ee params: {e}")
        auto_switch = payload.get("auto_switch_device", True) not in (False, 0, "0", "false")
        confirm_power_on = payload.get("confirm_power_on") in (True, 1, "1", "true", "yes", "y")
        try:
            with cls.with_ee_session(port, baud, timeout, gripper_slave, power_slave) as (_ser, power, gripper):
                if action == "power_set_voltage":
                    v = float(payload["voltage_v"])
                    power.set_voltage_v(v)
                    return _ok(f"电源电压已设为 {v} V", ee_port=port)

                if action == "power_set_current":
                    a = float(payload["current_a"])
                    power.set_current_a(a)
                    return _ok(f"电源限流已设为 {a} A", ee_port=port)

                if action == "power_output_on":
                    if not confirm_power_on:
                        return _fail("打开电源输出需 confirm_power_on", needs_confirm=True, confirm_kind="power_on")
                    power.output_on()
                    return _ok("电源输出已打开", power_status=power.read_status(), ee_port=port)

                if action == "power_output_off":
                    power.output_off()
                    return _ok("电源输出已关闭", ee_port=port)

                if action == "power_status":
                    return _ok(
                        "电源状态",
                        power_status=power.read_status(),
                        current_device=_device_manager_state.get("current_device"),
                        ee_port=port,
                    )

                if action == "device_switch_hand":
                    if not confirm_power_on:
                        return _fail(
                            "请确认已接好灵巧手后再打开电源",
                            needs_confirm=True,
                            confirm_kind="power_on",
                            profile=DEVICE_PROFILES["1"],
                        )
                    out = cls._switch_device(power, "1", True)
                    out["ee_port"] = port
                    return out

                if action == "device_switch_gripper":
                    if not confirm_power_on:
                        return _fail(
                            "请确认已接好夹爪后再打开电源",
                            needs_confirm=True,
                            confirm_kind="power_on",
                            profile=DEVICE_PROFILES["2"],
                        )
                    out = cls._switch_device(power, "2", True)
                    out["ee_port"] = port
                    return out

                grip_actions = {
                    "gripper_open": ("夹爪张开", lambda: gripper.open_once()),
                    "gripper_close": ("夹爪闭合", lambda: gripper.close_once()),
                    "gripper_demo": ("夹爪演示", lambda: gripper.demo_close_then_open()),
                }
                if action in grip_actions:
                    label, fn = grip_actions[action]
                    block = cls._ensure_device(power, "夹爪", auto_switch)
                    if block is not None and not block.get("ok"):
                        block["ee_port"] = port
                        return block
                    out_on = power.read_output_setting()
                    if out_on == 0:
                        return _fail("夹爪需要 24V 电源输出已开；请先「切换夹爪 24V」", ee_port=port)
                    fn()
                    st = gripper.read_status()
                    return _ok(
                        f"{label}完成",
                        gripper_status=st,
                        gripper_slave_addr=gripper.slave_addr,
                        ee_port=port,
                    )

                if action == "bus_monitor":
                    duration_s = float(payload.get("duration_s", 10))
                    duration_s = max(0.5, min(duration_s, 60))
                    samples = cls._ee_power_monitor(power, duration_s, 0.5)
                    return _ok(f"末端电源连续监测 {duration_s}s 完成", samples=samples, ee_port=port)

                return _fail(f"unknown ee action: {action}")
        except Exception as e:
            return _fail(str(e), ee_port=port)

    @classmethod
    def handle_action(cls, action: str, payload: dict) -> dict:
        if serial is None:
            return _fail("pyserial 未安装，无法控制 RS485 总线")
        if action in cls._MOTOR_ACTIONS:
            return cls._handle_motor_action(action, payload)
        if action in cls._EE_ACTIONS:
            return cls._handle_ee_action(action, payload)
        return _fail(f"unknown bus action: {action}")


# 网页示教器历史导入名；与 MotorPowerBus 等价
BusActionHandler = MotorPowerBus
