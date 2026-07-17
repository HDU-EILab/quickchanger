#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RH56 灵巧手串口协议（与 rh56_robot_arm/set.py 中 RH56 类一致，V1.09 手册）。"""
from __future__ import annotations

import struct
import time
from typing import List

import serial


class RH56:
    """通信参数：115200bps, 8N1"""

    def __init__(self, port: str, hand_id: int = 1):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1.0,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )
        self.ID = hand_id
        self.REG_CLEAR_ERROR = 0x03EC
        self.REG_ANGLE_SET = 0x05CE
        self.REG_FORCE_SET = 0x05DA
        self.REG_SPEED_SET = 0x05F2
        self.REG_ANGLE_ACT = 0x060A

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _pack(self, rw: int, addr: int, data: bytes = b"") -> bytes:
        addr_l, addr_h = addr & 0xFF, (addr >> 8) & 0xFF
        if rw == 0x11:
            payload = bytes([self.ID, 0x04, rw, addr_l, addr_h]) + data
        else:
            payload = bytes([self.ID, len(data) + 3, rw, addr_l, addr_h]) + data
        checksum = sum(payload) & 0xFF
        return b"\xEB\x90" + payload + bytes([checksum])

    def _read(self, addr: int, length_b: int) -> bytes:
        tx_frame = self._pack(0x11, addr, bytes([length_b]))
        self.ser.write(tx_frame)
        rsp_len = 8 + length_b
        rsp_data = self.ser.read(rsp_len)
        if len(rsp_data) == rsp_len and rsp_data[:2] == b"\x90\xEB":
            return rsp_data[7:-1]
        return b""

    def _write(self, addr: int, data: bytes) -> bool:
        tx_frame = self._pack(0x12, addr, data)
        self.ser.write(tx_frame)
        ack_data = self.ser.read(9)
        return len(ack_data) == 9 and ack_data[:2] == b"\x90\xEB"

    def clear_errors(self) -> bool:
        return self._write(self.REG_CLEAR_ERROR, b"\x01")

    def get_actual_angle(self):
        raw_data = self._read(self.REG_ANGLE_ACT, 12)
        return struct.unpack("<6h", raw_data) if len(raw_data) == 12 else None

    def move_to_angle(self, angles: List[int]) -> bool:
        valid_angles = [0xFFFF if a == -1 else max(0, min(1000, a)) for a in angles]
        data = struct.pack("<6h", *valid_angles)
        return self._write(self.REG_ANGLE_SET, data)

    def set_force(self, forces: List[int]) -> bool:
        valid_forces = [max(0, min(1000, f)) for f in forces]
        data = struct.pack("<6h", *valid_forces)
        return self._write(self.REG_FORCE_SET, data)

    def set_speed(self, speeds: List[int]) -> bool:
        valid_speeds = [max(0, min(1000, s)) for s in speeds]
        data = struct.pack("<6h", *valid_speeds)
        return self._write(self.REG_SPEED_SET, data)


def rh56_grip_sequence(port: str, hand_id: int = 1, settle_s: float = 0.08) -> None:
    """抓紧：四指与拇指指尖收紧，拇指根部张开（同 set.py 末段）。"""
    hand = RH56(port=port, hand_id=hand_id)
    try:
        hand.clear_errors()
        time.sleep(settle_s)
        hand.set_speed([300] * 6)
        hand.set_force([1000] * 6)
        hand.move_to_angle([0, 0, 0, 0, 0, 1000])
    finally:
        hand.close()


def rh56_open_sequence(port: str, hand_id: int = 1, settle_s: float = 0.08) -> None:
    """打开：各指最大张开（1000）。"""
    hand = RH56(port=port, hand_id=hand_id)
    try:
        hand.clear_errors()
        time.sleep(settle_s)
        hand.set_speed([300] * 6)
        hand.set_force([1000] * 6)
        hand.move_to_angle([1000] * 6)
    finally:
        hand.close()
