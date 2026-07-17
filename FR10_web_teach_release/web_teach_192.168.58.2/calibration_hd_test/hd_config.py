#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高分辨率测试用的 RealSense 取流配置（与主线 calibration/ 隔离）。"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class StreamProfile:
    name: str
    color_width: int
    color_height: int
    depth_width: int
    depth_height: int
    fps: int = 30

    @property
    def color_center(self) -> tuple[int, int]:
        return self.color_width // 2, self.color_height // 2

    @property
    def pixel_scale_vs_720p(self) -> float:
        return self.color_height / 720.0


LEGACY_PROFILE = StreamProfile(
    name="legacy",
    color_width=1280,
    color_height=720,
    depth_width=848,
    depth_height=480,
)

HD_PROFILE = StreamProfile(
    name="hd",
    color_width=1920,
    color_height=1080,
    depth_width=1280,
    depth_height=720,
)

PROFILES: dict[str, StreamProfile] = {
    LEGACY_PROFILE.name: LEGACY_PROFILE,
    HD_PROFILE.name: HD_PROFILE,
}
