#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""选点 GUI 缩放/平移：滚轮或触控板双指捏合放大，右键拖动平移。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import cv2
import numpy as np

ZOOM_STEP = 1.15
MIN_ZOOM = 1.0
MAX_ZOOM = 8.0

GUI_HELP = (
    "左键选点 | 滚轮/双指捏合缩放 | 右键拖动平移 | +/- 缩放 | f 复位 | "
    "m 移动 r 刷新 q 退出"
)


@dataclass
class ZoomPanView:
    img_w: int
    img_h: int
    zoom: float = 1.0
    center_u: float = field(init=False)
    center_v: float = field(init=False)
    _pan_active: bool = field(default=False, repr=False)
    _pan_last: tuple[int, int] | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        self.center_u = self.img_w / 2.0
        self.center_v = self.img_h / 2.0

    def reset(self) -> None:
        self.zoom = 1.0
        self.center_u = self.img_w / 2.0
        self.center_v = self.img_h / 2.0
        self._pan_active = False
        self._pan_last = None

    def visible_rect(self) -> tuple[float, float, float, float]:
        vw = self.img_w / self.zoom
        vh = self.img_h / self.zoom
        u0 = self.center_u - vw / 2.0
        v0 = self.center_v - vh / 2.0
        if vw >= self.img_w:
            u0 = 0.0
        else:
            u0 = float(np.clip(u0, 0.0, self.img_w - vw))
        if vh >= self.img_h:
            v0 = 0.0
        else:
            v0 = float(np.clip(v0, 0.0, self.img_h - vh))
        self.center_u = u0 + vw / 2.0
        self.center_v = v0 + vh / 2.0
        return u0, v0, vw, vh

    def display_to_image(self, x: int, y: int) -> tuple[float, float]:
        u0, v0, vw, vh = self.visible_rect()
        u = u0 + float(x) * vw / self.img_w
        v = v0 + float(y) * vh / self.img_h
        u = float(np.clip(u, 0.0, self.img_w - 1))
        v = float(np.clip(v, 0.0, self.img_h - 1))
        return u, v

    def zoom_at_display(self, x: int, y: int, direction: int) -> None:
        if direction == 0:
            return
        u_anchor, v_anchor = self.display_to_image(x, y)
        factor = ZOOM_STEP if direction > 0 else 1.0 / ZOOM_STEP
        new_zoom = float(np.clip(self.zoom * factor, MIN_ZOOM, MAX_ZOOM))
        if abs(new_zoom - self.zoom) < 1e-9:
            return
        self.zoom = new_zoom
        vw = self.img_w / self.zoom
        vh = self.img_h / self.zoom
        self.center_u = u_anchor - (float(x) / self.img_w - 0.5) * vw
        self.center_v = v_anchor - (float(y) / self.img_h - 0.5) * vh
        self.visible_rect()

    def zoom_at_center(self, direction: int) -> None:
        self.zoom_at_display(self.img_w // 2, self.img_h // 2, direction)

    def pan_by_display_delta(self, dx: int, dy: int) -> None:
        if self.zoom <= MIN_ZOOM + 1e-9:
            return
        _, _, vw, vh = self.visible_rect()
        self.center_u -= dx * vw / self.img_w
        self.center_v -= dy * vh / self.img_h
        self.visible_rect()

    def render(self, image: np.ndarray) -> np.ndarray:
        u0, v0, vw, vh = self.visible_rect()
        u0i = int(np.floor(u0))
        v0i = int(np.floor(v0))
        u1i = int(np.ceil(u0 + vw))
        v1i = int(np.ceil(v0 + vh))
        u0i = max(0, min(self.img_w - 1, u0i))
        v0i = max(0, min(self.img_h - 1, v0i))
        u1i = max(u0i + 1, min(self.img_w, u1i))
        v1i = max(v0i + 1, min(self.img_h, v1i))
        crop = image[v0i:v1i, u0i:u1i]
        if crop.size == 0:
            return image.copy()
        return cv2.resize(
            crop, (self.img_w, self.img_h), interpolation=cv2.INTER_LINEAR
        )

    def draw_hud(self, display: np.ndarray) -> None:
        if self.zoom <= MIN_ZOOM + 1e-9:
            return
        text = f"zoom {self.zoom:.1f}x  (f=reset)"
        cv2.putText(
            display,
            text,
            (12, self.img_h - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            text,
            (12, self.img_h - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (40, 40, 40),
            1,
            cv2.LINE_AA,
        )


def wheel_direction(flags: int) -> int:
    if hasattr(cv2, "getMouseWheelDelta"):
        delta = cv2.getMouseWheelDelta(flags)
        if delta > 0:
            return 1
        if delta < 0:
            return -1
    return 1 if flags > 0 else -1


def make_mouse_callback(
    view: ZoomPanView,
    on_image_click: Callable[[float, float], None],
) -> Callable:
    """左键在图像坐标选点；滚轮/捏合缩放；右键拖动平移。"""

    def on_mouse(event: int, x: int, y: int, flags: int, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            u, v = view.display_to_image(x, y)
            on_image_click(u, v)
            return

        wheel_event = getattr(cv2, "EVENT_MOUSEWHEEL", None)
        if wheel_event is not None and event == wheel_event:
            view.zoom_at_display(x, y, wheel_direction(flags))
            return

        if event == cv2.EVENT_RBUTTONDOWN:
            view._pan_active = True
            view._pan_last = (x, y)
            return

        if event == cv2.EVENT_RBUTTONUP:
            view._pan_active = False
            view._pan_last = None
            return

        if event == cv2.EVENT_MOUSEMOVE and view._pan_active and view._pan_last:
            lx, ly = view._pan_last
            view.pan_by_display_delta(x - lx, y - ly)
            view._pan_last = (x, y)

    return on_mouse


def handle_zoom_keys(view: ZoomPanView, key: int) -> bool:
    """处理 +/- / f；返回 True 表示已消费按键。"""
    if key in (ord("+"), ord("=")):
        view.zoom_at_center(1)
        return True
    if key == ord("-"):
        view.zoom_at_center(-1)
        return True
    if key in (ord("f"), ord("F")):
        view.reset()
        return True
    return False
