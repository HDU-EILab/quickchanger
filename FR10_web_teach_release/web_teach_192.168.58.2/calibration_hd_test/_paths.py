#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resolve pick_point calibration dir for Jetson and quickchanger repo layouts."""
from __future__ import annotations

from pathlib import Path


def pick_point_calib_dir() -> Path:
    hd_dir = Path(__file__).resolve().parent
    for candidate in (
        hd_dir.parent / "pick_point",  # quickchanger/calibration/pick_point
        hd_dir.parent / "calibration",  # jetson .../web_teach/calibration
    ):
        if (candidate / "pick_point_move.py").is_file():
            return candidate
    raise FileNotFoundError(
        "pick_point_move.py not found; expected sibling pick_point/ or calibration/"
    )
