"""Strict JSON loader for the Pi motion-command operating envelope."""

from __future__ import annotations

import json
import math
from pathlib import Path

from .command import MotionCommandError, MotionLimits


class MotionConfigError(ValueError):
    """Motion-control configuration is malformed or unsupported."""


def _require_dict(value: object, name: str) -> dict:
    if not isinstance(value, dict):
        raise MotionConfigError(f"{name} must be an object")
    return value


def _positive_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MotionConfigError(f"{name} must be a finite number > 0")

    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise MotionConfigError(f"{name} must be a finite number > 0")

    return result


def parse_motion_limits(data: object) -> MotionLimits:
    root = _require_dict(data, "root")

    if root.get("schema_version") != 1:
        raise MotionConfigError("schema_version must be 1")

    profile_id = root.get("profile_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise MotionConfigError("profile_id must be a non-empty string")

    units = _require_dict(root.get("units"), "units")
    if units.get("linear_velocity") != "millimeter_per_second":
        raise MotionConfigError("units.linear_velocity must be 'millimeter_per_second'")
    if units.get("yaw_rate") != "degree_per_second":
        raise MotionConfigError("units.yaw_rate must be 'degree_per_second'")

    limits = _require_dict(root.get("limits"), "limits")

    try:
        return MotionLimits(
            max_vx_mm_s=_positive_number(
                limits.get("max_vx_mm_s"),
                "limits.max_vx_mm_s",
            ),
            max_vy_mm_s=_positive_number(
                limits.get("max_vy_mm_s"),
                "limits.max_vy_mm_s",
            ),
            max_yaw_rate_deg_s=_positive_number(
                limits.get("max_yaw_rate_deg_s"),
                "limits.max_yaw_rate_deg_s",
            ),
        )
    except MotionCommandError as exc:
        raise MotionConfigError(str(exc)) from exc


def load_motion_limits(path: str | Path) -> MotionLimits:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise MotionConfigError(
            f"could not read motion-control config {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MotionConfigError(
            f"invalid JSON in motion-control config {path}: {exc}"
        ) from exc

    return parse_motion_limits(data)
