"""Configuration loading for Pi-side locomotion."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


class GaitConfigError(ValueError):
    """Tripod gait configuration is malformed or unsupported."""


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise GaitConfigError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise GaitConfigError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise GaitConfigError(f"{name} must be a finite number")
    return result


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise GaitConfigError(f"{name} must be a non-empty string")
    return value


@dataclass(frozen=True, slots=True)
class TripodGaitConfig:
    schema_version: int
    gait_id: str
    cycle_hz: float
    duty_factor: float
    step_height_mm: float
    stance_z_mm: float
    max_foot_offset_mm: float


def parse_tripod_gait_config(data: object) -> TripodGaitConfig:
    if not isinstance(data, dict):
        raise GaitConfigError("root must be an object")

    if data.get("schema_version") != 1:
        raise GaitConfigError("schema_version must be 1")

    gait_id = _nonempty_string(data.get("gait_id"), "gait_id")
    cycle_hz = _finite_float(data.get("cycle_hz"), "cycle_hz")
    duty = _finite_float(data.get("duty_factor"), "duty_factor")
    height = _finite_float(data.get("step_height_mm"), "step_height_mm")
    stance_z = _finite_float(data.get("stance_z_mm"), "stance_z_mm")
    max_offset = _finite_float(
        data.get("max_foot_offset_mm"),
        "max_foot_offset_mm",
    )

    if cycle_hz <= 0.0:
        raise GaitConfigError("cycle_hz must be > 0")
    if not 0.5 <= duty < 1.0:
        raise GaitConfigError("duty_factor must be in [0.5, 1.0)")
    if height < 0.0:
        raise GaitConfigError("step_height_mm must be >= 0")
    if max_offset <= 0.0:
        raise GaitConfigError("max_foot_offset_mm must be > 0")

    expected = {
        "schema_version",
        "gait_id",
        "cycle_hz",
        "duty_factor",
        "step_height_mm",
        "stance_z_mm",
        "max_foot_offset_mm",
    }
    extra = set(data) - expected
    if extra:
        raise GaitConfigError(
            "unsupported gait config keys: " + ", ".join(sorted(extra))
        )

    return TripodGaitConfig(
        schema_version=1,
        gait_id=gait_id,
        cycle_hz=cycle_hz,
        duty_factor=duty,
        step_height_mm=height,
        stance_z_mm=stance_z,
        max_foot_offset_mm=max_offset,
    )


def load_tripod_gait_config(path: str | Path) -> TripodGaitConfig:
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise GaitConfigError(f"could not read gait config {path}: {exc}") from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GaitConfigError(f"invalid JSON in gait config {path}: {exc}") from exc

    return parse_tripod_gait_config(data)
