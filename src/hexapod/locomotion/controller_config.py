"""Strict configuration for the deterministic locomotion controller."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


class LocomotionControllerConfigError(ValueError):
    """Locomotion-controller configuration is malformed or unsupported."""


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise LocomotionControllerConfigError(f"{name} must be a finite number")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LocomotionControllerConfigError(
            f"{name} must be a finite number"
        ) from exc

    if not math.isfinite(result):
        raise LocomotionControllerConfigError(f"{name} must be a finite number")

    return result


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise LocomotionControllerConfigError(f"{name} must be a non-empty string")
    return value


@dataclass(frozen=True, slots=True)
class LocomotionControllerConfig:
    schema_version: int
    controller_id: str
    start_blend_s: float
    stop_blend_s: float


def parse_locomotion_controller_config(
    data: object,
) -> LocomotionControllerConfig:
    if not isinstance(data, dict):
        raise LocomotionControllerConfigError("root must be an object")

    if data.get("schema_version") != 1:
        raise LocomotionControllerConfigError("schema_version must be 1")

    controller_id = _nonempty_string(
        data.get("controller_id"),
        "controller_id",
    )
    start_blend_s = _finite_float(
        data.get("start_blend_s"),
        "start_blend_s",
    )
    stop_blend_s = _finite_float(
        data.get("stop_blend_s"),
        "stop_blend_s",
    )

    if start_blend_s < 0.0:
        raise LocomotionControllerConfigError("start_blend_s must be >= 0")
    if stop_blend_s < 0.0:
        raise LocomotionControllerConfigError("stop_blend_s must be >= 0")

    expected = {
        "schema_version",
        "controller_id",
        "start_blend_s",
        "stop_blend_s",
    }
    extra = set(data) - expected
    if extra:
        raise LocomotionControllerConfigError(
            "unsupported locomotion controller config keys: " + ", ".join(sorted(extra))
        )

    return LocomotionControllerConfig(
        schema_version=1,
        controller_id=controller_id,
        start_blend_s=start_blend_s,
        stop_blend_s=stop_blend_s,
    )


def load_locomotion_controller_config(
    path: str | Path,
) -> LocomotionControllerConfig:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise LocomotionControllerConfigError(
            f"could not read locomotion controller config {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise LocomotionControllerConfigError(
            f"invalid JSON in locomotion controller config {path}: {exc}"
        ) from exc

    return parse_locomotion_controller_config(data)
