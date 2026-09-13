"""Strict JSON loader for configurable robot geometry."""

from __future__ import annotations

import json
import math
from pathlib import Path

from .geometry import (
    CANONICAL_LEG_ORDER,
    LegMount,
    LinkLengths,
    RobotGeometry,
)


class GeometryConfigError(ValueError):
    """Robot geometry configuration is malformed or unsupported."""


def _require_dict(value, name: str) -> dict:
    if not isinstance(value, dict):
        raise GeometryConfigError(f"{name} must be an object")
    return value


def _require_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GeometryConfigError(f"{name} must be a finite number")

    value = float(value)
    if not math.isfinite(value):
        raise GeometryConfigError(f"{name} must be a finite number")
    return value


def _require_positive_number(value, name: str) -> float:
    value = _require_number(value, name)
    if value <= 0.0:
        raise GeometryConfigError(f"{name} must be > 0")
    return value


def _require_vec3(value, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise GeometryConfigError(f"{name} must be an array of three numbers")

    return (
        _require_number(value[0], f"{name}[0]"),
        _require_number(value[1], f"{name}[1]"),
        _require_number(value[2], f"{name}[2]"),
    )


def parse_robot_geometry(data: object) -> RobotGeometry:
    root = _require_dict(data, "root")

    schema_version = root.get("schema_version")
    if schema_version != 1:
        raise GeometryConfigError("schema_version must be 1")

    robot_id = root.get("robot_id")
    if not isinstance(robot_id, str) or not robot_id:
        raise GeometryConfigError("robot_id must be a non-empty string")

    units = _require_dict(root.get("units"), "units")
    if units.get("length") != "millimeter":
        raise GeometryConfigError("units.length must be 'millimeter'")
    if units.get("angle") != "degree":
        raise GeometryConfigError("units.angle must be 'degree'")

    leg_order_raw = root.get("leg_order")
    if not isinstance(leg_order_raw, list):
        raise GeometryConfigError("leg_order must be an array")

    leg_order = tuple(leg_order_raw)
    if leg_order != CANONICAL_LEG_ORDER:
        raise GeometryConfigError(
            "leg_order must exactly match canonical order %r"
            % (CANONICAL_LEG_ORDER,)
        )

    links = _require_dict(root.get("links_mm"), "links_mm")
    link_lengths = LinkLengths(
        coxa_mm=_require_positive_number(links.get("coxa"), "links_mm.coxa"),
        femur_mm=_require_positive_number(links.get("femur"), "links_mm.femur"),
        tibia_mm=_require_positive_number(links.get("tibia"), "links_mm.tibia"),
    )

    mounts_raw = _require_dict(root.get("leg_mounts"), "leg_mounts")
    if set(mounts_raw) != set(CANONICAL_LEG_ORDER):
        raise GeometryConfigError(
            "leg_mounts must contain exactly the canonical six legs"
        )

    mounts: dict[str, LegMount] = {}
    for leg_name in CANONICAL_LEG_ORDER:
        item = _require_dict(mounts_raw[leg_name], f"leg_mounts.{leg_name}")
        mounts[leg_name] = LegMount(
            position_body_mm=_require_vec3(
                item.get("position_body_mm"),
                f"leg_mounts.{leg_name}.position_body_mm",
            ),
            yaw_deg=_require_number(
                item.get("yaw_deg"),
                f"leg_mounts.{leg_name}.yaw_deg",
            ),
        )

    reference = _require_dict(
        root.get("reference_stance"),
        "reference_stance",
    )
    neutral_foot_leg_mm = _require_vec3(
        reference.get("neutral_foot_leg_mm"),
        "reference_stance.neutral_foot_leg_mm",
    )

    return RobotGeometry(
        schema_version=schema_version,
        robot_id=robot_id,
        leg_order=leg_order,
        links=link_lengths,
        leg_mounts=mounts,
        neutral_foot_leg_mm=neutral_foot_leg_mm,
    )


def load_robot_geometry(path: str | Path) -> RobotGeometry:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise GeometryConfigError(
            f"could not read robot geometry file {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise GeometryConfigError(
            f"invalid JSON in robot geometry file {path}: {exc}"
        ) from exc

    return parse_robot_geometry(data)
