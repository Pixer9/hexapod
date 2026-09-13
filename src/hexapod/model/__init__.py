"""Canonical robot geometry model."""

from .geometry import (
    CANONICAL_LEG_ORDER,
    LegMount,
    LegTransform,
    LinkLengths,
    RobotGeometry,
    Vec3,
    body_to_leg,
    build_leg_transforms,
    leg_to_body,
)
from .loader import GeometryConfigError, load_robot_geometry

__all__ = [
    "CANONICAL_LEG_ORDER",
    "GeometryConfigError",
    "LegMount",
    "LegTransform",
    "LinkLengths",
    "RobotGeometry",
    "Vec3",
    "body_to_leg",
    "build_leg_transforms",
    "leg_to_body",
    "load_robot_geometry",
]
