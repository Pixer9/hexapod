"""Canonical body/leg geometry for the Raspberry Pi runtime.

Coordinate convention:
    +X forward
    +Y left
    +Z up

The canonical leg order is part of the robot/protocol contract and is therefore
not a tunable runtime setting. Physical dimensions and mount locations are
configuration data.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, TypeAlias


Vec3: TypeAlias = tuple[float, float, float]

CANONICAL_LEG_ORDER: tuple[str, ...] = (
    "RF",
    "RM",
    "RB",
    "LF",
    "LM",
    "LB",
)


@dataclass(frozen=True, slots=True)
class LinkLengths:
    coxa_mm: float
    femur_mm: float
    tibia_mm: float


@dataclass(frozen=True, slots=True)
class LegMount:
    position_body_mm: Vec3
    yaw_deg: float


@dataclass(frozen=True, slots=True)
class LegTransform:
    """Precomputed BODY <-> LEG rigid transform for one leg mount."""

    position_body_mm: Vec3
    cos_yaw: float
    sin_yaw: float


@dataclass(frozen=True, slots=True)
class RobotGeometry:
    schema_version: int
    robot_id: str
    leg_order: tuple[str, ...]
    links: LinkLengths
    leg_mounts: Mapping[str, LegMount]
    neutral_foot_leg_mm: Vec3

    def __post_init__(self) -> None:
        # Prevent callers from mutating the loaded mount map after validation.
        if not isinstance(self.leg_mounts, MappingProxyType):
            object.__setattr__(
                self,
                "leg_mounts",
                MappingProxyType(dict(self.leg_mounts)),
            )

    def mount(self, leg_name: str) -> LegMount:
        return self.leg_mounts[leg_name]


def build_leg_transforms(geometry: RobotGeometry) -> Mapping[str, LegTransform]:
    transforms: dict[str, LegTransform] = {}

    for leg_name in geometry.leg_order:
        mount = geometry.mount(leg_name)
        yaw = math.radians(mount.yaw_deg)
        transforms[leg_name] = LegTransform(
            position_body_mm=mount.position_body_mm,
            cos_yaw=math.cos(yaw),
            sin_yaw=math.sin(yaw),
        )

    return MappingProxyType(transforms)


def body_to_leg(point_body_mm: Vec3, transform: LegTransform) -> Vec3:
    """Convert one BODY-frame point into a LEG-frame point."""
    px, py, pz = point_body_mm
    lx, ly, lz = transform.position_body_mm

    x = float(px) - lx
    y = float(py) - ly
    z = float(pz) - lz

    # Rotate by -yaw.
    xr = transform.cos_yaw * x + transform.sin_yaw * y
    yr = -transform.sin_yaw * x + transform.cos_yaw * y
    return (xr, yr, z)


def leg_to_body(point_leg_mm: Vec3, transform: LegTransform) -> Vec3:
    """Convert one LEG-frame point into a BODY-frame point."""
    x, y, z = point_leg_mm
    lx, ly, lz = transform.position_body_mm

    # Rotate by +yaw.
    xr = transform.cos_yaw * float(x) - transform.sin_yaw * float(y)
    yr = transform.sin_yaw * float(x) + transform.cos_yaw * float(y)
    return (xr + lx, yr + ly, float(z) + lz)
