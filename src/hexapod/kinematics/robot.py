"""Whole-body kinematics for the six-leg robot.

This layer converts six BODY-frame foot targets into the canonical 18-element
logical joint vector expected by the Raspberry Pi motion pipeline.

It performs no servo calibration, channel mapping, direction inversion, trim,
pulse conversion, or hardware I/O. Those remain Servo 2040 responsibilities.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

from hexapod.model.geometry import (
    CANONICAL_LEG_ORDER,
    RobotGeometry,
    Vec3,
    body_to_leg,
    build_leg_transforms,
    leg_to_body,
)
from .leg import JointAngles, KinematicsError, leg_ik


JOINTS_PER_LEG = 3
LOGICAL_JOINT_COUNT = len(CANONICAL_LEG_ORDER) * JOINTS_PER_LEG


class WholeBodyKinematicsError(KinematicsError):
    """Base error for six-leg kinematics operations."""


class FootTargetSetError(WholeBodyKinematicsError):
    """The supplied BODY-frame foot-target set is malformed."""


class LegSolveError(WholeBodyKinematicsError):
    """One named leg could not be solved."""

    def __init__(self, leg_name: str, cause: Exception):
        self.leg_name = leg_name
        self.cause = cause
        super().__init__(f"{leg_name} leg solve failed: {cause}")


@dataclass(frozen=True, slots=True)
class RobotJointSolution:
    """Immutable canonical six-leg IK result.

    ``logical_vector_deg`` is ordered exactly:

        RF coxa, RF femur, RF tibia,
        RM coxa, RM femur, RM tibia,
        RB coxa, RB femur, RB tibia,
        LF coxa, LF femur, LF tibia,
        LM coxa, LM femur, LM tibia,
        LB coxa, LB femur, LB tibia.

    Values are mathematical logical joint angles in degrees, not physical
    Servo 2040 channel values.
    """

    leg_angles: Mapping[str, JointAngles]
    logical_vector_deg: tuple[float, ...]

    def __post_init__(self) -> None:
        if set(self.leg_angles) != set(CANONICAL_LEG_ORDER):
            raise ValueError("leg_angles must contain exactly the canonical six legs")

        if len(self.logical_vector_deg) != LOGICAL_JOINT_COUNT:
            raise ValueError(
                f"logical_vector_deg must contain {LOGICAL_JOINT_COUNT} values"
            )

        if not isinstance(self.leg_angles, MappingProxyType):
            object.__setattr__(
                self,
                "leg_angles",
                MappingProxyType(dict(self.leg_angles)),
            )

    def leg(self, leg_name: str) -> JointAngles:
        return self.leg_angles[leg_name]


def _normalize_target(point: object, name: str) -> Vec3:
    if not isinstance(point, (tuple, list)) or len(point) != 3:
        raise FootTargetSetError(f"{name} must contain exactly three coordinates")

    try:
        values = tuple(float(value) for value in point)
    except (TypeError, ValueError) as exc:
        raise FootTargetSetError(f"{name} must contain numeric coordinates") from exc

    if not all(math.isfinite(value) for value in values):
        raise FootTargetSetError(f"{name} must contain only finite coordinates")

    return values  # type: ignore[return-value]


def _normalize_target_set(
    foot_targets_body_mm: Mapping[str, Vec3],
) -> Mapping[str, Vec3]:
    if not isinstance(foot_targets_body_mm, Mapping):
        raise FootTargetSetError("foot targets must be a mapping keyed by leg name")

    supplied = set(foot_targets_body_mm)
    expected = set(CANONICAL_LEG_ORDER)

    if supplied != expected:
        missing = tuple(leg for leg in CANONICAL_LEG_ORDER if leg not in supplied)
        extra = tuple(sorted(supplied - expected))
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise FootTargetSetError(
            "foot targets must contain exactly the canonical six legs"
            + (": " + " ".join(details) if details else "")
        )

    normalized: dict[str, Vec3] = {}
    for leg_name in CANONICAL_LEG_ORDER:
        normalized[leg_name] = _normalize_target(
            foot_targets_body_mm[leg_name],
            f"foot target {leg_name}",
        )

    return MappingProxyType(normalized)


class RobotKinematics:
    """Reusable whole-body kinematics solver with precomputed leg transforms."""

    def __init__(self, geometry: RobotGeometry):
        self.geometry = geometry
        self.transforms = build_leg_transforms(geometry)

    def reference_stance_body(self) -> Mapping[str, Vec3]:
        """Return the configured reference local-leg stance in BODY coordinates."""
        targets: dict[str, Vec3] = {}

        for leg_name in self.geometry.leg_order:
            targets[leg_name] = leg_to_body(
                self.geometry.neutral_foot_leg_mm,
                self.transforms[leg_name],
            )

        return MappingProxyType(targets)

    def solve(
        self,
        foot_targets_body_mm: Mapping[str, Vec3],
    ) -> RobotJointSolution:
        """Solve all six BODY-frame foot targets atomically.

        No partial result is returned. If any leg fails, ``LegSolveError`` names
        the failed leg and retains the underlying exception as ``cause``.
        """
        targets = _normalize_target_set(foot_targets_body_mm)

        angles_by_leg: dict[str, JointAngles] = {}
        logical: list[float] = []

        for leg_name in self.geometry.leg_order:
            target_leg = body_to_leg(
                targets[leg_name],
                self.transforms[leg_name],
            )

            try:
                angles = leg_ik(
                    *target_leg,
                    self.geometry.links,
                )
            except KinematicsError as exc:
                raise LegSolveError(leg_name, exc) from exc

            angles_by_leg[leg_name] = angles
            logical.extend(
                (
                    angles.coxa_deg,
                    angles.femur_deg,
                    angles.tibia_deg,
                )
            )

        return RobotJointSolution(
            leg_angles=angles_by_leg,
            logical_vector_deg=tuple(logical),
        )
