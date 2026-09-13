"""Pi-side planning soft limits for canonical logical joint vectors.

These are robot-planning limits, not Servo 2040 actuator hard limits. They
exist so the Pi rejects a logically unsafe IK result before it reaches normal
trajectory streaming.

Validation is atomic: either the entire 18-joint vector is accepted or the
entire vector is rejected. Values are never silently clamped.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path

from hexapod.model import CANONICAL_LEG_ORDER

JOINTS_PER_LEG: tuple[str, ...] = ("coxa", "femur", "tibia")
CANONICAL_JOINT_NAMES: tuple[str, ...] = tuple(
    f"{leg.lower()}_{joint}"
    for leg in CANONICAL_LEG_ORDER
    for joint in JOINTS_PER_LEG
)
LOGICAL_JOINT_COUNT = len(CANONICAL_JOINT_NAMES)


class JointSoftLimitConfigError(ValueError):
    """Soft-limit configuration is malformed or unsupported."""


@dataclass(frozen=True, slots=True)
class JointSoftLimit:
    index: int
    name: str
    min_deg: float
    max_deg: float


@dataclass(frozen=True, slots=True)
class JointLimitViolation:
    index: int
    name: str
    value_deg: float
    min_deg: float
    max_deg: float


class JointVectorError(ValueError):
    """Canonical logical joint vector shape/value is invalid."""


class JointSoftLimitError(ValueError):
    """One or more joints violate the Pi planning soft envelope."""

    def __init__(self, violations: tuple[JointLimitViolation, ...]):
        self.violations = violations
        details = ", ".join(
            f"{item.name}={item.value_deg:.6f} not in "
            f"[{item.min_deg:.6f}, {item.max_deg:.6f}]"
            for item in violations
        )
        super().__init__(f"joint soft-limit violation: {details}")


@dataclass(frozen=True, slots=True)
class JointSoftLimitProfile:
    schema_version: int
    profile_id: str
    robot_id: str
    joints: tuple[JointSoftLimit, ...]

    def validate(
        self,
        vector_deg: Sequence[float],
    ) -> tuple[float, ...]:
        vector = _normalize_vector(vector_deg)

        violations: list[JointLimitViolation] = []
        for limit, value in zip(self.joints, vector):
            if value < limit.min_deg or value > limit.max_deg:
                violations.append(
                    JointLimitViolation(
                        index=limit.index,
                        name=limit.name,
                        value_deg=value,
                        min_deg=limit.min_deg,
                        max_deg=limit.max_deg,
                    )
                )

        if violations:
            raise JointSoftLimitError(tuple(violations))

        return vector


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise JointSoftLimitConfigError(
            f"{name} must be a finite number"
        )
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise JointSoftLimitConfigError(
            f"{name} must be a finite number"
        ) from exc
    if not math.isfinite(result):
        raise JointSoftLimitConfigError(
            f"{name} must be a finite number"
        )
    return result


def _normalize_vector(
    vector_deg: object,
) -> tuple[float, ...]:
    if (
        isinstance(vector_deg, (str, bytes))
        or not isinstance(vector_deg, Sequence)
        or len(vector_deg) != LOGICAL_JOINT_COUNT
    ):
        raise JointVectorError(
            f"joint vector must contain exactly "
            f"{LOGICAL_JOINT_COUNT} values"
        )

    result: list[float] = []
    for index, value in enumerate(vector_deg):
        if isinstance(value, bool):
            raise JointVectorError(
                f"joint vector index {index} must be a finite number"
            )
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise JointVectorError(
                f"joint vector index {index} must be a finite number"
            ) from exc
        if not math.isfinite(number):
            raise JointVectorError(
                f"joint vector index {index} must be a finite number"
            )
        result.append(number)
    return tuple(result)


def parse_joint_soft_limit_profile(
    data: object,
) -> JointSoftLimitProfile:
    if not isinstance(data, dict):
        raise JointSoftLimitConfigError("root must be an object")

    expected_root = {
        "schema_version",
        "profile_id",
        "robot_id",
        "angle_unit",
        "joint_count",
        "joints",
    }
    extra = set(data) - expected_root
    if extra:
        raise JointSoftLimitConfigError(
            "unsupported soft-limit profile keys: "
            + ", ".join(sorted(extra))
        )

    if data.get("schema_version") != 1:
        raise JointSoftLimitConfigError("schema_version must be 1")

    profile_id = data.get("profile_id")
    robot_id = data.get("robot_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise JointSoftLimitConfigError(
            "profile_id must be a non-empty string"
        )
    if not isinstance(robot_id, str) or not robot_id:
        raise JointSoftLimitConfigError(
            "robot_id must be a non-empty string"
        )
    if data.get("angle_unit") != "degree":
        raise JointSoftLimitConfigError(
            "angle_unit must be 'degree'"
        )
    if data.get("joint_count") != LOGICAL_JOINT_COUNT:
        raise JointSoftLimitConfigError(
            f"joint_count must be {LOGICAL_JOINT_COUNT}"
        )

    joints_raw = data.get("joints")
    if not isinstance(joints_raw, list):
        raise JointSoftLimitConfigError("joints must be an array")
    if len(joints_raw) != LOGICAL_JOINT_COUNT:
        raise JointSoftLimitConfigError(
            f"joints must contain {LOGICAL_JOINT_COUNT} entries"
        )

    joints: list[JointSoftLimit] = []
    for expected_index, item in enumerate(joints_raw):
        if not isinstance(item, dict):
            raise JointSoftLimitConfigError(
                f"joints[{expected_index}] must be an object"
            )

        expected_item = {"index", "name", "min_deg", "max_deg"}
        extra_item = set(item) - expected_item
        if extra_item:
            raise JointSoftLimitConfigError(
                f"unsupported joints[{expected_index}] keys: "
                + ", ".join(sorted(extra_item))
            )

        if item.get("index") != expected_index:
            raise JointSoftLimitConfigError(
                f"joints[{expected_index}].index must be "
                f"{expected_index}"
            )

        expected_name = CANONICAL_JOINT_NAMES[expected_index]
        if item.get("name") != expected_name:
            raise JointSoftLimitConfigError(
                f"joints[{expected_index}].name must be "
                f"{expected_name!r}"
            )

        minimum = _finite(
            item.get("min_deg"),
            f"joints[{expected_index}].min_deg",
        )
        maximum = _finite(
            item.get("max_deg"),
            f"joints[{expected_index}].max_deg",
        )
        if minimum >= maximum:
            raise JointSoftLimitConfigError(
                f"joints[{expected_index}] min_deg must be < max_deg"
            )

        joints.append(
            JointSoftLimit(
                index=expected_index,
                name=expected_name,
                min_deg=minimum,
                max_deg=maximum,
            )
        )

    return JointSoftLimitProfile(
        schema_version=1,
        profile_id=profile_id,
        robot_id=robot_id,
        joints=tuple(joints),
    )


def load_joint_soft_limit_profile(
    path: str | Path,
) -> JointSoftLimitProfile:
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise JointSoftLimitConfigError(
            f"could not read joint soft-limit profile {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise JointSoftLimitConfigError(
            f"invalid JSON in joint soft-limit profile {path}: {exc}"
        ) from exc

    return parse_joint_soft_limit_profile(data)
