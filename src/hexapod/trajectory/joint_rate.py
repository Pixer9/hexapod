"""Global rate scaling for canonical 18-joint trajectories.

This layer sits after whole-body IK and before the future HX1 output boundary.
It limits normal Pi-side commanded joint motion using one scale factor for the
entire canonical joint vector.

No joint is independently clipped or slewed. When one joint would exceed the
configured design rate, every joint delta for that update is multiplied by the
same factor. This preserves the requested joint-space segment direction while
stretching its execution time.

The configured rate is a Pi-side normal-motion design budget. It is not the
Servo 2040 hard actuator rate and does not qualify the robot for arming.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path

from hexapod.kinematics import LOGICAL_JOINT_COUNT


class JointTrajectoryError(ValueError):
    """Invalid joint-trajectory configuration, state, or request."""


@dataclass(frozen=True, slots=True)
class JointTrajectoryConfig:
    schema_version: int
    profile_id: str
    max_joint_rate_deg_s: float


@dataclass(frozen=True, slots=True)
class JointTrajectoryStep:
    """One globally scaled canonical joint-vector update."""

    target_vector_deg: tuple[float, ...]
    output_vector_deg: tuple[float, ...]
    scale: float
    limited: bool
    limiting_joint_index: int | None
    max_target_delta_deg: float
    required_duration_s: float
    peak_applied_rate_deg_s: float


def _positive_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise JointTrajectoryError(f"{name} must be a finite number > 0")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise JointTrajectoryError(f"{name} must be a finite number > 0") from exc

    if not math.isfinite(result) or result <= 0.0:
        raise JointTrajectoryError(f"{name} must be a finite number > 0")

    return result


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise JointTrajectoryError(f"{name} must be a finite number >= 0")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise JointTrajectoryError(f"{name} must be a finite number >= 0") from exc

    if not math.isfinite(result) or result < 0.0:
        raise JointTrajectoryError(f"{name} must be a finite number >= 0")

    return result


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise JointTrajectoryError(f"{name} must be a non-empty string")
    return value


def _normalize_vector(
    vector: object,
    name: str,
) -> tuple[float, ...]:
    if (
        isinstance(vector, (str, bytes))
        or not isinstance(vector, Sequence)
        or len(vector) != LOGICAL_JOINT_COUNT
    ):
        raise JointTrajectoryError(
            f"{name} must contain exactly {LOGICAL_JOINT_COUNT} joint angles"
        )

    normalized: list[float] = []
    for index, value in enumerate(vector):
        if isinstance(value, bool):
            raise JointTrajectoryError(f"{name}[{index}] must be a finite number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise JointTrajectoryError(
                f"{name}[{index}] must be a finite number"
            ) from exc

        if not math.isfinite(number):
            raise JointTrajectoryError(f"{name}[{index}] must be a finite number")
        normalized.append(number)

    return tuple(normalized)


def parse_joint_trajectory_config(
    data: object,
) -> JointTrajectoryConfig:
    if not isinstance(data, dict):
        raise JointTrajectoryError("root must be an object")

    if data.get("schema_version") != 1:
        raise JointTrajectoryError("schema_version must be 1")

    profile_id = _nonempty_string(
        data.get("profile_id"),
        "profile_id",
    )

    units = data.get("units")
    if not isinstance(units, dict):
        raise JointTrajectoryError("units must be an object")
    if units.get("joint_rate") != "degree_per_second":
        raise JointTrajectoryError("units.joint_rate must be 'degree_per_second'")

    limits = data.get("limits")
    if not isinstance(limits, dict):
        raise JointTrajectoryError("limits must be an object")

    expected_root = {
        "schema_version",
        "profile_id",
        "units",
        "limits",
    }
    extra_root = set(data) - expected_root
    if extra_root:
        raise JointTrajectoryError(
            "unsupported joint-trajectory config keys: " + ", ".join(sorted(extra_root))
        )

    if set(units) != {"joint_rate"}:
        extra = set(units) - {"joint_rate"}
        if extra:
            raise JointTrajectoryError(
                "unsupported joint-trajectory unit keys: " + ", ".join(sorted(extra))
            )

    expected_limits = {"max_joint_rate_deg_s"}
    extra_limits = set(limits) - expected_limits
    if extra_limits:
        raise JointTrajectoryError(
            "unsupported joint-trajectory limit keys: "
            + ", ".join(sorted(extra_limits))
        )

    return JointTrajectoryConfig(
        schema_version=1,
        profile_id=profile_id,
        max_joint_rate_deg_s=_positive_float(
            limits.get("max_joint_rate_deg_s"),
            "limits.max_joint_rate_deg_s",
        ),
    )


def load_joint_trajectory_config(
    path: str | Path,
) -> JointTrajectoryConfig:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise JointTrajectoryError(
            f"could not read joint-trajectory config {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise JointTrajectoryError(
            f"invalid JSON in joint-trajectory config {path}: {exc}"
        ) from exc

    return parse_joint_trajectory_config(data)


class JointTrajectoryRateScaler:
    """Stateful global rate scaler for canonical logical joint vectors.

    The scaler must be explicitly seeded with the currently commanded canonical
    joint vector before the first normal update. There is no universally safe
    all-zero physical pose, so this layer never invents an initial joint state.
    """

    def __init__(self, config: JointTrajectoryConfig):
        self.config = config
        self._output: tuple[float, ...] | None = None

    @property
    def initialized(self) -> bool:
        return self._output is not None

    @property
    def output_vector_deg(self) -> tuple[float, ...]:
        if self._output is None:
            raise JointTrajectoryError("joint trajectory scaler is not initialized")
        return self._output

    def seed(
        self,
        vector_deg: Sequence[float],
    ) -> tuple[float, ...]:
        """Explicitly set the currently commanded canonical joint vector."""
        normalized = _normalize_vector(
            vector_deg,
            "seed vector",
        )
        self._output = normalized
        return normalized

    def step(
        self,
        target_vector_deg: Sequence[float],
        dt_s: float,
    ) -> JointTrajectoryStep:
        """Move globally toward ``target_vector_deg`` within the rate budget."""
        if self._output is None:
            raise JointTrajectoryError(
                "joint trajectory scaler must be seeded before step()"
            )

        target = _normalize_vector(
            target_vector_deg,
            "target vector",
        )
        dt = _finite_nonnegative(dt_s, "dt_s")
        current = self._output

        deltas = tuple(
            target_value - current_value
            for current_value, target_value in zip(current, target)
        )
        abs_deltas = tuple(abs(delta) for delta in deltas)
        max_delta = max(abs_deltas)

        if max_delta == 0.0:
            step = JointTrajectoryStep(
                target_vector_deg=target,
                output_vector_deg=target,
                scale=1.0,
                limited=False,
                limiting_joint_index=None,
                max_target_delta_deg=0.0,
                required_duration_s=0.0,
                peak_applied_rate_deg_s=0.0,
            )
            self._output = target
            return step

        limiting_index = abs_deltas.index(max_delta)
        max_rate = self.config.max_joint_rate_deg_s
        required_duration = max_delta / max_rate
        allowed_delta = max_rate * dt

        if max_delta <= allowed_delta:
            scale = 1.0
            output = target
            limited = False
        elif allowed_delta == 0.0:
            scale = 0.0
            output = current
            limited = True
        else:
            scale = allowed_delta / max_delta
            output = tuple(
                current_value + scale * delta
                for current_value, delta in zip(current, deltas)
            )
            limited = True

        if dt == 0.0:
            peak_applied_rate = 0.0
        else:
            peak_applied_rate = max(
                abs(new_value - old_value) / dt
                for old_value, new_value in zip(current, output)
            )

        # Numerical tolerance only. This is an internal invariant rather than a
        # second independent clamp.
        tolerance = max(1e-9, max_rate * 1e-12)
        if peak_applied_rate > max_rate + tolerance:
            raise RuntimeError(
                "joint trajectory scaler violated configured rate budget"
            )

        self._output = output

        return JointTrajectoryStep(
            target_vector_deg=target,
            output_vector_deg=output,
            scale=scale,
            limited=limited,
            limiting_joint_index=limiting_index,
            max_target_delta_deg=max_delta,
            required_duration_s=required_duration,
            peak_applied_rate_deg_s=peak_applied_rate,
        )
