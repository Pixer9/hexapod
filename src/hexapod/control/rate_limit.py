"""Deterministic normal-command slew limiting.

This module shapes the already-selected planar ``MotionCommand`` before it
reaches locomotion. It deliberately owns no wall clock, source arbitration,
gait phase, inverse kinematics, lifecycle safety state, HX1, or hardware I/O.

Translation is limited as one 2-D velocity vector rather than as independent X
and Y axes. This avoids axis-order bias and preserves target direction when
accelerating from rest.

The limiter is for normal operator/autonomy command shaping. It is not an
ESTOP/fault mechanism and is not a substitute for Servo 2040 actuator safety
limits.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .._numeric import float_from_unknown
from .command import MotionCommand, MotionLimits


class CommandRateLimitError(ValueError):
    """Invalid command-rate limiter configuration or input."""


@dataclass(frozen=True, slots=True)
class CommandRateLimitConfig:
    schema_version: int
    profile_id: str
    translation_accel_mm_s2: float
    translation_decel_mm_s2: float
    yaw_accel_deg_s2: float
    yaw_decel_deg_s2: float


def _positive_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise CommandRateLimitError(f"{name} must be a finite number > 0")

    try:
        result = float_from_unknown(value)
    except (TypeError, ValueError) as exc:
        raise CommandRateLimitError(f"{name} must be a finite number > 0") from exc

    if not math.isfinite(result) or result <= 0.0:
        raise CommandRateLimitError(f"{name} must be a finite number > 0")

    return result


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise CommandRateLimitError(f"{name} must be a finite number >= 0")

    try:
        result = float_from_unknown(value)
    except (TypeError, ValueError) as exc:
        raise CommandRateLimitError(f"{name} must be a finite number >= 0") from exc

    if not math.isfinite(result) or result < 0.0:
        raise CommandRateLimitError(f"{name} must be a finite number >= 0")

    return result


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise CommandRateLimitError(f"{name} must be a non-empty string")
    return value


def parse_command_rate_limit_config(
    data: object,
) -> CommandRateLimitConfig:
    if not isinstance(data, dict):
        raise CommandRateLimitError("root must be an object")

    if data.get("schema_version") != 1:
        raise CommandRateLimitError("schema_version must be 1")

    profile_id = _nonempty_string(
        data.get("profile_id"),
        "profile_id",
    )

    units = data.get("units")
    if not isinstance(units, dict):
        raise CommandRateLimitError("units must be an object")
    if units.get("translation_acceleration") != "millimeter_per_second_squared":
        raise CommandRateLimitError(
            "units.translation_acceleration must be 'millimeter_per_second_squared'"
        )
    if units.get("yaw_acceleration") != "degree_per_second_squared":
        raise CommandRateLimitError(
            "units.yaw_acceleration must be 'degree_per_second_squared'"
        )

    limits = data.get("limits")
    if not isinstance(limits, dict):
        raise CommandRateLimitError("limits must be an object")

    expected_root = {
        "schema_version",
        "profile_id",
        "units",
        "limits",
    }
    extra_root = set(data) - expected_root
    if extra_root:
        raise CommandRateLimitError(
            "unsupported command-rate config keys: " + ", ".join(sorted(extra_root))
        )

    expected_units = {
        "translation_acceleration",
        "yaw_acceleration",
    }
    extra_units = set(units) - expected_units
    if extra_units:
        raise CommandRateLimitError(
            "unsupported command-rate unit keys: " + ", ".join(sorted(extra_units))
        )

    expected_limits = {
        "translation_accel_mm_s2",
        "translation_decel_mm_s2",
        "yaw_accel_deg_s2",
        "yaw_decel_deg_s2",
    }
    extra_limits = set(limits) - expected_limits
    if extra_limits:
        raise CommandRateLimitError(
            "unsupported command-rate limit keys: " + ", ".join(sorted(extra_limits))
        )

    return CommandRateLimitConfig(
        schema_version=1,
        profile_id=profile_id,
        translation_accel_mm_s2=_positive_float(
            limits.get("translation_accel_mm_s2"),
            "limits.translation_accel_mm_s2",
        ),
        translation_decel_mm_s2=_positive_float(
            limits.get("translation_decel_mm_s2"),
            "limits.translation_decel_mm_s2",
        ),
        yaw_accel_deg_s2=_positive_float(
            limits.get("yaw_accel_deg_s2"),
            "limits.yaw_accel_deg_s2",
        ),
        yaw_decel_deg_s2=_positive_float(
            limits.get("yaw_decel_deg_s2"),
            "limits.yaw_decel_deg_s2",
        ),
    )


def load_command_rate_limit_config(
    path: str | Path,
) -> CommandRateLimitConfig:
    path = Path(path)

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise CommandRateLimitError(
            f"could not read command-rate config {path}: {exc}"
        ) from exc

    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        raise CommandRateLimitError(
            f"invalid JSON in command-rate config {path}: {exc}"
        ) from exc

    return parse_command_rate_limit_config(data)


def _move_vector_toward(
    current: tuple[float, float],
    target: tuple[float, float],
    max_delta: float,
) -> tuple[float, float]:
    cx, cy = current
    tx, ty = target
    dx = tx - cx
    dy = ty - cy
    distance = math.hypot(dx, dy)

    if distance == 0.0 or max_delta <= 0.0:
        return current
    if distance <= max_delta:
        return target

    scale = max_delta / distance
    return (cx + dx * scale, cy + dy * scale)


def _decelerate_vector_to_zero(
    current: tuple[float, float],
    max_delta: float,
) -> tuple[float, float]:
    magnitude = math.hypot(*current)
    if magnitude == 0.0 or max_delta <= 0.0:
        return current
    if magnitude <= max_delta:
        return (0.0, 0.0)

    scale = (magnitude - max_delta) / magnitude
    return (current[0] * scale, current[1] * scale)


def _slew_translation(
    current: tuple[float, float],
    target: tuple[float, float],
    dt_s: float,
    accel: float,
    decel: float,
) -> tuple[float, float]:
    cx, cy = current
    tx, ty = target
    current_speed = math.hypot(cx, cy)
    target_speed = math.hypot(tx, ty)

    if dt_s == 0.0 or current == target:
        return current

    if current_speed == 0.0:
        return _move_vector_toward(
            current,
            target,
            accel * dt_s,
        )

    if target_speed == 0.0:
        return _decelerate_vector_to_zero(
            current,
            decel * dt_s,
        )

    dot = cx * tx + cy * ty

    # A target more than 90 degrees away is a true direction reversal. Do not
    # rotate the current velocity vector through zero at the acceleration rate.
    # First remove the existing velocity using the deceleration budget, then
    # use any leftover time to accelerate toward the new target.
    if dot < 0.0:
        time_to_zero = current_speed / decel
        if dt_s <= time_to_zero:
            return _decelerate_vector_to_zero(
                current,
                decel * dt_s,
            )

        remaining = dt_s - time_to_zero
        return _move_vector_toward(
            (0.0, 0.0),
            target,
            accel * remaining,
        )

    # For a non-reversing turn, classify the entire straight command-space
    # segment with a value whose sign cannot change as that segment is
    # consumed. This keeps the result independent of timestep partitioning.
    #
    # For collinear same-direction motion this reduces to the intuitive rule:
    # slowing down uses decel, speeding up uses accel.
    delta_x = tx - cx
    delta_y = ty - cy
    target_delta_projection = tx * delta_x + ty * delta_y

    rate = decel if target_delta_projection < 0.0 else accel

    return _move_vector_toward(
        current,
        target,
        rate * dt_s,
    )


def _move_scalar_toward(
    current: float,
    target: float,
    max_delta: float,
) -> float:
    delta = target - current
    if delta == 0.0 or max_delta <= 0.0:
        return current
    if abs(delta) <= max_delta:
        return target
    return current + math.copysign(max_delta, delta)


def _slew_yaw(
    current: float,
    target: float,
    dt_s: float,
    accel: float,
    decel: float,
) -> float:
    if dt_s == 0.0 or current == target:
        return current

    if current == 0.0:
        return _move_scalar_toward(
            current,
            target,
            accel * dt_s,
        )

    if target == 0.0:
        return _move_scalar_toward(
            current,
            0.0,
            decel * dt_s,
        )

    if current * target < 0.0:
        time_to_zero = abs(current) / decel
        if dt_s <= time_to_zero:
            return _move_scalar_toward(
                current,
                0.0,
                decel * dt_s,
            )

        remaining = dt_s - time_to_zero
        return _move_scalar_toward(
            0.0,
            target,
            accel * remaining,
        )

    rate = decel if abs(target) < abs(current) else accel
    return _move_scalar_toward(
        current,
        target,
        rate * dt_s,
    )


class CommandRateLimiter:
    """Stateful deterministic slew limiter for normal planar commands."""

    def __init__(
        self,
        config: CommandRateLimitConfig,
        limits: MotionLimits,
    ):
        self.config = config
        self.limits = limits
        self._output = MotionCommand.zero()

    @property
    def output(self) -> MotionCommand:
        return self._output

    def reset(
        self,
        command: MotionCommand | None = None,
    ) -> MotionCommand:
        """Hard-set limiter state.

        A hard reset is intentionally explicit. The future safety/runtime layer
        may use this boundary when normal command shaping must be bypassed.
        """
        value = MotionCommand.zero() if command is None else command
        if not isinstance(value, MotionCommand):
            raise CommandRateLimitError("reset command must be a MotionCommand")
        self.limits.validate(value)
        self._output = value
        return value

    def step(
        self,
        target: MotionCommand,
        dt_s: float,
    ) -> MotionCommand:
        """Advance the limiter by exactly ``dt_s`` toward ``target``."""
        if not isinstance(target, MotionCommand):
            raise CommandRateLimitError("target must be a MotionCommand")

        self.limits.validate(target)
        dt = _finite_nonnegative(dt_s, "dt_s")

        vx, vy = _slew_translation(
            (
                self._output.vx_mm_s,
                self._output.vy_mm_s,
            ),
            (
                target.vx_mm_s,
                target.vy_mm_s,
            ),
            dt,
            self.config.translation_accel_mm_s2,
            self.config.translation_decel_mm_s2,
        )

        yaw = _slew_yaw(
            self._output.yaw_rate_deg_s,
            target.yaw_rate_deg_s,
            dt,
            self.config.yaw_accel_deg_s2,
            self.config.yaw_decel_deg_s2,
        )

        result = MotionCommand(
            vx_mm_s=vx,
            vy_mm_s=vy,
            yaw_rate_deg_s=yaw,
        )
        self.limits.validate(result)
        self._output = result
        return result
