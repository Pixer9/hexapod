"""Hardware-independent robot motion-command contract.

Coordinate/sign convention:

* +vx moves the body forward along +X.
* +vy moves the body left along +Y.
* +yaw_rate rotates counter-clockwise about +Z when viewed from above.

These values describe desired robot motion. They are not foot trajectories,
joint angles, servo commands, or HX1 frames.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


class MotionCommandError(ValueError):
    """Base class for invalid robot motion commands."""


class MotionLimitError(MotionCommandError):
    """A command exceeds the configured normal operating envelope."""


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise MotionCommandError(f"{name} must be a finite number")

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise MotionCommandError(f"{name} must be a finite number") from exc

    if not math.isfinite(result):
        raise MotionCommandError(f"{name} must be a finite number")

    return result


@dataclass(frozen=True, slots=True)
class MotionCommand:
    """Desired planar robot-body velocity."""

    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    yaw_rate_deg_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "vx_mm_s",
            _finite_float(self.vx_mm_s, "vx_mm_s"),
        )
        object.__setattr__(
            self,
            "vy_mm_s",
            _finite_float(self.vy_mm_s, "vy_mm_s"),
        )
        object.__setattr__(
            self,
            "yaw_rate_deg_s",
            _finite_float(self.yaw_rate_deg_s, "yaw_rate_deg_s"),
        )

    @classmethod
    def zero(cls) -> "MotionCommand":
        return cls()

    @property
    def is_zero(self) -> bool:
        return (
            self.vx_mm_s == 0.0
            and self.vy_mm_s == 0.0
            and self.yaw_rate_deg_s == 0.0
        )


@dataclass(frozen=True, slots=True)
class MotionLimits:
    """Normal Pi-side operating envelope for robot motion commands."""

    max_vx_mm_s: float
    max_vy_mm_s: float
    max_yaw_rate_deg_s: float

    def __post_init__(self) -> None:
        for name in (
            "max_vx_mm_s",
            "max_vy_mm_s",
            "max_yaw_rate_deg_s",
        ):
            value = _finite_float(getattr(self, name), name)
            if value <= 0.0:
                raise MotionCommandError(f"{name} must be > 0")
            object.__setattr__(self, name, value)

    def validate(self, command: MotionCommand) -> MotionCommand:
        """Return ``command`` unchanged when it is inside the envelope.

        This function deliberately rejects instead of silently clamping.
        Source adapters are responsible for mapping their normalized input into
        a valid physical command before publication.
        """
        if abs(command.vx_mm_s) > self.max_vx_mm_s:
            raise MotionLimitError(
                f"vx_mm_s={command.vx_mm_s} exceeds +/-{self.max_vx_mm_s}"
            )

        if abs(command.vy_mm_s) > self.max_vy_mm_s:
            raise MotionLimitError(
                f"vy_mm_s={command.vy_mm_s} exceeds +/-{self.max_vy_mm_s}"
            )

        if abs(command.yaw_rate_deg_s) > self.max_yaw_rate_deg_s:
            raise MotionLimitError(
                "yaw_rate_deg_s="
                f"{command.yaw_rate_deg_s} exceeds "
                f"+/-{self.max_yaw_rate_deg_s}"
            )

        return command
