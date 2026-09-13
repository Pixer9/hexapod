"""Deterministic tripod gait foot trajectories in the BODY frame.

This module converts a planar ``MotionCommand`` plus an explicit gait phase into
six BODY-frame foot targets. It owns no wall clock, controller input, IK, servo
mapping, hardware I/O, lifecycle state, or command arbitration.

Coordinate convention:

* +X is forward.
* +Y is left.
* +Z is up.
* +yaw is counter-clockwise about +Z.

The trajectory uses quintic smootherstep for horizontal stance/swing motion and
an endpoint-flat sixth-order lift bump. Position, velocity, and acceleration are
continuous at stance/swing boundaries for a fixed command and blend.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping

from hexapod.control import MotionCommand
from hexapod.model import (
    CANONICAL_LEG_ORDER,
    RobotGeometry,
    Vec3,
    build_leg_transforms,
    leg_to_body,
)

from .config import TripodGaitConfig


TRIPOD_A: tuple[str, ...] = ("RF", "RB", "LM")
TRIPOD_B: tuple[str, ...] = ("RM", "LF", "LB")


class GaitError(ValueError):
    """Invalid gait input or trajectory request."""


@dataclass(frozen=True, slots=True)
class LegPhaseState:
    phase: float
    in_stance: bool
    progress: float


@dataclass(frozen=True, slots=True)
class GaitFrame:
    """One immutable six-foot trajectory sample."""

    global_phase: float
    blend: float
    command_scale: float
    foot_targets_body_mm: Mapping[str, Vec3]
    leg_states: Mapping[str, LegPhaseState]

    def __post_init__(self) -> None:
        if set(self.foot_targets_body_mm) != set(CANONICAL_LEG_ORDER):
            raise ValueError("foot targets must contain exactly the canonical six legs")
        if set(self.leg_states) != set(CANONICAL_LEG_ORDER):
            raise ValueError("leg states must contain exactly the canonical six legs")
        if not isinstance(self.foot_targets_body_mm, MappingProxyType):
            object.__setattr__(
                self,
                "foot_targets_body_mm",
                MappingProxyType(dict(self.foot_targets_body_mm)),
            )
        if not isinstance(self.leg_states, MappingProxyType):
            object.__setattr__(
                self,
                "leg_states",
                MappingProxyType(dict(self.leg_states)),
            )


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise GaitError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise GaitError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise GaitError(f"{name} must be a finite number")
    return result


def _wrap01(value: float) -> float:
    return value % 1.0


def _smootherstep(u: float) -> float:
    """Quintic 0..1 interpolation with zero first/second endpoint derivatives."""
    u = max(0.0, min(1.0, float(u)))
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def _swing_lift(u: float) -> float:
    """C2 endpoint-flat 0..1..0 lift profile with unit midpoint peak."""
    u = max(0.0, min(1.0, float(u)))
    return 64.0 * (u**3) * ((1.0 - u) ** 3)


def tripod_phase_for_leg(global_phase: float, leg_name: str) -> float:
    if leg_name not in CANONICAL_LEG_ORDER:
        raise GaitError(f"unknown leg name {leg_name!r}")
    offset = 0.5 if leg_name in TRIPOD_B else 0.0
    return _wrap01(global_phase + offset)


class TripodGait:
    """Pure tripod trajectory sampler for one robot geometry."""

    def __init__(self, geometry: RobotGeometry, config: TripodGaitConfig):
        self.geometry = geometry
        self.config = config
        transforms = build_leg_transforms(geometry)

        anchors: dict[str, Vec3] = {}
        for leg_name in geometry.leg_order:
            x, y, _ = leg_to_body(
                geometry.neutral_foot_leg_mm,
                transforms[leg_name],
            )
            anchors[leg_name] = (x, y, config.stance_z_mm)
        self._anchors = MappingProxyType(anchors)

    @property
    def stance_anchors_body_mm(self) -> Mapping[str, Vec3]:
        return self._anchors

    def sample(
        self,
        command: MotionCommand,
        global_phase: float,
        *,
        blend: float = 1.0,
    ) -> GaitFrame:
        """Sample all six BODY-frame foot targets.

        ``blend`` is a caller-owned 0..1 gait envelope for future start/stop
        transitions. It scales both horizontal excursion and foot lift. A zero
        ``MotionCommand`` always produces the flat stance even if ``blend`` is 1.
        """
        phase = _wrap01(_finite(global_phase, "global_phase"))
        blend_value = _finite(blend, "blend")
        if not 0.0 <= blend_value <= 1.0:
            raise GaitError("blend must be in [0, 1]")

        if command.is_zero:
            blend_value = 0.0

        raw_half_sweeps = self._raw_half_sweeps(command)
        max_raw = max(math.hypot(dx, dy) for dx, dy in raw_half_sweeps.values())
        if max_raw <= self.config.max_foot_offset_mm or max_raw == 0.0:
            command_scale = 1.0
        else:
            command_scale = self.config.max_foot_offset_mm / max_raw

        targets: dict[str, Vec3] = {}
        states: dict[str, LegPhaseState] = {}

        for leg_name in self.geometry.leg_order:
            local_phase = tripod_phase_for_leg(phase, leg_name)
            in_stance = local_phase < self.config.duty_factor

            if in_stance:
                progress = local_phase / self.config.duty_factor
            else:
                progress = (local_phase - self.config.duty_factor) / (
                    1.0 - self.config.duty_factor
                )

            progress = max(0.0, min(1.0, progress))
            s = _smootherstep(progress)

            raw_dx, raw_dy = raw_half_sweeps[leg_name]
            half_dx = raw_dx * command_scale * blend_value
            half_dy = raw_dy * command_scale * blend_value

            if in_stance:
                # Touch down ahead of the body-point motion and sweep backward.
                offset_x = half_dx * (1.0 - 2.0 * s)
                offset_y = half_dy * (1.0 - 2.0 * s)
                lift = 0.0
            else:
                # Return from rear to front while lifted.
                offset_x = half_dx * (-1.0 + 2.0 * s)
                offset_y = half_dy * (-1.0 + 2.0 * s)
                lift = self.config.step_height_mm * blend_value * _swing_lift(progress)

            anchor_x, anchor_y, anchor_z = self._anchors[leg_name]
            targets[leg_name] = (
                anchor_x + offset_x,
                anchor_y + offset_y,
                anchor_z + lift,
            )
            states[leg_name] = LegPhaseState(
                phase=local_phase,
                in_stance=in_stance,
                progress=progress,
            )

        return GaitFrame(
            global_phase=phase,
            blend=blend_value,
            command_scale=command_scale,
            foot_targets_body_mm=targets,
            leg_states=states,
        )

    def _raw_half_sweeps(
        self,
        command: MotionCommand,
    ) -> Mapping[str, tuple[float, float]]:
        """Return per-leg half stance sweep before global excursion limiting."""
        stance_time_s = self.config.duty_factor / self.config.cycle_hz
        half_time_s = 0.5 * stance_time_s
        omega_rad_s = math.radians(command.yaw_rate_deg_s)

        sweeps: dict[str, tuple[float, float]] = {}
        for leg_name in self.geometry.leg_order:
            x, y, _ = self._anchors[leg_name]

            # Rigid-body point velocity at this stance anchor:
            # v_point = v_body + omega x r.
            point_vx = command.vx_mm_s - omega_rad_s * y
            point_vy = command.vy_mm_s + omega_rad_s * x

            # During stance the foot moves oppositely relative to the body.
            # Starting at +half_sweep and ending at -half_sweep yields an
            # average relative velocity of -v_point over the stance interval.
            sweeps[leg_name] = (
                point_vx * half_time_s,
                point_vy * half_time_s,
            )

        return MappingProxyType(sweeps)
