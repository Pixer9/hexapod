"""Deterministic phase and start/stop envelope for Pi-side locomotion.

This layer owns locomotion-local timing state only. It has no wall clock and
must be advanced with an explicit ``dt_s`` supplied by the caller.

It deliberately does not own command arbitration, command-rate limiting,
robot lifecycle/safety state, inverse kinematics, HX1, or hardware I/O.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from hexapod.control import MotionCommand

from .controller_config import LocomotionControllerConfig
from .tripod import GaitFrame, TripodGait


class LocomotionControllerError(ValueError):
    """Invalid deterministic locomotion-controller input."""


class LocomotionMode(str, Enum):
    """Internal gait transition mode, separate from robot lifecycle state."""

    IDLE = "idle"
    STARTING = "starting"
    MOVING = "moving"
    STOPPING = "stopping"


@dataclass(frozen=True, slots=True)
class LocomotionFrame:
    """One deterministic locomotion-controller output sample."""

    mode: LocomotionMode
    global_phase: float
    blend: float
    requested_command: MotionCommand
    trajectory_command: MotionCommand
    gait_frame: GaitFrame


def _finite_nonnegative(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise LocomotionControllerError(
            f"{name} must be a finite number >= 0"
        )

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise LocomotionControllerError(
            f"{name} must be a finite number >= 0"
        ) from exc

    if not math.isfinite(result) or result < 0.0:
        raise LocomotionControllerError(
            f"{name} must be a finite number >= 0"
        )

    return result


def _smooth_envelope(progress: float) -> float:
    """Quintic 0..1 envelope with zero first/second endpoint derivatives."""
    u = max(0.0, min(1.0, float(progress)))
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


class LocomotionController:
    """Own phase advancement and normal start/stop gait blending.

    A normal zero motion command initiates a smooth return to the flat stance.
    During that STOPPING transition, the last non-zero command is retained only
    as the trajectory shape being faded out. This is necessary because the pure
    gait sampler intentionally maps an actual zero command directly to flat
    stance.

    Safety STOP/ESTOP behavior is outside this class and must not wait for this
    normal transition.
    """

    def __init__(
        self,
        gait: TripodGait,
        config: LocomotionControllerConfig,
    ):
        self.gait = gait
        self.config = config
        self.reset()

    @property
    def mode(self) -> LocomotionMode:
        return self._mode

    @property
    def global_phase(self) -> float:
        return self._phase

    @property
    def blend(self) -> float:
        return _smooth_envelope(self._envelope_progress)

    @property
    def trajectory_command(self) -> MotionCommand:
        return self._trajectory_command

    def reset(self) -> None:
        """Return immediately to deterministic flat-stance controller state."""
        self._mode = LocomotionMode.IDLE
        self._phase = 0.0
        self._envelope_progress = 0.0
        self._trajectory_command = MotionCommand.zero()

    def step(
        self,
        command: MotionCommand,
        dt_s: float,
    ) -> LocomotionFrame:
        """Advance the controller by exactly ``dt_s`` and sample the gait.

        ``dt_s`` is explicit so identical input/delta-time sequences produce
        identical outputs independent of wall-clock scheduling.
        """
        if not isinstance(command, MotionCommand):
            raise LocomotionControllerError(
                "command must be a MotionCommand"
            )

        dt = _finite_nonnegative(dt_s, "dt_s")
        wants_motion = not command.is_zero

        self._apply_intent(command, wants_motion)

        if self._mode is not LocomotionMode.IDLE:
            self._phase = (
                self._phase
                + dt * self.gait.config.cycle_hz
            ) % 1.0

        self._advance_envelope(dt)

        # Completing a stop makes the next start deterministic and guarantees
        # the idle output is exactly the configured flat stance.
        if self._mode is LocomotionMode.IDLE:
            self._phase = 0.0
            self._trajectory_command = MotionCommand.zero()

        blend = self.blend
        gait_frame = self.gait.sample(
            self._trajectory_command,
            self._phase,
            blend=blend,
        )

        return LocomotionFrame(
            mode=self._mode,
            global_phase=self._phase,
            blend=blend,
            requested_command=command,
            trajectory_command=self._trajectory_command,
            gait_frame=gait_frame,
        )

    def _apply_intent(
        self,
        command: MotionCommand,
        wants_motion: bool,
    ) -> None:
        if self._mode is LocomotionMode.IDLE:
            if wants_motion:
                self._mode = LocomotionMode.STARTING
                self._trajectory_command = command
            return

        if self._mode is LocomotionMode.STARTING:
            if wants_motion:
                self._trajectory_command = command
            else:
                self._mode = LocomotionMode.STOPPING
            return

        if self._mode is LocomotionMode.MOVING:
            if wants_motion:
                self._trajectory_command = command
            else:
                self._mode = LocomotionMode.STOPPING
            return

        # STOPPING keeps the last non-zero trajectory command while a normal
        # zero request fades the gait envelope toward flat stance. A renewed
        # non-zero command reverses the envelope direction from its current
        # progress rather than restarting from zero.
        if wants_motion:
            self._mode = LocomotionMode.STARTING
            self._trajectory_command = command

    def _advance_envelope(self, dt: float) -> None:
        if self._mode is LocomotionMode.IDLE:
            self._envelope_progress = 0.0
            return

        if self._mode is LocomotionMode.MOVING:
            self._envelope_progress = 1.0
            return

        if self._mode is LocomotionMode.STARTING:
            duration = self.config.start_blend_s
            if duration == 0.0:
                self._envelope_progress = 1.0
            else:
                self._envelope_progress = min(
                    1.0,
                    self._envelope_progress + dt / duration,
                )

            if self._envelope_progress >= 1.0:
                self._mode = LocomotionMode.MOVING
            return

        duration = self.config.stop_blend_s
        if duration == 0.0:
            self._envelope_progress = 0.0
        else:
            self._envelope_progress = max(
                0.0,
                self._envelope_progress - dt / duration,
            )

        if self._envelope_progress <= 0.0:
            self._mode = LocomotionMode.IDLE
