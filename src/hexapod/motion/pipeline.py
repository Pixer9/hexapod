"""Deterministic composition of the Pi-side normal motion pipeline.

This layer connects the already-established normal-motion stages:

    MotionCommand
        -> CommandRateLimiter
        -> LocomotionController / TripodGait
        -> RobotKinematics
        -> JointSoftLimitProfile
        -> JointTrajectoryRateScaler
        -> canonical 18-joint output

It owns no wall clock, command-source arbitration, HX1 transport, lifecycle
authority, heartbeat scheduling, target scheduling, or hardware I/O.
"""

from __future__ import annotations

from dataclasses import dataclass

from hexapod.control import CommandRateLimiter, MotionCommand
from hexapod.kinematics import RobotJointSolution, RobotKinematics
from hexapod.locomotion import LocomotionController, LocomotionFrame
from hexapod.trajectory import (
    JointSoftLimitProfile,
    JointTrajectoryRateScaler,
    JointTrajectoryStep,
)


class MotionPipelineError(RuntimeError):
    """The composed motion pipeline is not in a usable deterministic state."""


@dataclass(frozen=True, slots=True)
class MotionPipelineInitialization:
    """Flat-stance initialization result used by the future runtime STAGE step."""

    locomotion_frame: LocomotionFrame
    joint_solution: RobotJointSolution
    staged_joint_vector_deg: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class MotionPipelineFrame:
    """One complete deterministic Pi-side normal-motion update."""

    requested_command: MotionCommand
    shaped_command: MotionCommand
    locomotion_frame: LocomotionFrame
    joint_solution: RobotJointSolution
    target_joint_vector_deg: tuple[float, ...]
    trajectory_step: JointTrajectoryStep

    @property
    def output_joint_vector_deg(self) -> tuple[float, ...]:
        """Return the final canonical vector intended for HX1 TARGET."""
        return self.trajectory_step.output_vector_deg


class MotionPipeline:
    """Compose normal motion from robot command through final joint trajectory.

    ``initialize_flat_stance()`` must be called before ``step()``. Initialization
    computes the exact configured gait flat stance, validates it, and seeds the
    joint-rate scaler with the same vector that the future runtime must STAGE.

    Any exception during a normal update latches this object failed. That avoids
    continuing after an upstream stateful component may have advanced while a
    downstream solve or validation failed. Recovery requires explicit
    reinitialization while the higher-level runtime is in a safe lifecycle
    state.
    """

    def __init__(
        self,
        command_limiter: CommandRateLimiter,
        locomotion: LocomotionController,
        kinematics: RobotKinematics,
        soft_limits: JointSoftLimitProfile,
        joint_scaler: JointTrajectoryRateScaler,
    ):
        if not isinstance(command_limiter, CommandRateLimiter):
            raise TypeError("command_limiter must be a CommandRateLimiter")
        if not isinstance(locomotion, LocomotionController):
            raise TypeError("locomotion must be a LocomotionController")
        if not isinstance(kinematics, RobotKinematics):
            raise TypeError("kinematics must be a RobotKinematics")
        if not isinstance(soft_limits, JointSoftLimitProfile):
            raise TypeError("soft_limits must be a JointSoftLimitProfile")
        if not isinstance(joint_scaler, JointTrajectoryRateScaler):
            raise TypeError("joint_scaler must be a JointTrajectoryRateScaler")

        self.command_limiter = command_limiter
        self.locomotion = locomotion
        self.kinematics = kinematics
        self.soft_limits = soft_limits
        self.joint_scaler = joint_scaler

        self._initialized = False
        self._failed = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def failed(self) -> bool:
        return self._failed

    @property
    def output_joint_vector_deg(self) -> tuple[float, ...]:
        if not self._initialized:
            raise MotionPipelineError("motion pipeline is not initialized")
        if self._failed:
            raise MotionPipelineError("motion pipeline is failed")
        return self.joint_scaler.output_vector_deg

    def initialize_flat_stance(self) -> MotionPipelineInitialization:
        """Reset normal-motion state and seed the configured gait flat stance.

        This method performs no hardware I/O. The returned joint vector is the
        exact vector that the future runtime must send with HX1 STAGE before
        arming, so the software trajectory seed and MCU staged command agree.
        """
        self._initialized = False
        self._failed = False

        self.command_limiter.reset()
        self.locomotion.reset()

        try:
            locomotion_frame = self.locomotion.step(
                MotionCommand.zero(),
                0.0,
            )
            joint_solution = self.kinematics.solve(
                locomotion_frame.gait_frame.foot_targets_body_mm
            )
            staged = self.soft_limits.validate(joint_solution.logical_vector_deg)
            seeded = self.joint_scaler.seed(staged)
        except Exception:
            self._failed = True
            raise

        self._initialized = True

        return MotionPipelineInitialization(
            locomotion_frame=locomotion_frame,
            joint_solution=joint_solution,
            staged_joint_vector_deg=seeded,
        )

    def step(
        self,
        command: MotionCommand,
        dt_s: float,
    ) -> MotionPipelineFrame:
        """Advance the entire normal-motion pipeline by exactly ``dt_s``."""
        if not self._initialized:
            raise MotionPipelineError(
                "initialize_flat_stance() must be called before step()"
            )

        if self._failed:
            raise MotionPipelineError(
                "motion pipeline is failed; reinitialize before continuing"
            )

        if not isinstance(command, MotionCommand):
            self._failed = True
            raise TypeError("command must be a MotionCommand")

        try:
            shaped_command = self.command_limiter.step(
                command,
                dt_s,
            )
            locomotion_frame = self.locomotion.step(
                shaped_command,
                dt_s,
            )
            joint_solution = self.kinematics.solve(
                locomotion_frame.gait_frame.foot_targets_body_mm
            )
            target_joint_vector = self.soft_limits.validate(
                joint_solution.logical_vector_deg
            )
            trajectory_step = self.joint_scaler.step(
                target_joint_vector,
                dt_s,
            )
        except Exception:
            self._failed = True
            raise

        return MotionPipelineFrame(
            requested_command=command,
            shaped_command=shaped_command,
            locomotion_frame=locomotion_frame,
            joint_solution=joint_solution,
            target_joint_vector_deg=target_joint_vector,
            trajectory_step=trajectory_step,
        )
