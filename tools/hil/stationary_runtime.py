#!/usr/bin/env python3
"""Run the Milestone 2 stationary energized HX1 lifecycle on real hardware."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from hexapod.control import (
    CommandRateLimiter,
    load_command_rate_limit_config,
    load_motion_limits,
)
from hexapod.hx1 import (
    HX1ClientCore,
    HX1Link,
    create_serial_hx1_transport,
    load_hx1_client_config,
)
from hexapod.kinematics import RobotKinematics
from hexapod.locomotion import (
    LocomotionController,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
)
from hexapod.model import load_robot_geometry
from hexapod.motion import MotionPipeline
from hexapod.runtime import RobotRuntime, RobotRuntimeError
from hexapod.trajectory import (
    JointTrajectoryRateScaler,
    load_joint_soft_limit_profile,
    load_joint_trajectory_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]

ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
SOFT_LIMIT_CONFIG = REPO_ROOT / "config" / "robots" / "standard-joint-soft-limits.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
COMMAND_RATE_CONFIG = REPO_ROOT / "config" / "control" / "rate-limit.json"
JOINT_RATE_CONFIG = REPO_ROOT / "config" / "control" / "joint-trajectory.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
LOCOMOTION_CONFIG = REPO_ROOT / "config" / "locomotion" / "controller.json"
HX1_CONFIG = REPO_ROOT / "config" / "hardware" / "servo2040.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a stationary energized Pi -> Servo 2040 HX1 lifecycle. "
            "The robot must be mechanically supported."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="ACTIVE stationary target-stream duration in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--confirm-energized-hil",
        action="store_true",
        help=(
            "confirm that the robot is mechanically supported and that "
            "energized actuator HIL is intended"
        ),
    )
    return parser.parse_args()


def build_motion_pipeline() -> MotionPipeline:
    geometry = load_robot_geometry(ROBOT_CONFIG)
    motion_limits = load_motion_limits(MOTION_CONFIG)

    command_limiter = CommandRateLimiter(
        load_command_rate_limit_config(COMMAND_RATE_CONFIG),
        motion_limits,
    )

    gait = TripodGait(
        geometry,
        load_tripod_gait_config(GAIT_CONFIG),
    )

    return MotionPipeline(
        command_limiter=command_limiter,
        locomotion=LocomotionController(
            gait,
            load_locomotion_controller_config(LOCOMOTION_CONFIG),
        ),
        kinematics=RobotKinematics(geometry),
        soft_limits=load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG),
        joint_scaler=JointTrajectoryRateScaler(
            load_joint_trajectory_config(JOINT_RATE_CONFIG)
        ),
    )


def build_runtime() -> tuple[RobotRuntime, str]:
    hx1_config = load_hx1_client_config(HX1_CONFIG)
    client = HX1ClientCore(hx1_config)
    transport = create_serial_hx1_transport(hx1_config)
    link = HX1Link(client, transport)

    runtime = RobotRuntime(
        build_motion_pipeline(),
        link,
        hx1_config,
    )

    return runtime, hx1_config.device_path


def main() -> int:
    args = parse_args()

    if not args.confirm_energized_hil:
        print(
            "Refusing energized HIL without --confirm-energized-hil.",
            file=sys.stderr,
        )
        return 2

    if not 0.0 < args.duration <= 10.0:
        print(
            "--duration must be greater than 0 and no more than 10 seconds.",
            file=sys.stderr,
        )
        return 2

    runtime, device_path = build_runtime()

    print("Milestone 2 stationary energized HIL")
    print("Device:", device_path)
    print("ACTIVE duration:", f"{args.duration:.3f}s")
    print("Robot must remain mechanically supported.")
    print(
        "ARM may move unpowered servos to the configured flat stance "
        "before stationary streaming begins."
    )

    try:
        result = runtime.run_stationary(args.duration)
    except KeyboardInterrupt:
        print(
            "Interrupted. Runtime issued a best-effort ESTOP because "
            "energization may have occurred.",
            file=sys.stderr,
        )
        return 130
    except RobotRuntimeError as exc:
        print(f"Runtime failed: {exc}", file=sys.stderr)

        status = runtime.last_status
        if status is not None:
            print(
                "Last observed MCU status: "
                f"state={status.state} "
                f"fault={status.fault} "
                f"last_target_seq={status.last_target_seq} "
                f"target_age_ms={status.target_age_ms} "
                f"heartbeat_age_ms={status.heartbeat_age_ms}",
                file=sys.stderr,
            )

        print(
            "HX1 receive counters: "
            f"framing_errors={runtime.link.framing_errors} "
            f"protocol_errors={runtime.link.protocol_errors} "
            f"message_errors={runtime.link.message_errors}",
            file=sys.stderr,
        )

        return 1

    print()
    print("Stationary HX1 lifecycle successful.")
    print("Initial MCU state:", result.initial_status.state)
    print("Final MCU state:", result.final_status.state)
    print("Final MCU fault:", result.final_status.fault)
    print("TARGET frames sent:", result.target_frames_sent)
    print("HEARTBEAT frames sent:", result.heartbeat_frames_sent)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
