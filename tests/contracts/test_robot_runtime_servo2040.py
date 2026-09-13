"""Cross-boundary tests for the Milestone 2 Pi robot runtime."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PI_SRC = REPO_ROOT / "src"
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"

sys.path.insert(0, str(FIRMWARE_SRC))
sys.path.insert(0, str(PI_SRC))

# isort: off

from hexapod.control import (  # noqa: E402
    CommandRateLimiter,
    load_command_rate_limit_config,
    load_motion_limits,
)
from hexapod.hx1 import (  # noqa: E402
    HX1ClientCore,
    HX1Link,
    HX1TransportClosedError,
    HX1TransportValueError,
    load_hx1_client_config,
)
from hexapod.kinematics import RobotKinematics  # noqa: E402
from hexapod.locomotion import (  # noqa: E402
    LocomotionController,
    TripodGait,
    load_locomotion_controller_config,
    load_tripod_gait_config,
)
from hexapod.model import load_robot_geometry  # noqa: E402
from hexapod.motion import MotionPipeline  # noqa: E402
from hexapod.runtime import RobotRuntime  # noqa: E402
from hexapod.trajectory import (  # noqa: E402
    JointTrajectoryRateScaler,
    load_joint_soft_limit_profile,
    load_joint_trajectory_config,
)

from hexapod_mcu.hardware import HardwareError  # noqa: E402
from hexapod_mcu.profile import load_profile  # noqa: E402
from hexapod_mcu.protocol import parse_frame as mcu_parse_frame  # noqa: E402
from hexapod_mcu.runtime import RuntimeCoordinator  # noqa: E402
from hexapod_mcu.state_machine import (  # noqa: E402
    RuntimeStateMachine,
    State as MCUState,
)
from hexapod_mcu.telemetry import TelemetryEncoder  # noqa: E402

# isort: on

ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
SOFT_LIMIT_CONFIG = REPO_ROOT / "config" / "robots" / "standard-joint-soft-limits.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
COMMAND_RATE_CONFIG = REPO_ROOT / "config" / "control" / "rate-limit.json"
JOINT_RATE_CONFIG = REPO_ROOT / "config" / "control" / "joint-trajectory.json"
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
LOCOMOTION_CONFIG = REPO_ROOT / "config" / "locomotion" / "controller.json"
HX1_CONFIG = REPO_ROOT / "config" / "hardware" / "servo2040.json"
ACTUATOR_PROFILE = FIRMWARE_SRC / "config" / "actuator-profile.json"

SESSION_INT = 0x3ADFBFED


class FakeClock:
    def __init__(self, *, interrupt_at_s: float | None = None):
        self.now_s = 0.0
        self.interrupt_at_s = interrupt_at_s

    def monotonic(self) -> float:
        return self.now_s

    def sleep(self, duration_s: float) -> None:
        if duration_s < 0.0:
            raise ValueError("duration_s must be non-negative")

        self.now_s += duration_s

        if self.interrupt_at_s is not None and self.now_s >= self.interrupt_at_s:
            self.interrupt_at_s = None
            raise KeyboardInterrupt


class LoopbackHardware:
    def __init__(self):
        self.enabled = False
        self.enable_targets: list[tuple[int, ...]] = []
        self.applied_targets: list[tuple[int, ...]] = []

    def force_disabled(self):
        self.enabled = False

    def require_profile_compatible(self, profile):
        return True

    def enable_at_target(self, target):
        self.enabled = True
        self.enable_targets.append(tuple(target))

    def apply_target(self, target):
        if not self.enabled:
            raise HardwareError("loopback hardware is not enabled")

        self.applied_targets.append(tuple(target))


class FirmwareLoopbackTransport:
    """Route Pi HX1 bytes directly through the real Servo 2040 runtime."""

    def __init__(
        self,
        runtime: RuntimeCoordinator,
        clock: FakeClock,
    ):
        self.runtime = runtime
        self.clock = clock
        self._is_open = False
        self._read_buffer = bytearray()
        self.commands: list[str] = []

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    def write(self, data: bytes | bytearray) -> int:
        self._require_open()

        if not isinstance(data, (bytes, bytearray)):
            raise HX1TransportValueError(
                "transport write data must be bytes or bytearray"
            )

        payload = bytes(data)
        _, command, _ = mcu_parse_frame(payload)
        self.commands.append(command)

        responses = self.runtime.handle_frame(
            payload,
            self._now_ms(),
        )

        for response in responses:
            self._read_buffer.extend(response)

        return len(payload)

    def read(self, max_bytes: int = 4096) -> bytes:
        self._require_open()

        if (
            isinstance(max_bytes, bool)
            or not isinstance(max_bytes, int)
            or max_bytes <= 0
        ):
            raise HX1TransportValueError("max_bytes must be a positive integer")

        for event in self.runtime.tick(self._now_ms()):
            self._read_buffer.extend(event)

        if not self._read_buffer:
            return b""

        count = min(max_bytes, len(self._read_buffer))
        data = bytes(self._read_buffer[:count])
        del self._read_buffer[:count]
        return data

    def _now_ms(self) -> int:
        return int(round(self.clock.monotonic() * 1000.0))

    def _require_open(self) -> None:
        if not self._is_open:
            raise HX1TransportClosedError("HX1 transport is not open")


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


def build_runtime_stack(*, interrupt_at_s: float | None = None):
    profile = load_profile(ACTUATOR_PROFILE)
    state_machine = RuntimeStateMachine()
    hardware = LoopbackHardware()
    telemetry = TelemetryEncoder(
        profile=profile,
        firmware_version="contract-test",
        mcu_id="e661410403724132",
        capabilities=0,
    )
    mcu_runtime = RuntimeCoordinator(
        profile=profile,
        state_machine=state_machine,
        hardware=hardware,
        telemetry=telemetry,
        session_factory=lambda: SESSION_INT,
    )

    if not mcu_runtime.perform_self_test(0):
        raise AssertionError("real Servo 2040 runtime failed self-test")

    clock = FakeClock(interrupt_at_s=interrupt_at_s)
    transport = FirmwareLoopbackTransport(
        mcu_runtime,
        clock,
    )

    hx1_config = load_hx1_client_config(HX1_CONFIG)
    client = HX1ClientCore(hx1_config)
    link = HX1Link(client, transport)

    pi_runtime = RobotRuntime(
        build_motion_pipeline(),
        link,
        hx1_config,
        clock=clock,
    )

    return (
        pi_runtime,
        mcu_runtime,
        state_machine,
        hardware,
        transport,
    )


class RobotRuntimeContractTests(unittest.TestCase):
    def test_stationary_lifecycle_round_trip_uses_real_mcu_runtime(self):
        (
            pi_runtime,
            _,
            state_machine,
            hardware,
            transport,
        ) = build_runtime_stack()

        result = pi_runtime.run_stationary(0.12)

        self.assertEqual(state_machine.state, MCUState.DISARMED)
        self.assertFalse(hardware.enabled)
        self.assertFalse(transport.is_open)

        self.assertEqual(result.initial_status.state, MCUState.DISARMED)
        self.assertEqual(result.initial_status.fault, "NONE")
        self.assertEqual(result.final_status.state, MCUState.DISARMED)
        self.assertEqual(result.final_status.fault, "NONE")

        self.assertGreaterEqual(result.target_frames_sent, 4)
        self.assertEqual(
            transport.commands.count("TARGET"),
            result.target_frames_sent,
        )
        self.assertEqual(
            transport.commands.count("HEARTBEAT"),
            result.heartbeat_frames_sent,
        )
        self.assertNotIn("ESTOP", transport.commands)

        self.assertEqual(
            transport.commands[:7],
            [
                "HELLO",
                "GET_STATUS",
                "STAGE",
                "HEARTBEAT",
                "ARM",
                "HEARTBEAT",
                "START",
            ],
        )

        self.assertEqual(
            transport.commands[-3:],
            [
                "STOP",
                "DISARM",
                "GET_STATUS",
            ],
        )

        self.assertEqual(len(hardware.enable_targets), 1)
        self.assertGreaterEqual(len(hardware.applied_targets), 4)

        enabled_target = hardware.enable_targets[0]
        for applied_target in hardware.applied_targets:
            self.assertEqual(applied_target, enabled_target)

    def test_keyboard_interrupt_after_arm_requests_estop(self):
        (
            pi_runtime,
            _,
            state_machine,
            hardware,
            transport,
        ) = build_runtime_stack(interrupt_at_s=0.055)

        with self.assertRaises(KeyboardInterrupt):
            pi_runtime.run_stationary(1.0)

        self.assertEqual(state_machine.state, MCUState.ESTOP)
        self.assertFalse(hardware.enabled)
        self.assertFalse(transport.is_open)
        self.assertIn("ESTOP", transport.commands)
        self.assertEqual(transport.commands[-1], "ESTOP")


if __name__ == "__main__":
    unittest.main()
