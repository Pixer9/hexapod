"""Runtime regressions discovered while composing main.py."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.protocol import encode_frame, parse_frame  # noqa: E402
from hexapod_mcu.runtime import RuntimeCoordinator  # noqa: E402
from hexapod_mcu.state_machine import Fault, RuntimeStateMachine  # noqa: E402
from hexapod_mcu.telemetry import TelemetryEncoder  # noqa: E402


class InvalidProfile:
    profile_id = "invalid-profile"
    profile_revision = 0
    profile_hash = "0" * 64
    joint_count = 18
    joints = ()

    def require_arm_qualified(self):
        raise RuntimeError("invalid profile")


class FakeHardware:
    enabled = False

    def force_disabled(self):
        pass

    def require_profile_compatible(self, profile):
        return True


class RuntimeStageFaultRegressionTests(unittest.TestCase):
    def test_stage_while_faulted_rejects_before_profile_access(self):
        profile = InvalidProfile()
        sm = RuntimeStateMachine()
        sm.begin_self_test()
        sm.complete_self_test(False, now_ms=0, fault_code=Fault.PROFILE)
        sm.establish_session("A1B2C3D4", False)

        tx = TelemetryEncoder(
            profile,
            "0.1.0-rc1",
            "mcu",
        )
        runtime = RuntimeCoordinator(
            profile,
            sm,
            FakeHardware(),
            tx,
            lambda: 1,
        )

        target = (0,) * 18
        frame = encode_frame(
            10,
            "STAGE",
            "A1B2C3D4",
            *target
        )
        responses = runtime.handle_frame(frame, 100)

        self.assertEqual(len(responses), 1)
        _, message_type, fields = parse_frame(responses[0])
        self.assertEqual(message_type, "NACK")
        self.assertEqual(fields[-1], "ERR_FAULT_ACTIVE")


if __name__ == "__main__":
    unittest.main()
