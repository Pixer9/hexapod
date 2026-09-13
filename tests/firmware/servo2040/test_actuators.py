"""Host-side tests for pure Servo 2040 actuator validation/mapping."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.actuators import (  # noqa: E402
    PositionLimitError,
    RateLimitError,
    TargetShapeError,
    UnqualifiedRateError,
    logical_to_channel_vector,
    logical_to_servo_vector,
    required_rate_cd_s,
    validate_position_target,
    validate_rate_transition,
)
from hexapod_mcu.profile import load_profile, parse_profile_bytes  # noqa: E402


def safe_target():
    # Coxa 0 deg, femur 0 deg, tibia 10 deg for each leg.
    # 10 deg is the lower hard logical tibia limit in the migrated profile.
    return (
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
        0, 0, 1000,
    )


def qualified_profile(rate_cd_s=25000):
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))

    for joint in data["joints"]:
        joint["max_rate_cd_s"] = rate_cd_s

    return parse_profile_bytes(
        json.dumps(data, separators=(",", ":")).encode("utf-8")
    )


class PositionValidationTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile(PROFILE_PATH)

    def test_safe_vector_is_inside_current_hard_limits(self):
        target = safe_target()
        self.assertEqual(
            validate_position_target(self.profile, target),
            target,
        )

    def test_wrong_target_length_is_rejected(self):
        with self.assertRaises(TargetShapeError):
            validate_position_target(self.profile, safe_target()[:-1])

    def test_boolean_is_not_a_joint_angle(self):
        target = list(safe_target())
        target[0] = True

        with self.assertRaises(TargetShapeError):
            validate_position_target(self.profile, target)

    def test_lower_endpoint_is_inclusive(self):
        target = list(safe_target())
        target[0] = self.profile.joint(0).logical_min_cd
        validate_position_target(self.profile, target)

    def test_upper_endpoint_is_inclusive(self):
        target = list(safe_target())
        target[0] = self.profile.joint(0).logical_max_cd
        validate_position_target(self.profile, target)

    def test_below_lower_endpoint_is_rejected(self):
        target = list(safe_target())
        joint = self.profile.joint(0)
        target[0] = joint.logical_min_cd - 1

        with self.assertRaises(PositionLimitError) as ctx:
            validate_position_target(self.profile, target)

        self.assertEqual(ctx.exception.index, 0)
        self.assertEqual(ctx.exception.name, "rf_coxa")

    def test_above_upper_endpoint_is_rejected(self):
        target = list(safe_target())
        joint = self.profile.joint(17)
        target[17] = joint.logical_max_cd + 1

        with self.assertRaises(PositionLimitError) as ctx:
            validate_position_target(self.profile, target)

        self.assertEqual(ctx.exception.index, 17)
        self.assertEqual(ctx.exception.name, "lb_tibia")


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile(PROFILE_PATH)

    def test_direction_is_applied_in_servo_space(self):
        target = list(safe_target())
        target[0] = 1000   # RF coxa direction -1
        target[1] = 1000   # RF femur direction +1
        target[2] = 2000   # RF tibia direction +1

        servo = logical_to_servo_vector(self.profile, target)

        self.assertEqual(servo[0], -1400)
        self.assertEqual(servo[1], 4700)
        self.assertEqual(servo[2], -900)

    def test_channel_vector_uses_physical_channel_order(self):
        servo = logical_to_servo_vector(self.profile, safe_target())
        channels = logical_to_channel_vector(self.profile, safe_target())

        for joint in self.profile.joints:
            self.assertEqual(
                channels[joint.channel],
                servo[joint.index],
            )

    def test_known_safe_channel_vector(self):
        channels = logical_to_channel_vector(self.profile, safe_target())

        self.assertEqual(
            channels,
            (
                600, 4500, -3200,
                800, 4000, -2500,
                700, 4000, -2500,
                600, 3100, -2900,
                -400, 3700, -1900,
                900, 4800, -3000,
            ),
        )


class RateTests(unittest.TestCase):
    def test_required_rate_uses_actual_dt_and_rounds_up(self):
        self.assertEqual(required_rate_cd_s(0, 500, 20), 25000)
        self.assertEqual(required_rate_cd_s(0, 500, 25), 20000)
        self.assertEqual(required_rate_cd_s(0, 1, 3), 334)

    def test_current_profile_refuses_rate_validation(self):
        profile = load_profile(PROFILE_PATH)

        with self.assertRaises(UnqualifiedRateError):
            validate_rate_transition(
                profile,
                safe_target(),
                safe_target(),
                20,
            )

    def test_rate_at_exact_limit_is_accepted(self):
        profile = qualified_profile(25000)
        requested = list(safe_target())
        requested[0] = 500  # 500 cd / 20 ms = 25,000 cd/s

        self.assertEqual(
            validate_rate_transition(
                profile,
                safe_target(),
                requested,
                20,
            ),
            tuple(requested),
        )

    def test_rate_one_cd_over_limit_is_rejected(self):
        profile = qualified_profile(25000)
        requested = list(safe_target())
        requested[0] = 501

        with self.assertRaises(RateLimitError) as ctx:
            validate_rate_transition(
                profile,
                safe_target(),
                requested,
                20,
            )

        self.assertEqual(ctx.exception.index, 0)
        self.assertEqual(ctx.exception.required_rate_cd_s, 25050)

    def test_longer_actual_dt_can_make_same_delta_valid(self):
        profile = qualified_profile(25000)
        requested = list(safe_target())
        requested[0] = 600

        with self.assertRaises(RateLimitError):
            validate_rate_transition(
                profile,
                safe_target(),
                requested,
                20,
            )

        self.assertEqual(
            validate_rate_transition(
                profile,
                safe_target(),
                requested,
                25,
            ),
            tuple(requested),
        )

    def test_position_violation_wins_before_rate_check(self):
        profile = qualified_profile(25000)
        requested = list(safe_target())
        requested[0] = profile.joint(0).logical_max_cd + 1

        with self.assertRaises(PositionLimitError):
            validate_rate_transition(
                profile,
                safe_target(),
                requested,
                20,
            )

    def test_invalid_dt_is_rejected(self):
        profile = qualified_profile(25000)

        with self.assertRaises(ValueError):
            validate_rate_transition(
                profile,
                safe_target(),
                safe_target(),
                0,
            )


if __name__ == "__main__":
    unittest.main()
