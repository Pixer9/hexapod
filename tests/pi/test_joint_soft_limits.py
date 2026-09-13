"""Tests for Pi-side canonical joint planning soft limits."""

from __future__ import annotations

import itertools
import json
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"

ROBOT_CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
SOFT_LIMIT_CONFIG = (
    REPO_ROOT / "config" / "robots" / "standard-joint-soft-limits.json"
)
GAIT_CONFIG = REPO_ROOT / "config" / "locomotion" / "tripod.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
MCU_PROFILE = (
    REPO_ROOT
    / "firmware"
    / "servo2040"
    / "src"
    / "config"
    / "actuator-profile.json"
)

sys.path.insert(0, str(SRC))

from hexapod.control import MotionCommand, load_motion_limits  # noqa: E402
from hexapod.kinematics import RobotKinematics  # noqa: E402
from hexapod.locomotion import TripodGait, load_tripod_gait_config  # noqa: E402
from hexapod.model import load_robot_geometry  # noqa: E402
from hexapod.trajectory import (  # noqa: E402
    CANONICAL_JOINT_NAMES,
    JointSoftLimitConfigError,
    JointSoftLimitError,
    JointVectorError,
    load_joint_soft_limit_profile,
    parse_joint_soft_limit_profile,
)


class JointSoftLimitConfigTests(unittest.TestCase):
    def test_loads_standard_profile(self):
        profile = load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG)

        self.assertEqual(profile.schema_version, 1)
        self.assertEqual(
            profile.profile_id,
            "hexapod-standard-soft-joints-v1",
        )
        self.assertEqual(profile.robot_id, "hexapod-standard-v1")
        self.assertEqual(len(profile.joints), 18)
        self.assertEqual(
            tuple(item.name for item in profile.joints),
            CANONICAL_JOINT_NAMES,
        )

    def test_rejects_wrong_joint_name_or_order(self):
        data = json.loads(SOFT_LIMIT_CONFIG.read_text(encoding="utf-8"))
        data["joints"][3]["name"] = "wrong_name"

        with self.assertRaises(JointSoftLimitConfigError):
            parse_joint_soft_limit_profile(data)

    def test_rejects_inverted_range(self):
        data = json.loads(SOFT_LIMIT_CONFIG.read_text(encoding="utf-8"))
        data["joints"][0]["min_deg"] = 10.0
        data["joints"][0]["max_deg"] = -10.0

        with self.assertRaises(JointSoftLimitConfigError):
            parse_joint_soft_limit_profile(data)


class JointSoftLimitValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG)

    def valid_vector(self):
        values = []
        for item in self.profile.joints:
            if item.name.endswith("_tibia"):
                values.append(90.0)
            else:
                values.append(0.0)
        return values

    def test_accepts_values_on_boundaries(self):
        vector = tuple(
            item.min_deg if item.index % 2 == 0 else item.max_deg
            for item in self.profile.joints
        )

        self.assertEqual(self.profile.validate(vector), vector)

    def test_rejects_vector_atomically(self):
        vector = self.valid_vector()
        vector[1] = 39.0
        vector[2] = 121.0

        with self.assertRaises(JointSoftLimitError) as context:
            self.profile.validate(vector)

        violations = context.exception.violations
        self.assertEqual(len(violations), 2)
        self.assertEqual(violations[0].name, "rf_femur")
        self.assertEqual(violations[1].name, "rf_tibia")

    def test_never_clamps_rejected_values(self):
        vector = self.valid_vector()
        vector[1] = 50.0

        with self.assertRaises(JointSoftLimitError):
            self.profile.validate(vector)

        self.assertEqual(vector[1], 50.0)

    def test_rejects_wrong_vector_length(self):
        with self.assertRaises(JointVectorError):
            self.profile.validate((0.0,) * 17)

    def test_rejects_nonfinite_vector_value(self):
        vector = self.valid_vector()
        vector[7] = math.nan

        with self.assertRaises(JointVectorError):
            self.profile.validate(vector)


class PiMcuEnvelopeCompatibilityTests(unittest.TestCase):
    def test_pi_soft_limits_fit_inside_current_mcu_hard_envelope(self):
        soft = load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG)
        mcu = json.loads(MCU_PROFILE.read_text(encoding="utf-8"))

        self.assertEqual(mcu["joint_count"], 18)
        self.assertEqual(len(mcu["joints"]), 18)

        for soft_joint, hard_joint in zip(soft.joints, mcu["joints"]):
            self.assertEqual(soft_joint.index, hard_joint["index"])
            self.assertEqual(soft_joint.name, hard_joint["name"])

            direction = hard_joint["direction"]
            offset = hard_joint["offset_cd"] / 100.0
            servo_min = hard_joint["servo_min_cd"] / 100.0
            servo_max = hard_joint["servo_max_cd"] / 100.0

            if direction == 1:
                hard_min = servo_min - offset
                hard_max = servo_max - offset
            elif direction == -1:
                hard_min = offset - servo_max
                hard_max = offset - servo_min
            else:
                self.fail(f"invalid MCU direction for {soft_joint.name}")

            self.assertGreaterEqual(
                soft_joint.min_deg,
                hard_min,
                soft_joint.name,
            )
            self.assertLessEqual(
                soft_joint.max_deg,
                hard_max,
                soft_joint.name,
            )


class ConfiguredGaitEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(ROBOT_CONFIG)
        cls.gait_config = load_tripod_gait_config(GAIT_CONFIG)
        cls.gait = TripodGait(cls.geometry, cls.gait_config)
        cls.kinematics = RobotKinematics(cls.geometry)
        cls.soft = load_joint_soft_limit_profile(SOFT_LIMIT_CONFIG)
        cls.motion_limits = load_motion_limits(MOTION_CONFIG)

    def test_safe_baseline_step_height_is_30_mm(self):
        self.assertEqual(self.gait_config.step_height_mm, 30.0)

    def test_reference_stance_fits_soft_limits(self):
        solution = self.kinematics.solve(
            self.gait.stance_anchors_body_mm
        )
        self.soft.validate(solution.logical_vector_deg)

    def test_configured_command_phase_sweep_stays_inside_soft_limits(self):
        vx_values = (
            -self.motion_limits.max_vx_mm_s,
            -0.5 * self.motion_limits.max_vx_mm_s,
            0.0,
            0.5 * self.motion_limits.max_vx_mm_s,
            self.motion_limits.max_vx_mm_s,
        )
        vy_values = (
            -self.motion_limits.max_vy_mm_s,
            -0.5 * self.motion_limits.max_vy_mm_s,
            0.0,
            0.5 * self.motion_limits.max_vy_mm_s,
            self.motion_limits.max_vy_mm_s,
        )
        yaw_values = (
            -self.motion_limits.max_yaw_rate_deg_s,
            -0.5 * self.motion_limits.max_yaw_rate_deg_s,
            0.0,
            0.5 * self.motion_limits.max_yaw_rate_deg_s,
            self.motion_limits.max_yaw_rate_deg_s,
        )

        observed_min = [math.inf] * 18
        observed_max = [-math.inf] * 18

        for vx, vy, yaw in itertools.product(
            vx_values,
            vy_values,
            yaw_values,
        ):
            command = MotionCommand(
                vx_mm_s=vx,
                vy_mm_s=vy,
                yaw_rate_deg_s=yaw,
            )

            for phase_index in range(101):
                phase = phase_index / 100.0
                frame = self.gait.sample(
                    command,
                    phase,
                    blend=1.0,
                )
                solution = self.kinematics.solve(
                    frame.foot_targets_body_mm
                )
                vector = self.soft.validate(
                    solution.logical_vector_deg
                )

                for index, value in enumerate(vector):
                    observed_min[index] = min(
                        observed_min[index],
                        value,
                    )
                    observed_max[index] = max(
                        observed_max[index],
                        value,
                    )

        # Keep a useful regression assertion on the currently tightest
        # quantity: full configured motion should remain comfortably below the
        # 38-degree femur planning maximum.
        femur_max = max(observed_max[1::3])
        self.assertLess(femur_max, 36.0)


if __name__ == "__main__":
    unittest.main()
