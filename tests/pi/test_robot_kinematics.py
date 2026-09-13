"""Tests for six-leg BODY-frame kinematics and canonical joint ordering."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
sys.path.insert(0, str(SRC))

from hexapod.kinematics import (  # noqa: E402
    FootTargetSetError,
    LegSolveError,
    LOGICAL_JOINT_COUNT,
    RobotKinematics,
    UnreachableTargetError,
    leg_ik,
)
from hexapod.model import (  # noqa: E402
    CANONICAL_LEG_ORDER,
    leg_to_body,
    load_robot_geometry,
)


class WholeBodyKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(CONFIG)
        cls.kinematics = RobotKinematics(cls.geometry)

    def test_reference_stance_contains_exact_canonical_legs(self):
        targets = self.kinematics.reference_stance_body()

        self.assertEqual(tuple(targets), CANONICAL_LEG_ORDER)

    def test_reference_stance_solves_same_local_angles_for_all_legs(self):
        solution = self.kinematics.solve(
            self.kinematics.reference_stance_body()
        )

        expected = leg_ik(
            *self.geometry.neutral_foot_leg_mm,
            self.geometry.links,
        )

        for leg_name in CANONICAL_LEG_ORDER:
            with self.subTest(leg=leg_name):
                actual = solution.leg(leg_name)
                self.assertAlmostEqual(actual.coxa_deg, expected.coxa_deg, places=9)
                self.assertAlmostEqual(
                    actual.femur_deg,
                    expected.femur_deg,
                    places=9,
                )
                self.assertAlmostEqual(
                    actual.tibia_deg,
                    expected.tibia_deg,
                    places=9,
                )

    def test_reference_solution_is_18_values_in_canonical_triplets(self):
        solution = self.kinematics.solve(
            self.kinematics.reference_stance_body()
        )
        expected = leg_ik(
            *self.geometry.neutral_foot_leg_mm,
            self.geometry.links,
        )
        expected_triplet = (
            expected.coxa_deg,
            expected.femur_deg,
            expected.tibia_deg,
        )

        self.assertEqual(
            len(solution.logical_vector_deg),
            LOGICAL_JOINT_COUNT,
        )

        for leg_index in range(6):
            actual_triplet = solution.logical_vector_deg[
                leg_index * 3 : leg_index * 3 + 3
            ]
            for actual, wanted in zip(actual_triplet, expected_triplet):
                self.assertAlmostEqual(actual, wanted, places=9)

    def test_logical_vector_follows_rf_rm_rb_lf_lm_lb_order(self):
        local_targets = {
            "RF": (200.0, 0.0, -115.0),
            "RM": (195.0, 5.0, -120.0),
            "RB": (190.0, 10.0, -125.0),
            "LF": (185.0, 15.0, -130.0),
            "LM": (180.0, 20.0, -135.0),
            "LB": (175.0, 25.0, -140.0),
        }

        body_targets = {}
        expected = []

        for leg_name in CANONICAL_LEG_ORDER:
            body_targets[leg_name] = leg_to_body(
                local_targets[leg_name],
                self.kinematics.transforms[leg_name],
            )
            angles = leg_ik(
                *local_targets[leg_name],
                self.geometry.links,
            )
            expected.extend(
                (
                    angles.coxa_deg,
                    angles.femur_deg,
                    angles.tibia_deg,
                )
            )

        solution = self.kinematics.solve(body_targets)

        for actual, wanted in zip(solution.logical_vector_deg, expected):
            self.assertAlmostEqual(actual, wanted, places=9)

    def test_missing_leg_is_rejected_before_solving(self):
        targets = dict(self.kinematics.reference_stance_body())
        targets.pop("LB")

        with self.assertRaises(FootTargetSetError):
            self.kinematics.solve(targets)

    def test_extra_leg_is_rejected_before_solving(self):
        targets = dict(self.kinematics.reference_stance_body())
        targets["EXTRA"] = (0.0, 0.0, 0.0)

        with self.assertRaises(FootTargetSetError):
            self.kinematics.solve(targets)

    def test_non_numeric_coordinate_is_rejected(self):
        targets = dict(self.kinematics.reference_stance_body())
        targets["RF"] = ("bad", 0.0, -115.0)

        with self.assertRaises(FootTargetSetError):
            self.kinematics.solve(targets)

    def test_unreachable_leg_reports_which_leg_failed(self):
        targets = dict(self.kinematics.reference_stance_body())

        targets["RF"] = leg_to_body(
            (1000.0, 0.0, 0.0),
            self.kinematics.transforms["RF"],
        )

        with self.assertRaises(LegSolveError) as caught:
            self.kinematics.solve(targets)

        self.assertEqual(caught.exception.leg_name, "RF")
        self.assertIsInstance(
            caught.exception.cause,
            UnreachableTargetError,
        )

    def test_solver_does_not_mutate_supplied_target_mapping(self):
        targets = {
            leg: tuple(point)
            for leg, point in self.kinematics.reference_stance_body().items()
        }
        before = dict(targets)

        self.kinematics.solve(targets)

        self.assertEqual(targets, before)


if __name__ == "__main__":
    unittest.main()
