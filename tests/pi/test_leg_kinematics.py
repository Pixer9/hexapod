"""Tests for canonical leg IK/FK."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
sys.path.insert(0, str(SRC))

from hexapod.kinematics import (  # noqa: E402
    JointAngles,
    UnreachableTargetError,
    leg_fk,
    leg_ik,
)
from hexapod.model import load_robot_geometry  # noqa: E402


class LegKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(CONFIG)
        cls.links = cls.geometry.links

    def test_v4_reference_point_produces_expected_canonical_angles(self):
        angles = leg_ik(200.0, 0.0, -115.0, self.links)

        self.assertAlmostEqual(angles.coxa_deg, 0.0, places=9)
        self.assertAlmostEqual(angles.femur_deg, 30.183197053818166, places=9)
        self.assertAlmostEqual(angles.tibia_deg, 101.46506994927603, places=9)

    def test_reference_ik_fk_round_trip(self):
        target = self.geometry.neutral_foot_leg_mm
        angles = leg_ik(*target, self.links)
        recovered = leg_fk(angles, self.links)

        for actual, expected in zip(recovered, target):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_multiple_reachable_targets_round_trip(self):
        targets = (
            (200.0, 0.0, -115.0),
            (190.0, 20.0, -130.0),
            (210.0, -15.0, -100.0),
            (175.0, 30.0, -145.0),
        )

        for target in targets:
            with self.subTest(target=target):
                angles = leg_ik(*target, self.links)
                recovered = leg_fk(angles, self.links)

                for actual, expected in zip(recovered, target):
                    self.assertAlmostEqual(actual, expected, places=8)

    def test_fk_known_straight_leg(self):
        point = leg_fk(
            JointAngles(
                coxa_deg=0.0,
                femur_deg=0.0,
                tibia_deg=0.0,
            ),
            self.links,
        )

        self.assertAlmostEqual(point[0], 340.0, places=9)
        self.assertAlmostEqual(point[1], 0.0, places=9)
        self.assertAlmostEqual(point[2], 0.0, places=9)

    def test_unreachable_far_target_is_rejected(self):
        with self.assertRaises(UnreachableTargetError):
            leg_ik(1000.0, 0.0, 0.0, self.links)

    def test_unreachable_inner_target_is_rejected(self):
        # At x == coxa, the femur/tibia endpoint distance is zero, which is
        # inside the annular workspace hole for unequal femur/tibia lengths.
        with self.assertRaises(UnreachableTargetError):
            leg_ik(self.links.coxa_mm, 0.0, 0.0, self.links)


if __name__ == "__main__":
    unittest.main()
