"""Tests for the configurable canonical robot geometry."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
CONFIG = REPO_ROOT / "config" / "robots" / "standard.json"
sys.path.insert(0, str(SRC))

from hexapod.model import (  # noqa: E402
    CANONICAL_LEG_ORDER,
    GeometryConfigError,
    body_to_leg,
    build_leg_transforms,
    leg_to_body,
    load_robot_geometry,
)


class StandardGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_robot_geometry(CONFIG)
        cls.transforms = build_leg_transforms(cls.geometry)

    def test_v4_link_lengths_are_preserved(self):
        self.assertEqual(self.geometry.links.coxa_mm, 41.0)
        self.assertEqual(self.geometry.links.femur_mm, 116.0)
        self.assertEqual(self.geometry.links.tibia_mm, 183.0)

    def test_canonical_leg_order_is_preserved(self):
        self.assertEqual(self.geometry.leg_order, CANONICAL_LEG_ORDER)

    def test_v4_mount_radius_and_yaws_are_preserved(self):
        expected_yaws = {
            "RF": -45.0,
            "RM": -90.0,
            "RB": -135.0,
            "LF": 45.0,
            "LM": 90.0,
            "LB": 135.0,
        }

        for leg_name in CANONICAL_LEG_ORDER:
            mount = self.geometry.mount(leg_name)
            x, y, _ = mount.position_body_mm
            self.assertAlmostEqual(math.hypot(x, y), 105.0, places=9)
            self.assertEqual(mount.yaw_deg, expected_yaws[leg_name])

    def test_body_leg_transforms_round_trip(self):
        reference_leg = self.geometry.neutral_foot_leg_mm

        for leg_name in CANONICAL_LEG_ORDER:
            transform = self.transforms[leg_name]
            body = leg_to_body(reference_leg, transform)
            recovered = body_to_leg(body, transform)

            for actual, expected in zip(recovered, reference_leg):
                self.assertAlmostEqual(actual, expected, places=9)

    def test_reference_neutral_foot_matches_v4_geometry_default(self):
        self.assertEqual(
            self.geometry.neutral_foot_leg_mm,
            (200.0, 0.0, -115.0),
        )


class LoaderValidationTests(unittest.TestCase):
    def test_wrong_leg_order_is_rejected(self):
        data = json.loads(CONFIG.read_text(encoding="utf-8"))
        data["leg_order"][0], data["leg_order"][1] = (
            data["leg_order"][1],
            data["leg_order"][0],
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.json"
            path.write_text(json.dumps(data), encoding="utf-8")

            with self.assertRaises(GeometryConfigError):
                load_robot_geometry(path)

    def test_nonpositive_link_length_is_rejected(self):
        data = json.loads(CONFIG.read_text(encoding="utf-8"))
        data["links_mm"]["femur"] = 0

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.json"
            path.write_text(json.dumps(data), encoding="utf-8")

            with self.assertRaises(GeometryConfigError):
                load_robot_geometry(path)


if __name__ == "__main__":
    unittest.main()
