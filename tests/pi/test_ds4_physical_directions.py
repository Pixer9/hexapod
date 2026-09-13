"""Physical direction contract for the verified DS4 mapping."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
DS4_CONFIG = REPO_ROOT / "config" / "inputs" / "ds4.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
sys.path.insert(0, str(SRC))

from hexapod.control import load_motion_limits  # noqa: E402
from hexapod.inputs import DS4Mapper, load_ds4_config  # noqa: E402


class VerifiedDS4DirectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_ds4_config(DS4_CONFIG)
        cls.limits = load_motion_limits(MOTION_CONFIG)
        cls.mapper = DS4Mapper(cls.config, cls.limits)

    def test_verified_axis_bindings_and_inversions(self):
        self.assertEqual(self.config.axes["vx"].code, "ABS_Y")
        self.assertTrue(self.config.axes["vx"].invert)

        self.assertEqual(self.config.axes["vy"].code, "ABS_X")
        self.assertTrue(self.config.axes["vy"].invert)

        self.assertEqual(self.config.axes["yaw_rate"].code, "ABS_RX")
        self.assertTrue(self.config.axes["yaw_rate"].invert)

    def test_physical_forward_maps_to_positive_vx(self):
        command = self.mapper.command_from_axes({"ABS_Y": -1.0})
        self.assertEqual(command.vx_mm_s, 80.0)

    def test_physical_backward_maps_to_negative_vx(self):
        command = self.mapper.command_from_axes({"ABS_Y": 1.0})
        self.assertEqual(command.vx_mm_s, -80.0)

    def test_physical_left_maps_to_positive_vy(self):
        command = self.mapper.command_from_axes({"ABS_X": -1.0})
        self.assertEqual(command.vy_mm_s, 60.0)

    def test_physical_right_maps_to_negative_vy(self):
        command = self.mapper.command_from_axes({"ABS_X": 1.0})
        self.assertEqual(command.vy_mm_s, -60.0)

    def test_right_stick_left_maps_to_positive_ccw_yaw(self):
        command = self.mapper.command_from_axes({"ABS_RX": -1.0})
        self.assertEqual(command.yaw_rate_deg_s, 90.0)

    def test_right_stick_right_maps_to_negative_cw_yaw(self):
        command = self.mapper.command_from_axes({"ABS_RX": 1.0})
        self.assertEqual(command.yaw_rate_deg_s, -90.0)


if __name__ == "__main__":
    unittest.main()
