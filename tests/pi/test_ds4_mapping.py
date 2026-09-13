"""Tests for pure DS4 configuration, shaping, and command publication."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
DS4_CONFIG = REPO_ROOT / "config" / "inputs" / "ds4.json"
MOTION_CONFIG = REPO_ROOT / "config" / "control" / "motion.json"
sys.path.insert(0, str(SRC))

from hexapod.control import MotionSource, load_motion_limits  # noqa: E402
from hexapod.inputs import (  # noqa: E402
    DS4ActionType,
    DS4InputState,
    DS4Mapper,
    load_ds4_config,
    normalize_axis,
)


class AxisNormalizationTests(unittest.TestCase):
    def test_axis_endpoints_and_midpoint(self):
        self.assertEqual(normalize_axis(0, 0, 255), -1.0)
        self.assertEqual(normalize_axis(255, 0, 255), 1.0)
        self.assertAlmostEqual(
            normalize_axis(127.5, 0, 255),
            0.0,
            places=12,
        )

    def test_bad_range_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_axis(0, 10, 10)


class DS4MappingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_ds4_config(DS4_CONFIG)
        cls.limits = load_motion_limits(MOTION_CONFIG)
        cls.mapper = DS4Mapper(cls.config, cls.limits)

    def test_v4_axis_signs_are_preserved_in_config(self):
        self.assertEqual(self.config.axes["vx"].code, "ABS_Y")
        self.assertTrue(self.config.axes["vx"].invert)
        self.assertEqual(self.config.axes["vy"].code, "ABS_X")
        self.assertTrue(self.config.axes["vy"].invert)
        self.assertEqual(self.config.axes["yaw_rate"].code, "ABS_RX")
        self.assertTrue(self.config.axes["yaw_rate"].invert)

    def test_full_axes_map_to_motion_limits(self):
        command = self.mapper.command_from_axes(
            {
                "ABS_Y": -1.0,  # physical forward
                "ABS_X": -1.0,  # physical left
                "ABS_RX": -1.0,  # physical left / CCW
            }
        )

        self.assertEqual(command.vx_mm_s, 80.0)
        self.assertEqual(command.vy_mm_s, 60.0)
        self.assertEqual(command.yaw_rate_deg_s, 90.0)

    def test_deadzone_zeroes_small_input(self):
        command = self.mapper.command_from_axes(
            {
                "ABS_Y": 0.10,
                "ABS_X": -0.09,
                "ABS_RX": 0.0,
            }
        )

        self.assertTrue(command.is_zero)

    def test_expo_matches_v4_formula_after_deadzone_rescale(self):
        # Physical forward is negative ABS_Y on this controller.
        # x=.55 after inversion, with dz=.10, becomes .5 before expo.
        # (1-.35)*.5 + .35*(.5**3) = .36875.
        command = self.mapper.command_from_axes({"ABS_Y": -0.55})

        self.assertAlmostEqual(
            command.vx_mm_s,
            80.0 * 0.36875,
            places=10,
        )


class DS4InputStateTests(unittest.TestCase):
    def setUp(self):
        config = load_ds4_config(DS4_CONFIG)
        limits = load_motion_limits(MOTION_CONFIG)
        self.state = DS4InputState(config, limits)

    def test_start_is_disabled_by_default(self):
        self.state.connect(
            device_path="/dev/input/event9",
            device_name="Wireless Controller",
            now_s=1.0,
        )
        self.state.update_axis("ABS_Y", -1.0, now_s=1.2)

        snapshot = self.state.snapshot()

        self.assertFalse(snapshot.motion_enabled)
        self.assertTrue(snapshot.command.is_zero)

    def test_start_button_enables_motion_and_publishes_action(self):
        self.state.connect(
            device_path="/dev/input/event9",
            device_name="Wireless Controller",
            now_s=1.0,
        )
        self.state.button_pressed("BTN_START", now_s=2.0)

        snapshot = self.state.snapshot()
        actions = self.state.drain_actions()

        self.assertTrue(snapshot.motion_enabled)
        self.assertEqual(actions[0].action, DS4ActionType.MOTION_ENABLED)

    def test_held_stick_remains_fresh_without_new_axis_events(self):
        self.state.connect(
            device_path="/dev/input/event9",
            device_name="Wireless Controller",
            now_s=1.0,
        )
        self.state.button_pressed("BTN_START", now_s=1.1)
        self.state.update_axis("ABS_Y", -1.0, now_s=1.2)

        first = self.state.command_sample(now_s=2.0)
        second = self.state.command_sample(now_s=10.0)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        self.assertEqual(first.source, MotionSource.DS4)
        self.assertEqual(first.command.vx_mm_s, 80.0)
        self.assertEqual(second.command.vx_mm_s, 80.0)
        self.assertEqual(second.received_at_s, 10.0)
        self.assertAlmostEqual(second.expires_at_s, 10.25)

    def test_disconnected_controller_does_not_publish_command(self):
        self.state.connect(
            device_path="/dev/input/event9",
            device_name="Wireless Controller",
            now_s=1.0,
        )
        self.state.disconnect(now_s=2.0)

        self.assertIsNone(self.state.command_sample(now_s=3.0))

        actions = self.state.drain_actions()
        self.assertEqual(
            tuple(action.action for action in actions),
            (
                DS4ActionType.DISCONNECTED,
                DS4ActionType.ESTOP,
            ),
        )

    def test_ps_button_emits_estop_request(self):
        self.state.button_pressed("BTN_MODE", now_s=3.0)

        actions = self.state.drain_actions()

        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0].action, DS4ActionType.ESTOP)

    def test_select_emits_clear_estop_request_only(self):
        self.state.button_pressed("BTN_SELECT", now_s=3.0)

        actions = self.state.drain_actions()

        self.assertEqual(len(actions), 1)
        self.assertEqual(
            actions[0].action,
            DS4ActionType.CLEAR_ESTOP_REQUEST,
        )


if __name__ == "__main__":
    unittest.main()
