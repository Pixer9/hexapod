"""Tests for DS4 evdev discovery without requiring real Linux input devices."""

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
from hexapod.inputs import (  # noqa: E402
    DS4ActionType,
    DS4EvdevError,
    DS4EvdevReader,
    discover_ds4_device,
    load_ds4_config,
)


class FakeECodes:
    EV_ABS = 3
    EV_KEY = 1
    ABS_X = 0
    ABS_Y = 1
    ABS_RX = 3
    BTN_START = 315
    BTN_MODE = 316
    BTN_SELECT = 314


class FakeDevice:
    def __init__(self, path, name, abs_codes):
        self.path = path
        self.name = name
        self._abs_codes = tuple(abs_codes)
        self.closed = False

    def capabilities(self, absinfo=False):
        return {FakeECodes.EV_ABS: self._abs_codes}

    def close(self):
        self.closed = True


class FakeEvdev:
    ecodes = FakeECodes

    def __init__(self, devices):
        self.devices = devices

    def list_devices(self):
        return list(self.devices)

    def InputDevice(self, path):
        return self.devices[path]


class FakeReadLoopDevice:
    def __init__(self, error=None):
        self.error = error
        self.closed = False

    def read_loop(self):
        if self.error is not None:
            raise self.error
        return iter(())

    def close(self):
        self.closed = True


class DiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.config = load_ds4_config(DS4_CONFIG)

    def test_discovers_controller_node_with_required_axes(self):
        evdev = FakeEvdev(
            {
                "/dev/input/event2": FakeDevice(
                    "/dev/input/event2",
                    "Wireless Controller Motion Sensors",
                    (),
                ),
                "/dev/input/event7": FakeDevice(
                    "/dev/input/event7",
                    "Wireless Controller",
                    (
                        FakeECodes.ABS_X,
                        FakeECodes.ABS_Y,
                        FakeECodes.ABS_RX,
                    ),
                ),
            }
        )

        info = discover_ds4_device(
            self.config,
            evdev_module=evdev,
        )

        self.assertEqual(info.path, "/dev/input/event7")
        self.assertEqual(info.name, "Wireless Controller")

    def test_wrong_capabilities_are_rejected_even_when_name_matches(self):
        evdev = FakeEvdev(
            {
                "/dev/input/event2": FakeDevice(
                    "/dev/input/event2",
                    "Wireless Controller",
                    (FakeECodes.ABS_X,),
                )
            }
        )

        with self.assertRaises(DS4EvdevError):
            discover_ds4_device(
                self.config,
                evdev_module=evdev,
            )

    def test_stable_path_sorting_selects_first_matching_node(self):
        evdev = FakeEvdev(
            {
                "/dev/input/event9": FakeDevice(
                    "/dev/input/event9",
                    "Wireless Controller",
                    (
                        FakeECodes.ABS_X,
                        FakeECodes.ABS_Y,
                        FakeECodes.ABS_RX,
                    ),
                ),
                "/dev/input/event4": FakeDevice(
                    "/dev/input/event4",
                    "Wireless Controller",
                    (
                        FakeECodes.ABS_X,
                        FakeECodes.ABS_Y,
                        FakeECodes.ABS_RX,
                    ),
                ),
            }
        )

        info = discover_ds4_device(
            self.config,
            evdev_module=evdev,
        )

        self.assertEqual(info.path, "/dev/input/event4")


class ReaderTerminationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_ds4_config(DS4_CONFIG)
        self.limits = load_motion_limits(MOTION_CONFIG)

    def make_reader(self, device):
        reader = DS4EvdevReader(
            self.config,
            self.limits,
            clock=lambda: 10.0,
            evdev_module=FakeEvdev({}),
        )
        reader._device = device

        with reader._lock:
            reader._state.connect(
                device_path="/dev/input/fake",
                device_name="Wireless Controller",
                now_s=1.0,
            )

        return reader

    def test_clean_unexpected_reader_exit_disconnects_and_estops(self):
        reader = self.make_reader(FakeReadLoopDevice())

        reader._run()

        snapshot = reader.snapshot()
        self.assertFalse(snapshot.connected)
        self.assertIsNone(reader.command_sample(now_s=10.0))

        actions = reader.drain_actions()
        self.assertEqual(
            tuple(action.action for action in actions),
            (
                DS4ActionType.DISCONNECTED,
                DS4ActionType.ESTOP,
            ),
        )

    def test_reader_oserror_disconnects_and_estops(self):
        reader = self.make_reader(
            FakeReadLoopDevice(
                error=OSError("device removed"),
            )
        )

        reader._run()

        self.assertFalse(reader.snapshot().connected)
        self.assertIsNone(reader.command_sample(now_s=10.0))

        actions = reader.drain_actions()
        self.assertEqual(
            tuple(action.action for action in actions),
            (
                DS4ActionType.DISCONNECTED,
                DS4ActionType.ESTOP,
            ),
        )

    def test_intentional_stop_is_quiet(self):
        device = FakeReadLoopDevice()
        reader = self.make_reader(device)

        reader.stop()

        self.assertTrue(device.closed)
        self.assertFalse(reader.snapshot().connected)
        self.assertIsNone(reader.command_sample(now_s=10.0))
        self.assertEqual(reader.drain_actions(), ())


if __name__ == "__main__":
    unittest.main()
