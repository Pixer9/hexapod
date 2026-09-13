"""Tests for DS4 evdev discovery without requiring real Linux input devices."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
DS4_CONFIG = REPO_ROOT / "config" / "inputs" / "ds4.json"
sys.path.insert(0, str(SRC))

from hexapod.inputs import (  # noqa: E402
    DS4EvdevError,
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


if __name__ == "__main__":
    unittest.main()
