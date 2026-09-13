"""Host-side tests for the thin Servo 2040 hardware boundary."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.hardware import (  # noqa: E402
    HardwareCompatibilityError,
    HardwareError,
    HardwareStateError,
    ServoOutputHardware,
    create_servo2040_hardware,
)


class FakeServoCluster:
    def __init__(self, count=18, min_deg=-90.0, max_deg=90.0):
        self._count = count
        self._min_deg = [float(min_deg)] * count
        self._max_deg = [float(max_deg)] * count
        self.calls = []
        self.shadow = [None] * count
        self.active = [None] * count
        self.fail_on_channel = None
        self.fail_disable = False

    def count(self):
        return self._count

    def min_value(self, channel):
        return self._min_deg[channel]

    def max_value(self, channel):
        return self._max_deg[channel]

    def value(self, channel, value, load=True):
        self.calls.append(("value", channel, float(value), bool(load)))
        if channel == self.fail_on_channel:
            raise RuntimeError("injected backend failure")
        self.shadow[channel] = float(value)
        if load:
            self.load()

    def load(self):
        self.calls.append(("load",))
        self.active = list(self.shadow)

    def disable_all(self, load=True):
        self.calls.append(("disable_all", bool(load)))
        if self.fail_disable:
            raise RuntimeError("injected disable failure")
        self.shadow = [None] * self._count
        if load:
            self.active = [None] * self._count


def load_profile_view():
    data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
    joints = tuple(SimpleNamespace(**item) for item in data["joints"])
    return SimpleNamespace(joint_count=data["joint_count"], joints=joints)


def clipped_profile_view():
    profile = load_profile_view()
    clipped = []
    for joint in profile.joints:
        values = dict(vars(joint))
        values["servo_min_cd"] = max(values["servo_min_cd"], -9000)
        values["servo_max_cd"] = min(values["servo_max_cd"], 9000)
        clipped.append(SimpleNamespace(**values))
    return SimpleNamespace(joint_count=profile.joint_count, joints=tuple(clipped))


def safe_channel_target():
    # Physical channel order for the known-valid logical pose used by the
    # actuator-core tests: coxa=0, femur=0, tibia=10 degrees.
    return (
        600, 4500, -3200,
        800, 4000, -2500,
        700, 4000, -2500,
        600, 3100, -2900,
        -400, 3700, -1900,
        900, 4800, -3000,
    )


class CompatibilityTests(unittest.TestCase):
    def test_wrong_cluster_count_is_rejected(self):
        with self.assertRaises(HardwareCompatibilityError):
            ServoOutputHardware(FakeServoCluster(count=17))

    def test_backend_range_is_reported_in_centidegrees(self):
        hardware = ServoOutputHardware(FakeServoCluster())
        self.assertEqual(hardware.backend_value_range_cd(0), (-9000, 9000))

    def test_current_profile_exposes_four_backend_range_mismatches(self):
        hardware = ServoOutputHardware(FakeServoCluster())
        errors = hardware.profile_compatibility_errors(load_profile_view())

        self.assertEqual(len(errors), 4)
        self.assertTrue(any("rf_tibia" in error for error in errors))
        self.assertTrue(any("rm_tibia" in error for error in errors))
        self.assertTrue(any("lm_tibia" in error for error in errors))
        self.assertTrue(any("lb_tibia" in error for error in errors))

    def test_current_profile_is_not_backend_compatible(self):
        hardware = ServoOutputHardware(FakeServoCluster())
        with self.assertRaises(HardwareCompatibilityError):
            hardware.require_profile_compatible(load_profile_view())

    def test_clipped_profile_is_backend_compatible(self):
        hardware = ServoOutputHardware(FakeServoCluster())
        self.assertTrue(hardware.require_profile_compatible(clipped_profile_view()))


class FactoryTests(unittest.TestCase):
    def _create_with_cluster(self, cluster):
        fake_servo = ModuleType("servo")
        fake_servo.ServoCluster = lambda **kwargs: cluster
        fake_servo.servo2040 = SimpleNamespace(
            SERVO_1=1,
            SERVO_18=18,
        )

        had_servo = "servo" in sys.modules
        previous_servo = sys.modules.get("servo")
        sys.modules["servo"] = fake_servo
        try:
            return create_servo2040_hardware()
        finally:
            if had_servo:
                sys.modules["servo"] = previous_servo
            else:
                sys.modules.pop("servo", None)

    def test_factory_retains_adapter_after_initial_disable_failure(self):
        cluster = FakeServoCluster()
        cluster.fail_disable = True

        hardware = self._create_with_cluster(cluster)

        self.assertIsNone(hardware.enabled)
        self.assertEqual(cluster.calls, [("disable_all", True)])

        cluster.fail_disable = False
        hardware.force_disabled()
        self.assertFalse(hardware.enabled)

    def test_factory_retains_disable_authority_if_full_adapter_init_fails(self):
        cluster = FakeServoCluster(count=17)

        hardware = self._create_with_cluster(cluster)

        self.assertFalse(hardware.enabled)
        with self.assertRaises(HardwareCompatibilityError):
            hardware.require_profile_compatible(load_profile_view())
        self.assertEqual(cluster.calls, [("disable_all", True)])

    def test_disable_only_fallback_recovers_from_initial_disable_failure(self):
        cluster = FakeServoCluster(count=17)
        cluster.fail_disable = True

        hardware = self._create_with_cluster(cluster)

        self.assertIsNone(hardware.enabled)
        cluster.fail_disable = False
        hardware.force_disabled()
        self.assertFalse(hardware.enabled)


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.cluster = FakeServoCluster()
        self.hardware = ServoOutputHardware(self.cluster)
        self.target = safe_channel_target()
        self.hardware.force_disabled()
        self.cluster.calls.clear()

    def test_new_adapter_starts_unknown_until_disable_commits(self):
        cluster = FakeServoCluster()
        hardware = ServoOutputHardware(cluster)

        self.assertIsNone(hardware.enabled)
        with self.assertRaises(HardwareStateError):
            hardware.enable_at_target(self.target)
        self.assertEqual(cluster.calls, [])

    def test_force_disabled_commits_disable(self):
        self.hardware.force_disabled()
        self.assertFalse(self.hardware.enabled)
        self.assertEqual(self.cluster.calls, [("disable_all", True)])

    def test_enable_at_target_stages_all_channels_then_loads_once(self):
        self.hardware.enable_at_target(self.target)

        self.assertEqual(len(self.cluster.calls), 19)
        for channel, call in enumerate(self.cluster.calls[:18]):
            self.assertEqual(call[0], "value")
            self.assertEqual(call[1], channel)
            self.assertFalse(call[3])
        self.assertEqual(self.cluster.calls[-1], ("load",))
        self.assertTrue(self.hardware.enabled)

    def test_enable_path_never_calls_enable_all(self):
        self.hardware.enable_at_target(self.target)
        self.assertNotIn("enable_all", [call[0] for call in self.cluster.calls])

    def test_complete_target_is_committed(self):
        self.hardware.enable_at_target(self.target)
        self.assertEqual(
            self.cluster.active,
            [value / 100.0 for value in self.target],
        )

    def test_apply_requires_enabled_outputs(self):
        with self.assertRaises(HardwareStateError):
            self.hardware.apply_target(self.target)

    def test_apply_uses_one_final_load(self):
        self.hardware.enable_at_target(self.target)
        self.cluster.calls.clear()

        updated = list(self.target)
        updated[0] += 100
        self.hardware.apply_target(updated)

        self.assertEqual(len(self.cluster.calls), 19)
        self.assertEqual(self.cluster.calls[-1], ("load",))
        self.assertEqual(self.cluster.active[0], updated[0] / 100.0)

    def test_force_disabled_clears_local_target_authority(self):
        self.hardware.enable_at_target(self.target)
        self.cluster.calls.clear()
        self.hardware.force_disabled()

        self.assertFalse(self.hardware.enabled)
        self.assertIsNone(self.hardware.last_channel_target_cd)
        self.assertEqual(self.cluster.calls, [("disable_all", True)])

    def test_backend_failure_forces_disable(self):
        self.cluster.fail_on_channel = 7

        with self.assertRaises(HardwareError):
            self.hardware.enable_at_target(self.target)

        self.assertFalse(self.hardware.enabled)
        self.assertIsNone(self.hardware.last_channel_target_cd)
        self.assertEqual(self.cluster.calls[-1], ("disable_all", True))

    def test_backend_range_rejected_before_any_commit(self):
        bad = list(self.target)
        bad[0] = 9001

        with self.assertRaises(HardwareCompatibilityError):
            self.hardware.enable_at_target(bad)

        self.assertEqual(self.cluster.calls, [])
        self.assertFalse(self.hardware.enabled)


if __name__ == "__main__":
    unittest.main()
