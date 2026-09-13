"""Host-side tests for Servo 2040 actuator-profile loading."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
PROFILE_PATH = FIRMWARE_SRC / "config" / "actuator-profile.json"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.profile import (  # noqa: E402
    CANONICAL_JOINT_NAMES,
    ProfileError,
    ProfileQualificationError,
    load_profile,
    parse_profile_bytes,
    sha256_hex,
)


def load_raw_dict():
    return json.loads(PROFILE_PATH.read_text(encoding="utf-8"))


def encode_dict(data):
    return json.dumps(data, separators=(",", ":")).encode("utf-8")


class HashTests(unittest.TestCase):
    def test_sha256_standard_vector(self):
        self.assertEqual(
            sha256_hex(b"abc"),
            "BA7816BF8F01CFEA414140DE5DAE2223"
            "B00361A396177A9CB410FF61F20015AD",
        )

    def test_hash_is_exact_byte_sensitive(self):
        self.assertNotEqual(
            sha256_hex(b'{"a":1}'),
            sha256_hex(b'{"a": 1}'),
        )


class CurrentProfileTests(unittest.TestCase):
    def test_current_profile_is_structurally_valid(self):
        profile = load_profile(PROFILE_PATH)

        self.assertEqual(profile.schema_version, 1)
        self.assertEqual(profile.profile_id, "hexapod-standard-v1")
        self.assertEqual(profile.profile_revision, 3)
        self.assertEqual(profile.joint_count, 18)
        self.assertEqual(len(profile.joints), 18)
        self.assertEqual(
            tuple(joint.name for joint in profile.joints),
            CANONICAL_JOINT_NAMES,
        )
        self.assertEqual(len(profile.profile_hash), 64)

    def test_current_profile_is_arm_qualified(self):
        profile = load_profile(PROFILE_PATH)

        self.assertTrue(profile.arm_qualified)
        self.assertEqual(profile.qualification_errors(), ())
        self.assertTrue(profile.require_arm_qualified())
        self.assertEqual(
            tuple(joint.max_rate_cd_s for joint in profile.joints),
            (25000,) * 18,
        )

    def test_channel_mapping_matches_contract(self):
        profile = load_profile(PROFILE_PATH)

        expected = (12, 13, 14, 6, 7, 8, 0, 1, 2, 15, 16, 17, 9, 10, 11, 3, 4, 5)
        self.assertEqual(
            tuple(joint.channel for joint in profile.joints),
            expected,
        )

    def test_direction_pattern_matches_contract(self):
        profile = load_profile(PROFILE_PATH)

        expected = (-1, 1, 1) * 6
        self.assertEqual(
            tuple(joint.direction for joint in profile.joints),
            expected,
        )

    def test_derived_logical_ranges_match_backend_compatible_calibration(self):
        profile = load_profile(PROFILE_PATH)

        expected = (
            (-9400, 8600),
            (-10000, 4000),
            (1000, 11900),
            (-8300, 9700),
            (-10000, 4000),
            (1000, 12500),
            (-8400, 9600),
            (-10000, 4000),
            (1000, 13000),
            (-8100, 9900),
            (-10000, 4000),
            (1000, 13000),
            (-8400, 9600),
            (-10000, 4000),
            (1000, 12900),
            (-8200, 9800),
            (-10000, 4000),
            (1000, 12500),
        )

        actual = tuple(
            (joint.logical_min_cd, joint.logical_max_cd)
            for joint in profile.joints
        )
        self.assertEqual(actual, expected)

    def test_joint_lookup(self):
        profile = load_profile(PROFILE_PATH)
        self.assertEqual(profile.joint(0).name, "rf_coxa")
        self.assertEqual(profile.joint(17).name, "lb_tibia")

        with self.assertRaises(IndexError):
            profile.joint(18)


class QualificationTests(unittest.TestCase):

    def test_profile_becomes_unqualified_when_any_rate_is_missing(self):
        data = load_raw_dict()
        data["joints"][0]["max_rate_cd_s"] = None

        profile = parse_profile_bytes(encode_dict(data))

        self.assertFalse(profile.arm_qualified)
        self.assertEqual(len(profile.qualification_errors()), 1)

        with self.assertRaises(ProfileQualificationError):
            profile.require_arm_qualified()


class StructuralValidationTests(unittest.TestCase):
    def mutate(self):
        return copy.deepcopy(load_raw_dict())

    def assert_invalid(self, data):
        with self.assertRaises(ProfileError):
            parse_profile_bytes(encode_dict(data))

    def test_wrong_schema_version_is_rejected(self):
        data = self.mutate()
        data["schema_version"] = 2
        self.assert_invalid(data)

    def test_wrong_joint_count_is_rejected(self):
        data = self.mutate()
        data["joint_count"] = 17
        self.assert_invalid(data)

    def test_missing_joint_is_rejected(self):
        data = self.mutate()
        data["joints"].pop()
        self.assert_invalid(data)

    def test_out_of_order_index_is_rejected(self):
        data = self.mutate()
        data["joints"][0]["index"] = 1
        self.assert_invalid(data)

    def test_wrong_canonical_name_is_rejected(self):
        data = self.mutate()
        data["joints"][0]["name"] = "front_right_coxa"
        self.assert_invalid(data)

    def test_duplicate_channel_is_rejected(self):
        data = self.mutate()
        data["joints"][1]["channel"] = data["joints"][0]["channel"]
        self.assert_invalid(data)

    def test_channel_out_of_range_is_rejected(self):
        data = self.mutate()
        data["joints"][0]["channel"] = 18
        self.assert_invalid(data)

    def test_bad_direction_is_rejected(self):
        data = self.mutate()
        data["joints"][0]["direction"] = 0
        self.assert_invalid(data)

    def test_servo_range_must_be_ordered(self):
        data = self.mutate()
        data["joints"][0]["servo_min_cd"] = 9000
        data["joints"][0]["servo_max_cd"] = -9000
        self.assert_invalid(data)

    def test_zero_rate_is_rejected_when_rate_is_present(self):
        data = self.mutate()
        data["joints"][0]["max_rate_cd_s"] = 0
        self.assert_invalid(data)

    def test_boolean_is_not_accepted_as_integer(self):
        data = self.mutate()
        data["joints"][0]["channel"] = True
        self.assert_invalid(data)

    def test_bad_profile_identifier_is_rejected(self):
        data = self.mutate()
        data["profile_id"] = "hexapod standard"
        self.assert_invalid(data)


if __name__ == "__main__":
    unittest.main()
