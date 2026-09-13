"""Tests for transport-independent HX1 client/session behavior."""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"

HX1_CONFIG = (
    REPO_ROOT / "config" / "hardware" / "servo2040.json"
)
ACTUATOR_PROFILE = (
    REPO_ROOT
    / "firmware"
    / "servo2040"
    / "src"
    / "config"
    / "actuator-profile.json"
)

sys.path.insert(0, str(SRC))

from hexapod.hx1 import (  # noqa: E402
    HX1ClientCore,
    HX1PeerMismatchError,
    HX1SessionError,
    HX1UnexpectedResponseError,
    load_hx1_client_config,
    encode_frame,
    parse_frame,
)


class HX1ClientConfigTests(unittest.TestCase):
    def test_loads_standard_servo2040_link(self):
        config = load_hx1_client_config(HX1_CONFIG)

        self.assertEqual(config.client_minor, 1)
        self.assertEqual(config.required_server_minor, 1)
        self.assertEqual(config.joint_count, 18)
        self.assertEqual(config.heartbeat_period_ms, 100)
        self.assertEqual(config.target_period_ms, 20)
        self.assertEqual(
            config.expected_profile_id,
            "hexapod-standard-v1",
        )

    def test_expected_profile_hash_matches_exact_mcu_profile_bytes(self):
        config = load_hx1_client_config(HX1_CONFIG)
        raw = ACTUATOR_PROFILE.read_bytes()

        actual = hashlib.sha256(raw).hexdigest().upper()

        self.assertEqual(
            config.expected_profile_sha256,
            actual,
        )


class HX1ClientCoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_hx1_client_config(HX1_CONFIG)

    def make_client(self):
        return HX1ClientCore(self.config)

    def info_frame(
        self,
        hello_seq,
        *,
        session="A1B2C3D4",
        server_minor=1,
        profile_id=None,
        profile_revision=None,
        profile_hash=None,
        joint_count=None,
        state="DISARMED",
    ):
        return encode_frame(
            100,
            "INFO",
            hello_seq,
            session,
            server_minor,
            "fw-1.0",
            "e661410403724132",
            (
                self.config.expected_profile_id
                if profile_id is None
                else profile_id
            ),
            (
                self.config.expected_profile_revision
                if profile_revision is None
                else profile_revision
            ),
            (
                self.config.expected_profile_sha256
                if profile_hash is None
                else profile_hash
            ),
            (
                self.config.joint_count
                if joint_count is None
                else joint_count
            ),
            "00000020",
            state,
        )

    def negotiate(self, client):
        hello = client.hello()
        message = client.accept_frame(
            self.info_frame(hello.seq)
        )
        return hello, message

    def test_hello_uses_expected_profile_and_invalidates_old_authority(self):
        client = self.make_client()
        first_hello, _ = self.negotiate(client)
        self.assertTrue(client.negotiated)

        second_hello = client.hello()

        self.assertFalse(client.negotiated)
        self.assertIsNone(client.session)
        self.assertEqual(
            client.pending_hello_seq,
            second_hello.seq,
        )

        parsed = parse_frame(second_hello.frame)
        self.assertEqual(parsed.message_type, "HELLO")
        self.assertEqual(
            parsed.fields,
            (
                str(self.config.client_minor),
                self.config.expected_profile_id,
            ),
        )
        self.assertNotEqual(first_hello.seq, second_hello.seq)

    def test_matching_info_establishes_session(self):
        client = self.make_client()
        hello = client.hello()

        info = client.accept_frame(
            self.info_frame(hello.seq)
        )

        self.assertTrue(client.negotiated)
        self.assertEqual(client.session, "A1B2C3D4")
        self.assertEqual(
            client.peer.profile_revision,
            self.config.expected_profile_revision,
        )
        self.assertEqual(
            client.peer.profile_hash,
            self.config.expected_profile_sha256,
        )
        self.assertEqual(info.ref_seq, hello.seq)

    def test_info_without_pending_hello_is_rejected(self):
        client = self.make_client()

        with self.assertRaises(HX1UnexpectedResponseError):
            client.accept_frame(self.info_frame(0))

    def test_info_wrong_ref_seq_is_rejected(self):
        client = self.make_client()
        hello = client.hello()

        with self.assertRaises(HX1UnexpectedResponseError):
            client.accept_frame(
                self.info_frame(hello.seq + 1)
            )

    def test_peer_profile_mismatch_prevents_session(self):
        client = self.make_client()
        hello = client.hello()

        with self.assertRaises(HX1PeerMismatchError):
            client.accept_frame(
                self.info_frame(
                    hello.seq,
                    profile_id="wrong-profile",
                )
            )

        self.assertFalse(client.negotiated)
        self.assertIsNone(client.session)

    def test_peer_hash_mismatch_prevents_session(self):
        client = self.make_client()
        hello = client.hello()

        with self.assertRaises(HX1PeerMismatchError):
            client.accept_frame(
                self.info_frame(
                    hello.seq,
                    profile_hash="0" * 64,
                )
            )

        self.assertFalse(client.negotiated)

    def test_server_minor_too_old_prevents_session(self):
        client = self.make_client()
        hello = client.hello()

        with self.assertRaises(HX1PeerMismatchError):
            client.accept_frame(
                self.info_frame(
                    hello.seq,
                    server_minor=0,
                )
            )

    def test_session_command_is_blocked_before_negotiation(self):
        client = self.make_client()

        with self.assertRaises(HX1SessionError):
            client.get_status()

    def test_estop_is_available_without_session(self):
        client = self.make_client()

        estop = client.estop("controller_estop")
        parsed = parse_frame(estop.frame)

        self.assertEqual(parsed.message_type, "ESTOP")
        self.assertEqual(
            parsed.fields,
            (
                "00000000",
                "controller_estop",
            ),
        )

    def test_stage_converts_degrees_to_centidegrees(self):
        client = self.make_client()
        self.negotiate(client)

        values = [0.0] * 18
        values[0] = 12.345
        values[1] = -8.505

        stage = client.stage_degrees(values)
        parsed = parse_frame(stage.frame)

        self.assertEqual(parsed.message_type, "STAGE")
        self.assertEqual(parsed.fields[0], client.session)
        self.assertEqual(parsed.fields[1], "1235")
        self.assertEqual(parsed.fields[2], "-851")
        self.assertEqual(len(parsed.fields), 19)

    def test_target_uses_configured_20_ms_period(self):
        client = self.make_client()
        self.negotiate(client)

        target = client.target_degrees([0.0] * 18)
        parsed = parse_frame(target.frame)

        self.assertEqual(parsed.message_type, "TARGET")
        self.assertEqual(parsed.fields[0], client.session)
        self.assertEqual(parsed.fields[1], "20")
        self.assertEqual(len(parsed.fields), 20)

    def test_heartbeat_uses_negotiated_session(self):
        client = self.make_client()
        self.negotiate(client)

        heartbeat = client.heartbeat(123456)
        parsed = parse_frame(heartbeat.frame)

        self.assertEqual(parsed.message_type, "HEARTBEAT")
        self.assertEqual(
            parsed.fields,
            (client.session, "123456"),
        )


if __name__ == "__main__":
    unittest.main()
