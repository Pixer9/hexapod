"""Tests for the raw-byte HX1 transport contract and deterministic fake."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
HX1_CONFIG = REPO_ROOT / "config" / "hardware" / "servo2040.json"
sys.path.insert(0, str(SRC))

from hexapod.hx1 import (  # noqa: E402
    FakeHX1Transport,
    HX1ClientCore,
    HX1LineFramer,
    HX1Transport,
    HX1TransportClosedError,
    HX1TransportValueError,
    encode_frame,
    load_hx1_client_config,
    parse_frame,
)


class FakeHX1TransportTests(unittest.TestCase):
    def test_fake_satisfies_transport_protocol(self):
        self.assertIsInstance(FakeHX1Transport(), HX1Transport)

    def test_starts_closed_and_open_close_are_idempotent(self):
        transport = FakeHX1Transport()

        self.assertFalse(transport.is_open)

        transport.open()
        transport.open()
        self.assertTrue(transport.is_open)

        transport.close()
        transport.close()
        self.assertFalse(transport.is_open)

    def test_read_and_write_require_open_transport(self):
        transport = FakeHX1Transport()

        with self.assertRaises(HX1TransportClosedError):
            transport.read()

        with self.assertRaises(HX1TransportClosedError):
            transport.write(b"abc")

    def test_write_captures_exact_bytes(self):
        transport = FakeHX1Transport()
        transport.open()

        payload = b"HX1|example\n"
        accepted = transport.write(payload)

        self.assertEqual(accepted, len(payload))
        self.assertEqual(transport.writes, (payload,))

    def test_take_writes_returns_and_clears_capture(self):
        transport = FakeHX1Transport()
        transport.open()
        transport.write(b"one")
        transport.write(b"two")

        self.assertEqual(
            transport.take_writes(),
            (b"one", b"two"),
        )
        self.assertEqual(transport.writes, ())

    def test_read_is_nonblocking_when_no_data_is_available(self):
        transport = FakeHX1Transport()
        transport.open()

        self.assertEqual(transport.read(), b"")

    def test_injected_read_data_can_be_consumed_in_chunks(self):
        transport = FakeHX1Transport()
        transport.inject_read_data(b"abcdefgh")
        transport.open()

        self.assertEqual(transport.pending_read_bytes, 8)
        self.assertEqual(transport.read(3), b"abc")
        self.assertEqual(transport.pending_read_bytes, 5)
        self.assertEqual(transport.read(3), b"def")
        self.assertEqual(transport.read(99), b"gh")
        self.assertEqual(transport.read(), b"")

    def test_invalid_transport_arguments_are_rejected(self):
        transport = FakeHX1Transport()
        transport.open()

        with self.assertRaises(HX1TransportValueError):
            transport.write("not-bytes")

        with self.assertRaises(HX1TransportValueError):
            transport.read(0)

        with self.assertRaises(HX1TransportValueError):
            transport.read(True)

    def test_clear_preserves_open_state(self):
        transport = FakeHX1Transport()
        transport.open()
        transport.inject_read_data(b"incoming")
        transport.write(b"outgoing")

        transport.clear()

        self.assertTrue(transport.is_open)
        self.assertEqual(transport.pending_read_bytes, 0)
        self.assertEqual(transport.writes, ())


class HX1FakeTransportIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_hx1_client_config(HX1_CONFIG)

    def test_hello_fragmented_info_round_trip(self):
        client = HX1ClientCore(self.config)
        transport = FakeHX1Transport()
        framer = HX1LineFramer()
        transport.open()

        hello = client.hello()
        transport.write(hello.frame)

        captured = transport.take_writes()
        self.assertEqual(captured, (hello.frame,))
        self.assertEqual(
            parse_frame(captured[0]).message_type,
            "HELLO",
        )

        info = encode_frame(
            100,
            "INFO",
            hello.seq,
            "A1B2C3D4",
            1,
            "fw-1.0",
            "e661410403724132",
            self.config.expected_profile_id,
            self.config.expected_profile_revision,
            self.config.expected_profile_sha256,
            self.config.joint_count,
            "00000020",
            "DISARMED",
        )

        # Deliberately fragment the peer response to prove the raw transport
        # contract and HX1 line framer compose correctly.
        transport.inject_read_data(info)
        complete = []

        while transport.pending_read_bytes:
            complete.extend(framer.feed(transport.read(7)))

        self.assertEqual(complete, [info])

        message = client.accept_frame(complete[0])

        self.assertEqual(message.session, "A1B2C3D4")
        self.assertTrue(client.negotiated)

    def test_fake_transport_does_not_generate_implicit_peer_behavior(self):
        client = HX1ClientCore(self.config)
        transport = FakeHX1Transport()
        transport.open()

        transport.write(client.hello().frame)

        self.assertEqual(transport.read(), b"")
        self.assertFalse(client.negotiated)


if __name__ == "__main__":
    unittest.main()
