"""Tests for the synchronous HX1 link/pump composition layer."""

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
    HX1Ack,
    HX1ClientCore,
    HX1Info,
    HX1Link,
    HX1PeerMismatchError,
    HX1TransportError,
    encode_frame,
    load_hx1_client_config,
)


class FaultingTransport(FakeHX1Transport):
    def __init__(self):
        super().__init__()
        self.fail_read = False
        self.fail_write = False
        self.read_calls = 0

    def read(self, max_bytes=4096):
        self.read_calls += 1
        if self.fail_read:
            raise HX1TransportError("synthetic read failure")
        return super().read(max_bytes)

    def write(self, data):
        if self.fail_write:
            raise HX1TransportError("synthetic write failure")
        return super().write(data)


class HX1LinkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_hx1_client_config(HX1_CONFIG)

    def make_link(self, transport=None):
        client = HX1ClientCore(self.config)
        transport = FakeHX1Transport() if transport is None else transport
        return HX1Link(client, transport)

    def info_frame(
        self,
        hello_seq,
        *,
        profile_id=None,
    ):
        return encode_frame(
            100,
            "INFO",
            hello_seq,
            "A1B2C3D4",
            1,
            "fw-1.0",
            "e661410403724132",
            (self.config.expected_profile_id if profile_id is None else profile_id),
            self.config.expected_profile_revision,
            self.config.expected_profile_sha256,
            self.config.joint_count,
            "00000020",
            "DISARMED",
        )

    def negotiate(self, link):
        if not link.is_open:
            link.open()

        hello = link.client.hello()
        link.send(hello)
        link.transport.take_writes()
        link.transport.inject_read_data(self.info_frame(hello.seq))

        messages = link.poll()
        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], HX1Info)
        self.assertTrue(link.client.negotiated)
        return hello, messages[0]

    def test_open_sends_nothing(self):
        link = self.make_link()

        link.open()

        self.assertTrue(link.is_open)
        self.assertEqual(link.transport.writes, ())
        self.assertFalse(link.client.negotiated)

    def test_repeated_open_preserves_negotiated_session(self):
        link = self.make_link()
        self.negotiate(link)
        session = link.client.session

        link.open()

        self.assertEqual(link.client.session, session)
        self.assertTrue(link.client.negotiated)

    def test_close_forgets_local_session_authority(self):
        link = self.make_link()
        self.negotiate(link)

        link.close()

        self.assertFalse(link.is_open)
        self.assertFalse(link.client.negotiated)
        self.assertIsNone(link.client.session)

    def test_send_writes_exact_outbound_frame(self):
        link = self.make_link()
        link.open()

        hello = link.client.hello()
        accepted = link.send(hello)

        self.assertEqual(accepted, len(hello.frame))
        self.assertEqual(
            link.transport.take_writes(),
            (hello.frame,),
        )

    def test_poll_is_nonblocking_when_no_data_available(self):
        transport = FaultingTransport()
        link = self.make_link(transport)
        link.open()

        self.assertEqual(link.poll(), ())
        self.assertEqual(transport.read_calls, 1)

    def test_fragmented_info_is_reassembled_across_poll_calls(self):
        link = self.make_link()
        link.open()

        hello = link.client.hello()
        link.send(hello)

        info = self.info_frame(hello.seq)
        split = len(info) // 2

        link.transport.inject_read_data(info[:split])
        self.assertEqual(link.poll(), ())
        self.assertFalse(link.client.negotiated)

        link.transport.inject_read_data(info[split:])
        messages = link.poll()

        self.assertEqual(len(messages), 1)
        self.assertIsInstance(messages[0], HX1Info)
        self.assertTrue(link.client.negotiated)

    def test_multiple_complete_frames_are_returned_in_wire_order(self):
        link = self.make_link()
        hello, _ = self.negotiate(link)

        ack_one = encode_frame(
            101,
            "ACK",
            hello.seq,
            "HELLO",
        )
        ack_two = encode_frame(
            102,
            "ACK",
            77,
            "GET_STATUS",
        )
        link.transport.inject_read_data(ack_one + ack_two)

        messages = link.poll()

        self.assertEqual(len(messages), 2)
        self.assertIsInstance(messages[0], HX1Ack)
        self.assertIsInstance(messages[1], HX1Ack)
        self.assertEqual(messages[0].seq, 101)
        self.assertEqual(messages[1].seq, 102)

    def test_crc_bad_frame_is_dropped_and_counted(self):
        link = self.make_link()
        link.open()

        raw = bytearray(encode_frame(1, "EVENT", "BOOT", "ready"))
        raw[-3] = ord("0")
        link.transport.inject_read_data(raw)

        self.assertEqual(link.poll(), ())
        self.assertEqual(link.protocol_errors, 1)

    def test_semantically_bad_frame_is_dropped_and_counted(self):
        link = self.make_link()
        link.open()

        # ACK requires REF_SEQ and COMMAND. Framing/CRC are valid, but the
        # message schema is incomplete.
        link.transport.inject_read_data(encode_frame(1, "ACK", 4))

        self.assertEqual(link.poll(), ())
        self.assertEqual(link.message_errors, 1)

    def test_peer_mismatch_remains_explicit_not_silently_dropped(self):
        link = self.make_link()
        link.open()

        hello = link.client.hello()
        link.send(hello)
        link.transport.inject_read_data(
            self.info_frame(
                hello.seq,
                profile_id="wrong-profile",
            )
        )

        with self.assertRaises(HX1PeerMismatchError):
            link.poll()

        self.assertFalse(link.client.negotiated)

    def test_transport_failure_invalidates_negotiated_session(self):
        transport = FaultingTransport()
        link = self.make_link(transport)
        self.negotiate(link)
        self.assertTrue(link.client.negotiated)

        transport.fail_read = True

        with self.assertRaises(HX1TransportError):
            link.poll()

        self.assertFalse(link.client.negotiated)
        self.assertIsNone(link.client.session)


if __name__ == "__main__":
    unittest.main()
