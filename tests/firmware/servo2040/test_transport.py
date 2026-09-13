"""Host-side tests for the bounded USB-CDC transport."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.protocol import encode_frame  # noqa: E402
from hexapod_mcu.transport import (  # noqa: E402
    TransportError,
    USBTextTransport,
)


class FakeReader:
    def __init__(self, text=""):
        self.buffer = list(text)
        self.fail = False

    def available(self):
        return bool(self.buffer)

    def read(self, count):
        if self.fail:
            raise OSError("injected read failure")
        if not self.buffer:
            return ""
        return self.buffer.pop(0)


class FakePoller:
    def __init__(self, reader):
        self.reader = reader
        self.fail = False

    def poll(self, timeout):
        self.last_timeout = timeout
        if self.fail:
            raise OSError("injected poll failure")
        return [(1, 1)] if self.reader.available() else []


class FakeWriter:
    def __init__(self):
        self.text = ""
        self.flush_count = 0
        self.fail = False

    def write(self, text):
        if self.fail:
            raise OSError("injected write failure")
        self.text += text

    def flush(self):
        self.flush_count += 1


def make_transport(text="", budget=256):
    reader = FakeReader(text)
    writer = FakeWriter()
    poller = FakePoller(reader)
    return USBTextTransport(reader, writer, poller, budget), reader, writer, poller


class ReceiveTests(unittest.TestCase):
    def test_no_input_returns_immediately(self):
        transport, _, _, poller = make_transport()
        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(poller.last_timeout, 0)

    def test_complete_frame_is_returned(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        transport, _, _, _ = make_transport(frame.decode("ascii"))
        self.assertEqual(transport.poll_frames(), [frame])

    def test_fragmented_frame_survives_multiple_polls(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        text = frame.decode("ascii")

        transport, reader, _, _ = make_transport(text[:5])
        self.assertEqual(transport.poll_frames(), [])

        reader.buffer.extend(text[5:])
        self.assertEqual(transport.poll_frames(), [frame])

    def test_multiple_frames_are_returned_in_order(self):
        first = encode_frame(1, "ARM", "A1B2C3D4")
        second = encode_frame(2, "START", "A1B2C3D4")
        text = (first + second).decode("ascii")

        transport, _, _, _ = make_transport(text)
        self.assertEqual(transport.poll_frames(), [first, second])

    def test_rx_budget_bounds_work_per_call(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        text = frame.decode("ascii")

        transport, _, _, _ = make_transport(text, budget=5)

        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(transport.poll_frames(), [frame])

    def test_non_ascii_text_discards_current_line_and_recovers(self):
        valid = encode_frame(2, "STOP", "A1B2C3D4")
        text = "HX1|1|ARM|é|0000\n" + valid.decode("ascii")

        transport, _, _, _ = make_transport(text)
        self.assertEqual(transport.poll_frames(), [valid])
        self.assertEqual(transport.framing_errors, 1)

    def test_reader_failure_is_counted_without_crashing_loop(self):
        transport, reader, _, _ = make_transport("X")
        reader.fail = True

        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(transport.read_errors, 1)

    def test_poller_failure_is_a_transport_error(self):
        transport, _, _, poller = make_transport("X")
        poller.fail = True

        with self.assertRaises(TransportError):
            transport.poll_frames()

        self.assertEqual(transport.read_errors, 1)


class SendTests(unittest.TestCase):
    def test_send_writes_exact_ascii_frame_and_flushes(self):
        frame = encode_frame(7, "ACK", 6, "ARM")
        transport, _, writer, _ = make_transport()

        transport.send_frame(frame)

        self.assertEqual(writer.text, frame.decode("ascii"))
        self.assertEqual(writer.flush_count, 1)

    def test_send_rejects_missing_newline(self):
        transport, _, _, _ = make_transport()

        with self.assertRaises(TransportError):
            transport.send_frame(b"HX1|1|ACK|0|ARM|0000")

    def test_write_failure_is_counted(self):
        frame = encode_frame(7, "ACK", 6, "ARM")
        transport, _, writer, _ = make_transport()
        writer.fail = True

        with self.assertRaises(TransportError):
            transport.send_frame(frame)

        self.assertEqual(transport.write_errors, 1)


if __name__ == "__main__":
    unittest.main()
