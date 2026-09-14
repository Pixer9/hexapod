"""Host-side tests for the dedicated buffered USB-CDC HX1 transport."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.protocol import encode_frame  # noqa: E402
from hexapod_mcu.transport import (  # noqa: E402
    BufferedUSBCDCTransport,
    TransportError,
)


class FakeCDC:
    def __init__(self, incoming=b""):
        self.open = True
        self.dtr = True
        self.incoming = bytearray(incoming)
        self.written = bytearray()
        self.write_args = []
        self.read_sizes = []
        self.poll_calls = 0
        self.write_limit = None
        self.fail_poll = False
        self.fail_read = False
        self.fail_write = False

    def is_open(self):
        return self.open

    def ioctl(self, request, argument):
        self.poll_calls += 1
        if self.fail_poll:
            raise OSError("injected poll failure")
        if request != 3 or argument != 1:
            raise AssertionError("unexpected ioctl")
        return 1 if self.incoming else 0

    def readinto(self, buffer):
        self.read_sizes.append(len(buffer))
        if self.fail_read:
            raise OSError("injected read failure")
        if not self.incoming:
            return None

        count = min(len(buffer), len(self.incoming))
        buffer[:count] = self.incoming[:count]
        del self.incoming[:count]
        return count

    def write(self, data):
        if self.fail_write:
            raise OSError("injected write failure")

        # Retain the exact object passed by the transport. This catches
        # transports that mutate or resize a buffer after write() returns.
        self.write_args.append(data)

        if self.write_limit is None:
            count = len(data)
        else:
            count = min(self.write_limit, len(data))

        self.written.extend(bytes(data[:count]))
        return count


def make_transport(incoming=b"", rx_budget=512, tx_queue=4096):
    cdc = FakeCDC(incoming)
    transport = BufferedUSBCDCTransport(
        cdc,
        rx_budget_bytes=rx_budget,
        tx_queue_bytes=tx_queue,
    )
    return transport, cdc


class ReceiveTests(unittest.TestCase):
    def test_no_input_returns_immediately(self):
        transport, cdc = make_transport()

        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(cdc.poll_calls, 1)
        self.assertEqual(cdc.read_sizes, [])

    def test_complete_frame_is_returned(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        transport, _ = make_transport(frame)

        self.assertEqual(transport.poll_frames(), [frame])

    def test_fragmented_frame_survives_multiple_polls(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        transport, cdc = make_transport(frame[:5])

        self.assertEqual(transport.poll_frames(), [])

        cdc.incoming.extend(frame[5:])
        self.assertEqual(transport.poll_frames(), [frame])

    def test_multiple_frames_are_returned_in_order(self):
        first = encode_frame(1, "ARM", "A1B2C3D4")
        second = encode_frame(2, "START", "A1B2C3D4")
        transport, _ = make_transport(first + second)

        self.assertEqual(transport.poll_frames(), [first, second])

    def test_rx_budget_bounds_each_scheduler_poll(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        transport, cdc = make_transport(frame, rx_budget=5)

        frames = []
        while not frames:
            frames = transport.poll_frames()

        self.assertEqual(frames, [frame])
        self.assertTrue(cdc.read_sizes)
        self.assertTrue(all(size == 5 for size in cdc.read_sizes))

    def test_disconnected_interface_is_not_read(self):
        frame = encode_frame(1, "ARM", "A1B2C3D4")
        transport, cdc = make_transport(frame)
        cdc.dtr = False

        self.assertEqual(transport.poll_frames(), [])
        self.assertEqual(cdc.read_sizes, [])

    def test_disconnect_discards_partial_inbound_frame(self):
        partial = encode_frame(1, "ARM", "A1B2C3D4")
        valid = encode_frame(2, "STOP", "A1B2C3D4")
        transport, cdc = make_transport(partial[:7])

        self.assertEqual(transport.poll_frames(), [])

        cdc.dtr = False
        self.assertEqual(transport.poll_frames(), [])

        cdc.dtr = True
        cdc.incoming.extend(valid)
        self.assertEqual(transport.poll_frames(), [valid])

    def test_poll_failure_is_counted_and_raised(self):
        transport, cdc = make_transport()
        cdc.fail_poll = True

        with self.assertRaises(TransportError):
            transport.poll_frames()

        self.assertEqual(transport.read_errors, 1)

    def test_read_failure_is_counted_and_raised(self):
        transport, cdc = make_transport(b"X")
        cdc.fail_read = True

        with self.assertRaises(TransportError):
            transport.poll_frames()

        self.assertEqual(transport.read_errors, 1)


class SendTests(unittest.TestCase):
    def test_send_writes_exact_frame_bytes(self):
        frame = encode_frame(7, "ACK", 6, "ARM")
        transport, cdc = make_transport()

        transport.send_frame(frame)

        self.assertEqual(bytes(cdc.written), frame)
        self.assertEqual(transport.tx_pending_bytes, 0)
        self.assertEqual(cdc.write_args, [frame])
        self.assertIsInstance(cdc.write_args[0], bytes)

    def test_partial_write_is_queued_and_drained_without_reordering(self):
        first = encode_frame(7, "ACK", 6, "ARM")
        second = encode_frame(8, "ACK", 7, "START")
        transport, cdc = make_transport()
        cdc.write_limit = 5

        transport.send_frame(first)
        transport.send_frame(second)

        while transport.tx_pending_bytes:
            transport.poll_frames()

        self.assertEqual(bytes(cdc.written), first + second)
        self.assertTrue(cdc.write_args)
        self.assertTrue(
            all(isinstance(write_arg, bytes) for write_arg in cdc.write_args)
        )
        self.assertEqual(cdc.write_args[0], first)

    def test_disconnected_frames_are_dropped_not_replayed(self):
        stale = encode_frame(7, "ACK", 6, "ARM")
        current = encode_frame(8, "ACK", 7, "START")
        transport, cdc = make_transport()
        cdc.dtr = False

        transport.send_frame(stale)
        self.assertEqual(transport.disconnected_drops, 1)
        self.assertEqual(bytes(cdc.written), b"")

        cdc.dtr = True
        transport.send_frame(current)
        self.assertEqual(bytes(cdc.written), current)

    def test_tx_queue_overflow_is_counted_and_raised(self):
        frame = encode_frame(7, "ACK", 6, "ARM")
        transport, cdc = make_transport(tx_queue=len(frame) + 1)
        cdc.write_limit = 0

        transport.send_frame(frame)

        with self.assertRaises(TransportError):
            transport.send_frame(frame)

        self.assertEqual(transport.tx_overflow_errors, 1)
        self.assertEqual(transport.write_errors, 1)

    def test_send_rejects_missing_newline(self):
        transport, _ = make_transport()

        with self.assertRaises(TransportError):
            transport.send_frame(b"HX1|1|ACK|0|ARM|0000")

    def test_send_rejects_non_ascii_frame(self):
        transport, _ = make_transport()

        with self.assertRaises(TransportError):
            transport.send_frame(b"HX1|1|ACK|\xff\n")

    def test_write_failure_is_counted_and_raised(self):
        frame = encode_frame(7, "ACK", 6, "ARM")
        transport, cdc = make_transport()
        cdc.fail_write = True

        with self.assertRaises(TransportError):
            transport.send_frame(frame)

        self.assertEqual(transport.write_errors, 1)


if __name__ == "__main__":
    unittest.main()
