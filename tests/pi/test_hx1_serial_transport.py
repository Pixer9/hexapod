"""Tests for the pyserial-backed HX1 transport without real hardware."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC = REPO_ROOT / "src"
HX1_CONFIG = (
    REPO_ROOT / "config" / "hardware" / "servo2040.json"
)
sys.path.insert(0, str(SRC))

from hexapod.hx1 import (  # noqa: E402
    HX1Transport,
    HX1TransportClosedError,
    HX1TransportError,
    SerialHX1Transport,
    create_serial_hx1_transport,
    load_hx1_client_config,
)


class FakeSerialBackend:
    def __init__(
        self,
        *,
        rx=b"",
        write_chunks=None,
        is_open=True,
        read_error=None,
        write_error=None,
        waiting_error=None,
        close_error=None,
    ):
        self.rx = bytearray(rx)
        self.write_chunks = list(write_chunks or [])
        self.is_open = is_open
        self.read_error = read_error
        self.write_error = write_error
        self.waiting_error = waiting_error
        self.close_error = close_error
        self.closed = False
        self.writes = []

    @property
    def in_waiting(self):
        if self.waiting_error is not None:
            raise self.waiting_error
        return len(self.rx)

    def read(self, count):
        if self.read_error is not None:
            raise self.read_error
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        if self.write_error is not None:
            raise self.write_error

        payload = bytes(data)
        if self.write_chunks:
            count = self.write_chunks.pop(0)
        else:
            count = len(payload)

        count = min(count, len(payload))
        self.writes.append(payload[:count])
        return count

    def close(self):
        self.closed = True
        self.is_open = False
        if self.close_error is not None:
            raise self.close_error


class RecordingFactory:
    def __init__(self, backend=None, error=None):
        self.backend = (
            backend if backend is not None else FakeSerialBackend()
        )
        self.error = error
        self.calls = []

    def __call__(self, **kwargs):
        self.calls.append(dict(kwargs))
        if self.error is not None:
            raise self.error
        return self.backend


class SerialHX1TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_hx1_client_config(HX1_CONFIG)

    def make_transport(self, backend=None):
        factory = RecordingFactory(backend)
        transport = create_serial_hx1_transport(
            self.config,
            serial_factory=factory,
        )
        return transport, factory

    def test_config_contains_serial_transport_parameters(self):
        self.assertEqual(self.config.baudrate, 115200)
        self.assertAlmostEqual(
            self.config.write_timeout_s,
            0.25,
        )
        self.assertTrue(
            self.config.device_path.startswith(
                "/dev/serial/by-id/"
            )
        )

    def test_serial_transport_satisfies_transport_protocol(self):
        transport, _ = self.make_transport()

        self.assertIsInstance(transport, HX1Transport)

    def test_open_uses_nonblocking_pyserial_settings(self):
        transport, factory = self.make_transport()

        transport.open()

        self.assertTrue(transport.is_open)
        self.assertEqual(len(factory.calls), 1)

        options = factory.calls[0]
        self.assertEqual(
            options["port"],
            self.config.device_path,
        )
        self.assertEqual(options["baudrate"], 115200)
        self.assertEqual(options["timeout"], 0)
        self.assertEqual(options["write_timeout"], 0.25)
        self.assertFalse(options["xonxoff"])
        self.assertFalse(options["rtscts"])
        self.assertFalse(options["dsrdtr"])
        self.assertTrue(options["exclusive"])

    def test_open_is_idempotent(self):
        transport, factory = self.make_transport()

        transport.open()
        transport.open()

        self.assertEqual(len(factory.calls), 1)

    def test_close_is_idempotent(self):
        backend = FakeSerialBackend()
        transport, _ = self.make_transport(backend)
        transport.open()

        transport.close()
        transport.close()

        self.assertFalse(transport.is_open)
        self.assertTrue(backend.closed)

    def test_closed_read_and_write_are_rejected(self):
        transport, _ = self.make_transport()

        with self.assertRaises(HX1TransportClosedError):
            transport.read()

        with self.assertRaises(HX1TransportClosedError):
            transport.write(b"x")

    def test_read_is_nonblocking_when_no_bytes_wait(self):
        transport, _ = self.make_transport(
            FakeSerialBackend(rx=b"")
        )
        transport.open()

        self.assertEqual(transport.read(), b"")

    def test_read_respects_available_bytes_and_max_bytes(self):
        backend = FakeSerialBackend(rx=b"abcdefgh")
        transport, _ = self.make_transport(backend)
        transport.open()

        self.assertEqual(transport.read(3), b"abc")
        self.assertEqual(transport.read(99), b"defgh")
        self.assertEqual(transport.read(), b"")

    def test_write_completes_across_short_backend_writes(self):
        backend = FakeSerialBackend(
            write_chunks=[2, 1, 99]
        )
        transport, _ = self.make_transport(backend)
        transport.open()

        count = transport.write(b"abcdef")

        self.assertEqual(count, 6)
        self.assertEqual(
            b"".join(backend.writes),
            b"abcdef",
        )

    def test_zero_progress_write_is_transport_error(self):
        backend = FakeSerialBackend(write_chunks=[0])
        transport, _ = self.make_transport(backend)
        transport.open()

        with self.assertRaises(HX1TransportError):
            transport.write(b"abc")

    def test_backend_errors_are_wrapped_as_transport_errors(self):
        cases = (
            FakeSerialBackend(
                waiting_error=OSError("gone")
            ),
            FakeSerialBackend(
                read_error=OSError("gone"),
                rx=b"x",
            ),
            FakeSerialBackend(
                write_error=OSError("gone")
            ),
        )

        for backend in cases:
            with self.subTest(backend=backend):
                transport, _ = self.make_transport(backend)
                transport.open()

                with self.assertRaises(HX1TransportError):
                    if backend.waiting_error is not None:
                        transport.read()
                    elif backend.read_error is not None:
                        transport.read()
                    else:
                        transport.write(b"x")

    def test_open_failure_does_not_leave_transport_open(self):
        factory = RecordingFactory(
            error=OSError("permission denied")
        )
        transport = SerialHX1Transport(
            self.config.device_path,
            self.config.baudrate,
            write_timeout_s=self.config.write_timeout_s,
            serial_factory=factory,
        )

        with self.assertRaises(HX1TransportError):
            transport.open()

        self.assertFalse(transport.is_open)


if __name__ == "__main__":
    unittest.main()
