"""Raw-byte transport boundary for HX1.

The transport contract deliberately knows nothing about HX1 frames, sessions,
commands, robot state, or safety policy. It only moves bytes.

Real transports must provide non-blocking reads: ``read()`` returns currently
available bytes, or ``b""`` when nothing is available. A later runtime/link
layer owns framing, scheduling, reconnect policy, and client/session behavior.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


class HX1TransportError(RuntimeError):
    """Base class for HX1 transport failures."""


class HX1TransportClosedError(HX1TransportError):
    """An operation requiring an open transport was attempted while closed."""


class HX1TransportValueError(HX1TransportError, ValueError):
    """Transport input is malformed or unsupported."""


@runtime_checkable
class HX1Transport(Protocol):
    """Minimal raw-byte transport required by the future HX1 link layer."""

    @property
    def is_open(self) -> bool:
        """Whether the transport is currently available for I/O."""
        ...

    def open(self) -> None:
        """Open the transport.

        Implementations should make repeated ``open()`` calls idempotent.
        """
        ...

    def close(self) -> None:
        """Close the transport.

        Implementations should make repeated ``close()`` calls idempotent.
        """
        ...

    def write(self, data: bytes | bytearray) -> int:
        """Write all supplied bytes and return the number accepted."""
        ...

    def read(self, max_bytes: int = 4096) -> bytes:
        """Return up to ``max_bytes`` currently available bytes without blocking."""
        ...


def _normalize_write_data(data: object) -> bytes:
    if not isinstance(data, (bytes, bytearray)):
        raise HX1TransportValueError("transport write data must be bytes or bytearray")
    return bytes(data)


def _validate_max_bytes(max_bytes: object) -> int:
    if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or max_bytes <= 0:
        raise HX1TransportValueError("max_bytes must be a positive integer")
    return max_bytes


class FakeHX1Transport:
    """Deterministic in-memory raw-byte transport for host-side tests.

    Test code may inject bytes that the Pi should later read and inspect bytes
    written by the Pi. No background thread, wall clock, or implicit peer
    behavior exists.
    """

    def __init__(self):
        self._is_open = False
        self._read_buffer = bytearray()
        self._writes: list[bytes] = []

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def pending_read_bytes(self) -> int:
        """Number of injected peer bytes not yet consumed by ``read()``."""
        return len(self._read_buffer)

    @property
    def writes(self) -> tuple[bytes, ...]:
        """Immutable snapshot of all writes not yet removed by ``take_writes``."""
        return tuple(self._writes)

    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    def write(self, data: bytes | bytearray) -> int:
        self._require_open()
        payload = _normalize_write_data(data)
        self._writes.append(payload)
        return len(payload)

    def read(self, max_bytes: int = 4096) -> bytes:
        self._require_open()
        count = _validate_max_bytes(max_bytes)

        if not self._read_buffer:
            return b""

        count = min(count, len(self._read_buffer))
        data = bytes(self._read_buffer[:count])
        del self._read_buffer[:count]
        return data

    def inject_read_data(self, data: bytes | bytearray) -> None:
        """Queue deterministic peer bytes for future ``read()`` calls.

        Injection is a test-side operation and is allowed while the transport
        is closed so a peer response may be scripted before opening the link.
        """
        payload = _normalize_write_data(data)
        self._read_buffer.extend(payload)

    def take_writes(self) -> tuple[bytes, ...]:
        """Return and clear captured Pi writes."""
        captured = tuple(self._writes)
        self._writes.clear()
        return captured

    def clear(self) -> None:
        """Clear injected and captured bytes without changing open state."""
        self._read_buffer.clear()
        self._writes.clear()

    def _require_open(self) -> None:
        if not self._is_open:
            raise HX1TransportClosedError("HX1 transport is not open")
