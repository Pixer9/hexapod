"""Nonblocking USB-CDC text transport for HX1.

The Pimoroni/MicroPython runtime exposes USB serial through ``sys.stdin`` and
``sys.stdout``. This adapter deliberately reads one character only after
``uselect.poll(0)`` reports input readiness, avoiding ``readline()`` blocking
on a partial frame.

The per-call RX budget bounds work done by the transport so command processing,
watchdogs, and actuator updates cannot be starved by an arbitrarily large input
backlog.

This module performs framing only. CRC and command parsing remain in
``protocol.py``.
"""

from .constants import MAX_FRAME_BYTES
from .protocol import LineFramer

DEFAULT_RX_BUDGET_BYTES = 256


class TransportError(RuntimeError):
    """Base USB-CDC transport error."""


class USBTextTransport:
    """Bounded nonblocking text-stream adapter.

    ``reader`` must provide ``read(1)``.
    ``writer`` must provide ``write(str)``.
    ``poller`` must provide ``poll(0)`` and already be registered for input.
    """

    def __init__(
        self,
        reader,
        writer,
        poller,
        rx_budget_bytes=DEFAULT_RX_BUDGET_BYTES,
        framer=None,
    ):
        if (
            isinstance(rx_budget_bytes, bool)
            or not isinstance(rx_budget_bytes, int)
            or rx_budget_bytes <= 0
        ):
            raise ValueError("rx_budget_bytes must be a positive integer")

        self._reader = reader
        self._writer = writer
        self._poller = poller
        self._rx_budget_bytes = rx_budget_bytes
        self._framer = framer if framer is not None else LineFramer()

        self.read_errors = 0
        self.write_errors = 0

    @property
    def framing_errors(self):
        return self._framer.framing_errors

    @property
    def rx_budget_bytes(self):
        return self._rx_budget_bytes

    def poll_frames(self):
        """Return zero or more complete raw frames without blocking.

        At most ``rx_budget_bytes`` stream bytes/characters are consumed in one
        call, even when additional USB input remains queued.
        """
        frames = []
        consumed = 0

        while consumed < self._rx_budget_bytes:
            try:
                ready = self._poller.poll(0)
            except Exception as exc:
                self.read_errors += 1
                raise TransportError("USB input poll failed: %s" % exc)

            if not ready:
                break

            try:
                chunk = self._reader.read(1)
            except Exception:
                self.read_errors += 1
                break

            if chunk is None or chunk == "" or chunk == b"":
                break

            if isinstance(chunk, str):
                # read(1) should return exactly one character. Handle a larger
                # result safely anyway, while still applying the byte budget.
                for char in chunk:
                    if consumed >= self._rx_budget_bytes:
                        break

                    try:
                        raw = char.encode("ascii")
                    except UnicodeEncodeError:
                        # Do not substitute or normalize malformed input: that
                        # would change the bytes over which CRC was defined.
                        self._framer.discard_current_line()
                        consumed += 1
                        continue

                    frames.extend(self._framer.feed(raw))
                    consumed += 1
            elif isinstance(chunk, (bytes, bytearray)):
                # A custom/test reader may be byte oriented. Preserve bytes
                # exactly; parse_frame() will reject non-ASCII complete frames.
                raw_chunk = bytes(chunk)
                remaining = self._rx_budget_bytes - consumed
                raw_chunk = raw_chunk[:remaining]
                frames.extend(self._framer.feed(raw_chunk))
                consumed += len(raw_chunk)
            else:
                self.read_errors += 1
                self._framer.discard_current_line()
                break

        return frames

    def send_frame(self, frame):
        """Write one already-encoded HX1 frame to the USB text stream."""
        if not isinstance(frame, (bytes, bytearray)):
            raise TypeError("frame must be bytes or bytearray")

        frame = bytes(frame)

        if not frame or not frame.endswith(b"\n"):
            raise TransportError("outbound frame must end with newline")

        if len(frame) > MAX_FRAME_BYTES:
            raise TransportError("outbound frame exceeds maximum frame size")

        try:
            text = frame.decode("ascii")
        except UnicodeDecodeError:
            raise TransportError("outbound frame must be ASCII")

        try:
            self._writer.write(text)

            flush = getattr(self._writer, "flush", None)
            if flush is not None:
                flush()
        except Exception as exc:
            self.write_errors += 1
            raise TransportError("USB output write failed: %s" % exc)


def create_usb_cdc_transport(rx_budget_bytes=DEFAULT_RX_BUDGET_BYTES):
    """Create the production MicroPython USB-CDC transport."""
    import sys

    import uselect

    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)

    return USBTextTransport(
        reader=sys.stdin,
        writer=sys.stdout,
        poller=poller,
        rx_budget_bytes=rx_budget_bytes,
    )
