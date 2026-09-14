"""Buffered dedicated USB-CDC transport for HX1.

The built-in MicroPython CDC interface remains reserved for REPL/mpremote
maintenance. HX1 uses a second runtime CDC interface backed by MicroPython's
``usb-device`` / ``usb-device-cdc`` packages.

RX is bounded per scheduler iteration so a large host backlog cannot starve
watchdogs or actuator servicing. TX is nonblocking: frames are appended to a
small bounded software queue and copied into the CDC driver's own TX buffer as
space becomes available.

This module performs framing only. CRC and command parsing remain in
``protocol.py``.
"""

from .constants import MAX_FRAME_BYTES
from .protocol import LineFramer

DEFAULT_RX_BUDGET_BYTES = 512
DEFAULT_CDC_RX_BUFFER_BYTES = 4096
DEFAULT_CDC_TX_BUFFER_BYTES = 4096
DEFAULT_TX_QUEUE_BYTES = 4096

# MicroPython stream ioctl values used by usb.device.cdc.CDCInterface.
_MP_STREAM_POLL = 3
_MP_STREAM_POLL_RD = 0x01


class TransportError(RuntimeError):
    """Base dedicated USB-CDC transport error."""


def _require_positive_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("%s must be a positive integer" % name)
    return value


class BufferedUSBCDCTransport:
    """Bounded nonblocking HX1 adapter over a buffered CDC stream.

    ``cdc`` must provide the subset of ``usb.device.cdc.CDCInterface`` used by
    this class: ``is_open()``, ``dtr``, ``ioctl()``, ``readinto()``, and
    ``write(buf)``.

    The interface is considered host-connected only while both the USB
    interface is configured and DTR is asserted by the host serial client.
    Frames generated while disconnected are intentionally discarded rather
    than accumulated for later delivery.
    """

    def __init__(
        self,
        cdc,
        rx_budget_bytes=DEFAULT_RX_BUDGET_BYTES,
        tx_queue_bytes=DEFAULT_TX_QUEUE_BYTES,
        framer=None,
    ):
        self._cdc = cdc
        self._rx_budget_bytes = _require_positive_int(
            rx_budget_bytes,
            "rx_budget_bytes",
        )
        self._tx_queue_limit = _require_positive_int(
            tx_queue_bytes,
            "tx_queue_bytes",
        )
        self._framer = framer if framer is not None else LineFramer()
        self._rx_buffer = bytearray(self._rx_budget_bytes)

        # Keep outbound frames immutable once handed to CDC. Partial writes
        # advance an offset into the head frame instead of resizing a shared
        # bytearray after write().
        self._tx_frames = []
        self._tx_head_offset = 0
        self._tx_pending_count = 0

        self._host_connected = False

        self.read_errors = 0
        self.write_errors = 0
        self.tx_overflow_errors = 0
        self.disconnected_drops = 0

    @property
    def framing_errors(self):
        return self._framer.framing_errors

    @property
    def rx_budget_bytes(self):
        return self._rx_budget_bytes

    @property
    def tx_queue_bytes(self):
        return self._tx_queue_limit

    @property
    def tx_pending_bytes(self):
        return self._tx_pending_count

    @property
    def host_connected(self):
        return self._sync_connection_state()

    def poll_frames(self):
        """Return zero or more complete HX1 frames without blocking.

        At most ``rx_budget_bytes`` are copied out of the CDC driver's receive
        buffer in one scheduler iteration. Pending TX bytes are also advanced
        once per call.
        """
        if not self._sync_connection_state():
            return []

        self._flush_tx_pending()

        try:
            readable = self._cdc.ioctl(_MP_STREAM_POLL, _MP_STREAM_POLL_RD)
        except Exception as exc:
            self.read_errors += 1
            raise TransportError("USB CDC poll failed: %s" % exc)

        if not readable & _MP_STREAM_POLL_RD:
            return []

        try:
            count = self._cdc.readinto(self._rx_buffer)
        except Exception as exc:
            self.read_errors += 1
            raise TransportError("USB CDC read failed: %s" % exc)

        if count is None:
            return []

        if (
            isinstance(count, bool)
            or not isinstance(count, int)
            or count < 0
            or count > self._rx_budget_bytes
        ):
            self.read_errors += 1
            raise TransportError("USB CDC read returned an invalid byte count")

        if count == 0:
            return []

        return self._framer.feed(bytes(self._rx_buffer[:count]))

    def send_frame(self, frame):
        """Queue one already-encoded HX1 frame for nonblocking CDC delivery."""
        frame = self._validate_frame(frame)

        if not self._sync_connection_state():
            self.disconnected_drops += 1
            return

        # First consume any space that became available since the last scheduler
        # iteration. New frame bytes must remain ordered after older pending data.
        self._flush_tx_pending()

        if self._tx_pending_count + len(frame) > self._tx_queue_limit:
            self.write_errors += 1
            self.tx_overflow_errors += 1
            raise TransportError("USB CDC TX queue capacity exceeded")

        self._tx_frames.append(frame)
        self._tx_pending_count += len(frame)
        self._flush_tx_pending()

    def _validate_frame(self, frame):
        if not isinstance(frame, (bytes, bytearray)):
            raise TypeError("frame must be bytes or bytearray")

        frame = bytes(frame)

        if not frame or not frame.endswith(b"\n"):
            raise TransportError("outbound frame must end with newline")

        if len(frame) > MAX_FRAME_BYTES:
            raise TransportError("outbound frame exceeds maximum frame size")

        try:
            frame.decode("ascii")
        except UnicodeDecodeError:
            raise TransportError("outbound frame must be ASCII")

        return frame

    def _connection_is_active(self):
        try:
            return bool(self._cdc.is_open()) and bool(self._cdc.dtr)
        except Exception:
            return False

    def _sync_connection_state(self):
        connected = self._connection_is_active()

        if connected != self._host_connected:
            # Never allow a partial frame or stale response queue to cross a
            # host disconnect/reconnect boundary. Reset the framer immediately
            # so the first frame after a reconnect is not discarded.
            self._reset_framer_boundary()
            self._clear_tx_pending()
            self._host_connected = connected

        return connected

    def _clear_tx_pending(self):
        self._tx_frames = []
        self._tx_head_offset = 0
        self._tx_pending_count = 0

    def _reset_framer_boundary(self):
        errors = getattr(self._framer, "framing_errors", 0)
        reset = getattr(self._framer, "reset", None)

        if reset is not None:
            reset()
            try:
                self._framer.framing_errors = errors
            except Exception:
                pass
            return

        self._framer.discard_current_line()

    def _flush_tx_pending(self):
        if not self._tx_frames:
            return

        if not self._connection_is_active():
            self._sync_connection_state()
            return

        head = self._tx_frames[0]
        chunk = head[self._tx_head_offset :]

        try:
            written = self._cdc.write(chunk)
        except Exception as exc:
            self.write_errors += 1
            raise TransportError("USB CDC write failed: %s" % exc)

        if written is None:
            written = 0

        if (
            isinstance(written, bool)
            or not isinstance(written, int)
            or written < 0
            or written > len(chunk)
        ):
            self.write_errors += 1
            raise TransportError("USB CDC write returned an invalid byte count")

        if not written:
            return

        self._tx_head_offset += written
        self._tx_pending_count -= written

        if self._tx_head_offset == len(head):
            self._tx_frames.pop(0)
            self._tx_head_offset = 0


def create_buffered_usb_cdc_transport(
    rx_budget_bytes=DEFAULT_RX_BUDGET_BYTES,
    cdc_rx_buffer_bytes=DEFAULT_CDC_RX_BUFFER_BYTES,
    cdc_tx_buffer_bytes=DEFAULT_CDC_TX_BUFFER_BYTES,
    tx_queue_bytes=DEFAULT_TX_QUEUE_BYTES,
):
    """Create the dedicated runtime CDC interface used only by HX1.

    ``builtin_driver=True`` preserves MicroPython's original CDC interface for
    REPL/mpremote access while adding a second CDC function for robot control.
    ``timeout=0`` makes CDC reads/writes nonblocking from the scheduler's point
    of view; this adapter owns any remaining TX queueing.
    """
    rx_budget_bytes = _require_positive_int(rx_budget_bytes, "rx_budget_bytes")
    cdc_rx_buffer_bytes = _require_positive_int(
        cdc_rx_buffer_bytes,
        "cdc_rx_buffer_bytes",
    )
    cdc_tx_buffer_bytes = _require_positive_int(
        cdc_tx_buffer_bytes,
        "cdc_tx_buffer_bytes",
    )
    tx_queue_bytes = _require_positive_int(tx_queue_bytes, "tx_queue_bytes")

    import usb.device
    from usb.device.cdc import CDCInterface

    cdc = CDCInterface(
        timeout=0,
        rxbuf=cdc_rx_buffer_bytes,
        txbuf=cdc_tx_buffer_bytes,
    )

    usb.device.get().init(
        cdc,
        builtin_driver=True,
    )

    return BufferedUSBCDCTransport(
        cdc=cdc,
        rx_budget_bytes=rx_budget_bytes,
        tx_queue_bytes=tx_queue_bytes,
    )
