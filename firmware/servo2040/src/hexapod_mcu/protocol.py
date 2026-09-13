"""Pure protocol primitives for the HX1 Pi <-> Servo 2040 protocol.

This module intentionally contains no hardware I/O and no runtime-state logic.
It is written to remain usable under both CPython and MicroPython.
"""

from .constants import (
    MAX_FRAME_BYTES,
    MAX_MESSAGE_TYPE_CHARS,
    PROTOCOL_PREFIX,
    UINT32_HALF_RANGE,
    UINT32_MAX,
)


class ProtocolError(Exception):
    """Base class for protocol framing/parsing errors."""


class FrameTooLongError(ProtocolError):
    """A frame exceeded the accepted maximum size."""


class FrameFormatError(ProtocolError):
    """A frame is structurally malformed."""


class CRCError(ProtocolError):
    """A frame's CRC does not match its body."""


def crc16_ccitt_false(data):
    """Return CRC-16/CCITT-FALSE for a bytes-like object."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def _validate_seq(seq):
    if not isinstance(seq, int):
        raise ValueError("sequence number must be an integer")
    if seq < 0 or seq > UINT32_MAX:
        raise ValueError("sequence number must be in range 0..2^32-1")


def _validate_message_type(message_type):
    if not isinstance(message_type, str) or not message_type:
        raise ValueError("message type must be a non-empty string")

    if len(message_type) > MAX_MESSAGE_TYPE_CHARS:
        raise ValueError("message type exceeds %d characters" % MAX_MESSAGE_TYPE_CHARS)

    for char in message_type:
        if not ("A" <= char <= "Z" or "0" <= char <= "9" or char == "_"):
            raise ValueError("message type must be an uppercase ASCII identifier")


def _field_to_text(field):
    if isinstance(field, bool):
        raise ValueError("boolean fields are not valid protocol fields")

    if isinstance(field, int):
        text = str(field)
    elif isinstance(field, str):
        text = field
    else:
        raise ValueError("protocol fields must be str or int")

    if not text:
        raise ValueError("protocol fields must not be empty")

    if text != text.strip():
        raise ValueError("protocol fields must not contain surrounding whitespace")

    if "|" in text or "\r" in text or "\n" in text:
        raise ValueError("protocol fields contain a reserved character")

    try:
        text.encode("ascii")
    except UnicodeEncodeError:
        raise ValueError("protocol fields must be ASCII")

    return text


def encode_frame(seq, message_type, *fields):
    """Encode one complete HX1 frame, including trailing newline."""
    _validate_seq(seq)
    _validate_message_type(message_type)

    body_fields = [PROTOCOL_PREFIX, str(seq), message_type]
    body_fields.extend(_field_to_text(field) for field in fields)

    body_text = "|".join(body_fields)
    body = body_text.encode("ascii")
    crc = crc16_ccitt_false(body)

    frame = body + ("|%04X\n" % crc).encode("ascii")

    if len(frame) > MAX_FRAME_BYTES:
        raise FrameTooLongError("encoded frame exceeds maximum frame size")

    return frame


def _normalize_complete_frame(raw):
    if isinstance(raw, str):
        try:
            raw = raw.encode("ascii")
        except UnicodeEncodeError:
            raise FrameFormatError("frame must be ASCII")
    elif not isinstance(raw, (bytes, bytearray)):
        raise FrameFormatError("frame must be bytes, bytearray, or str")

    raw = bytes(raw)

    if len(raw) > MAX_FRAME_BYTES:
        raise FrameTooLongError("frame exceeds maximum frame size")

    if raw.endswith(b"\n"):
        raw = raw[:-1]
        if raw.endswith(b"\r"):
            raw = raw[:-1]

    if b"\n" in raw or b"\r" in raw:
        raise FrameFormatError("frame contains an unexpected line break")

    if not raw:
        raise FrameFormatError("empty frame")

    return raw


def parse_frame(raw):
    """Parse and CRC-check one complete frame.

    Returns:
        (seq, message_type, fields)

    ``fields`` is a tuple of strings. Message-specific semantic validation
    belongs in the command-dispatch layer, not in this framing module.
    """
    raw = _normalize_complete_frame(raw)

    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError:
        raise FrameFormatError("frame must be ASCII")

    parts = text.split("|")

    if len(parts) < 4:
        raise FrameFormatError("frame has too few fields")

    if parts[0] != PROTOCOL_PREFIX:
        raise FrameFormatError("unsupported protocol prefix")

    seq_text = parts[1]
    if not seq_text or not seq_text.isdigit():
        raise FrameFormatError("sequence number is not unsigned decimal")

    seq = int(seq_text)
    if seq < 0 or seq > UINT32_MAX:
        raise FrameFormatError("sequence number is outside uint32 range")

    message_type = parts[2]
    try:
        _validate_message_type(message_type)
    except ValueError as exc:
        raise FrameFormatError(str(exc))

    crc_text = parts[-1]
    if len(crc_text) != 4:
        raise FrameFormatError("CRC must contain exactly four hexadecimal digits")

    for char in crc_text:
        if not ("0" <= char <= "9" or "A" <= char <= "F"):
            raise FrameFormatError("CRC must be uppercase hexadecimal")

    fields = tuple(parts[3:-1])
    for field in fields:
        if not field:
            raise FrameFormatError("protocol fields must not be empty")
        if field != field.strip():
            raise FrameFormatError("protocol fields contain surrounding whitespace")

    body_text = "|".join(parts[:-1])
    body = body_text.encode("ascii")
    expected_crc = crc16_ccitt_false(body)
    received_crc = int(crc_text, 16)

    if received_crc != expected_crc:
        raise CRCError("CRC mismatch")

    return seq, message_type, fields


def sequence_is_newer(candidate, previous):
    """Return True when candidate is newer under uint32 half-range ordering."""
    _validate_seq(candidate)
    _validate_seq(previous)

    delta = (candidate - previous) & UINT32_MAX
    return 0 < delta < UINT32_HALF_RANGE


class LineFramer:
    """Bounded newline framer for a serial byte stream.

    Oversized or explicitly aborted input is discarded through the next
    newline. The object then automatically resumes collecting the following
    frame.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._discarding = False
        self.framing_errors = 0

    def reset(self):
        self._buffer = bytearray()
        self._discarding = False
        self.framing_errors = 0

    def discard_current_line(self):
        """Discard the current partial line through its next newline.

        Used by the text USB-CDC adapter when the stream yields a non-ASCII
        character that cannot be represented faithfully as protocol bytes.
        """
        self._buffer = bytearray()
        self._discarding = True
        self.framing_errors += 1

    def feed(self, data):
        """Feed bytes and return a list of complete frame byte strings."""
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("LineFramer.feed expects bytes or bytearray")

        frames = []

        for byte in data:
            if self._discarding:
                if byte == 0x0A:
                    self._discarding = False
                continue

            self._buffer.append(byte)

            if byte == 0x0A:
                frame = bytes(self._buffer)
                self._buffer = bytearray()

                if len(frame) > MAX_FRAME_BYTES:
                    self.framing_errors += 1
                    continue

                frames.append(frame)
                continue

            if len(self._buffer) > MAX_FRAME_BYTES:
                self._buffer = bytearray()
                self._discarding = True
                self.framing_errors += 1

        return frames
