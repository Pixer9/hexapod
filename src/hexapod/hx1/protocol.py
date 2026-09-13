"""Pure framing primitives for the HX1 Pi <-> Servo 2040 protocol.

This module intentionally owns no serial I/O, runtime safety state, gait logic,
or hardware behavior. Its framing behavior mirrors the accepted HX1 v1
contract and the Servo 2040 implementation.
"""

from __future__ import annotations

from dataclasses import dataclass

PROTOCOL_PREFIX = "HX1"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 1

MAX_FRAME_BYTES = 512
MAX_MESSAGE_TYPE_CHARS = 32

UINT32_MAX = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


class HX1ProtocolError(ValueError):
    """Base class for HX1 framing/parsing errors."""


class HX1FrameTooLongError(HX1ProtocolError):
    """A complete or partial frame exceeded the v1 bound."""


class HX1FrameFormatError(HX1ProtocolError):
    """A complete frame is structurally malformed."""


class HX1CRCError(HX1ProtocolError):
    """A complete frame failed CRC validation."""


@dataclass(frozen=True, slots=True)
class HX1Frame:
    seq: int
    message_type: str
    fields: tuple[str, ...]


class HX1Sequence:
    """Monotonic uint32 sender sequence with modulo wrap."""

    def __init__(self, initial: int = 0):
        _validate_seq(initial)
        self._next = initial

    @property
    def next_value(self) -> int:
        return self._next

    def take(self) -> int:
        value = self._next
        self._next = (value + 1) & UINT32_MAX
        return value

    def reset(self, value: int = 0) -> None:
        _validate_seq(value)
        self._next = value


def crc16_ccitt_false(data: bytes | bytearray) -> int:
    """Return CRC-16/CCITT-FALSE."""
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("crc16_ccitt_false expects bytes or bytearray")

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _validate_seq(seq: object) -> int:
    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ValueError("sequence number must be an integer")
    if seq < 0 or seq > UINT32_MAX:
        raise ValueError("sequence number must be in range 0..2^32-1")
    return seq


def _validate_message_type(message_type: object) -> str:
    if not isinstance(message_type, str) or not message_type:
        raise ValueError("message type must be a non-empty string")
    if len(message_type) > MAX_MESSAGE_TYPE_CHARS:
        raise ValueError(
            f"message type exceeds {MAX_MESSAGE_TYPE_CHARS} characters"
        )
    for char in message_type:
        if not ("A" <= char <= "Z" or "0" <= char <= "9" or char == "_"):
            raise ValueError(
                "message type must be an uppercase ASCII identifier"
            )
    return message_type


def _field_to_text(field: object) -> str:
    if isinstance(field, bool):
        raise ValueError("boolean fields are not valid HX1 fields")
    if isinstance(field, int):
        text = str(field)
    elif isinstance(field, str):
        text = field
    else:
        raise ValueError("HX1 fields must be str or int")

    if not text:
        raise ValueError("HX1 fields must not be empty")
    if text != text.strip():
        raise ValueError("HX1 fields must not contain surrounding whitespace")
    if "|" in text or "\r" in text or "\n" in text:
        raise ValueError("HX1 field contains a reserved character")
    try:
        text.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("HX1 fields must be ASCII") from exc
    return text


def encode_frame(
    seq: int,
    message_type: str,
    *fields: str | int,
) -> bytes:
    """Encode one complete HX1 frame including trailing newline."""
    _validate_seq(seq)
    _validate_message_type(message_type)

    body_fields = [PROTOCOL_PREFIX, str(seq), message_type]
    body_fields.extend(_field_to_text(field) for field in fields)

    body = "|".join(body_fields).encode("ascii")
    crc = crc16_ccitt_false(body)
    frame = body + f"|{crc:04X}\n".encode("ascii")

    if len(frame) > MAX_FRAME_BYTES:
        raise HX1FrameTooLongError(
            "encoded frame exceeds maximum frame size"
        )
    return frame


def parse_frame(raw: bytes | bytearray | str) -> HX1Frame:
    """Parse and CRC-check one complete HX1 frame."""
    if isinstance(raw, str):
        try:
            raw_bytes = raw.encode("ascii")
        except UnicodeEncodeError as exc:
            raise HX1FrameFormatError("frame must be ASCII") from exc
    elif isinstance(raw, (bytes, bytearray)):
        raw_bytes = bytes(raw)
    else:
        raise HX1FrameFormatError(
            "frame must be bytes, bytearray, or str"
        )

    if len(raw_bytes) > MAX_FRAME_BYTES:
        raise HX1FrameTooLongError("frame exceeds maximum frame size")

    if raw_bytes.endswith(b"\n"):
        raw_bytes = raw_bytes[:-1]
        if raw_bytes.endswith(b"\r"):
            raw_bytes = raw_bytes[:-1]

    if b"\n" in raw_bytes or b"\r" in raw_bytes:
        raise HX1FrameFormatError(
            "frame contains an unexpected line break"
        )
    if not raw_bytes:
        raise HX1FrameFormatError("empty frame")

    try:
        text = raw_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise HX1FrameFormatError("frame must be ASCII") from exc

    parts = text.split("|")
    if len(parts) < 4:
        raise HX1FrameFormatError("frame has too few fields")
    if parts[0] != PROTOCOL_PREFIX:
        raise HX1FrameFormatError("unsupported protocol prefix")

    seq_text = parts[1]
    if not seq_text or not seq_text.isdigit():
        raise HX1FrameFormatError(
            "sequence number is not unsigned decimal"
        )
    seq = int(seq_text)
    if seq > UINT32_MAX:
        raise HX1FrameFormatError(
            "sequence number is outside uint32 range"
        )

    message_type = parts[2]
    try:
        _validate_message_type(message_type)
    except ValueError as exc:
        raise HX1FrameFormatError(str(exc)) from exc

    crc_text = parts[-1]
    if len(crc_text) != 4:
        raise HX1FrameFormatError(
            "CRC must contain exactly four hexadecimal digits"
        )
    if any(
        not ("0" <= char <= "9" or "A" <= char <= "F")
        for char in crc_text
    ):
        raise HX1FrameFormatError(
            "CRC must be uppercase hexadecimal"
        )

    fields = tuple(parts[3:-1])
    for field in fields:
        if not field:
            raise HX1FrameFormatError(
                "protocol fields must not be empty"
            )
        if field != field.strip():
            raise HX1FrameFormatError(
                "protocol fields contain surrounding whitespace"
            )

    body = "|".join(parts[:-1]).encode("ascii")
    expected_crc = crc16_ccitt_false(body)
    received_crc = int(crc_text, 16)
    if received_crc != expected_crc:
        raise HX1CRCError("CRC mismatch")

    return HX1Frame(
        seq=seq,
        message_type=message_type,
        fields=fields,
    )


def sequence_is_newer(candidate: int, previous: int) -> bool:
    """Return True under HX1 uint32 half-range ordering."""
    _validate_seq(candidate)
    _validate_seq(previous)
    delta = (candidate - previous) & UINT32_MAX
    return 0 < delta < UINT32_HALF_RANGE


class HX1LineFramer:
    """Bounded newline framer for a byte stream."""

    def __init__(self):
        self._buffer = bytearray()
        self._discarding = False
        self.framing_errors = 0

    def reset(self) -> None:
        self._buffer = bytearray()
        self._discarding = False
        self.framing_errors = 0

    def discard_current_line(self) -> None:
        self._buffer = bytearray()
        self._discarding = True
        self.framing_errors += 1

    def feed(self, data: bytes | bytearray) -> tuple[bytes, ...]:
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError(
                "HX1LineFramer.feed expects bytes or bytearray"
            )

        frames: list[bytes] = []

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

        return tuple(frames)
