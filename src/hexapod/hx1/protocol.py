"""Pure framing primitives for the HX1 Pi <-> Servo 2040 protocol.

HX1 v1 keeps the existing newline-delimited ASCII telemetry format for MCU ->
Pi messages while Pi -> MCU commands use a compact binary command frame.  The
split removes decimal parsing from the Servo 2040 real-time command path without
changing the higher-level lifecycle/session/watchdog contract.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

PROTOCOL_PREFIX = "HX1"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 1

MAX_FRAME_BYTES = 512
MAX_MESSAGE_TYPE_CHARS = 32

UINT32_MAX = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000

COMMAND_MAGIC = b"HX"
COMMAND_WIRE_VERSION = 1
COMMAND_HEADER_FORMAT = "<2sBBHII"
COMMAND_HEADER_BYTES = struct.calcsize(COMMAND_HEADER_FORMAT)
COMMAND_CRC_BYTES = 2
MAX_COMMAND_PAYLOAD_BYTES = 96
MAX_COMMAND_FRAME_BYTES = (
    COMMAND_HEADER_BYTES + MAX_COMMAND_PAYLOAD_BYTES + COMMAND_CRC_BYTES
)

_COMMAND_IDS = {
    "HELLO": 1,
    "HEARTBEAT": 2,
    "STAGE": 3,
    "ARM": 4,
    "START": 5,
    "TARGET": 6,
    "STOP": 7,
    "DISARM": 8,
    "ESTOP": 9,
    "CLEAR_ESTOP": 10,
    "CLEAR_FAULT": 11,
    "GET_STATUS": 12,
}
_COMMAND_NAMES = {value: key for key, value in _COMMAND_IDS.items()}
_COMMAND_TYPES = frozenset(_COMMAND_IDS)
_SIMPLE_SESSION_COMMANDS = frozenset(
    {"ARM", "START", "STOP", "DISARM", "CLEAR_ESTOP", "CLEAR_FAULT", "GET_STATUS"}
)


class HX1ProtocolError(ValueError):
    """Base class for HX1 framing/parsing errors."""


class HX1FrameTooLongError(HX1ProtocolError):
    """A complete or partial frame exceeded the accepted bound."""


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
        raise ValueError(f"message type exceeds {MAX_MESSAGE_TYPE_CHARS} characters")
    for char in message_type:
        if not ("A" <= char <= "Z" or "0" <= char <= "9" or char == "_"):
            raise ValueError("message type must be an uppercase ASCII identifier")
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


def _uint32_field(field: object, name: str, *, positive: bool = False) -> int:
    if isinstance(field, bool):
        raise ValueError(f"{name} must be uint32")
    if isinstance(field, int):
        value = field
    elif isinstance(field, str) and field and field.isdigit():
        value = int(field)
    else:
        raise ValueError(f"{name} must be uint32")
    if value < 0 or value > UINT32_MAX or (positive and value == 0):
        raise ValueError(f"{name} outside accepted uint32 range")
    return value


def _int16_field(field: object, name: str) -> int:
    if isinstance(field, bool):
        raise ValueError(f"{name} must be signed int16")
    if isinstance(field, int):
        value = field
    elif isinstance(field, str):
        try:
            value = int(field, 10)
        except ValueError as exc:
            raise ValueError(f"{name} must be signed int16") from exc
    else:
        raise ValueError(f"{name} must be signed int16")
    if value < -32768 or value > 32767:
        raise ValueError(f"{name} outside signed int16 range")
    return value


def _session_to_uint32(field: object, *, allow_zero: bool) -> int:
    if not isinstance(field, str) or len(field) != 8:
        raise ValueError("session must contain exactly eight hex digits")
    if any(not ("0" <= c <= "9" or "A" <= c <= "F") for c in field):
        raise ValueError("session must be uppercase hexadecimal")
    value = int(field, 16)
    if not allow_zero and value == 0:
        raise ValueError("session must be non-zero")
    return value


def _session_text(value: int) -> str:
    return f"{value:08X}"


def _encode_text_frame(
    seq: int,
    message_type: str,
    *fields: str | int,
) -> bytes:
    body_fields = [PROTOCOL_PREFIX, str(seq), message_type]
    body_fields.extend(_field_to_text(field) for field in fields)
    body = "|".join(body_fields).encode("ascii")
    crc = crc16_ccitt_false(body)
    frame = body + f"|{crc:04X}\n".encode("ascii")
    if len(frame) > MAX_FRAME_BYTES:
        raise HX1FrameTooLongError("encoded frame exceeds maximum frame size")
    return frame


def encode_command_frame(
    seq: int,
    message_type: str,
    *fields: str | int,
) -> bytes:
    """Encode one Pi -> MCU binary command frame."""
    _validate_seq(seq)
    _validate_message_type(message_type)
    try:
        command_id = _COMMAND_IDS[message_type]
    except KeyError as exc:
        raise ValueError(f"unsupported binary command {message_type!r}") from exc

    session = 0
    payload = b""

    if message_type == "HELLO":
        if len(fields) != 2:
            raise ValueError("HELLO requires client_minor and expected_profile_id")
        client_minor = _uint32_field(fields[0], "client_minor")
        profile_text = _field_to_text(fields[1])
        profile_bytes = profile_text.encode("ascii")
        payload = struct.pack("<I", client_minor) + profile_bytes

    elif message_type == "HEARTBEAT":
        if len(fields) != 2:
            raise ValueError("HEARTBEAT requires session and host_uptime_ms")
        session = _session_to_uint32(fields[0], allow_zero=False)
        host_uptime_ms = _uint32_field(fields[1], "host_uptime_ms")
        payload = struct.pack("<I", host_uptime_ms)

    elif message_type == "STAGE":
        if len(fields) != 19:
            raise ValueError("STAGE requires session and 18 joints")
        session = _session_to_uint32(fields[0], allow_zero=False)
        joints = tuple(
            _int16_field(value, f"joint[{index}]")
            for index, value in enumerate(fields[1:])
        )
        payload = struct.pack("<18h", *joints)

    elif message_type in _SIMPLE_SESSION_COMMANDS:
        if len(fields) != 1:
            raise ValueError(f"{message_type} requires exactly one session field")
        session = _session_to_uint32(
            fields[0],
            allow_zero=(message_type == "GET_STATUS"),
        )

    elif message_type == "TARGET":
        if len(fields) != 20:
            raise ValueError("TARGET requires session, period_ms, and 18 joints")
        session = _session_to_uint32(fields[0], allow_zero=False)
        period_ms = _uint32_field(fields[1], "period_ms", positive=True)
        joints = tuple(
            _int16_field(value, f"joint[{index}]")
            for index, value in enumerate(fields[2:])
        )
        payload = struct.pack("<I18h", period_ms, *joints)

    elif message_type == "ESTOP":
        if len(fields) != 2:
            raise ValueError("ESTOP requires session and reason")
        session = _session_to_uint32(fields[0], allow_zero=True)
        reason = _field_to_text(fields[1]).encode("ascii")
        payload = reason

    else:  # pragma: no cover - protected by _COMMAND_IDS lookup
        raise ValueError(f"unsupported binary command {message_type!r}")

    if len(payload) > MAX_COMMAND_PAYLOAD_BYTES:
        raise HX1FrameTooLongError("binary command payload exceeds maximum size")

    header = struct.pack(
        COMMAND_HEADER_FORMAT,
        COMMAND_MAGIC,
        COMMAND_WIRE_VERSION,
        command_id,
        len(payload),
        seq,
        session,
    )
    body = header + payload
    crc = crc16_ccitt_false(body)
    return body + struct.pack("<H", crc)


def encode_frame(
    seq: int,
    message_type: str,
    *fields: str | int,
) -> bytes:
    """Encode HX1 using the direction-appropriate wire representation.

    Known Pi -> MCU commands are encoded in the binary command format.  MCU ->
    Pi telemetry/response message types retain the accepted ASCII HX1 format.
    """
    _validate_seq(seq)
    _validate_message_type(message_type)
    if message_type in _COMMAND_TYPES:
        return encode_command_frame(seq, message_type, *fields)
    return _encode_text_frame(seq, message_type, *fields)


def _normalize_text_frame(raw: bytes | bytearray | str) -> bytes:
    if isinstance(raw, str):
        try:
            raw_bytes = raw.encode("ascii")
        except UnicodeEncodeError as exc:
            raise HX1FrameFormatError("frame must be ASCII") from exc
    elif isinstance(raw, (bytes, bytearray)):
        raw_bytes = bytes(raw)
    else:
        raise HX1FrameFormatError("frame must be bytes, bytearray, or str")

    if len(raw_bytes) > MAX_FRAME_BYTES:
        raise HX1FrameTooLongError("frame exceeds maximum frame size")
    if raw_bytes.endswith(b"\n"):
        raw_bytes = raw_bytes[:-1]
        if raw_bytes.endswith(b"\r"):
            raw_bytes = raw_bytes[:-1]
    if b"\n" in raw_bytes or b"\r" in raw_bytes:
        raise HX1FrameFormatError("frame contains an unexpected line break")
    if not raw_bytes:
        raise HX1FrameFormatError("empty frame")
    return raw_bytes


def _parse_text_frame(raw: bytes | bytearray | str) -> HX1Frame:
    raw_bytes = _normalize_text_frame(raw)
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
        raise HX1FrameFormatError("sequence number is not unsigned decimal")
    seq = int(seq_text)
    if seq > UINT32_MAX:
        raise HX1FrameFormatError("sequence number is outside uint32 range")

    message_type = parts[2]
    try:
        _validate_message_type(message_type)
    except ValueError as exc:
        raise HX1FrameFormatError(str(exc)) from exc

    crc_text = parts[-1]
    if len(crc_text) != 4:
        raise HX1FrameFormatError("CRC must contain exactly four hexadecimal digits")
    if any(not ("0" <= char <= "9" or "A" <= char <= "F") for char in crc_text):
        raise HX1FrameFormatError("CRC must be uppercase hexadecimal")

    fields = tuple(parts[3:-1])
    for field in fields:
        if not field:
            raise HX1FrameFormatError("protocol fields must not be empty")
        if field != field.strip():
            raise HX1FrameFormatError("protocol fields contain surrounding whitespace")

    body = "|".join(parts[:-1]).encode("ascii")
    if int(crc_text, 16) != crc16_ccitt_false(body):
        raise HX1CRCError("CRC mismatch")

    return HX1Frame(seq=seq, message_type=message_type, fields=fields)


def _decode_command_frame(raw: bytes | bytearray) -> tuple[int, str, tuple[str, ...]]:
    if not isinstance(raw, (bytes, bytearray)):
        raise HX1FrameFormatError("binary command frame must be bytes or bytearray")
    raw_bytes = bytes(raw)
    if len(raw_bytes) < COMMAND_HEADER_BYTES + COMMAND_CRC_BYTES:
        raise HX1FrameFormatError("binary command frame is too short")
    if len(raw_bytes) > MAX_COMMAND_FRAME_BYTES:
        raise HX1FrameTooLongError("binary command frame exceeds maximum size")

    try:
        magic, version, command_id, payload_len, seq, session = struct.unpack_from(
            COMMAND_HEADER_FORMAT, raw_bytes, 0
        )
    except struct.error as exc:
        raise HX1FrameFormatError("binary command header is malformed") from exc

    if magic != COMMAND_MAGIC:
        raise HX1FrameFormatError("binary command magic mismatch")
    if version != COMMAND_WIRE_VERSION:
        raise HX1FrameFormatError("unsupported binary command wire version")
    if payload_len > MAX_COMMAND_PAYLOAD_BYTES:
        raise HX1FrameTooLongError("binary command payload exceeds maximum size")

    expected_len = COMMAND_HEADER_BYTES + payload_len + COMMAND_CRC_BYTES
    if len(raw_bytes) != expected_len:
        raise HX1FrameFormatError("binary command frame length mismatch")

    body = raw_bytes[:-COMMAND_CRC_BYTES]
    received_crc = struct.unpack_from("<H", raw_bytes, len(raw_bytes) - 2)[0]
    if received_crc != crc16_ccitt_false(body):
        raise HX1CRCError("CRC mismatch")

    message_type = _COMMAND_NAMES.get(command_id, f"CMD_{command_id}")
    payload = raw_bytes[COMMAND_HEADER_BYTES:-COMMAND_CRC_BYTES]
    session_text = _session_text(session)

    if message_type == "HELLO":
        if session != 0 or payload_len < 5:
            raise HX1FrameFormatError("HELLO binary payload is malformed")
        client_minor = struct.unpack_from("<I", payload, 0)[0]
        try:
            profile_id = payload[4:].decode("ascii")
        except UnicodeDecodeError as exc:
            raise HX1FrameFormatError("HELLO profile id must be ASCII") from exc
        if not profile_id:
            raise HX1FrameFormatError("HELLO profile id must not be empty")
        return seq, message_type, (str(client_minor), profile_id)

    if message_type == "HEARTBEAT":
        if payload_len != 4:
            raise HX1FrameFormatError("HEARTBEAT binary payload has wrong length")
        uptime = struct.unpack_from("<I", payload, 0)[0]
        return seq, message_type, (session_text, str(uptime))

    if message_type == "STAGE":
        if payload_len != 36:
            raise HX1FrameFormatError("STAGE binary payload has wrong length")
        joints = struct.unpack_from("<18h", payload, 0)
        return seq, message_type, (session_text,) + tuple(str(v) for v in joints)

    if message_type in _SIMPLE_SESSION_COMMANDS:
        if payload_len != 0:
            raise HX1FrameFormatError(f"{message_type} binary payload must be empty")
        return seq, message_type, (session_text,)

    if message_type == "TARGET":
        if payload_len != 40:
            raise HX1FrameFormatError("TARGET binary payload has wrong length")
        values = struct.unpack_from("<I18h", payload, 0)
        return (
            seq,
            message_type,
            (session_text, str(values[0])) + tuple(str(v) for v in values[1:]),
        )

    if message_type == "ESTOP":
        if payload_len < 1:
            raise HX1FrameFormatError("ESTOP reason must not be empty")
        try:
            reason = payload.decode("ascii")
        except UnicodeDecodeError as exc:
            raise HX1FrameFormatError("ESTOP reason must be ASCII") from exc
        return seq, message_type, (session_text, reason)

    return seq, message_type, (session_text,)


def _looks_binary(raw: object) -> bool:
    return (
        isinstance(raw, (bytes, bytearray))
        and len(raw) >= 2
        and bytes(raw[:2]) == COMMAND_MAGIC
        and not bytes(raw).startswith(b"HX1|")
    )


def parse_frame(raw: bytes | bytearray | str) -> HX1Frame:
    """Parse either a binary command frame or ASCII telemetry frame."""
    if _looks_binary(raw):
        seq, message_type, fields = _decode_command_frame(raw)  # type: ignore[arg-type]
        return HX1Frame(seq=seq, message_type=message_type, fields=fields)
    return _parse_text_frame(raw)


def sequence_is_newer(candidate: int, previous: int) -> bool:
    """Return True under HX1 uint32 half-range ordering."""
    _validate_seq(candidate)
    _validate_seq(previous)
    delta = (candidate - previous) & UINT32_MAX
    return 0 < delta < UINT32_HALF_RANGE


class HX1LineFramer:
    """Bounded newline framer for MCU -> Pi ASCII telemetry."""

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
            raise TypeError("HX1LineFramer.feed expects bytes or bytearray")

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
