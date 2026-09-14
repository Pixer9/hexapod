"""HX1 protocol primitives shared by Servo 2040 telemetry and command RX.

MCU -> Pi telemetry remains newline-delimited ASCII HX1.  Pi -> MCU commands use
compact length-framed binary packets so the Servo 2040 does not spend its
real-time budget decoding 18 decimal joint strings on every STAGE/TARGET.
"""

import struct

from .constants import (
    MAX_FRAME_BYTES,
    MAX_MESSAGE_TYPE_CHARS,
    PROTOCOL_PREFIX,
    UINT32_HALF_RANGE,
    UINT32_MAX,
)

COMMAND_MAGIC = b"HX"
COMMAND_WIRE_VERSION = 1
COMMAND_HEADER_FORMAT = "<2sBBHII"
COMMAND_HEADER_BYTES = 14
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
_COMMAND_NAMES = {}
for _name, _command_id in _COMMAND_IDS.items():
    _COMMAND_NAMES[_command_id] = _name
_COMMAND_TYPES = tuple(_COMMAND_IDS.keys())
_FIXED_PAYLOAD_LENGTHS = {
    2: 4,  # HEARTBEAT
    3: 36,  # STAGE
    4: 0,  # ARM
    5: 0,  # START
    6: 40,  # TARGET
    7: 0,  # STOP
    8: 0,  # DISARM
    10: 0,  # CLEAR_ESTOP
    11: 0,  # CLEAR_FAULT
    12: 0,  # GET_STATUS
}
_SIMPLE_SESSION_COMMANDS = (
    "ARM",
    "START",
    "STOP",
    "DISARM",
    "CLEAR_ESTOP",
    "CLEAR_FAULT",
    "GET_STATUS",
)


class ProtocolError(Exception):
    """Base class for protocol framing/parsing errors."""


class FrameTooLongError(ProtocolError):
    """A frame exceeded the accepted maximum size."""


class FrameFormatError(ProtocolError):
    """A frame is structurally malformed."""


class CRCError(ProtocolError):
    """A frame's CRC does not match its body."""


_CRC16_CCITT_FALSE_TABLE = (
    0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50A5,
    0x60C6,
    0x70E7,
    0x8108,
    0x9129,
    0xA14A,
    0xB16B,
    0xC18C,
    0xD1AD,
    0xE1CE,
    0xF1EF,
    0x1231,
    0x0210,
    0x3273,
    0x2252,
    0x52B5,
    0x4294,
    0x72F7,
    0x62D6,
    0x9339,
    0x8318,
    0xB37B,
    0xA35A,
    0xD3BD,
    0xC39C,
    0xF3FF,
    0xE3DE,
    0x2462,
    0x3443,
    0x0420,
    0x1401,
    0x64E6,
    0x74C7,
    0x44A4,
    0x5485,
    0xA56A,
    0xB54B,
    0x8528,
    0x9509,
    0xE5EE,
    0xF5CF,
    0xC5AC,
    0xD58D,
    0x3653,
    0x2672,
    0x1611,
    0x0630,
    0x76D7,
    0x66F6,
    0x5695,
    0x46B4,
    0xB75B,
    0xA77A,
    0x9719,
    0x8738,
    0xF7DF,
    0xE7FE,
    0xD79D,
    0xC7BC,
    0x48C4,
    0x58E5,
    0x6886,
    0x78A7,
    0x0840,
    0x1861,
    0x2802,
    0x3823,
    0xC9CC,
    0xD9ED,
    0xE98E,
    0xF9AF,
    0x8948,
    0x9969,
    0xA90A,
    0xB92B,
    0x5AF5,
    0x4AD4,
    0x7AB7,
    0x6A96,
    0x1A71,
    0x0A50,
    0x3A33,
    0x2A12,
    0xDBFD,
    0xCBDC,
    0xFBBF,
    0xEB9E,
    0x9B79,
    0x8B58,
    0xBB3B,
    0xAB1A,
    0x6CA6,
    0x7C87,
    0x4CE4,
    0x5CC5,
    0x2C22,
    0x3C03,
    0x0C60,
    0x1C41,
    0xEDAE,
    0xFD8F,
    0xCDEC,
    0xDDCD,
    0xAD2A,
    0xBD0B,
    0x8D68,
    0x9D49,
    0x7E97,
    0x6EB6,
    0x5ED5,
    0x4EF4,
    0x3E13,
    0x2E32,
    0x1E51,
    0x0E70,
    0xFF9F,
    0xEFBE,
    0xDFDD,
    0xCFFC,
    0xBF1B,
    0xAF3A,
    0x9F59,
    0x8F78,
    0x9188,
    0x81A9,
    0xB1CA,
    0xA1EB,
    0xD10C,
    0xC12D,
    0xF14E,
    0xE16F,
    0x1080,
    0x00A1,
    0x30C2,
    0x20E3,
    0x5004,
    0x4025,
    0x7046,
    0x6067,
    0x83B9,
    0x9398,
    0xA3FB,
    0xB3DA,
    0xC33D,
    0xD31C,
    0xE37F,
    0xF35E,
    0x02B1,
    0x1290,
    0x22F3,
    0x32D2,
    0x4235,
    0x5214,
    0x6277,
    0x7256,
    0xB5EA,
    0xA5CB,
    0x95A8,
    0x8589,
    0xF56E,
    0xE54F,
    0xD52C,
    0xC50D,
    0x34E2,
    0x24C3,
    0x14A0,
    0x0481,
    0x7466,
    0x6447,
    0x5424,
    0x4405,
    0xA7DB,
    0xB7FA,
    0x8799,
    0x97B8,
    0xE75F,
    0xF77E,
    0xC71D,
    0xD73C,
    0x26D3,
    0x36F2,
    0x0691,
    0x16B0,
    0x6657,
    0x7676,
    0x4615,
    0x5634,
    0xD94C,
    0xC96D,
    0xF90E,
    0xE92F,
    0x99C8,
    0x89E9,
    0xB98A,
    0xA9AB,
    0x5844,
    0x4865,
    0x7806,
    0x6827,
    0x18C0,
    0x08E1,
    0x3882,
    0x28A3,
    0xCB7D,
    0xDB5C,
    0xEB3F,
    0xFB1E,
    0x8BF9,
    0x9BD8,
    0xABBB,
    0xBB9A,
    0x4A75,
    0x5A54,
    0x6A37,
    0x7A16,
    0x0AF1,
    0x1AD0,
    0x2AB3,
    0x3A92,
    0xFD2E,
    0xED0F,
    0xDD6C,
    0xCD4D,
    0xBDAA,
    0xAD8B,
    0x9DE8,
    0x8DC9,
    0x7C26,
    0x6C07,
    0x5C64,
    0x4C45,
    0x3CA2,
    0x2C83,
    0x1CE0,
    0x0CC1,
    0xEF1F,
    0xFF3E,
    0xCF5D,
    0xDF7C,
    0xAF9B,
    0xBFBA,
    0x8FD9,
    0x9FF8,
    0x6E17,
    0x7E36,
    0x4E55,
    0x5E74,
    0x2E93,
    0x3EB2,
    0x0ED1,
    0x1EF0,
)


def crc16_ccitt_false(data):
    """Return CRC-16/CCITT-FALSE for a bytes-like object."""
    crc = 0xFFFF
    for byte in data:
        index = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ _CRC16_CCITT_FALSE_TABLE[index]) & 0xFFFF
    return crc


def _validate_seq(seq):
    if not isinstance(seq, int) or isinstance(seq, bool):
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


def _uint32_field(field, name, positive=False):
    if isinstance(field, bool):
        raise ValueError("%s must be uint32" % name)
    if isinstance(field, int):
        value = field
    elif isinstance(field, str) and field and field.isdigit():
        value = int(field)
    else:
        raise ValueError("%s must be uint32" % name)
    if value < 0 or value > UINT32_MAX or (positive and value == 0):
        raise ValueError("%s outside accepted uint32 range" % name)
    return value


def _int16_field(field, name):
    if isinstance(field, bool):
        raise ValueError("%s must be signed int16" % name)
    if isinstance(field, int):
        value = field
    elif isinstance(field, str):
        try:
            value = int(field)
        except ValueError:
            raise ValueError("%s must be signed int16" % name)
    else:
        raise ValueError("%s must be signed int16" % name)
    if value < -32768 or value > 32767:
        raise ValueError("%s outside signed int16 range" % name)
    return value


def _session_to_uint32(field, allow_zero):
    if not isinstance(field, str) or len(field) != 8:
        raise ValueError("session must contain exactly eight hex digits")
    for char in field:
        if not ("0" <= char <= "9" or "A" <= char <= "F"):
            raise ValueError("session must be uppercase hexadecimal")
    value = int(field, 16)
    if not allow_zero and value == 0:
        raise ValueError("session must be non-zero")
    return value


def _session_text(value):
    return "%08X" % value


def _encode_text_frame(seq, message_type, *fields):
    body_fields = [PROTOCOL_PREFIX, str(seq), message_type]
    body_fields.extend(_field_to_text(field) for field in fields)
    body = "|".join(body_fields).encode("ascii")
    crc = crc16_ccitt_false(body)
    frame = body + ("|%04X\n" % crc).encode("ascii")
    if len(frame) > MAX_FRAME_BYTES:
        raise FrameTooLongError("encoded frame exceeds maximum frame size")
    return frame


def encode_command_frame(seq, message_type, *fields):
    """Encode one Pi -> MCU binary command frame."""
    _validate_seq(seq)
    _validate_message_type(message_type)
    if message_type not in _COMMAND_IDS:
        raise ValueError("unsupported binary command %r" % message_type)

    command_id = _COMMAND_IDS[message_type]
    session = 0
    payload = b""

    if message_type == "HELLO":
        if len(fields) != 2:
            raise ValueError("HELLO requires client_minor and expected_profile_id")
        client_minor = _uint32_field(fields[0], "client_minor")
        profile_bytes = _field_to_text(fields[1]).encode("ascii")
        payload = struct.pack("<I", client_minor) + profile_bytes

    elif message_type == "HEARTBEAT":
        if len(fields) != 2:
            raise ValueError("HEARTBEAT requires session and host_uptime_ms")
        session = _session_to_uint32(fields[0], False)
        payload = struct.pack("<I", _uint32_field(fields[1], "host_uptime_ms"))

    elif message_type == "STAGE":
        if len(fields) != 19:
            raise ValueError("STAGE requires session and 18 joints")
        session = _session_to_uint32(fields[0], False)
        joints = []
        for index, value in enumerate(fields[1:]):
            joints.append(_int16_field(value, "joint[%d]" % index))
        payload = struct.pack("<18h", *joints)

    elif message_type in _SIMPLE_SESSION_COMMANDS:
        if len(fields) != 1:
            raise ValueError("%s requires exactly one session field" % message_type)
        session = _session_to_uint32(
            fields[0],
            message_type == "GET_STATUS",
        )

    elif message_type == "TARGET":
        if len(fields) != 20:
            raise ValueError("TARGET requires session, period_ms, and 18 joints")
        session = _session_to_uint32(fields[0], False)
        period_ms = _uint32_field(fields[1], "period_ms", positive=True)
        joints = []
        for index, value in enumerate(fields[2:]):
            joints.append(_int16_field(value, "joint[%d]" % index))
        payload = struct.pack("<I18h", period_ms, *joints)

    elif message_type == "ESTOP":
        if len(fields) != 2:
            raise ValueError("ESTOP requires session and reason")
        session = _session_to_uint32(fields[0], True)
        payload = _field_to_text(fields[1]).encode("ascii")

    if len(payload) > MAX_COMMAND_PAYLOAD_BYTES:
        raise FrameTooLongError("binary command payload exceeds maximum size")

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
    return body + struct.pack("<H", crc16_ccitt_false(body))


def encode_frame(seq, message_type, *fields):
    """Encode commands as binary and telemetry/responses as ASCII HX1."""
    _validate_seq(seq)
    _validate_message_type(message_type)
    if message_type in _COMMAND_TYPES:
        return encode_command_frame(seq, message_type, *fields)
    return _encode_text_frame(seq, message_type, *fields)


def _normalize_text_frame(raw):
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


def _parse_text_frame(raw):
    raw = _normalize_text_frame(raw)
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
    body = "|".join(parts[:-1]).encode("ascii")
    if int(crc_text, 16) != crc16_ccitt_false(body):
        raise CRCError("CRC mismatch")
    return seq, message_type, fields


def _command_header(raw):
    if not isinstance(raw, (bytes, bytearray)):
        raise FrameFormatError("binary command frame must be bytes or bytearray")
    if not isinstance(raw, bytes):
        raw = bytes(raw)
    minimum = COMMAND_HEADER_BYTES + COMMAND_CRC_BYTES
    if len(raw) < minimum:
        raise FrameFormatError("binary command frame is too short")
    if len(raw) > MAX_COMMAND_FRAME_BYTES:
        raise FrameTooLongError("binary command frame exceeds maximum size")
    try:
        magic, version, command_id, payload_len, seq, session = struct.unpack_from(
            COMMAND_HEADER_FORMAT, raw, 0
        )
    except Exception:
        raise FrameFormatError("binary command header is malformed")
    if magic != COMMAND_MAGIC:
        raise FrameFormatError("binary command magic mismatch")
    if version != COMMAND_WIRE_VERSION:
        raise FrameFormatError("unsupported binary command wire version")
    if payload_len > MAX_COMMAND_PAYLOAD_BYTES:
        raise FrameTooLongError("binary command payload exceeds maximum size")
    expected_len = COMMAND_HEADER_BYTES + payload_len + COMMAND_CRC_BYTES
    if len(raw) != expected_len:
        raise FrameFormatError("binary command frame length mismatch")
    received_crc = struct.unpack_from("<H", raw, expected_len - 2)[0]
    if received_crc != crc16_ccitt_false(raw[:-2]):
        raise CRCError("CRC mismatch")
    return raw, command_id, payload_len, seq, session


def parse_command_frame(raw):
    """Fast MCU command decoder returning native ints for joint vectors."""
    raw, command_id, payload_len, seq, session = _command_header(raw)
    message_type = _COMMAND_NAMES.get(command_id, "CMD_%d" % command_id)
    payload_offset = COMMAND_HEADER_BYTES
    session_text = _session_text(session)

    if message_type == "HELLO":
        if session != 0 or payload_len < 5:
            raise FrameFormatError("HELLO binary payload is malformed")
        client_minor = struct.unpack_from("<I", raw, payload_offset)[0]
        try:
            profile_id = raw[payload_offset + 4 : -2].decode("ascii")
        except UnicodeDecodeError:
            raise FrameFormatError("HELLO profile id must be ASCII")
        if not profile_id:
            raise FrameFormatError("HELLO profile id must not be empty")
        return seq, message_type, (client_minor, profile_id)

    if message_type == "HEARTBEAT":
        if payload_len != 4:
            raise FrameFormatError("HEARTBEAT binary payload has wrong length")
        uptime = struct.unpack_from("<I", raw, payload_offset)[0]
        return seq, message_type, (session_text, uptime)

    if message_type == "STAGE":
        if payload_len != 36:
            raise FrameFormatError("STAGE binary payload has wrong length")
        joints = struct.unpack_from("<18h", raw, payload_offset)
        return seq, message_type, (session_text,) + joints

    if message_type in _SIMPLE_SESSION_COMMANDS:
        if payload_len != 0:
            raise FrameFormatError("%s binary payload must be empty" % message_type)
        return seq, message_type, (session_text,)

    if message_type == "TARGET":
        if payload_len != 40:
            raise FrameFormatError("TARGET binary payload has wrong length")
        values = struct.unpack_from("<I18h", raw, payload_offset)
        return seq, message_type, (session_text, values[0]) + values[1:]

    if message_type == "ESTOP":
        if payload_len < 1:
            raise FrameFormatError("ESTOP reason must not be empty")
        try:
            reason = raw[payload_offset:-2].decode("ascii")
        except UnicodeDecodeError:
            raise FrameFormatError("ESTOP reason must be ASCII")
        return seq, message_type, (session_text, reason)

    return seq, message_type, (session_text,)


def _looks_binary(raw):
    return (
        isinstance(raw, (bytes, bytearray))
        and len(raw) >= 2
        and bytes(raw[:2]) == COMMAND_MAGIC
        and not bytes(raw).startswith(b"HX1|")
    )


def parse_frame(raw):
    """Compatibility parser for tests/diagnostics; returns string fields."""
    if not _looks_binary(raw):
        return _parse_text_frame(raw)
    seq, message_type, fields = parse_command_frame(raw)
    return seq, message_type, tuple(str(value) for value in fields)


def sequence_is_newer(candidate, previous):
    _validate_seq(candidate)
    _validate_seq(previous)
    delta = (candidate - previous) & UINT32_MAX
    return 0 < delta < UINT32_HALF_RANGE


class LineFramer:
    """Bounded stream framer for binary commands and ASCII diagnostics.

    Production Pi -> MCU traffic is binary. ASCII framing remains recognized so
    the pure protocol/test helpers retain their existing newline behavior; the
    production runtime still calls ``parse_command_frame`` and therefore accepts
    only binary commands.
    """

    def __init__(self):
        self._buffer = bytearray()
        self._discarding_ascii = False
        self.framing_errors = 0

    def reset(self):
        self._buffer = bytearray()
        self._discarding_ascii = False
        self.framing_errors = 0

    def discard_current_line(self):
        """Discard an explicitly aborted ASCII line through its next newline."""
        self._buffer = bytearray()
        self._discarding_ascii = True
        self.framing_errors += 1

    def _discard_garbage_prefix(self):
        marker = self._buffer.find(COMMAND_MAGIC)
        newline = self._buffer.find(b"\n")

        if newline >= 0 and (marker < 0 or newline < marker):
            self._buffer = self._buffer[newline + 1 :]
            self.framing_errors += 1
            return True

        if marker > 0:
            self._buffer = self._buffer[marker:]
            self.framing_errors += 1
            return True

        if marker < 0:
            keep_h = bool(self._buffer) and self._buffer[-1] == COMMAND_MAGIC[0]
            if len(self._buffer) > (1 if keep_h else 0):
                self.framing_errors += 1
            self._buffer = bytearray(b"H" if keep_h else b"")
            return False

        return True

    def feed(self, data):
        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("LineFramer.feed expects bytes or bytearray")
        if not data:
            return []

        if self._discarding_ascii:
            newline = data.find(b"\n")
            if newline < 0:
                return []
            self._discarding_ascii = False
            data = data[newline + 1 :]
            if not data:
                return []

        self._buffer.extend(data)
        frames = []

        while self._buffer:
            if len(self._buffer) < 2:
                break

            if bytes(self._buffer[:2]) != COMMAND_MAGIC:
                if not self._discard_garbage_prefix():
                    break
                continue

            # ASCII HX1 telemetry/test frame. Production runtime will reject it
            # as a command, but retaining this framer mode preserves diagnostics.
            if len(self._buffer) >= 4 and bytes(self._buffer[:4]) == b"HX1|":
                newline = self._buffer.find(b"\n")
                if newline < 0:
                    if len(self._buffer) > MAX_FRAME_BYTES:
                        self._buffer = bytearray()
                        self._discarding_ascii = True
                        self.framing_errors += 1
                    break
                end = newline + 1
                if end > MAX_FRAME_BYTES:
                    self._buffer = self._buffer[end:]
                    self.framing_errors += 1
                    continue
                frames.append(bytes(self._buffer[:end]))
                self._buffer = self._buffer[end:]
                continue

            if len(self._buffer) < 3:
                break

            # A third byte of ASCII '1' may be a fragmented HX1| prefix.
            if self._buffer[2] == ord("1"):
                if len(self._buffer) < 4:
                    break
                self._buffer = self._buffer[1:]
                self.framing_errors += 1
                continue

            # Binary command path.
            if self._buffer[2] != COMMAND_WIRE_VERSION:
                self._buffer = self._buffer[1:]
                self.framing_errors += 1
                continue

            if len(self._buffer) < COMMAND_HEADER_BYTES:
                break

            try:
                magic, version, _command_id, payload_len, _seq, _session = (
                    struct.unpack_from(COMMAND_HEADER_FORMAT, self._buffer, 0)
                )
            except Exception:
                self._buffer = self._buffer[1:]
                self.framing_errors += 1
                continue

            if (
                magic != COMMAND_MAGIC
                or version != COMMAND_WIRE_VERSION
                or payload_len > MAX_COMMAND_PAYLOAD_BYTES
            ):
                self._buffer = self._buffer[1:]
                self.framing_errors += 1
                continue

            fixed_payload_len = _FIXED_PAYLOAD_LENGTHS.get(_command_id)
            if fixed_payload_len is not None and payload_len != fixed_payload_len:
                self._buffer = self._buffer[1:]
                self.framing_errors += 1
                continue

            frame_len = COMMAND_HEADER_BYTES + payload_len + COMMAND_CRC_BYTES
            if len(self._buffer) < frame_len:
                break

            frames.append(bytes(self._buffer[:frame_len]))
            self._buffer = self._buffer[frame_len:]

        return frames


# Explicit name for tests/documentation; transport keeps importing LineFramer.
CommandFramer = LineFramer
