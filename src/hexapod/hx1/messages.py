"""Typed HX1 commands and Servo 2040 telemetry messages."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TypeAlias

from .protocol import HX1Frame, encode_frame, UINT32_MAX

JOINT_COUNT = 18
ZERO_SESSION = "00000000"

VALID_STATES = frozenset(
    {
        "BOOTING",
        "SELF_TEST",
        "DISARMED",
        "ARMED",
        "ACTIVE",
        "FAULT",
        "ESTOP",
    }
)

VALID_ERRORS = frozenset(
    {
        "ERR_BAD_SESSION",
        "ERR_BAD_STATE",
        "ERR_BAD_ARG_COUNT",
        "ERR_BAD_VALUE",
        "ERR_LIMIT",
        "ERR_RATE_LIMIT",
        "ERR_PROFILE_MISMATCH",
        "ERR_NOT_READY",
        "ERR_SEQ",
        "ERR_ESTOP",
        "ERR_FAULT_ACTIVE",
        "ERR_STAGE_EXPIRED",
        "ERR_UNSUPPORTED",
    }
)


class HX1MessageError(ValueError):
    """HX1 message fields are malformed or semantically invalid."""


@dataclass(frozen=True, slots=True)
class HX1Outbound:
    seq: int
    command: str
    frame: bytes


@dataclass(frozen=True, slots=True)
class HX1Info:
    seq: int
    ref_seq: int
    session: str
    server_minor: int
    firmware_version: str
    mcu_id: str
    profile_id: str
    profile_revision: int
    profile_hash: str
    joint_count: int
    capabilities: int
    state: str


@dataclass(frozen=True, slots=True)
class HX1Ack:
    seq: int
    ref_seq: int
    command: str


@dataclass(frozen=True, slots=True)
class HX1Nack:
    seq: int
    ref_seq: int
    command: str
    error: str


@dataclass(frozen=True, slots=True)
class HX1Event:
    seq: int
    event_type: str
    detail: str


@dataclass(frozen=True, slots=True)
class HX1Status:
    seq: int
    session: str
    state: str
    fault: str
    last_target_seq: int | None
    target_age_ms: int | None
    heartbeat_age_ms: int | None
    foot_mask: int
    bus_mv: int | None
    bus_ma: int | None
    uptime_ms: int
    command_valid: bool
    commanded_joint_cd: tuple[int, ...] | None


@dataclass(frozen=True, slots=True)
class HX1UnknownMessage:
    seq: int
    message_type: str
    fields: tuple[str, ...]


HX1Inbound: TypeAlias = (
    HX1Info
    | HX1Ack
    | HX1Nack
    | HX1Event
    | HX1Status
    | HX1UnknownMessage
)


def _uint32(text: str, name: str) -> int:
    if not text or not text.isdigit():
        raise HX1MessageError(f"{name} must be unsigned decimal")
    value = int(text)
    if value < 0 or value > UINT32_MAX:
        raise HX1MessageError(f"{name} outside uint32 range")
    return value


def _positive_int(text: str, name: str) -> int:
    value = _uint32(text, name)
    if value <= 0:
        raise HX1MessageError(f"{name} must be > 0")
    return value


def _signed_int(text: str, name: str) -> int:
    if not text:
        raise HX1MessageError(f"{name} must be decimal")
    start = 1 if text[0] == "-" else 0
    if start == len(text) or not text[start:].isdigit():
        raise HX1MessageError(f"{name} must be decimal")
    return int(text)


def _nonnegative_int(text: str, name: str) -> int:
    value = _signed_int(text, name)
    if value < 0:
        raise HX1MessageError(f"{name} must be non-negative")
    return value


def _age(text: str, name: str) -> int | None:
    value = _signed_int(text, name)
    if value == -1:
        return None
    if value < 0:
        raise HX1MessageError(f"{name} must be -1 or non-negative")
    return value


def _optional_positive_wire_value(
    text: str,
    name: str,
) -> int | None:
    value = _signed_int(text, name)
    if value == -1:
        return None
    return value


def _session(text: str, *, allow_zero: bool = True) -> str:
    if len(text) != 8:
        raise HX1MessageError(
            "session must contain exactly eight hex digits"
        )
    if any(
        not ("0" <= c <= "9" or "A" <= c <= "F")
        for c in text
    ):
        raise HX1MessageError(
            "session must be uppercase hexadecimal"
        )
    if not allow_zero and text == ZERO_SESSION:
        raise HX1MessageError("session must be non-zero")
    return text


def _hex32(text: str, name: str) -> int:
    if len(text) != 8 or any(
        not ("0" <= c <= "9" or "A" <= c <= "F")
        for c in text
    ):
        raise HX1MessageError(
            f"{name} must contain eight uppercase hex digits"
        )
    return int(text, 16)


def _token(text: str, name: str) -> str:
    if not text:
        raise HX1MessageError(f"{name} must be non-empty")
    allowed = (
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789"
        "._-:"
    )
    if any(c not in allowed for c in text):
        raise HX1MessageError(
            f"{name} contains an invalid character"
        )
    return text


def _joint_vector_cd(
    values: object,
) -> tuple[int, ...]:
    if (
        isinstance(values, (str, bytes))
        or not isinstance(values, (tuple, list))
        or len(values) != JOINT_COUNT
    ):
        raise HX1MessageError(
            f"joint vector must contain exactly {JOINT_COUNT} values"
        )

    out: list[int] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, int):
            raise HX1MessageError(
                f"joint[{index}] must be an integer centidegree value"
            )
        out.append(value)
    return tuple(out)


def degrees_to_centidegrees(value_deg: object) -> int:
    """Quantize degrees to nearest centidegree, ties away from zero."""
    if isinstance(value_deg, bool):
        raise HX1MessageError("joint angle must be a finite number")
    try:
        value = float(value_deg)
    except (TypeError, ValueError) as exc:
        raise HX1MessageError(
            "joint angle must be a finite number"
        ) from exc
    if not math.isfinite(value):
        raise HX1MessageError("joint angle must be a finite number")

    scaled = value * 100.0
    if scaled >= 0.0:
        return math.floor(scaled + 0.5)
    return math.ceil(scaled - 0.5)


def joint_degrees_to_centidegrees(
    values_deg: object,
) -> tuple[int, ...]:
    if (
        isinstance(values_deg, (str, bytes))
        or not isinstance(values_deg, (tuple, list))
        or len(values_deg) != JOINT_COUNT
    ):
        raise HX1MessageError(
            f"joint vector must contain exactly {JOINT_COUNT} values"
        )
    return tuple(
        degrees_to_centidegrees(value)
        for value in values_deg
    )


def build_hello(
    seq: int,
    client_minor: int,
    expected_profile_id: str,
) -> HX1Outbound:
    return HX1Outbound(
        seq=seq,
        command="HELLO",
        frame=encode_frame(
            seq,
            "HELLO",
            client_minor,
            expected_profile_id,
        ),
    )


def build_heartbeat(
    seq: int,
    session: str,
    host_uptime_ms: int,
) -> HX1Outbound:
    _session(session, allow_zero=False)
    if (
        isinstance(host_uptime_ms, bool)
        or not isinstance(host_uptime_ms, int)
        or host_uptime_ms < 0
        or host_uptime_ms > UINT32_MAX
    ):
        raise HX1MessageError(
            "host_uptime_ms must be uint32"
        )
    return HX1Outbound(
        seq=seq,
        command="HEARTBEAT",
        frame=encode_frame(
            seq,
            "HEARTBEAT",
            session,
            host_uptime_ms,
        ),
    )


def build_stage(
    seq: int,
    session: str,
    joint_cd: object,
) -> HX1Outbound:
    _session(session, allow_zero=False)
    joints = _joint_vector_cd(joint_cd)
    return HX1Outbound(
        seq=seq,
        command="STAGE",
        frame=encode_frame(seq, "STAGE", session, *joints),
    )


def build_simple_session_command(
    seq: int,
    command: str,
    session: str,
) -> HX1Outbound:
    if command not in {
        "ARM",
        "START",
        "STOP",
        "DISARM",
        "CLEAR_ESTOP",
        "CLEAR_FAULT",
        "GET_STATUS",
    }:
        raise HX1MessageError(
            f"unsupported simple session command {command!r}"
        )
    _session(session, allow_zero=False)
    return HX1Outbound(
        seq=seq,
        command=command,
        frame=encode_frame(seq, command, session),
    )


def build_target(
    seq: int,
    session: str,
    period_ms: int,
    joint_cd: object,
) -> HX1Outbound:
    _session(session, allow_zero=False)
    if (
        isinstance(period_ms, bool)
        or not isinstance(period_ms, int)
        or period_ms <= 0
        or period_ms > UINT32_MAX
    ):
        raise HX1MessageError(
            "period_ms must be a positive uint32 integer"
        )
    joints = _joint_vector_cd(joint_cd)
    return HX1Outbound(
        seq=seq,
        command="TARGET",
        frame=encode_frame(
            seq,
            "TARGET",
            session,
            period_ms,
            *joints,
        ),
    )


def build_estop(
    seq: int,
    session_or_zero: str,
    reason: str,
) -> HX1Outbound:
    _session(session_or_zero, allow_zero=True)
    _token(reason, "reason")
    return HX1Outbound(
        seq=seq,
        command="ESTOP",
        frame=encode_frame(
            seq,
            "ESTOP",
            session_or_zero,
            reason,
        ),
    )


def parse_inbound(frame: HX1Frame) -> HX1Inbound:
    fields = frame.fields
    message_type = frame.message_type

    if message_type == "INFO":
        if len(fields) != 11:
            raise HX1MessageError(
                "INFO must contain exactly 11 fields"
            )
        state = _token(fields[10], "state")
        if state not in VALID_STATES:
            raise HX1MessageError(f"unknown MCU state {state!r}")
        return HX1Info(
            seq=frame.seq,
            ref_seq=_uint32(fields[0], "ref_seq"),
            session=_session(fields[1], allow_zero=False),
            server_minor=_uint32(fields[2], "server_minor"),
            firmware_version=_token(
                fields[3],
                "firmware_version",
            ),
            mcu_id=_token(fields[4], "mcu_id"),
            profile_id=_token(fields[5], "profile_id"),
            profile_revision=_positive_int(
                fields[6],
                "profile_revision",
            ),
            profile_hash=_token(
                fields[7],
                "profile_hash",
            ),
            joint_count=_positive_int(
                fields[8],
                "joint_count",
            ),
            capabilities=_hex32(fields[9], "capabilities"),
            state=state,
        )

    if message_type == "ACK":
        if len(fields) != 2:
            raise HX1MessageError(
                "ACK must contain exactly 2 fields"
            )
        return HX1Ack(
            seq=frame.seq,
            ref_seq=_uint32(fields[0], "ref_seq"),
            command=_token(fields[1], "command").upper(),
        )

    if message_type == "NACK":
        if len(fields) != 3:
            raise HX1MessageError(
                "NACK must contain exactly 3 fields"
            )
        error = _token(fields[2], "error").upper()
        if error not in VALID_ERRORS:
            # Forward-compatible diagnostic parsing: unknown future NACK
            # identifiers remain visible instead of corrupting the stream.
            error = error
        return HX1Nack(
            seq=frame.seq,
            ref_seq=_uint32(fields[0], "ref_seq"),
            command=_token(fields[1], "command").upper(),
            error=error,
        )

    if message_type == "EVENT":
        if len(fields) != 2:
            raise HX1MessageError(
                "EVENT must contain exactly 2 fields"
            )
        return HX1Event(
            seq=frame.seq,
            event_type=_token(
                fields[0],
                "event_type",
            ).upper(),
            detail=_token(fields[1], "detail"),
        )

    if message_type == "STATUS":
        if len(fields) != 11 + JOINT_COUNT:
            raise HX1MessageError(
                f"STATUS must contain exactly "
                f"{11 + JOINT_COUNT} fields"
            )

        state = _token(fields[1], "state")
        if state not in VALID_STATES:
            raise HX1MessageError(f"unknown MCU state {state!r}")

        fault = _token(fields[2], "fault").upper()

        last_target_raw = _signed_int(
            fields[3],
            "last_target_seq",
        )
        if last_target_raw == -1:
            last_target_seq = None
        elif 0 <= last_target_raw <= UINT32_MAX:
            last_target_seq = last_target_raw
        else:
            raise HX1MessageError(
                "last_target_seq must be -1 or uint32"
            )

        command_valid_raw = _signed_int(
            fields[10],
            "command_valid",
        )
        if command_valid_raw not in (0, 1):
            raise HX1MessageError(
                "command_valid must be 0 or 1"
            )

        joints = tuple(
            _signed_int(
                text,
                f"joint[{index}]",
            )
            for index, text in enumerate(fields[11:])
        )

        return HX1Status(
            seq=frame.seq,
            session=_session(fields[0], allow_zero=True),
            state=state,
            fault=fault,
            last_target_seq=last_target_seq,
            target_age_ms=_age(fields[4], "target_age_ms"),
            heartbeat_age_ms=_age(
                fields[5],
                "heartbeat_age_ms",
            ),
            foot_mask=_uint32(fields[6], "foot_mask"),
            bus_mv=_optional_positive_wire_value(
                fields[7],
                "bus_mv",
            ),
            bus_ma=_optional_positive_wire_value(
                fields[8],
                "bus_ma",
            ),
            uptime_ms=_nonnegative_int(
                fields[9],
                "uptime_ms",
            ),
            command_valid=bool(command_valid_raw),
            commanded_joint_cd=(
                joints if command_valid_raw == 1 else None
            ),
        )

    return HX1UnknownMessage(
        seq=frame.seq,
        message_type=message_type,
        fields=fields,
    )
