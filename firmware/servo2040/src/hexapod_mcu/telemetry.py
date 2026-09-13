"""Outbound HX1 messages for the Servo 2040 runtime.

This module formats MCU -> Pi protocol frames. It owns no serial I/O and makes
no safety-state decisions.

Protocol minor 1 adds COMMAND_VALID to STATUS. When COMMAND_VALID is 0, the
J0..J17 values are placeholders and must not be interpreted as a commanded
pose. When it is 1, the vector is the last meaningful commanded logical target.

The joint vector always represents commanded state, never measured position.
"""

from .constants import JOINT_COUNT, PROTOCOL_MINOR, UINT32_MAX
from .protocol import encode_frame


_TOKEN_CHARS = (
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "._-:"
)


class TelemetryError(ValueError):
    """Outbound message data is invalid."""


def _require_uint32(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TelemetryError("%s must be an unsigned 32-bit integer" % name)
    if value < 0 or value > UINT32_MAX:
        raise TelemetryError("%s must be an unsigned 32-bit integer" % name)
    return value


def _hex32(value, name):
    value = _require_uint32(value, name)
    return "%08X" % value


def _require_nonnegative_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise TelemetryError("%s must be a non-negative integer" % name)
    return value


def _require_int(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TelemetryError("%s must be an integer" % name)
    return value


def _token(value, name, uppercase=False):
    if not isinstance(value, str) or not value:
        raise TelemetryError("%s must be a non-empty string" % name)

    for char in value:
        if char not in _TOKEN_CHARS:
            raise TelemetryError("%s contains an invalid character" % name)

    return value.upper() if uppercase else value


def _session_text(session_id):
    if session_id is None:
        return "00000000"

    if not isinstance(session_id, str) or len(session_id) != 8:
        raise TelemetryError("session_id must contain eight hex digits")

    for char in session_id:
        if not ("0" <= char <= "9" or "A" <= char <= "F"):
            raise TelemetryError(
                "session_id must contain uppercase hexadecimal digits"
            )

    return session_id


class TelemetryEncoder:
    """Deterministic MCU outbound-message encoder with its own sequence space."""

    def __init__(
        self,
        profile,
        firmware_version,
        mcu_id,
        capabilities=0,
        server_minor=PROTOCOL_MINOR,
        initial_seq=0,
    ):
        self._profile = profile
        self._firmware_version = _token(
            firmware_version,
            "firmware_version",
        )
        self._mcu_id = _token(mcu_id, "mcu_id")
        self._capabilities = _require_uint32(
            capabilities,
            "capabilities",
        )

        if (
            isinstance(server_minor, bool)
            or not isinstance(server_minor, int)
            or server_minor < 0
        ):
            raise TelemetryError("server_minor must be a non-negative integer")
        self._server_minor = server_minor

        self._next_seq = _require_uint32(initial_seq, "initial_seq")

    @property
    def next_seq(self):
        return self._next_seq

    @property
    def capabilities(self):
        return self._capabilities

    def info(self, ref_seq, session_id, state):
        """Build INFO for a successfully handled HELLO."""
        _require_uint32(ref_seq, "ref_seq")
        session = _session_text(session_id)
        if session == "00000000":
            raise TelemetryError("INFO requires a non-zero session")

        state = _token(state, "state", uppercase=True)

        return self._emit(
            "INFO",
            ref_seq,
            session,
            self._server_minor,
            self._firmware_version,
            self._mcu_id,
            self._profile.profile_id,
            self._profile.profile_revision,
            self._profile.profile_hash,
            self._profile.joint_count,
            _hex32(self._capabilities, "capabilities"),
            state,
        )

    def ack(self, ref_seq, command):
        _require_uint32(ref_seq, "ref_seq")
        command = _token(command, "command", uppercase=True)
        return self._emit("ACK", ref_seq, command)

    def nack(self, ref_seq, command, error):
        _require_uint32(ref_seq, "ref_seq")
        command = _token(command, "command", uppercase=True)
        error = _token(error, "error", uppercase=True)
        return self._emit("NACK", ref_seq, command, error)

    def event(self, event_type, detail):
        event_type = _token(event_type, "event_type", uppercase=True)
        detail = _token(detail, "detail")
        return self._emit("EVENT", event_type, detail)

    def status(
        self,
        runtime,
        uptime_ms,
        foot_mask=0,
        bus_mv=-1,
        bus_ma=-1,
    ):
        """Build a STATUS frame from authoritative runtime state."""
        uptime_ms = _require_nonnegative_int(uptime_ms, "uptime_ms")
        foot_mask = _require_uint32(foot_mask, "foot_mask")
        bus_mv = _require_int(bus_mv, "bus_mv")
        bus_ma = _require_int(bus_ma, "bus_ma")

        session = _session_text(runtime.session_id)
        state = _token(runtime.state, "state", uppercase=True)
        fault = _token(runtime.fault, "fault", uppercase=True)

        if runtime.last_target_seq is None:
            last_target_seq = -1
        else:
            last_target_seq = _require_uint32(
                runtime.last_target_seq,
                "last_target_seq",
            )

        target_age_ms = runtime.target_age_ms(uptime_ms)
        heartbeat_age_ms = runtime.heartbeat_age_ms(uptime_ms)

        if runtime.commanded_target is None:
            command_valid = 0
            commanded = (0,) * JOINT_COUNT
        else:
            command_valid = 1
            commanded = tuple(runtime.commanded_target)

            if len(commanded) != JOINT_COUNT:
                raise TelemetryError(
                    "runtime commanded target has wrong joint count"
                )

            for index, value in enumerate(commanded):
                _require_int(value, "commanded[%d]" % index)

        return self._emit(
            "STATUS",
            session,
            state,
            fault,
            last_target_seq,
            target_age_ms,
            heartbeat_age_ms,
            foot_mask,
            bus_mv,
            bus_ma,
            uptime_ms,
            command_valid,
            *commanded
        )

    def _emit(self, message_type, *fields):
        seq = self._next_seq
        frame = encode_frame(seq, message_type, *fields)
        self._next_seq = (seq + 1) & UINT32_MAX
        return frame
