"""HX1 command coordinator for the Servo 2040 runtime.

This module is the integration layer between:

* protocol framing/parsing;
* the authoritative runtime state machine;
* actuator position/rate validation;
* the physical hardware adapter; and
* outbound telemetry encoding.

It deliberately owns no gait, IK, navigation, or robot-level trajectory logic.

The coordinator is transport-independent: ``handle_frame()`` accepts one
complete raw HX1 frame and returns zero or more already-encoded response frames.
The main loop owns USB polling and sending those responses.

All command handling is single-threaded and deterministic.
"""

from .actuators import (
    PositionLimitError,
    RateLimitError,
    TargetShapeError,
    UnqualifiedRateError,
    logical_to_channel_vector,
    validate_position_target,
    validate_rate_transition,
)
from .constants import JOINT_COUNT, LINK_TIMEOUT_MS, UINT32_MAX
from .hardware import HardwareCompatibilityError, HardwareError
from .profile import ProfileQualificationError
from .protocol import ProtocolError, parse_frame, sequence_is_newer
from .state_machine import Error, Fault, State
from .watchdogs import age_ms, ticks_diff


class RuntimeErrorInternal(RuntimeError):
    """An invariant failed inside the MCU runtime coordinator."""


def _parse_uint32(text, name):
    if not isinstance(text, str) or not text or not text.isdigit():
        raise ValueError("%s must be unsigned decimal" % name)

    value = int(text)
    if value < 0 or value > UINT32_MAX:
        raise ValueError("%s outside uint32 range" % name)

    return value


def _parse_positive_int(text, name):
    value = _parse_uint32(text, name)
    if value <= 0:
        raise ValueError("%s must be > 0" % name)
    return value


def _parse_signed_int(text, name):
    if not isinstance(text, str) or not text:
        raise ValueError("%s must be decimal" % name)

    start = 1 if text[0] == "-" else 0
    if start == len(text) or not text[start:].isdigit():
        raise ValueError("%s must be decimal" % name)

    return int(text)


def _parse_session(text):
    if not isinstance(text, str) or len(text) != 8:
        raise ValueError("session must contain eight hex digits")

    for char in text:
        if not ("0" <= char <= "9" or "A" <= char <= "F"):
            raise ValueError("session must be uppercase hexadecimal")

    return text


_TOKEN_CHARS = (
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "._-:"
)


def _parse_token(text, name):
    if not isinstance(text, str) or not text:
        raise ValueError("%s must be a non-empty token" % name)

    for char in text:
        if char not in _TOKEN_CHARS:
            raise ValueError("%s contains an invalid character" % name)

    return text


def _parse_joint_vector(fields):
    if len(fields) != JOINT_COUNT:
        raise ValueError("joint vector has wrong length")

    out = []
    for index, text in enumerate(fields):
        out.append(_parse_signed_int(text, "joint[%d]" % index))

    return tuple(out)


class RuntimeCoordinator:
    """Deterministic HX1 command coordinator."""

    def __init__(
        self,
        profile,
        state_machine,
        hardware,
        telemetry,
        session_factory,
    ):
        self.profile = profile
        self.state_machine = state_machine
        self.hardware = hardware
        self.telemetry = telemetry
        self.session_factory = session_factory

        self.protocol_errors = 0
        self.semantic_errors = 0
        self.internal_errors = 0

    def perform_self_test(self, now_ms):
        sm = self.state_machine

        if sm.state != State.BOOTING:
            raise RuntimeErrorInternal(
                "self-test may begin only from BOOTING"
            )

        begin = sm.begin_self_test()
        if not begin:
            raise RuntimeErrorInternal("failed to enter SELF_TEST")

        try:
            self.hardware.force_disabled()
        except HardwareError:
            sm.complete_self_test(
                False,
                now_ms=now_ms,
                fault_code=Fault.HARDWARE,
            )
            return False

        try:
            self.profile.require_arm_qualified()
            self.hardware.require_profile_compatible(self.profile)
        except ProfileQualificationError:
            sm.complete_self_test(
                False,
                now_ms=now_ms,
                fault_code=Fault.PROFILE,
            )
            return False
        except HardwareCompatibilityError:
            sm.complete_self_test(
                False,
                now_ms=now_ms,
                fault_code=Fault.PROFILE,
            )
            return False
        except HardwareError:
            sm.complete_self_test(
                False,
                now_ms=now_ms,
                fault_code=Fault.HARDWARE,
            )
            return False
        except Exception:
            sm.complete_self_test(
                False,
                now_ms=now_ms,
                fault_code=Fault.INTERNAL,
            )
            return False

        sm.complete_self_test(True, now_ms=now_ms)
        return True

    def handle_frame(self, raw_frame, now_ms):
        try:
            seq, message_type, fields = parse_frame(raw_frame)
        except ProtocolError:
            self.protocol_errors += 1
            return ()

        pre_responses = ()

        # Commands that explicitly remove or replace authority are allowed to
        # execute immediately. Everything else must first observe a watchdog
        # deadline that may already have expired. This prevents a late
        # HEARTBEAT or TARGET from retroactively rescuing stale authority.
        if message_type not in ("ESTOP", "DISARM", "HELLO"):
            try:
                pre_responses = tuple(self.tick(now_ms))
            except Exception:
                self.internal_errors += 1
                return self._safe_internal_failure(
                    seq,
                    message_type,
                    now_ms,
                )

        try:
            responses = tuple(
                self._dispatch(
                    seq,
                    message_type,
                    fields,
                    now_ms,
                )
            )
            return pre_responses + responses
        except Exception:
            self.internal_errors += 1
            return pre_responses + self._safe_internal_failure(
                seq,
                message_type,
                now_ms,
            )

    def _dispatch(self, seq, message_type, fields, now_ms):
        if message_type == "HELLO":
            return self._hello(seq, fields, now_ms)
        if message_type == "HEARTBEAT":
            return self._heartbeat(seq, fields, now_ms)
        if message_type == "STAGE":
            return self._stage(seq, fields, now_ms)
        if message_type == "ARM":
            return self._arm(seq, fields, now_ms)
        if message_type == "START":
            return self._start(seq, fields, now_ms)
        if message_type == "TARGET":
            return self._target(seq, fields, now_ms)
        if message_type == "STOP":
            return self._stop(seq, fields, now_ms)
        if message_type == "DISARM":
            return self._disarm(seq, fields, now_ms)
        if message_type == "ESTOP":
            return self._estop(seq, fields, now_ms)
        if message_type == "CLEAR_ESTOP":
            return self._clear_estop(seq, fields, now_ms)
        if message_type == "CLEAR_FAULT":
            return self._clear_fault(seq, fields, now_ms)
        if message_type == "GET_STATUS":
            return self._get_status(seq, fields, now_ms)

        return (
            self.telemetry.nack(
                seq,
                message_type,
                Error.UNSUPPORTED,
            ),
        )

    def _hello(self, seq, fields, now_ms):
        if len(fields) != 2:
            return self._nack(seq, "HELLO", Error.BAD_ARG_COUNT)

        try:
            _parse_uint32(fields[0], "client_minor")
        except ValueError:
            return self._nack(seq, "HELLO", Error.BAD_VALUE)

        expected_profile_id = fields[1]
        if not expected_profile_id:
            return self._nack(seq, "HELLO", Error.BAD_VALUE)

        try:
            session_id = self._new_session_id()
        except Exception:
            return self._fail_internal(seq, "HELLO", now_ms)

        was_energized = self.state_machine.state in (
            State.ARMED,
            State.ACTIVE,
        )

        result = self.state_machine.establish_session(
            session_id,
            expected_profile_id == self.profile.profile_id,
        )

        if not result:
            return self._nack(seq, "HELLO", result.error)

        if was_energized or getattr(self.hardware, "enabled", False):
            try:
                self.hardware.force_disabled()
            except HardwareError:
                self.state_machine.enter_fault(
                    Fault.HARDWARE,
                    now_ms=now_ms,
                    hold_pwm=False,
                )
                return (
                    self.telemetry.info(
                        seq,
                        session_id,
                        self.state_machine.state,
                    ),
                    self.telemetry.event("FAULT", Fault.HARDWARE),
                )

        return (
            self.telemetry.info(
                seq,
                session_id,
                self.state_machine.state,
            ),
        )

    def _heartbeat(self, seq, fields, now_ms):
        if len(fields) != 2:
            return self._nack(
                seq,
                "HEARTBEAT",
                Error.BAD_ARG_COUNT,
            )

        try:
            session = _parse_session(fields[0])
            _parse_uint32(fields[1], "host_uptime_ms")
        except ValueError:
            return self._nack(seq, "HEARTBEAT", Error.BAD_VALUE)

        result = self.state_machine.heartbeat(session, now_ms)
        if not result:
            return self._nack(seq, "HEARTBEAT", result.error)

        return ()

    def _stage(self, seq, fields, now_ms):
        if len(fields) != 1 + JOINT_COUNT:
            return self._nack(seq, "STAGE", Error.BAD_ARG_COUNT)

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "STAGE", Error.BAD_VALUE)

        state_error = self._expected_state_error(State.DISARMED)
        if state_error is not None:
            return self._nack(seq, "STAGE", state_error)

        if not self.state_machine.session_matches(session):
            return self._nack(seq, "STAGE", Error.BAD_SESSION)

        try:
            target = _parse_joint_vector(fields[1:])
        except ValueError:
            return self._nack(seq, "STAGE", Error.BAD_VALUE)

        try:
            target = validate_position_target(self.profile, target)
        except TargetShapeError:
            return self._nack(seq, "STAGE", Error.BAD_ARG_COUNT)
        except PositionLimitError:
            return self._nack(seq, "STAGE", Error.LIMIT)

        result = self.state_machine.stage_target(
            session,
            target,
            now_ms,
        )

        if not result:
            return self._nack(seq, "STAGE", result.error)

        return (self.telemetry.ack(seq, "STAGE"),)

    def _arm(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(seq, "ARM", Error.BAD_ARG_COUNT)

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "ARM", Error.BAD_VALUE)

        state_error = self._expected_state_error(State.DISARMED)
        if state_error is not None:
            return self._nack(seq, "ARM", state_error)

        if not self.state_machine.session_matches(session):
            return self._nack(seq, "ARM", Error.BAD_SESSION)

        if not self.state_machine.session_profile_match:
            return self._nack(
                seq,
                "ARM",
                Error.PROFILE_MISMATCH,
            )

        try:
            self.profile.require_arm_qualified()
            self.hardware.require_profile_compatible(self.profile)
        except (ProfileQualificationError, HardwareCompatibilityError):
            return self._nack(seq, "ARM", Error.NOT_READY)
        except HardwareError:
            return self._fault_after_command(
                seq,
                "ARM",
                Fault.HARDWARE,
                now_ms,
            )

        result = self.state_machine.arm(session, now_ms)
        if not result:
            return self._nack(seq, "ARM", result.error)

        try:
            channel_target = logical_to_channel_vector(
                self.profile,
                self.state_machine.commanded_target,
            )
            self.hardware.enable_at_target(channel_target)
        except (PositionLimitError, TargetShapeError):
            return self._fault_after_command(
                seq,
                "ARM",
                Fault.INTERNAL,
                now_ms,
            )
        except HardwareError:
            return self._fault_after_command(
                seq,
                "ARM",
                Fault.HARDWARE,
                now_ms,
            )

        return (self.telemetry.ack(seq, "ARM"),)

    def _start(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(seq, "START", Error.BAD_ARG_COUNT)

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "START", Error.BAD_VALUE)

        result = self.state_machine.start(session, now_ms)
        if not result:
            return self._nack(seq, "START", result.error)

        return (self.telemetry.ack(seq, "START"),)

    def _target(self, seq, fields, now_ms):
        if len(fields) != 2 + JOINT_COUNT:
            return self._nack(seq, "TARGET", Error.BAD_ARG_COUNT)

        try:
            session = _parse_session(fields[0])
            _parse_positive_int(fields[1], "period_ms")
            target = _parse_joint_vector(fields[2:])
        except ValueError:
            return self._nack(seq, "TARGET", Error.BAD_VALUE)

        state_error = self._expected_state_error(State.ACTIVE)
        if state_error is not None:
            return self._nack(seq, "TARGET", state_error)

        if not self.state_machine.session_matches(session):
            return self._nack(
                seq,
                "TARGET",
                Error.BAD_SESSION,
            )

        previous_seq = self.state_machine.last_target_seq
        if previous_seq is not None:
            if not sequence_is_newer(seq, previous_seq):
                return self._nack(seq, "TARGET", Error.SEQ)

        previous_target = self.state_machine.commanded_target
        if previous_target is None:
            return self._fault_after_command(
                seq,
                "TARGET",
                Fault.INTERNAL,
                now_ms,
            )

        if self.state_machine.last_target_at_ms is None:
            anchor_ms = self.state_machine.active_at_ms
        else:
            anchor_ms = self.state_machine.last_target_at_ms

        if anchor_ms is None:
            return self._fault_after_command(
                seq,
                "TARGET",
                Fault.INTERNAL,
                now_ms,
            )

        dt_ms = ticks_diff(now_ms, anchor_ms)
        if dt_ms <= 0:
            return self._nack(
                seq,
                "TARGET",
                Error.RATE_LIMIT,
            )

        try:
            target = validate_rate_transition(
                self.profile,
                previous_target,
                target,
                dt_ms,
            )
            channel_target = logical_to_channel_vector(
                self.profile,
                target,
            )
        except TargetShapeError:
            return self._nack(
                seq,
                "TARGET",
                Error.BAD_ARG_COUNT,
            )
        except PositionLimitError:
            return self._nack(seq, "TARGET", Error.LIMIT)
        except RateLimitError:
            return self._nack(
                seq,
                "TARGET",
                Error.RATE_LIMIT,
            )
        except UnqualifiedRateError:
            return self._nack(
                seq,
                "TARGET",
                Error.NOT_READY,
            )
        except ValueError:
            return self._nack(
                seq,
                "TARGET",
                Error.RATE_LIMIT,
            )

        try:
            self.hardware.apply_target(channel_target)
        except HardwareError:
            return self._fault_after_command(
                seq,
                "TARGET",
                Fault.HARDWARE,
                now_ms,
            )

        result = self.state_machine.accept_target(
            session,
            seq,
            target,
            now_ms,
        )

        if not result:
            return self._fault_after_command(
                seq,
                "TARGET",
                Fault.INTERNAL,
                now_ms,
            )

        return ()

    def _stop(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(seq, "STOP", Error.BAD_ARG_COUNT)

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "STOP", Error.BAD_VALUE)

        result = self.state_machine.stop(session)
        if not result:
            return self._nack(seq, "STOP", result.error)

        return (self.telemetry.ack(seq, "STOP"),)

    def _disarm(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(
                seq,
                "DISARM",
                Error.BAD_ARG_COUNT,
            )

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "DISARM", Error.BAD_VALUE)

        result = self.state_machine.disarm(session)
        if not result:
            return self._nack(seq, "DISARM", result.error)

        try:
            self.hardware.force_disabled()
        except HardwareError:
            return self._fault_after_command(
                seq,
                "DISARM",
                Fault.HARDWARE,
                now_ms,
            )

        return (self.telemetry.ack(seq, "DISARM"),)

    def _estop(self, seq, fields, now_ms):
        if len(fields) != 2:
            return self._nack(seq, "ESTOP", Error.BAD_ARG_COUNT)

        try:
            _parse_session(fields[0])
        except ValueError:
            return self._nack(seq, "ESTOP", Error.BAD_VALUE)

        try:
            reason = _parse_token(fields[1], "reason")
        except ValueError:
            return self._nack(seq, "ESTOP", Error.BAD_VALUE)

        self.state_machine.estop(reason, now_ms=now_ms)

        responses = [
            self.telemetry.ack(seq, "ESTOP"),
            self.telemetry.event("ESTOP", reason),
        ]

        try:
            self.hardware.force_disabled()
        except HardwareError:
            self.state_machine.enter_fault(
                Fault.HARDWARE,
                now_ms=now_ms,
                hold_pwm=False,
            )
            responses.append(
                self.telemetry.event("FAULT", Fault.HARDWARE)
            )

        return tuple(responses)

    def _clear_estop(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(
                seq,
                "CLEAR_ESTOP",
                Error.BAD_ARG_COUNT,
            )

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(
                seq,
                "CLEAR_ESTOP",
                Error.BAD_VALUE,
            )

        underlying_cleared = self._fault_condition_cleared(now_ms)

        result = self.state_machine.clear_estop(
            session,
            released=True,
            underlying_fault_cleared=underlying_cleared,
        )

        if not result:
            return self._nack(
                seq,
                "CLEAR_ESTOP",
                result.error,
            )

        try:
            self.hardware.force_disabled()
        except HardwareError:
            return self._fault_after_command(
                seq,
                "CLEAR_ESTOP",
                Fault.HARDWARE,
                now_ms,
            )

        return (
            self.telemetry.ack(seq, "CLEAR_ESTOP"),
            self.telemetry.event(
                "ESTOP_CLEARED",
                "operator",
            ),
        )

    def _clear_fault(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(
                seq,
                "CLEAR_FAULT",
                Error.BAD_ARG_COUNT,
            )

        try:
            session = _parse_session(fields[0])
        except ValueError:
            return self._nack(
                seq,
                "CLEAR_FAULT",
                Error.BAD_VALUE,
            )

        result = self.state_machine.clear_fault(
            session,
            underlying_cleared=self._fault_condition_cleared(
                now_ms
            ),
        )

        if not result:
            return self._nack(
                seq,
                "CLEAR_FAULT",
                result.error,
            )

        try:
            self.hardware.force_disabled()
        except HardwareError:
            return self._fault_after_command(
                seq,
                "CLEAR_FAULT",
                Fault.HARDWARE,
                now_ms,
            )

        return (
            self.telemetry.ack(seq, "CLEAR_FAULT"),
            self.telemetry.event(
                "FAULT_CLEARED",
                "operator",
            ),
        )

    def _get_status(self, seq, fields, now_ms):
        if len(fields) != 1:
            return self._nack(
                seq,
                "GET_STATUS",
                Error.BAD_ARG_COUNT,
            )

        try:
            supplied_session = _parse_session(fields[0])
        except ValueError:
            return self._nack(
                seq,
                "GET_STATUS",
                Error.BAD_VALUE,
            )

        current_session = self.state_machine.session_id

        if current_session is None:
            if supplied_session != "00000000":
                return self._nack(
                    seq,
                    "GET_STATUS",
                    Error.BAD_SESSION,
                )
        elif supplied_session != current_session:
            return self._nack(
                seq,
                "GET_STATUS",
                Error.BAD_SESSION,
            )

        return (
            self.telemetry.status(
                self.state_machine,
                uptime_ms=now_ms,
            ),
        )

    def tick(self, now_ms):
        responses = []

        fault = self.state_machine.poll(now_ms)
        if fault is not None:
            responses.append(
                self.telemetry.event("FAULT", fault)
            )

        should_enable = self.state_machine.pwm_should_be_enabled(
            now_ms
        )
        hardware_enabled = getattr(self.hardware, "enabled", None)

        if should_enable and hardware_enabled is not True:
            # Known-disabled or unknown hardware cannot satisfy energized
            # authority. Fault immediately and make a best-effort disable.
            self._latch_hardware_fault(now_ms, responses)
            try:
                self.hardware.force_disabled()
            except HardwareError:
                pass
            return tuple(responses)

        if not should_enable and hardware_enabled is not False:
            # True means definitely energized; None means the previous disable
            # failed and the electrical state is unknown. Both require another
            # disable attempt.
            try:
                self.hardware.force_disabled()
            except HardwareError:
                self._latch_hardware_fault(now_ms, responses)

        return tuple(responses)

    def _latch_hardware_fault(self, now_ms, responses):
        """Latch one hardware fault event while allowing repeated disable tries."""
        already_latched = self.state_machine.fault == Fault.HARDWARE

        if not already_latched:
            self.state_machine.enter_fault(
                Fault.HARDWARE,
                now_ms=now_ms,
                hold_pwm=False,
            )
            responses.append(
                self.telemetry.event(
                    "FAULT",
                    Fault.HARDWARE,
                )
            )

    def status_frame(self, now_ms):
        return self.telemetry.status(
            self.state_machine,
            uptime_ms=now_ms,
        )

    def _new_session_id(self):
        raw = self.session_factory()

        if isinstance(raw, bool):
            raise RuntimeErrorInternal("invalid session factory result")

        if isinstance(raw, int):
            if raw <= 0 or raw > UINT32_MAX:
                raise RuntimeErrorInternal(
                    "session factory returned invalid uint32"
                )
            return "%08X" % raw

        if isinstance(raw, str):
            session = _parse_session(raw)
            if session == "00000000":
                raise RuntimeErrorInternal(
                    "session factory returned zero session"
                )
            return session

        raise RuntimeErrorInternal(
            "session factory returned unsupported type"
        )

    def _expected_state_error(self, expected):
        state = self.state_machine.state

        if state == State.ESTOP:
            return Error.ESTOP
        if state == State.FAULT:
            return Error.FAULT_ACTIVE
        if state != expected:
            return Error.BAD_STATE

        return None

    def _fault_condition_cleared(self, now_ms):
        fault = self.state_machine.fault

        if fault == Fault.NONE:
            return True

        if fault in (
            Fault.LINK_TIMEOUT,
            Fault.MOTION_TIMEOUT,
        ):
            heartbeat_age = age_ms(
                now_ms,
                self.state_machine.last_heartbeat_at_ms,
            )
            return 0 <= heartbeat_age < LINK_TIMEOUT_MS

        return False

    def _nack(self, ref_seq, command, error):
        self.semantic_errors += 1
        return (
            self.telemetry.nack(
                ref_seq,
                command,
                error,
            ),
        )

    def _fault_after_command(
        self,
        ref_seq,
        command,
        fault_code,
        now_ms,
    ):
        self.state_machine.enter_fault(
            fault_code,
            now_ms=now_ms,
            hold_pwm=False,
        )

        try:
            self.hardware.force_disabled()
        except Exception:
            pass

        return (
            self.telemetry.nack(
                ref_seq,
                command,
                Error.NOT_READY,
            ),
            self.telemetry.event(
                "FAULT",
                fault_code,
            ),
        )

    def _fail_internal(
        self,
        ref_seq,
        command,
        now_ms,
    ):
        return self._fault_after_command(
            ref_seq,
            command,
            Fault.INTERNAL,
            now_ms,
        )

    def _safe_internal_failure(self, ref_seq, command, now_ms):
        """Contain even an exception raised while reporting another failure."""
        try:
            return tuple(
                self._fail_internal(
                    ref_seq,
                    command,
                    now_ms,
                )
            )
        except Exception:
            try:
                self.state_machine.enter_fault(
                    Fault.INTERNAL,
                    now_ms=now_ms,
                    hold_pwm=False,
                )
            except Exception:
                pass

            try:
                self.hardware.force_disabled()
            except Exception:
                pass

            return ()
