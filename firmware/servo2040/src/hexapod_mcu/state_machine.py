"""Pure Servo 2040 runtime safety state machine.

This module owns runtime safety state and command legality only. It does not
perform serial parsing, profile validation, PWM I/O, or servo mapping.

Expected integration order is:

    1. receive and validate protocol framing;
    2. evaluate already-expired watchdog deadlines before any command that can
       refresh or extend motion authority;
    3. apply the valid command;
    4. periodically call ``poll(now_ms)`` and synchronize physical outputs with
       ``pwm_should_be_enabled(now_ms)``.

A command received at or after a watchdog deadline must not retroactively erase
the timeout that has already occurred.
"""

from .constants import (
    ARM_STAGE_MAX_AGE_MS,
    FAULT_HOLD_MS,
    JOINT_COUNT,
    LINK_TIMEOUT_MS,
    MOTION_TIMEOUT_MS,
    UINT32_MAX,
)
from .protocol import sequence_is_newer
from .watchdogs import age_ms, is_fresh, is_timed_out, ticks_diff


class State:
    BOOTING = "BOOTING"
    SELF_TEST = "SELF_TEST"
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class Error:
    BAD_SESSION = "ERR_BAD_SESSION"
    BAD_STATE = "ERR_BAD_STATE"
    BAD_ARG_COUNT = "ERR_BAD_ARG_COUNT"
    BAD_VALUE = "ERR_BAD_VALUE"
    LIMIT = "ERR_LIMIT"
    RATE_LIMIT = "ERR_RATE_LIMIT"
    PROFILE_MISMATCH = "ERR_PROFILE_MISMATCH"
    NOT_READY = "ERR_NOT_READY"
    SEQ = "ERR_SEQ"
    ESTOP = "ERR_ESTOP"
    FAULT_ACTIVE = "ERR_FAULT_ACTIVE"
    STAGE_EXPIRED = "ERR_STAGE_EXPIRED"
    UNSUPPORTED = "ERR_UNSUPPORTED"


class Fault:
    NONE = "NONE"
    SELF_TEST = "SELF_TEST"
    LINK_TIMEOUT = "LINK_TIMEOUT"
    MOTION_TIMEOUT = "MOTION_TIMEOUT"
    INTERNAL = "INTERNAL"
    PROFILE = "PROFILE"
    HARDWARE = "HARDWARE"


class Result:
    """Small allocation-light command result."""

    def __init__(self, ok, error=None):
        self.ok = bool(ok)
        self.error = error

    def __bool__(self):
        return self.ok

    def __repr__(self):
        if self.ok:
            return "Result(ok=True)"
        return "Result(ok=False, error=%r)" % (self.error,)


OK = Result(True)


def rejected(error):
    return Result(False, error)


def _valid_target_shape(target):
    try:
        return len(target) == JOINT_COUNT
    except TypeError:
        return False


class RuntimeStateMachine:
    """Authoritative MCU runtime safety state.

    The state machine assumes that actuator hard-limit/rate validation happens
    before ``stage_target`` or ``accept_target`` is called. It still validates
    vector completeness and target sequence freshness.
    """

    def __init__(self):
        self.state = State.BOOTING
        self.fault = Fault.NONE

        self.session_id = None
        self.session_profile_match = False

        self.staged_target = None
        self.staged_at_ms = None

        # Last logical target that was commanded/held. This is diagnostic state,
        # not authorization to re-arm.
        self.commanded_target = None

        self.last_target_seq = None
        self.last_target_at_ms = None
        self.last_heartbeat_at_ms = None

        self.armed_at_ms = None
        self.active_at_ms = None

        self.fault_entered_at_ms = None
        self.fault_hold_pwm = False

        self.estop_reason = None

    # ------------------------------------------------------------------
    # Boot / self-test
    # ------------------------------------------------------------------

    def begin_self_test(self):
        if self.state != State.BOOTING:
            return rejected(Error.BAD_STATE)

        self.state = State.SELF_TEST
        return OK

    def complete_self_test(self, passed, now_ms=0, fault_code=Fault.SELF_TEST):
        if self.state != State.SELF_TEST:
            return rejected(Error.BAD_STATE)

        if passed:
            self.state = State.DISARMED
            self.fault = Fault.NONE
            return OK

        self.enter_fault(
            fault_code=fault_code,
            now_ms=now_ms,
            hold_pwm=False,
        )
        return OK

    # ------------------------------------------------------------------
    # Session lifecycle
    # ------------------------------------------------------------------

    def establish_session(self, session_id, profile_match):
        """Install a newly negotiated host session.

        A new session always invalidates staged motion, target sequence history,
        and watchdog freshness.

        Safety refinement: replacing a session while ARMED or ACTIVE immediately
        returns the MCU to DISARMED. A new host process must stage and arm again.
        """
        if not isinstance(session_id, str) or not session_id:
            return rejected(Error.BAD_VALUE)

        if self.state in (State.ARMED, State.ACTIVE):
            self._enter_disarmed(clear_commanded_target=False)

        self.session_id = session_id
        self.session_profile_match = bool(profile_match)

        self._invalidate_stage()
        self.last_target_seq = None
        self.last_target_at_ms = None
        self.last_heartbeat_at_ms = None

        return OK

    def has_session(self):
        return self.session_id is not None

    def session_matches(self, session_id):
        return self.session_id is not None and session_id == self.session_id

    # ------------------------------------------------------------------
    # Input freshness
    # ------------------------------------------------------------------

    def heartbeat(self, session_id, now_ms):
        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        self.last_heartbeat_at_ms = now_ms
        return OK

    def heartbeat_age_ms(self, now_ms):
        return age_ms(now_ms, self.last_heartbeat_at_ms)

    def target_age_ms(self, now_ms):
        return age_ms(now_ms, self.last_target_at_ms)

    # ------------------------------------------------------------------
    # Staging / arm / active motion
    # ------------------------------------------------------------------

    def stage_target(self, session_id, target, now_ms):
        state_error = self._energized_command_state_error(State.DISARMED)
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        if not _valid_target_shape(target):
            return rejected(Error.BAD_ARG_COUNT)

        self.staged_target = tuple(target)
        self.staged_at_ms = now_ms
        return OK

    def stage_is_fresh(self, now_ms):
        return self.staged_target is not None and is_fresh(
            now_ms, self.staged_at_ms, ARM_STAGE_MAX_AGE_MS
        )

    def arm(self, session_id, now_ms):
        state_error = self._energized_command_state_error(State.DISARMED)
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        if not self.session_profile_match:
            return rejected(Error.PROFILE_MISMATCH)

        if self.staged_target is None:
            return rejected(Error.NOT_READY)

        if not self.stage_is_fresh(now_ms):
            self._invalidate_stage()
            return rejected(Error.STAGE_EXPIRED)

        self.commanded_target = self.staged_target
        self._invalidate_stage()

        self.state = State.ARMED
        self.armed_at_ms = now_ms
        self.active_at_ms = None
        self.last_target_at_ms = None

        return OK

    def start(self, session_id, now_ms):
        state_error = self._energized_command_state_error(State.ARMED)
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        heartbeat_age = age_ms(now_ms, self.last_heartbeat_at_ms)
        if not (0 <= heartbeat_age < LINK_TIMEOUT_MS):
            return rejected(Error.NOT_READY)

        self.state = State.ACTIVE
        self.active_at_ms = now_ms

        # A fresh ACTIVE interval must receive a new accepted target within the
        # motion timeout. Preserve sequence history across STOP/START, but reset
        # target freshness for this ACTIVE interval.
        self.last_target_at_ms = None

        return OK

    def accept_target(self, session_id, seq, target, now_ms):
        state_error = self._energized_command_state_error(State.ACTIVE)
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        if not isinstance(seq, int) or seq < 0 or seq > UINT32_MAX:
            return rejected(Error.BAD_VALUE)

        if not _valid_target_shape(target):
            return rejected(Error.BAD_ARG_COUNT)

        if self.last_target_seq is not None:
            if not sequence_is_newer(seq, self.last_target_seq):
                return rejected(Error.SEQ)

        self.commanded_target = tuple(target)
        self.last_target_seq = seq
        self.last_target_at_ms = now_ms

        return OK

    def stop(self, session_id):
        state_error = self._energized_command_state_error(State.ACTIVE)
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        self.state = State.ARMED
        self.active_at_ms = None
        self.last_target_at_ms = None

        return OK

    def disarm(self, session_id):
        state_error = self._energized_command_state_error((State.ARMED, State.ACTIVE))
        if state_error is not None:
            return rejected(state_error)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        self._enter_disarmed(clear_commanded_target=False)
        return OK

    # ------------------------------------------------------------------
    # Fault / E-stop
    # ------------------------------------------------------------------

    def enter_fault(self, fault_code, now_ms, hold_pwm):
        """Enter or record a latched fault.

        If ESTOP is already active it remains the dominant state, but the
        underlying fault code is retained so CLEAR_ESTOP cannot bypass it.

        A fault grace hold may preserve an already-energized output briefly; it
        must never energize PWM that was disabled before the fault.
        """
        if not fault_code or fault_code == Fault.NONE:
            fault_code = Fault.INTERNAL

        was_energized = self.state in (State.ARMED, State.ACTIVE)

        self.fault = fault_code
        self._invalidate_stage()
        self.active_at_ms = None

        if self.state == State.ESTOP:
            self.fault_hold_pwm = False
            self.fault_entered_at_ms = now_ms
            return

        self.state = State.FAULT
        self.fault_entered_at_ms = now_ms
        self.fault_hold_pwm = bool(
            hold_pwm and was_energized and self.commanded_target is not None
        )

    def clear_fault(self, session_id, underlying_cleared):
        if self.state == State.ESTOP:
            return rejected(Error.ESTOP)

        if self.state != State.FAULT:
            return rejected(Error.BAD_STATE)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        if not underlying_cleared:
            return rejected(Error.FAULT_ACTIVE)

        self.fault = Fault.NONE
        self.fault_entered_at_ms = None
        self.fault_hold_pwm = False
        self._enter_disarmed(clear_commanded_target=False)
        return OK

    def estop(self, reason, now_ms=0):
        self.estop_reason = str(reason) if reason is not None else "unspecified"
        self._invalidate_stage()
        self.state = State.ESTOP
        self.armed_at_ms = None
        self.active_at_ms = None
        self.fault_hold_pwm = False
        self.fault_entered_at_ms = now_ms
        return OK

    def clear_estop(self, session_id, released, underlying_fault_cleared=False):
        if self.state != State.ESTOP:
            return rejected(Error.BAD_STATE)

        if not self.session_matches(session_id):
            return rejected(Error.BAD_SESSION)

        if not released:
            return rejected(Error.ESTOP)

        # Do not allow an ESTOP round-trip to erase a pre-existing or newly
        # detected fault condition.
        if self.fault != Fault.NONE and not underlying_fault_cleared:
            return rejected(Error.FAULT_ACTIVE)

        self.fault = Fault.NONE
        self.estop_reason = None
        self.fault_entered_at_ms = None
        self._enter_disarmed(clear_commanded_target=False)
        return OK

    # ------------------------------------------------------------------
    # Watchdog evaluation / outputs
    # ------------------------------------------------------------------

    def poll(self, now_ms):
        """Evaluate watchdogs after processing current-loop commands."""
        if self.state not in (State.ARMED, State.ACTIVE):
            return None

        # LINK watchdog is active in ARMED and ACTIVE.
        if self.last_heartbeat_at_ms is None:
            link_anchor = self.armed_at_ms
        else:
            link_anchor = self.last_heartbeat_at_ms

        if link_anchor is not None and is_timed_out(
            now_ms, link_anchor, LINK_TIMEOUT_MS
        ):
            self.enter_fault(
                fault_code=Fault.LINK_TIMEOUT,
                now_ms=now_ms,
                hold_pwm=True,
            )
            return Fault.LINK_TIMEOUT

        # MOTION watchdog is active only in ACTIVE.
        if self.state == State.ACTIVE:
            if self.last_target_at_ms is None:
                motion_anchor = self.active_at_ms
            else:
                motion_anchor = self.last_target_at_ms

            if motion_anchor is not None and is_timed_out(
                now_ms, motion_anchor, MOTION_TIMEOUT_MS
            ):
                self.enter_fault(
                    fault_code=Fault.MOTION_TIMEOUT,
                    now_ms=now_ms,
                    hold_pwm=True,
                )
                return Fault.MOTION_TIMEOUT

        return None

    def pwm_should_be_enabled(self, now_ms):
        if self.state in (State.ARMED, State.ACTIVE):
            return True

        if self.state == State.FAULT and self.fault_hold_pwm:
            if self.fault_entered_at_ms is None:
                return False

            elapsed = ticks_diff(now_ms, self.fault_entered_at_ms)
            return elapsed >= 0 and elapsed < FAULT_HOLD_MS

        return False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _invalidate_stage(self):
        self.staged_target = None
        self.staged_at_ms = None

    def _enter_disarmed(self, clear_commanded_target):
        self.state = State.DISARMED
        self._invalidate_stage()

        self.armed_at_ms = None
        self.active_at_ms = None
        self.last_target_at_ms = None

        if clear_commanded_target:
            self.commanded_target = None

    def _energized_command_state_error(self, expected):
        if self.state == State.ESTOP:
            return Error.ESTOP

        if self.state == State.FAULT:
            return Error.FAULT_ACTIVE

        if isinstance(expected, tuple):
            if self.state not in expected:
                return Error.BAD_STATE
        elif self.state != expected:
            return Error.BAD_STATE

        return None
