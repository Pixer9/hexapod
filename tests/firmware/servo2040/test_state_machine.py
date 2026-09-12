"""Host-side tests for the Servo 2040 runtime safety state machine."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

from hexapod_mcu.constants import (  # noqa: E402
    ARM_STAGE_MAX_AGE_MS,
    FAULT_HOLD_MS,
    JOINT_COUNT,
    LINK_TIMEOUT_MS,
    MOTION_TIMEOUT_MS,
)
from hexapod_mcu.state_machine import (  # noqa: E402
    Error,
    Fault,
    RuntimeStateMachine,
    State,
)


SESSION = "A1B2C3D4"
TARGET = tuple(0 for _ in range(JOINT_COUNT))


def ready_disarmed(profile_match=True):
    sm = RuntimeStateMachine()
    assert sm.begin_self_test()
    assert sm.complete_self_test(True)
    assert sm.establish_session(SESSION, profile_match)
    return sm


def ready_armed(now_ms=100):
    sm = ready_disarmed()
    assert sm.stage_target(SESSION, TARGET, now_ms)
    assert sm.arm(SESSION, now_ms)
    return sm


def ready_active(now_ms=100):
    sm = ready_armed(now_ms)
    assert sm.heartbeat(SESSION, now_ms)
    assert sm.start(SESSION, now_ms)
    return sm


class BootTests(unittest.TestCase):
    def test_boot_to_self_test_to_disarmed(self):
        sm = RuntimeStateMachine()
        self.assertEqual(sm.state, State.BOOTING)
        self.assertTrue(sm.begin_self_test())
        self.assertEqual(sm.state, State.SELF_TEST)
        self.assertTrue(sm.complete_self_test(True))
        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(sm.pwm_should_be_enabled(0))

    def test_failed_self_test_latches_fault(self):
        sm = RuntimeStateMachine()
        sm.begin_self_test()
        sm.complete_self_test(False, now_ms=25, fault_code=Fault.PROFILE)

        self.assertEqual(sm.state, State.FAULT)
        self.assertEqual(sm.fault, Fault.PROFILE)
        self.assertFalse(sm.pwm_should_be_enabled(25))


class SessionTests(unittest.TestCase):
    def test_new_session_resets_session_scoped_motion_state(self):
        sm = ready_disarmed()
        self.assertTrue(sm.stage_target(SESSION, TARGET, 10))
        self.assertTrue(sm.heartbeat(SESSION, 20))

        self.assertTrue(sm.establish_session("11223344", True))

        self.assertEqual(sm.state, State.DISARMED)
        self.assertIsNone(sm.staged_target)
        self.assertIsNone(sm.last_heartbeat_at_ms)
        self.assertIsNone(sm.last_target_seq)

    def test_new_session_while_armed_disarms_immediately(self):
        sm = ready_armed(100)
        self.assertTrue(sm.pwm_should_be_enabled(100))

        self.assertTrue(sm.establish_session("11223344", True))

        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(sm.pwm_should_be_enabled(100))

    def test_new_session_while_active_disarms_immediately(self):
        sm = ready_active(100)
        self.assertEqual(sm.state, State.ACTIVE)

        self.assertTrue(sm.establish_session("11223344", True))

        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(sm.pwm_should_be_enabled(100))


class StageAndArmTests(unittest.TestCase):
    def test_stage_requires_current_session(self):
        sm = ready_disarmed()
        result = sm.stage_target("DEADBEEF", TARGET, 0)
        self.assertFalse(result)
        self.assertEqual(result.error, Error.BAD_SESSION)

    def test_stage_requires_complete_vector(self):
        sm = ready_disarmed()
        result = sm.stage_target(SESSION, TARGET[:-1], 0)
        self.assertFalse(result)
        self.assertEqual(result.error, Error.BAD_ARG_COUNT)

    def test_arm_requires_profile_match(self):
        sm = ready_disarmed(profile_match=False)
        sm.stage_target(SESSION, TARGET, 0)

        result = sm.arm(SESSION, 0)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.PROFILE_MISMATCH)
        self.assertEqual(sm.state, State.DISARMED)

    def test_arm_requires_stage(self):
        sm = ready_disarmed()

        result = sm.arm(SESSION, 0)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.NOT_READY)

    def test_stage_is_fresh_through_exact_max_age(self):
        sm = ready_disarmed()
        sm.stage_target(SESSION, TARGET, 0)

        self.assertTrue(sm.arm(SESSION, ARM_STAGE_MAX_AGE_MS))
        self.assertEqual(sm.state, State.ARMED)

    def test_expired_stage_is_rejected_and_invalidated(self):
        sm = ready_disarmed()
        sm.stage_target(SESSION, TARGET, 0)

        result = sm.arm(SESSION, ARM_STAGE_MAX_AGE_MS + 1)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.STAGE_EXPIRED)
        self.assertIsNone(sm.staged_target)

    def test_successful_arm_consumes_stage_and_holds_target(self):
        sm = ready_disarmed()
        sm.stage_target(SESSION, TARGET, 100)

        self.assertTrue(sm.arm(SESSION, 100))

        self.assertEqual(sm.state, State.ARMED)
        self.assertEqual(sm.commanded_target, TARGET)
        self.assertIsNone(sm.staged_target)
        self.assertTrue(sm.pwm_should_be_enabled(100))


class ActiveTests(unittest.TestCase):
    def test_start_requires_fresh_heartbeat(self):
        sm = ready_armed(100)

        result = sm.start(SESSION, 100)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.NOT_READY)
        self.assertEqual(sm.state, State.ARMED)

    def test_start_accepts_fresh_heartbeat(self):
        sm = ready_armed(100)
        sm.heartbeat(SESSION, 120)

        self.assertTrue(sm.start(SESSION, 120))
        self.assertEqual(sm.state, State.ACTIVE)

    def test_target_requires_active_state(self):
        sm = ready_armed(100)
        result = sm.accept_target(SESSION, 1, TARGET, 120)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.BAD_STATE)

    def test_target_sequence_must_advance(self):
        sm = ready_active(100)

        self.assertTrue(sm.accept_target(SESSION, 100, TARGET, 120))

        duplicate = sm.accept_target(SESSION, 100, TARGET, 140)
        stale = sm.accept_target(SESSION, 99, TARGET, 160)

        self.assertFalse(duplicate)
        self.assertEqual(duplicate.error, Error.SEQ)
        self.assertFalse(stale)
        self.assertEqual(stale.error, Error.SEQ)
        self.assertEqual(sm.last_target_at_ms, 120)

    def test_target_sequence_wraps(self):
        sm = ready_active(100)

        self.assertTrue(sm.accept_target(SESSION, 0xFFFFFFFF, TARGET, 120))
        self.assertTrue(sm.accept_target(SESSION, 0, TARGET, 140))
        self.assertEqual(sm.last_target_seq, 0)

    def test_stop_holds_target_and_returns_to_armed(self):
        sm = ready_active(100)
        changed = tuple(10 for _ in range(JOINT_COUNT))
        sm.accept_target(SESSION, 1, changed, 120)

        self.assertTrue(sm.stop(SESSION))

        self.assertEqual(sm.state, State.ARMED)
        self.assertEqual(sm.commanded_target, changed)
        self.assertTrue(sm.pwm_should_be_enabled(130))

    def test_disarm_turns_pwm_authority_off(self):
        sm = ready_active(100)

        self.assertTrue(sm.disarm(SESSION))

        self.assertEqual(sm.state, State.DISARMED)
        self.assertFalse(sm.pwm_should_be_enabled(100))


class WatchdogTests(unittest.TestCase):
    def test_armed_without_heartbeat_faults_after_link_timeout(self):
        sm = ready_armed(100)

        self.assertIsNone(sm.poll(100 + LINK_TIMEOUT_MS - 1))
        fault = sm.poll(100 + LINK_TIMEOUT_MS)

        self.assertEqual(fault, Fault.LINK_TIMEOUT)
        self.assertEqual(sm.state, State.FAULT)

    def test_heartbeat_refreshes_link_watchdog(self):
        sm = ready_armed(100)
        sm.heartbeat(SESSION, 500)

        self.assertIsNone(sm.poll(500 + LINK_TIMEOUT_MS - 1))
        self.assertEqual(
            sm.poll(500 + LINK_TIMEOUT_MS),
            Fault.LINK_TIMEOUT,
        )

    def test_active_without_target_faults_after_motion_timeout(self):
        sm = ready_active(100)

        self.assertIsNone(sm.poll(100 + MOTION_TIMEOUT_MS - 1))
        fault = sm.poll(100 + MOTION_TIMEOUT_MS)

        self.assertEqual(fault, Fault.MOTION_TIMEOUT)
        self.assertEqual(sm.state, State.FAULT)

    def test_accepted_target_refreshes_motion_watchdog(self):
        sm = ready_active(100)
        sm.accept_target(SESSION, 1, TARGET, 150)

        self.assertIsNone(sm.poll(150 + MOTION_TIMEOUT_MS - 1))
        self.assertEqual(
            sm.poll(150 + MOTION_TIMEOUT_MS),
            Fault.MOTION_TIMEOUT,
        )

    def test_bad_target_does_not_refresh_motion_watchdog(self):
        sm = ready_active(100)
        sm.accept_target(SESSION, 1, TARGET, 150)

        rejected = sm.accept_target(SESSION, 1, TARGET, 250)
        self.assertFalse(rejected)

        self.assertEqual(
            sm.poll(150 + MOTION_TIMEOUT_MS),
            Fault.MOTION_TIMEOUT,
        )

    def test_link_fault_has_deterministic_priority(self):
        sm = ready_active(100)

        # No subsequent heartbeat or target. Both are late here, but LINK is
        # checked first by contract in this implementation.
        fault = sm.poll(100 + LINK_TIMEOUT_MS)

        self.assertEqual(fault, Fault.LINK_TIMEOUT)


class FaultOutputTests(unittest.TestCase):
    def test_watchdog_fault_holds_pwm_for_grace_interval(self):
        sm = ready_armed(100)
        fault_at = 100 + LINK_TIMEOUT_MS
        sm.poll(fault_at)

        self.assertTrue(sm.pwm_should_be_enabled(fault_at))
        self.assertTrue(sm.pwm_should_be_enabled(fault_at + FAULT_HOLD_MS - 1))
        self.assertFalse(sm.pwm_should_be_enabled(fault_at + FAULT_HOLD_MS))

    def test_immediate_fault_does_not_hold_pwm(self):
        sm = ready_armed(100)
        sm.enter_fault(Fault.HARDWARE, now_ms=150, hold_pwm=False)

        self.assertEqual(sm.state, State.FAULT)
        self.assertFalse(sm.pwm_should_be_enabled(150))

    def test_clear_fault_requires_condition_cleared(self):
        sm = ready_armed(100)
        sm.enter_fault(Fault.HARDWARE, now_ms=150, hold_pwm=False)

        rejected = sm.clear_fault(SESSION, underlying_cleared=False)
        self.assertFalse(rejected)
        self.assertEqual(rejected.error, Error.FAULT_ACTIVE)

        self.assertTrue(sm.clear_fault(SESSION, underlying_cleared=True))
        self.assertEqual(sm.state, State.DISARMED)
        self.assertEqual(sm.fault, Fault.NONE)


class EstopTests(unittest.TestCase):
    def test_estop_from_active_disables_pwm_immediately(self):
        sm = ready_active(100)

        self.assertTrue(sm.estop("operator", now_ms=150))

        self.assertEqual(sm.state, State.ESTOP)
        self.assertFalse(sm.pwm_should_be_enabled(150))
        self.assertIsNone(sm.staged_target)

    def test_estop_does_not_require_matching_session(self):
        sm = ready_active(100)

        # Session validation is intentionally absent from the ESTOP state
        # transition; the protocol layer only needs a syntactically/CRC-valid
        # ESTOP before invoking it.
        self.assertTrue(sm.estop("pi_safety", now_ms=150))
        self.assertEqual(sm.state, State.ESTOP)

    def test_clear_estop_requires_release(self):
        sm = ready_active(100)
        sm.estop("operator", now_ms=150)

        result = sm.clear_estop(SESSION, released=False)

        self.assertFalse(result)
        self.assertEqual(result.error, Error.ESTOP)
        self.assertEqual(sm.state, State.ESTOP)

    def test_estop_cannot_bypass_underlying_fault(self):
        sm = ready_armed(100)
        sm.enter_fault(Fault.HARDWARE, now_ms=150, hold_pwm=False)
        sm.estop("operator", now_ms=160)

        result = sm.clear_estop(
            SESSION,
            released=True,
            underlying_fault_cleared=False,
        )

        self.assertFalse(result)
        self.assertEqual(result.error, Error.FAULT_ACTIVE)
        self.assertEqual(sm.state, State.ESTOP)

        self.assertTrue(
            sm.clear_estop(
                SESSION,
                released=True,
                underlying_fault_cleared=True,
            )
        )
        self.assertEqual(sm.state, State.DISARMED)
        self.assertEqual(sm.fault, Fault.NONE)


if __name__ == "__main__":
    unittest.main()
