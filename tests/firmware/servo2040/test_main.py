"""Host-side tests for the Servo 2040 firmware scheduler."""

from __future__ import annotations

import importlib
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_SRC = REPO_ROOT / "firmware" / "servo2040" / "src"
sys.path.insert(0, str(FIRMWARE_SRC))

firmware_main = importlib.import_module("main")

from hexapod_mcu.protocol import encode_frame, parse_frame  # noqa: E402
from hexapod_mcu.state_machine import Fault, State  # noqa: E402


class FakeClock:
    def __init__(self, now=0):
        self.now = now
        self.sleep_calls = []

    def ticks_ms(self):
        return self.now

    def ticks_diff(self, now, then):
        return now - then

    def sleep_ms(self, duration):
        self.sleep_calls.append(duration)
        self.now += duration


class FakeStateMachine:
    def __init__(self):
        self.state = State.DISARMED
        self.fault = Fault.NONE
        self.entered_faults = []

    def enter_fault(self, fault_code, now_ms, hold_pwm):
        self.state = State.FAULT
        self.fault = fault_code
        self.entered_faults.append((fault_code, now_ms, hold_pwm))


class FakeTelemetry:
    def __init__(self):
        self.seq = 0

    def event(self, event_type, detail):
        frame = encode_frame(self.seq, "EVENT", event_type, detail)
        self.seq += 1
        return frame


class FakeRuntime:
    def __init__(self):
        self.state_machine = FakeStateMachine()
        self.telemetry = FakeTelemetry()
        self.handled = []
        self.ticks = []
        self.statuses = []

    def handle_frame(self, frame, now_ms):
        self.handled.append((frame, now_ms))
        seq, message_type, _ = parse_frame(frame)
        return (encode_frame(seq + 100, "ACK", seq, message_type),)

    def tick(self, now_ms):
        self.ticks.append(now_ms)
        return ()

    def status_frame(self, now_ms):
        self.statuses.append(now_ms)
        return encode_frame(900 + len(self.statuses), "EVENT", "STATUS_TICK", now_ms)


class FakeHardware:
    def __init__(self):
        self.enabled = False
        self.disable_calls = 0

    def force_disabled(self):
        self.disable_calls += 1
        self.enabled = False


class FakeTransport:
    def __init__(self, frames=()):
        self.frames = list(frames)
        self.sent = []
        self.poll_error = False
        self.send_error = False

    def poll_frames(self):
        if self.poll_error:
            raise OSError("injected poll failure")
        frames = tuple(self.frames)
        self.frames = []
        return frames

    def send_frame(self, frame):
        if self.send_error:
            raise OSError("injected send failure")
        self.sent.append(frame)


class SessionGeneratorTests(unittest.TestCase):
    def test_sessions_are_nonzero_and_change(self):
        generator = firmware_main.SessionIdGenerator(
            b"\x01\x02\x03\x04",
            1234,
        )

        first = generator()
        second = generator()

        self.assertNotEqual(first, 0)
        self.assertNotEqual(second, 0)
        self.assertNotEqual(first, second)

    def test_same_seed_is_reproducible(self):
        a = firmware_main.SessionIdGenerator(b"abc", 100)
        b = firmware_main.SessionIdGenerator(b"abc", 100)

        self.assertEqual(a(), b())
        self.assertEqual(a(), b())


class SchedulerTests(unittest.TestCase):
    def make_app(self, frames=()):
        runtime = FakeRuntime()
        hardware = FakeHardware()
        transport = FakeTransport(frames)
        clock = FakeClock(100)
        app = firmware_main.FirmwareApp(
            runtime=runtime,
            transport=transport,
            hardware=hardware,
            clock=clock,
            status_period_ms=100,
        )
        return app, runtime, hardware, transport, clock

    def test_inbound_frames_are_processed_and_responses_sent(self):
        frame = encode_frame(1, "STOP", "A1B2C3D4")
        app, runtime, _, transport, _ = self.make_app((frame,))

        app.run_once()

        self.assertEqual(len(runtime.handled), 1)
        self.assertGreaterEqual(len(transport.sent), 2)  # ACK + first STATUS
        self.assertEqual(parse_frame(transport.sent[0])[1], "ACK")

    def test_first_loop_emits_periodic_status(self):
        app, runtime, _, transport, _ = self.make_app()

        app.run_once()

        self.assertEqual(runtime.statuses, [100])
        self.assertEqual(parse_frame(transport.sent[-1])[1], "EVENT")

    def test_status_waits_for_period(self):
        app, runtime, _, _, clock = self.make_app()

        app.run_once()
        clock.now = 199
        app.run_once()

        self.assertEqual(runtime.statuses, [100])

        clock.now = 200
        app.run_once()

        self.assertEqual(runtime.statuses, [100, 200])

    def test_poll_failure_latches_internal_fault_and_disables(self):
        app, runtime, hardware, transport, _ = self.make_app()
        transport.poll_error = True

        app.run_once()

        self.assertEqual(runtime.state_machine.state, State.FAULT)
        self.assertEqual(runtime.state_machine.fault, Fault.INTERNAL)
        self.assertEqual(hardware.disable_calls, 1)
        self.assertEqual(app.loop_errors, 1)

    def test_send_failure_does_not_stop_safety_tick(self):
        frame = encode_frame(1, "STOP", "A1B2C3D4")
        app, runtime, _, transport, _ = self.make_app((frame,))
        transport.send_error = True

        app.run_once()

        self.assertEqual(runtime.ticks, [100])
        self.assertGreater(app.tx_errors, 0)

    def test_periodic_status_is_disabled_by_default(self):
        runtime = FakeRuntime()
        hardware = FakeHardware()
        transport = FakeTransport()
        clock = FakeClock(100)

        app = firmware_main.FirmwareApp(
            runtime=runtime,
            transport=transport,
            hardware=hardware,
            clock=clock,
        )

        app.run_once()

        self.assertEqual(runtime.statuses, [])


class ReleaseCandidateTests(unittest.TestCase):
    def test_firmware_version_is_rc5(self):
        self.assertEqual(firmware_main.FIRMWARE_VERSION, "0.1.0-rc5")

    def test_scheduler_keeps_one_millisecond_idle(self):
        self.assertEqual(firmware_main.LOOP_SLEEP_MS, 1)


class DiagnosticFallbackTests(unittest.TestCase):
    def test_invalid_profile_metadata_is_protocol_safe(self):
        profile = firmware_main.InvalidProfile()

        self.assertEqual(profile.profile_id, "invalid-profile")
        self.assertEqual(profile.profile_revision, 0)
        self.assertEqual(len(profile.profile_hash), 64)
        self.assertEqual(profile.joint_count, 18)


if __name__ == "__main__":
    unittest.main()
