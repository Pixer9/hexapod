"""Hexapod Servo 2040 firmware entrypoint.

This file performs composition and scheduling only. Robot kinematics, protocol
semantics, safety-state policy, actuator validation, and hardware mapping live
in their dedicated modules.

The module is safe to import under CPython for host-side tests. Board-specific
imports occur only while building the production application.
"""

FIRMWARE_VERSION = "0.1.0-rc1"
PROFILE_PATH = "config/actuator-profile.json"

STATUS_RATE_HZ = 10
STATUS_PERIOD_MS = 100
LOOP_SLEEP_MS = 1

# Protocol-v1 release candidate currently advertises no optional sensor
# capability until its acquisition path is implemented and tested.
CAPABILITIES = 0x00000000


class SessionIdGenerator:
    """Small deterministic 32-bit session generator.

    Session IDs prevent stale traffic from a prior host process from inheriting
    current control authority. They are not cryptographic authentication
    tokens, so a simple per-boot xorshift32 generator is sufficient.

    The seed mixes the MCU unique ID with a monotonic startup tick.
    """

    def __init__(self, unique_id_bytes, startup_tick):
        if not isinstance(unique_id_bytes, (bytes, bytearray)):
            raise TypeError("unique_id_bytes must be bytes")

        if isinstance(startup_tick, bool) or not isinstance(startup_tick, int):
            raise TypeError("startup_tick must be an integer")

        state = (0x9E3779B9 ^ (startup_tick & 0xFFFFFFFF)) & 0xFFFFFFFF

        for byte in unique_id_bytes:
            state ^= byte
            state = (state * 0x01000193) & 0xFFFFFFFF

        if state == 0:
            state = 0x6D2B79F5

        self._state = state

    def __call__(self):
        x = self._state

        x ^= (x << 13) & 0xFFFFFFFF
        x ^= (x >> 17) & 0xFFFFFFFF
        x ^= (x << 5) & 0xFFFFFFFF
        x &= 0xFFFFFFFF

        if x == 0:
            x = 0xA5A5A5A5

        self._state = x
        return x


class UnavailableHardware:
    """Fail-safe adapter used when real ServoCluster initialization fails."""

    enabled = False

    def __init__(self, reason="unavailable"):
        self.reason = reason

    def force_disabled(self):
        from hexapod_mcu.hardware import HardwareError
        raise HardwareError(self.reason)

    def require_profile_compatible(self, profile):
        from hexapod_mcu.hardware import HardwareError
        raise HardwareError(self.reason)

    def enable_at_target(self, target):
        from hexapod_mcu.hardware import HardwareError
        raise HardwareError(self.reason)

    def apply_target(self, target):
        from hexapod_mcu.hardware import HardwareError
        raise HardwareError(self.reason)


class InvalidProfile:
    """Diagnostic profile used only when actuator-profile loading itself fails."""

    schema_version = 0
    profile_id = "invalid-profile"
    profile_revision = 0
    joint_count = 18
    angle_unit = "centidegree"
    rate_unit = "centidegree_per_second"
    profile_hash = "0" * 64
    joints = ()

    def require_arm_qualified(self):
        from hexapod_mcu.profile import ProfileQualificationError
        raise ProfileQualificationError("actuator profile failed to load")


class FirmwareApp:
    """Single-threaded deterministic firmware scheduler."""

    def __init__(
        self,
        runtime,
        transport,
        hardware,
        clock,
        status_period_ms=STATUS_PERIOD_MS,
    ):
        if (
            isinstance(status_period_ms, bool)
            or not isinstance(status_period_ms, int)
            or status_period_ms <= 0
        ):
            raise ValueError("status_period_ms must be a positive integer")

        self.runtime = runtime
        self.transport = transport
        self.hardware = hardware
        self.clock = clock
        self.status_period_ms = status_period_ms

        self.last_status_ms = None
        self.tx_errors = 0
        self.loop_errors = 0
        self._fatal_loop_fault_reported = False

    def run_once(self):
        """Execute one bounded firmware-loop iteration."""
        try:
            frames = self.transport.poll_frames()
        except Exception:
            self._latch_loop_fault()
            frames = ()

        for raw_frame in frames:
            now_ms = self.clock.ticks_ms()
            responses = self.runtime.handle_frame(raw_frame, now_ms)
            self._send_frames(responses)

        now_ms = self.clock.ticks_ms()

        try:
            events = self.runtime.tick(now_ms)
        except Exception:
            self._latch_loop_fault(now_ms)
            events = ()

        self._send_frames(events)

        if self._status_due(now_ms):
            try:
                status = self.runtime.status_frame(now_ms)
            except Exception:
                self._latch_loop_fault(now_ms)
            else:
                self._send_frames((status,))
                self.last_status_ms = now_ms

    def run_forever(self):
        while True:
            self.run_once()
            self.clock.sleep_ms(LOOP_SLEEP_MS)

    def _status_due(self, now_ms):
        if self.last_status_ms is None:
            return True

        return (
            self.clock.ticks_diff(now_ms, self.last_status_ms)
            >= self.status_period_ms
        )

    def _send_frames(self, frames):
        for frame in frames:
            try:
                self.transport.send_frame(frame)
            except Exception:
                # A TX-path failure must not crash or stall safety processing.
                # Inbound heartbeat/target freshness remains authoritative.
                self.tx_errors += 1

    def _latch_loop_fault(self, now_ms=None):
        """Fail safe on an unexpected scheduler/transport exception."""
        self.loop_errors += 1

        if now_ms is None:
            now_ms = self.clock.ticks_ms()

        try:
            from hexapod_mcu.state_machine import Fault
            self.runtime.state_machine.enter_fault(
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

        if not self._fatal_loop_fault_reported:
            try:
                frame = self.runtime.telemetry.event(
                    "FAULT",
                    "INTERNAL",
                )
                self._send_frames((frame,))
            except Exception:
                pass

            self._fatal_loop_fault_reported = True


class MicroPythonClock:
    """Wrap-safe clock adapter over MicroPython's time module."""

    def __init__(self, time_module):
        self._time = time_module

    def ticks_ms(self):
        return self._time.ticks_ms()

    def ticks_diff(self, now_ms, then_ms):
        return self._time.ticks_diff(now_ms, then_ms)

    def sleep_ms(self, duration_ms):
        self._time.sleep_ms(duration_ms)


def _mcu_identity(machine_module, ubinascii_module):
    raw = machine_module.unique_id()
    if not isinstance(raw, (bytes, bytearray)) or not raw:
        return "unknown"

    try:
        return ubinascii_module.hexlify(bytes(raw)).decode("ascii")
    except Exception:
        return "unknown"


def _startup_tick(time_module):
    ticks_us = getattr(time_module, "ticks_us", None)
    if ticks_us is not None:
        try:
            return int(ticks_us())
        except Exception:
            pass

    return int(time_module.ticks_ms())


def build_application():
    """Compose the production MicroPython application.

    This function intentionally delays every board-specific import until actual
    firmware startup, keeping ``main.py`` importable under CPython tests.
    """
    import time
    import machine

    try:
        import ubinascii
    except ImportError:
        import binascii as ubinascii

    from hexapod_mcu.hardware import (
        create_servo2040_hardware,
    )
    from hexapod_mcu.profile import load_profile
    from hexapod_mcu.runtime import RuntimeCoordinator
    from hexapod_mcu.state_machine import RuntimeStateMachine
    from hexapod_mcu.telemetry import TelemetryEncoder
    from hexapod_mcu.transport import create_usb_cdc_transport

    clock = MicroPythonClock(time)

    # Hardware creation is attempted before normal runtime begins. The real
    # adapter explicitly disables all PWM outputs during its own construction.
    try:
        hardware = create_servo2040_hardware()
    except Exception as exc:
        hardware = UnavailableHardware(str(exc))

    try:
        profile = load_profile(PROFILE_PATH)
    except Exception:
        profile = InvalidProfile()

    transport = create_usb_cdc_transport()

    mcu_id = _mcu_identity(machine, ubinascii)
    unique_id = machine.unique_id()

    session_factory = SessionIdGenerator(
        unique_id,
        _startup_tick(time),
    )

    state_machine = RuntimeStateMachine()

    telemetry = TelemetryEncoder(
        profile=profile,
        firmware_version=FIRMWARE_VERSION,
        mcu_id=mcu_id,
        capabilities=CAPABILITIES,
    )

    runtime = RuntimeCoordinator(
        profile=profile,
        state_machine=state_machine,
        hardware=hardware,
        telemetry=telemetry,
        session_factory=session_factory,
    )

    # Self-test is intentionally allowed to fail into a latched FAULT while
    # leaving the protocol loop alive for diagnostics.
    runtime.perform_self_test(clock.ticks_ms())

    return FirmwareApp(
        runtime=runtime,
        transport=transport,
        hardware=hardware,
        clock=clock,
    )


def main():
    app = build_application()
    app.run_forever()


if __name__ == "__main__":
    main()
