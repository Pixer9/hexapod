# HX1 Servo 2040 on-board hot-path profiler.
# Run only while the robot is unarmed. Executing through mpremote interrupts
# production main.py; main.py's finally path performs a best-effort hardware disable.

import gc
import time

from hexapod_mcu.actuators import validate_position_target
from hexapod_mcu.profile import load_profile
from hexapod_mcu.protocol import (
    LineFramer,
    crc16_ccitt_false,
    encode_frame,
    parse_frame,
)
from hexapod_mcu.runtime import RuntimeCoordinator, _parse_joint_vector
from hexapod_mcu.state_machine import RuntimeStateMachine
from hexapod_mcu.telemetry import TelemetryEncoder

PROFILE_PATH = "config/actuator-profile.json"
SESSION = "A1B2C3D4"


class StubHardware:
    def __init__(self):
        self.enabled = False

    def force_disabled(self):
        self.enabled = False

    def require_profile_compatible(self, profile):
        return None

    def enable_at_target(self, target):
        raise RuntimeError("profiler must never energize hardware")

    def apply_target(self, target):
        raise RuntimeError("profiler must never apply TARGET")


def _session_factory():
    return int(SESSION, 16)


_ticks_us = getattr(time, "ticks_us", None)
if _ticks_us is not None:

    def now_tick():
        return _ticks_us()

    def elapsed_us(start):
        return time.ticks_diff(_ticks_us(), start)
else:

    def now_tick():
        return time.ticks_ms()

    def elapsed_us(start):
        return time.ticks_diff(time.ticks_ms(), start) * 1000


def bench(name, iterations, func):
    gc.collect()
    start = now_tick()
    for i in range(iterations):
        func(i)
    total_us = elapsed_us(start)
    avg_ms = total_us / iterations / 1000.0
    print("%-30s %8.3f ms/call  (%d calls)" % (name, avg_ms, iterations))


profile = load_profile(PROFILE_PATH)
target = tuple(
    (joint.logical_min_cd + joint.logical_max_cd) // 2 for joint in profile.joints
)
joint_fields = tuple(str(value) for value in target)

stage = encode_frame(2, "STAGE", SESSION, *target)
stage_body = stage[:-6]
ack_fields = (2, "STAGE")
status_fields = (
    SESSION,
    "DISARMED",
    "NONE",
    -1,
    -1,
    -1,
    0,
    -1,
    -1,
    123456,
    0,
) + ((0,) * 18)

framer = LineFramer()

state_machine = RuntimeStateMachine()
telemetry = TelemetryEncoder(
    profile=profile,
    firmware_version="bench",
    mcu_id="bench",
    capabilities=0,
)
hardware = StubHardware()
runtime = RuntimeCoordinator(
    profile=profile,
    state_machine=state_machine,
    hardware=hardware,
    telemetry=telemetry,
    session_factory=_session_factory,
)

if not runtime.perform_self_test(0):
    raise RuntimeError("profiler self-test failed")

hello = encode_frame(1, "HELLO", 1, profile.profile_id)
hello_responses = runtime.handle_frame(hello, 1)
if not hello_responses:
    raise RuntimeError("profiler HELLO failed")
if state_machine.session_id != SESSION:
    raise RuntimeError("profiler session setup failed")


def do_crc(_):
    crc16_ccitt_false(stage_body)


def do_parse(_):
    parse_frame(stage)


def do_ack_encode(_):
    encode_frame(100, "ACK", *ack_fields)


def do_status_encode(_):
    encode_frame(101, "STATUS", *status_fields)


def do_framer(_):
    frames = framer.feed(stage)
    if len(frames) != 1:
        raise RuntimeError("framer did not return exactly one frame")


def do_joint_parse(_):
    _parse_joint_vector(joint_fields)


def do_position_validate(_):
    validate_position_target(profile, target)


def do_full_stage(i):
    responses = runtime.handle_frame(stage, 1000 + i)
    if len(responses) != 1:
        raise RuntimeError("STAGE did not return exactly one ACK")


print("HX1 Servo 2040 hot-path profiler")
print("--------------------------------")
print("STAGE frame bytes:", len(stage))
print("STAGE body bytes:", len(stage_body))
print("MCU state:", state_machine.state)
print("Hardware enabled:", hardware.enabled)
print()

bench("CRC(STAGE body)", 50, do_crc)
bench("parse_frame(STAGE)", 30, do_parse)
bench("encode_frame(ACK)", 50, do_ack_encode)
bench("encode_frame(STATUS)", 20, do_status_encode)
bench("LineFramer.feed(STAGE)", 50, do_framer)
bench("_parse_joint_vector(18)", 50, do_joint_parse)
bench("validate_position_target", 50, do_position_validate)
bench("Runtime.handle_frame(STAGE)", 30, do_full_stage)

print()
print("Final MCU state:", state_machine.state)
print("Stub hardware enabled:", hardware.enabled)
print("Profiler complete. Hard-reset the Servo 2040 before resuming HX1 tests.")
