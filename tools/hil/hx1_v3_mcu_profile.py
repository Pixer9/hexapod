# HX1 v3 production-path microprofiler for Servo 2040.
# Safe: uses stub hardware only. No PWM enable or TARGET application is possible.

import gc
import time

from hexapod_mcu.profile import load_profile
from hexapod_mcu.protocol import (
    LineFramer,
    encode_frame,
    parse_command_frame,
)
from hexapod_mcu.runtime import RuntimeCoordinator
from hexapod_mcu.state_machine import RuntimeStateMachine
from hexapod_mcu.telemetry import TelemetryEncoder

PROFILE_PATH = "config/actuator-profile.json"
SESSION_INT = 0xA1B2C3D4
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


_ticks_us = getattr(time, "ticks_us", None)

if _ticks_us is not None:

    def tick():
        return _ticks_us()

    def elapsed_us(start):
        return time.ticks_diff(_ticks_us(), start)
else:

    def tick():
        return time.ticks_ms()

    def elapsed_us(start):
        return time.ticks_diff(time.ticks_ms(), start) * 1000


def bench(name, iterations, func):
    gc.collect()
    start = tick()
    for i in range(iterations):
        func(i)
    total_us = elapsed_us(start)
    print(
        "%-34s %8.3f ms/call  (%d calls)"
        % (name, total_us / iterations / 1000.0, iterations)
    )


profile = load_profile(PROFILE_PATH)

# Use the midpoint of each logical hard range so this remains profile-derived
# and valid without depending on Pi-side motion code.
target = tuple(
    (joint.logical_min_cd + joint.logical_max_cd) // 2 for joint in profile.joints
)

state_machine = RuntimeStateMachine()
hardware = StubHardware()
telemetry = TelemetryEncoder(
    profile=profile,
    firmware_version="v3-profiler",
    mcu_id="bench",
    capabilities=0,
)
runtime = RuntimeCoordinator(
    profile=profile,
    state_machine=state_machine,
    hardware=hardware,
    telemetry=telemetry,
    session_factory=lambda: SESSION_INT,
)

if not runtime.perform_self_test(0):
    raise RuntimeError("profiler self-test failed")

hello = encode_frame(1, "HELLO", 1, profile.profile_id)
hello_responses = runtime.handle_frame(hello, 1)
if len(hello_responses) != 1:
    raise RuntimeError("profiler HELLO failed")
if state_machine.session_id != SESSION:
    raise RuntimeError("profiler session setup failed")

stage = encode_frame(2, "STAGE", SESSION, *target)

# A separate framer instance can consume the same complete packet repeatedly
# because each call empties the internal buffer after returning the frame.
framer = LineFramer()


def do_framer(_):
    frames = framer.feed(stage)
    if len(frames) != 1 or frames[0] != stage:
        raise RuntimeError("binary framer mismatch")


def do_parse(_):
    seq, command, fields = parse_command_frame(stage)
    if seq != 2 or command != "STAGE":
        raise RuntimeError("binary parse mismatch")
    if tuple(fields[1:]) != target:
        raise RuntimeError("binary joint decode mismatch")


def do_ack(i):
    frame = telemetry.ack(1000 + i, "STAGE")
    if not frame:
        raise RuntimeError("ACK encode failed")


def do_tick(i):
    responses = runtime.tick(100 + i)
    if responses:
        raise RuntimeError("unexpected tick response")


def do_status(i):
    frame = runtime.status_frame(1000 + i)
    if not frame:
        raise RuntimeError("STATUS encode failed")


def do_runtime_stage(i):
    responses = runtime.handle_frame(stage, 2000 + i)
    if len(responses) != 1:
        raise RuntimeError("STAGE did not return one ACK")


print("HX1 v3 Servo 2040 production-path profiler")
print("-------------------------------------------")
print("Binary STAGE frame bytes:", len(stage))
print("MCU state:", state_machine.state)
print("Hardware enabled:", hardware.enabled)
print()

bench("LineFramer.feed(STAGE)", 100, do_framer)
bench("parse_command_frame(STAGE)", 100, do_parse)
bench("TelemetryEncoder.ack(STAGE)", 100, do_ack)
bench("Runtime.tick(DISARMED)", 100, do_tick)
bench("Runtime.status_frame()", 30, do_status)
bench("Runtime.handle_frame(STAGE)", 100, do_runtime_stage)

print()
print("Final MCU state:", state_machine.state)
print("Stub hardware enabled:", hardware.enabled)
print("Profiler complete. Hard-reset the Servo 2040 before resuming HX1 tests.")
