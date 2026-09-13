# HX1 Cross-Boundary Contract Tests

**Status:** Host-side release gate for the Pi/Servo 2040 boundary

## Purpose

The Raspberry Pi and Servo 2040 contain independent implementations of HX1.

Unit tests on either side prove that each implementation is internally
consistent. They do not, by themselves, prove that one side's real output is
accepted by the other side's real parser and runtime.

The cross-boundary suite therefore imports both production implementations into
the same CPython process and connects them directly.

No third test-only HX1 implementation is introduced.

## Location

```text
tests/contracts/test_hx1_pi_servo2040.py
```

Run it with:

```bash
python -m unittest discover \
  -s tests/contracts \
  -p 'test_*.py' \
  -v
```

The contract suite is separate from the normal Pi and firmware unit suites
because it intentionally depends on both source trees.

## Production implementations under test

Pi:

```text
src/hexapod/hx1/
src/hexapod/trajectory/joint_limits.py
config/hardware/servo2040.json
config/robots/standard-joint-soft-limits.json
```

Servo 2040:

```text
firmware/servo2040/src/hexapod_mcu/
firmware/servo2040/src/config/actuator-profile.json
```

## Contract categories

### Protocol primitives

The suite locks the shared protocol identity and framing contract:

- `HX1` protocol prefix;
- protocol major/minor;
- 512-byte frame bound;
- 32-character message-type bound;
- uint32 sequence space and half-range ordering;
- CRC-16/CCITT-FALSE;
- line-framer fragmentation and oversize recovery;
- current runtime-state and NACK vocabularies.

The Pi encoder is parsed by the MCU parser and the MCU encoder is parsed by the
Pi parser.

Representative frames must be byte-for-byte identical when both encoders are
given the same logical fields.

### Profile and joint semantics

The suite verifies that the Pi's expected MCU peer policy matches the exact
current actuator profile:

- profile ID;
- profile revision;
- exact-byte SHA-256;
- joint count.

It also verifies that the canonical 18-joint order is identical on both sides
and that every Pi planning soft limit fits inside the corresponding MCU hard
logical envelope.

This supplements, rather than replaces, the existing Pi gait-envelope tests.

### Timing contract

The configured Pi transmission cadences are checked against the MCU contract:

```text
heartbeat  10 Hz / 100 ms
target     50 Hz / 20 ms
```

The normal cadences must also remain inside the authoritative MCU watchdog
timeouts.

### Pi command schemas

Every Pi command builder is parsed by the real MCU framing parser, including:

```text
HELLO
HEARTBEAT
STAGE
ARM
START
TARGET
STOP
DISARM
ESTOP
CLEAR_ESTOP
CLEAR_FAULT
GET_STATUS
```

This catches argument-count, ordering, session, period, and framing drift.

### MCU telemetry schemas

The end-to-end tests cause the real MCU runtime/telemetry encoder to emit frames
that are consumed by the real Pi typed parser/client.

The exercised response types include:

```text
INFO
STATUS
ACK
NACK
EVENT
```

Periodic pre-session `STATUS` is also verified because the MCU may emit status
before the Pi has negotiated a session.

## End-to-end runtime flows

### Unqualified-profile diagnostic flow

A host-only profile variant with unqualified actuator rates proves the safe
diagnostic path:

```text
MCU SELF_TEST
    -> FAULT / PROFILE

Pi HELLO
    -> MCU INFO
    -> Pi accepts matching peer/session

Pi GET_STATUS
    -> MCU STATUS
    -> Pi sees FAULT / PROFILE
```

The test proves that an actuator profile can intentionally block energization
without blocking HX1 diagnostics.

### Qualified lifecycle flow

A second host-only profile variant supplies deterministic non-null hard rate
limits solely so the complete control wire contract can be exercised without
changing the deployed actuator profile.

It proves:

```text
HELLO -> INFO
STAGE -> ACK
HEARTBEAT
ARM -> ACK
START -> ACK
TARGET -> no normal ACK
GET_STATUS -> STATUS
STOP -> ACK
DISARM -> ACK
```

The synthetic qualified profile exists only inside the host test process. It is
not an actuator qualification result and must never be copied to the robot as
physical calibration.

### Profile mismatch

The suite verifies the intentional asymmetric behavior:

1. the MCU may establish a diagnostic session while marking the expected
   profile as mismatched;
2. `INFO` reports the MCU's actual profile;
3. the Pi rejects local control authority when the actual peer profile does not
   match its configured expectation.

### Zero-session E-stop

A Pi client with no negotiated session sends:

```text
ESTOP|00000000|...
```

The real MCU runtime must accept it and return typed `ACK` and `EVENT` frames.

## Release-gate rule

Before promoting a Servo 2040 firmware candidate, all three layers must pass:

```text
Pi unit suite
Servo 2040 firmware unit suite
Pi/Servo 2040 cross-boundary contract suite
```

A failing contract test is not fixed by weakening the test to match whichever
side changed.

First resolve the intended behavior against:

```text
docs/architecture/control-boundary.md
docs/architecture/runtime-state-machine.md
docs/protocols/pi-servo2040-v1.md
docs/decisions/
```

Then update both implementations and the contract test deliberately when the
shared contract itself has changed.

## Hardware scope

These tests are host-side and perform no USB, PWM, or real servo I/O.

They are a prerequisite for hardware-in-the-loop testing, not a replacement for
it.

The first physical HIL gate remains diagnostic-only:

```text
OPEN
HELLO
INFO
GET_STATUS
STATUS
CLOSE
```

No physical `STAGE`, `ARM`, `START`, or `TARGET` belongs in that first gate.
