# ADR-0008: Servo 2040 Runtime Hardening Invariants

**Status:** Accepted for implementation  
**Date:** 2026-09-13

## Context

The first complete host-tested Servo 2040 firmware checkpoint passed 165 tests,
but a final safety review identified several failure paths that were independent
of the still-evolving Raspberry Pi motion stack.

The firmware must remain safe when:

- USB input contains Ctrl-C;
- a heartbeat or target arrives at or after an already-expired watchdog deadline;
- disabling PWM reports a backend failure;
- malformed or pathological protocol input reaches error-reporting paths;
- an exception escapes a command handler;
- a fault is entered from a state that was not already energized.

## Decision

### USB CDC owns the input stream

Production firmware disables MicroPython keyboard-interrupt handling with:

```text
micropython.kbd_intr(-1)
```

The USB CDC stream is reserved for HX1 while the production runtime is active.
The top-level firmware entrypoint also performs a best-effort `force_disabled()`
in a `finally` block so `KeyboardInterrupt`, `SystemExit`, and other top-level
exit paths do not intentionally leave PWM authority active.

### Watchdog deadlines are not retroactively rescuable

A command that may refresh or extend authority must first observe any watchdog
deadline that has already expired.

At an exact timeout boundary, the timeout wins.

The following commands may execute without this pre-check because they remove
or replace control authority:

```text
ESTOP
DISARM
HELLO
```

### PWM state may be unknown

The hardware adapter distinguishes:

```text
True  = outputs are known enabled
False = outputs are known disabled
None  = physical output state is unknown
```

A failed disable operation produces `None`, never `False`.

When the runtime requires PWM to be off and hardware state is `True` or `None`,
it continues attempting `force_disabled()` on subsequent loop iterations.
Repeated failures do not repeatedly emit the same fault event.

### Fault hold cannot create PWM authority

A watchdog fault grace hold may preserve outputs only if the state immediately
before the fault was `ARMED` or `ACTIVE` and a commanded target exists.

A fault entered from `DISARMED`, `BOOTING`, or another non-energized state must
never cause `pwm_should_be_enabled()` to become true.

### E-stop fault clearing defaults safe

If a caller omits explicit proof that an underlying fault has cleared,
`clear_estop()` treats that proof as false. A retained fault therefore cannot be
accidentally erased by relying on a permissive default.

### Protocol error paths are bounded

HX1 message type identifiers are limited to 32 ASCII characters.

Unexpected exceptions during command dispatch or while constructing an error
response are contained at both the runtime-coordinator and scheduler boundaries.
The MCU attempts to latch `INTERNAL` fault state and disable PWM rather than
allowing the exception to terminate the firmware loop.

`ESTOP` reason identifiers use the same compact token character set used by
outbound telemetry so a syntactically valid but semantically invalid reason is
rejected with `ERR_BAD_VALUE` rather than turning into an internal exception.

## Consequences

- Late traffic cannot erase an already-missed safety deadline.
- Software never claims PWM is off after a reported disable failure.
- Unknown electrical state triggers continued disable attempts.
- Ctrl-C on the production protocol stream no longer acts as a normal REPL
  interrupt.
- Pathological frames cannot use the error path to escape the safety loop.
- The release checkpoint remains intentionally non-armable until actuator rates
  and backend-compatible physical limits are qualified.

## Related

- `docs/decisions/0003-servo2040-runtime-safety.md`
- `docs/decisions/0005-session-replacement-and-estop-recovery.md`
- `docs/decisions/0007-fault-recovery-proof.md`
- `docs/protocols/pi-servo2040-v1.md`
