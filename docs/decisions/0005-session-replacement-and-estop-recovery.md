# ADR-0005: Session Replacement and E-stop Recovery Semantics

**Status:** Accepted for implementation  
**Date:** 2026-09-12

## Context

The accepted protocol and runtime-safety documents define session replacement,
latched faults, and E-stop recovery, but leave two edge cases implicit:

1. what happens if a new `HELLO` creates a replacement session while the MCU is
   already `ARMED` or `ACTIVE`;
2. whether entering and clearing `ESTOP` can erase or bypass an underlying fault.

Both cases must be deterministic before implementing the runtime state machine.

## Decision

### Session replacement while energized

A successfully established new session invalidates the prior session's movement
authority.

If a new session is established while the MCU is:

```text
ARMED
ACTIVE
```

the MCU immediately disables movement authority and transitions to:

```text
DISARMED
```

PWM is disabled.

The newly established session remains valid, but the host must provide:

```text
fresh STAGE
explicit ARM
```

before PWM may be enabled again.

The MCU does not automatically restore the previous held target as authorization
to re-arm.

This behavior is fail-safe and makes host reconnect behavior deterministic.

### Session replacement in other states

A new session does not clear:

```text
FAULT
ESTOP
```

and does not bypass self-test.

In safe states, session replacement invalidates:

- staged target;
- target-sequence history;
- heartbeat freshness;
- motion-target freshness.

### E-stop over an existing fault

`ESTOP` remains the dominant runtime state while active.

If an underlying fault exists when E-stop is entered, or a safety fault is
detected while E-stop is active, that fault condition is retained.

`CLEAR_ESTOP` succeeds only when:

- the E-stop release condition is satisfied;
- the supplied session is current;
- any retained underlying fault condition has cleared.

If an underlying fault is still active, `CLEAR_ESTOP` is rejected with:

```text
ERR_FAULT_ACTIVE
```

and the MCU remains in `ESTOP`.

A successful `CLEAR_ESTOP` still returns only to:

```text
DISARMED
```

and never re-arms automatically.

## Rationale

A replacement host session must not inherit energized authority from a previous
host process.

Likewise, an operator must not be able to bypass a hardware or watchdog-related
fault by entering E-stop and then clearing E-stop.

Both rules favor explicit re-authorization and preserve the v1 principle that
safe recovery returns to `DISARMED`.

## Consequences

- Host reconnect while energized causes a safe loss of PWM rather than seamless
  control transfer.
- Reconnection requires a fresh stage-and-arm sequence.
- E-stop cannot be used as a fault-clearing shortcut.
- The state machine can be tested deterministically without hardware I/O.

## Related

- `docs/architecture/runtime-state-machine.md`
- `docs/protocols/pi-servo2040-v1.md`
- `docs/decisions/0003-servo2040-runtime-safety.md`
