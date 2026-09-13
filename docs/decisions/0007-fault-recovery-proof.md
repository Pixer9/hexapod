# ADR-0007: Fault Recovery Proof for Protocol v1

**Status:** Accepted for implementation  
**Date:** 2026-09-13

## Context

`CLEAR_FAULT` and `CLEAR_ESTOP` are allowed only when the underlying unsafe
condition is no longer active.

Protocol v1 needs deterministic proof rules for the faults currently generated
by the release-candidate MCU runtime.

## Decision

The runtime uses the following proof rules:

### Link watchdog timeout

`LINK_TIMEOUT` is considered cleared only after a valid heartbeat for the
current session is fresh again according to `LINK_TIMEOUT_MS`.

### Motion watchdog timeout

`MOTION_TIMEOUT` is considered cleared only after a valid heartbeat for the
current session is fresh again.

A fresh motion target is not required while the MCU is in `FAULT`, because
normal `TARGET` commands are not accepted in that state. After fault clearing,
the MCU returns to `DISARMED`; the host must stage, arm, and start again before
motion can resume.

### Profile, hardware, internal, and self-test faults

The following faults are not clearable through the normal runtime protocol:

```text
PROFILE
HARDWARE
INTERNAL
SELF_TEST
```

They require MCU restart or a future explicit diagnostic/maintenance path that
can prove the underlying condition has been corrected.

### E-stop with retained fault

If an underlying fault is retained while `ESTOP` is active, `CLEAR_ESTOP`
cannot bypass that fault. The same proof rules above must be satisfied first.

## Rationale

The normal runtime protocol should only clear faults whose recovery condition
can be proven using information already available to the safety runtime.

Hardware, profile, internal, and self-test failures require stronger evidence
than the normal runtime command channel currently provides.

## Consequences

- Watchdog recovery is deterministic and testable.
- Motion timeout recovery does not require accepting movement while faulted.
- A hardware or profile failure cannot be cleared merely by issuing a command.
- Future maintenance/calibration functionality must define its own explicit
  recovery proof rather than weakening this rule.

## Related

- `docs/architecture/runtime-state-machine.md`
- `docs/protocols/pi-servo2040-v1.md`
- `docs/decisions/0003-servo2040-runtime-safety.md`
- `docs/decisions/0005-session-replacement-and-estop-recovery.md`
