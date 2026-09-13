# ADR-0006: STATUS Commanded-Vector Validity

**Status:** Accepted for implementation  
**Date:** 2026-09-13

## Context

Protocol v1 originally required every `STATUS` frame to include `J0...J17` as
the current commanded logical joint vector.

Before the first successful `ARM`, however, no commanded logical target exists.
`GET_STATUS` is still expected to work in that condition.

Using a legal joint vector as a placeholder would make diagnostic state
ambiguous. Omitting the vector would make `STATUS` variable-length.

## Decision

Protocol minor version is incremented from:

```text
1.0
```

to:

```text
1.1
```

`STATUS` gains one integer field:

```text
COMMAND_VALID
```

The schema becomes:

```text
HX1|SEQ|STATUS|SESSION|STATE|FAULT|LAST_TARGET_SEQ|TARGET_AGE_MS|HEARTBEAT_AGE_MS|FOOT_MASK|BUS_MV|BUS_MA|UPTIME_MS|COMMAND_VALID|J0|J1|...|J17|CRC16
```

`COMMAND_VALID` is:

```text
0 = J0...J17 are placeholders and must not be interpreted as a commanded pose
1 = J0...J17 contain the last meaningful commanded logical target
```

When `COMMAND_VALID = 0`, the MCU transmits zero for all joint fields.

The validity flag is diagnostic only. It does not imply PWM is enabled or that
the target is authorized for future motion.

For example, a previously commanded vector may remain diagnostically valid
after a disarm while PWM is off.

The vector remains commanded state only. It is never measured servo position.

## Rationale

An explicit validity bit is deterministic, fixed-width, and does not overload a
legal joint angle or state transition with sentinel semantics.

## Consequences

- `PROTOCOL_MINOR` becomes `1`.
- Pi-side STATUS parsing must include `COMMAND_VALID`.
- A protocol-1.0 client must not assume the protocol-1.1 STATUS field layout.
- The release-candidate Pi client and MCU firmware will be built against the
  same protocol-minor contract before hardware deployment.

## Related

- `docs/protocols/pi-servo2040-v1.md`
- `docs/architecture/runtime-state-machine.md`
