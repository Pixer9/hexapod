# Pi HX1 Transport Boundary

**Status:** Deterministic raw-byte transport contract and fake implementation

## Purpose

HX1 protocol/session behavior and physical byte transport are separate layers.

```text
HX1ClientCore
    |
    | complete outbound HX1 frame bytes
    v
HX1Transport
    |
    | raw byte stream
    v
Servo 2040 connection
```

The transport does not understand:

- HX1 message types;
- sequence numbers;
- sessions;
- CRC;
- robot lifecycle;
- joint vectors;
- safety state.

It only opens/closes a byte channel and moves bytes.

This separation prevents serial-port behavior from leaking into protocol,
session, motion, or safety logic.

## Contract

The transport interface is intentionally small:

```python
transport.open()
transport.close()

transport.is_open

transport.write(data) -> int
transport.read(max_bytes=4096) -> bytes
```

`open()` and `close()` are idempotent.

`write()` accepts raw `bytes` or `bytearray` and returns the number of bytes
accepted.

`read()` is non-blocking. It returns:

- up to `max_bytes` currently available bytes; or
- `b""` when no bytes are currently available.

The transport never waits for a complete HX1 line.

Complete-line reconstruction belongs to `HX1LineFramer`.

## Layer composition

The future runtime/link layer will compose the pieces as:

```text
serial/read bytes
    |
    v
HX1Transport.read()
    |
    v
HX1LineFramer.feed()
    |
    v
complete frame bytes
    |
    v
HX1ClientCore.accept_frame()
```

Outbound traffic flows in the opposite direction:

```text
HX1ClientCore command method
    |
    v
HX1Outbound.frame
    |
    v
HX1Transport.write()
```

This design keeps framing deterministic even when USB CDC divides one HX1 frame
across multiple reads or returns multiple complete frames in one read.

## Fake transport

`FakeHX1Transport` is the deterministic host-side implementation.

It provides explicit test helpers:

```python
transport.inject_read_data(peer_bytes)
transport.writes
transport.take_writes()
transport.pending_read_bytes
transport.clear()
```

It has no background activity and no implicit simulated MCU.

If a test wants an `INFO`, `ACK`, `NACK`, `STATUS`, or `EVENT`, the test must
explicitly inject those bytes.

This makes protocol tests reproducible and prevents fake behavior from hiding
missing runtime logic.

## Closed-state behavior

Normal read/write operations against a closed transport raise:

```text
HX1TransportClosedError
```

Test-side peer injection is allowed while the fake transport is closed so a
deterministic response may be scripted before opening the link.

Closing the fake does not discard queued bytes. Connection-lifecycle policy
belongs to the future link/reconnect layer, not the raw transport.

## Real serial implementation

The next implementation will be:

```text
SerialHX1Transport
```

using the configured stable device:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
```

The real adapter must preserve this contract, including non-blocking reads.

It will not:

- create sessions;
- schedule heartbeats;
- schedule targets;
- retry commands;
- automatically reconnect;
- ARM or START the MCU.

Those behaviors belong above the raw transport boundary.

## Safety

The transport is not a safety authority.

A successful write means only that bytes were accepted by the local transport
implementation. It does not mean:

- the MCU parsed the frame;
- the MCU accepted the command;
- a watchdog was refreshed;
- motion was authorized;
- a target was applied.

Those truths come only from HX1 protocol semantics and MCU state/telemetry.
