# Pi HX1 Transport Boundary

**Status:** Deterministic fake transport plus physical pyserial adapter

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

`write()` accepts raw `bytes` or `bytearray` and returns only after the complete
supplied byte sequence has been accepted by the local transport backend.

`read()` is non-blocking. It returns:

- up to `max_bytes` currently available bytes; or
- `b""` when no bytes are currently available.

The transport never waits for a complete HX1 line.

Complete-line reconstruction belongs to `HX1LineFramer`.

## Layer composition

The future runtime/link layer composes the pieces as:

```text
serial bytes
    |
    v
HX1Transport.read()
    |
    v
HX1LineFramer.feed()
    |
    v
complete HX1 frame bytes
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

This keeps framing deterministic when USB CDC divides one HX1 frame across
multiple reads or combines multiple frames in one read.

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

## Physical serial transport

`SerialHX1Transport` is the Raspberry Pi Linux implementation.

It uses pyserial but imports that dependency lazily. Pure HX1 modules and unit
tests therefore do not require pyserial merely to import `hexapod.hx1`.

The physical device path comes from:

```text
config/hardware/servo2040.json
```

and is currently:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
```

The adapter never depends on `/dev/ttyACM0`.

Current serial settings:

```text
baudrate       115200
read timeout   0 seconds
write timeout  0.25 seconds
exclusive      true
software flow  disabled
RTS/CTS        disabled
DSR/DTR        disabled
```

The baudrate is the host-side CDC line-coding value. The Servo 2040
MicroPython USB CDC implementation exposes the link through `sys.stdin` and
`sys.stdout`; it does not implement a physical UART baud clock.

The zero read timeout is not configurable because the `HX1Transport` contract
requires non-blocking reads.

Exclusive access is requested on Linux so another process cannot silently share
the Servo 2040 control link.

## Short writes

The transport contract requires a complete supplied byte sequence to be
accepted locally.

If pyserial reports a short write, `SerialHX1Transport` continues writing the
remaining bytes until either:

- all bytes have been accepted; or
- the backend fails or stops making forward progress.

A zero-progress write is treated as a transport error rather than retried
forever.

A successful local write does not mean the MCU accepted the HX1 command.

## Errors

Closed-state I/O raises:

```text
HX1TransportClosedError
```

Physical open/read/write/close failures are surfaced as:

```text
HX1TransportError
```

If the production adapter is opened without pyserial installed, it raises:

```text
HX1SerialDependencyError
```

The raw transport does not automatically reconnect after failure.

Reconnect/session-replacement behavior belongs to the future HX1 link/runtime
layer.

## Dependency

The Raspberry Pi project venv will require:

```text
pyserial
```

before physical serial use.

The dependency does not need to be installed to run fake-transport or protocol
tests because the import is lazy.

## Safety

The transport is not a safety authority.

A successful write means only that bytes were accepted by the local serial
backend. It does not mean:

- the MCU parsed the frame;
- the MCU accepted the command;
- a watchdog was refreshed;
- motion was authorized;
- a target was applied.

Those truths come only from HX1 protocol semantics and MCU state/telemetry.

`SerialHX1Transport` itself never creates a session, sends a heartbeat, sends a
target, arms the MCU, starts motion, or reconnects automatically.
