# Pi HX1 Synchronous Link Pump

**Status:** Deterministic synchronous composition baseline

## Purpose

The HX1 link pump is the thin composition layer between:

- `HX1ClientCore`;
- `HX1LineFramer`; and
- an `HX1Transport`.

It provides the first complete Pi-side path from already-built HX1 commands to
raw transport bytes and from raw incoming bytes back to typed HX1 messages.

```text
outbound

HX1ClientCore
    |
    | HX1Outbound
    v
HX1Link.send()
    |
    v
HX1Transport.write()


inbound

HX1Transport.read()
    |
    v
HX1LineFramer.feed()
    |
    v
HX1ClientCore.accept_frame()
    |
    v
typed HX1 message
```

This layer is intentionally synchronous.

It does not create a worker thread, own a wall clock, sleep, reconnect
automatically, schedule heartbeats, schedule targets, or decide robot lifecycle
transitions.

## Open and close

`HX1Link.open()` opens the transport only.

It sends no `HELLO`, heartbeat, status request, or other HX1 command.

Opening a previously closed link clears local session authority and starts a
fresh receive-framing context.

Calling `open()` while the link is already open is idempotent and does not
destroy an already-negotiated session.

`HX1Link.close()` closes the transport and always clears local session
authority.

A session is therefore never intentionally carried across a physical/logical
link close.

## Sending

The caller first uses `HX1ClientCore` to build an outbound command:

```python
hello = client.hello()
```

and then explicitly sends it:

```python
link.send(hello)
```

`send()` accepts only `HX1Outbound`.

The method verifies that the transport accepted the complete encoded frame. A
short/incomplete transport acceptance is treated as a link error.

A local successful send does not mean the MCU accepted the command.

## Polling

`HX1Link.poll()` performs exactly one non-blocking transport read.

If no bytes are available:

```python
messages = link.poll()
```

returns:

```python
()
```

If bytes are available, they are passed through the bounded
`HX1LineFramer`. Any complete lines are then passed to
`HX1ClientCore.accept_frame()` in wire order.

A frame may be split across any number of calls to `poll()`.

Several complete frames received in one byte chunk are returned in wire order.

## Malformed inbound traffic

Framing/CRC/structural protocol errors are dropped and counted as:

```text
protocol_errors
```

Semantically malformed typed messages are dropped and counted as:

```text
message_errors
```

The line framer separately exposes:

```text
framing_errors
```

This allows noise or malformed traffic to be diagnosed without converting every
bad frame into a process-level failure.

Negotiation errors are different.

For example:

- unexpected `INFO`;
- `INFO` for the wrong HELLO;
- actuator profile mismatch;
- unsupported required server minor.

Those remain explicit `HX1ClientCore` errors and are not silently swallowed by
the link pump.

## Transport failure

A read or write transport failure invalidates local HX1 session authority before
the transport error is re-raised.

The link pump does not automatically reconnect.

The future runtime layer decides whether and when reconnect is appropriate.

Any successful reconnect must establish a new HX1 session before normal
session-bound commands may resume.

## Deliberate non-goals

This layer does not yet implement:

- background serial I/O;
- reconnect/backoff;
- HELLO timeout/retry;
- ACK timeout/retry;
- heartbeat scheduling;
- periodic status requests;
- 50 Hz target scheduling;
- runtime state orchestration;
- automatic `STAGE`;
- automatic `ARM`;
- automatic `START`;
- physical actuator qualification.

Those behaviors remain separate so the link boundary can be verified before
time-dependent runtime behavior is introduced.

## First physical HIL use

Once the synchronous link is accepted and pyserial is installed, the first
real-board exercise should remain read/control-plane only:

```text
open
HELLO
INFO
GET_STATUS
STATUS
close
```

No `STAGE`, `ARM`, `START`, or `TARGET` is required for that test.

The current Servo 2040 actuator profile is not arm-qualified, so physical
energization remains intentionally out of scope.
