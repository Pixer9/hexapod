# ADR 0009: Binary Pi-to-MCU HX1 command wire

## Status

Accepted and HIL-qualified.

## Context

The dedicated Servo 2040 USB CDC link itself has demonstrated substantial
headroom, but the original ASCII HX1 command representation could not sustain
the required 50 Hz control rate on MicroPython. A DISARMED 50 Hz STAGE test
showed queue growth even after CRC and line-framing optimization.

On-board profiling isolated the remaining cost:

- `parse_frame(STAGE)`: 6.817 ms/call
- 18 decimal joint conversions: 10.478 ms/call
- complete `RuntimeCoordinator.handle_frame(STAGE)`: 23.616 ms/call

A follow-up benchmark on the actual Servo 2040 measured a representative binary
TARGET decode plus CRC at 1.183 ms/call, while decimal conversion of the 18
joints alone cost 10.593 ms/call.

## Decision

HX1 keeps its existing lifecycle, session, sequence, watchdog, profile, and
safety semantics, but uses asymmetric wire representations:

- Pi -> Servo 2040 commands: compact binary frames.
- Servo 2040 -> Pi INFO/ACK/NACK/EVENT/STATUS: existing ASCII HX1 frames.

The binary command header is little-endian:

| Field | Size |
| --- | ---: |
| Magic `HX` | 2 bytes |
| Command wire version | 1 byte |
| Command ID | 1 byte |
| Payload length | 2 bytes |
| Sequence | 4 bytes |
| Session | 4 bytes |
| Typed payload | 0-96 bytes |
| CRC-16/CCITT-FALSE | 2 bytes |

CRC covers the complete header and payload, excluding only the final two CRC
bytes. Fixed-size commands have their payload length validated by the MCU
framer before a frame is accepted.

The 18 logical joint commands remain signed integer centidegrees. STAGE packs
`18h` (36 payload bytes) and TARGET packs `I18h` (period plus 18 joints).
Therefore STAGE is 52 bytes total and TARGET is 56 bytes total.

## Safety and compatibility

The production MCU runtime calls the binary-only command decoder. Legacy ASCII
commands are not accepted as authority-bearing commands. Old/new host-MCU
mismatches therefore fail closed during negotiation rather than silently
falling back.

MCU telemetry remains ASCII so field diagnostics, existing host parsing, and
operator visibility are retained. The public Pi-side client API is unchanged;
only the serialized `HX1Outbound.frame` representation changes.

## Qualification gate

Before any energized HIL test, v3 must pass the DISARMED STAGE
qualification at 50 Hz for 250 commands with:

- 100% ACKs
- zero NACKs, missing ACKs, and duplicate ACKs
- median ACK latency <= 20 ms
- maximum ACK latency <= 40 ms
- second-half average minus first-half average <= +5 ms
- final MCU state `DISARMED`, fault `NONE`

P95 and P99 ACK latency remain diagnostic measurements but are not release
gates. STAGE produces an ASCII ACK for every command, while production TARGET
traffic is intentionally unacknowledged; therefore STAGE round-trip tail
latency is not equivalent to production TARGET delivery latency.

Qualification on the Servo 2040 produced:

- 250/250 ACKs
- zero protocol errors
- median 17.802 ms
- P99 39.184 ms
- maximum 39.462 ms
- trend +1.202 ms
- final state `DISARMED`, fault `NONE`

Periodic unsolicited STATUS telemetry is disabled by default for rc5 after
on-MCU profiling measured STATUS generation at approximately 14.3 ms per
frame and HIL testing demonstrated that it introduced recurring scheduler
backlog. GET_STATUS remains available for explicit status queries. Periodic
telemetry may be reintroduced later using a lower-cost representation or
lower-priority scheduling.
