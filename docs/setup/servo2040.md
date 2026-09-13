# Servo 2040 Setup

## Validated board baseline

Current controller:

```text
Board           Pimoroni Servo 2040
MCU             RP2040
MicroPython     1.23.0
mpremote        1.29.0
HX1 minor       1
```

Current board identity:

```text
e661410403724132
```

## Dual-CDC architecture

Release candidate `0.1.0-rc4` moves HX1 off MicroPython's built-in
`sys.stdin` / `sys.stdout` CDC stream and onto a second runtime USB CDC
interface with buffered bulk endpoints.

The USB interfaces have separate roles:

```text
if00  built-in MicroPython CDC   REPL / mpremote maintenance
if02  runtime CDC                HX1 robot-control protocol only
```

The Raspberry Pi HX1 client must therefore use:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if02
```

The maintenance interface remains:

```text
/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
```

The configured HX1 device path lives in:

```text
config/hardware/servo2040.json
```

Do not hardcode `/dev/ttyACM0` or `/dev/ttyACM1`; interface numbering can move
while the stable by-id path remains authoritative.

## Why HX1 uses a dedicated CDC interface

Physical HIL showed that the built-in MicroPython stdin path could not sustain
HX1 traffic. A 102-byte TARGET-sized echo workload produced approximately
150-640 ms round-trip latency and a growing receive backlog.

A temporary second-CDC benchmark on the same Servo 2040 produced:

```text
50 Hz   150/150 returned   median 2.803 ms   P99 4.912 ms
100 Hz  300/300 returned   median 2.816 ms   P99 5.110 ms
200 Hz  598/598 returned   median 5.054 ms   P99 9.844 ms
```

No packets were missing or malformed in those qualification runs. The 50 Hz
production target cadence therefore has substantial transport headroom.

## Vendored USB dependencies

The firmware artifact vendors the MicroPython USB modules it depends on under:

```text
firmware/servo2040/src/lib/usb/
```

Qualified package versions:

```text
usb-device      0.2.1
usb-device-cdc  0.1.4
```

Deployment must copy the `lib/usb` tree with the rest of the firmware. Do not
rely on a separately-installed board package as the production dependency.

## Active firmware candidates

Last previously validated checkpoint:

```text
Firmware tag      servo2040-v0.1.0-rc3
Firmware version  0.1.0-rc3
Profile            hexapod-standard-v1
Profile revision   3
Profile SHA-256    DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177
```

Current transport candidate:

```text
Firmware version  0.1.0-rc4
Profile            hexapod-standard-v1
Profile revision   3
Profile SHA-256    DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177
Change             dedicated buffered HX1 CDC interface
```

Do not tag rc4 as physically validated until the DISARMED HX1 throughput test
and the supported stationary ARM/START/TARGET/STOP/DISARM lifecycle both pass.

## Maintenance access

`mpremote` is installed with pipx rather than inside the Hexapod project venv.

Example:

```bash
DEVICE=/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
mpremote connect "$DEVICE"
```

The runtime CDC is not the maintenance port. Use `if00` for REPL and file
management even after rc4 is installed.

## Deployment rule

Treat a Servo 2040 deployment as a versioned hardware artifact:

1. deploy only files from a known Git commit/tag;
2. deploy the complete `firmware/servo2040/src/lib/usb` dependency tree;
3. verify both `if00` and `if02` enumerate after reboot;
4. verify firmware version through HX1 on `if02`;
5. verify profile ID/revision and exact profile SHA-256;
6. verify boot state is `DISARMED` / `NONE` before energized testing;
7. perform DISARMED transport qualification before ARM;
8. record physical HIL evidence before tagging a new release candidate.

GitHub-hosted CI cannot replace these physical checks.
