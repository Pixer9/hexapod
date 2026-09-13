# Servo 2040 Setup

## Validated board baseline

Current validated controller:

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

The Raspberry Pi uses the stable USB serial by-id path configured in:

```text
config/hardware/servo2040.json
```

## Active firmware

Deployable firmware source lives under:

```text
firmware/servo2040/src/
```

The legacy tree under:

```text
firmware/servo2040/legacy-backup/
```

is archival and must not be deployed as current firmware.

## Current release candidate

The current validated controller checkpoint is:

```text
Firmware tag      servo2040-v0.1.0-rc3
Firmware version  0.1.0-rc3
Profile            hexapod-standard-v1
Profile revision   3
Profile SHA-256    DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177
```

Revision 3 is arm-qualified for controlled HIL/commissioning. That does not
mean integrated gait or loaded walking validation is complete.

## Maintenance access

`mpremote` is installed with pipx rather than inside the Hexapod project venv.

Example:

```bash
DEVICE=/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e661410403724132-if00
mpremote connect "$DEVICE"
```

## Deployment rule

Treat a Servo 2040 deployment as a versioned hardware artifact:

1. deploy only files from a known Git commit/tag;
2. verify firmware version after deployment;
3. verify profile ID/revision;
4. verify the exact profile SHA-256;
5. verify boot state before energized motion testing;
6. record physical HIL evidence before tagging a new release candidate.

GitHub-hosted CI cannot replace these physical checks.
