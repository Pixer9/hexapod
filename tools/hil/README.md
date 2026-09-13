# Hardware-in-the-Loop Tools

This directory contains Raspberry Pi tools that interact with physical Hexapod
hardware.

These tools are intentionally separate from host-only automated tests and
GitHub-hosted CI. A passing CI run does not replace physical hardware
validation.

## HX1 read-only probe

Run:

```bash
python tools/hil/hx1_probe.py
```

The probe opens the configured Servo 2040 serial link and performs only:

1. `HELLO` / `INFO` negotiation.
2. `GET_STATUS` / `STATUS`.

It does not send actuator-motion or lifecycle-changing commands such as
`STAGE`, `ARM`, `START`, `TARGET`, `STOP`, `DISARM`, `ESTOP`,
`CLEAR_ESTOP`, or `CLEAR_FAULT`.

The default configuration is:

```text
config/hardware/servo2040.json
```

A different configuration file may be supplied explicitly:

```bash
python tools/hil/hx1_probe.py --config path/to/servo2040.json
```

Use `--help` to inspect the CLI without opening hardware:

```bash
python tools/hil/hx1_probe.py --help
```

## Current validated checkpoint

The read-only probe has been validated against the Servo 2040 release candidate
documented in `docs/setup/servo2040.md`.

Physical HIL results should be interpreted according to the scope of the tool.
This probe verifies HX1 negotiation, identity compatibility, session handling,
and status retrieval. It does not qualify energized actuator motion, watchdog
behavior, E-stop behavior, recovery behavior, gait execution, or loaded
walking.
