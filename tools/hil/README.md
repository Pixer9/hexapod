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

## Stationary energized runtime

Milestone 2 adds the first energized production-runtime HIL tool:

```bash
python tools/hil/stationary_runtime.py --help
```

The tool performs:

```text
HELLO
GET_STATUS
STAGE
HEARTBEAT
ARM
HEARTBEAT
START
stationary TARGET stream
STOP
DISARM
GET_STATUS
```

The `TARGET` stream remains at the exact configured flat-stance vector. It does
not execute a gait.

This test can still cause physical motion when `ARM` first energizes an
unpowered servo and drives it toward the staged flat stance.

For that reason the tool refuses to run unless the operator explicitly supplies:

```text
--confirm-energized-hil
```

The robot must be mechanically supported before using that flag.

The default ACTIVE duration is two seconds. The commissioning tool limits a
single invocation to at most ten seconds:

```bash
python tools/hil/stationary_runtime.py \
  --duration 2 \
  --confirm-energized-hil
```

Do not use the energized tool as a substitute for the read-only probe when only
link/profile/status validation is intended.

## Current validated checkpoint

The read-only probe has been validated against the Servo 2040 release candidate
documented in `docs/setup/servo2040.md`.

Physical HIL results should be interpreted according to the scope of each tool.

The read-only probe verifies HX1 negotiation, identity compatibility, session
handling, and status retrieval.

The stationary energized runtime is intended to validate the next commissioning
checkpoint: production Pi lifecycle scheduling and stationary target streaming.
It does not qualify gait execution, loaded walking, watchdog recovery, E-stop
recovery, or sensor behavior.
