# DualShock 4 Input Boundary

**Status:** Initial implementation baseline

## Purpose

The DS4 adapter converts Linux evdev controller state into the shared
`MotionCommand` / `CommandSample` boundary.

It does not know about gait phase, foot positions, IK, joint vectors, HX1, or
Servo 2040 channels.

```text
Linux evdev
    |
    v
DS4 adapter
    |
    v
MotionCommand
    |
    v
CommandSample(source=DS4)
    |
    v
normal command arbiter
```

## Configuration

DS4-specific behavior lives in:

```text
config/inputs/ds4.json
```

Configurable items include:

- device-path override;
- device-name discovery fragments;
- axis names;
- axis inversion;
- deadzone;
- expo;
- command freshness duration;
- initial motion-enable state;
- button mappings.

The shared physical motion maxima remain in `config/control/motion.json`.
They are not duplicated in the DS4 config.

## V4 migration

The default mapping preserves the previously working v4 mapping:

```text
vx       <- -ABS_Y
vy       <- -ABS_X
yaw_rate <- -ABS_RX
deadzone = 0.10
expo     = 0.35
```

V4 explicitly documented its mapping as device/kernel-specific and
"non-standard." Therefore these signs are configuration and MUST be verified
on the actual Raspberry Pi / DS4 combination before they are treated as final.

## Freshness and held sticks

evdev generally reports an absolute-axis event when the axis changes. A stick
held at a constant non-zero position may therefore produce no new axis events.

For that reason DS4 command freshness is based on the runtime publishing its
current connected controller state, not on the time of the last axis event.

While the evdev device remains connected:

```text
current stick state
    -> new CommandSample(now)
    -> expires now + command_ttl_s
```

If the input device disconnects, raises a reader error, or its evdev read loop
ends unexpectedly, the adapter stops publishing DS4 commands and emits both
`DISCONNECTED` and `ESTOP` actions for the future safety supervisor.

An intentional adapter shutdown via `stop()` is quiet and does not manufacture
disconnect or emergency-stop actions.

This deliberately replaces the v4 "idle event timeout" behavior.

## Motion enable

The initial config starts motion disabled.

Pressing the configured Start button toggles DS4 motion enable. A connected,
disabled DS4 still publishes a fresh zero command, so the human operator retains
normal-source authority over web/autonomy while intentionally stopped.

Motion enable is not the same thing as Servo 2040 arming. The future runtime
state/safety layer will decide when MCU arming is allowed.

## Safety actions

Controller buttons produce intents:

```text
BTN_MODE    -> ESTOP
BTN_SELECT  -> CLEAR_ESTOP_REQUEST
```

The adapter does not directly clear robot safety state. A future safety
supervisor will validate and act on those requests.

## Device discovery

When `device.path` is null, the evdev adapter scans input nodes and requires
both:

1. a configured controller-name match; and
2. all configured motion axes on the same evdev node.

This avoids accidentally opening a DS4 motion-sensor or touchpad node merely
because its device name resembles the controller.

An explicit device path may be configured later if stable platform naming makes
that preferable.
