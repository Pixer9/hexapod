# Raspberry Pi Motion Command and Arbitration

**Status:** Initial implementation baseline

## Purpose

All normal control producers emit the same hardware-independent planar motion
command:

```text
vx_mm_s
vy_mm_s
yaw_rate_deg_s
```

Sign convention follows the canonical BODY frame:

- `+vx` = forward along +X;
- `+vy` = left along +Y;
- `+yaw_rate` = counter-clockwise about +Z when viewed from above.

The command is robot intent. It is not a gait phase, foot target, joint target,
servo command, or HX1 packet.

## Configurable operating envelope

Normal motion limits are loaded from:

```text
config/control/motion.json
```

The initial standard values preserve the known v4 controller envelope:

```text
max vx       80 mm/s
max vy       60 mm/s
max yaw      90 deg/s
```

These are migration defaults, not yet a statement of final physical
qualification.

Limits are validated at the common command boundary. Commands outside the
envelope are rejected rather than silently clamped.

Input-specific shaping belongs to the input adapter. For example, the future
DS4 adapter will own stick normalization, deadzone, expo, and mapping normalized
stick values into this physical command envelope.

## Normal command sources

Normal source priority is fixed:

```text
DS4 > WEB > AUTONOMY > IDLE
```

This priority is a control/safety contract and is not a runtime configuration
knob.

A fresh DS4 zero command still owns the robot over web/autonomy. This lets a
connected human operator intentionally command a stop without lower-priority
sources taking over.

Each producer publishes a time-bounded `CommandSample` with:

```text
source
command
received_at_s
expires_at_s
```

The producer owns its freshness duration. That allows controller, web, and
autonomy adapters to use different configured timeouts without baking device
policy into the arbiter.

At the exact expiration timestamp a sample is stale.

## Emergency stop

E-stop is deliberately not another `MotionSource`.

Safety authority sits above normal command arbitration:

```text
safety / e-stop
      |
      v
normal command arbiter
      |
      v
locomotion
```

An active safety condition can suppress locomotion and disarm/stop the MCU
regardless of which normal command source is fresh.

## Future sources

Additional normal sources can be added only with an explicit priority decision.
They should still emit `MotionCommand` instead of bypassing the locomotion
pipeline.

This keeps DS4, the future web dashboard, and eventual SLAM/autonomy on one
shared command path.
