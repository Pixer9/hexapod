# Joint-Rate Qualification — Actuator Profile Revision 3

**Date:** 2026-09-13  
**Profile:** `hexapod-standard-v1`  
**Profile revision:** 3  
**Qualified maximum rate:** `25000 cd/s` (`250 deg/s`)  
**Profile SHA-256:** `DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177`

## Purpose

This record documents the physical qualification basis for populating
`max_rate_cd_s` in actuator-profile revision 3.

`max_rate_cd_s` is the MCU-enforced maximum logical command slew rate. It is
not a claim that the physical servo shaft tracks that velocity under every
possible mechanical load.

## Test Configuration

- installed DS3235 5G servos;
- installed Servo 2040 controller;
- 7.4 V servo rail;
- robot mechanically supported;
- tested leg unloaded;
- one actuator energized at a time;
- 50 Hz command updates;
- actual elapsed time used to bound every commanded transition;
- all non-selected ServoCluster channels verified disabled;
- all channels disabled at test completion.

## Representative Joint-Class Qualification

The complete rate ladder was exercised on the right-back leg.

### RB Coxa — canonical joint 6 / channel 0

Tested through:

- 10 deg/s
- 25 deg/s
- 50 deg/s
- 100 deg/s
- 150 deg/s
- 200 deg/s
- 250 deg/s

Result: PASS through 250 deg/s.

### RB Femur — canonical joint 7 / channel 1

Tested through:

- 10 deg/s
- 25 deg/s
- 50 deg/s
- 100 deg/s
- 150 deg/s
- 200 deg/s
- 250 deg/s

Result: PASS through 250 deg/s.

### RB Tibia — canonical joint 8 / channel 2

Tested through:

- 10 deg/s
- 25 deg/s
- 50 deg/s
- 100 deg/s
- 150 deg/s
- 200 deg/s
- 250 deg/s

Result: PASS through 250 deg/s.

## Cross-Leg Sanity Checks

Additional installed-actuator checks were performed on the right-front leg.

- RF coxa: +/-15 deg at 10 deg/s — PASS
- RF coxa: +/-15 deg at 250 deg/s — PASS
- RF femur: +/-15 deg at 10 deg/s — PASS

No unexpected actuator motion, binding, sustained oscillation, or output
isolation failures were observed.

## Selected Limit

Revision 3 assigns:

```text
max_rate_cd_s = 25000
```

to all 18 joints.

The common limit is intentionally conservative relative to theoretical servo
speed and matches the Pi trajectory-rate budget already used by the production
motion pipeline.

## Scope

This qualification establishes the MCU hard logical command-rate ceiling and
makes actuator-profile revision 3 arm-qualified for controlled energized HIL
and commissioning.

It does not replace later integrated robot commissioning, including:

- all-joint mapping/direction verification;
- supported whole-body pose tests;
- watchdog and E-stop energized tests;
- controlled gait tests;
- loaded walking validation.

Those activities validate integrated robot behavior rather than the meaning of
the hard command-rate field itself.

## Requalification

Requalification is required if the actuator type, control frequency, mechanical
geometry, command-rate policy, or other assumptions affecting safe actuator
command rates change materially.
