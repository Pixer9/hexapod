# Joint-Rate Qualification Tool

This is a **software-only engineering qualification tool** for estimating the logical joint velocities required by the Hexapod motion envelope.

It does not connect to the Servo 2040 or any other hardware.

## Why it exists

`max_rate_cd_s` is a hard actuator-safety parameter in the Servo 2040 actuator profile. It should not be guessed.

This tool establishes an initial rate baseline by replaying the proven V4 geometry, tripod trajectory, gait configuration, and controller command envelope in the new canonical logical-joint coordinate system.

It reports:

- peak joint rate;
- 99th percentile joint rate;
- 99.9th percentile joint rate;
- per-joint and per-joint-family values;
- the scenario that produced the overall peak;
- a **candidate** value computed with configurable engineering headroom.

The candidate is not automatically written into the firmware profile.

## Run

From the repository root:

```bash
python tools/qualification/joint_rates/qualify.py
```

No third-party Python packages are required.

Output is written to:

```text
build/qualification/joint-rates/
├── samples.csv
└── summary.json
```

The repository already ignores `build/`, so repeated qualification runs do not dirty the working tree.

## Baseline scope

The initial scenario file uses:

- 50 Hz planned Pi target rate;
- 41 / 116 / 183 mm coxa/femur/tibia links;
- 1.0 Hz tripod gait;
- 70 mm step height;
- -135 mm stance height;
- 0.5 duty factor;
- 50 mm max step;
- 80 mm/s max X velocity;
- 60 mm/s max Y velocity;
- 90 deg/s max yaw rate;
- V4 one-pole + asymmetric slew command conditioning (translation tau 0.18 s, yaw tau 0.16 s; 260/520 mm/s² translation accel/decel; 280/700 deg/s² yaw accel/decel).

It includes axial motion, yaw, maximum mixed-axis corners, full reversals, and a start/stop transition.

## Important limitation

This tool is a **software-envelope baseline**, not final actuator qualification.

Its generated candidates must not be promoted into `actuator-profile.json` solely
because this tool produced them. A hard actuator limit requires separate
physical qualification of the installed actuator/mechanism and documented
safety evidence.

Actuator-profile revision 3 currently contains:

```json
"max_rate_cd_s": 25000
```

for all 18 joints. That value was promoted only after separate progressive
physical qualification and is documented in:

```text
docs/qualification/joint-rate-rev3.md
```

The revision 3 result does not make this software-only tool authoritative for
future profile updates.

Use this tool to answer:

> What rates does the modeled motion envelope request, and how close are those
> rates to the currently qualified actuator ceiling?

Re-run the baseline whenever motion-envelope inputs change. If production demand
approaches or exceeds the qualified ceiling, review the motion design and repeat
physical rate qualification as appropriate.
