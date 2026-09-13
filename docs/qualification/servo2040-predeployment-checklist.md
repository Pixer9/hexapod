# Servo 2040 Pre-Deployment Checklist

**Status:** Open  
**Checkpoint:** actuator-rate qualification complete; before integrated energized HIL/commissioning

The MCU architecture and actuator-rate boundary have completed host-side and
representative physical qualification. The robot is not yet approved for normal
gait or loaded walking.

## Code hardening completed

- bounded HX1 framing and CRC validation;
- bounded message-type identifiers;
- session/state authority model;
- independent link and motion watchdogs;
- watchdog deadlines evaluated before late authority-refreshing commands;
- atomic complete-vector hard position validation;
- hard command-rate validation using actual MCU elapsed time;
- fail-safe top-level exception containment;
- maintenance Ctrl-C escape on the HX1 USB CDC stream;
- best-effort PWM release on top-level firmware exit;
- tri-state hardware output truth (`enabled`, `disabled`, `unknown`);
- repeated disable attempts while hardware output state is unknown;
- watchdog grace hold cannot energize outputs from a non-energized state.

## Actuator profile completed

- revision 2 corrected migrated tibia upper ranges to remain within the
  ServoCluster angular backend envelope;
- representative coxa, femur, and tibia actuators were physically exercised
  through the selected hard command-rate ceiling;
- additional cross-leg sanity checks were performed;
- revision 3 assigns `max_rate_cd_s = 25000` to all 18 joints;
- revision 3 is arm-qualified for controlled HIL/commissioning;
- exact revision 3 profile SHA-256 is
  `DF71DEBDB81A02999715B201DBEE7B7CF1363937E81C9D19449C4DBAA9178177`;
- Pi expected profile identity is revision 3 with the same exact-byte hash;
- qualification evidence is recorded in
  `docs/qualification/joint-rate-rev3.md`.

## Must be resolved before normal energized robot operation

### Applied-output behavior

- validate the accepted-target hard rate limit in integrated Pi-to-MCU motion;
- physically validate ARM, START, TARGET, STOP, DISARM, watchdog hold, fault,
  E-stop, and recovery behavior with revision 3 deployed;
- verify that no integrated motion path can bypass the MCU hard position or rate
  envelope.

### Protocol/client contract

- preserve protocol-minor compatibility policy during further client work;
- preserve the bounded 32-character maximum `TYPE` behavior;
- keep `TARGET.PERIOD_MS` semantics consistent between implementation and
  protocol documentation;
- confirm whether `TARGET_AGE_MS` diagnostic behavior should change through
  STOP/DISARM;
- confirm whether future uptime telemetry should represent accumulated uptime or
  the raw wrapping MicroPython tick value.

### MicroPython / Servo 2040 board validation

Completed before rate promotion:

- production modules executed on the installed Pimoroni MicroPython build;
- ServoCluster channel enable/disable behavior was exercised on real hardware;
- exact selected-channel isolation was verified during single-joint tests;
- all outputs were verified disabled after each qualification run;
- the maintenance Ctrl-C escape and board reset path were exercised.

Still to evaluate during later hardening:

- whether outbound USB writes can block long enough to interfere with safety
  timing;
- whether to enable RP2040 `machine.WDT` as protection against a wedged
  interpreter/USB write path.

## Physical commissioning sequence

Completed:

1. Servo 2040 validation with servo rail disconnected.
2. Conservative single-joint powered checks.
3. Representative right-back coxa/femur/tibia rate ladders through 250 deg/s.
4. Additional right-front actuator sanity checks.
5. Exact channel isolation and post-test disable verification.

Next:

6. Deploy the revision 3 / rc3 candidate.
7. Verify HIL boot reaches `DISARMED` rather than `FAULT/PROFILE`.
8. Exercise controlled ARM/START/TARGET/STOP/DISARM behavior while mechanically
   supported.
9. Exercise watchdog, E-stop, fault, and recovery behavior while energized.
10. Perform all-joint mapping/direction and supported whole-body pose checks.
11. Integrate the production Pi motion generator.
12. Perform controlled gait tests.
13. Validate loaded walking and power/current behavior.

## Deferred, not forgotten

The following remain outside the current checkpoint:

- foot-contact acquisition;
- IMU/ToF MCU acquisition decision;
- voltage/current telemetry;
- status LED policy;
- physical E-stop input;
- maintenance/calibration protocol beyond the current maintenance escape;
- final production lifecycle/orchestration behavior.
