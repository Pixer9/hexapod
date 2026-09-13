# Servo 2040 Pre-Deployment Checklist

**Status:** Open  
**Checkpoint:** post-MCU-hardening, before physical servo deployment

The current MCU code is a host-tested architecture checkpoint. It is not yet
approved for energized robot operation.

## Code hardening completed

- bounded HX1 framing and CRC validation;
- bounded message-type identifiers;
- session/state authority model;
- independent link and motion watchdogs;
- watchdog deadlines evaluated before late authority-refreshing commands;
- atomic complete-vector hard position validation;
- hard command-rate validation using actual MCU elapsed time;
- fail-safe top-level exception containment;
- production Ctrl-C keyboard interrupt disabled on the HX1 USB CDC stream;
- best-effort PWM release on top-level firmware exit;
- tri-state hardware output truth (`enabled`, `disabled`, `unknown`);
- repeated disable attempts while hardware output state is unknown;
- watchdog grace hold cannot energize outputs from a non-energized state.

## Must be resolved before energized deployment

### Actuator profile

- qualify every `max_rate_cd_s`; current `null` values intentionally block
  successful self-test/arming;
- revise the four migrated tibia upper ranges that exceed Pimoroni's default
  angular value envelope, unless a separately qualified calibration is adopted;
- increment profile revision and capture the exact deployed profile hash.

### Applied-output behavior

- decide whether the accepted-target hard rate limit alone satisfies the MCU
  hard-slew responsibility or whether a separate applied-output slew stage is
  required;
- physically validate arm transitions, target updates, STOP, DISARM, watchdog
  hold, and E-stop behavior.

### Protocol/client contract

- reconcile protocol-minor policy before implementing the production Pi client;
- update the protocol document to state the 32-character maximum `TYPE` length;
- decide whether `TARGET.PERIOD_MS` remains advisory/diagnostic or becomes part
  of negotiated semantics;
- decide whether `TARGET_AGE_MS` should preserve diagnostic age through
  STOP/DISARM instead of sharing the ACTIVE watchdog timestamp;
- decide whether `UPTIME_MS` means a true accumulated uptime counter or the raw
  wrapping MicroPython tick value.

### MicroPython / Servo 2040 board validation

- run the complete firmware modules on the actual installed Pimoroni
  MicroPython build before enabling servo power;
- verify `ujson`, `uhashlib`, `machine.unique_id`, `micropython.kbd_intr`,
  `uselect.poll`, and all used `time.ticks_*` APIs;
- verify the exact Pimoroni `ServoCluster` constructor, `value(load=False)`,
  `load()`, `disable_all(load=True)`, `min_value()`, and `max_value()` behavior;
- evaluate whether outbound USB writes can block long enough to interfere with
  safety timing;
- decide whether to enable RP2040 `machine.WDT` as protection against a wedged
  interpreter/USB write path.

### Physical qualification sequence

1. Servo 2040 only, servo rail disconnected.
2. One unloaded servo with conservative limits.
3. Verify direction, offset, disable, E-stop, and watchdog behavior.
4. Expand to one complete leg.
5. Expand to all 18 channels while mechanically unloaded where practical.
6. Validate current draw and power behavior.
7. Integrate Pi and MCU only after both sides pass independent boundary tests.

## Deferred, not forgotten

The following are intentionally outside the current checkpoint rather than
assumed complete:

- foot-contact acquisition;
- IMU/ToF MCU acquisition decision;
- voltage/current telemetry;
- status LED policy;
- physical E-stop input;
- maintenance/calibration protocol;
- production Pi HX1 client and MCU simulator.
