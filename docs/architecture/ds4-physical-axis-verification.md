# Verified DualShock 4 Axis Directions

Physical verification on the Raspberry Pi 5 / paired DualShock 4 established:

```text
left stick forward   -> ABS_Y -> 0
left stick backward  -> ABS_Y -> 255

left stick left      -> ABS_X -> 0
left stick right     -> ABS_X -> 255

right stick left     -> ABS_RX -> 0
right stick right    -> ABS_RX -> 255
```

The evdev adapter normalizes `0 -> -1` and `255 -> +1`.

Given the canonical body command convention:

```text
+vx       forward
+vy       left
+yaw_rate counter-clockwise / turn-left
```

the verified mapping is:

```text
vx       <- -ABS_Y
vy       <- -ABS_X
yaw_rate <- -ABS_RX
```

Therefore all three configured DS4 motion axes use `invert: true`.

This supersedes the v4 migrated `vx <- +ABS_Y` assumption for this actual
controller/kernel combination.
