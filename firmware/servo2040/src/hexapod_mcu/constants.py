"""Servo 2040 firmware constants derived from the accepted v1 contracts."""

# Protocol identity.
PROTOCOL_PREFIX = "HX1"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0

# Framing.
MAX_FRAME_BYTES = 512
UINT32_MAX = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000

# Robot/profile contract.
JOINT_COUNT = 18

# Runtime timing contract.
TARGET_RATE_HZ = 50
TARGET_PERIOD_MS = 20
HEARTBEAT_RATE_HZ = 10
MOTION_TIMEOUT_MS = 200
LINK_TIMEOUT_MS = 750
FAULT_HOLD_MS = 500
ARM_STAGE_MAX_AGE_MS = 2000
