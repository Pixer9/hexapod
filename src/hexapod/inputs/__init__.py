"""Hardware input adapters for the Raspberry Pi runtime."""

from .ds4 import (
    DS4Action,
    DS4ActionType,
    DS4Config,
    DS4ConfigError,
    DS4InputState,
    DS4Mapper,
    DS4Snapshot,
    load_ds4_config,
    normalize_axis,
)
from .ds4_evdev import (
    DS4DeviceInfo,
    DS4EvdevError,
    DS4EvdevReader,
    discover_ds4_device,
)

__all__ = [
    "DS4Action",
    "DS4ActionType",
    "DS4Config",
    "DS4ConfigError",
    "DS4DeviceInfo",
    "DS4EvdevError",
    "DS4EvdevReader",
    "DS4InputState",
    "DS4Mapper",
    "DS4Snapshot",
    "discover_ds4_device",
    "load_ds4_config",
    "normalize_axis",
]
