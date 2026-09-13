"""Pi-side locomotion and gait trajectory generation."""

from .config import (
    GaitConfigError,
    TripodGaitConfig,
    load_tripod_gait_config,
    parse_tripod_gait_config,
)
from .tripod import (
    GaitError,
    GaitFrame,
    LegPhaseState,
    TRIPOD_A,
    TRIPOD_B,
    TripodGait,
    tripod_phase_for_leg,
)

__all__ = [
    "GaitConfigError",
    "GaitError",
    "GaitFrame",
    "LegPhaseState",
    "TRIPOD_A",
    "TRIPOD_B",
    "TripodGait",
    "TripodGaitConfig",
    "load_tripod_gait_config",
    "parse_tripod_gait_config",
    "tripod_phase_for_leg",
]
