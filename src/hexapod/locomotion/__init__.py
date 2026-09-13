"""Pi-side locomotion and gait trajectory generation."""

from .config import (
    GaitConfigError,
    TripodGaitConfig,
    load_tripod_gait_config,
    parse_tripod_gait_config,
)
from .controller import (
    LocomotionController,
    LocomotionControllerError,
    LocomotionFrame,
    LocomotionMode,
)
from .controller_config import (
    LocomotionControllerConfig,
    LocomotionControllerConfigError,
    load_locomotion_controller_config,
    parse_locomotion_controller_config,
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
    "LocomotionController",
    "LocomotionControllerConfig",
    "LocomotionControllerConfigError",
    "LocomotionControllerError",
    "LocomotionFrame",
    "LocomotionMode",
    "TRIPOD_A",
    "TRIPOD_B",
    "TripodGait",
    "TripodGaitConfig",
    "load_locomotion_controller_config",
    "load_tripod_gait_config",
    "parse_locomotion_controller_config",
    "parse_tripod_gait_config",
    "tripod_phase_for_leg",
]
