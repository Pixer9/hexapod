"""Pure kinematics for the hexapod."""

from .leg import (
    JointAngles,
    KinematicsError,
    UnreachableTargetError,
    leg_fk,
    leg_ik,
)

__all__ = [
    "JointAngles",
    "KinematicsError",
    "UnreachableTargetError",
    "leg_fk",
    "leg_ik",
]
