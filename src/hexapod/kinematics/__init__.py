"""Pure kinematics for the hexapod."""

from .leg import (
    JointAngles,
    KinematicsError,
    UnreachableTargetError,
    leg_fk,
    leg_ik,
)
from .robot import (
    JOINTS_PER_LEG,
    LOGICAL_JOINT_COUNT,
    FootTargetSetError,
    LegSolveError,
    RobotJointSolution,
    RobotKinematics,
    WholeBodyKinematicsError,
)

__all__ = [
    "FootTargetSetError",
    "JOINTS_PER_LEG",
    "JointAngles",
    "KinematicsError",
    "LOGICAL_JOINT_COUNT",
    "LegSolveError",
    "RobotJointSolution",
    "RobotKinematics",
    "UnreachableTargetError",
    "WholeBodyKinematicsError",
    "leg_fk",
    "leg_ik",
]
