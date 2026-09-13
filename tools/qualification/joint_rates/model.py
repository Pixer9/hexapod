"""Canonical Hexapod geometry and IK used by the joint-rate qualification tool.

This module intentionally works in logical robot-joint space only.
It contains no Servo 2040 channel mapping, trim, direction, or PWM logic.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

LEG_ORDER = ("RF", "RM", "RB", "LF", "LM", "LB")
JOINT_ORDER = ("coxa", "femur", "tibia")

R_MM = 105.0
C_MM = R_MM / math.sqrt(2.0)

LEG_YAW_DEG = {
    "RF": -45.0,
    "RM": -90.0,
    "RB": -135.0,
    "LF": 45.0,
    "LM": 90.0,
    "LB": 135.0,
}

LEG_POS_BODY_MM = {
    "RF": (C_MM, -C_MM, 0.0),
    "RM": (0.0, -R_MM, 0.0),
    "RB": (-C_MM, -C_MM, 0.0),
    "LF": (C_MM, C_MM, 0.0),
    "LM": (0.0, R_MM, 0.0),
    "LB": (-C_MM, C_MM, 0.0),
}

DEFAULT_NEUTRAL_FOOT_LEG = (200.0, 0.0, -115.0)


@dataclass(frozen=True)
class LinkLengths:
    coxa: float
    femur: float
    tibia: float


@dataclass(frozen=True)
class JointAngles:
    coxa: float
    femur: float
    tibia: float


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap01(value: float) -> float:
    return value % 1.0


def smoothstep(value: float) -> float:
    x = clamp(value, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def rotz_deg(x: float, y: float, deg: float) -> tuple[float, float]:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return (c * x - s * y, s * x + c * y)


def leg_to_body_frame(
    point: tuple[float, float, float], leg: str
) -> tuple[float, float, float]:
    """Convert a point from a leg-local frame to the body frame."""
    x, y, z = point
    yaw = math.radians(LEG_YAW_DEG[leg])
    c = math.cos(yaw)
    s = math.sin(yaw)
    xr = c * x - s * y
    yr = s * x + c * y
    lx, ly, lz = LEG_POS_BODY_MM[leg]
    return (xr + lx, yr + ly, z + lz)


def body_to_leg_frame(
    point: tuple[float, float, float], leg: str
) -> tuple[float, float, float]:
    """Convert a point from the body frame to a leg-local frame."""
    x, y, z = point
    lx, ly, lz = LEG_POS_BODY_MM[leg]
    x -= lx
    y -= ly
    z -= lz
    yaw = math.radians(LEG_YAW_DEG[leg])
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x + s * y, -s * x + c * y, z)


def neutral_foot_body() -> dict[str, tuple[float, float, float]]:
    return {leg: leg_to_body_frame(DEFAULT_NEUTRAL_FOOT_LEG, leg) for leg in LEG_ORDER}


def leg_ik_leg_frame(
    x: float,
    y: float,
    z: float,
    lengths: LinkLengths,
) -> JointAngles:
    """Canonical 3-DOF leg IK.

    Conventions:
      * +Z is up.
      * Coxa is yaw around +Z.
      * Femur moves in the radial/Z plane.
      * Tibia is the knee angle: 0 degrees straight, positive bends.
    """
    eps = 1e-6
    theta_coxa = math.atan2(y, x)

    rho = math.hypot(x, y)
    r = rho - lengths.coxa
    if abs(r) < eps:
        r = eps if r >= 0.0 else -eps

    d = math.hypot(r, z)
    d_min = abs(lengths.femur - lengths.tibia) + eps
    d_max = (lengths.femur + lengths.tibia) - eps
    d = clamp(d, d_min, d_max)

    f2 = lengths.femur * lengths.femur
    t2 = lengths.tibia * lengths.tibia
    d2 = d * d

    cos_alpha = (f2 + d2 - t2) / (2.0 * lengths.femur * d)
    alpha = math.acos(clamp(cos_alpha, -1.0, 1.0))

    cos_beta = (f2 + t2 - d2) / (2.0 * lengths.femur * lengths.tibia)
    beta = math.acos(clamp(cos_beta, -1.0, 1.0))

    gamma = math.atan2(z, r)

    return JointAngles(
        coxa=math.degrees(theta_coxa),
        femur=math.degrees(gamma + alpha),
        tibia=math.degrees(math.pi - beta),
    )


def compute_logical_frame(
    targets_body: dict[str, tuple[float, float, float]],
    lengths: LinkLengths,
) -> list[float]:
    """Compute the canonical 18-joint vector in RF..LB / coxa,femur,tibia order."""
    frame: list[float] = []

    for leg in LEG_ORDER:
        tx, ty, tz = body_to_leg_frame(targets_body[leg], leg)
        angles = leg_ik_leg_frame(tx, ty, tz, lengths)
        frame.extend((angles.coxa, angles.femur, angles.tibia))

    return frame


def joint_names() -> list[str]:
    return [f"{leg.lower()}_{joint}" for leg in LEG_ORDER for joint in JOINT_ORDER]
