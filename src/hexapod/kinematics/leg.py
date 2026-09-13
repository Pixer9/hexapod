"""Pure 3-DOF leg inverse and forward kinematics.

These equations preserve the proven v4 mathematical convention:

* +Z is up.
* Coxa is yaw about +Z.
* Femur moves in the radial/Z plane.
* Tibia/knee is 0 degrees when straight and positive when bent.

Unlike v4, unreachable targets are rejected explicitly instead of silently
clamping the requested distance onto the reachable workspace boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from hexapod.model.geometry import LinkLengths, Vec3


class KinematicsError(ValueError):
    """Base class for deterministic kinematics failures."""


class UnreachableTargetError(KinematicsError):
    """Requested foot target is outside the geometric leg workspace."""


@dataclass(frozen=True, slots=True)
class JointAngles:
    coxa_deg: float
    femur_deg: float
    tibia_deg: float


def _clamp_unit(value: float) -> float:
    # Only protects acos against tiny floating-point overshoot after the
    # geometric reachability test has already succeeded.
    if value < -1.0:
        return -1.0
    if value > 1.0:
        return 1.0
    return value


def leg_ik(x_mm: float, y_mm: float, z_mm: float, links: LinkLengths) -> JointAngles:
    """Solve one foot target expressed in the local LEG frame."""
    x = float(x_mm)
    y = float(y_mm)
    z = float(z_mm)

    if not all(math.isfinite(v) for v in (x, y, z)):
        raise KinematicsError("foot target must contain finite coordinates")

    theta_coxa = math.atan2(y, x)

    rho = math.hypot(x, y)
    radial = rho - links.coxa_mm
    distance = math.hypot(radial, z)

    minimum = abs(links.femur_mm - links.tibia_mm)
    maximum = links.femur_mm + links.tibia_mm

    tolerance = 1e-9
    if distance < minimum - tolerance or distance > maximum + tolerance:
        raise UnreachableTargetError(
            "foot target is outside leg workspace: "
            f"distance={distance:.6f} mm, reachable=[{minimum:.6f}, {maximum:.6f}] mm"
        )

    # Avoid division by zero only for degenerate link sets; valid configured
    # links plus the reachability interval should keep distance positive.
    if distance <= 0.0:
        raise KinematicsError("degenerate leg geometry produced zero distance")

    f = links.femur_mm
    t = links.tibia_mm
    d2 = distance * distance
    f2 = f * f
    t2 = t * t

    cos_alpha = (f2 + d2 - t2) / (2.0 * f * distance)
    alpha = math.acos(_clamp_unit(cos_alpha))

    cos_beta = (f2 + t2 - d2) / (2.0 * f * t)
    beta = math.acos(_clamp_unit(cos_beta))

    gamma = math.atan2(z, radial)

    theta_femur = gamma + alpha
    theta_tibia = math.pi - beta

    return JointAngles(
        coxa_deg=math.degrees(theta_coxa),
        femur_deg=math.degrees(theta_femur),
        tibia_deg=math.degrees(theta_tibia),
    )


def leg_fk(angles: JointAngles, links: LinkLengths) -> Vec3:
    """Compute the local LEG-frame foot position from canonical joint angles."""
    tc = math.radians(float(angles.coxa_deg))
    tf = math.radians(float(angles.femur_deg))
    tk = math.radians(float(angles.tibia_deg))

    radial_after_coxa = links.femur_mm * math.cos(tf) + links.tibia_mm * math.cos(
        tf - tk
    )
    z = links.femur_mm * math.sin(tf) + links.tibia_mm * math.sin(tf - tk)

    radial_total = links.coxa_mm + radial_after_coxa
    x = radial_total * math.cos(tc)
    y = radial_total * math.sin(tc)

    return (x, y, z)
