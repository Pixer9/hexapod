"""Reference tripod gait used for the initial joint-rate baseline.

This intentionally mirrors the proven V4 tripod trajectory behavior closely enough
to establish a migration baseline. It is not a declaration that the new runtime
must retain this gait implementation forever.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from model import LEG_ORDER, clamp, lerp, rotz_deg, smoothstep, wrap01

TRIPOD_A = ("RF", "RB", "LM")
TRIPOD_B = ("RM", "LF", "LB")


@dataclass
class GaitParams:
    cycle_hz: float
    step_height: float
    stance_z: float
    duty: float
    max_step: float
    start_blend_s: float


@dataclass
class LegLatch:
    in_stance: bool | None = None
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    dx_target: float = 0.0
    dy_target: float = 0.0
    dyaw_target: float = 0.0


class TripodGait:
    def __init__(self) -> None:
        self._latches = {leg: LegLatch() for leg in LEG_ORDER}

    def reset(self) -> None:
        self._latches = {leg: LegLatch() for leg in LEG_ORDER}

    @staticmethod
    def _phase_for_leg(global_phase: float, leg: str) -> float:
        if leg in TRIPOD_B:
            return wrap01(global_phase + 0.5)
        return wrap01(global_phase)

    def foot_body_target(
        self,
        *,
        t: float,
        leg: str,
        vx: float,
        vy: float,
        yaw_rate_deg_s: float,
        neutral_foot_body: dict[str, tuple[float, float, float]],
        params: GaitParams,
    ) -> tuple[float, float, float]:
        global_phase = wrap01(t * params.cycle_hz)
        phase = self._phase_for_leg(global_phase, leg)

        in_stance = phase < params.duty
        if in_stance:
            u = phase / max(params.duty, 1e-6)
        else:
            u = (phase - params.duty) / max(1.0 - params.duty, 1e-6)

        blend = 1.0
        if params.start_blend_s > 1e-6:
            blend = smoothstep(clamp(t / params.start_blend_s, 0.0, 1.0))

        half_period = 0.5 / max(params.cycle_hz, 1e-6)
        dx_now = clamp((-vx) * half_period, -params.max_step, params.max_step) * blend
        dy_now = clamp(vy * half_period, -params.max_step, params.max_step) * blend
        dyaw_now = clamp(yaw_rate_deg_s * half_period, -45.0, 45.0) * blend

        state = self._latches[leg]
        state.dx_target = dx_now
        state.dy_target = dy_now
        state.dyaw_target = dyaw_now

        if state.in_stance is None or in_stance != state.in_stance:
            state.in_stance = in_stance
            state.dx = state.dx_target
            state.dy = state.dy_target
            state.dyaw = state.dyaw_target

        chase = 0.18
        state.dx += (state.dx_target - state.dx) * chase
        state.dy += (state.dy_target - state.dy) * chase
        state.dyaw += (state.dyaw_target - state.dyaw) * chase

        dx = state.dx
        dy = state.dy
        dyaw = state.dyaw
        nx, ny, _ = neutral_foot_body[leg]
        s = smoothstep(u)

        if in_stance:
            x = nx - lerp(-dx, dx, s)
            y = ny - lerp(-dy, dy, s)
            x2, y2 = rotz_deg(x, y, -lerp(-dyaw, dyaw, s))
            return (x2, y2, params.stance_z)

        x = nx + lerp(-dx, dx, s)
        y = ny + lerp(-dy, dy, s)
        x2, y2 = rotz_deg(x, y, lerp(-dyaw, dyaw, s))
        lift = (params.step_height * blend) * math.sin(math.pi * u)
        return (x2, y2, params.stance_z + lift)
