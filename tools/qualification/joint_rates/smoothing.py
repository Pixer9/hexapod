"""Command conditioning used by the V4 reference motion baseline."""

from __future__ import annotations

import math


class OnePoleSlewUD:
    """Stable one-pole low-pass filter followed by asymmetric slew limiting."""

    def __init__(
        self,
        y0: float,
        tau: float,
        rate_up: float,
        rate_down: float,
        *,
        dt_max: float = 0.10,
    ) -> None:
        self.y = float(y0)
        self.tau = float(tau)
        self.rate_up = max(0.0, float(rate_up))
        self.rate_down = max(0.0, float(rate_down))
        self.dt_max = max(1e-6, float(dt_max))

    def reset(self, y: float) -> None:
        self.y = float(y)

    def update(self, x: float, dt_s: float) -> float:
        dt = max(1e-6, min(float(dt_s), self.dt_max))
        x = float(x)

        if self.tau <= 0.0:
            target = x
        else:
            a = 1.0 - math.exp(-dt / max(1e-6, self.tau))
            target = self.y + a * (x - self.y)

        delta = target - self.y
        if delta > 0.0:
            self.y += min(delta, self.rate_up * dt)
        elif delta < 0.0:
            self.y += max(delta, -self.rate_down * dt)

        return self.y
