"""Classical kinematic trajectory-tracking baselines.

All controllers share the interface ``compute(t, x, ref, obstacles) -> (u, info)``
and the robot-frame tracking error (Kanayama et al., 1990):

    [e_x]   [ cos th  sin th 0] [x_r - x]
    [e_y] = [-sin th  cos th 0] [y_r - y]
    [e_th]  [   0       0    1] [th_r - th]   (e_th wrapped to (-pi, pi])

with error dynamics
    e_x'  =  omega e_y - v + v_r cos e_th
    e_y'  = -omega e_x     + v_r sin e_th
    e_th' =  omega_r - omega
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model import wrap_angle
from .references import Reference


def tracking_error(x: np.ndarray, xr: np.ndarray) -> np.ndarray:
    c, s = np.cos(x[2]), np.sin(x[2])
    dx, dy = xr[0] - x[0], xr[1] - x[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap_angle(xr[2] - x[2])])


class Controller:
    """Base class. ``dt`` is the control period."""

    name = "controller"

    def __init__(self, dt: float = 0.1):
        self.dt = dt

    def reset(self) -> None:  # noqa: D401 - stateless by default
        pass

    def compute(self, t: float, x: np.ndarray, ref: Reference, obstacles=None):
        raise NotImplementedError

    def _ref(self, t, ref):
        xr, ur = ref.sample(t)
        return xr[:, 0], ur[:, 0]


# ---------------------------------------------------------------------------
@dataclass
class KanayamaGains:
    k_x: float = 2.0
    k_y: float = 50.0             # 1/m^2
    k_theta: float | None = None  # default: critical damping 2 sqrt(k_y)


class Kanayama(Controller):
    """v = v_r cos e_th + k_x e_x ;  omega = omega_r + v_r (k_y e_y + k_th sin e_th).

    Lyapunov function V = (e_x^2 + e_y^2)/2 + (1 - cos e_th)/k_y gives
    V' = -k_x e_x^2 - v_r k_th/k_y sin^2 e_th  (needs v_r > 0).
    """

    name = "kanayama"

    def __init__(self, dt=0.1, gains: KanayamaGains | None = None):
        super().__init__(dt)
        self.g = gains or KanayamaGains()
        self.k_th = self.g.k_theta if self.g.k_theta is not None else 2 * np.sqrt(self.g.k_y)

    def compute(self, t, x, ref, obstacles=None):
        xr, ur = self._ref(t, ref)
        e = tracking_error(x, xr)
        v = ur[0] * np.cos(e[2]) + self.g.k_x * e[0]
        w = ur[1] + ur[0] * (self.g.k_y * e[1] + self.k_th * np.sin(e[2]))
        return np.array([v, w]), {"err": e}


# ---------------------------------------------------------------------------
@dataclass
class SamsonGains:
    zeta: float = 0.7
    b: float = 30.0   # 1/m^2


class Samson(Controller):
    """Gain-scheduled Samson law == kinematic backstepping (De Luca, Oriolo, Vendittelli).

    v = v_r cos e_th + k1 e_x
    omega = omega_r + k2 v_r sinc(e_th) e_y + k3 e_th
    k1 = k3 = 2 zeta sqrt(omega_r^2 + b v_r^2),  k2 = b
    V = k2/2 (e_x^2 + e_y^2) + e_th^2 / 2  =>  V' = -k1 k2 e_x^2 - k3 e_th^2.
    """

    name = "samson"

    def __init__(self, dt=0.1, gains: SamsonGains | None = None):
        super().__init__(dt)
        self.g = gains or SamsonGains()

    def compute(self, t, x, ref, obstacles=None):
        xr, ur = self._ref(t, ref)
        e = tracking_error(x, xr)
        k = 2 * self.g.zeta * np.sqrt(ur[1] ** 2 + self.g.b * ur[0] ** 2)
        k = max(k, 0.3)  # keep some authority when the reference stops
        v = ur[0] * np.cos(e[2]) + k * e[0]
        w = ur[1] + self.g.b * ur[0] * np.sinc(e[2] / np.pi) * e[1] + k * e[2]
        return np.array([v, w]), {"err": e}


# ---------------------------------------------------------------------------
@dataclass
class SMCGains:
    k1: float = 1.0
    k2: float = 4.0
    k3: float = 2.0
    K_v: float = 0.10   # switching gains (reaching speed)
    K_w: float = 0.30
    P_v: float = 1.0    # proportional reaching terms
    P_w: float = 2.0
    phi: float = 0.05   # boundary-layer width


def sat(z):
    return np.clip(z, -1.0, 1.0)


class SlidingMode(Controller):
    """Kinematic SMC with El Youssef et al. (IFAC 2014) surfaces.

    s_v = k1 e_x,   s_w = k2 e_y + k3 sin(e_th / 2)
    Reaching law  s' = -K sat(s/phi) - P s  (boundary layer against chattering).
    Solving s_w' for omega and then s_v' for v gives

    omega = [k2 v_r sin e_th + (k3/2) cos(e_th/2) omega_r + K_w sat(s_w/phi) + P_w s_w]
            / [k2 e_x + (k3/2) cos(e_th/2)]
    v     = omega e_y + v_r cos e_th + [K_v sat(s_v/phi) + P_v s_v] / k1

    At zero error this reduces to the feed-forward (v_r, omega_r).
    """

    name = "smc"

    def __init__(self, dt=0.1, gains: SMCGains | None = None):
        super().__init__(dt)
        self.g = gains or SMCGains()

    def compute(self, t, x, ref, obstacles=None):
        g = self.g
        xr, ur = self._ref(t, ref)
        ex, ey, eth = tracking_error(x, xr)
        s_v = g.k1 * ex
        s_w = g.k2 * ey + g.k3 * np.sin(eth / 2)
        c = 0.5 * g.k3 * np.cos(eth / 2)
        den = g.k2 * ex + c
        if abs(den) < 0.2 * c + 1e-3:  # guard against the singular surface
            den = np.copysign(0.2 * c + 1e-3, den if den != 0 else 1.0)
        w = (g.k2 * ur[0] * np.sin(eth) + c * ur[1]
             + g.K_w * sat(s_w / g.phi) + g.P_w * s_w) / den
        v = w * ey + ur[0] * np.cos(eth) + (g.K_v * sat(s_v / g.phi) + g.P_v * s_v) / g.k1
        return np.array([v, w]), {"err": np.array([ex, ey, eth]), "s": np.array([s_v, s_w])}


BASELINES = {"kanayama": Kanayama, "samson": Samson, "smc": SlidingMode}
