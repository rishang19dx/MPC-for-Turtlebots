"""Reference trajectory generation via differential flatness.

The unicycle's position (x, y) is a flat output, so any C^2 path gives
    theta_r = atan2(y', x'),  v_r = sqrt(x'^2 + y'^2),
    omega_r = (x' y'' - y' x'') / (x'^2 + y'^2).

Every generator returns a :class:`Reference` sampled on a dense uniform time
grid. After the final time the reference holds the final pose with zero input.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline

from .model import BURGER, RobotLimits, wrap_angle

DT_DENSE = 0.01  # s, internal sampling of references
SPEED_MARGIN = 0.8  # fraction of actuator limits used by references (feedback headroom)


@dataclass
class Reference:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray  # unwrapped (continuous)
    v: np.ndarray
    omega: np.ndarray
    name: str = "reference"
    meta: dict = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return float(self.t[-1])

    @property
    def start_pose(self) -> np.ndarray:
        return np.array([self.x[0], self.y[0], wrap_angle(self.theta[0])])

    @property
    def end_pose(self) -> np.ndarray:
        return np.array([self.x[-1], self.y[-1], wrap_angle(self.theta[-1])])

    # ------------------------------------------------------------------ sampling
    def sample(self, t) -> tuple[np.ndarray, np.ndarray]:
        """Reference state(s) [3, n] and input(s) [2, n] at time(s) t."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        past_end = t > self.t[-1]
        tc = np.clip(t, self.t[0], self.t[-1])
        xs = np.vstack([
            np.interp(tc, self.t, self.x),
            np.interp(tc, self.t, self.y),
            np.interp(tc, self.t, self.theta),
        ])
        us = np.vstack([np.interp(tc, self.t, self.v), np.interp(tc, self.t, self.omega)])
        us[:, past_end] = 0.0
        return xs, us

    def horizon(self, t0: float, n: int, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """States at t0 + k dt (k = 0..n) and inputs at k = 0..n-1."""
        ts = t0 + dt * np.arange(n + 1)
        xs, us = self.sample(ts)
        return xs, us[:, :n]

    # ------------------------------------------------------------- transforms
    def transformed(self, pose: np.ndarray) -> "Reference":
        """Rigidly move the whole trajectory so its local frame sits at ``pose``."""
        c, s = np.cos(pose[2]), np.sin(pose[2])
        x = pose[0] + c * self.x - s * self.y
        y = pose[1] + s * self.x + c * self.y
        return Reference(self.t, x, y, self.theta + pose[2], self.v, self.omega,
                         self.name, dict(self.meta))

    def anchored_at(self, pose: np.ndarray) -> "Reference":
        """Move the trajectory so its *start pose* equals ``pose``."""
        p0 = self.start_pose
        # T_new = pose * inv(T_start)
        c, s = np.cos(-p0[2]), np.sin(-p0[2])
        local = Reference(self.t,
                          c * (self.x - p0[0]) - s * (self.y - p0[1]),
                          s * (self.x - p0[0]) + c * (self.y - p0[1]),
                          self.theta - p0[2], self.v, self.omega, self.name, dict(self.meta))
        return local.transformed(pose)

    # ------------------------------------------------------------ constructors
    @classmethod
    def from_xy(cls, t: np.ndarray, x: np.ndarray, y: np.ndarray, name: str,
                theta0: float | None = None, **meta) -> "Reference":
        """Flatness: derive theta, v, omega from densely sampled x(t), y(t)."""
        dx, dy = np.gradient(x, t), np.gradient(y, t)
        ddx, ddy = np.gradient(dx, t), np.gradient(dy, t)
        v = np.hypot(dx, dy)
        moving = v > 1e-4
        theta = np.arctan2(dy, dx)
        # Heading is undefined while stopped: hold the nearest defined value.
        if not moving.any():
            theta[:] = 0.0 if theta0 is None else theta0
        else:
            idx = np.where(moving, np.arange(len(t)), 0)
            np.maximum.accumulate(idx, out=idx)
            first = np.argmax(moving)
            idx[:first] = first
            theta = theta[idx]
        theta = np.unwrap(theta)
        omega = np.where(moving, (dx * ddy - dy * ddx) / np.maximum(v, 1e-4) ** 2, 0.0)
        return cls(t, x, y, theta, v, omega, name, meta)


# ======================================================================
# Closed-form families
# ======================================================================
def circle(radius: float = 1.0, speed: float = 0.15, laps: float = 1.0) -> Reference:
    w0 = speed / radius
    T = laps * 2 * np.pi / w0
    t = np.arange(0.0, T + 1e-9, DT_DENSE)
    # starts at the origin heading +x, centre at (0, R)
    x = radius * np.sin(w0 * t)
    y = radius * (1 - np.cos(w0 * t))
    return Reference.from_xy(t, x, y, "circle", radius=radius, speed=speed)


def figure8(a: float = 1.2, w0: float = 0.1, laps: float = 1.0) -> Reference:
    """Lemniscate of Gerono: x = A sin(w t), y = (A/2) sin(2 w t). Peak speed A w sqrt(2)."""
    T = laps * 2 * np.pi / w0
    t = np.arange(0.0, T + 1e-9, DT_DENSE)
    x = a * np.sin(w0 * t)
    y = 0.5 * a * np.sin(2 * w0 * t)
    return Reference.from_xy(t, x, y, "figure8", a=a, w0=w0)


def sinusoid(length: float = 3.0, amp: float = 0.4, wavelength: float = 1.5,
             speed: float = 0.15) -> Reference:
    # x advances at roughly constant speed; y oscillates.
    k = 2 * np.pi / wavelength
    v_x = speed / np.sqrt(1 + (amp * k) ** 2 / 2)  # mean-speed correction
    T = length / v_x
    t = np.arange(0.0, T + 1e-9, DT_DENSE)
    x = v_x * t
    y = amp * np.sin(k * x)
    return Reference.from_xy(t, x, y, "sinusoid", amp=amp, wavelength=wavelength)


def lissajous(peak_speed: float = 0.16, periods: float = 1.0) -> Reference:
    """Lissajous shape of Sawarkar et al. (2026): [1.8 sin(0.23 pi s), 0.95 cos(0.15 pi s)].

    NOTE: at s = t the original curve peaks at ~1.37 m/s (far above the Burger's
    0.22 m/s), so the curve is time-scaled here to the requested peak speed.
    Shifted to start at the origin. One period of the y-term = 13.3 s of s.
    """
    s_end = periods * 2 / 0.15
    s = np.linspace(0, s_end, 20000)
    sp = np.hypot(1.8 * 0.23 * np.pi * np.cos(0.23 * np.pi * s), 0.95 * 0.15 * np.pi * np.sin(0.15 * np.pi * s))
    scale = sp.max() / peak_speed  # t = scale * s
    t = np.arange(0.0, scale * s_end + 1e-9, DT_DENSE)
    s_t = t / scale
    x = 1.8 * np.sin(0.23 * np.pi * s_t)
    y = 0.95 * np.cos(0.15 * np.pi * s_t) - 0.95
    return Reference.from_xy(t, x, y, "lissajous", time_scale=scale)


# ======================================================================
# Path-based families (time-parameterised under the robot's limits)
# ======================================================================
def _time_parameterize(px: np.ndarray, py: np.ndarray, name: str, lim: RobotLimits,
                       v_start: float = 0.0, v_end: float = 0.0, margin: float = SPEED_MARGIN,
                       **meta) -> Reference:
    """Speed profile v(s) = min(v_max, w_max/|kappa|, wheel diamond) + accel limits."""
    seg = np.hypot(np.diff(px), np.diff(py))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    # curvature from the dense path
    dx, dy = np.gradient(px, s), np.gradient(py, s)
    ddx, ddy = np.gradient(dx, s), np.gradient(dy, s)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum(np.hypot(dx, dy), 1e-9) ** 3
    v_lim = np.minimum.reduce([
        np.full_like(s, margin * lim.v_max),
        margin * lim.w_max / np.maximum(kappa, 1e-9),
        margin * lim.wheel_speed_max / (1.0 + lim.half_track * kappa),
    ])
    a = 0.5 * margin * lim.a_max
    v = v_lim.copy()
    v[0], v[-1] = min(v[0], v_start), min(v[-1], v_end)
    for i in range(1, len(s)):  # forward pass (acceleration)
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a * seg[i - 1]))
    for i in range(len(s) - 2, -1, -1):  # backward pass (deceleration)
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a * seg[i]))
    v_avg = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-4)
    t_path = np.concatenate([[0.0], np.cumsum(seg / v_avg)])
    t = np.arange(0.0, t_path[-1] + 1e-9, DT_DENSE)
    s_t = np.interp(t, t_path, s)
    x, y = np.interp(s_t, s, px), np.interp(s_t, s, py)
    theta0 = np.arctan2(py[1] - py[0], px[1] - px[0])
    return Reference.from_xy(t, x, y, name, theta0=theta0, length=float(s[-1]), **meta)


def random_spline(seed: int, arena: float = 3.0, n_wp: tuple[int, int] = (5, 8),
                  min_sep: float = 0.4, max_turn_deg: float = 120.0,
                  max_curvature: float = 1 / 0.15, lim: RobotLimits = BURGER,
                  max_tries: int = 500) -> Reference:
    """Reproducible random smooth trajectory through random waypoints.

    Waypoints in a square arena (centred on the origin), >= ``min_sep`` apart,
    turn angle <= ``max_turn_deg``; C^2 cubic spline on chord length; rejected
    if curvature exceeds ``max_curvature``. The result starts at rest at the
    first waypoint and ends at rest at the last.
    """
    rng = np.random.default_rng(seed)
    half = arena / 2
    for _ in range(max_tries):
        n = rng.integers(n_wp[0], n_wp[1] + 1)
        wps = [rng.uniform(-half, half, 2)]
        ok = True
        while len(wps) < n and ok:
            for _ in range(100):
                p = rng.uniform(-half, half, 2)
                if np.linalg.norm(p - wps[-1]) < min_sep:
                    continue
                if len(wps) >= 2:
                    d1, d2 = wps[-1] - wps[-2], p - wps[-1]
                    cosang = d1 @ d2 / (np.linalg.norm(d1) * np.linalg.norm(d2))
                    if np.degrees(np.arccos(np.clip(cosang, -1, 1))) > max_turn_deg:
                        continue
                wps.append(p)
                break
            else:
                ok = False
        if not ok:
            continue
        wps = np.array(wps)
        chord = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(wps, axis=0), axis=1))])
        cs = CubicSpline(chord, wps, bc_type="natural")
        ss = np.linspace(0, chord[-1], max(200, int(chord[-1] / 0.005)))
        pts = cs(ss)
        d1, d2 = cs(ss, 1), cs(ss, 2)
        kappa = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.linalg.norm(d1, axis=1) ** 3
        if kappa.max() > max_curvature or np.abs(pts).max() > half + 0.3:
            continue
        return _time_parameterize(pts[:, 0], pts[:, 1], "random_spline", lim,
                                  seed=seed, waypoints=wps.tolist())
    raise RuntimeError(f"random_spline: no valid path for seed {seed}")


def point_to_point(start: np.ndarray, goal: np.ndarray, lim: RobotLimits = BURGER,
                   tangent_scale: float = 0.6) -> Reference:
    """Rest-to-rest cubic Bezier path between two poses [x, y, theta]."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    d = np.linalg.norm(goal[:2] - start[:2])
    L = tangent_scale * d
    p0 = start[:2]
    p1 = p0 + L * np.array([np.cos(start[2]), np.sin(start[2])])
    p3 = goal[:2]
    p2 = p3 - L * np.array([np.cos(goal[2]), np.sin(goal[2])])
    u = np.linspace(0, 1, max(200, int(d / 0.005)))[:, None]
    pts = (1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * p1 + 3 * (1 - u) * u ** 2 * p2 + u ** 3 * p3
    return _time_parameterize(pts[:, 0], pts[:, 1], "point_to_point", lim,
                              start=start.tolist(), goal=goal.tolist())


FAMILIES = {
    "circle": lambda seed=None: circle(),
    "figure8": lambda seed=None: figure8(),
    "sinusoid": lambda seed=None: sinusoid(),
    "lissajous": lambda seed=None: lissajous(),
    "random_spline": lambda seed=0: random_spline(seed),
}


def make_reference(spec: dict) -> Reference:
    """Build a reference from a config dict, e.g. ``{"type": "figure8", "a": 1.0}``."""
    spec = dict(spec)
    kind = spec.pop("type")
    if kind == "circle":
        return circle(**spec)
    if kind == "figure8":
        return figure8(**spec)
    if kind == "sinusoid":
        return sinusoid(**spec)
    if kind == "lissajous":
        return lissajous(**spec)
    if kind == "random_spline":
        return random_spline(**spec)
    if kind == "point_to_point":
        return point_to_point(np.asarray(spec["start"]), np.asarray(spec["goal"]))
    raise ValueError(f"unknown reference type {kind!r}")
