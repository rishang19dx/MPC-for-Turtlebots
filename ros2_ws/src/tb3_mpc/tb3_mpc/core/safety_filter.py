"""CBF-QP safety filter used to give the classical baselines obstacle avoidance.

Given a nominal command u_nom, solve

    min_u  (u - u_nom)^T W (u - u_nom)
    s.t.   dh_j/dt(x, u) >= -alpha h_j(x)        for every obstacle j
           box limits + wheel-speed diamond

with the same look-ahead barrier as the NMPC,
    h_j(x) = ||p_l - c_j||^2 - r_j^2,  p_l = p + l [cos th, sin th],
    dh_j/dt = 2 (p_l - c_j)^T [[cos th, -l sin th], [sin th, l cos th]] u,
which is affine in u, so the filter is a 2-variable QP. When no constraint is
active the output equals u_nom (minimally invasive). Infeasible -> brake.
"""
from __future__ import annotations

import casadi as ca
import numpy as np

from .model import BURGER, RobotLimits


class CBFSafetyFilter:
    def __init__(self, max_obstacles: int = 8, alpha: float = 1.0, lookahead: float = 0.05,
                 weights=(1.0, 0.05), limits: RobotLimits = BURGER):
        self.M, self.alpha, self.l, self.lim = max_obstacles, alpha, lookahead, limits
        u = ca.SX.sym("u", 2)
        A = ca.SX.sym("A", max_obstacles, 2)   # constraint rows   A u >= b
        un = ca.SX.sym("un", 2)
        W = ca.diag(ca.DM(weights))
        a = limits.half_track
        g = [ca.mtimes(A, u)]
        for sv in (1, -1):
            for sw in (1, -1):
                g.append(sv * u[0] + sw * a * u[1])
        qp = {"x": u, "f": ca.mtimes([(u - un).T, W, (u - un)]), "g": ca.vertcat(*g),
              "p": ca.vertcat(ca.vec(A), un)}
        self.solver = ca.qpsol("cbf_qp", "osqp", qp, {
            "error_on_fail": False,
            "osqp": {"verbose": False, "eps_abs": 1e-6, "eps_rel": 1e-6, "polish": True},
        })

    def filter(self, x: np.ndarray, u_nom: np.ndarray, obstacles) -> tuple[np.ndarray, dict]:
        if obstacles is None or len(obstacles) == 0:
            return np.asarray(u_nom, float), {"active": False}
        obs = np.asarray(obstacles, float).reshape(-1, 3)[: self.M]
        c, s = np.cos(x[2]), np.sin(x[2])
        pl = np.array([x[0] + self.l * c, x[1] + self.l * s])
        G = np.array([[c, -self.l * s], [s, self.l * c]])
        A = np.zeros((self.M, 2))
        b = np.full(self.M, -1e6)
        for j, (ox, oy, r) in enumerate(obs):
            d = pl - np.array([ox, oy])
            h = d @ d - r * r
            A[j] = 2 * d @ G
            b[j] = -self.alpha * h
        lim = self.lim
        lbg = np.concatenate([b, np.full(4, -np.inf)])
        ubg = np.concatenate([np.full(self.M, np.inf), np.full(4, lim.wheel_speed_max)])
        sol = self.solver(x0=u_nom, p=np.concatenate([A.T.ravel(), u_nom]),
                          lbx=[lim.v_min, -lim.w_max], ubx=[lim.v_max, lim.w_max],
                          lbg=lbg, ubg=ubg)
        ok = self.solver.stats().get("success", False)
        if not ok:
            return np.zeros(2), {"active": True, "infeasible": True}
        u = np.asarray(sol["x"]).ravel()
        return u, {"active": bool(np.linalg.norm(u - u_nom) > 1e-6), "infeasible": False}


class FilteredController:
    """Wraps a baseline: nominal tracking law followed by the CBF-QP filter."""

    def __init__(self, base, safety: CBFSafetyFilter):
        self.base, self.safety = base, safety
        self.dt = base.dt
        self.name = f"{base.name}+cbf"

    def reset(self):
        self.base.reset()

    def compute(self, t, x, ref, obstacles=None):
        u_nom, info = self.base.compute(t, x, ref, obstacles)
        u, finfo = self.safety.filter(x, u_nom, obstacles)
        info.update(filter=finfo, u_nom=u_nom)
        return u, info
