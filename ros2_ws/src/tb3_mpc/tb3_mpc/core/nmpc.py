"""Nonlinear MPC for unicycle trajectory tracking with obstacle constraints.

Optimal control problem solved every control period (multiple shooting):

    min   sum_{k=0}^{N-1} [ ||p_k - p_k^r||^2_Qp + q_th 2(1 - cos(th_k - th_k^r))
                            + ||u_k - u_k^r||^2_R + ||u_k - u_{k-1}||^2_S ]
          + ||p_N - p_N^r||^2_Pp + p_th 2(1 - cos(th_N - th_N^r))
          + rho * sum(slack)                                   (exact L1 penalty)
    s.t.  x_0 = x_hat                                          (measured / delay-compensated)
          x_{k+1} = F_RK4(x_k, u_k)                            (continuity)
          v_min <= v_k <= v_max,  |w_k| <= w_max                (box, via variable bounds)
          +-v_k +- (L/2) w_k <= v_wheel_max                     (wheel-speed diamond)
          |u_k - u_{k-1}| <= [a_max, alpha_max] dt              (rate limits, u_{-1} = applied)
          obstacle constraints, one of
            'dc' : h_j(x_k) >= -s                               k = 1..N
            'cbf': h_j(x_{k+1}) >= (1 - gamma) h_j(x_k) - s     k = 0..N_c-1
            'pf' : no constraint, cost += w_pf * exp(-h_j / sigma)
          with h_j(x) = ||p_l(x) - c_j||^2 - r_j^2 and look-ahead point
          p_l(x) = p + l [cos th, sin th]   (puts heading into h).

Obstacles enter as a fixed-size parameter block (M circles, inflated radii);
unused slots are filled with far-away dummies so the NLP is never rebuilt.
The problem is built once with CasADi SX and solved by IPOPT with warm starts.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from .model import BURGER, RobotLimits
from .references import Reference

DUMMY_OBSTACLE = (1e3, 1e3, 0.1)


@dataclass
class NMPCConfig:
    N: int = 20
    dt: float = 0.1
    # weights
    q_xy: float = 10.0
    q_th: float = 1.0
    r_v: float = 0.5
    r_w: float = 0.05
    s_v: float = 1.0
    s_w: float = 0.1
    terminal_factor: float = 10.0
    # obstacles
    obstacle_mode: str = "cbf"       # 'none' | 'dc' | 'cbf' | 'pf'
    max_obstacles: int = 8
    gamma: float = 0.2               # CBF decay rate in (0, 1]
    cbf_horizon: int | None = None   # constrain first N_c steps (default: N)
    lookahead: float = 0.05          # m
    slack_weight: float = 1e4        # L1 exact-penalty weight
    pf_weight: float = 5.0
    pf_sigma: float = 0.05           # m^2
    # limits
    limits: RobotLimits = field(default_factory=lambda: BURGER)
    v_min: float | None = None       # override limits.v_min (e.g. 0.0 forbids reversing)
    # solver
    max_iter: int = 100
    tol: float = 1e-4
    max_wall_time: float = 0.08
    max_consecutive_failures: int = 3


class NMPC:
    """Builds the NLP once; ``compute()`` warm-starts and solves it."""

    name = "nmpc"

    def __init__(self, cfg: NMPCConfig | None = None):
        self.cfg = cfg or NMPCConfig()
        self.dt = self.cfg.dt
        self._build()
        self.reset()

    # ------------------------------------------------------------------ build
    def _build(self) -> None:
        c = self.cfg
        lim = c.limits
        N, M = c.N, c.max_obstacles
        mode = c.obstacle_mode
        Nc = min(c.cbf_horizon or N, N)
        n_slack_steps = {"dc": N, "cbf": Nc}.get(mode, 0)

        # --- dynamics
        xs, us = ca.SX.sym("x", 3), ca.SX.sym("u", 2)
        f = ca.Function("f", [xs, us], [ca.vertcat(us[0] * ca.cos(xs[2]), us[0] * ca.sin(xs[2]), us[1])])
        k1 = f(xs, us)
        k2 = f(xs + c.dt / 2 * k1, us)
        k3 = f(xs + c.dt / 2 * k2, us)
        k4 = f(xs + c.dt * k3, us)
        F = ca.Function("F", [xs, us], [xs + c.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])

        # --- decision variables
        X = ca.SX.sym("X", 3, N + 1)
        U = ca.SX.sym("U", 2, N)
        S = ca.SX.sym("S", M, n_slack_steps) if n_slack_steps else ca.SX.zeros(0, 0)
        # --- parameters
        x0 = ca.SX.sym("x0", 3)
        Xr = ca.SX.sym("Xr", 3, N + 1)
        Ur = ca.SX.sym("Ur", 2, N)
        u_prev = ca.SX.sym("u_prev", 2)
        OBS = ca.SX.sym("obs", 3, M)

        def h(xk, j):
            px = xk[0] + c.lookahead * ca.cos(xk[2])
            py = xk[1] + c.lookahead * ca.sin(xk[2])
            return (px - OBS[0, j]) ** 2 + (py - OBS[1, j]) ** 2 - OBS[2, j] ** 2

        Qp, qth = c.q_xy, c.q_th
        Pp, pth = c.terminal_factor * c.q_xy, c.terminal_factor * c.q_th
        R = ca.diag(ca.DM([c.r_v, c.r_w]))
        Sw = ca.diag(ca.DM([c.s_v, c.s_w]))

        J = 0
        g, lbg, ubg = [], [], []
        g.append(X[:, 0] - x0); lbg += [0] * 3; ubg += [0] * 3
        a = lim.half_track
        du_max = np.array([lim.a_max, lim.alpha_max]) * c.dt
        for k in range(N):
            e = X[:2, k] - Xr[:2, k]
            J += Qp * ca.sumsqr(e) + qth * 2 * (1 - ca.cos(X[2, k] - Xr[2, k]))
            du_ref = U[:, k] - Ur[:, k]
            J += ca.mtimes([du_ref.T, R, du_ref])
            du = U[:, k] - (u_prev if k == 0 else U[:, k - 1])
            J += ca.mtimes([du.T, Sw, du])
            # continuity
            g.append(X[:, k + 1] - F(X[:, k], U[:, k])); lbg += [0] * 3; ubg += [0] * 3
            # wheel diamond (4 linear constraints)
            for sv in (1, -1):
                for sw in (1, -1):
                    g.append(sv * U[0, k] + sw * a * U[1, k]); lbg.append(-ca.inf); ubg.append(lim.wheel_speed_max)
            # rate limits
            g.append(du); lbg += list(-du_max); ubg += list(du_max)
        eN = X[:2, N] - Xr[:2, N]
        J += Pp * ca.sumsqr(eN) + pth * 2 * (1 - ca.cos(X[2, N] - Xr[2, N]))

        # --- obstacles
        if mode == "dc":
            for k in range(N):
                for j in range(M):
                    g.append(h(X[:, k + 1], j) + S[j, k]); lbg.append(0); ubg.append(ca.inf)
        elif mode == "cbf":
            for k in range(Nc):
                for j in range(M):
                    g.append(h(X[:, k + 1], j) - (1 - c.gamma) * h(X[:, k], j) + S[j, k])
                    lbg.append(0); ubg.append(ca.inf)
        elif mode == "pf":
            for k in range(1, N + 1):
                for j in range(M):
                    J += c.pf_weight * ca.exp(-h(X[:, k], j) / c.pf_sigma)
        elif mode != "none":
            raise ValueError(f"unknown obstacle_mode {mode!r}")
        if n_slack_steps:
            J += c.slack_weight * ca.sum1(ca.vec(S))

        w = ca.vertcat(ca.vec(X), ca.vec(U), ca.vec(S) if n_slack_steps else ca.SX.zeros(0, 1))
        p = ca.vertcat(x0, ca.vec(Xr), ca.vec(Ur), u_prev, ca.vec(OBS))
        nlp = {"x": w, "f": J, "g": ca.vertcat(*g), "p": p}
        opts = {
            "ipopt.max_iter": c.max_iter,
            "ipopt.tol": c.tol,
            "ipopt.acceptable_tol": 10 * c.tol,
            "ipopt.acceptable_iter": 5,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_wall_time": c.max_wall_time,
            "ipopt.warm_start_init_point": "yes",
            "ipopt.mu_init": 1e-3,
            "print_time": False,
        }
        self.solver = ca.nlpsol("nmpc", "ipopt", nlp, opts)

        # bounds on decision variables
        v_min = lim.v_min if c.v_min is None else c.v_min
        nX, nU, nS = 3 * (N + 1), 2 * N, M * n_slack_steps
        lbw = np.full(nX + nU + nS, -np.inf)
        ubw = np.full(nX + nU + nS, np.inf)
        lbw[nX:nX + nU:2], ubw[nX:nX + nU:2] = v_min, lim.v_max
        lbw[nX + 1:nX + nU:2], ubw[nX + 1:nX + nU:2] = -lim.w_max, lim.w_max
        lbw[nX + nU:] = 0.0
        self._lbw, self._ubw = lbw, ubw
        self._lbg, self._ubg = np.array(lbg, float), np.array(ubg, float)
        self._nX, self._nU, self._nS = nX, nU, nS
        self._n_slack_steps = n_slack_steps

    # ------------------------------------------------------------------ solve
    def reset(self) -> None:
        self._w0 = None
        self._lam_g0 = None
        self._lam_x0 = None
        self._u_prev = np.zeros(2)
        self._plan_U = None
        self._fails = 0

    def set_applied_input(self, u: np.ndarray) -> None:
        """Tell the MPC what was actually sent (after saturation) for the rate cost."""
        self._u_prev = np.asarray(u, float).copy()

    def _pack_obstacles(self, obstacles) -> np.ndarray:
        M = self.cfg.max_obstacles
        obs = np.tile(np.array(DUMMY_OBSTACLE, float)[:, None], (1, M))
        if obstacles is not None and len(obstacles):
            o = np.asarray(obstacles, float).reshape(-1, 3)
            o = o[:M]
            obs[:, : len(o)] = o.T
        return obs

    def _initial_guess(self, x0, Xr, Ur, obs) -> np.ndarray:
        c = self.cfg
        N = c.N
        if self._w0 is None:
            # Cold start: follow the reference unless it runs through an obstacle,
            # in which case start from "stand still" (collision-free by construction).
            d = np.hypot(Xr[0][:, None] - obs[0][None], Xr[1][:, None] - obs[1][None]) - obs[2][None]
            if c.obstacle_mode != "none" and (d < 0).any():
                X = np.tile(x0[:, None], (1, N + 1))
                U = np.zeros((2, N))
            else:
                X = Xr.copy()
                X[:, 0] = x0
                U = Ur.copy()
            S = np.zeros(self._nS)
            return np.concatenate([X.T.ravel(), U.T.ravel(), S])
        w = self._w0
        X = w[: self._nX].reshape(N + 1, 3).T
        U = w[self._nX: self._nX + self._nU].reshape(N, 2).T
        S = w[self._nX + self._nU:]
        # shift by one step, repeat the last column
        X = np.hstack([X[:, 1:], X[:, -1:]])
        U = np.hstack([U[:, 1:], U[:, -1:]])
        if self._n_slack_steps:
            S = S.reshape(self._n_slack_steps, -1).T
            S = np.hstack([S[:, 1:], S[:, -1:]]).T.ravel()
        # keep the heading guess on the same 2*pi branch as the measurement
        X[2] += 2 * np.pi * np.round((x0[2] - X[2, 0]) / (2 * np.pi))
        X[:, 0] = x0
        return np.concatenate([X.T.ravel(), U.T.ravel(), S])

    def compute(self, t: float, x: np.ndarray, ref: Reference, obstacles=None):
        c = self.cfg
        x = np.asarray(x, float)
        Xr, Ur = ref.horizon(t, c.N, c.dt)
        # keep reference heading on the same branch as the robot (cost is 1-cos anyway;
        # this only improves the cold-start guess)
        Xr[2] += 2 * np.pi * np.round((x[2] - Xr[2, 0]) / (2 * np.pi))
        obs = self._pack_obstacles(obstacles)
        p = np.concatenate([x, Xr.T.ravel(), Ur.T.ravel(), self._u_prev, obs.T.ravel()])
        w0 = self._initial_guess(x, Xr, Ur, obs)
        args = dict(x0=w0, p=p, lbx=self._lbw, ubx=self._ubw, lbg=self._lbg, ubg=self._ubg)
        if self._lam_g0 is not None:
            args["lam_g0"] = self._lam_g0
            args["lam_x0"] = self._lam_x0
        tic = time.perf_counter()
        sol = self.solver(**args)
        solve_time = time.perf_counter() - tic
        st = self.solver.stats()
        status = st.get("return_status", "unknown")
        ok = st.get("success", False) or status in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        w = np.asarray(sol["x"]).ravel()
        N = c.N
        X = w[: self._nX].reshape(N + 1, 3).T
        U = w[self._nX: self._nX + self._nU].reshape(N, 2).T
        slack = w[self._nX + self._nU:]

        if ok:
            self._fails = 0
            self._w0 = w
            self._lam_g0 = np.asarray(sol["lam_g"]).ravel()
            self._lam_x0 = np.asarray(sol["lam_x"]).ravel()
            self._plan_U = U
            u = U[:, 0].copy()
            fallback = "none"
        else:
            self._fails += 1
            # keep shifting the last good primal guess; drop the (now stale) duals
            self._w0 = w0 if self._fails <= c.max_consecutive_failures else None
            self._lam_g0, self._lam_x0 = None, None
            if self._plan_U is not None and self._fails <= c.max_consecutive_failures and self._plan_U.shape[1] > 1:
                self._plan_U = self._plan_U[:, 1:]
                u = self._plan_U[:, 0].copy()
                fallback = "previous_plan"
            else:
                u = np.zeros(2)
                fallback = "brake"
        info = {
            "solve_time": solve_time,
            "status": status,
            "success": bool(ok),
            "iters": int(st.get("iter_count", -1)),
            "fallback": fallback,
            "pred_X": X,
            "pred_U": U,
            "ref_X": Xr,
            "max_slack": float(slack.max()) if slack.size else 0.0,
            "cost": float(sol["f"]),
        }
        return u, info
