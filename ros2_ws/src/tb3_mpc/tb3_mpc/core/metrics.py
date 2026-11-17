"""Episode metrics (tracking, effort, constraints, safety, computation)."""
from __future__ import annotations

import numpy as np

from .controllers import tracking_error
from .model import wrap_angle

FAIL_MAX_ERROR = 0.5  # m: a tracking run with larger max error counts as failed


def episode_metrics(log: dict, transient: float = 0.0, lam_w: float = 0.1) -> dict:
    """Scalar metrics for one episode.

    Tracking metrics (rmse_*, max_pos) use the window ``transient <= t <= T_ref``:
    the initial-offset transient is excluded (captured by IAE/ITAE/settling time
    instead) and so is the hold phase after the reference ends, where references
    that stop abruptly at speed would otherwise dominate the max error
    (captured by ``final_pos_error`` / ``stop_overshoot`` instead).
    """
    t, x, xr, u, u_raw = log["t"], log["x"], log["xr"], log["u"], log["u_raw"]
    dt = log["dt"]
    if len(t) == 0:
        return {}
    e = np.array([tracking_error(xi, xri) for xi, xri in zip(x, xr)])  # robot frame
    e_pos = np.hypot(x[:, 0] - xr[:, 0], x[:, 1] - xr[:, 1])
    e_th = wrap_angle(xr[:, 2] - x[:, 2])
    m = (t >= transient) & (t <= log["ref_duration"])
    if not m.any():
        m = t >= 0
    du = np.diff(u, axis=0)
    dw_sign = np.sign(np.diff(u[:, 1]))
    chatter = np.sum(dw_sign[1:] * dw_sign[:-1] < 0) / max(t[-1], dt)
    st = log["solve_time"]
    st = st[np.isfinite(st)]
    settle_idx = np.where(e_pos > 0.02)[0]
    final_err = float(np.hypot(*(log["x_final"][:2] - log["end_pose_ref"][:2])))
    out = {
        "rmse_pos": float(np.sqrt(np.mean(e_pos[m] ** 2))),
        "max_pos": float(e_pos[m].max()),
        "rmse_heading": float(np.sqrt(np.mean(e_th[m] ** 2))),
        "rmse_along": float(np.sqrt(np.mean(e[m, 0] ** 2))),
        "rmse_lateral": float(np.sqrt(np.mean(e[m, 1] ** 2))),
        "iae": float(np.sum(e_pos) * dt),
        "itae": float(np.sum(t * e_pos) * dt),
        "settling_time": float(t[settle_idx[-1]] + dt) if len(settle_idx) else 0.0,
        "final_pos_error": final_err,
        "stop_overshoot": float(e_pos[t > log["ref_duration"]].max()) if (t > log["ref_duration"]).any() else 0.0,
        "effort": float(np.sum(u[:, 0] ** 2 + lam_w * u[:, 1] ** 2) * dt),
        "smoothness": float(np.sum(du ** 2)),
        "total_variation": float(np.sum(np.abs(du))),
        "chattering": float(chatter),
        "saturated_pct": float(100 * np.mean(np.linalg.norm(u_raw - u, axis=1) > 1e-6)),
        "min_clearance": float(np.min(log["clearance"])),
        "collided": bool(log["collided"]),
        "completed": bool(len(t) * dt >= log["ref_duration"]),
        "solve_mean_ms": float(1e3 * st.mean()) if st.size else np.nan,
        "solve_p95_ms": float(1e3 * np.percentile(st, 95)) if st.size else np.nan,
        "solve_max_ms": float(1e3 * st.max()) if st.size else np.nan,
        "deadline_misses": int(np.sum(st > dt)) if st.size else 0,
        "solver_failures": int(np.sum(~log["success"].astype(bool))),
        "max_slack": float(np.max(log["max_slack"])) if len(log["max_slack"]) else 0.0,
    }
    has_obs = len(log["obstacles"]) > 0
    if has_obs:
        out["success"] = (not out["collided"]) and out["completed"] and final_err < 0.15
    else:
        out["success"] = out["completed"] and out["max_pos"] < FAIL_MAX_ERROR
    return out
