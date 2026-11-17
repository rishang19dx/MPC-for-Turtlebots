"""Kinematic closed-loop simulator for fast, reproducible Monte Carlo.

The *plant* is the unicycle plus optional, controller-invisible perturbations:
  * input dead time of ``delay_steps`` control periods
  * first-order actuator lag (time constant ``actuator_tau``)
  * wheel-radius mismatch (right/left scale factors -> speed and turning bias)
  * Gaussian pose-measurement noise
Every controller receives identical treatment: same rate, same saturation
(:func:`model.saturate`), same measurement, same seed-driven noise sequence.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from . import lidar
from .model import BURGER, RobotLimits, body_twist, predict_forward, rk4_step, saturate, wheel_speeds, wrap_angle
from .references import Reference


@dataclass
class PlantConfig:
    delay_steps: int = 0
    actuator_tau: float = 0.0     # s, 0 = ideal
    wheel_scale_r: float = 1.0
    wheel_scale_l: float = 1.0
    pos_noise: float = 0.0        # m (std)
    th_noise: float = 0.0         # rad (std)
    substeps: int = 10


@dataclass
class EpisodeConfig:
    dt: float = 0.1
    t_extra: float = 3.0          # s simulated after the reference ends
    compensate_delay: bool = False
    inflation_margin: float = 0.05  # m added to robot radius when inflating obstacles
    perceive_with_lidar: bool = False
    lidar_noise: float = 0.01
    max_obstacles: int = 8
    stop_on_collision: bool = True


def inflate(obstacles, lim: RobotLimits, margin: float) -> np.ndarray:
    o = np.asarray(obstacles, float).reshape(-1, 3).copy()
    o[:, 2] += lim.robot_radius + margin
    return o


def run_episode(controller, ref: Reference, x0: np.ndarray, obstacles=None,
                plant: PlantConfig | None = None, ep: EpisodeConfig | None = None,
                seed: int = 0, lim: RobotLimits = BURGER) -> dict:
    plant = plant or PlantConfig()
    ep = ep or EpisodeConfig()
    rng = np.random.default_rng(seed)
    dt = ep.dt
    true_obs = np.zeros((0, 3)) if obstacles is None else np.asarray(obstacles, float).reshape(-1, 3)
    known_obs = inflate(true_obs, lim, ep.inflation_margin)

    controller.reset()
    x = np.asarray(x0, float).copy()
    u_act = np.zeros(2)
    u_sent = np.zeros(2)
    pending = deque([np.zeros(2)] * plant.delay_steps)  # commands sent, not yet applied
    n_steps = int(np.ceil((ref.duration + ep.t_extra) / dt))
    log = {k: [] for k in ("t", "x", "x_meas", "xr", "ur", "u_raw", "u", "solve_time",
                           "success", "fallback", "clearance", "max_slack")}
    collided = False
    for i in range(n_steps):
        t = i * dt
        x_meas = x + np.array([rng.normal(0, plant.pos_noise), rng.normal(0, plant.pos_noise),
                               rng.normal(0, plant.th_noise)]) if (plant.pos_noise or plant.th_noise) else x.copy()
        x_meas[2] = wrap_angle(x_meas[2])

        # ---- perception
        if len(true_obs) and ep.perceive_with_lidar:
            scan = lidar.simulate_scan(x, true_obs, noise=ep.lidar_noise, rng=rng)
            circ = lidar.extract_circles(scan, 0.0, 2 * np.pi / len(scan), 0.12, 3.5)
            c, s = np.cos(x_meas[2]), np.sin(x_meas[2])
            if len(circ):
                circ = np.column_stack([x_meas[0] + c * circ[:, 0] - s * circ[:, 1],
                                        x_meas[1] + s * circ[:, 0] + c * circ[:, 1], circ[:, 2]])
            obs_ctrl = lidar.nearest_k(inflate(circ, lim, ep.inflation_margin), x_meas[:2], ep.max_obstacles)
        else:
            obs_ctrl = lidar.nearest_k(known_obs, x_meas[:2], ep.max_obstacles) if len(known_obs) else None

        # ---- control (optionally planning from the delay-compensated state)
        if ep.compensate_delay and plant.delay_steps:
            x_plan = predict_forward(x_meas, list(pending), dt)
            t_plan = t + plant.delay_steps * dt
        else:
            x_plan, t_plan = x_meas, t
        u_raw, info = controller.compute(t_plan, x_plan, ref, obs_ctrl)
        u = saturate(u_raw, lim, u_prev=u_sent, dt=dt)
        if hasattr(controller, "set_applied_input"):
            controller.set_applied_input(u)
        u_sent = u

        # ---- logging (before the plant moves)
        xr, ur = ref.sample(t)
        log["t"].append(t); log["x"].append(x.copy()); log["x_meas"].append(x_meas)
        log["xr"].append(xr[:, 0]); log["ur"].append(ur[:, 0])
        log["u_raw"].append(np.asarray(u_raw, float)); log["u"].append(u)
        log["solve_time"].append(info.get("solve_time", np.nan))
        log["success"].append(info.get("success", True))
        log["fallback"].append(info.get("fallback", "none"))
        log["max_slack"].append(info.get("max_slack", 0.0))
        if len(true_obs):
            clear = np.hypot(true_obs[:, 0] - x[0], true_obs[:, 1] - x[1]) - true_obs[:, 2] - lim.robot_radius
            log["clearance"].append(float(clear.min()))
        else:
            log["clearance"].append(np.inf)

        # ---- plant
        pending.append(u)
        u_apply = pending.popleft()
        h = dt / plant.substeps
        for _ in range(plant.substeps):
            if plant.actuator_tau > 0:
                u_act = u_act + (u_apply - u_act) * (1 - np.exp(-h / plant.actuator_tau))
            else:
                u_act = u_apply
            vr, vl = wheel_speeds(u_act, lim)
            twist = body_twist(vr * plant.wheel_scale_r, vl * plant.wheel_scale_l, lim)
            x = rk4_step(x, twist, h)
        if len(true_obs):
            if (np.hypot(true_obs[:, 0] - x[0], true_obs[:, 1] - x[1]) - true_obs[:, 2] - lim.robot_radius).min() < 0:
                collided = True
                if ep.stop_on_collision:
                    break

    out = {k: np.asarray(v) for k, v in log.items()}
    out.update(collided=collided, controller=getattr(controller, "name", "?"), ref_name=ref.name,
               ref_duration=ref.duration, end_pose_ref=ref.end_pose, x_final=x, dt=dt,
               obstacles=true_obs)
    return out
