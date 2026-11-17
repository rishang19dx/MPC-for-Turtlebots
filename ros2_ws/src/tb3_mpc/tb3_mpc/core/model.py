"""Unicycle kinematics and actuator limits of the TurtleBot3 Burger.

State x = [p_x, p_y, theta] (m, m, rad) in the ``odom`` frame (REP-103/105).
Input u = [v, omega] (m/s, rad/s) in the robot body frame.

    p_x' = v cos(theta)
    p_y' = v sin(theta)
    theta' = omega
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotLimits:
    """Burger limits (ROBOTIS e-manual, turtlebot3_node/param/burger.yaml)."""

    v_max: float = 0.22          # m/s
    v_min: float = -0.22         # m/s (reverse allowed by the driver)
    w_max: float = 2.84          # rad/s
    wheel_separation: float = 0.160  # m
    wheel_radius: float = 0.033      # m
    a_max: float = 0.5           # m/s^2   engineering choice (no official value)
    alpha_max: float = 3.0       # rad/s^2 engineering choice
    robot_radius: float = 0.105  # m, footprint circle used for collision checks

    @property
    def wheel_speed_max(self) -> float:
        """Max linear wheel-rim speed; equal to v_max for a diff drive."""
        return self.v_max

    @property
    def half_track(self) -> float:
        return 0.5 * self.wheel_separation


BURGER = RobotLimits()


def wrap_angle(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def unicycle_f(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Continuous-time unicycle vector field."""
    return np.array([u[0] * np.cos(x[2]), u[0] * np.sin(x[2]), u[1]])


def rk4_step(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """One RK4 step with zero-order-hold input."""
    k1 = unicycle_f(x, u)
    k2 = unicycle_f(x + 0.5 * dt * k1, u)
    k3 = unicycle_f(x + 0.5 * dt * k2, u)
    k4 = unicycle_f(x + dt * k3, u)
    return x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def predict_forward(x: np.ndarray, commands, dt: float) -> np.ndarray:
    """Roll the nominal model forward through already-committed commands.

    Used for delay compensation: if the plant applies commands with a dead time
    of d control periods, the controller should plan from the state it will be
    in once its new command actually takes effect.
    """
    x = np.asarray(x, dtype=float).copy()
    for u in commands:
        x = rk4_step(x, np.asarray(u, dtype=float), dt)
    return x


def wheel_speeds(u: np.ndarray, lim: RobotLimits = BURGER) -> tuple[float, float]:
    """(v, omega) -> (right, left) wheel rim speeds in m/s."""
    return u[0] + lim.half_track * u[1], u[0] - lim.half_track * u[1]


def body_twist(v_r: float, v_l: float, lim: RobotLimits = BURGER) -> np.ndarray:
    """(right, left) wheel rim speeds -> (v, omega)."""
    return np.array([0.5 * (v_r + v_l), (v_r - v_l) / lim.wheel_separation])


def saturate(u, lim: RobotLimits = BURGER, u_prev=None, dt: float | None = None) -> np.ndarray:
    """Shared actuator model applied to *every* controller's output.

    1. Rate limits (if ``u_prev`` and ``dt`` given).
    2. Box limits on v and omega.
    3. Wheel-speed "diamond" |v| + (L/2)|omega| <= v_wheel_max, enforced by
       uniform scaling so the commanded path curvature v/omega is preserved.
    """
    u = np.array(u, dtype=float)
    if u_prev is not None and dt is not None:
        du_max = np.array([lim.a_max * dt, lim.alpha_max * dt])
        u = np.asarray(u_prev, dtype=float) + np.clip(u - u_prev, -du_max, du_max)
    u[0] = np.clip(u[0], lim.v_min, lim.v_max)
    u[1] = np.clip(u[1], -lim.w_max, lim.w_max)
    load = abs(u[0]) + lim.half_track * abs(u[1])
    if load > lim.wheel_speed_max:
        u *= lim.wheel_speed_max / load
    return u
