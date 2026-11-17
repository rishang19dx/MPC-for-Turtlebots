"""Seeded scenarios (reference + initial pose + obstacles) shared by every tier.

Frames
------
A reference lives in its own *reference frame* R. The robot starts at pose
``x0_R = ref.start_pose (+) offset`` in R (``offset`` is expressed in the
reference start frame, so it means "how far the robot is from where the
trajectory begins"). On a real robot / in Gazebo the robot's start pose in
the odom (or world) frame X is known, which fixes T_X<-R:

    T_X<-R = T_X<-robot0 * inv(T_R<-robot0)

:func:`reference_frame_in` computes that so the controller node (odom frame)
and the Gazebo obstacle spawner (world frame) place things consistently.
"""
from __future__ import annotations

import numpy as np

from .model import wrap_angle
from .references import Reference, make_reference


# --------------------------------------------------------------- SE(2) helpers
def compose(a, b) -> np.ndarray:
    """a (+) b : pose b expressed in frame a, returned in a's parent frame."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2])])


def inverse(a) -> np.ndarray:
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c * a[0] - s * a[1], s * a[0] - c * a[1], wrap_angle(-a[2])])


def transform_points(T, pts) -> np.ndarray:
    pts = np.asarray(pts, float).reshape(-1, 3).copy()
    c, s = np.cos(T[2]), np.sin(T[2])
    x, y = pts[:, 0].copy(), pts[:, 1].copy()
    pts[:, 0] = T[0] + c * x - s * y
    pts[:, 1] = T[1] + s * x + c * y
    return pts


def robot_start_in_ref(ref: Reference, offset) -> np.ndarray:
    return compose(ref.start_pose, np.asarray(offset, float))


def reference_frame_in(robot0_in_X, ref: Reference, offset) -> np.ndarray:
    """Pose of the reference frame R in frame X, given the robot start pose in X."""
    return compose(np.asarray(robot0_in_X, float), inverse(robot_start_in_ref(ref, offset)))


# --------------------------------------------------------------- generators
def random_offset(rng: np.random.Generator, pos: float = 0.2, ang: float = np.pi / 6) -> np.ndarray:
    return np.array([rng.uniform(-pos, pos), rng.uniform(-pos, pos), rng.uniform(-ang, ang)])


def obstacles_on_path(ref: Reference, rng: np.random.Generator, n: int = 2,
                      radius=(0.08, 0.18), lateral: float = 0.12, end_clearance: float = 0.6,
                      min_sep: float = 0.8) -> np.ndarray:
    """Place ``n`` cylinders on (or near) the reference path, away from its ends."""
    seg = np.hypot(np.diff(ref.x), np.diff(ref.y))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    L = s[-1]
    obs = []
    for _ in range(200):
        if len(obs) == n:
            break
        si = rng.uniform(end_clearance, max(end_clearance + 1e-3, L - end_clearance))
        i = int(np.searchsorted(s, si))
        i = min(i, len(ref.x) - 1)
        th = ref.theta[i]
        lat = rng.uniform(-lateral, lateral)
        p = np.array([ref.x[i] - np.sin(th) * lat, ref.y[i] + np.cos(th) * lat])
        # keep away from start/end poses and from other obstacles
        if np.hypot(*(p - [ref.x[0], ref.y[0]])) < end_clearance:
            continue
        if np.hypot(*(p - [ref.x[-1], ref.y[-1]])) < end_clearance:
            continue
        if any(np.hypot(*(p - o[:2])) < min_sep for o in obs):
            continue
        obs.append(np.array([p[0], p[1], rng.uniform(*radius)]))
    return np.array(obs).reshape(-1, 3)


def make_scenario(family: str, seed: int, offset_pos: float = 0.2, offset_ang: float = np.pi / 6,
                  n_obstacles: int = 0) -> dict:
    """Reproducible scenario. Everything is expressed in the reference frame R."""
    rng = np.random.default_rng(10_000 + seed)
    if family == "random_spline":
        ref = make_reference({"type": "random_spline", "seed": seed})
    elif family == "point_to_point":
        # random goal 1.5-3 m away, random final heading
        d, phi = rng.uniform(1.5, 3.0), rng.uniform(-np.pi, np.pi)
        goal = [d * np.cos(phi), d * np.sin(phi), rng.uniform(-np.pi, np.pi)]
        ref = make_reference({"type": "point_to_point", "start": [0, 0, rng.uniform(-np.pi, np.pi)],
                              "goal": goal})
    else:
        ref = make_reference({"type": family})
    offset = random_offset(rng, offset_pos, offset_ang)
    obstacles = obstacles_on_path(ref, rng, n_obstacles) if n_obstacles else np.zeros((0, 3))
    return {
        "family": family, "seed": seed, "ref": ref, "offset": offset,
        "x0": robot_start_in_ref(ref, offset), "obstacles": obstacles,
    }
