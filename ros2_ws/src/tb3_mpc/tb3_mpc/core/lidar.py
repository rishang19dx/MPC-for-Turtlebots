"""LaserScan -> obstacle circles (pure numpy, ROS-free, unit-testable).

Pipeline
  1. mask invalid beams (inf, nan, 0.0, outside [range_min, range_max])
  2. polar -> Cartesian in the sensor frame
  3. adaptive-breakpoint segmentation (Borges & Aldon 2004):
       break if ||p_i - p_{i-1}|| > r_{i-1} sin(dphi)/sin(lambda - dphi) + 3 sigma_r
     and merge the wrap-around segment of a 360 deg scan
  4. long segments (walls) are split into chunks of <= ``max_segment_len``
  5. Kasa least-squares circle fit per chunk; radius clamped to [r_min, r_max]
     (fallback: centroid + half the chord for nearly straight chunks)
  6. inflate by robot radius + safety margin (done by the caller / node)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class LidarParams:
    lambda_deg: float = 10.0      # ABD incidence-angle parameter
    sigma_r: float = 0.01         # m, range noise
    min_points: int = 3
    max_segment_len: float = 0.4  # m, longer segments are chained into several circles
    r_min: float = 0.05
    r_max: float = 0.5
    max_range: float = 3.0        # m, ignore anything further (not relevant for N*dt*v)


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> tuple[np.ndarray, np.ndarray]:
    r = np.asarray(ranges, float)
    ang = angle_min + angle_increment * np.arange(len(r))
    valid = np.isfinite(r) & (r > max(range_min, 1e-3)) & (r < range_max)
    r = np.where(valid, r, 0.0)
    return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1), valid


def segment(points: np.ndarray, valid: np.ndarray, angle_increment: float,
            p: LidarParams, full_circle: bool = True) -> list[np.ndarray]:
    ranges = np.linalg.norm(points, axis=1)
    lam, dphi = np.radians(p.lambda_deg), abs(angle_increment)
    segs, cur = [], []
    prev = None
    for i in range(len(points)):
        if not valid[i] or ranges[i] > p.max_range:
            if cur:
                segs.append(cur)
            cur, prev = [], None
            continue
        if prev is not None:
            dmax = ranges[prev] * np.sin(dphi) / np.sin(lam - dphi) + 3 * p.sigma_r
            if np.linalg.norm(points[i] - points[prev]) > dmax:
                segs.append(cur)
                cur = []
        cur.append(i)
        prev = i
    if cur:
        segs.append(cur)
    # merge wrap-around (first and last beams adjacent in a 360 deg scan)
    if full_circle and len(segs) >= 2 and segs[0][0] == 0 and segs[-1][-1] == len(points) - 1:
        i, j = segs[-1][-1], segs[0][0]
        dmax = ranges[i] * np.sin(dphi) / np.sin(lam - dphi) + 3 * p.sigma_r
        if np.linalg.norm(points[i] - points[j]) <= dmax:
            segs[0] = segs.pop() + segs[0]
    return [points[s] for s in segs if len(s) >= p.min_points]


def fit_circle(pts: np.ndarray, p: LidarParams) -> tuple[float, float, float]:
    """Kasa algebraic fit: x^2 + y^2 + D x + E y + F = 0."""
    chord = np.linalg.norm(pts[-1] - pts[0])
    centroid = pts.mean(axis=0)
    if len(pts) >= 3:
        A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
        b = -(pts[:, 0] ** 2 + pts[:, 1] ** 2)
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy = -sol[0] / 2, -sol[1] / 2
        r2 = cx * cx + cy * cy - sol[2]
        if r2 > 0:
            r = np.sqrt(r2)
            # accept only if the centre is *behind* the visible arc (seen from the sensor)
            if r <= p.r_max and np.hypot(cx, cy) > np.linalg.norm(centroid):
                return cx, cy, float(np.clip(r, p.r_min, p.r_max))
    # nearly straight chunk / wall: centre at the chord midpoint, pushed back slightly
    r = float(np.clip(0.5 * chord, p.r_min, p.r_max))
    mid = 0.5 * (pts[0] + pts[-1])
    away = mid / max(np.linalg.norm(mid), 1e-9)
    cx, cy = mid + away * 0.5 * r
    return cx, cy, r


def extract_circles(ranges, angle_min, angle_increment, range_min, range_max,
                    p: LidarParams | None = None) -> np.ndarray:
    """Returns an (K, 3) array of [cx, cy, r] in the scan frame (not inflated)."""
    p = p or LidarParams()
    pts, valid = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max)
    full = abs(angle_increment * len(ranges)) > 2 * np.pi - 2 * abs(angle_increment)
    circles = []
    for seg in segment(pts, valid, angle_increment, p, full_circle=full):
        seglen = np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1))
        n_chunks = max(1, int(np.ceil(seglen / p.max_segment_len)))
        for chunk in np.array_split(seg, n_chunks):
            if len(chunk) >= 2:
                circles.append(fit_circle(chunk, p))
    return np.array(circles, float).reshape(-1, 3)


def nearest_k(circles: np.ndarray, robot_xy: np.ndarray, k: int) -> np.ndarray:
    """Keep the K circles with the smallest surface distance to the robot."""
    if len(circles) == 0:
        return circles.reshape(0, 3)
    d = np.hypot(circles[:, 0] - robot_xy[0], circles[:, 1] - robot_xy[1]) - circles[:, 2]
    return circles[np.argsort(d)[:k]]


def simulate_scan(pose: np.ndarray, obstacles: np.ndarray, n_beams: int = 360,
                  range_min: float = 0.12, range_max: float = 3.5, noise: float = 0.0,
                  rng: np.random.Generator | None = None) -> np.ndarray:
    """Ray-cast a 360 deg scan against circles (for tests and the kinematic sim)."""
    ang = pose[2] + np.arange(n_beams) * 2 * np.pi / n_beams
    d = np.stack([np.cos(ang), np.sin(ang)], 1)
    r = np.full(n_beams, np.inf)
    for cx, cy, rad in np.asarray(obstacles).reshape(-1, 3):
        oc = pose[:2] - np.array([cx, cy])
        b = d @ oc
        cterm = oc @ oc - rad * rad
        disc = b * b - cterm
        hit = disc >= 0
        t = -b - np.sqrt(np.where(hit, disc, 0))
        t = np.where(hit & (t > 0), t, np.inf)
        r = np.minimum(r, t)
    if noise > 0:
        rng = rng or np.random.default_rng()
        r = r + rng.normal(0, noise, n_beams)
    r[(r < range_min) | (r > range_max)] = np.inf
    return r
