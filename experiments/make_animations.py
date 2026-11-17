#!/usr/bin/env python3
"""Data-driven animations (MP4 + GIF) from the kinematic simulator.

  python3 experiments/make_animations.py          # -> media/*.mp4, media/*.gif

1. nmpc_horizon      NMPC-CBF on an obstacle scenario: reference, LiDAR-perceived
                     (inflated) circles, the predicted horizon, trail
2. obstacle_compare  same scenario, NMPC-CBF vs the three baselines + CBF-QP filter
3. delay_compare     0.3 s input delay: baselines and NMPC with/without compensation
"""
from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tb3_mpc"))
sys.path.insert(0, str(ROOT / "experiments"))

from analyze import COLORS, INK, LABELS, MUTED  # noqa: E402
from run_benchmark import get_controller  # noqa: E402
from tb3_mpc.core.model import BURGER  # noqa: E402
from tb3_mpc.core.scenarios import make_scenario  # noqa: E402
from tb3_mpc.core.simulator import EpisodeConfig, PlantConfig, run_episode  # noqa: E402

MEDIA = ROOT / "media"
STRIDE = 2      # simulation steps per frame (0.2 s)
FPS = 15        # -> 3x real time


class Recorder:
    """Wraps a controller and keeps what it saw and planned at every step."""

    def __init__(self, ctrl):
        self.ctrl, self.dt, self.name = ctrl, ctrl.dt, ctrl.name
        self.preds, self.obs = [], []
        if hasattr(ctrl, "set_applied_input"):
            self.set_applied_input = ctrl.set_applied_input

    def reset(self):
        self.ctrl.reset()
        self.preds, self.obs = [], []

    def compute(self, t, x, ref, obstacles=None):
        u, info = self.ctrl.compute(t, x, ref, obstacles)
        self.preds.append(info.get("pred_X"))
        self.obs.append(None if obstacles is None else np.array(obstacles))
        return u, info


def encode(frames_dir: Path, name: str, width: int = 720):
    MEDIA.mkdir(exist_ok=True)
    mp4, gif = MEDIA / f"{name}.mp4", MEDIA / f"{name}.gif"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(FPS), "-i", str(frames_dir / "f%05d.png"),
                    "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2,format=yuv420p", "-c:v", "libx264", "-crf", "20", str(mp4)],
                   check=True)
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", str(FPS), "-i", str(frames_dir / "f%05d.png"),
                    "-vf", f"scale={width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];"
                           "[b][p]paletteuse=dither=bayer:bayer_scale=5", str(gif)], check=True)
    print(f"  {mp4.name} ({mp4.stat().st_size / 1e6:.1f} MB), {gif.name} ({gif.stat().st_size / 1e6:.1f} MB)")


def draw_robot(ax, x, color, size=BURGER.robot_radius):
    ax.add_patch(plt.Circle(x[:2], size, color=color, alpha=0.9, zorder=5))
    ax.arrow(x[0], x[1], 0.16 * np.cos(x[2]), 0.16 * np.sin(x[2]), width=0.012, color=INK, zorder=6,
             length_includes_head=True)


def draw_static(ax, sc, lims):
    r = sc["ref"]
    ax.plot(r.x, r.y, color=MUTED, lw=1, ls="--", zorder=1)
    for ox, oy, rad in sc["obstacles"]:
        ax.add_patch(plt.Circle((ox, oy), rad, color="#55544f", zorder=2))
    ax.set_xlim(*lims[0])
    ax.set_ylim(*lims[1])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def limits(*xys, pad=0.35):
    a = np.vstack(xys)
    lo, hi = a.min(0) - pad, a.max(0) + pad
    return (lo[0], hi[0]), (lo[1], hi[1])


def pick_obstacle_seed() -> int:
    """A random-spline seed where NMPC succeeds and every filtered baseline fails (deadlock)."""
    df = pd.read_csv(ROOT / "results" / "obstacles.csv")
    df = df[df.family == "random_spline"]
    piv = df.pivot_table(index="seed", columns="variant", values="success")
    good = piv[(piv.nmpc_cbf == 1) & (piv[["kanayama_cbfqp", "samson_cbfqp", "smc_cbfqp"]].sum(1) == 0)]
    return int(good.index[0]) if len(good) else 0


# ------------------------------------------------------------------ animations
def nmpc_horizon(seed: int):
    sc = make_scenario("random_spline", seed, n_obstacles=2)
    rec = Recorder(get_controller({"variant": "nmpc_cbf"}))
    log = run_episode(rec, sc["ref"], sc["x0"], obstacles=sc["obstacles"],
                      ep=EpisodeConfig(perceive_with_lidar=True), seed=seed)
    X = log["x"]
    lims = limits(np.column_stack([sc["ref"].x, sc["ref"].y]), X[:, :2])
    tmp = Path(tempfile.mkdtemp())
    for i, k in enumerate(range(0, len(X), STRIDE)):
        fig, ax = plt.subplots(figsize=(7.2, 6.0), dpi=100)
        draw_static(ax, sc, lims)
        if rec.obs[k] is not None:
            for ox, oy, rad in rec.obs[k]:
                if ox < 100:
                    ax.add_patch(plt.Circle((ox, oy), rad, fill=False, ls=":", lw=1.2, color=COLORS["nmpc_dc"],
                                            zorder=3))
        ax.plot(X[:k + 1, 0], X[:k + 1, 1], color=COLORS["nmpc"], lw=2, zorder=4)
        P = rec.preds[k]
        ax.plot(P[0], P[1], "o-", color=COLORS["nmpc"], ms=3, lw=1, alpha=0.55, zorder=4)
        xr = log["xr"][k]
        ax.plot(xr[0], xr[1], "x", color=INK, ms=8, mew=2, zorder=6)
        draw_robot(ax, X[k], COLORS["nmpc"])
        ax.set_title(f"NMPC-CBF  ·  t = {log['t'][k]:4.1f} s  ·  solve {1e3 * log['solve_time'][k]:4.1f} ms  ·  "
                     f"clearance {log['clearance'][k]:.2f} m", fontsize=10, color=INK)
        ax.text(0.01, 0.01, "dashed: reference   x: reference now   dots: predicted horizon (N=20)   "
                            "dotted: LiDAR circles (inflated)", transform=ax.transAxes, fontsize=7, color=MUTED)
        fig.savefig(tmp / f"f{i:05d}.png", bbox_inches="tight", facecolor="white")
        plt.close(fig)
    encode(tmp, "nmpc_horizon")
    shutil.rmtree(tmp)


def obstacle_compare(seed: int):
    sc = make_scenario("random_spline", seed, n_obstacles=2)
    variants = ["nmpc_cbf", "kanayama_cbfqp", "samson_cbfqp", "smc_cbfqp"]
    logs = {v: run_episode(get_controller({"variant": v}), sc["ref"], sc["x0"], obstacles=sc["obstacles"],
                           ep=EpisodeConfig(perceive_with_lidar=True), seed=seed) for v in variants}
    n = max(len(l["t"]) for l in logs.values())
    lims = limits(np.column_stack([sc["ref"].x, sc["ref"].y]), *[l["x"][:, :2] for l in logs.values()])
    tmp = Path(tempfile.mkdtemp())
    for i, k in enumerate(range(0, n, STRIDE)):
        fig, axes = plt.subplots(2, 2, figsize=(8.4, 7.6), dpi=90)
        for ax, v in zip(axes.ravel(), variants):
            L = logs[v]
            kk = min(k, len(L["t"]) - 1)
            draw_static(ax, sc, lims)
            ax.plot(L["x"][:kk + 1, 0], L["x"][:kk + 1, 1], color=COLORS[v], lw=2)
            xr = L["xr"][kk]
            ax.plot(xr[0], xr[1], "x", color=INK, ms=7, mew=2)
            draw_robot(ax, L["x"][kk], COLORS[v])
            err = np.hypot(*(L["x"][kk, :2] - xr[:2]))
            ax.set_title(f"{LABELS[v]}   |e| = {err:.2f} m", fontsize=10, color=INK)
        fig.suptitle(f"Same scenario, same LiDAR, same limits  ·  t = {k * 0.1:4.1f} s", fontsize=11, color=INK)
        fig.savefig(tmp / f"f{i:05d}.png", bbox_inches="tight", facecolor="white")
        plt.close(fig)
    encode(tmp, "obstacle_compare")
    shutil.rmtree(tmp)


def delay_compare(seed: int = 4, delay: int = 3):
    sc = make_scenario("random_spline", seed)
    variants = ["nmpc", "nmpc_comp", "kanayama", "smc"]
    plant = PlantConfig(delay_steps=delay)
    logs = {v: run_episode(get_controller({"variant": v}), sc["ref"], sc["x0"], plant=plant,
                           ep=EpisodeConfig(compensate_delay=(v == "nmpc_comp")), seed=seed) for v in variants}
    n = max(len(l["t"]) for l in logs.values())
    lims = limits(np.column_stack([sc["ref"].x, sc["ref"].y]), *[l["x"][:, :2] for l in logs.values()])
    tmp = Path(tempfile.mkdtemp())
    for i, k in enumerate(range(0, n, STRIDE)):
        fig, ax = plt.subplots(figsize=(7.2, 6.0), dpi=100)
        draw_static(ax, sc, lims)
        for v in variants:
            L = logs[v]
            kk = min(k, len(L["t"]) - 1)
            ls = "--" if v == "nmpc_comp" else "-"
            ax.plot(L["x"][:kk + 1, 0], L["x"][:kk + 1, 1], color=COLORS[v], lw=1.8, ls=ls, label=LABELS[v])
            draw_robot(ax, L["x"][kk], COLORS[v], size=0.06)
        ax.legend(loc="upper right", fontsize=8, frameon=False)
        ax.set_title(f"Input delay {delay * 100} ms  ·  t = {k * 0.1:4.1f} s", fontsize=10, color=INK)
        fig.savefig(tmp / f"f{i:05d}.png", bbox_inches="tight", facecolor="white")
        plt.close(fig)
    encode(tmp, "delay_compare")
    shutil.rmtree(tmp)


def main():
    seed = pick_obstacle_seed()
    print(f"obstacle scenario: random_spline seed {seed}")
    print("[nmpc_horizon]")
    nmpc_horizon(seed)
    print("[obstacle_compare]")
    obstacle_compare(seed)
    print("[delay_compare]")
    delay_compare()


if __name__ == "__main__":
    main()
