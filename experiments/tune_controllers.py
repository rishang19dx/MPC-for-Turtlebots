#!/usr/bin/env python3
"""Equal-budget grid tuning of every controller on a HELD-OUT seed set.

Fairness rule: each controller gets the same number of candidate configurations
(~24) evaluated on the same tuning scenarios (seeds 1000+, disjoint from the
benchmark seeds 0..M-1). Objective: median steady-state RMSE over the tuning
set, plus a large penalty per failed run. The winners are frozen to
results/tuned_gains.json, which run_benchmark.py picks up automatically.

  python3 experiments/tune_controllers.py --seeds 10 --workers 8
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tb3_mpc"))

from tb3_mpc.core import make_controller  # noqa: E402
from tb3_mpc.core.metrics import episode_metrics  # noqa: E402
from tb3_mpc.core.scenarios import make_scenario  # noqa: E402
from tb3_mpc.core.simulator import run_episode  # noqa: E402

TUNE_SEED0 = 1000
FAMILIES = ["random_spline", "figure8", "point_to_point"]


def grid(d: dict) -> list[dict]:
    keys = list(d)
    return [dict(zip(keys, vals)) for vals in itertools.product(*d.values())]


GRIDS = {  # 24 candidates each
    "kanayama": grid({"k_x": [0.5, 1.0, 2.0, 4.0], "k_y": [10.0, 25.0, 50.0, 100.0, 200.0, 400.0]}),
    "samson": grid({"zeta": [0.5, 0.7, 0.9, 1.2], "b": [5.0, 15.0, 30.0, 60.0, 120.0, 240.0]}),
    "smc": grid({"k2": [2.0, 4.0, 8.0], "k3": [1.0, 2.0], "phi": [0.02, 0.05, 0.1, 0.2]}),
    "nmpc": grid({"q_xy": [5.0, 10.0, 30.0], "q_th": [0.5, 2.0], "r_v": [0.1, 0.5],
                  "s_v": [0.3, 1.0]}),
}


def evaluate(args):
    name, params, seeds = args
    p = dict(params)
    if name == "nmpc":
        p.update(obstacle_mode="none", max_wall_time=1.0)
    ctrl = make_controller(name, p)
    rmses, fails = [], 0
    for fam in FAMILIES:
        for s in seeds:
            sc = make_scenario(fam, s)
            m = episode_metrics(run_episode(ctrl, sc["ref"], sc["x0"], seed=s), transient=5.0)
            rmses.append(m["rmse_pos"])
            fails += not m["success"]
    score = float(np.median(rmses) + 1.0 * fails)
    return name, params, score, float(np.median(rmses)), fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args()
    seeds = list(range(TUNE_SEED0, TUNE_SEED0 + args.seeds))
    jobs = [(n, p, seeds) for n, g in GRIDS.items() for p in g]
    best: dict = {}
    with ProcessPoolExecutor(args.workers) as pool:
        for name, params, score, med, fails in pool.map(evaluate, jobs):
            if name not in best or score < best[name]["score"]:
                best[name] = {"params": params, "score": score, "median_rmse": med, "fails": fails}
    out = ROOT / "results" / "tuned_gains.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(best, indent=2))
    for n, b in best.items():
        print(f"{n:9s} {b['params']}  median RMSE {b['median_rmse']:.4f} m  fails {b['fails']}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
