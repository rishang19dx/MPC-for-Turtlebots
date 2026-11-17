#!/usr/bin/env python3
"""Paired Monte Carlo benchmark in the kinematic simulator (tier 1).

Every controller sees the *same* seed -> same reference (for random families),
same initial offset, same noise sequence, same obstacles. Results: one row per
episode in results/<suite>.csv, plus a few full traces in results/traces/.

Suites
  tracking     6 families x 4 controllers, obstacle-free
  robustness   random_spline under delay / noise / wheel mismatch / large offsets
  obstacles    random_spline + point_to_point with 2 LiDAR-perceived obstacles:
               NMPC (cbf, dc, pf) vs baselines wrapped in the CBF-QP filter
  horizon      NMPC horizon ablation N in {5, 10, 15, 20, 30}
  gamma        obstacle-constraint ablation: DC, PF, CBF gamma sweep, at N = 10 / 20
  timing       serial run (no parallel load) for honest solve-time statistics

Usage
  python3 experiments/run_benchmark.py --suite tracking --seeds 30
  python3 experiments/run_benchmark.py --suite all --seeds 30 --workers 8   # overnight
  python3 experiments/run_benchmark.py --suite tracking --seeds 3 --quick   # smoke test
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "tb3_mpc"))

from tb3_mpc.core import make_controller  # noqa: E402
from tb3_mpc.core.metrics import episode_metrics  # noqa: E402
from tb3_mpc.core.scenarios import make_scenario  # noqa: E402
from tb3_mpc.core.simulator import EpisodeConfig, PlantConfig, run_episode  # noqa: E402

RESULTS = ROOT / "results"
BASELINES = ["kanayama", "samson", "smc"]
BENCH_NMPC = {"max_wall_time": 1.0}  # deterministic: never truncate; deadline misses are reported
TRANSIENT = 5.0  # s excluded from steady-state tracking metrics

# variant label -> (controller name, params, safety_filter)
VARIANTS = {
    "nmpc": ("nmpc", {"obstacle_mode": "none"}, False),
    "kanayama": ("kanayama", {}, False),
    "samson": ("samson", {}, False),
    "smc": ("smc", {}, False),
    "nmpc_cbf": ("nmpc", {"obstacle_mode": "cbf"}, False),
    "nmpc_dc": ("nmpc", {"obstacle_mode": "dc"}, False),
    "nmpc_pf": ("nmpc", {"obstacle_mode": "pf"}, False),
    "kanayama_cbfqp": ("kanayama", {}, True),
    "samson_cbfqp": ("samson", {}, True),
    "smc_cbfqp": ("smc", {}, True),
}


def task_list(suite: str, seeds: int, quick: bool) -> list[dict]:
    S = range(seeds)
    T = []
    if suite == "tracking":
        fams = ["circle", "figure8", "sinusoid", "lissajous", "random_spline", "point_to_point"]
        if quick:
            fams = ["figure8", "random_spline"]
        for fam in fams:
            for s in S:
                for v in ["nmpc", *BASELINES]:
                    T.append(dict(suite=suite, family=fam, seed=s, variant=v))
    elif suite == "robustness":
        sweeps = {
            "delay_steps": [0, 1, 2, 4],
            "noise": [0.0, 0.005, 0.01, 0.02],               # m; heading noise = 2.5 * this (rad)
            "wheel_sym": [0.0, 0.05, 0.10],                   # both wheels scaled by (1 - level)
            "wheel_asym": [0.0, 0.02, 0.05],                  # right +level, left -level
            "offset": [0.1, 0.2, 0.4],                        # m; heading offset = level * 5 rad/m
        }
        if quick:
            sweeps = {k: v[::2] for k, v in sweeps.items()}
        for pert, levels in sweeps.items():
            for lvl in levels:
                for s in S:
                    vs = ["nmpc", *BASELINES] + (["nmpc_comp"] if pert == "delay_steps" and lvl else [])
                    for v in vs:
                        T.append(dict(suite=suite, family="random_spline", seed=s, variant=v,
                                      perturbation=pert, level=lvl))
    elif suite == "obstacles":
        for fam in ["random_spline", "point_to_point"]:
            for s in S:
                for v in ["nmpc_cbf", "nmpc_dc", "nmpc_pf", "kanayama_cbfqp", "samson_cbfqp", "smc_cbfqp"]:
                    T.append(dict(suite=suite, family=fam, seed=s, variant=v, n_obstacles=2, lidar=True))
    elif suite == "horizon":
        for N in [5, 10, 15, 20, 30]:
            for s in S:
                T.append(dict(suite=suite, family="random_spline", seed=s, variant="nmpc", N=N))
    elif suite == "gamma":
        arms = [("dc", None), ("pf", None)] + [("cbf", g) for g in [0.05, 0.1, 0.2, 0.3, 0.5, 1.0]]
        for N in [10, 20]:
            for mode, g in arms:
                for s in S:
                    T.append(dict(suite=suite, family="point_to_point", seed=s, variant=f"nmpc_{mode}",
                                  N=N, mode=mode, gamma=g, n_obstacles=2))
    elif suite == "timing":
        for s in range(min(seeds, 10)):
            for v, N in [("nmpc", 20), ("nmpc_cbf", 20), ("nmpc_cbf", 10), ("nmpc_cbf", 30)]:
                T.append(dict(suite=suite, family="random_spline", seed=s, variant=v, N=N,
                              n_obstacles=2 if v != "nmpc" else 0, realtime=True))
    else:
        raise ValueError(suite)
    return T


_CTRL_CACHE: dict = {}
_TUNED_PATH = RESULTS / "tuned_gains.json"
TUNED = ({k: v["params"] for k, v in json.loads(_TUNED_PATH.read_text()).items()}
         if _TUNED_PATH.exists() else {})


def get_controller(task: dict):
    v = task["variant"]
    base = "nmpc" if v == "nmpc_comp" else v
    name, params, sf = VARIANTS[base]
    params = {**TUNED.get(name, {}), **params}  # frozen tuned gains, then variant overrides
    if name == "nmpc":
        params.update(BENCH_NMPC)
        if task.get("realtime"):
            params["max_wall_time"] = 0.08
        if "N" in task:
            params["N"] = task["N"]
        if task.get("mode"):
            params["obstacle_mode"] = task["mode"]
        if task.get("gamma") is not None:
            params["gamma"] = task["gamma"]
    key = (name, tuple(sorted(params.items())), sf)
    if key not in _CTRL_CACHE:
        _CTRL_CACHE[key] = make_controller(name, params, safety_filter=sf)
    return _CTRL_CACHE[key]


def run_task(task: dict, keep_trace: bool = False) -> tuple[dict, dict | None]:
    pert, lvl = task.get("perturbation"), task.get("level", 0.0)
    off = lvl if pert == "offset" else 0.2
    sc = make_scenario(task["family"], task["seed"], offset_pos=off,
                       offset_ang=(lvl * 5.0 if pert == "offset" else np.pi / 6),
                       n_obstacles=task.get("n_obstacles", 0))
    plant = PlantConfig()
    ep = EpisodeConfig(perceive_with_lidar=task.get("lidar", False))
    if pert == "delay_steps":
        plant.delay_steps = int(lvl)
        ep.compensate_delay = task["variant"] == "nmpc_comp"
    elif pert == "noise":
        plant.pos_noise, plant.th_noise = lvl, 2.5 * lvl
    elif pert == "wheel_sym":
        plant.wheel_scale_r = plant.wheel_scale_l = 1.0 - lvl
    elif pert == "wheel_asym":
        plant.wheel_scale_r, plant.wheel_scale_l = 1.0 + lvl, 1.0 - lvl
    ctrl = get_controller(task)
    log = run_episode(ctrl, sc["ref"], sc["x0"], obstacles=sc["obstacles"], plant=plant, ep=ep,
                      seed=task["seed"])
    m = episode_metrics(log, transient=TRANSIENT)
    row = {**task, **m, "ref_duration": sc["ref"].duration}
    trace = None
    if keep_trace:
        trace = {k: log[k] for k in ("t", "x", "xr", "u", "u_raw", "solve_time", "clearance")}
        trace["obstacles"] = sc["obstacles"]
        trace["ref_xy"] = np.column_stack([sc["ref"].x[::10], sc["ref"].y[::10]])
    return row, trace


def _worker(args):
    task, keep = args
    try:
        return run_task(task, keep)
    except Exception as e:  # noqa: BLE001 - record and continue
        return {**task, "error": repr(e)}, None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", default="tracking",
                    choices=["tracking", "robustness", "obstacles", "horizon", "gamma", "timing", "all"])
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--traces", type=int, default=2, help="keep full traces for seeds < this")
    args = ap.parse_args()

    suites = ["tracking", "robustness", "obstacles", "horizon", "gamma", "timing"] if args.suite == "all" else [args.suite]
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "traces").mkdir(exist_ok=True)
    for suite in suites:
        tasks = task_list(suite, args.seeds, args.quick)
        workers = 1 if suite == "timing" else args.workers
        print(f"[{suite}] {len(tasks)} episodes on {workers} worker(s)", flush=True)
        tic = time.time()
        rows, traces = [], {}
        # timing: keep every trace so the analysis can build per-step solve-time CDFs
        jobs = [(t, suite == "timing" or t["seed"] < args.traces) for t in tasks]
        if workers == 1:
            results = map(_worker, jobs)
        else:
            pool = ProcessPoolExecutor(max_workers=workers)
            futs = [pool.submit(_worker, j) for j in jobs]
            results = (f.result() for f in as_completed(futs))
        for i, (row, tr) in enumerate(results, 1):
            rows.append(row)
            if tr is not None:
                key = "__".join(f"{k}={row.get(k)}" for k in ("family", "variant", "seed", "perturbation",
                                                               "level", "N", "gamma") if row.get(k) is not None)
                traces[key] = tr
            if i % max(1, len(tasks) // 20) == 0:
                print(f"  {i}/{len(tasks)}  ({time.time() - tic:.0f}s)", flush=True)
        if workers != 1:
            pool.shutdown()
        df = pd.DataFrame(rows)
        out = RESULTS / f"{suite}.csv"
        df.to_csv(out, index=False)
        np.savez_compressed(RESULTS / "traces" / f"{suite}.npz",
                            **{k: np.array(v, dtype=object) for k, v in traces.items()})
        n_err = int(df["error"].notna().sum()) if "error" in df else 0
        print(f"[{suite}] wrote {out} ({len(df)} rows, {n_err} errors) in {time.time() - tic:.0f}s", flush=True)


if __name__ == "__main__":
    main()
