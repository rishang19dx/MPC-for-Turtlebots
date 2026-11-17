#!/usr/bin/env python3
"""Tables, paired statistics and report figures from results/*.csv.

  python3 experiments/analyze.py            # everything available
Outputs: results/tables/*.md|.tex, results/figures/*.pdf|.png, results/stats.md

Statistics: per cell median [IQR] with a 95% bootstrap CI of the median;
NMPC vs each baseline with a paired Wilcoxon signed-rank test on the same
seeds, Holm-corrected across all comparisons in a table.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy import stats  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results"
FIG = RES / "figures"
TAB = RES / "tables"

# Fixed categorical order (never cycled): colour follows the controller, not its rank.
COLORS = {
    "nmpc": "#2a78d6", "kanayama": "#eb6834", "samson": "#1baf7a", "smc": "#eda100",
    "nmpc_comp": "#6250d6",
    "nmpc_cbf": "#2a78d6", "nmpc_dc": "#e87ba4", "nmpc_pf": "#008300",
    "kanayama_cbfqp": "#eb6834", "samson_cbfqp": "#1baf7a", "smc_cbfqp": "#eda100",
}
LABELS = {
    "nmpc": "NMPC", "kanayama": "Kanayama", "samson": "Samson / backstepping", "smc": "Kinematic SMC",
    "nmpc_comp": "NMPC + delay comp.",
    "nmpc_cbf": "NMPC-CBF", "nmpc_dc": "NMPC-DC", "nmpc_pf": "NMPC-PF",
    "kanayama_cbfqp": "Kanayama + CBF-QP", "samson_cbfqp": "Samson + CBF-QP", "smc_cbfqp": "SMC + CBF-QP",
}
TRACKERS = ["nmpc", "kanayama", "samson", "smc"]
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3de"

plt.rcParams.update({
    "figure.dpi": 150, "savefig.dpi": 200, "font.size": 9, "axes.titlesize": 10,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
    "lines.linewidth": 2.0,
})


def save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"{name}.{ext}", bbox_inches="tight")
    plt.close(fig)
    print(f"  figure {name}")


def med_iqr(x) -> str:
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return "-"
    q1, q2, q3 = np.percentile(x, [25, 50, 75])
    return f"{q2:.4g} [{q1:.3g}, {q3:.3g}]"


def boot_ci(x, n=2000):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < 3:
        return (np.nan, np.nan)
    r = stats.bootstrap((x,), np.median, n_resamples=n, method="percentile", random_state=0)
    return r.confidence_interval.low, r.confidence_interval.high


def holm(pvals):
    p = np.asarray(pvals, float)
    order = np.argsort(p)
    adj = np.empty_like(p)
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(p) - rank) * p[i]))
        adj[i] = running
    return adj


def paired_tests(df, group_col, metric, ref="nmpc", others=("kanayama", "samson", "smc")):
    rows = []
    for g, d in df.groupby(group_col):
        piv = d.pivot_table(index="seed", columns="variant", values=metric)
        for o in others:
            if ref not in piv or o not in piv:
                continue
            pair = piv[[ref, o]].dropna()
            if len(pair) < 5 or np.allclose(pair[ref], pair[o]):
                continue
            w = stats.wilcoxon(pair[ref], pair[o])
            rows.append({group_col: g, "comparison": f"{LABELS[ref]} vs {LABELS[o]}", "n": len(pair),
                         f"median diff ({metric})": float(np.median(pair[ref] - pair[o])),
                         "NMPC better in": f"{100 * np.mean(pair[ref] < pair[o]):.0f}%",
                         "p": w.pvalue})
    out = pd.DataFrame(rows)
    if len(out):
        out["p (Holm)"] = holm(out["p"])
        out["significant"] = out["p (Holm)"] < 0.05
    return out


def write_table(df, name, caption):
    TAB.mkdir(parents=True, exist_ok=True)
    (TAB / f"{name}.md").write_text(f"**{caption}**\n\n" + df.to_markdown(index=False, floatfmt=".4g") + "\n")
    (TAB / f"{name}.tex").write_text(df.to_latex(index=False, float_format="%.4g", caption=caption,
                                                  label=f"tab:{name}", escape=True))
    print(f"  table {name}")


def legend_handles(variants, ls=None):
    from matplotlib.lines import Line2D
    return [Line2D([], [], color=COLORS[v], lw=2, ls=(ls or {}).get(v, "-"), label=LABELS[v]) for v in variants]


# =========================================================================== suites
def tracking():
    df = pd.read_csv(RES / "tracking.csv")
    df = df[df.get("error").isna()] if "error" in df else df
    fams = list(dict.fromkeys(df.family))
    summ = []
    for (fam, v), d in df.groupby(["family", "variant"], sort=False):
        lo, hi = boot_ci(d.rmse_pos)
        summ.append({"family": fam, "controller": LABELS[v], "RMSE pos [m]": med_iqr(d.rmse_pos),
                     "95% CI median": f"[{lo:.3g}, {hi:.3g}]", "max err [m]": med_iqr(d.max_pos),
                     "heading RMSE [rad]": med_iqr(d.rmse_heading), "effort": med_iqr(d.effort),
                     "smoothness ΣΔu²": med_iqr(d.smoothness), "saturated %": med_iqr(d.saturated_pct),
                     "success %": f"{100 * d.success.mean():.0f}"})
    write_table(pd.DataFrame(summ), "tracking_summary", "Obstacle-free tracking, median [IQR] over paired seeds")
    t = paired_tests(df, "family", "rmse_pos")
    if len(t):
        write_table(t, "tracking_wilcoxon", "Paired Wilcoxon signed-rank, NMPC vs baselines (RMSE), Holm-corrected")

    # box plots, one panel per family (small multiples, shared log y)
    n = len(fams)
    fig, axes = plt.subplots(1, n, figsize=(2.1 * n, 3.0), sharey=True)
    axes = np.atleast_1d(axes)
    for ax, fam in zip(axes, fams):
        d = df[df.family == fam]
        data = [d[d.variant == v].rmse_pos.values for v in TRACKERS]
        bp = ax.boxplot(data, widths=0.6, patch_artist=True, showfliers=True,
                        medianprops=dict(color=INK, lw=1.5), flierprops=dict(ms=3, mec=MUTED))
        for patch, v in zip(bp["boxes"], TRACKERS):
            patch.set(facecolor=COLORS[v], edgecolor="white", linewidth=1.0, alpha=0.9)
        ax.set_yscale("log")
        ax.set_title(fam.replace("_", " "))
        ax.set_xticks([])
    axes[0].set_ylabel("position RMSE after 5 s [m]")
    fig.legend(handles=legend_handles(TRACKERS), loc="lower center", ncol=4, bbox_to_anchor=(0.5, -0.06))
    save(fig, "tracking_rmse_box")

    # XY overlays from traces
    tr = np.load(RES / "traces" / "tracking.npz", allow_pickle=True)
    show = [f for f in ["figure8", "random_spline", "point_to_point"] if f in fams]
    fig, axes = plt.subplots(1, len(show), figsize=(3.4 * len(show), 3.2))
    for ax, fam in zip(np.atleast_1d(axes), show):
        ref_drawn = False
        for v in TRACKERS:
            key = f"family={fam}__variant={v}__seed=0"
            if key not in tr:
                continue
            T = tr[key].item()
            if not ref_drawn:
                ax.plot(T["ref_xy"][:, 0], T["ref_xy"][:, 1], color=MUTED, lw=1, ls="--", label="reference")
                ref_drawn = True
            ax.plot(T["x"][:, 0], T["x"][:, 1], color=COLORS[v], lw=1.3, label=LABELS[v])
            ax.plot(*T["x"][0, :2], "o", color=COLORS[v], ms=5, mec="white")
        ax.set_aspect("equal")
        ax.set_title(f"{fam.replace('_', ' ')} (seed 0)")
        ax.set_xlabel("x [m]")
    np.atleast_1d(axes)[0].set_ylabel("y [m]")
    np.atleast_1d(axes)[0].legend(loc="best", fontsize=7)
    save(fig, "tracking_xy")


def robustness():
    df = pd.read_csv(RES / "robustness.csv")
    titles = {"delay_steps": "input delay [steps of 0.1 s]", "noise": "pose noise σ [m]",
              "wheel_sym": "symmetric wheel-radius error", "wheel_asym": "asymmetric wheel error ±",
              "offset": "initial offset [m]"}
    perts = [p for p in titles if p in set(df.perturbation)]
    fig, axes = plt.subplots(1, len(perts), figsize=(2.6 * len(perts), 3.0), sharey=True)
    variants = TRACKERS + ["nmpc_comp"]
    for ax, p in zip(np.atleast_1d(axes), perts):
        d = df[df.perturbation == p]
        for v in variants:
            dv = d[d.variant == v]
            if dv.empty:
                continue
            g = dv.groupby("level").rmse_pos
            q = g.quantile([0.25, 0.5, 0.75]).unstack()
            ls = "--" if v == "nmpc_comp" else "-"
            ax.plot(q.index, q[0.5], color=COLORS[v], ls=ls, marker="o", ms=4)
            ax.fill_between(q.index, q[0.25], q[0.75], color=COLORS[v], alpha=0.15, lw=0)
        ax.set_yscale("log")
        ax.set_xlabel(titles[p])
    np.atleast_1d(axes)[0].set_ylabel("position RMSE [m] (median, IQR band)")
    fig.legend(handles=legend_handles(variants, {"nmpc_comp": "--"}), loc="lower center", ncol=5,
               bbox_to_anchor=(0.5, -0.08))
    save(fig, "robustness_degradation")
    summ = (df.groupby(["perturbation", "level", "variant"])
              .agg(rmse=("rmse_pos", "median"), success=("success", "mean")).reset_index())
    summ["variant"] = summ.variant.map(LABELS)
    summ["success"] = (100 * summ.success).round(0)
    write_table(summ, "robustness_summary", "Robustness sweeps (median RMSE [m], success %)")
    df["cell"] = df.perturbation + "=" + df.level.astype(str)
    t = paired_tests(df, "cell", "rmse_pos")
    if len(t):
        write_table(t, "robustness_wilcoxon", "Paired Wilcoxon per robustness cell, Holm-corrected")


def obstacles():
    df = pd.read_csv(RES / "obstacles.csv")
    order = ["nmpc_cbf", "nmpc_dc", "nmpc_pf", "kanayama_cbfqp", "samson_cbfqp", "smc_cbfqp"]
    rows = []
    for (fam, v), d in df.groupby(["family", "variant"]):
        rows.append({"family": fam, "controller": LABELS[v], "success %": round(100 * d.success.mean()),
                     "collisions": int(d.collided.sum()), "min clearance [m]": med_iqr(d.min_clearance),
                     "final error [m]": med_iqr(d.final_pos_error), "RMSE [m]": med_iqr(d.rmse_pos),
                     "solve p95 [ms]": med_iqr(d.solve_p95_ms)})
    write_table(pd.DataFrame(rows), "obstacles_summary",
                "Obstacle scenarios with LiDAR-perceived obstacles (2 per run)")
    fams = sorted(df.family.unique())
    fig, axes = plt.subplots(1, len(fams), figsize=(3.6 * len(fams), 3.0), sharey=True)
    for ax, fam in zip(np.atleast_1d(axes), fams):
        d = df[df.family == fam]
        data = [d[d.variant == v].min_clearance.values for v in order]
        bp = ax.boxplot(data, widths=0.6, patch_artist=True, medianprops=dict(color=INK, lw=1.5),
                        flierprops=dict(ms=3, mec=MUTED))
        for patch, v in zip(bp["boxes"], order):
            patch.set(facecolor=COLORS[v], edgecolor="white", alpha=0.9,
                      hatch="////" if v.endswith("cbfqp") else None)
        ax.axhline(0, color="#e34948", lw=1)
        ax.set_xticks([])
        ax.set_title(fam.replace("_", " "))
    np.atleast_1d(axes)[0].set_ylabel("min clearance to true obstacle [m]")
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(facecolor=COLORS[v], hatch="////" if v.endswith("cbfqp") else None,
                              edgecolor="white", label=LABELS[v]) for v in order],
               loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.12))
    save(fig, "obstacles_clearance")
    tr = np.load(RES / "traces" / "obstacles.npz", allow_pickle=True)
    for fam in fams:
        fig, ax = plt.subplots(figsize=(4.2, 3.6))
        drawn = False
        for v in ["nmpc_cbf", "nmpc_dc", "kanayama_cbfqp", "smc_cbfqp"]:
            key = f"family={fam}__variant={v}__seed=0"
            if key not in tr:
                continue
            T = tr[key].item()
            if not drawn:
                ax.plot(T["ref_xy"][:, 0], T["ref_xy"][:, 1], color=MUTED, lw=1, ls="--", label="reference")
                for ox, oy, r in T["obstacles"]:
                    ax.add_patch(plt.Circle((ox, oy), r, color="#6b6a64", alpha=0.5))
                    ax.add_patch(plt.Circle((ox, oy), r + 0.105, fill=False, color="#6b6a64", ls=":"))
                drawn = True
            ax.plot(T["x"][:, 0], T["x"][:, 1], color=COLORS[v], lw=1.3, label=LABELS[v])
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"{fam.replace('_', ' ')} with obstacles (seed 0)")
        ax.legend(fontsize=7, loc="best")
        save(fig, f"obstacles_xy_{fam}")


def horizon():
    df = pd.read_csv(RES / "horizon.csv")
    q = df.groupby("N").agg(rmse=("rmse_pos", "median"), solve=("solve_mean_ms", "median"),
                            p95=("solve_p95_ms", "median"))
    fig, axes = plt.subplots(1, 2, figsize=(6.4, 2.8))
    axes[0].plot(q.index, q.rmse, color=COLORS["nmpc"], marker="o")
    axes[0].set_yscale("log")
    axes[0].set_xlabel("horizon N")
    axes[0].set_ylabel("median RMSE [m]")
    axes[1].plot(q.index, q.solve, color=COLORS["nmpc"], marker="o", label="mean")
    axes[1].plot(q.index, q.p95, color=COLORS["nmpc"], marker="s", ls="--", label="p95")
    axes[1].axhline(100, color="#e34948", lw=1)
    axes[1].text(q.index.min(), 100, " deadline (Δt)", va="bottom", color=MUTED, fontsize=7)
    axes[1].set_xlabel("horizon N")
    axes[1].set_ylabel("solve time [ms]")
    axes[1].legend()
    save(fig, "ablation_horizon")
    write_table(q.reset_index().round(4), "ablation_horizon", "Horizon ablation (random splines)")


def gamma():
    df = pd.read_csv(RES / "gamma.csv")
    cbf = df[df["mode"] == "cbf"]
    fig, axes = plt.subplots(1, 2, figsize=(6.6, 2.8))
    for N, ls in [(10, "--"), (20, "-")]:
        d = cbf[cbf.N == N].groupby("gamma")
        axes[0].plot(d.min_clearance.median().index, d.min_clearance.median(), color=COLORS["nmpc_cbf"],
                     ls=ls, marker="o", label=f"CBF, N={N}")
        axes[1].plot(d.success.mean().index, 100 * d.success.mean(), color=COLORS["nmpc_cbf"], ls=ls,
                     marker="o", label=f"CBF, N={N}")
        for mode in ("dc", "pf"):
            dm = df[(df["mode"] == mode) & (df.N == N)]
            axes[0].axhline(dm.min_clearance.median(), color=COLORS[f"nmpc_{mode}"], ls=ls, lw=1.2,
                            label=f"{mode.upper()}, N={N}")
            axes[1].axhline(100 * dm.success.mean(), color=COLORS[f"nmpc_{mode}"], ls=ls, lw=1.2)
    axes[0].set_xscale("log")
    axes[1].set_xscale("log")
    axes[0].set_xlabel("CBF decay rate γ")
    axes[1].set_xlabel("CBF decay rate γ")
    axes[0].set_ylabel("median min clearance [m]")
    axes[1].set_ylabel("success [%]")
    axes[0].legend(fontsize=6.5, ncol=2)
    save(fig, "ablation_gamma")
    t = df.groupby(["N", "mode", "gamma"], dropna=False).agg(
        clearance=("min_clearance", "median"), success=("success", "mean"),
        collisions=("collided", "sum"), final_err=("final_pos_error", "median"),
        solve_p95=("solve_p95_ms", "median")).reset_index()
    write_table(t.round(4), "ablation_gamma", "Obstacle-constraint ablation (point-to-point, 2 obstacles)")


def timing():
    tr = np.load(RES / "traces" / "timing.npz", allow_pickle=True)
    groups: dict = {}
    for key in tr.files:
        parts = dict(kv.split("=", 1) for kv in key.split("__"))
        label = f"{LABELS[parts['variant']]}, N={parts.get('N', '20')}"
        st = tr[key].item()["solve_time"]
        groups.setdefault(label, []).append(1e3 * st[1:])  # first solve (cold) reported separately
    fig, ax = plt.subplots(figsize=(4.6, 3.0))
    palette = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
    rows = []
    for (label, arrs), c in zip(sorted(groups.items()), palette):
        x = np.sort(np.concatenate(arrs))
        ax.plot(x, np.arange(1, len(x) + 1) / len(x), color=c, label=label)
        rows.append({"config": label, "samples": len(x), "mean [ms]": x.mean(), "median": np.median(x),
                     "p95": np.percentile(x, 95), "p99": np.percentile(x, 99), "max": x.max(),
                     "deadline misses %": 100 * np.mean(x > 100)})
    ax.axvline(100, color="#e34948", lw=1)
    ax.text(100, 0.05, " Δt = 100 ms", color=MUTED, fontsize=7)
    ax.set_xscale("log")
    ax.set_xlabel("solve time [ms] (laptop, serial, IPOPT)")
    ax.set_ylabel("empirical CDF")
    ax.legend(fontsize=7, loc="lower right")
    save(fig, "solve_time_cdf")
    write_table(pd.DataFrame(rows).round(2), "solve_time", "NMPC solve-time statistics (warm-started)")


# =========================================================================== report assets
REPORT = ROOT / "report"


def _tex_table(path, caption, label, colspec, header, rows, note=None):
    body = "\n".join(" & ".join(r) + r" \\" for r in rows)
    note_tex = f"\n\\par\\smallskip\\footnotesize {note}" if note else ""
    path.write_text(
        "\\begin{table}[t]\n\\centering\\small\n"
        f"\\caption{{{caption}}}\\label{{{label}}}\n"
        f"\\resizebox{{\\linewidth}}{{!}}{{%\n\\begin{{tabular}}{{{colspec}}}\n\\toprule\n"
        f"{header} \\\\\n\\midrule\n{body}\n\\bottomrule\n\\end{{tabular}}}}{note_tex}\n\\end{{table}}\n")
    print(f"  report table {path.name}")


def _bold_min(vals, fmt):
    finite = [v for v in vals if np.isfinite(v)]
    best = min(finite) if finite else None
    return [("\\textbf{" + fmt(v) + "}") if v == best else fmt(v) for v in vals]


def report_assets():
    """Compact LaTeX tables for report/main.tex and a copy of every figure PDF."""
    (REPORT / "tables").mkdir(parents=True, exist_ok=True)
    (REPORT / "figures").mkdir(parents=True, exist_ok=True)
    for f in FIG.glob("*.pdf"):
        (REPORT / "figures" / f.name).write_bytes(f.read_bytes())
    mm = lambda v: f"{1e3 * v:.2f}"  # noqa: E731

    if (RES / "tracking.csv").exists():
        df = pd.read_csv(RES / "tracking.csv")
        tests = paired_tests(df, "family", "rmse_pos")
        rows = []
        for fam in dict.fromkeys(df.family):
            d = df[df.family == fam]
            med = [d[d.variant == v].rmse_pos.median() for v in TRACKERS]
            ft = tests[tests.family == fam] if len(tests) else tests
            diff = ft.filter(like="median diff").iloc[:, 0] if len(ft) else pd.Series(dtype=float)
            better_all = len(ft) == 3 and bool((ft.significant & (diff < 0)).all())
            worse_any = bool((ft.significant & (diff > 0)).any()) if len(ft) else False
            cells = _bold_min(med, mm)
            cells[0] += "$^{*}$" if better_all else ("$^{\\dagger}$" if worse_any else "")
            mx = [d[d.variant == v].max_pos.median() for v in TRACKERS]
            rows.append([fam.replace("_", " ")] + cells + _bold_min(mx, mm))
        _tex_table(REPORT / "tables" / "tracking.tex",
                   "Obstacle-free tracking: median position RMSE and median max error over 30 paired seeds "
                   "(mm, $t\\in[5\\,\\mathrm{s},T_{\\mathrm{ref}}]$). Best in bold.",
                   "tab:tracking", "l" + "r" * 8,
                   "& \\multicolumn{4}{c}{RMSE [mm]} & \\multicolumn{4}{c}{max error [mm]} \\\\\n"
                   "\\cmidrule(lr){2-5}\\cmidrule(lr){6-9}\nfamily & NMPC & Kanayama & Samson & SMC & NMPC & Kanayama & Samson & SMC",
                   rows, "$^{*}$ NMPC significantly better than every baseline; $^{\\dagger}$ at least one baseline significantly better than NMPC (paired Wilcoxon signed-rank, Holm-corrected, $p<0.05$).")

    if (RES / "robustness.csv").exists():
        df = pd.read_csv(RES / "robustness.csv")
        names = {"delay_steps": "delay [ms]", "noise": "pose noise $\\sigma$ [mm]", "wheel_sym": "wheel radius error [\\%]",
                 "wheel_asym": "wheel asymmetry $\\pm$[\\%]", "offset": "initial offset [m]"}
        scale = {"delay_steps": 100, "noise": 1000, "wheel_sym": 100, "wheel_asym": 100, "offset": 1}
        vs = TRACKERS + ["nmpc_comp"]
        rows = []
        for p in names:
            for lvl in sorted(df[df.perturbation == p].level.unique()):
                d = df[(df.perturbation == p) & (df.level == lvl)]
                med = [d[d.variant == v].rmse_pos.median() if (d.variant == v).any() else np.nan for v in vs]
                cells = _bold_min(med, lambda v: "--" if not np.isfinite(v) else mm(v))
                rows.append([names[p], f"{lvl * scale[p]:g}"] + cells)
        _tex_table(REPORT / "tables" / "robustness.tex",
                   "Robustness sweeps on random splines: median position RMSE [mm] (30 paired seeds per cell). "
                   "Perturbations act on the plant only.", "tab:robustness", "llrrrrr",
                   "perturbation & level & NMPC & Kanayama & Samson & SMC & NMPC + delay comp.", rows)

    if (RES / "obstacles.csv").exists():
        df = pd.read_csv(RES / "obstacles.csv")
        order = ["nmpc_cbf", "nmpc_dc", "nmpc_pf", "kanayama_cbfqp", "samson_cbfqp", "smc_cbfqp"]
        rows = []
        for v in order:
            r = [LABELS[v]]
            for fam in ["random_spline", "point_to_point"]:
                d = df[(df.family == fam) & (df.variant == v)]
                r += [f"{100 * d.success.mean():.0f}", f"{int(d.collided.sum())}", f"{100 * d.min_clearance.median():.1f}"]
            rows.append(r)
        _tex_table(REPORT / "tables" / "obstacles.tex",
                   "Obstacle scenarios (2 LiDAR-perceived obstacles, 30 paired seeds per family). Clearance is the "
                   "median minimum distance between robot footprint and true obstacle boundary.",
                   "tab:obstacles", "lrrrrrr",
                   "& \\multicolumn{3}{c}{random spline} & \\multicolumn{3}{c}{point-to-point} \\\\\n"
                   "\\cmidrule(lr){2-4}\\cmidrule(lr){5-7}\ncontroller & success [\\%] & collisions & clearance [cm] "
                   "& success [\\%] & collisions & clearance [cm]", rows)

    if (RES / "gamma.csv").exists():
        df = pd.read_csv(RES / "gamma.csv")
        rows = []
        for (mode, g), d in df.groupby(["mode", "gamma"], dropna=False):
            r = [mode.upper() + ("" if pd.isna(g) else f", $\\gamma={g:g}$")]
            for N in (10, 20):
                dn = d[d.N == N]
                r += [f"{100 * dn.min_clearance.median():.1f}", f"{100 * dn.success.mean():.0f}",
                      f"{dn.solve_p95_ms.median():.1f}"]
            rows.append(r)
        _tex_table(REPORT / "tables" / "gamma.tex",
                   "Obstacle-constraint ablation (point-to-point, 2 obstacles): median clearance, success rate and "
                   "median per-run p95 solve time. $\\gamma=1$ recovers the distance constraint.",
                   "tab:gamma", "lrrrrrr",
                   "& \\multicolumn{3}{c}{$N=10$} & \\multicolumn{3}{c}{$N=20$} \\\\\n"
                   "\\cmidrule(lr){2-4}\\cmidrule(lr){5-7}\narm & clear. [cm] & succ. [\\%] & p95 [ms] "
                   "& clear. [cm] & succ. [\\%] & p95 [ms]", rows)

    if (TAB / "solve_time.md").exists() and (RES / "traces" / "timing.npz").exists():
        tr = np.load(RES / "traces" / "timing.npz", allow_pickle=True)
        groups: dict = {}
        for key in tr.files:
            parts = dict(kv.split("=", 1) for kv in key.split("__"))
            groups.setdefault((LABELS[parts["variant"]], int(parts.get("N", 20))), []).append(
                1e3 * tr[key].item()["solve_time"][1:])
        rows = []
        for (lab, N), arrs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
            x = np.concatenate(arrs)
            rows.append([lab, str(N), str(len(x))] + [f"{v:.1f}" for v in
                        (x.mean(), np.median(x), np.percentile(x, 95), np.percentile(x, 99), x.max())]
                        + [f"{100 * np.mean(x > 100):.1f}"])
        cpu = "Intel i5-1240P"
        _tex_table(REPORT / "tables" / "solve_time.tex",
                   f"Warm-started NMPC solve times (IPOPT, serial, {cpu}); first (cold) solve excluded. "
                   "Random splines, 2 obstacles for CBF rows.", "tab:solvetime", "lrrrrrrrr",
                   "configuration & $N$ & solves & mean & median & p95 & p99 & max [ms] & misses [\\%]", rows)

    tuned = RES / "tuned_gains.json"
    if tuned.exists():
        import json
        t = json.loads(tuned.read_text())
        def fmt_param(p, v):
            return "$" + p.replace("_", r"\_") + f"={v:g}$"

        rows = [[LABELS[k], ", ".join(fmt_param(p, v) for p, v in b["params"].items()),
                 f"{1e3 * b['median_rmse']:.2f}"] for k, b in t.items()]
        _tex_table(REPORT / "tables" / "tuned_gains.tex",
                   "Frozen gains after equal-budget tuning (24 candidates per controller, held-out seeds).",
                   "tab:gains", "llr", "controller & tuned parameters & tuning RMSE [mm]", rows)


def main():
    for name, fn in [("tracking", tracking), ("robustness", robustness), ("obstacles", obstacles),
                     ("horizon", horizon), ("gamma", gamma), ("timing", timing)]:
        if (RES / f"{name}.csv").exists():
            print(f"[{name}]")
            fn()
        else:
            print(f"[{name}] skipped (no results/{name}.csv)")
    print("[report assets]")
    report_assets()


if __name__ == "__main__":
    main()
