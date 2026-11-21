"""Phase 7 analysis: tables, numbers and figures for the paper.

    python3 paper/analyze.py            (from the repository root)

Reads results/phase7/<tag>.csv (one row per episode, written by
experiments/run_phase7.m) and writes
    paper/generated/tab_nav.tex     navigation results table
    paper/generated/tab_margin.tex  conformal margin table
    paper/generated/macros.tex      numbers quoted in the text
    paper/figs/fig_acc_safety.pdf   prediction accuracy vs navigation safety
    paper/figs/fig_margin.pdf       margin size vs contacts / goal reached
Every configuration runs the same 120 episodes, so comparisons are paired.
"""
import glob
import os
import numpy as np
import pandas as pd
from scipy import stats
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results", "phase7")
GEN = os.path.join(ROOT, "paper", "generated")
FIG = os.path.join(ROOT, "paper", "figs")
os.makedirs(GEN, exist_ok=True)
os.makedirs(FIG, exist_ok=True)
RNG = np.random.default_rng(0)

# Prediction accuracy (ADE / FDE, m) from exp_phase2_prediction and
# Phase 3 training (README); ETH is the held-out test scene.
ACC = {  # name: (val ADE, val FDE, ETH ADE, ETH FDE)
    "cv": (0.440, 0.972, 1.075, 2.282),
    "kalman": (0.443, 0.977, 1.071, 2.277),
    "sf": (0.461, 0.993, 1.056, 2.238),
    "mlp": (0.431, 0.938, 1.030, 2.091),
    "lstm": (0.440, 0.958, 1.003, 2.045),
    "tcn": (0.430, 0.936, 1.021, 2.058),
    "sgcnn": (0.429, 0.934, 1.044, 2.155),
}
LABEL = {"none": "none (current pos.)", "cv": "Const. velocity", "kalman": "Kalman CV",
         "sf": "Social Force", "mlp": "MLP", "lstm": "LSTM", "tcn": "Temporal CNN",
         "sgcnn": "Social-grid CNN"}
SHORT = {"cv": "CV", "lstm": "LSTM"}
ROBOT = {"r1": "A* + pure pursuit", "r2": "A* + DWA", "r3": "ST-A* + MPC"}
UNIV = ("students001", "students003")


def load():
    D = {}
    for f in sorted(glob.glob(os.path.join(RES, "*.csv"))):
        tag = os.path.basename(f)[:-4]
        d = pd.read_csv(f)
        d["key"] = d["scene"] + "_" + d["t0"].round(1).astype(str) + "_" + d["flip"].astype(str)
        D[tag] = d.set_index("key").sort_index()
    return D


def boot_ci(x, n=4000):
    x = np.asarray(x, float)
    m = RNG.choice(x, (n, len(x))).mean(1)
    return x.mean(), np.percentile(m, 2.5), np.percentile(m, 97.5)


def summary(d):
    c, lo, hi = boot_ci(d["collisions"])
    u = d[d["scene"].isin(UNIV)]
    return dict(n=len(d), success=100 * d["success"].mean(), reached=100 * d["reached"].mean(),
                contacts=c, lo=lo, hi=hi, fault=d["faultCollisions"].mean(),
                univ_contacts=u["collisions"].mean(), univ_success=100 * u["success"].mean(),
                time=d.loc[d["reached"] == 1, "timeToGoal"].mean(),
                freeze=100 * d["freezeFrac"].mean(), pushed=d["crowdDisp"].mean(),
                intr=d["intrusions"].mean(), plan=d["planMs"].mean(),
                minclr=d["minClear"].median())


def paired(a, b, col="collisions"):
    """Mean difference b - a over shared episodes, bootstrap CI, Wilcoxon p."""
    k = a.index.intersection(b.index)
    diff = (b.loc[k, col] - a.loc[k, col]).to_numpy(float)
    m, lo, hi = boot_ci(diff)
    p = stats.wilcoxon(diff, zero_method="zsplit").pvalue if np.any(diff != 0) else 1.0
    return m, lo, hi, p, len(k)


def fmt_p(p):
    return "$<$0.001" if p < 0.001 else f"{p:.3f}"


def main():
    D = load()
    if not D:
        print("no results yet")
        return
    S = {t: summary(d) for t, d in D.items()}
    print(f"{len(S)} configurations: " + ", ".join(sorted(S)))
    for t in sorted(S):
        s = S[t]
        print(f"{t:16s} n={s['n']:3d} succ {s['success']:5.1f}% reach {s['reached']:5.1f}% "
              f"contacts {s['contacts']:.2f} [{s['lo']:.2f},{s['hi']:.2f}] fault {s['fault']:.2f} "
              f"univ {s['univ_contacts']:.2f}/{s['univ_success']:.0f}% time {s['time']:.1f} "
              f"freeze {s['freeze']:.1f}% pushed {s['pushed']:.2f}")

    # ---- navigation table -------------------------------------------------
    rows = []
    for r in ("r1", "r2", "r3"):
        tags = [t for t in S if t.startswith(r + "_") and "_cp" not in t and "replay" not in t]
        order = ["none", "cv", "kalman", "sf", "mlp", "lstm", "tcn", "sgcnn"]
        tags.sort(key=lambda t: order.index(t.split("_", 1)[1]))
        if not tags:
            continue
        rows.append(f"\\multicolumn{{9}}{{l}}{{\\textit{{{ROBOT[r]}}}}}\\\\")
        for t in tags:
            p = t.split("_", 1)[1]
            s = S[t]
            ade = f"{ACC[p][2]:.2f}" if p in ACC else "--"
            rows.append(f"\\quad {LABEL[p]} & {ade} & {s['success']:.0f} & {s['reached']:.0f} & "
                        f"{s['contacts']:.2f} {{\\scriptsize[{s['lo']:.2f}, {s['hi']:.2f}]}} & "
                        f"{s['fault']:.2f} & {s['univ_contacts']:.2f} & {s['time']:.1f} & {s['pushed']:.2f}\\\\")
    with open(os.path.join(GEN, "tab_nav.tex"), "w") as f:
        f.write("\n".join(rows) + "\n")

    # ---- margin table / figure -------------------------------------------
    mrows = []
    for p in ("cv", "lstm"):
        for lvl in (0, 50, 70, 90):
            t = f"r3_{p}" if lvl == 0 else f"r3_{p}_cp{lvl}"
            if t not in S:
                continue
            s = S[t]
            lv = "none" if lvl == 0 else f"{lvl}\\%"
            mrows.append(f"{SHORT[p]} & {lv} & {s['success']:.0f} & {s['reached']:.0f} & "
                         f"{s['contacts']:.2f} & {s['univ_success']:.0f} & {s['freeze']:.1f} & {s['pushed']:.2f}\\\\")
    with open(os.path.join(GEN, "tab_margin.tex"), "w") as f:
        f.write("\n".join(mrows) + "\n")

    # ---- macros --------------------------------------------------------------
    M = {}

    def put(name, val):
        M[name] = val

    for t, s in S.items():
        base = t.replace("_", "")
        for k in ("success", "reached", "univ_success", "freeze"):
            put(f"{base}{k.replace('_', '')}", f"{s[k]:.0f}")
        for k in ("contacts", "fault", "univ_contacts", "pushed", "time"):
            put(f"{base}{k.replace('_', '')}", f"{s[k]:.2f}" if k != "time" else f"{s[k]:.1f}")
    # paired comparisons
    comps = [("r3_cv", "r3_lstm"), ("r3_cv", "r3_sgcnn"), ("r3_cv", "r3_tcn"), ("r3_cv", "r3_mlp"),
             ("r3_cv", "r3_kalman"), ("r3_cv", "r3_sf"), ("r3_none", "r3_cv"), ("r2_cv", "r3_cv"),
             ("r1_cv", "r2_cv"), ("r2_none", "r2_cv"), ("r3_cv", "r3_cv_cp90"), ("r3_cv", "r3_cv_cp50"),
             ("r3_cv_replay", "r3_cv"), ("r2_cv", "r2_lstm")]
    lines = []
    for a, b in comps:
        if a in D and b in D:
            m, lo, hi, p, n = paired(D[a], D[b])
            name = (a + "vs" + b).replace("_", "")
            put(name + "d", f"{m:+.2f}")
            put(name + "ci", f"[{lo:+.2f}, {hi:+.2f}]")
            put(name + "p", fmt_p(p))
            lines.append(f"{b} - {a}: contacts {m:+.2f} [{lo:+.2f},{hi:+.2f}] p={p:.3g} (n={n})")
    print("\n".join(lines))
    # accuracy vs safety correlation over R3 predictors
    xs, ys = [], []
    for p in ACC:
        if f"r3_{p}" in S:
            xs.append(ACC[p][3]); ys.append(S[f"r3_{p}"]["contacts"])
    if len(xs) >= 3:
        rho, pr = stats.spearmanr(xs, ys)
        put("rhoaccsafe", f"{rho:+.2f}")
        put("rhoaccsafep", f"{pr:.2f}")
        print(f"Spearman(ETH FDE, contacts) over R3 predictors: rho={rho:+.2f} p={pr:.2f}")
    words = dict(zip("0123456789", ["zero", "one", "two", "three", "four", "five", "six",
                                     "seven", "eight", "nine"]))
    with open(os.path.join(GEN, "macros.tex"), "w") as f:
        for k, v in sorted(M.items()):
            name = "".join(words.get(ch, ch) for ch in k)
            f.write(f"\\newcommand{{\\R{name}}}{{{v}}}\n")

    # ---- figure: accuracy vs safety -----------------------------------------
    plt.rcParams.update({"font.size": 8, "font.family": "serif", "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(3.4, 2.3))
    NAME = {"cv": "CV", "kalman": "Kalman", "sf": "SF", "mlp": "MLP", "lstm": "LSTM",
            "tcn": "TCN", "sgcnn": "SG-CNN"}
    OFF = {"cv": (4, -9), "kalman": (4, 2), "sf": (4, 2), "mlp": (4, 2), "lstm": (-4, -10),
           "tcn": (3, 4), "sgcnn": (4, -9)}
    for r, mk, col, dx in (("r2", "s", "#4C72B0", 0.004), ("r3", "o", "#C44E52", -0.004)):
        X, Y, L, H = [], [], [], []
        for p in ACC:
            t = f"{r}_{p}"
            if t in S:
                X.append(ACC[p][3] + dx); Y.append(S[t]["contacts"])
                L.append(S[t]["contacts"] - S[t]["lo"]); H.append(S[t]["hi"] - S[t]["contacts"])
                if r == "r3":
                    below = p in ("cv", "lstm")
                    yy = S[t]["lo"] if below else S[t]["hi"]
                    ax.annotate(NAME[p], (ACC[p][3] + dx, yy), textcoords="offset points",
                                xytext=(6 if p == "kalman" else 0, -7 if below else 2), fontsize=6.5,
                                color=col, ha="left" if p == "kalman" else "center")
        if X:
            ax.errorbar(X, Y, yerr=[L, H], fmt=mk, color=col, ms=3.5, capsize=2, lw=0.8,
                        label=ROBOT[r])
        if f"{r}_none" in S:
            y0 = S[f"{r}_none"]["contacts"]
            ax.axhline(y0, color=col, ls=":", lw=0.8)
            ax.text(2.33, y0 + 0.03, f"{ROBOT[r]}, no prediction", color=col, fontsize=6, ha="right",
                    va="bottom")
    ax.set_xlabel("Prediction error on ETH, FDE (m)")
    ax.set_ylabel("Contacts per episode")
    ax.legend(frameon=False, fontsize=7, loc="lower right")
    ax.set_ylim(0, 1.7)
    ax.set_xlim(2.015, 2.33)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "fig_acc_safety.pdf"))
    plt.close(fig)

    # ---- figure: margins -------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.4, 1.9))
    for p, col, mk, dx in (("cv", "#4C72B0", "s", -1.2), ("lstm", "#C44E52", "o", 1.2)):
        lv, c, lo, hi, rc, us = [], [], [], [], [], []
        for lvl in (0, 50, 70, 90):
            t = f"r3_{p}" if lvl == 0 else f"r3_{p}_cp{lvl}"
            if t in S:
                lv.append(lvl + dx); c.append(S[t]["contacts"])
                lo.append(S[t]["contacts"] - S[t]["lo"]); hi.append(S[t]["hi"] - S[t]["contacts"])
                rc.append(S[t]["reached"]); us.append(S[t]["univ_success"])
        if lv:
            ax1.errorbar(lv, c, yerr=[lo, hi], fmt=mk + "-", color=col, ms=3, lw=0.9, capsize=1.5,
                         label=SHORT[p])
            ax2.plot(lv, rc, mk + "-", color=col, ms=3, lw=0.9, label=f"{SHORT[p]} reached")
            ax2.plot(lv, us, mk + ":", color=col, ms=3, lw=0.9, label=f"{SHORT[p]} Univ success")
    ax1.set_ylabel("Contacts per episode")
    ax1.set_ylim(0, 1.3)
    ax2.set_ylabel("Episodes (%)")
    ax2.set_ylim(0, 100)
    for ax in (ax1, ax2):
        ax.set_xticks([0, 50, 70, 90])
        ax.set_xticklabels(["none", "50", "70", "90"])
        ax.set_xlabel("Margin coverage (%)")
    ax1.legend(frameon=False, fontsize=6, loc="lower left")
    ax2.legend(frameon=False, fontsize=5.5, loc="center left", bbox_to_anchor=(0.0, 0.52))
    fig.tight_layout(pad=0.3, w_pad=0.8)
    fig.savefig(os.path.join(FIG, "fig_margin.pdf"))
    plt.close(fig)


if __name__ == "__main__":
    main()
