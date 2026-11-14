"""NOTE: the k rows need the controller and sim_opt of commit 36984c1a (FW_DEBT_K was removed
afterwards; on later commits FW_DEBT_K is ignored and those rows run without a debt).

Sweep of the loss debt's horizon in expected losses, k (2026-10-05).

The plan uses L (1 - D / k): every owed loss withholds 1/k of a class's budget, a debt of k withholds
all of it, and k -> infinity is no debt. Compared with no debt, fixed horizons in messages (R) and the
oracle, all on the firmware estimator with the hardware table:

    python3 k_sweep.py

Selection rule, fixed before the results: the k that maximizes the mean saving over all scenarios
subject to long-run loss <= 1.02 L in every binding cell and every mixed-workload class (4 h runs).

Scenarios: binding fixed-loss cells (p > L, p <= 20 % where the hardware table is measured), slack
cells (p < L), varying channels (incl. L = 1 %), the hardware step schedule, two mixed workloads,
and the reference channel at one message per minute.
"""
import math

import numpy as np

from mix_study import simulate as simulate_mix
from opt_analyze import fopt
from opt_run import PHYS, SCHED, run, summary

DUR = 14400
BINDING = [(0.01, 0.02), (0.01, 0.04), (0.01, 0.08), (0.025, 0.05), (0.025, 0.10), (0.025, 0.20),
           (0.05, 0.10), (0.05, 0.20), (0.10, 0.20)]
SLACK = [(0.01, 0.005), (0.025, 0.01), (0.05, 0.02), (0.10, 0.05)]
REF = dict(PHYS, TAU_C=5.7, SIG_S=5)
CHANNELS = {
    "reference": REF,
    "volatile": dict(REF, TAU_S=180, SIG_S=6, INT_EVERY=600, FADE_EVERY=900),
    "weak link": dict(REF, M0=10),
    "ref L=2.5%": dict(REF, L=0.025),
    "ref L=1%": dict(REF, L=0.01),
}
MIXES = {
    "hw mix 15%": (dict(CHAN="sched", SCHEDULE=f"{DUR}:0.15", L=0.1, MIX_L="0.5,0.1,0.01", MIX_SHARE="0.6,0.3,0.1"),
                   [0.5, 0.1, 0.01]),
    "binding mix 10%": (dict(CHAN="sched", SCHEDULE=f"{DUR}:0.10", L=0.025, MIX_L="0.05,0.025,0.01",
                             MIX_SHARE="0.5,0.4,0.1"), [0.05, 0.025, 0.01]),
    "hw mix ref": (dict(REF, L=0.1, MIX_L="0.5,0.1,0.01", MIX_SHARE="0.6,0.3,0.1"), [0.5, 0.1, 0.01]),
}
KS = [2, 3, 5, 7.5, 10, 15, 20, 30, 50]
VARIANTS = {"no debt": dict(EST="fw")}
VARIANTS.update({f"R={r}": dict(EST="fw", FW_DEBT_R=r) for r in (100, 300, 1000)})
VARIANTS.update({f"k={k:g}": dict(EST="fw", FW_DEBT_K=k) for k in KS})
VARIANTS["oracle"] = dict(EST="oracle")


def fixed(L, p):
    return dict(CHAN="sched", SCHEDULE=f"{DUR}:{p}", L=L)


def main():
    names, cfgs = list(VARIANTS), list(VARIANTS.values())
    bind = [[summary(r, L) for r in run(cfgs, chan=fixed(L, p), seeds=200)] for L, p in BINDING]
    slack = [[summary(r, L) for r in run(cfgs, chan=fixed(L, p), seeds=200)] for L, p in SLACK]
    chans = {n: [summary(r, float(c["L"])) for r in run(cfgs, chan=c, seeds=200)] for n, c in CHANNELS.items()}
    sched_runs = run(cfgs, chan=SCHED, seeds=500)
    sched = [summary(r, 0.05) for r in sched_runs]
    slow = [summary(r, 0.05) for r in run(cfgs, chan=dict(REF, MSG_DT=60, DUR=7200), seeds=100)]
    ps = [float(x.split(":")[1]) for x in SCHED["SCHEDULE"].split(",")]
    mixes = {}
    for mname, (chan, Ls) in MIXES.items():
        seeds = 200
        mixes[mname] = []
        for cfg in cfgs:
            if cfg["EST"] == "oracle":
                mixes[mname].append(None)
                continue
            tot, _ = simulate_mix(chan, cfg, seeds)
            mixes[mname].append([tot[3 * j + 1] / tot[3 * j] / L for j, L in enumerate(Ls)])

    oracle = names.index("oracle")
    print(f"binding cells (L/p): " + " ".join(f"{100 * L:g}/{100 * p:g}" for L, p in BINDING))
    print(f"{'variant':10s} | binding loss/L  max  mean  >1.5L(5min) gap-to-oracle | slack saving |"
          f" channels max loss/L, mean saving | sched 2->20 react 40->2udp | 60s loss/L sav |"
          f" mixes worst class loss/L | overall mean saving")
    rows = []
    for i, v in enumerate(names):
        lb = [c[i]["loss_L"] for c in bind]
        gap = np.mean([100 * (c[oracle]["saving"] - c[i]["saving"]) for c in bind])
        viol = np.mean([c[i]["viol"] for c in bind])
        ss = np.mean([100 * c[i]["saving"] for c in slack])
        cl = [chans[n][i]["loss_L"] for n in CHANNELS]
        cs = np.mean([100 * chans[n][i]["saving"] for n in CHANNELS])
        runs = sched_runs[i]
        n1 = sum(x["ph"][1][0] for x in runs)
        loss20 = sum(x["ph"][1][2] for x in runs) / n1 / 0.05
        react = sorted(x["st"][1][3] if not math.isnan(x["st"][1][3]) else 1e9 for x in runs)[len(runs) // 2]
        n7 = sum(x["ph"][7][0] for x in runs)
        udp40 = sum(x["ph"][7][1] for x in runs) / n7 / fopt(ps[7], 0.05)
        mixw = max((max(m[i]) for m in mixes.values() if m[i] is not None), default=float("nan"))
        allsav = np.mean([100 * c[i]["saving"] for c in bind + slack] + [100 * chans[n][i]["saving"] for n in CHANNELS]
                         + [100 * sched[i]["saving"]])
        rows.append((v, max(lb), mixw, allsav))
        print(f"{v:10s} | {max(lb):19.2f} {np.mean(lb):5.2f} {viol:11.3f} {gap:12.1f} pt | {ss:11.1f}% |"
              f" {max(cl):14.2f} {cs:15.1f}% | {loss20:11.2f} {react:5.0f} {udp40:8.2f} |"
              f" {slow[i]['loss_L']:9.2f} {100 * slow[i]['saving']:4.1f}% | {mixw:22.2f} | {allsav:8.1f}%")
    print("\nper-cell binding loss / L:")
    for i, v in enumerate(names):
        print(f"  {v:10s} " + " ".join(f"{c[i]['loss_L']:5.2f}" for c in bind))
    print("\nper-class mix loss / L (classes as listed in MIXES):")
    for mname, vals in mixes.items():
        print(f"  {mname}: " + "  ".join(f"{v}: {'/'.join(f'{x:.2f}' for x in vals[i])}"
                                       for i, v in enumerate(names) if vals[i] is not None))
    ok = [r for r in rows if r[0][:2] in ("k=", "R=") and r[1] <= 1.02 and r[2] <= 1.02]
    print("\nselection (max overall saving, binding and mix classes <= 1.02 L): " +
          (max(ok, key=lambda r: r[3])[0] if ok else "no k satisfies the constraint"))


if __name__ == "__main__":
    main()
