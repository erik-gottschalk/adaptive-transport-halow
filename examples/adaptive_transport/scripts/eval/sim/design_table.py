"""Table S6 of the paper: each row replaces one component of the firmware (time counter
T = 20, ageing T_a = 20, probe floor 1/x, per-class loss debt R = 250) and keeps the rest.

    python3 design_table.py [runs]      (default 2000; cached in opt_out/ like every opt_run sweep)

Scenarios, as in the table's caption, each with the same number of runs (seeds 0 .. runs-1): the six
binding cells of Table S5 (1 h runs), the three mixed workloads of the bias-correction
paragraph (k_sweep.MIXES, 4 h), the reference channel (4 h) and the hardware schedule (its
2 -> 20 % phase for the step columns). 2000 runs, because at 200 the worst mix class scatters by
about +-0.012 L between batches (mix_ref_seeds.py). Mean
saving: the plain mean over these eleven scenarios, the oracle included (it plans every message
with the true loss rate and its class's L).
Variants without ageing run the fixed probe and feedback cycle (opt_run.probe_mode); the
mixed-workload runs bypass opt_run, so every configuration states FW_RANDOM_PROBE explicitly.
"""
import math
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from k_sweep import MIXES, simulate_mix
from opt_run import SCHED, probe_mode, run, summary
from ge_channels import longer
from paper_sweep import REF

# k_sweep's mixes, on the paper's reference channel, and the fixed-loss mixes as long as it
# (ge_channels.HOURS)
_cfg, _ls = MIXES["hw mix ref"]
MIXES = {k: (longer(c), ls) for k, (c, ls) in MIXES.items()}
MIXES["hw mix ref"] = (dict(REF, **{k: _cfg[k] for k in ("L", "MIX_L", "MIX_SHARE")}), _ls)

FW = dict(EST="fw", FW_DEBT_R=250)
INV = dict(EST="invs", MINV=16, AGE_T=20)
VARIANTS = {
    "firmware": FW,
    "forgetting per UDP sample": dict(EST="blend", AGE_T=20, FW_DEBT_R=250),
    "longer memory, T = 80, no debt": dict(EST="fw", FW_TE=80),
    "inverse sampling, M = 16, no debt": INV,
    "inverse sampling + change detector, no debt": dict(INV, CS_H=5),
    "fixed schedule: forced UDP and TCP every x": dict(FW, FW_TA=1e9, FW_RANDOM_PROBE=0),
    "no debt": dict(EST="fw"),
    "budget scaled to 0.8L, no debt": dict(EST="fw", ALPHA=0.8),
    "one shared debt account": dict(FW, FW_DEBT_PERCLASS=0),
    "oracle": dict(EST="oracle"),
}
for cfg in VARIANTS.values():
    cfg["FW_RANDOM_PROBE"] = probe_mode(cfg)
RUNS = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
BINDING = [(0.025, 0.05), (0.025, 0.10), (0.025, 0.20), (0.05, 0.10), (0.05, 0.20), (0.10, 0.20)]


def main():
    names, cfgs = list(VARIANTS), list(VARIANTS.values())
    bind = [[summary(r, L) for r in run(cfgs, chan=dict(CHAN="sched", SCHEDULE=f"3600:{p}", L=L), seeds=RUNS)]
            for L, p in BINDING]
    ref = [summary(r, REF["L"]) for r in run(cfgs, chan=REF, seeds=RUNS)]
    sched_runs = run(cfgs, chan=SCHED, seeds=RUNS)
    sched = [summary(r, SCHED["L"]) for r in sched_runs]
    jobs = [(mname, chan, Ls, cfg) for mname, (chan, Ls) in MIXES.items() for cfg in cfgs]
    with ThreadPoolExecutor(30) as ex:
        out = list(ex.map(lambda j: simulate_mix(j[1], j[3], RUNS), jobs))
    mixes = {m: [] for m in MIXES}
    for (mname, _, Ls, _), (tot, s) in zip(jobs, out):
        mixes[mname].append(([tot[3 * j + 1] / tot[3 * j] / L for j, L in enumerate(Ls)], s["saving"]))

    print(f"{'variant':46s} | bind max  range       | worst mix | ref loss/L saving fb/1000 | mean saving (n) |"
          f" step loss/L react | sched saving")
    for i, v in enumerate(names):
        lb = [c[i]["loss_L"] for c in bind]
        savings = [c[i]["saving"] for c in bind] + [ref[i]["saving"], sched[i]["saving"]]
        mix = [m[i] for m in mixes.values() if m[i] is not None]
        savings += [m[1] for m in mix]
        worst = max(max(m[0]) for m in mix) if mix else float("nan")
        runs = sched_runs[i]
        n1 = sum(x["ph"][1][0] for x in runs)
        loss20 = sum(x["ph"][1][2] for x in runs) / n1 / SCHED["L"]
        react = sorted(x["st"][1][3] if not math.isnan(x["st"][1][3]) else 1e9 for x in runs)[len(runs) // 2]
        print(f"{v:46s} | {max(lb):8.2f}  {min(lb):.2f}-{max(lb):.2f} | {worst:9.2f} | "
              f"{ref[i]['loss_L']:10.2f} {100 * ref[i]['saving']:5.1f}% {1000 * ref[i]['fb_h'] / ref[i]['msgs_h']:7.0f} | "
              f"{100 * np.mean(savings):10.1f}% ({len(savings):2d}) | {loss20:11.2f} {react:5.0f} | "
              f"{100 * sched[i]['saving']:6.1f}%")
    print("\nper binding cell loss/L: " + "  ".join(f"{100 * L:g}/{100 * p:g}" for L, p in BINDING))
    for i, v in enumerate(names):
        print(f"  {v:46s} " + " ".join(f"{c[i]['loss_L']:.2f}" for c in bind))
    print("\nper mix, class loss/L:")
    for mname, rows in mixes.items():
        print(f"  {mname}: " + "  ".join(f"{names[i][:18]}: {'/'.join(f'{x:.2f}' for x in r[0])}"
                                        for i, r in enumerate(rows) if r is not None))


if __name__ == "__main__":
    main()
