"""Paper runs on the hardware step schedule as the testbed ran it (40 % phases at 20 %): Table
S7's two schedule rows and Table S8's schedule columns.

    python3 sched20_paper.py
"""
import math

from opt_analyze import fopt
from opt_run import PAPER_RUNS, run, summary
SCHED20 = dict(CHAN="sched", SCHEDULE="300:0.02,300:0.20,300:0.02,240:0.05,60:0.20,300:0.05,300:0.20,300:0.02",
               L=0.05)   # as step20_sim.py

ps = [float(x.split(":")[1]) for x in SCHED20["SCHEDULE"].split(",")]
L = SCHED20["L"]
MAIN = {"no debt": dict(EST="fw"), "firmware": dict(EST="fw", FW_DEBT_R=250), "oracle": dict(EST="oracle")}
PARAMS = {f"T={te} Ta={'off' if ta > 1e6 else ta}": dict(EST="fw", FW_TE=te, FW_TA=ta, FW_DEBT_R=250)
          for te, ta in ((20, 10), (20, 20), (20, 40), (20, 1e9), (10, 20), (40, 20))}


def ph(runs, i, what):
    n = sum(x["ph"][i][0] for x in runs)
    if what == "loss":
        return sum(x["ph"][i][2] for x in runs) / n / L
    return sum(x["ph"][i][1] for x in runs) / n / fopt(ps[i], L)


for label, chan in (("i.i.d.", SCHED20), ("bursts", dict(SCHED20, BURST=3))):
    print(f"== Table S7 row: hardware schedule, {label}")
    for name, runs in zip(MAIN, run(list(MAIN.values()), chan=chan, seeds=PAPER_RUNS)):
        s = summary(runs, L)
        print(f"  {name:10s} loss/L {s['loss_L']:.2f}  >1.5L {100 * s['viol']:.1f}% ({s['viol']:.5f})  saving {100 * s['saving']:.1f}%")

print("== Table params, schedule columns: 2->20 loss/L, reaction, UDP/f_opt in the last 2 % phase")
for name, runs in zip(PARAMS, run(list(PARAMS.values()), chan=SCHED20, seeds=PAPER_RUNS)):
    react = sorted(x["st"][1][3] if not math.isnan(x["st"][1][3]) else 1e9 for x in runs)
    print(f"  {name:14s} {ph(runs, 1, 'loss'):.2f}  {react[len(react) // 2]:.0f} s  {ph(runs, 7, 'udp'):.2f}")
