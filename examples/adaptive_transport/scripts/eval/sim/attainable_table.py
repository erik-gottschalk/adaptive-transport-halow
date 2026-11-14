"""Long-run loss against what the channel lets a controller deliver (supplement Tables S7 and
S8, and the long reporting intervals of Section S4-A).

    python3 attainable_table.py

On a varying channel loss/L = 1 is out of reach for any controller that never exceeds L at the current
loss rate: where q < L it sends everything as UDP and delivers only q. The attainable loss
L* = E[min(q, L)] over a variant's own messages is the loss of the clairvoyant bound, which spends the
budget fully wherever the channel allows it, so loss/L* = 1 and saving / clairvoyant saving = 1 are
what a perfect controller reaches. Same channels, seeds and variants as paper_sweep.py; the runs are
tagged L_CLAIR=1 so they are simulated afresh with the l_clair_msg column (common random numbers
make every other number identical to paper_sweep's).
"""
import os

from design_table import VARIANTS as DESIGN
from opt_run import PAPER_RUNS, run, summary
from opt_sweep import sweep
from paper_sweep import CHANNELS, INTERVAL_REF, INTERVALS, PARAMS, REF, key

# as sched20_paper.py (40 % phases at 20 %), which gives the paper's two hardware-schedule rows
SCHED20 = dict(CHAN="sched", SCHEDULE="300:0.02,300:0.20,300:0.02,240:0.05,60:0.20,300:0.05,300:0.20,300:0.02",
               L=0.05)

SEEDS = int(os.environ.get("SEEDS", PAPER_RUNS))
FAMS = {
    "no debt": dict(EST="fw"),
    "firmware": dict(EST="fw", FW_DEBT_R=250),
    "oracle": dict(EST="oracle"),
}
TAG = dict(L_CLAIR=1)


def row(label, rows):
    by = {r["family"]: r["summary"] for r in rows}
    att = by["firmware"]["attain_L"]
    cells = []
    for f in FAMS:
        s = by[f]
        cells.append(f"{s['loss_L'] / s['attain_L']:5.2f} {100 * s['viol']:5.1f}% "
                     f"{100 * s['saving']:5.1f}% {s['saving'] / s['saving_clair']:5.2f}")
    chk = "  ".join(f"{by[f]['loss_L']:.2f}/{by[f]['attain_L']:.2f}" for f in FAMS)
    print(f"{label:28s} {att:5.2f}   " + "   ".join(cells) + f"   | loss/L, L*/L: {chk}")


def main():
    hdr = "loss/L* >1.5L saving  /clv"
    print(f"{'channel':28s} {'L*/L':>5s}   " + "   ".join(f"{f + ': ' + hdr:>29s}" for f in FAMS))
    for name, chan in CHANNELS.items():
        if name.startswith("hardware schedule"):    # the paper's rows: the schedule as the testbed ran it
            chan = dict(chan, SCHEDULE=SCHED20["SCHEDULE"])     # keeps BURST and the ge settings
        row(name, sweep(key("attain " + name), chan, [FAMS], SEEDS, alphas=[1.0], extra=TAG))
    print("\n== message interval, reference channel")
    for d in INTERVALS:
        row(f"{d} s", sweep(key(f"attain interval {d}"), dict(INTERVAL_REF, MSG_DT=d, DUR=7200), [FAMS], SEEDS,
                            alphas=[1.0], extra=TAG))
    print("  (per variant: loss / its own attainable loss L*, share of 5-min windows above 1.5 L, saving"
          " against pure TCP, and saving / the clairvoyant saving on the same messages; L*/L of the firmware)")

    # The reference-channel columns of Table S6 (design_table.py) and Table S8 (paper_sweep.py)
    for title, variants in (("design variants (Table S6)", DESIGN), ("half-lives (Table S8)", PARAMS)):
        print(f"\n== {title}, reference channel: loss/L, L*/L, loss/L*, saving, saving / clairvoyant")
        cfgs = [dict(c, **TAG) for c in variants.values()]
        for name, runs in zip(variants, run(cfgs, chan=REF, seeds=SEEDS)):
            s = summary(runs, REF["L"])
            print(f"  {name:46s} {s['loss_L']:.2f}  {s['attain_L']:.2f}  {s['loss_L'] / s['attain_L']:.2f}"
                  f"  {100 * s['saving']:5.1f}%  {s['saving'] / s['saving_clair']:.2f}")


if __name__ == "__main__":
    main()
