"""(Fixed-loss scenarios run 4 h: a rare class needs long runs to show its steady state.)
Loss debt in a mixed-criticality workload: one shared account (A) vs. one account per class (C).

    python3 mix_study.py

Per scenario and variant: each class's long-run loss / L and UDP share (pooled over seeds), and
the energy saving against pure TCP. The firmware's own mixed-criticality hardware run (no debt,
2026-10-03) is the check for the first scenario: telemetry 0.30 L, sensor 1.06 L, alarm 0.83 L.
"""
import os
import subprocess
import tempfile

import numpy as np

from opt_run import PAPER_RUNS, SCHED, parse, summary

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim_opt")
from ge_channels import REF, longer  # the paper's reference channel

HW_MIX = dict(MIX_L="0.5,0.1,0.01", MIX_SHARE="0.6,0.3,0.1")
SCENARIOS = {
    "hardware mix, 15 % i.i.d.": dict(CHAN="sched", SCHEDULE="14400:0.15", L=0.1, **HW_MIX),
    "paper mix (70/29/1), 15 %": dict(CHAN="sched", SCHEDULE="14400:0.15", L=0.1,
                                      MIX_L="0.5,0.1,0.01", MIX_SHARE="0.70,0.29,0.01"),
    "all binding (5/2.5/1 %), 10 %": dict(CHAN="sched", SCHEDULE="14400:0.10", L=0.025,
                                          MIX_L="0.05,0.025,0.01", MIX_SHARE="0.5,0.4,0.1"),
    "hardware mix, reference channel": dict(REF, L=0.1, **HW_MIX),
    "hardware mix, hardware schedule": dict(SCHED, L=0.1, **HW_MIX),
}
# the fixed-loss scenarios run as long as the reference channel (ge_channels.HOURS)
SCENARIOS = {k: longer(c) for k, c in SCENARIOS.items()}
VARIANTS = {
    "no debt": dict(EST="fw"),
    "A shared": dict(EST="fw", FW_DEBT_R=250, FW_DEBT_PERCLASS=0),
    "C per class": dict(EST="fw", FW_DEBT_R=250, FW_DEBT_PERCLASS=1),
    "C per class, R=100": dict(EST="fw", FW_DEBT_R=100, FW_DEBT_PERCLASS=1),
    "C per class, R=30": dict(EST="fw", FW_DEBT_R=30, FW_DEBT_PERCLASS=1),
}


def simulate(chan, cfg, seeds):
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
        path = f.name
    env = dict(os.environ, **{k: str(v) for k, v in dict(chan, **cfg, SEEDS=seeds, MIXOUT=path).items()})
    out = subprocess.run([SIM], env=env, check=True, capture_output=True, text=True).stdout
    mix = np.loadtxt(path, ndmin=2)
    os.unlink(path)
    runs = [parse(x) for x in out.splitlines()]
    return mix.sum(axis=0), summary(runs, float(chan["L"]))


def main():
    for name, chan in SCENARIOS.items():
        Ls = [float(x) for x in chan["MIX_L"].split(",")]
        seeds = PAPER_RUNS
        print(f"\n== {name}  (classes L = {', '.join(f'{x:g}' for x in Ls)})")
        print(f"{'variant':22s}" + "".join(f"   L={x:<6g} loss/L  UDP" for x in Ls) + "   saving")
        for v, cfg in VARIANTS.items():
            tot, s = simulate(chan, cfg, seeds)
            cells = ""
            for k, L in enumerate(Ls):
                n, lost, udp = tot[3 * k: 3 * k + 3]
                cells += f"            {lost / n / L:6.2f} {udp / n:4.2f}"
            print(f"{v:22s}{cells}   {100 * s['saving']:5.1f}%")


if __name__ == "__main__":
    main()
