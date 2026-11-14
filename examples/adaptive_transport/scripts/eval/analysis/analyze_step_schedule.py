"""Step schedule on hardware (paper Fig. 4 and Section IV-C2): per-phase delivered loss / L and UDP / f_opt
of the step captures (loss debt, random probe, schedule with the 40 % phases at 20 %), reaction
times, feedback round-trips per hour, whole-schedule loss and saving per message sent against plain
TCP at each phase's loss.

    python3 analyze_step_schedule.py
"""
import math
import re
import statistics as st
import sys
from pathlib import Path

import numpy as np

EVAL = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(EVAL / "analysis"))
import analyze_dynamic_cd as a  # noqa: E402
import summarize_pwr_runs as spr  # noqa: E402

E_TCP = {0.0: 24.448, 0.05: 28.486, 0.10: 32.435, 0.20: 42.617}   # hw_table.txt, mJ per attempt
# A TCP send that asked the server for a UDP count: one feedback round-trip.
FEEDBACK_RE = re.compile(r"TCP demo packet sent .*expected_udp=[1-9]")


def e_tcp(p):
    ks = sorted(E_TCP)
    return float(np.interp(p, ks, [E_TCP[k] for k in ks]))


def mean_ci(xs):
    m = st.mean(xs)
    if len(xs) < 2:
        return m, math.nan
    return m, st.stdev(xs) / math.sqrt(len(xs)) * {2: 12.706, 3: 4.303, 4: 3.182}.get(len(xs), 2.776)


def report(label, campaign):
    manifests = sorted((EVAL / "measurement_data" / campaign).glob("pwr_*.json"))
    if not manifests:
        print(f"\n===== {label}: no captures in measurement_data/{campaign}, skipped")
        return []
    runs = [a.load_run(p) for p in manifests]
    print(f"\n===== {label}: {len(runs)} runs ({campaign})")

    print("per phase: loss/L, udp/f_opt (mean of runs, +- t-CI half-width)")
    for k, (t0, t1, p) in enumerate(runs[0]["phases"]):
        rows = [a.window_stats(r, t0, t1, p) for r in runs]
        lo, lc = mean_ci([r["loss_vs_L"] for r in rows])
        uo, uc = mean_ci([r["udp_vs_opt"] for r in rows])
        print(f"  {k + 1}: {100 * p:4.0f}% {t1 - t0:4.0f}s  loss/L {lo:.2f} +-{lc:.2f} "
              f"[{' '.join(f'{x:.2f}' for x in [r['loss_vs_L'] for r in rows])}]  udp/f_opt {uo:.2f} +-{uc:.2f}")

    print("transitions: reaction (s to halfway in p_udp_hat)")
    for i, tr in enumerate(a.transitions(runs[0], 120)):
        reacts = [a.transitions(r, 120)[i]["react_s"] for r in runs]
        fin = [x for x in reacts if not math.isnan(x)]
        print(f"  {tr['label']:12s} react median {st.median(fin) if fin else math.nan:.0f} s "
              f"[{' '.join(f'{x:.0f}' for x in reacts)}]")

    # Whole schedule: loss / L, saving per message sent against TCP at the same phase loss, and
    # feedback round-trips per hour of the console (warm-up included: the same for every run).
    tot = []
    cache = spr.load_cache()
    for r, man in zip(runs, manifests):
        pk = r["packets"]
        pk = pk[(pk[:, 0] >= 0) & (pk[:, 0] < r["phases"][-1][1])]   # the capture, not the warm-up
        n = len(pk)
        lost = n - pk[:, 2].sum()
        ref = 0.0
        for t0, t1, p in r["phases"]:
            ref += ((pk[:, 0] >= t0) & (pk[:, 0] < t1)).sum() * e_tcp(p)
        row = spr.process_one(man, cache)
        e_att = row["out"]["energy"]["mJ_per_packet_attempted"]
        lines = man.with_name(man.stem + ".console.log").read_text(errors="replace").splitlines()
        fb = sum(1 for ln in lines if FEEDBACK_RE.search(ln))
        sends = sum(1 for ln in lines if a.DECISION_RE.match(ln))
        tot.append((lost / n / r["L"], 1 - e_att / (ref / n), 3600 * fb / sends))
    spr.save_cache(cache)
    lo, lc = mean_ci([x[0] for x in tot])
    so, sc = mean_ci([x[1] for x in tot])
    fo, fc = mean_ci([x[2] for x in tot])
    print(f"whole schedule: loss/L {lo:.2f} +-{lc:.2f}  saving per sent {100 * so:.1f}% +-{100 * sc:.1f}  "
          f"feedback/h {fo:.0f} +-{fc:.0f}  "
          f"[{' '.join(f'{x[0]:.2f}/{100 * x[1]:.1f}%/{x[2]:.0f}' for x in tot)}]")
    return runs


report("step schedule", "step")
