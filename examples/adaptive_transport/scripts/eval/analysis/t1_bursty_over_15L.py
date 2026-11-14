"""IEEE version, analysis task T1: share of 5-minute windows whose loss exceeds 1.5 L on the bursty
schedule (Table II, column ">1.5L"), for the plain-UDP and plain-TCP captures, by exactly
the method of the adaptive row in analyze_ge_schedule.main(): the capture's messages (load(), with
failed TCP connects added as lost, capture() for the schedule's span), windows of WIN_S = 300 s by
send time, a window counting if its fraction of undelivered messages exceeds 1.5 L. L = 5 %, the
adaptive captures' budget. The adaptive row is recomputed alongside as a check (paper: 0.7 %).

    python3 t1_bursty_over_15L.py
"""
import statistics as st
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_ge_schedule as g  # noqa: E402

L = 0.05

for proto in ("udp", "tcp", "adp"):
    mans, runs = g.load(proto)
    shares = []
    for man, r in zip(mans, runs):
        pk = g.capture(r)
        w = (pk[:, 0] // g.WIN_S).astype(int)
        over = [((pk[w == k, 2] == 0).mean() > 1.5 * L) for k in np.unique(w)]
        shares.append(float(np.mean(over)))
        print(f"{proto}  {man.name}: {sum(over)}/{len(over)} windows above 1.5L = {100 * shares[-1]:.1f}%")
    print(f"{proto}  mean over {len(shares)} captures: {100 * st.mean(shares):.1f}%\n")
