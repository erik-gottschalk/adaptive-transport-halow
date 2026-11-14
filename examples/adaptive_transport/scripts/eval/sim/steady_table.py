"""Simulation columns of the paper's Table S5: the hardware grid's binding cells, without
the debt, the firmware, and the oracle, long-run loss / L and saving per message sent against pure
TCP. Same configurations and run count as design_table.py, so the runs are shared through the cache.

    python3 steady_table.py [runs]      (default 2000 runs of 1 h per cell)
"""
import sys

from design_table import BINDING, VARIANTS
from opt_run import run, summary

RUNS = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
COLS = ["no debt", "firmware", "oracle"]

print(f"{'L':>5} {'p':>4} | " + " | ".join(f"{c:>17s}" for c in COLS) + " | clairvoyant")
for L, p in BINDING:
    res = run([VARIANTS[c] for c in COLS], chan=dict(CHAN="sched", SCHEDULE=f"3600:{p}", L=L), seeds=RUNS)
    cells = [summary(r, L) for r in res]
    clair = cells[COLS.index("firmware")]["saving_clair"]     # on the firmware's own messages
    print(f"{100 * L:5.1f} {100 * p:4.0f} | " + " | ".join(
        f"{c['loss_L']:6.2f} L {100 * c['saving']:5.1f}%" for c in cells)
        + (f" | {100 * clair:10.1f}%" if clair is not None else " |         --"))
