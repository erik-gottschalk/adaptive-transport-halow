"""The alarm class of the hardware mixed workload against run length (Section S4-C:
"under [the fixed schedule] the alarm class needs a day to settle below 1.02L, where the firmware
needs 8 hours (1000 runs per run length)"; and Mixed Workloads: 0.99L on the bursty schedule).

Hardware mix (telemetry L = 0.5, sensor 0.1, alarm 0.01, shares 60/30/10 %) on the testbed's bursty
schedule (ge_channels.REF_4H: runs of 3, final firmware model), repeated to each run length; the
firmware and the fixed feedback schedule of design_table.py. Per run length: the alarm class's
pooled loss / L, its standard error over runs, and the saving against pure TCP.

    python3 mix_alarm_run_length.py    -> run_paper.sh simulations: mix_alarm_run_length.txt

Mixed-workload runs bypass opt_run's cache (mix_study.simulate), so this simulates every time.
"""
import os
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from design_table import VARIANTS
from ge_channels import GE_SCHEDULE, REF_4H
from opt_run import parse, summary

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim_opt")
RUNS = int(os.environ.get("MIX_RUNS", 1000))
HOURS = (4, 8, 12, 24, 48, 168)
ALARM, ALARM_L = 2, 0.01            # class index in MIX_L, and its budget
MIX = dict(L=0.1, MIX_L="0.5,0.1,0.01", MIX_SHARE="0.6,0.3,0.1")
SHOWN = {"firmware": "firmware", "fixed schedule": "fixed schedule: forced UDP and TCP every x"}


def channel(hours):
    return dict(REF_4H, SCHEDULE=",".join([GE_SCHEDULE] * (hours // 4)), **MIX)


def simulate(chan, cfg, seeds):
    """Per seed the alarm class's (n, lost), and the run summary (mix_study.simulate, per seed)."""
    with tempfile.NamedTemporaryFile(suffix=".txt", delete=False) as f:
        path = f.name
    env = dict(os.environ, **{k: str(v) for k, v in dict(chan, **cfg, SEEDS=seeds, MIXOUT=path).items()})
    out = subprocess.run([SIM], env=env, check=True, capture_output=True, text=True).stdout
    mix = np.loadtxt(path, ndmin=2)
    os.unlink(path)
    return mix[:, 3 * ALARM:3 * ALARM + 2], summary([parse(x) for x in out.splitlines()], float(chan["L"]))


def main():
    jobs = [(h, label, channel(h), VARIANTS[v]) for h in HOURS for label, v in SHOWN.items()]
    with ThreadPoolExecutor(int(os.environ.get("OPT_WORKERS", 10))) as ex:
        res = list(ex.map(lambda j: simulate(j[2], j[3], RUNS), jobs))
    by_h = {}
    for (h, label, _, _), (alarm, s) in zip(jobs, res):
        n, lost = alarm[:, 0], alarm[:, 1]
        per_run = lost / np.maximum(n, 1) / ALARM_L
        by_h.setdefault(h, []).append(f"{label:14s} {lost.sum() / n.sum() / ALARM_L:.3f}L "
                                      f"(se {per_run.std(ddof=1) / np.sqrt(len(per_run)):.3f}) "
                                      f"saving {100 * s['saving']:.1f}%")
    for h in HOURS:
        print(f"{h:3d} h   " + "   ".join(by_h[h]))


if __name__ == "__main__":
    main()
