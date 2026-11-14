"""Channels of the paper: bursty, regime-switching loss. paper_sweep.py and mix_study.py use them,
and so design_table.py, attainable_table.py and the rest.

    OPT_WORKERS=16 ../run_paper.sh simulations      -> opt_out/rerun_window_ge/

REF is the bursty schedule the testbed replays (eval/ge_schedules/ge_regime_s1, runs of 3), so the
design choices and half-lives are scored on a channel the hardware validates. CHANNELS adds the
regime channel it was drawn from (make_ge_schedule.py: sim_opt's CHAN=markov capped at the testbed's
20 %) and varies what the testbed fixes. INTERVAL_REF is the uncapped regime channel, for the long
reporting intervals (MSG_DT applies to markov/phys channels only).
"""
import json
import os

from opt_run import SCHED

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "..", "ge_schedules", "ge_regime_s1.json")) as f:
    GE_SCHEDULE = json.load(f)["sim_schedule"]

# Runs last HOURS (24 h; the 4-hour results are kept in opt_out/rerun_window_ge_4h): long enough for
# a rare class to repay its debt -- the mixed workload's alarm class reads 1.04L at 4 h and 0.98L at
# 24 h -- and a duration no hardware capture covers. The 4-hour bursty schedule stays as the testbed
# ran it (REF_4H), for the comparison with the captures.
HOURS = 24
# The final firmware (03569b6c): failed TCP sends charged to the debt, on every channel (it acts only
# with a debt, so the no-debt variants are unaffected). Under bursts TCP connections fail to open; the
# probability C * p^K, C = 0.145, K = 1.5, fits the final firmware's captures with runs of 3 (~1.3 % of
# connects at a steady 20 %, measured 1.1-1.3 %; ~0.16 % over ge_regime_s1, measured 0.18 % for plain
# TCP). Independent loss: no failed connects (none in the final grid cell at 20 %). Earlier 24-hour
# runs without either are kept in opt_out/rerun_window_ge_24h_v1.
FINAL = dict(FW_DEBT_TCP=1)
FAIL = dict(TCP_FAIL_C=0.145, TCP_FAIL_K=1.5)
NO_FAIL = dict(TCP_FAIL_C=0)
REF_4H = dict(CHAN="sched", SCHEDULE=GE_SCHEDULE, BURST=3, L=0.05, **FINAL, **FAIL)
REF = dict(REF_4H, SCHEDULE=",".join([GE_SCHEDULE] * (HOURS // 4)))
# make_ge_schedule.py's draw, continuous levels instead of whole percent
REGIME = dict(CHAN="markov", DUR=HOURS * 3600, DWELL=900, QMIN=0.01, QMAX=0.20, FADE_EVERY=1800,
              FADE_MIN=20, FADE_MAX=120, FADE_QMIN=0.20, FADE_QMAX=0.20, BURST=3, L=0.05, **FINAL, **FAIL)
UNCAPPED = dict(REGIME, QMAX=0.40, FADE_QMIN=0.40, FADE_QMAX=0.70)

# "reference" and "hardware schedule, i.i.d." are the keys paper_sweep.py's half-life sweep reads;
# attainable_table.py replays the "hardware schedule" rows with the 40 % phases at 20 %.
CHANNELS = {
    "reference": REF,
    "bursty schedule, 4 h (testbed)": REF_4H,
    "bursty schedule, L = 2.5 %": dict(REF, L=0.025),
    "regime, runs of 3": REGIME,
    "regime, independent": dict(REGIME, BURST=0, **NO_FAIL),
    "regime, runs of 10": dict(REGIME, BURST=10),
    "regime, 5 min": dict(REGIME, DWELL=300),
    "regime, 60 min": dict(REGIME, DWELL=3600),
    "regime, up to 40 %": UNCAPPED,
    "regime, L = 2.5 %": dict(REGIME, L=0.025),
    "regime, L = 10 %": dict(REGIME, L=0.10),
    "hardware schedule, i.i.d.": dict(SCHED, **FINAL),
    "hardware schedule, bursts": dict(SCHED, BURST=3, **FINAL, **FAIL),
}
INTERVAL_REF = UNCAPPED


def longer(chan):
    """A fixed-loss mixed-workload scenario ('<s>:<p>') stretched to HOURS."""
    sched = chan.get("SCHEDULE", "")
    if chan.get("CHAN") != "sched" or "," in sched:
        return chan
    return dict(chan, SCHEDULE=f"{HOURS * 3600}:{sched.split(':')[1]}")
