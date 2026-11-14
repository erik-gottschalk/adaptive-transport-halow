"""Run sim_opt configurations in parallel with an on-disk cache, and summarise them.

    from opt_run import run, summary
    rows = run([dict(EST="base", MODE=1), dict(EST="hmm", NU=1/600)], chan=REF, seeds=200)

A configuration is a dict of sim_opt environment variables; `chan` (a dict too) is merged under it.
Raw per-seed lines are cached in opt_out/<hash>.txt, so re-running a sweep only simulates what is
new. Summaries pair every variant with the same seeds (common random numbers in sim_opt).
"""
import hashlib
import json
import math
import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "opt_out")
SIM = os.path.join(HERE, "sim_opt")

# Hardware model (energy, send time, late replies per loss; see sim_opt.c HW_TABLE and hw_table.py):
# hw_table.txt next to this file if it exists, unless HW_TABLE is already set ("builtin" = the
# earlier built-in constants). Set in os.environ so every sim_opt started from these scripts --
# including paper_plots.py's direct calls -- uses the same table, and its content hash goes into
# the cache key below, so a rebuilt table never reuses results simulated with an older one.
if os.environ.get("HW_TABLE") == "builtin":
    del os.environ["HW_TABLE"]
elif "HW_TABLE" not in os.environ and os.path.exists(os.path.join(HERE, "hw_table.txt")):
    os.environ["HW_TABLE"] = os.path.join(HERE, "hw_table.txt")
HW_TABLE_SHA = None
if os.environ.get("HW_TABLE"):
    with open(os.environ["HW_TABLE"], "rb") as _f:
        HW_TABLE_SHA = hashlib.sha1(_f.read()).hexdigest()[:12]

# Reference realistic channel: Markov loss levels with fades and bursts.
REF = dict(CHAN="markov", DUR=4 * 3600, DWELL=900, QMIN=0.005, QMAX=0.40, FADE_EVERY=1800,
           FADE_MIN=20, FADE_MAX=120, FADE_QMIN=0.4, FADE_QMAX=0.7, BURST=3, L=0.05)
HW = "300:0.02,300:0.20,300:0.02,240:0.05,60:0.40,300:0.05,300:0.40,300:0.02"
SCHED = dict(CHAN="sched", SCHEDULE=HW, L=0.05)
# Physical (link-margin) channel, reference parameters at sim_opt.c gen_phys(); M0 sets the link.
PHYS = dict(CHAN="phys", DUR=4 * 3600, M0=14, L=0.05)
CALM = dict(PHYS, TAU_S=1800, SIG_S=4, INT_EVERY=3600, FADE_EVERY=3600)
VOLATILE = dict(PHYS, TAU_S=180, SIG_S=6, INT_EVERY=600, FADE_EVERY=900)
JUMP = REF


# Runs per scenario for every simulation the paper reports (env PAPER_RUNS overrides), so that a
# configuration that appears in two tables reads the same in both. The pre-registered horizon sweep
# (k_sweep_round2.py) keeps its own counts.
PAPER_RUNS = int(os.environ.get("PAPER_RUNS", 2000))


def _key(env):
    if HW_TABLE_SHA:        # without a table the key is unchanged, so earlier cached runs stay valid
        env = dict(env, HW_TABLE_SHA=HW_TABLE_SHA)
    # The controller's probe default changed to the random probe (FW_RANDOM_PROBE=1): runs cached
    # before then simulated the forced cycle, and keep their key only with FW_RANDOM_PROBE=0.
    if "FW_RANDOM_PROBE" not in env:
        env = dict(env, PROBE="random")
    elif not int(env["FW_RANDOM_PROBE"]):
        env = {k: v for k, v in env.items() if k != "FW_RANDOM_PROBE"}
    return hashlib.sha1(json.dumps(env, sort_keys=True).encode()).hexdigest()[:16]


N_TOTALS = 14      # fields after the last "|" of a run line since the attainable loss (2026-10-21)


def _stale(path):
    """A cached result from before the simulator printed every total: re-run it. The simulator is
    deterministic, so the re-run reproduces the earlier columns and only adds the new ones."""
    with open(path) as f:
        f.readline()
        first = f.readline()
    return bool(first) and len(first.rsplit("|", 1)[-1].split()) < N_TOTALS


def _one(env):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, _key(env) + ".txt")
    if not os.path.exists(path) or _stale(path):
        e = dict(os.environ)
        e.update({k: str(v) for k, v in env.items()})
        res = subprocess.run([SIM], env=e, capture_output=True, text=True, check=True)
        with open(path + ".tmp", "w") as f:
            f.write(json.dumps(env, sort_keys=True) + "\n" + res.stdout)
        os.replace(path + ".tmp", path)
    with open(path) as f:
        lines = f.read().splitlines()[1:]
    return [parse(x) for x in lines]


def parse(line):
    a, b, c = line.split("|")
    a, b, c = a.split(), b.split(), c.split()
    nph = len(a) // 4
    ph = [tuple(int(v) for v in a[4 * i:4 * i + 4]) for i in range(nph)]
    st = [(int(b[4 * i]), int(b[4 * i + 1]), int(b[4 * i + 2]), float(b[4 * i + 3]))
          for i in range(nph)]
    tot = dict(energy=float(c[0]), excess=float(c[1]), nwin=int(c[2]), nviol=int(c[3]),
               fopt=float(c[4]), feedbacks=int(c[5]), deficit=float(c[6]), e_tcp=float(c[7]),
               # window spread, only in runs cached since 2026-10-05
               nunder=int(c[8]) if len(c) > 9 else None, sq=float(c[9]) if len(c) > 9 else None,
               # clairvoyant bound (true q, optimal mix, no probe or feedback), since 2026-10-21: on
               # the variant's own messages, and on the channel's nominal slots with pure TCP there
               e_clair_msg=float(c[10]) if len(c) > 12 else None,
               e_clair=float(c[11]) if len(c) > 12 else None,
               e_clair_tcp=float(c[12]) if len(c) > 12 else None,
               # and its expected losses, sum of min(q, L) on the variant's own messages, since
               # 2026-10-21 (configurations tagged L_CLAIR=1, so they do not reuse earlier runs)
               l_clair_msg=float(c[13]) if len(c) > 13 else None)
    return dict(ph=ph, st=st, **tot)


def probe_mode(env):
    """FW_RANDOM_PROBE for a configuration that does not set it. The random probe (the firmware
    default) has no feedback slot: feedback comes from the TCP sends that ageing produces. A
    variant without ageing -- the time counter with FW_TA off, or a hooked estimator without
    AGE_T -- would then never ask for feedback, so it keeps the forced cycle it was designed with
    (and its runs cached from before the random probe). The oracle plans with the true loss rate
    and needs no feedback; it runs the firmware's random probe."""
    if "FW_RANDOM_PROBE" in env:
        return int(env["FW_RANDOM_PROBE"])
    est = env.get("EST", "base")
    aged = (est == "fw" and float(env.get("FW_TA", 20)) < 1e6) or float(env.get("AGE_T", 0)) > 0
    return int(aged or est == "oracle")


def run(configs, chan=None, seeds=200, workers=int(os.environ.get("OPT_WORKERS", 30))):
    envs = []
    for cfg in configs:
        env = dict(chan or {})
        env.update(cfg)
        env["SEEDS"] = seeds
        env["FW_RANDOM_PROBE"] = probe_mode(env)
        envs.append(env)
    with ThreadPoolExecutor(workers) as ex:
        return list(ex.map(_one, envs))


def summary(runs, L, exwin=300):
    """Totals over seeds: energy/msg, UDP/f_opt, loss/L, excess per hour, 5-min violations."""
    n = sum(sum(p[0] for p in r["ph"]) for r in runs)
    udp = sum(sum(p[1] for p in r["ph"]) for r in runs)
    lost = sum(sum(p[2] for p in r["ph"]) for r in runs)
    # In message slots (= s at 1 msg/s); NaN if a broken variant reports no windows at all.
    hours = sum(r["nwin"] for r in runs) * exwin / 3600 or float("nan")
    return dict(
        e=sum(r["energy"] for r in runs) / n,
        udp_fopt=udp / sum(r["fopt"] for r in runs),
        loss_L=lost / n / L,
        ex_h=sum(r["excess"] for r in runs) / hours,
        viol=sum(r["nviol"] for r in runs) / max(1, sum(r["nwin"] for r in runs)),
        fb_h=sum(r["feedbacks"] for r in runs) / hours,
        alarms_h=sum(sum(p[3] for p in r["ph"]) for r in runs) / hours,
        def_h=sum(r["deficit"] for r in runs) / hours,
        msgs_h=n / hours,
        saving=1 - sum(r["energy"] for r in runs) / sum(r["e_tcp"] for r in runs),
        # the bound for this variant's own messages, and per nominal slot (variant-independent)
        saving_clair=(1 - sum(r["e_clair_msg"] for r in runs) / sum(r["e_tcp"] for r in runs)
                      if all(r.get("e_clair_msg") is not None for r in runs) else None),
        saving_clair_slot=(1 - sum(r["e_clair"] for r in runs) / sum(r["e_clair_tcp"] for r in runs)
                           if all(r.get("e_clair") is not None for r in runs) else None),
        # the attainable loss L* (the clairvoyant bound's loss on this variant's messages) over L
        attain_L=(sum(r["l_clair_msg"] for r in runs) / n / L
                  if all(r.get("l_clair_msg") is not None for r in runs) else None),
    )


def paired(runs, ref, key):
    """Mean and standard error of the per-seed difference runs - ref of a per-run quantity."""
    d = [key(a) - key(b) for a, b in zip(runs, ref)]
    m = sum(d) / len(d)
    sd = math.sqrt(sum((x - m) ** 2 for x in d) / max(1, len(d) - 1))
    return m, sd / math.sqrt(len(d))


def fmt(label, s):
    return (f"{label:34s} E={s['e']:6.2f} mJ  udp/fopt={s['udp_fopt']:.3f}  loss/L={s['loss_L']:.3f}"
            f"  excess/h={s['ex_h']:6.2f}  deficit/h={s['def_h']:6.1f}  viol={s['viol']:.3f}  fb/h={s['fb_h']:5.1f}"
            f"  alarms/h={s['alarms_h']:.2f}")
