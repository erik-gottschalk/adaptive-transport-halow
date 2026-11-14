"""
make_ge_schedule.py

Draws one regime-jump loss schedule for the bursty (Gilbert-Elliott, --burst) hardware captures:
the simulator's CHAN=markov channel (opt_run.py's JUMP) capped at the testbed's 20 %, drawn once
from a fixed seed so every capture -- UDP, TCP and adaptive, every replicate -- and the simulator
replay see the same loss levels at the same times. Only the per-packet drops differ between
captures; the burst length is the capture's --burst.

  - regimes: the loss level holds for an exponential time (mean --dwell, at least --min-dwell),
    then jumps to an independent level, log-uniform in [--qmin, --qmax] and rounded to whole
    percent (the nft/channel_scheduler unit);
  - bad periods: at Poisson times (mean gap --fade-every) the level goes to --qmax for
    U[--fade-min, --fade-max] s, on top of the regime.

Writes <out>.json (parameters, phases, and both schedule strings) and <out>.txt (the
run_power_experiment.py --schedule string). Use it as
    --schedule "$(cat ge_regime_s1.txt)" --burst 3
and in the simulator as SCHEDULE=<json's sim_schedule> BURST=3 CHAN=sched.
"""
import argparse
import json
import math
import random
from collections import Counter
from pathlib import Path


def draw(seed, total, dwell, min_dwell, qmin, qmax, fade_every, fade_min, fade_max):
    rng = random.Random(seed)
    level = [0] * total
    t = 0
    while t < total:
        d = max(min_dwell, round(rng.expovariate(1.0 / dwell)))
        q = round(math.exp(rng.uniform(math.log(qmin), math.log(qmax))))
        level[t:t + d] = [q] * len(level[t:t + d])
        t += d
    if fade_every > 0:
        t = rng.expovariate(1.0 / fade_every)
        while t < total:
            d = rng.randint(fade_min, fade_max)
            a = int(t)
            level[a:a + d] = [qmax] * len(level[a:a + d])
            t += d + rng.expovariate(1.0 / fade_every)
    phases = []
    for q in level:
        if phases and phases[-1][1] == q:
            phases[-1][0] += 1
        else:
            phases.append([1, q])
    return [tuple(p) for p in phases]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--total", type=int, default=4 * 3600, help="seconds (default 4 h)")
    ap.add_argument("--dwell", type=float, default=900, help="mean regime length, s")
    ap.add_argument("--min-dwell", type=int, default=60, help="shortest regime, s")
    ap.add_argument("--qmin", type=float, default=1, help="lowest regime level, %%")
    ap.add_argument("--qmax", type=int, default=20, help="highest level and bad-period level, %%")
    ap.add_argument("--fade-every", type=float, default=1800, help="mean gap between bad periods, "
                    "s (0 = none)")
    ap.add_argument("--fade-min", type=int, default=20)
    ap.add_argument("--fade-max", type=int, default=120)
    ap.add_argument("--L", type=float, default=0.05, help="only for the printed attainable loss")
    ap.add_argument("--out", default=None, help="output path without extension "
                    "(default eval/ge_schedules/ge_regime_s<seed>)")
    args = ap.parse_args()

    phases = draw(args.seed, args.total, args.dwell, args.min_dwell, args.qmin, args.qmax,
                  args.fade_every, args.fade_min, args.fade_max)
    hw = ",".join(f"{d}:{q}" for d, q in phases)
    sim = ",".join(f"{d}:{q / 100:.2f}" for d, q in phases)
    mean_q = sum(d * q for d, q in phases) / args.total / 100
    attain = sum(d * min(q / 100, args.L) for d, q in phases) / args.total
    by_level = Counter()
    for d, q in phases:
        by_level[q] += d

    out = Path(args.out) if args.out else \
        Path(__file__).parent / "eval" / "ge_schedules" / f"ge_regime_s{args.seed}"
    out.parent.mkdir(parents=True, exist_ok=True)
    meta = {k: v for k, v in vars(args).items() if k != "out"}
    out.with_suffix(".json").write_text(json.dumps({
        "params": meta, "phases": phases, "hw_schedule": hw, "sim_schedule": sim,
        "mean_loss": mean_q, "attainable_loss_at_L": attain,
    }, indent=1) + "\n", encoding="utf-8")
    out.with_suffix(".txt").write_text(hw + "\n", encoding="utf-8")

    print(f"{len(phases)} phases over {args.total} s -> {out}.{{json,txt}}")
    print(f"time-weighted mean loss {100 * mean_q:.1f}%, attainable loss at L={args.L:g}: "
          f"{100 * attain:.2f}% ({attain / args.L:.2f}L)")
    print("minutes per level:", {q: round(s / 60) for q, s in sorted(by_level.items())})
    print("first phases:", hw[:120], "...")


if __name__ == "__main__":
    main()
