"""Score sweep results: J(mu) = energy per hour + mu * excess lost messages per hour (mJ/h).

    python3 opt_analyze.py <experiment> [mu ...]

mu prices one message lost above the budget (per 5-min window) in mJ. Repaying it later by sending
1/q messages over TCP instead of UDP costs G(q)/q ~ 130 mJ on these channels (mu = 130 is the
risk-neutral Lagrangian price); larger mu encodes "losing is worse than wasting energy".
For every family (and window k), alpha is chosen to minimise J; reports J above the oracle's best.
"""
import json
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))


def load(exp):
    with open(os.path.join(HERE, "opt_out", exp + ".json")) as f:
        return json.load(f)


def J(s, mu):
    return s["e"] * s["msgs_h"] + mu * s["ex_h"]


def best(rows, mu):
    """family -> (J, row) minimising J over alpha."""
    out = {}
    for r in rows:
        j = J(r["summary"], mu)
        if r["family"] not in out or j < out[r["family"]][0]:
            out[r["family"]] = (j, r)
    return out


if __name__ == "__main__":
    exp = sys.argv[1]
    mus = [float(m) for m in sys.argv[2:]] or [130, 500, 2000]
    rows = load(exp)
    for mu in mus:
        b = best(rows, mu)
        j0 = min(j for f, (j, r) in b.items() if f.startswith("oracle"))
        print(f"\n== {exp}  mu = {mu:.0f} mJ per excess message   (J - J_oracle, mJ/h; energy saving vs pure TCP)")
        groups = defaultdict(list)
        for f, (j, r) in b.items():
            groups[f.split(" | ")[0]].append((j, f, r))
        for g, items in sorted(groups.items(), key=lambda kv: min(x[0] for x in kv[1])):
            items.sort()
            j, f, r = items[0]
            s = r["summary"]
            def kof(lab):
                return lab.split(" | k=")[-1] if " | k=" in lab else "-"
            ks = "  ".join(f"{kof(x[1])}:{x[0] - j0:7.0f}" for x in
                           sorted(items, key=lambda x: -int(kof(x[1])) if kof(x[1]) != "-" else 0))
            print(f"{g:36s} best {f.split(' | ')[-1]:6s} a={r['cfg']['ALPHA']:.1f}  dJ={j - j0:7.0f}  "
                  f"excess/h={s['ex_h']:5.1f}  E={s['e']:.2f}  loss/L={s['loss_L']:.2f}   [{ks}]")


def robustness(channels, mus=(130, 500, 2000), ref="phys_ref"):
    """alpha fixed per family and mu on the reference channel, then applied to every channel."""
    data = {c: load("final_" + c) for c in channels}
    for mu in mus:
        alpha = {f: r["cfg"]["ALPHA"] for f, (j, r) in best(data[ref], mu).items()}
        fams = list(alpha)
        print(f"\n== robustness, mu = {mu}: J - J_oracle(best alpha) in mJ/h; alpha fixed on {ref}: "
              + ", ".join(f"{f.split(' (')[0][:18]}={a}" for f, a in alpha.items()))
        print(f"{'channel':16s}" + "".join(f"{f[:22]:>24s}" for f in fams))
        for c in channels:
            rows = data[c]
            j0 = min(J(r["summary"], mu) for r in rows if r["family"].startswith("oracle"))
            cells = []
            for f in fams:
                r = next(r for r in rows if r["family"] == f and r["cfg"]["ALPHA"] == alpha[f])
                cells.append(f"{J(r['summary'], mu) - j0:8.0f} ({r['summary']['ex_h']:5.1f}/h)")
            print(f"{c:16s}" + "".join(f"{x:>24s}" for x in cells))


def fopt(p, L):
    return min(1.0, L / p) if p > 0 else 1.0


def hw_table(exp, alpha=1.0):
    """The handoff's per-phase table for the hardware schedule (loss/L, UDP/f_opt per 5-min phase)."""
    from opt_run import run
    rows = [r for r in load(exp) if r["cfg"]["ALPHA"] == alpha]
    print(f"\n== {exp} (alpha = {alpha})")
    print(f"{'variant':30s} {'2->20 loss/L':>12s} {'react s':>8s} {'fade loss/L':>11s} {'after-fade udp':>14s} "
          f"{'5->40 loss/L':>12s} {'20->2 udp':>9s} {'40->2 udp':>9s} {'2% udp':>7s} {'5% udp':>7s} {'save':>6s}")
    for r in rows:
        cfg = dict(r["cfg"])
        runs = run([cfg], chan=r["chan"], seeds=500 if r["chan"]["CHAN"] == "sched" else 200)[0]
        L = float(r["chan"]["L"])
        ps = [float(x.split(":")[1]) for x in r["chan"]["SCHEDULE"].split(",")]

        def ph(i, what):
            n = sum(x["ph"][i][0] for x in runs)
            if what == "loss":
                return sum(x["ph"][i][2] for x in runs) / n / L
            return sum(x["ph"][i][1] for x in runs) / n / fopt(ps[i], L)

        react = sorted(x["st"][1][3] for x in runs)
        med = react[len(react) // 2]
        print(f"{r['family'][:30]:30s} {ph(1, 'loss'):12.2f} {med:8.0f} {ph(4, 'loss'):11.2f} {ph(5, 'udp'):14.2f} "
              f"{ph(6, 'loss'):12.2f} {ph(2, 'udp'):9.2f} {ph(7, 'udp'):9.2f} {ph(0, 'udp'):7.2f} {ph(3, 'udp'):7.2f} "
              f"{r['summary']['saving']:6.1%}")
