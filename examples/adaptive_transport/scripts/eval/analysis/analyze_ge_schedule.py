"""Bursty, changing loss on hardware (Section IV-F, Table II): the captures of bursty/
-- the ge_regime_s1 schedule (make_ge_schedule.py, seed 1), Gilbert-Elliott loss at the AP with runs
of 3 packets (run_power_experiment.py --burst 3) -- for UDP and TCP in baselines/, for the adaptive
controller in controller/. Its figure (Fig. 7) is export_ieee_figures.py's.

    python3 analyze_ge_schedule.py            the numbers of Table II and Section IV-F
    python3 analyze_ge_schedule.py binding    the grid's binding cell under bursts (bursty/binding_cell)

Per capture: messages, delivered loss, energy per message sent (summarize_pwr_runs.process_one).
Adaptive captures also: loss / L and / L* (L* = mean of min(p, L) over the capture's own messages),
the share of 5-minute windows above 1.5 L, the saving against plain TCP (measured, the TCP captures
of the same schedule; and modelled, hw_table's TCP energy at each message's loss, as
analyze_step_schedule.py does), the clairvoyant bound's saving on the same messages (UDP with
probability min(1, L/p) at hw_table's energies), the reaction to the upward jumps (seconds until
p_udp_hat is halfway, analyze_dynamic_cd.react_time), and which transport the lost messages took.
"""
import datetime as dt
import json
import math
import re
import statistics as st
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
sys.path.insert(0, str(HERE))
import analyze_dynamic_cd as a  # noqa: E402
import summarize_pwr_runs as spr  # noqa: E402

BURSTY = EVAL / "measurement_data" / "bursty"
WIN_S = 300                 # windows for the share above 1.5 L
JUMP_TO = 0.11              # upward jumps to at least this level count for the reaction time

# hw_table.txt, mJ per attempted message (fresh connection), by injected loss
P_TAB = [0.0, 0.05, 0.10, 0.20]
E_UDP = [14.066, 14.018, 14.035, 14.044]
E_TCP = [24.448, 28.486, 32.435, 42.617]


def e_udp(p):
    return float(np.interp(p, P_TAB, E_UDP))


def e_tcp(p):
    return float(np.interp(p, P_TAB, E_TCP))


def mean_ci(xs):
    m = st.mean(xs)
    if len(xs) < 2:
        return m, math.nan
    return m, st.stdev(xs) / math.sqrt(len(xs)) * {2: 12.706, 3: 4.303, 4: 3.182}.get(len(xs), 2.776)


def p_at(run, t):
    for t0, t1, p in run["phases"]:
        if t0 <= t < t1:
            return p
    return run["phases"][-1][2]


def capture(run):
    pk = run["packets"]
    return pk[(pk[:, 0] >= 0) & (pk[:, 0] < run["phases"][-1][1])]


def energy(man, cache):
    return spr.process_one(man, cache)["out"]["energy"]["mJ_per_packet_attempted"]


CONNECT_FAIL_RE = re.compile(r"^(?P<ts>\S+) TCP connect to \S+ failed \(errno \d+, seq=(?P<seq>\d+)\)")


def with_failed_sends(run, man):
    """analyze_dynamic_cd.load_run counts the messages the console logs as sent; a TCP message whose
    connection failed is logged as a failed connect instead and would drop out of both the count and
    the losses. Under independent loss TCP never failed; under bursts it does, so add those messages
    here as sent over TCP and lost."""
    m = json.loads(man.read_text())
    start = dt.datetime.fromisoformat(m["capture_start_utc"])
    tz = dt.timezone(dt.timedelta(hours=a.TZ_OFFSETS_H[m["tz"]]))
    lines = (man.parent / m["console_name"]).read_text(encoding="utf-8", errors="replace").splitlines()
    inits = [i for i, ln in enumerate(lines) if "ATC: init" in ln]
    lines = lines[inits[-1]:] if inits else lines
    extra = []
    for ln in lines:
        mm = CONNECT_FAIL_RE.match(ln)
        if mm:
            t = (dt.datetime.fromisoformat(mm["ts"]).replace(tzinfo=tz).astimezone(dt.timezone.utc) - start).total_seconds()
            extra.append((t, 0.0, 0.0))
    if extra:
        pk = np.vstack([run["packets"], np.array(extra)])
        run["packets"] = pk[np.argsort(pk[:, 0], kind="stable")]
    run["failed_connects"] = len(extra)
    return run


def load(proto, folder=None):
    """The captures of one protocol: the controller's in controller/, UDP and TCP in baselines/."""
    d = BURSTY / (folder or ("controller" if proto == "adp" else "baselines"))
    mans = sorted(d.glob(f"pwr_{proto}_*.json"))
    return mans, [with_failed_sends(a.load_run(m), m) for m in mans]


def main():
    cache = spr.load_cache()
    res = {}
    for proto in ("udp", "tcp", "adp"):
        mans, runs = load(proto)
        rows = []
        for man, r in zip(mans, runs):
            pk = capture(r)
            n = len(pk)
            lost = int(n - pk[:, 2].sum())
            row = {"name": r["name"], "n": n, "lost": lost, "loss": lost / n, "E": energy(man, cache),
                   "failed": r["failed_connects"]}
            if proto == "adp":
                L = r["L"]
                ps = np.array([p_at(r, t) for t in pk[:, 0]])
                row["L"] = L
                row["Lstar"] = float(np.minimum(ps, L).mean())
                # windows above 1.5 L
                w = (pk[:, 0] // WIN_S).astype(int)
                over = [((pk[w == k, 2] == 0).mean() > 1.5 * L) for k in np.unique(w)]
                row["over"] = float(np.mean(over))
                # saving: modelled TCP reference on the same messages, and the clairvoyant bound
                ref = sum(e_tcp(p) for p in ps)
                f = np.minimum(1.0, L / np.maximum(ps, 1e-9))
                clair = sum(fi * e_udp(p) + (1 - fi) * e_tcp(p) for fi, p in zip(f, ps))
                row["save_model"] = 1 - row["E"] * n / ref
                row["save_clair"] = 1 - clair / ref
                # lost messages by transport
                lost_tcp = int(((pk[:, 1] == 0) & (pk[:, 2] == 0)).sum())
                row["lost_tcp"] = lost_tcp
                row["udp_share"] = float(pk[:, 1].mean())
                # reaction to upward jumps
                row["react"] = [tr["react_s"] for tr in a.transitions(r, 120)
                                if tr["kind"] == "degrade" and float(tr["label"].split("->")[1].strip(" %")) / 100 >= JUMP_TO]
            rows.append(row)
        res[proto] = (runs, rows)
    spr.save_cache(cache)

    for proto, (_, rows) in res.items():
        print(f"\n== {proto}: {len(rows)} captures")
        for x in rows:
            extra = ""
            if proto == "adp":
                extra = (f"  L*/L {x['Lstar'] / x['L']:.3f}  loss/L {x['loss'] / x['L']:.3f}  loss/L* {x['loss'] / x['Lstar']:.3f}"
                         f"  >1.5L {100 * x['over']:.1f}%  save(model) {100 * x['save_model']:.1f}%"
                         f"  clair {100 * x['save_clair']:.1f}%  of bound {x['save_model'] / x['save_clair']:.2f}"
                         f"  UDP {100 * x['udp_share']:.1f}%  lost TCP {x['lost_tcp']}/{x['lost']}"
                         f"  react {[None if math.isnan(v) else round(v) for v in x['react']]}")
            print(f"  {x['name']}: {x['n']} msgs, lost {x['lost']} ({100 * x['loss']:.2f}%; failed connects "
                  f"{x['failed']}), E {x['E']:.2f} mJ/msg{extra}")
        if rows:
            lo, lc = mean_ci([x["loss"] for x in rows])
            eo, ec = mean_ci([x["E"] for x in rows])
            print(f"  mean: loss {100 * lo:.2f}% +-{100 * lc:.2f}  E {eo:.2f} +-{ec:.2f} mJ/msg")

    adp = res["adp"][1]
    if adp and res["tcp"][1]:
        e_tcp_meas = st.mean(x["E"] for x in res["tcp"][1])
        e_adp = st.mean(x["E"] for x in adp)
        print("\n== adaptive vs measured plain TCP on the same schedule")
        per = ", ".join("%.1f%%" % (100 * (1 - x["E"] / e_tcp_meas)) for x in adp)
        print(f"  saving per message sent {100 * (1 - e_adp / e_tcp_meas):.1f}% (per capture {per})")
        for key, lab in (("Lstar", "L*/L"),):
            print(f"  {lab}: {st.mean(x['Lstar'] / x['L'] for x in adp):.3f}")
        print(f"  loss/L* {st.mean(x['loss'] / x['Lstar'] for x in adp):.3f}  loss/L {st.mean(x['loss'] / x['L'] for x in adp):.3f}"
              f"  >1.5L {100 * st.mean(x['over'] for x in adp):.1f}%"
              f"  of bound {st.mean(x['save_model'] / x['save_clair'] for x in adp):.2f}"
              f"  clairvoyant saving {100 * st.mean(x['save_clair'] for x in adp):.1f}%")
        reacts = [v for x in adp for v in x["react"] if not math.isnan(v)]
        print(f"  reaction to jumps to >= {100 * JUMP_TO:.0f}%: median {st.median(reacts):.0f} s over {len(reacts)} "
              f"jumps, range {min(reacts):.0f}-{max(reacts):.0f} s")
        lt = sum(x["lost_tcp"] for x in adp)
        ll = sum(x["lost"] for x in adp)
        print(f"  lost messages that went as TCP: {lt}/{ll} ({100 * lt / ll:.1f}%)")


def cycle_energy(man):
    """Per-cycle energy of a capture, as (send time [s from capture start], mJ): the cycles of
    power_common.detect_cycles, which summarize_pwr_runs.process_one also integrates."""
    import power_common as pc
    m = json.loads(man.read_text())
    cyc, _ = pc.load_cycles(man.with_suffix(".csv"), dt_ms=m.get("dt_ms", pc.DT_MS))
    return cyc["rise"].to_numpy() * m.get("dt_ms", pc.DT_MS) / 1000, cyc["cycle_mJ"].to_numpy()


def binding():
    """The grid's binding cell under bursts (bursty/binding_cell: L = 5 %, p = 20 %, runs of 3, 3 x 1 h)."""
    cache = spr.load_cache()
    mans, runs = load("adp", "binding_cell")
    rows = []
    for man, r in zip(mans, runs):
        pk = capture(r)
        n = len(pk)
        lost = int(n - pk[:, 2].sum())
        lost_tcp = int(((pk[:, 1] == 0) & (pk[:, 2] == 0)).sum())
        rows.append((lost / n / r["L"], lost_tcp, lost, energy(man, cache), float(pk[:, 1].mean())))
        print(f"  {r['name']}: {n} msgs, loss/L {lost / n / r['L']:.3f}, lost TCP {lost_tcp}/{lost} "
              f"(failed connects {r['failed_connects']}), UDP {100 * pk[:, 1].mean():.1f}%, E {rows[-1][3]:.2f} mJ/msg")
    spr.save_cache(cache)
    lo, lc = mean_ci([x[0] for x in rows])
    print(f"  mean loss/L {lo:.3f} +-{lc:.3f}  range {min(x[0] for x in rows):.3f}-{max(x[0] for x in rows):.3f}  "
          f"lost TCP {sum(x[1] for x in rows)}/{sum(x[2] for x in rows)}  E {st.mean(x[3] for x in rows):.2f} mJ/msg")


if __name__ == "__main__":
    binding() if sys.argv[1:] == ["binding"] else main()
