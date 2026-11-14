"""In-text hardware numbers of the IEEE version that no table or figure script prints, each next to
the value the paper states.

    python3 paper_numbers.py

  - G/C for the least favourable combination of single baseline captures (Section IV-E, Section S3)
  - the smallest saving of any adaptive grid capture with L > 0 against the cheapest TCP baseline
    capture at its loss rate (Section IV-E)
  - TCP messages lost in all grid captures up to 20 % (Section IV-F)
  - the largest feedback window, in UDP messages per request (Section S2)
  - the module's awake period after a send at p = 0 (Section IV-E, Fig. 6)
  - the number of captures behind the paper's hardware results (Section IV-B: "188 captures")

The grid selection is export_ieee_figures.steady()'s: grid/debt with the L = 5 %, p = 20 % cell
from grid/binding_cell, L = 0 from grid/fixed_probe, and the single-protocol baselines.
"""
from __future__ import annotations

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
DATA = EVAL / "measurement_data"
sys.path.insert(0, str(HERE))
import derived  # noqa: E402
import power_common as pc  # noqa: E402
import summarize_pwr_runs as spr  # noqa: E402

P_MAX = 20


def cell_of(path):
    m = json.loads(path.read_text())
    return round(float(m["L"]), 4), round(float(m["schedule"][0][1]), 4)


def grid_manifests():
    """(baselines, adaptive) as export_ieee_figures.steady() selects them."""
    grid_v3, grid_v5 = DATA / "grid" / "fixed_probe", DATA / "grid" / "debt"
    base = sorted((DATA / "baselines").glob("pwr_*.json"))
    v5 = [p for p in sorted(grid_v5.glob("pwr_adp_*.json")) if cell_of(p)[1] <= P_MAX]
    rerun = sorted((DATA / "grid" / "binding_cell").glob("pwr_adp_*.json"))
    v5 = [p for p in v5 if cell_of(p) not in {cell_of(q) for q in rerun}] + rerun
    v3 = [p for p in sorted(grid_v3.glob("pwr_adp_*.json"))
          if cell_of(p)[1] <= P_MAX and cell_of(p) not in {cell_of(q) for q in v5}]
    return base, v5 + v3


def energy(row):
    return row["out"]["energy"]["mJ_per_packet_attempted"]


def usable(row):
    """As spr.cell_aggregate's power columns: no FAIL that invalidates the energy."""
    return "error" not in row and not spr.excluded(row, spr.POWER_INVALIDATES)


def worst_case_gc(base_rows):
    """Per p: G = min E_TCP^0 - max E_UDP, C = max E_TCP^r - min E_TCP^0 over single captures."""
    by = defaultdict(lambda: defaultdict(list))
    for r in base_rows:
        if usable(r):
            by[r["manifest"]["schedule"][0][1]][r["manifest"]["protocol"]].append(energy(r))
    print("G/C, least favourable single captures (paper: >= 2.18):")
    worst = float("inf")
    for p in sorted(by):
        e = by[p]
        if not all(k in e for k in ("udp", "tcp", "tcp_reply")):
            continue
        G = min(e["tcp"]) - max(e["udp"])
        C = max(e["tcp_reply"]) - min(e["tcp"])
        worst = min(worst, G / C)
        print(f"  p = {p:2d} %: G = {G:6.2f}  C = {C:5.2f}  G/C = {G / C:.2f}   "
              f"(captures: udp {len(e['udp'])}, tcp {len(e['tcp'])}, tcp_reply {len(e['tcp_reply'])})")
    print(f"  minimum: {worst:.2f}")


def min_saving(base_rows, adp_rows):
    tcp_min = defaultdict(lambda: float("inf"))
    for r in base_rows:
        if usable(r) and r["manifest"]["protocol"] == "tcp":
            p = r["manifest"]["schedule"][0][1]
            tcp_min[p] = min(tcp_min[p], energy(r))
    savings = []
    for r in adp_rows:
        if not usable(r) or float(r["manifest"]["L"]) <= 0:
            continue
        p = r["manifest"]["schedule"][0][1]
        savings.append((1 - energy(r) / tcp_min[p], r["basename"], r["manifest"]["L"], p))
    s, name, L, p = min(savings)
    print(f"\nAdaptive grid captures with L > 0: {len(savings)} (paper: 31); smallest saving against the "
          f"cheapest TCP baseline capture at the same p: {100 * s:.1f} % ({name}, L = {L}, p = {p} %) "
          f"(paper: 2.2 %, L = 2.5 %, p = 20 %)")


def tcp_losses(rows):
    lost, sent = 0, 0
    for r in rows:
        lo = (r.get("out") or {}).get("loss") or {}
        if "n_tcp_sent" in lo and usable(r):
            lost += lo["n_tcp_sent"] - lo["n_tcp_received"]
            sent += lo["n_tcp_sent"]
    print(f"\nTCP messages lost in the grid captures up to {P_MAX} %: {lost} of {sent} (paper: three)")


def max_window(manifests):
    """The largest expected_udp a request carried (server log), over the given captures."""
    best = (0, "")
    for man in manifests:
        log = man.with_name(json.loads(man.read_text())["basename"] + ".server.log")
        if not log.exists():
            continue
        for line in log.read_text(errors="replace").splitlines():
            if '"tcp_received"' not in line:
                continue
            n = (json.loads(line).get("data") or {}).get("expected_udp") or 0
            best = max(best, (n, man.name))
    print(f"\nLargest feedback window: {best[0]} UDP messages ({best[1]}) (paper: no window above 156)")


PROFILE_S = 1.0          # the send interval of the p = 0 baselines
AWAKE_THR_MA = 10.0      # about half the awake current: the mean profile falls through it at sleep


def mean_profile(csv, gpio):
    """The current after a send of ``gpio``, averaged over every such send of the capture that has
    PROFILE_S of trace after it, in 1 ms bins [mA], and the mean energy of those cycles [mJ]."""
    m = json.loads(csv.with_suffix(".json").read_text())
    dt_ms, v = m["dt_ms"], m["tap_voltage_mV"] / 1000
    cur, d0, d1 = pc.load_current_d0(csv)
    cyc = pc.detect_cycles(cur, d0, d1, dt_ms=dt_ms, v_supply=v)
    sel = cyc[cyc["protocol_gpio"] == gpio]
    n = int(round(PROFILE_S * 1000 / dt_ms))
    per_bin = int(round(1 / dt_ms))
    rises = [int(r) for r in sel["rise"] if r + n <= len(cur)]
    prof = np.mean([cur[r:r + n].astype(np.float64) for r in rises], axis=0) / 1000
    return dict(profile_mA=prof.reshape(-1, per_bin).mean(axis=1).tolist(), n_cycles=len(rises),
                cycle_mJ=float(sel["cycle_mJ"].mean()), v_supply=v)


def awake_numbers():
    print(f"\nAwake period after a send at p = 0 (baselines, r01): mean current profile "
          f"after the send, awake while it exceeds {AWAKE_THR_MA:.0f} mA")
    for proto, gpio, paper in (("udp", "UDP", "about 18 mA for ~210 ms, then about 0.5 mA; "
                                              "12.7 of the 14.0 mJ in the awake period"),
                               ("tcp", "TCP", "awake for ~380 ms")):
        csv = DATA / "baselines" / f"pwr_{proto}_L025_p00_r01.csv"
        a = derived.get("awake_profile", derived.key_for(csv) + f" {gpio}",
                        lambda: mean_profile(csv, gpio), [csv])
        prof = np.array(a["profile_mA"])
        above = np.flatnonzero(prof > AWAKE_THR_MA)
        end = int(above[-1]) + 1 if above.size else 0          # in 1 ms bins
        e_awake = prof[:end].sum() * 1e-3 * a["v_supply"]      # mA * s * V = mJ
        print(f"  {gpio:4s} {a['n_cycles']} sends: awake {end} ms at {prof[:end].mean():.1f} mA, then "
              f"{prof[end:].mean():.2f} mA mean ({np.median(prof[end:]):.2f} median); {e_awake:.1f} of "
              f"{a['cycle_mJ']:.1f} mJ per message in the awake period  (paper: {paper})")


def binding_tcp(cache):
    """Plain TCP in the bursty binding cell (bursty/binding_cell_tcp: L = 5 %, p = 20 %, runs
    of 3), and the controller's saving against it (analyze_ge_schedule.py binding prints the
    controller's side)."""
    import statistics as st
    import analyze_ge_schedule as g
    out = {}
    for proto, d in (("tcp", "binding_cell_tcp"), ("adp", "binding_cell")):
        mans = sorted((DATA / "bursty" / d).glob(f"pwr_{proto}_*.json"))
        rows = []
        for man in mans:
            import analyze_dynamic_cd as a
            r = g.with_failed_sends(a.load_run(man), man)
            pk = g.capture(r)
            n, lost = len(pk), int(len(pk) - pk[:, 2].sum())
            rows.append(dict(n=n, lost=lost, failed=r["failed_connects"], E=g.energy(man, cache)))
        out[proto] = rows
    tcp, adp = out["tcp"], out["adp"]
    if not tcp:
        return
    print("\nPlain TCP in the bursty binding cell (L = 5 %, p = 20 %, b = 3; paper: up to 2.1 % lost, "
          "0.4 % lost although the send completed):")
    for x in tcp:
        print(f"  {x['n']} msgs: lost {100 * x['lost'] / x['n']:.2f} %, of them after a completed send "
              f"{100 * (x['lost'] - x['failed']) / x['n']:.2f} % (failed connects {x['failed']}), "
              f"E {x['E']:.2f} mJ/msg")
    if adp:
        e_tcp, e_adp = st.mean(x["E"] for x in tcp), st.mean(x["E"] for x in adp)
        print(f"  controller saving per message sent against this TCP: {100 * (1 - e_adp / e_tcp):.1f} % "
              f"(paper: 9.4 %); against the independent-loss TCP baseline at 20 % "
              f"({g.e_tcp(0.20):.2f} mJ): {100 * (1 - e_adp / g.e_tcp(0.20)):.1f} %")


# The folders of the paper's hardware results (the published data).
PAPER_CAMPAIGNS = [
    "baselines", "grid/debt", "grid/no_debt", "grid/fixed_probe", "grid/binding_cell", "step", "mixed",
    "bursty/baselines", "bursty/controller", "bursty/binding_cell", "bursty/binding_cell_tcp",
    "udp_ack", "deep_sleep/power_save", "deep_sleep/wnm_sleep",
]


def capture_count():
    print("\nCaptures in the campaigns the paper's hardware results use (non-recursive per folder):")
    total = 0
    for c in PAPER_CAMPAIGNS:
        n = len([p for p in (DATA / c).glob("pwr_*.json")]) if (DATA / c).is_dir() else 0
        total += n
        print(f"  {c:40s} {n:3d}")
    print(f"  total {total} (paper, Section IV-B: 188 captures)")


def main():
    base, adp = grid_manifests()
    cache = spr.load_cache()
    base_rows = [spr.process_one(p, cache) for p in base]
    adp_rows = [spr.process_one(p, cache) for p in adp]
    spr.save_cache(cache)
    for r in base_rows + adp_rows:
        if "error" in r:
            print(f"  SKIPPED {r['basename']}: {r['error']}")
    worst_case_gc(base_rows)
    min_saving(base_rows, adp_rows)
    tcp_losses(base_rows + adp_rows)
    # the adaptive captures the paper shows: the grid as above, the step schedule, the mixed
    # workload, the bursty schedule and its binding cell (final firmware)
    shown = adp + [p for c in ("step", "mixed", "bursty/controller", "bursty/binding_cell")
                   for p in sorted((DATA / c).glob("pwr_adp_*.json"))]
    max_window(shown)
    binding_tcp(cache)
    spr.save_cache(cache)
    awake_numbers()
    capture_count()


if __name__ == "__main__":
    main()
