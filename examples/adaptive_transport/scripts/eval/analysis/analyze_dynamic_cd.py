#!/usr/bin/env python3
"""Sudden-channel-change analysis, change detection on vs. off; its functions (load_run,
window_stats, transitions) also serve analyze_step_schedule.py, which is how the paper uses it:
how quickly and how well the adaptive controller follows abrupt AP-loss steps in each arm.

Reads every capture under <root>/{cd_on,cd_off}/ (all replicates), splits each at the schedule's
phase boundaries (manifest["schedule"], capture-relative -- see run_power_experiment.capture_power)
and computes, per transition:

  react_s     seconds from the loss step until the controller's estimate p_udp_hat has moved at
              least halfway from the old to the new true UDP delivery rate (1 - p). NaN if it
              never does within the phase (censored -- shown as ">phase").
  loss/L      delivered loss in the first WINDOW seconds after the step, over L. Degradations: how
              much the transient overshoots the budget before the controller catches up.
  udp/f_opt   UDP fraction in that window over f_opt = min(1, L/p) of the new phase. Recoveries:
              how much of the newly available UDP the controller leaves unused while catching up.
  resets      change-detection resets in that window.

plus whole-phase delivered loss, UDP fraction and measured UDP loss for context.

Ground truth comes from the logs, not the PPK2 capture: every packet the device sent (console.log,
by sequence number and send time) is matched against what the server received (server.log). TCP
counts as delivered if the server got it at any point (late arrivals included).

Usage:
  python analyze_dynamic_cd.py                      # measurement_data/pwr_runs_dynamic_cd
  python analyze_dynamic_cd.py --root <dir>         # any dir with cd_on/ and cd_off/ subdirs
  python analyze_dynamic_cd.py --window-s 60
Writes plots/dynamic_cd_comparison.pdf (+ .png) and prints the tables; --show to display instead.
"""
import argparse
import datetime as dt
import json
import math
import re
import statistics as st
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
FIGDIR = EVAL / "plots"
DEFAULT_ROOT = EVAL / "measurement_data" / "pwr_runs_dynamic_cd"

ARMS = ("cd_on", "cd_off")
ARM_LABEL = {"cd_on": "change detection on", "cd_off": "change detection off"}
# Categorical slots 1 and 2 of the dataviz reference palette (blue, orange), plus a line-style
# difference so the arms never depend on colour alone.
ARM_COLOR = {"cd_on": "#2a78d6", "cd_off": "#eb6834"}
ARM_STYLE = {"cd_on": "-", "cd_off": "--"}
COLOR_TRUTH = "#898781"
COLOR_INK = "#0b0b0b"
COLOR_SHADE = "#e1e0d9"

# Same fixed setup-local abbreviations as plot_change_point.py / plot_dynamic_schedule.py.
TZ_OFFSETS_H = {"CEST": 2, "CET": 1, "UTC": 0}

DECISION_RE = re.compile(r"^(?P<ts>\S+) ATC: -> (?P<proto>UDP|TCP)\b")
EST_RE = re.compile(r"^(?P<ts>\S+) ATC: -> \w+ \(.*p_udp_hat=(?P<p_udp>[\d.]+) p_tcp_hat=")
RESET_RE = re.compile(r"^(?P<ts>\S+) ATC: change-detected: ")
UDP_SENT_RE = re.compile(r"^(?P<ts>\S+) UDP demo packet .*seq=(?P<seq>\d+)\)")
TCP_SENT_RE = re.compile(r"^(?P<ts>\S+) TCP demo (?:packet sent|reply from).*seq=(?P<seq>\d+)\)")

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "STIXGeneral"],
    "mathtext.fontset": "stix",
    "font.size": 8, "axes.labelsize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "axes.edgecolor": "#999999", "axes.linewidth": 0.6,
    "axes.grid": True, "grid.color": "#dddddd", "grid.linewidth": 0.4,
    "axes.spines.top": False, "axes.spines.right": False,
    "pdf.fonttype": 42,
})


def load_run(manifest_path: Path):
    manifest = json.loads(manifest_path.read_text())
    run_dir = manifest_path.parent
    capture_start = dt.datetime.fromisoformat(manifest["capture_start_utc"])
    tz_offset = dt.timezone(dt.timedelta(hours=TZ_OFFSETS_H[manifest["tz"]]))

    def to_t(ts: str) -> float:
        local = dt.datetime.fromisoformat(ts).replace(tzinfo=tz_offset)
        return (local.astimezone(dt.timezone.utc) - capture_start).total_seconds()

    lines = (run_dir / manifest["console_name"]).read_text(encoding="utf-8",
                                                          errors="replace").splitlines()
    inits = [i for i, line in enumerate(lines) if "ATC: init" in line]
    lines = lines[inits[-1]:] if inits else lines          # drop previous-boot leftovers

    sent = {}            # seq -> (t, proto); first log line per seq wins
    t_dec, dec_udp, t_est, est, resets = [], [], [], [], []
    for line in lines:
        m = DECISION_RE.match(line)
        if m:
            t_dec.append(to_t(m["ts"]))
            dec_udp.append(m["proto"] == "UDP")
        m = EST_RE.match(line)
        if m:
            t_est.append(to_t(m["ts"]))
            est.append(float(m["p_udp"]))
        if RESET_RE.match(line):
            resets.append(to_t(line.split(" ", 1)[0]))
        for regex, proto in ((UDP_SENT_RE, "UDP"), (TCP_SENT_RE, "TCP")):
            m = regex.match(line)
            if m and int(m["seq"]) not in sent:
                sent[int(m["seq"])] = (to_t(m["ts"]), proto)

    events = [json.loads(s) for s in (run_dir / f"{manifest['basename']}.server.log")
              .read_text(errors="replace").splitlines() if s.startswith("{")]
    confirmed = [i for i, e in enumerate(events) if e.get("event_type") == "config_confirmed"]
    events = events[confirmed[-1]:] if confirmed else events
    received = {(e["data"]["seq"], "UDP" if e["event_type"] == "udp_received" else "TCP")
                for e in events if e.get("event_type") in ("udp_received", "tcp_received")}

    packets = np.array([(t, proto == "UDP", (seq, proto) in received)
                        for seq, (t, proto) in sorted(sent.items())], dtype=float)

    L = (manifest.get("remote_config_confirmed") or {}).get("ATC_L")
    phases, t = [], 0.0
    for secs, pct in manifest["schedule"]:
        phases.append((t, t + secs, pct / 100.0))
        t += secs
    return {
        "name": manifest["basename"], "L": float(L if L is not None else manifest["L"]),
        "phases": phases, "packets": packets,           # columns: t, is_udp, delivered
        "t_dec": np.array(t_dec), "dec_udp": np.array(dec_udp, dtype=float),
        "t_est": np.array(t_est), "est": np.array(est), "resets": np.array(resets),
    }


def window_stats(run, t0, t1, p_new):
    pk = run["packets"]
    sel = pk[(pk[:, 0] >= t0) & (pk[:, 0] < t1)]
    n = len(sel)
    if n == 0:
        return {}
    udp = sel[sel[:, 1] == 1]
    lost = int(n - sel[:, 2].sum())
    f_opt = min(1.0, run["L"] / p_new) if p_new > 0 else 1.0
    return {
        "n": n,
        "f_udp": len(udp) / n,
        "udp_vs_opt": (len(udp) / n) / f_opt,
        "loss_vs_L": (lost / n) / run["L"],
        "udp_loss_meas": (1 - udp[:, 2].mean()) if len(udp) >= 5 else math.nan,
        "resets": int(((run["resets"] >= t0) & (run["resets"] < t1)).sum()),
    }


def react_time(run, t_step, t_end, p_old, p_new):
    """Seconds until p_udp_hat has moved at least halfway from 1-p_old towards 1-p_new."""
    target = 1 - (p_old + p_new) / 2
    sel = (run["t_est"] >= t_step) & (run["t_est"] < t_end)
    ts, est = run["t_est"][sel], run["est"][sel]
    hit = est <= target if p_new > p_old else est >= target
    return float(ts[hit][0] - t_step) if hit.any() else math.nan


def transitions(run, window_s):
    out = []
    phases = run["phases"]
    for (_, t_step, p_old), (_, t_end, p_new) in zip(phases, phases[1:]):
        w = window_stats(run, t_step, min(t_step + window_s, t_end), p_new)
        out.append({
            "label": f"{p_old:.0%} -> {p_new:.0%}", "t": t_step,
            "kind": "degrade" if p_new > p_old else "recover",
            "react_s": react_time(run, t_step, t_end, p_old, p_new),
            "phase_s": t_end - t_step, **w,
        })
    return out


def fmt(vals, spec=".2f", censored_as=None):
    def one(v):
        if isinstance(v, float) and math.isnan(v):
            return censored_as or "nan"
        return format(v, spec)
    finite = [v for v in vals if not (isinstance(v, float) and math.isnan(v))]
    mean = format(st.mean(finite), spec) if finite and len(finite) == len(vals) else "--"
    return f"{mean:>6s} ({' '.join(one(v) for v in vals)})"


def print_tables(runs, window_s):
    print(f"\nTransitions (window = first {window_s:.0f} s after each step; "
          "react = time to halfway in p_udp_hat)\n")
    hdr = (f"{'step':14s} {'arm':7s} | {'react_s':>22s} | {'loss/L in window':>24s} | "
           f"{'udp/f_opt in window':>24s} | resets")
    print(hdr)
    print("-" * len(hdr))
    per_arm = {arm: [transitions(r, window_s) for r in runs[arm]] for arm in ARMS if runs[arm]}
    n_tr = len(next(iter(per_arm.values()))[0])
    for i in range(n_tr):
        for arm, trs in per_arm.items():
            rows = [tr[i] for tr in trs]
            phase_s = rows[0]["phase_s"]
            print(f"{rows[0]['label']:14s} {arm:7s} | "
                  f"{fmt([r['react_s'] for r in rows], '.0f', f'>{phase_s:.0f}'):>22s} | "
                  f"{fmt([r.get('loss_vs_L', math.nan) for r in rows]):>24s} | "
                  f"{fmt([r.get('udp_vs_opt', math.nan) for r in rows]):>24s} | "
                  f"{sum(r.get('resets', 0) for r in rows)}")
        print()

    print("Whole phases\n")
    hdr = (f"{'phase':16s} {'arm':7s} | {'UDP loss meas':>20s} | {'udp/f_opt':>24s} | "
           f"{'loss/L':>24s} | resets")
    print(hdr)
    print("-" * len(hdr))
    ref = next(r for arm in ARMS for r in runs[arm])
    for k, (t0, t1, p) in enumerate(ref["phases"]):
        for arm in ARMS:
            if not runs[arm]:
                continue
            rows = [window_stats(r, t0, t1, p) for r in runs[arm]]
            print(f"{k + 1}: {p:4.0%} {t1 - t0:4.0f}s   {arm:7s} | "
                  f"{fmt([r.get('udp_loss_meas', math.nan) for r in rows]):>20s} | "
                  f"{fmt([r.get('udp_vs_opt', math.nan) for r in rows]):>24s} | "
                  f"{fmt([r.get('loss_vs_L', math.nan) for r in rows]):>24s} | "
                  f"{sum(r.get('resets', 0) for r in rows)}")
        print()


def step_hold(t_src, v_src, grid):
    """Value of a step-held series (last value at or before each grid time; NaN before the first)."""
    idx = np.searchsorted(t_src, grid, side="right") - 1
    out = np.where(idx >= 0, v_src[np.clip(idx, 0, None)], np.nan)
    return out


def rolling_fraction(t_src, v_src, grid, window_s):
    out = np.full(len(grid), np.nan)
    for i, g in enumerate(grid):
        sel = (t_src > g - window_s) & (t_src <= g)
        if sel.sum() >= 5:
            out[i] = v_src[sel].mean()
    return out


def plot(runs, out_path, rolling_s, show):
    ref = next(r for arm in ARMS for r in runs[arm])
    end = ref["phases"][-1][1]
    grid = np.arange(0.0, end, 5.0)
    gm = grid / 60.0

    fig, axes = plt.subplots(3, 1, figsize=(7.0, 5.6), sharex=True,
                             gridspec_kw={"height_ratios": [0.7, 1.4, 1.4]})
    ax_loss, ax_est, ax_udp = axes

    truth_t = [p[0] / 60 for p in ref["phases"]] + [end / 60]
    p_steps = [p[2] for p in ref["phases"]]
    for ax in axes:                                   # alternate phase shading, recessive
        for k, (t0, t1, _) in enumerate(ref["phases"]):
            if k % 2:
                ax.axvspan(t0 / 60, t1 / 60, color=COLOR_SHADE, alpha=0.5, lw=0, zorder=0)

    ax_loss.stairs([p * 100 for p in p_steps], truth_t, color=COLOR_INK, lw=1.2)
    ax_loss.set_ylabel("AP loss [%]")
    for (t0, t1, p) in ref["phases"]:
        ax_loss.text((t0 + t1) / 120, p * 100, f"{p:.0%}", ha="center", va="bottom",
                     fontsize=6.5, color=COLOR_INK)
    ax_loss.set_ylim(0, max(p_steps) * 100 * 1.3)

    ax_est.stairs([1 - p for p in p_steps], truth_t, color=COLOR_TRUTH, lw=1.0, ls=(0, (2, 2)),
                  label="true UDP delivery rate")
    f_opt = [min(1.0, ref["L"] / p) if p > 0 else 1.0 for p in p_steps]
    ax_udp.stairs(f_opt, truth_t, color=COLOR_TRUTH, lw=1.0, ls=(0, (2, 2)),
                  label=r"$f_\mathrm{opt} = \min(1, L/p)$")

    for arm in ARMS:
        if not runs[arm]:
            continue
        est = np.array([step_hold(r["t_est"], r["est"], grid) for r in runs[arm]])
        udp = np.array([rolling_fraction(r["t_dec"], r["dec_udp"], grid, rolling_s)
                        for r in runs[arm]])
        for series, ax in ((est, ax_est), (udp, ax_udp)):
            for row in series:                        # individual replicates, faint
                ax.plot(gm, row, color=ARM_COLOR[arm], lw=0.5, alpha=0.3, ls=ARM_STYLE[arm])
            ax.plot(gm, np.nanmean(series, axis=0), color=ARM_COLOR[arm], lw=2.0,
                    ls=ARM_STYLE[arm], label=f"{ARM_LABEL[arm]} (mean of {len(series)})")
        for r in runs[arm]:
            for t in r["resets"]:
                ax_est.plot(t / 60, 1.02, marker="v", ms=4, color=ARM_COLOR[arm], clip_on=False)

    ax_est.set_ylabel(r"$\hat{p}_\mathrm{UDP}$ (controller estimate)")
    ax_est.set_ylim(0.4, 1.04)
    ax_est.legend(loc="lower left", frameon=False, ncol=1)
    ax_udp.set_ylabel(f"UDP fraction ({rolling_s:.0f} s rolling)")
    ax_udp.set_ylim(0, 1.05)
    ax_udp.set_xlabel("time since capture start [min]")
    ax_udp.legend(loc="lower left", frameon=False, ncol=1)
    ax_udp.set_xlim(0, end / 60)
    fig.text(0.99, 0.005, "triangles: change-detection resets", ha="right", fontsize=6,
             color=COLOR_TRUTH)
    fig.tight_layout()

    if show:
        plt.show()
    else:
        fig.savefig(out_path)
        fig.savefig(out_path.with_suffix(".png"), dpi=200, facecolor="white")
        print(f"\nwrote {out_path} (+ .png)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                    help="directory with cd_on/ and cd_off/ subdirs (default: %(default)s)")
    ap.add_argument("--window-s", type=float, default=120.0,
                    help="post-step window for the transition metrics (default 120)")
    ap.add_argument("--rolling-s", type=float, default=60.0,
                    help="rolling window for the UDP-fraction panel (default 60)")
    ap.add_argument("-o", "--out", type=Path, default=FIGDIR / "dynamic_cd_comparison.pdf")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    runs = {arm: [load_run(p) for p in sorted((args.root / arm).glob("pwr_*.json"))]
            for arm in ARMS}
    if not any(runs.values()):
        raise SystemExit(f"no captures under {args.root}/{{cd_on,cd_off}}")
    for arm in ARMS:
        print(f"{arm}: {', '.join(r['name'] for r in runs[arm]) or 'none'}")
    schedules = {tuple(p for p in r["phases"]) for arm in ARMS for r in runs[arm]}
    if len(schedules) > 1:
        raise SystemExit("captures use different schedules -- analyse them separately")

    print_tables(runs, args.window_s)
    plot(runs, args.out, args.rolling_s, args.show)


if __name__ == "__main__":
    main()
