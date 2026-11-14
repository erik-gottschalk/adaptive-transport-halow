"""The paper's figures from the measurement data, at IEEE column (3.5 in) or page (7.16 in) width,
every text at least 8 pt.

    python export_ieee_figures.py [--out DIR] [--only steady,traces,step,mixed,bursty]

  halow_ieee_steady.pdf             Fig. 3: (a) loss rate against injected loss for each L, its target
                                    dashed; (b) energy per message sent, UDP and TCP and the controller
  halow_ieee_step_timeseries.pdf    Fig. 4: the step schedule, five captures
  halow_ieee_mixed.pdf              Fig. 5: mixed workload, loss / L per message class
  halow_ieee_packet_traces.pdf      Fig. 6: one send of UDP and of fresh-connection TCP at p = 0
  halow_ieee_bursty_timeseries.pdf  Fig. 7: bursty, changing loss

Default output: eval/plots/. Fig. 1 and 2 are TikZ drawings in the paper's sources.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.text  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
DATA = EVAL / "measurement_data"
sys.path.insert(0, str(HERE))
import summarize_pwr_runs as spr  # noqa: E402

MIN_PT = 8
COL_W, PAGE_W = 3.5, 7.16

C_UDP, C_TCP = "#1baf7a", "#2a78d6"
SERIES = [(0.025, "#8b3aa7", "L=2.5%"), (0.050, "#d1495b", "L=5%"), (0.100, "#b8860b", "L=10%")]


def base_style():
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "STIXGeneral"],
        "mathtext.fontset": "stix", "font.size": MIN_PT, "axes.labelsize": MIN_PT,
        "xtick.labelsize": MIN_PT, "ytick.labelsize": MIN_PT, "legend.fontsize": MIN_PT,
        "axes.edgecolor": "#999999", "axes.linewidth": 0.6, "axes.grid": True,
        "grid.color": "#dddddd", "grid.linewidth": 0.4, "axes.spines.top": False,
        "axes.spines.right": False, "pdf.fonttype": 42,
    })


def floor_fonts(fig):
    """Raise every text in @p fig to at least MIN_PT."""
    for t in fig.findobj(matplotlib.text.Text):
        if t.get_fontsize() < MIN_PT:
            t.set_fontsize(MIN_PT)


def plot_series(ax, df, color, label, mean_col, lo_col=None, hi_col=None, marker="o", **kw):
    """Mean with the min-max range over the captures of a cell."""
    d = df.sort_values("p%").dropna(subset=[mean_col]) if mean_col in df else df.iloc[0:0]
    if d.empty:
        return
    x, y = d["p%"].to_numpy(), d[mean_col].to_numpy()
    if lo_col and lo_col in d and d[lo_col].notna().any():
        lo, hi = d[lo_col].to_numpy(), d[hi_col].to_numpy()
        yerr = np.where(np.isfinite(np.vstack([y - lo, hi - y])), np.vstack([y - lo, hi - y]), 0)
        ax.errorbar(x, y, yerr=yerr, color=color, marker=marker, lw=1.1, ms=3.5, capsize=2,
                    label=label, **kw)
    else:
        ax.plot(x, y, color=color, marker=marker, lw=1.1, ms=3.5, label=label, **kw)


def cell_of(path):
    m = json.loads(path.read_text())
    return round(float(m["L"]), 4), round(float(m["schedule"][0][1]), 4)


def steady(out: Path):
    """The single-protocol baselines and the controller: grid/debt, its cell L = 5 %, p = 20 % from
    grid/binding_cell, L = 0 from grid/fixed_probe; p <= 20 %."""
    grid_v3, grid_v5 = DATA / "grid" / "fixed_probe", DATA / "grid" / "debt"
    base_dirs = [DATA / "baselines"]
    v5 = [p for p in sorted(grid_v5.glob("pwr_adp_*.json")) if cell_of(p)[1] <= 20]
    rerun = sorted((DATA / "grid" / "binding_cell").glob("pwr_adp_*.json"))
    v5 = [p for p in v5 if cell_of(p) not in {cell_of(q) for q in rerun}] + rerun
    v3 = [p for p in sorted(grid_v3.glob("pwr_adp_*.json"))
          if cell_of(p)[1] <= 20 and cell_of(p) not in {cell_of(q) for q in v5}]
    main = sorted(m for d in base_dirs for m in d.glob("pwr_*.json")) + v5 + v3

    cache = spr.load_cache()
    rows = [spr.process_one(p, cache) for p in main]
    spr.save_cache(cache)
    for r in rows:
        if "error" in r:
            print(f"  SKIPPED {r['basename']}: {r['error']}")
    ct = spr.cell_aggregate(rows)
    for col in ("delivered_loss_mean", "delivered_loss_min", "delivered_loss_max"):
        ct[col + "_pct"] = ct[col] * 100 if col in ct else float("nan")

    base_style()
    fig, (axL, axE) = plt.subplots(2, 1, figsize=(COL_W, 3.8), sharex=True)
    for L, color, label in SERIES:
        sub = ct[(ct.proto == "adp") & np.isclose(ct.L, L)]
        plot_series(axL, sub, color, label, "delivered_loss_mean_pct",
                    "delivered_loss_min_pct", "delivered_loss_max_pct")
        axL.axhline(L * 100, color=color, ls="--", lw=0.8)
    axL.set_ylabel("Loss rate [%]")
    axL.legend(frameon=False, ncol=2, loc="upper left")
    axL.text(-0.17, 1.0, "(a)", transform=axL.transAxes, fontweight="bold", va="top")

    plot_series(axE, ct[ct.proto == "udp"], C_UDP, "UDP", "mJ/pkt_attempted_mean")
    plot_series(axE, ct[ct.proto == "tcp"], C_TCP, "TCP", "mJ/pkt_attempted_mean")
    for L, color, label in SERIES:
        plot_series(axE, ct[(ct.proto == "adp") & np.isclose(ct.L, L)], color, label,
                    "mJ/pkt_attempted_mean", "mJ/pkt_attempted_min", "mJ/pkt_attempted_max")
    axE.set_xlabel("Injected AP packet loss $p$ [%]")
    axE.set_ylabel("Energy per message [mJ]")
    axE.legend(frameon=False, ncol=2, loc="upper left")
    axE.text(-0.17, 1.0, "(b)", transform=axE.transAxes, fontweight="bold", va="top")
    floor_fonts(fig)
    fig.tight_layout(h_pad=0.6)
    fig.savefig(out / "halow_ieee_steady.pdf", bbox_inches="tight")
    print("wrote halow_ieee_steady.pdf")


def step(out: Path):
    """The five captures of step/: the controller's estimate, and the 60 s
    rolling UDP fraction and loss / L."""
    import analyze_dynamic_cd as a
    roll_s, c_hw, muted = 60, "#d1495b", "#666666"
    runs = [a.load_run(p) for p in sorted((DATA / "step").glob("pwr_*.json"))]
    phases, L = runs[0]["phases"], runs[0]["L"]
    end = phases[-1][1]
    grid = np.arange(0, end + 1, 5.0)
    edges = np.array([p[0] for p in phases] + [end]) / 60
    ps = [p[2] for p in phases]
    shade = {0.02: "white", 0.05: "#f0f0f0", 0.20: "#dcdcdc"}
    est = np.array([a.step_hold(r["t_est"], r["est"], grid) for r in runs])
    udp = np.array([a.rolling_fraction(r["packets"][:, 0], r["packets"][:, 1], grid, roll_s)
                    for r in runs])
    loss = np.array([a.rolling_fraction(r["packets"][:, 0], 1 - r["packets"][:, 2], grid, roll_s) / L
                     for r in runs])
    base_style()
    fig, axes = plt.subplots(4, 1, figsize=(COL_W, 3.5), sharex=True,
                             gridspec_kw={"height_ratios": [0.55, 1, 1, 1]})
    ax_p, ax_est, ax_udp, ax_loss = axes
    x = grid / 60
    for ax in axes:
        for k, p in enumerate(ps):
            ax.axvspan(edges[k], edges[k + 1], color=shade[round(p, 2)], lw=0, zorder=0)
        ax.grid(axis="x", visible=False)
    ax_p.stairs([100 * p for p in ps], edges, color="black", lw=1.0)
    ax_p.set_ylabel("$p$ [%]")
    ax_p.set_ylim(0, 25)
    ax_est.stairs([1 - p for p in ps], edges, color=muted, lw=1.0, ls=(0, (2, 2)),
                  label="$1 - p$")
    ax_udp.stairs([min(1, L / p) for p in ps], edges, color=muted, lw=1.0, ls=(0, (2, 2)),
                  label=r"$f_\mathrm{opt}$")
    ax_loss.axhline(1, color=muted, lw=1.0, ls=(0, (2, 2)), label="cap $L$")
    for ax, v in ((ax_est, est), (ax_udp, udp), (ax_loss, loss)):
        for row in v:
            ax.plot(x, row, color=c_hw, lw=0.5, alpha=0.3, zorder=2)
        ax.plot(x, np.nanmean(v, axis=0), color=c_hw, lw=1.3, zorder=3,
                label="mean of five" if ax is ax_est else None)
    ax_est.set_ylabel("$1 - q_U$")
    ax_est.set_ylim(0.6, 1.02)
    ax_udp.set_ylabel("UDP frac.")
    ax_udp.set_ylim(0, 1.05)
    ax_loss.set_ylabel("loss$/L$")
    ax_loss.set_ylim(0, 4)
    ax_loss.set_xlabel("time [min]")
    ax_loss.set_xlim(0, edges[-1])
    handles, labels = [], []
    for ax in (ax_est, ax_udp, ax_loss):
        h, lab = ax.get_legend_handles_labels()
        handles, labels = handles + h, labels + lab
    fig.legend(handles, labels, loc="upper center", ncol=4, frameon=False,
               bbox_to_anchor=(0.55, 1.02), handlelength=1.6, columnspacing=1.0)
    floor_fonts(fig)
    fig.align_ylabels(axes)
    fig.tight_layout(rect=(0, 0, 1, 0.95), h_pad=0.2)
    fig.savefig(out / "halow_ieee_step_timeseries.pdf", bbox_inches="tight")
    print("wrote halow_ieee_step_timeseries.pdf")


def finish(fig, path: Path):
    """Fonts at least MIN_PT, y-labels aligned, tight layout, saved with a tight bounding box."""
    floor_fonts(fig)
    fig.align_ylabels(fig.axes)
    fig.tight_layout(h_pad=0.2)
    fig.savefig(path, bbox_inches="tight")
    print(f"wrote {path.name}")


# ---------------------------------------------------------------------------------------------
# Fig. 6: one send of each baseline protocol at p = 0
# ---------------------------------------------------------------------------------------------

TRACE_DIR = DATA / "baselines"                      # r01 at p = 0
T_MIN, T_MAX = -15, 500                              # window around the send [ms]
EXAMPLE = 20                                         # the EXAMPLE-th send of the capture


def example_segment(csv_path, protocol, v_supply, dt_ms):
    """The plotted window around the EXAMPLE-th send of ``protocol``: time [ms], current [mA] and
    the send call's duration [ms]."""
    import power_common as pc
    current_ua, d0, d1 = pc.load_current_d0(csv_path)
    cycles = pc.detect_cycles(current_ua, d0, d1, dt_ms=dt_ms, v_supply=v_supply)
    match = cycles[cycles.protocol_gpio == protocol]
    ev = match.iloc[min(EXAMPLE, len(match) - 1)]
    r = int(ev["rise"])
    lo, hi = r + int(T_MIN / dt_ms), r + int(T_MAX / dt_ms)
    t = (np.arange(lo, hi) - r) * dt_ms
    seg_ma = current_ua[lo:hi].astype(np.float64) / 1000
    return {"t_ms": t.tolist(), "current_mA": seg_ma.tolist(), "width_ms": float(ev["width_ms"])}


def traces(out: Path):
    import derived
    base_style()
    plt.rcParams.update({"xtick.labelsize": 7, "ytick.labelsize": 7})   # floored to MIN_PT on saving
    fig, axes = plt.subplots(1, 2, figsize=(COL_W, 1.4), sharex=True, sharey=True)
    peaks = []
    for ax, proto, color, title in ((axes[0], "udp", C_UDP, "UDP"),
                                    (axes[1], "tcp", C_TCP, "TCP (fresh connection)")):
        csv_path = TRACE_DIR / f"pwr_{proto}_L025_p00_r01.csv"
        m = json.loads(csv_path.with_suffix(".json").read_text())
        # without the trace: the stored window (derived/example_traces.json, see derived.py)
        seg = derived.get("example_traces", f"{derived.key_for(csv_path)} {proto.upper()} #{EXAMPLE}",
                          lambda: example_segment(csv_path, proto.upper(), m["tap_voltage_mV"] / 1000,
                                                  m["dt_ms"]), [csv_path])
        t, seg_ma = np.array(seg["t_ms"]), np.array(seg["current_mA"])
        ax.axvspan(0, seg["width_ms"], color="#e6e6e6", lw=0)
        ax.plot(t, seg_ma, color=color, lw=0.7)
        ax.annotate(title, (0.985, 0.92), xycoords="axes fraction", ha="right", va="top",
                    color=color, fontsize=MIN_PT, fontweight="bold")
        peaks.append(seg_ma.max())
    for ax in axes:
        ax.set_ylim(0, max(peaks) * 1.15)
        ax.set_xlabel("Time since send call [ms]")
    axes[0].set_ylabel("Current [mA]")
    axes[0].set_xlim(T_MIN, T_MAX)
    fig.tight_layout(w_pad=0.6)
    finish(fig, out / "halow_ieee_packet_traces.pdf")


# ---------------------------------------------------------------------------------------------
# Fig. 5: mixed workload, loss / L per message class
# ---------------------------------------------------------------------------------------------

def mixed(out: Path):
    import pandas as pd
    import analyze_mixed_criticality as mc
    root = DATA / "mixed"
    frames = [f for f in (mc.analyze_run(p) for p in sorted(root.glob("pwr_adp_*.json"))) if f is not None]
    t = mc.per_class(pd.concat(frames, ignore_index=True))
    pooled = t[t.run == "POOLED"].set_index("class")
    c_hw, muted = "#d1495b", "#666666"
    base_style()
    fig, ax = plt.subplots(figsize=(COL_W, 1.8))
    for c in range(3):
        r = pooled.loc[c]
        y, v = 2 - c, r["u=loss/L"]
        lo, hi = r["loss_ci95_lo"] / r["L"], r["loss_ci95_hi"] / r["L"]
        ax.errorbar(v, y, xerr=[[v - lo], [hi - v]], ls="none", marker="o", ms=4, capsize=2,
                    color=c_hw, mew=0.9, lw=1.0)
    ax.axvline(1, color=muted, lw=0.7, ls=":")
    ax.set_yticks([2, 1, 0])
    ax.set_yticklabels([f"{n}\n$L={pooled.loc[c, 'L']:g}$, UDP {pooled.loc[c, 'f_udp']:.2f}"
                        for c, n in enumerate(["telemetry", "sensor", "alarm"])])
    ax.set_ylim(-0.5, 2.5)
    ax.set_xlim(0, 2.2)
    ax.set_xlabel("Loss rate / $L$")
    ax.grid(axis="y", visible=False)
    finish(fig, out / "halow_ieee_mixed.pdf")
    print(pooled[["L", "n", "f_udp", "u=loss/L"]])


# ---------------------------------------------------------------------------------------------
# Fig. 7: bursty, changing loss
# ---------------------------------------------------------------------------------------------

def bursty(out: Path):
    """The injected loss, and cumulative over the schedule the delivered loss against the
    attainable loss L* and the saving against plain TCP against the clairvoyant bound's, both on
    the controller's own messages (the quantities of Table II, whose final values they reach)."""
    import analyze_ge_schedule as g
    c_hw, c_bound, muted = "#d1495b", "#00798c", "#666666"
    mans, runs = g.load("adp")
    phases, L = runs[0]["phases"], runs[0]["L"]
    end = phases[-1][1]
    grid = np.arange(300.0, end + 1, 10.0)    # from 5 min on: over fewer messages a cumulative ratio is noise
    edges = np.array([p[0] for p in phases] + [end]) / 60
    ps = [p[2] for p in phases]
    loss, lstar, save, clair = [], [], [], []
    for r, man in zip(runs, mans):
        pk = g.capture(r)
        t = pk[:, 0]
        p = np.array([g.p_at(r, x) for x in t])
        k = np.searchsorted(t, grid, side="right")            # messages sent up to each grid time
        n = np.maximum(k, 1)
        lost = np.concatenate([[0], np.cumsum(1 - pk[:, 2])])[k]
        att = np.concatenate([[0], np.cumsum(np.minimum(p, L))])[k]
        ref = np.concatenate([[0], np.cumsum([g.e_tcp(x) for x in p])])[k]
        f = np.minimum(1.0, L / np.maximum(p, 1e-9))
        bound = np.concatenate([[0], np.cumsum([fi * g.e_udp(x) + (1 - fi) * g.e_tcp(x)
                                                for fi, x in zip(f, p)])])[k]
        t_cyc, e_cyc = g.cycle_energy(man)
        e_adp = np.concatenate([[0], np.cumsum(e_cyc)])[np.searchsorted(t_cyc, grid, side="right")]
        loss.append(lost / n / L)
        lstar.append(att / n / L)
        save.append(100 * (1 - e_adp / ref))
        clair.append(100 * (1 - bound / ref))
    loss, lstar, save, clair = map(np.array, (loss, lstar, save, clair))
    print(f"  figure end values: loss/L {loss[:, -1].mean():.3f}  L*/L {lstar[:, -1].mean():.3f}"
          f"  loss/L* {np.mean(loss[:, -1] / lstar[:, -1]):.3f}  saving {save[:, -1].mean():.1f}%"
          f"  clairvoyant {clair[:, -1].mean():.1f}%  of bound {np.mean(save[:, -1] / clair[:, -1]):.2f}")

    base_style()
    plt.rcParams.update({"xtick.labelsize": 7, "ytick.labelsize": 7})   # floored to MIN_PT on saving
    fig, axes = plt.subplots(3, 1, figsize=(PAGE_W, 2.7), sharex=True,
                             gridspec_kw={"height_ratios": [0.5, 1.1, 1.1]})
    ax_p, ax_loss, ax_save = axes
    x = grid / 60
    for ax in axes:
        for j, p in enumerate(ps):
            ax.axvspan(edges[j], edges[j + 1], color=str(1 - 0.9 * p / 0.2 * 0.25), lw=0, zorder=0)
        ax.grid(axis="x", visible=False)
    ax_p.stairs([100 * p for p in ps], edges, color="black", lw=1.0)
    ax_p.set_ylabel("$p$ [%]")
    ax_p.set_ylim(0, 22)
    ax_loss.axhline(1, color=muted, lw=1.0, ls=(0, (2, 2)), label="cap $L$")
    for ax, v, ref_v, ref_lab in ((ax_loss, loss, lstar, "attainable $L^*$"),
                                  (ax_save, save, clair, "clairvoyant bound")):
        for row in v:
            ax.plot(x, row, color=c_hw, lw=0.5, alpha=0.3, zorder=2)
        ax.plot(x, v.mean(axis=0), color=c_hw, lw=1.2, zorder=3,
                label=f"adaptive (mean of {len(runs)} captures; thin: single)")
        ax.plot(x, ref_v.mean(axis=0), color=c_bound, lw=1.2, zorder=3, label=ref_lab)
    ax_loss.set_ylabel("loss$/L$")
    ax_loss.set_ylim(0, 1.3)
    ax_save.set_ylabel("saving [%]")
    ax_save.set_ylim(0, 60)
    ax_save.set_xlabel("time [min]")
    ax_save.set_xlim(0, edges[-1])
    for ax in (ax_loss, ax_save):
        ax.legend(loc="lower right", ncol=3, frameon=False, fontsize=MIN_PT)
    fig.tight_layout(h_pad=0.3)
    finish(fig, out / "halow_ieee_bursty_timeseries.pdf")


FIGURES = {"steady": steady, "traces": traces, "step": step, "mixed": mixed, "bursty": bursty}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=EVAL / "plots")
    ap.add_argument("--only", default=",".join(FIGURES))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for name in args.only.split(","):
        FIGURES[name](args.out)


if __name__ == "__main__":
    main()
