"""The estimator variants, parameters and reporting intervals on the paper's channels
(ge_channels.py), all at alpha = 1 (the firmware as is); design_table.py and attainable_table.py
build their tables on these runs and channels.

    python3 paper_sweep.py            -> opt_out/paper_ge_*.json, then prints the tables

The loss cap is read as a long-run target: runs are scored by long-run delivered loss / L and energy
saving against pure TCP, and transients on the hardware schedule (per-phase loss / L, reaction time).
"""
import math

from opt_analyze import fopt
from opt_run import PAPER_RUNS, run, summary
from opt_sweep import sweep

from ge_channels import CHANNELS, INTERVAL_REF, REF  # the paper's channels
MAIN = {
    "earlier: sample-weighted + change detection": dict(EST="base", MODE=1),
    "earlier: sample-weighted, no change detection": dict(EST="base", MODE=0),
    "earlier + ageing": dict(EST="blend", AGE_T=20),
    "time counter, no ageing": dict(EST="fw", FW_TA=1e9),
    "time counter + ageing": dict(EST="fw"),
    "time counter + ageing + loss debt (firmware)": dict(EST="fw", FW_DEBT_R=250),
    "oracle": dict(EST="oracle"),
}
PARAMS = {f"T={te} Ta={'off' if ta > 1e6 else ta}": dict(EST="fw", FW_TE=te, FW_TA=ta, FW_DEBT_R=250)
          for te in (10, 20, 40) for ta in (10, 20, 40, 1e9)}
INTERVALS = (1, 10, 30, 60, 300)
# Fixed-loss cells where the hardware grid (grid/fixed_probe, the controller without the
# loss debt, 2 replicates) measured the steady-state overshoot: (L, p) -> measured loss / L.
BINDING = {(0.025, 0.05): (1.12, 1.15), (0.025, 0.10): (1.21, 1.40), (0.025, 0.20): (1.30, 1.29),
           (0.05, 0.10): (1.03, 1.06), (0.05, 0.20): (1.24, 1.26), (0.10, 0.20): (1.04, 1.12)}
FIXED = {"time counter + ageing": dict(EST="fw"),
         "time counter + ageing + loss debt (firmware)": dict(EST="fw", FW_DEBT_R=250),
         "oracle": dict(EST="oracle")}


def key(name):
    return "paper_ge_" + "".join(c if c.isalnum() else "_" for c in name)


def seeds(chan):
    return PAPER_RUNS


def main():
    res = {}
    for name, chan in CHANNELS.items():
        res[name] = sweep(key(name), chan, [MAIN], seeds(chan), alphas=[1.0])
    res_p = {n: sweep(key("params " + n), CHANNELS[n], [PARAMS], seeds(CHANNELS[n]), alphas=[1.0])
             for n in ("reference", "hardware schedule, i.i.d.")}
    res_dt = {d: sweep(key(f"interval {d}"), dict(INTERVAL_REF, MSG_DT=d, DUR=7200), [MAIN], PAPER_RUNS, alphas=[1.0])
              for d in INTERVALS}

    print("\n== fixed-loss cells: long-run loss / L, saving -- simulated vs. hardware (no debt)")
    for (L, p), hw in BINDING.items():
        rows = sweep(key(f"fixed {L} {p}"), dict(CHAN="sched", SCHEDULE=f"3600:{p}", L=L), [FIXED], PAPER_RUNS,
                     alphas=[1.0])
        by = {r["family"]: r["summary"] for r in rows}
        print(f"  L={100 * L:4.1f}% p={100 * p:4.0f}%  hardware {hw[0]:.2f},{hw[1]:.2f}  " + "  ".join(
            f"{f[:22]}: {by[f]['loss_L']:.2f} {100 * by[f]['saving']:.1f}%" for f in FIXED))

    print("\n== long-run loss / L and saving vs pure TCP (alpha = 1)")
    fams = list(MAIN)
    print(f"{'channel':28s}" + "".join(f"{f[:26]:>30s}" for f in fams)
          + "   clairvoyant: fw msgs  oracle msgs  per slot | fw msgs/h")
    fw, orc = fams[-2], fams[-1]
    for name, rows in res.items():
        by = {r["family"]: r["summary"] for r in rows}
        b = [by[fw]["saving_clair"], by[orc]["saving_clair"], by[fw]["saving_clair_slot"]]
        print(f"{name:28s}" + "".join(f"{by[f]['loss_L']:7.2f} {100 * by[f]['viol']:5.1f}% {100 * by[f]['saving']:7.1f}%      "
                                       for f in fams)
              + ("".join(f"{100 * x:12.1f}%" for x in b) if None not in b else "      --")
              + f" | {by[fw]['msgs_h']:6.0f}")
    print(f"  (per family: loss/L, share of 5-min windows above 1.5 L, saving; the clairvoyant bound on"
          f" the messages of '{fw}' and of '{orc}', and per nominal slot)")

    print("\n== message interval (reference channel): long-run loss / L, saving")
    for d, rows in res_dt.items():
        by = {r["family"]: r["summary"] for r in rows}
        print(f"{d:5d} s  " + "  ".join(f"{f[:24]}: {by[f]['loss_L']:.2f} {100 * by[f]['saving']:.1f}%" for f in fams))

    print("\n== parameters, reference channel: loss / L, saving, feedbacks per 1000 messages")
    for r in res_p["reference"]:
        s = r["summary"]
        print(f"  {r['family']:16s} loss/L {s['loss_L']:.2f}  saving {100 * s['saving']:.1f}%  fb/1000 {1000 * s['fb_h'] / s['msgs_h']:.0f}")
    print("\n== parameters, hardware schedule (i.i.d.): transients")
    hw_transients(res_p["hardware schedule, i.i.d."], CHANNELS["hardware schedule, i.i.d."])
    print("\n== main variants, hardware schedule (i.i.d.)")
    hw_transients(res["hardware schedule, i.i.d."], CHANNELS["hardware schedule, i.i.d."])
    print("\n== main variants, hardware schedule (bursts)")
    hw_transients(res["hardware schedule, bursts"], CHANNELS["hardware schedule, bursts"])


def hw_transients(rows, chan):
    L = float(chan["L"])
    ps = [float(x.split(":")[1]) for x in chan["SCHEDULE"].split(",")]
    print(f"  {'variant':44s} 2->20 loss/L react  fade loss/L  after-fade udp  5->40 loss/L  20->2 udp  40->2 udp  2% udp  5% udp  saving")
    for r in rows:
        runs = run([r["cfg"]], chan=chan, seeds=seeds(chan))[0]

        def ph(i, what):
            n = sum(x["ph"][i][0] for x in runs)
            if what == "loss":
                return sum(x["ph"][i][2] for x in runs) / n / L
            return sum(x["ph"][i][1] for x in runs) / n / fopt(ps[i], L)

        react = sorted(x["st"][1][3] if not math.isnan(x["st"][1][3]) else 1e9 for x in runs)
        print(f"  {r['family'][:44]:44s} {ph(1, 'loss'):12.2f} {react[len(react) // 2]:5.0f} {ph(4, 'loss'):12.2f}"
              f" {ph(5, 'udp'):15.2f} {ph(6, 'loss'):13.2f} {ph(2, 'udp'):10.2f} {ph(7, 'udp'):10.2f}"
              f" {ph(0, 'udp'):7.2f} {ph(3, 'udp'):7.2f} {100 * r['summary']['saving']:6.1f}%")


if __name__ == "__main__":
    main()
