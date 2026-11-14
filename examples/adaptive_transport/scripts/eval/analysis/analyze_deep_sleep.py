"""Values of the paper's deep-sleep paragraph (Section IV-E, Table S2
and Table S3) and sim_opt's WNM-sleep hardware table, from the deep-sleep captures
(run_paper.sh deep_sleep): single-protocol baselines at a 10 s send interval with
the HaLow module in WNM sleep with power-down between reports (mode1, p = 0 and 10 %) or in 802.11
power save (mode0, p = 0), one 20-minute capture per cell.

    python3 analyze_deep_sleep.py [-o ../sim/hw_table_wnm.txt]

Per capture: energy per message attempted (power_common.packet_normalized_energy, as Table
I), the report cycle, and the module's sleep floor (median current 3-8 s after a
send, mean over all cycles, WNM only). Per cell: G = E_TCP^0 - E_UDP, C = E_TCP^r - E_TCP^0. Then:
  - the interval at which WNM sleep becomes cheaper than power save, per transport, from the
    power-save idle power (median over seconds 2-10 of the cycle of the per-second mean, which
    leaves out TCP's closing exchange) and the WNM energies;
  - the clean-channel share G / (E_TCP^0 + P_s (tau - 10 s)) of Table S3;
  - the WNM HW_TABLE: the measured energies at 0 and 10 %, 5 % interpolated between them, and
    above 10 % the 10 % point plus the 1 s table's rise (../sim/hw_table.txt), conservative since
    TCP's cost rose much faster with loss in WNM sleep than in power save; sim_opt extrapolates
    beyond 20 % along the last segment. Send times and late-reply probabilities from the 1 s table,
    which the sleep does not change.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "measurement_data" / "deep_sleep"
SIM = HERE.parent / "sim"
sys.path.insert(0, str(HERE))
import derived  # noqa: E402
import power_common as pc  # noqa: E402

PROTOS = ("udp", "tcp", "tcp_reply")
SLEEP_SHARE_INTERVALS = (60, 300, 600, 3600, 86400)
EXTRA_UA = (0, 10, 100)          # device sleep current beyond the module's own [uA]


def capture(mode, proto, p):
    """The capture's values from its trace, or without the trace from derived/deep_sleep.json;
    None if the capture does not exist."""
    csv = DATA / ("wnm_sleep" if mode else "power_save") / f"pwr_{proto}_L050_p{p:02d}_r01.csv"
    if not csv.with_suffix(".json").exists():
        return None
    return derived.get("deep_sleep", derived.key_for(csv), lambda: _capture(csv), [csv])


def _capture(csv):
    cur, d0, d1 = pc.load_current_d0(csv)
    cyc = pc.detect_cycles(cur, d0, d1).iloc[1:-1]      # drop the partial first/last cycle
    e = pc.packet_normalized_energy(cyc)
    rises = cyc["rise"].to_numpy()
    step = int(round(1000 / pc.DT_MS))                   # samples per second
    floor = np.mean(np.concatenate([cur[r + 3 * step:r + 8 * step] for r in rises]))
    per_s = np.mean([[cur[r + k * step:r + (k + 1) * step].mean() for k in range(1, 10)] for r in rises], axis=0)
    idle = float(np.median(per_s)) * pc.V_SUPPLY / 1000                    # mW
    return dict(E=float(e["mJ_per_packet_attempted"]), cycle=float(cyc["cycle_s"].median()),
                floor_ua=float(floor), idle_mW=float(idle), n=len(cyc))


def read_hw_table(path):
    rows = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#")[0].split()
        if len(line) == 7:
            rows[float(line[0])] = [float(x) for x in line[1:]]
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default=str(SIM / "hw_table_wnm.txt"))
    args = ap.parse_args()

    res = {}
    for mode, p in ((0, 0), (1, 0), (1, 10)):
        res[mode, p] = {pr: capture(mode, pr, p) for pr in PROTOS}
    print("== energy per report [mJ], 10 s interval (one capture each)")
    for (mode, p), c in res.items():
        if any(v is None for v in c.values()):
            print(f"  mode{mode} p={p}%: incomplete")
            continue
        u, t, r = (c[pr]["E"] for pr in PROTOS)
        name = "WNM sleep " if mode else "power save"
        print(f"  {name} p={p:2d}%  UDP {u:6.1f}  TCP {t:6.1f}  TCP+r {r:6.1f}  G {t - u:5.1f}  C {r - t:5.1f}"
              f"  G/C {(t - u) / (r - t):4.1f}  G/E_TCP {100 * (t - u) / t:4.1f}%"
              f"  cycle {c['udp']['cycle']:.2f}-{c['tcp']['cycle']:.2f} s")
    wnm0, ps0 = res[1, 0], res[0, 0]
    floor = np.median([wnm0[pr]["floor_ua"] for pr in PROTOS])
    print(f"\n== WNM sleep floor (HaLow module): {floor:.1f} uA")

    print("== WNM sleep cheaper than power save beyond:")
    p_floor = floor * pc.V_SUPPLY / 1000
    for pr in PROTOS:
        idle = ps0[pr]["idle_mW"]
        active = ps0[pr]["E"] - idle * 9                 # all but the idle seconds 2-10
        print(f"  {pr:9s} {(wnm0[pr]['E'] - active) / (idle - p_floor) + 1:5.0f} s  (power-save idle {idle:.2f} mW)")

    g, e_tcp = wnm0["tcp"]["E"] - wnm0["udp"]["E"], wnm0["tcp"]["E"]
    print("\n== clean-channel share G / (E_TCP^0 + P_s (tau - 10 s)) [%], module floor + extra current")
    print("  interval " + "".join(f"{f'+{x} uA':>9s}" for x in EXTRA_UA))
    for tau in SLEEP_SHARE_INTERVALS:
        cells = [100 * g / (e_tcp + (floor + x) * pc.V_SUPPLY / 1000 * (tau - 10)) for x in EXTRA_UA]
        print(f"  {tau:7d} s" + "".join(f"{v:9.1f}" for v in cells))

    if any(v is None for v in res[1, 10].values()):
        print("\nWNM p = 10 % incomplete: HW_TABLE not written")
        return
    hw1 = read_hw_table(SIM / "hw_table.txt")
    e0 = [res[1, 0][pr]["E"] for pr in PROTOS]
    e10 = [res[1, 10][pr]["E"] for pr in PROTOS]
    lines = ["# sim_opt HW_TABLE, WNM sleep between reports (analyze_deep_sleep.py): energies measured at",
             "# p = 0 and 10 % (deep_sleep/wnm_sleep, one capture each, 10 s interval), 5 % between them,",
             "# 20 % the 10 % point plus the rise of hw_table.txt; send times and late replies from hw_table.txt",
             "# p  e_udp  e_tcp  e_tcpr  d_tcp  d_tcpr  late"]
    for p in (0.0, 0.05, 0.10, 0.20):
        if p <= 0.10:
            e = [a + (b - a) * p / 0.10 for a, b in zip(e0, e10)]
        else:
            e = [b + hw1[p][i] - hw1[0.10][i] for i, b in enumerate(e10)]
        lines.append(f"{p:.3f} {e[0]:.3f} {e[1]:.3f} {e[2]:.3f} " + " ".join(f"{x:.4f}" for x in hw1[p][3:]))
    Path(args.out).write_text("\n".join(lines) + "\n")
    print(f"\nwrote {args.out}:\n" + "\n".join(lines))


if __name__ == "__main__":
    main()
