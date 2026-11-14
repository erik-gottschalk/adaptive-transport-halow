"""Long reporting intervals charged the WNM-sleep energies (Section IV-E and the controller's values
of Table S3).

    python3 ../analysis/analyze_deep_sleep.py      # writes hw_table_wnm.txt
    python3 interval_wnm.py                        -> opt_out/paper_ge_interval_{1s,wnm}_<s>.json

The firmware on the uncapped regime channel (ge_channels.INTERVAL_REF) at the intervals of
paper_sweep.py, once with the 1 s table (hw_table.txt, as paper_sweep.py) and once with
hw_table_wnm.txt. Prints loss / L, the saving against plain TCP and, for Table S3,
the controller's saving diluted by the device's sleep power: dE / (E_TCP + P_s (tau - 10 s)),
with the 5-minute run standing in for longer intervals.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TABLES = {"1 s": os.path.join(HERE, "hw_table.txt"), "wnm": os.path.join(HERE, "hw_table_wnm.txt")}
INTERVALS = (1, 10, 30, 60, 300)
SHARE_INTERVALS = (60, 300, 600, 3600, 86400)
FLOOR_UA, EXTRA_UA, V = 2.2, (0, 10, 100), 3.3


def child(table):
    # opt_run reads HW_TABLE at import, so each table runs in its own process
    os.environ["HW_TABLE"] = TABLES[table]
    sys.path.insert(0, HERE)
    from opt_run import PAPER_RUNS
    from opt_sweep import sweep
    from ge_channels import INTERVAL_REF
    fw = {"time counter + ageing + loss debt (firmware)": dict(EST="fw", FW_DEBT_R=250)}
    tag = "wnm_" if table == "wnm" else "1s_"     # not paper_sweep.py's paper_ge_interval_<s>
    for d in INTERVALS:
        s = sweep(f"paper_ge_interval_{tag}{d}", dict(INTERVAL_REF, MSG_DT=d, DUR=7200), [fw], PAPER_RUNS,
                  alphas=[1.0])[0]["summary"]
        e_tcp = s["e"] / (1 - s["saving"])
        print(f"{d} {s['loss_L']:.4f} {s['saving']:.5f} {e_tcp:.3f} {e_tcp - s['e']:.3f}", flush=True)


def main():
    if len(sys.argv) > 1:
        return child(sys.argv[1])
    res = {}
    for table in TABLES:
        out = subprocess.run([sys.executable, __file__, table], capture_output=True, text=True, check=True).stdout
        res[table] = {int(r[0]): [float(x) for x in r[1:]] for r in (l.split() for l in out.splitlines()) if r}
    print("== interval: loss/L, saving vs plain TCP (1 s energies | WNM-sleep energies)")
    for d in INTERVALS:
        a, b = res["1 s"][d], res["wnm"][d]
        print(f"  {d:4d} s  loss/L {b[0]:.2f}   saving {100 * a[1]:5.1f}% | {100 * b[1]:5.1f}%")
    print("\n== Table S3, controller (WNM): dE / (E_TCP + P_s (tau - 10 s)) [%]")
    print("  interval " + "".join(f"{f'+{x} uA':>9s}" for x in EXTRA_UA))
    for tau in SHARE_INTERVALS:
        _, _, e_tcp, de = res["wnm"][min(tau, 300) if tau in INTERVALS else 300]
        cells = [100 * de / (e_tcp + (FLOOR_UA + x) * V / 1000 * (tau - 10)) for x in EXTRA_UA]
        print(f"  {tau:7d} s" + "".join(f"{v:9.1f}" for v in cells))


if __name__ == "__main__":
    main()
