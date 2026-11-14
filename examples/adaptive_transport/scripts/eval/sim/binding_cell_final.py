"""The bursty binding cell in simulation (Section S4-B): L = 5 %, a
steady 20 % with runs of 3, the final firmware model (failed TCP sends charged to the debt, TCP
connections failing with 0.145 p^1.5, ge_channels.FAIL), 1 h as the captures and 24 h; 2000 runs.
Hardware counterpart: analyze_ge_schedule.py binding (1.03 L, 9.4 % saving).

    python3 binding_cell_final.py      -> run_paper.sh simulations: binding_cell_final.txt
"""
from design_table import FW
from ge_channels import FAIL, FINAL
from opt_run import PAPER_RUNS, run, summary

L, P = 0.05, 0.20


def main():
    for hours in (1, 24):
        # "0.20": the schedule string is part of the result cache's key (opt_run._key)
        chan = dict(CHAN="sched", SCHEDULE=f"{hours * 3600}:{P:.2f}", L=L, BURST=3, **FINAL, **FAIL)
        s = summary(run([FW], chan=chan, seeds=PAPER_RUNS)[0], L)
        print(f"bursty binding cell, final firmware model, {hours} h: loss/L {s['loss_L']:.3f}  "
              f"saving {100 * s['saving']:.1f}%")


if __name__ == "__main__":
    main()
