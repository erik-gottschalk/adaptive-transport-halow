"""Per-class results of the mixed-criticality runs (run_power_experiment.py --mix-L/--mix-share,
run_paper.sh mixed): for each message class, how many messages it had,
how many went as UDP, and the delivered loss against that class's own L -- per run and pooled
over runs.

Each message's class comes from the firmware's per-message "MIX: seq=N class=C L=..." console line,
its protocol from the "ATC: -> UDP/TCP" decision line that follows it, and its delivery from the
server log (seq received at all, by either transport). Only messages sent inside the PPK2 capture
window count, minus the last GRACE_S seconds: a late TCP retransmission of those can still be in
flight when the server is stopped, and would read as lost.

Both logs also hold the tail of the previous boot (the device keeps sending until the next
capture's reset), whose sequence numbers collide with this boot's. Console lines are therefore
taken from the last "Remote config:" line on (this boot's config fetch), and receipts from that
moment on.

Also reports the feasibility floor of the forced probe, L_min = q_T + (q_U - q_T) / x (paper
Section V-A), from the run's own measured UDP and TCP loss: a class with
L below it cannot be met by any mixture while the probe runs.

Usage:
    python analyze_mixed_criticality.py                      # measurement_data/mixed/
    python analyze_mixed_criticality.py --root <dir> --csv per_class.csv
"""
from __future__ import annotations

import argparse
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd

EVAL = Path(__file__).resolve().parent.parent
DEFAULT_ROOT = EVAL / "measurement_data" / "mixed"

GRACE_S = 10.0

MIX_RE = re.compile(r"^(\S+) MIX: seq=(\d+) class=(\d+) L=(\S+)")
DECISION_RE = re.compile(r"^\S+ ATC: -> (UDP|TCP)")


def utc_epoch(text: str) -> float:
    """ISO timestamp -> epoch; naive timestamps are UTC (server.py and the manifest's *_utc)."""
    d = datetime.fromisoformat(text)
    if d.tzinfo is None:
        d = d.replace(tzinfo=timezone.utc)
    return d.timestamp()


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95 % Wilson score interval for a proportion k/n."""
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return centre - half, centre + half


def analyze_run(json_path: Path) -> pd.DataFrame | None:
    m = json.loads(json_path.read_text())
    pushed = m.get("remote_config_pushed") or {}
    if not pushed.get("MIX_L"):
        return None
    base = m["basename"]
    mix_L = [float(v) for v in str(pushed["MIX_L"]).split(",")]
    shares = [float(v) for v in str(pushed["MIX_SHARE"]).split(",")]
    x = int(pushed.get("ATC_X", 50))

    # Console timestamps are host-local wall time (manifest "tz"); t_end is the same instant as
    # capture_end_utc in local time, which gives the local -> UTC offset (rounded to 15 min).
    local_minus_utc = (datetime.fromisoformat(m["t_end"]).replace(tzinfo=timezone.utc).timestamp()
                       - utc_epoch(m["capture_end_utc"]))
    local_minus_utc = round(local_minus_utc / 900) * 900
    to_utc = lambda local: utc_epoch(local) - local_minus_utc

    lines = json_path.with_name(base + ".console.log").read_text(errors="replace").splitlines()
    boot = max((i for i, ln in enumerate(lines) if " Remote config: " in ln), default=0)
    boot_utc = to_utc(lines[boot].split(" ", 1)[0]) if lines else float("-inf")

    lo = utc_epoch(m["capture_start_utc"])
    hi = utc_epoch(m["capture_end_utc"]) - GRACE_S
    msgs, pending = [], None
    for ln in lines[boot:]:
        mm = MIX_RE.match(ln)
        if mm:
            pending = {"t": to_utc(mm[1]), "seq": int(mm[2]), "cls": int(mm[3])}
            continue
        dm = DECISION_RE.match(ln)
        if dm and pending is not None:
            pending["proto"] = dm[1]
            if lo <= pending["t"] <= hi:
                msgs.append(pending)
            pending = None

    # Server clock = this machine's clock - clock_skew_s (see run_power_experiment.py).
    skew = float(m.get("clock_skew_s", 0.0))
    received = set()
    for ln in json_path.with_name(base + ".server.log").read_text(errors="replace").splitlines():
        try:
            ev = json.loads(ln)
        except ValueError:
            continue
        if (ev.get("event_type", "").endswith("_received")
                and utc_epoch(ev["timestamp"]) + skew >= boot_utc - 2.0):
            received.add(ev["data"]["seq"])

    df = pd.DataFrame(msgs)
    if df.empty:
        return None
    df["delivered"] = df["seq"].isin(received)
    df["run"] = base

    # Run-level q_U / q_T from all classes together -> the probe floor.
    udp, tcp = df[df.proto == "UDP"], df[df.proto == "TCP"]
    q_U = 1 - udp.delivered.mean() if len(udp) else float("nan")
    q_T = 1 - tcp.delivered.mean() if len(tcp) else float("nan")
    df["q_U_run"], df["q_T_run"] = q_U, q_T
    df["L_min_run"] = q_T + (q_U - q_T) / x
    df["L"] = df["cls"].map(lambda c: mix_L[c])
    df["share_want"] = df["cls"].map(lambda c: shares[c] / sum(shares))
    return df


def per_class(df: pd.DataFrame) -> pd.DataFrame:
    recs = []
    for (run, cls), g in df.groupby(["run", "cls"]):
        recs.append(summary_row(run, cls, g, len(df[df.run == run])))
    if df["run"].nunique() > 1:
        for cls, g in df.groupby("cls"):
            recs.append(summary_row("POOLED", cls, g, len(df)))
    return pd.DataFrame(recs)


def summary_row(run: str, cls: int, g: pd.DataFrame, n_run: int) -> dict:
    n, lost = len(g), int((~g.delivered).sum())
    ci_lo, ci_hi = wilson(lost, n)
    L = float(g["L"].iloc[0])
    return {
        "run": run, "class": cls, "L": L,
        "share": n / n_run, "share_want": float(g["share_want"].iloc[0]),
        "n": n, "f_udp": float((g.proto == "UDP").mean()),
        "loss": lost / n, "loss_ci95_lo": ci_lo, "loss_ci95_hi": ci_hi,
        "u=loss/L": (lost / n) / L if L else float("nan"),
        "L_min": float(g["L_min_run"].mean()),
        "feasible": L >= float(g["L_min_run"].mean()),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    ap.add_argument("--csv", help="also write the per-class table to this CSV path")
    args = ap.parse_args()

    frames = [f for f in (analyze_run(p) for p in sorted(args.root.glob("pwr_adp_*.json")))
              if f is not None]
    if not frames:
        raise SystemExit(f"no mixed-criticality runs (MIX_L set) under {args.root}")
    table = per_class(pd.concat(frames, ignore_index=True))

    pd.set_option("display.width", 200)
    print(table.to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    print(f"\nMessages sent in the capture window minus the last {GRACE_S:.0f} s. loss = not "
          f"received by the server at all; L_min = q_T + (q_U - q_T)/x from the run's own UDP/TCP "
          f"loss (probe floor).")
    if args.csv:
        table.to_csv(args.csv, index=False)
        print(f"Wrote {args.csv}")


if __name__ == "__main__":
    main()
