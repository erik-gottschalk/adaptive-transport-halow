"""Build sim_opt's hardware table (HW_TABLE) from the single-protocol baseline captures.

    python3 hw_table.py [--root <baselines dir> ...] [--pmax 20] [-o hw_table.txt]

Per injected loss p, from the udp / tcp / tcp_reply baselines, averaged over the replicates that
pass the summary's quality gates. Default: measurement_data/baselines/ (run_paper.sh baselines;
r01 and r02-r03 were captured on different development versions of the firmware) -- neither the loss debt
nor the lwIP socket fix touches a single-protocol send at p <= 20 %, and the replicates agree:
  e_udp e_tcp e_tcpr   energy per message ATTEMPTED [mJ] -- what the device pays per send, whatever
                       became of it (power_common.packet_normalized_energy)
  d_tcp d_tcpr         mean time a send takes beyond the send interval [s]: capture time / sends
                       - interval. Grows with loss (handshake and data retransmissions while the
                       send blocks), and for TCP+reply includes the 3 s in-line waits for late
                       replies
  late                 fraction of TCP+reply sends whose reply missed the 3 s wait (console
                       "TCP demo reply ... failed"), i.e. the probability sim_opt holds a reply

Loss levels above --pmax are left out (40 % is excluded: TCP connects fail on the device there,
as lwIP runs out of sockets and memory); sim_opt extrapolates linearly from the last two rows.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "analysis"))
import summarize_pwr_runs as spr  # noqa: E402

DEFAULT_ROOTS = [HERE.parent / "measurement_data" / "baselines"]
LATE_RE = re.compile(r"TCP demo reply .* failed \(errno 11")
SEQ_RE = re.compile(r"seq=(\d+)")


def late_fraction(console: Path) -> float:
    """Late replies / sends over the boot the capture ran under (its last config fetch on)."""
    lines = console.read_text(errors="replace").splitlines()
    boot = max((i for i, ln in enumerate(lines) if " Remote config: " in ln), default=0)
    seqs = {int(m[1]) for m in map(SEQ_RE.search, lines[boot:]) if m}
    late = sum(bool(LATE_RE.search(ln)) for ln in lines[boot:])
    return late / len(seqs) if seqs else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, nargs="+", default=DEFAULT_ROOTS)
    ap.add_argument("--pmax", type=float, default=20, help="highest loss %% to include")
    ap.add_argument("-o", "--out", type=Path, default=HERE / "hw_table.txt")
    args = ap.parse_args()

    cache = spr.load_cache()
    cells = defaultdict(list)       # (protocol, p%) -> [(energy, extra_s, late), ...]
    for man in sorted(m for root in args.root for m in root.glob("pwr_*.json")):
        row = spr.process_one(man, cache)
        m = row["manifest"]
        if m["protocol"] not in ("udp", "tcp", "tcp_reply") or "error" in row:
            continue
        if spr.excluded(row, spr.POWER_INVALIDATES):
            print(f"  skip {row['basename']}: {[f[1] for f in row['flags'] if f[0] == 'FAIL']}")
            continue
        p = row["p_nom"] * 100
        if p > args.pmax:
            continue
        out = row["out"]
        duration = sum(d for d, _ in m["schedule"])
        extra = duration / out["n_cycles"] - spr.send_interval_s(m)
        late = (late_fraction(man.with_name(m["basename"] + ".console.log"))
                if m["protocol"] == "tcp_reply" else 0.0)
        cells[(m["protocol"], p)].append(
            (out["energy"]["mJ_per_packet_attempted"], extra, late))
    spr.save_cache(cache)

    mean = lambda xs: sum(xs) / len(xs)
    ps = sorted({p for _, p in cells})
    rows = []
    for p in ps:
        if not all((proto, p) in cells for proto in ("udp", "tcp", "tcp_reply")):
            print(f"  p={p:g}%: missing a protocol, left out")
            continue
        u, t, r = cells[("udp", p)], cells[("tcp", p)], cells[("tcp_reply", p)]
        rows.append((p / 100, mean([x[0] for x in u]), mean([x[0] for x in t]),
                     mean([x[0] for x in r]), max(0.0, mean([x[1] for x in t])),
                     max(0.0, mean([x[1] for x in r])), mean([x[2] for x in r]),
                     min(len(u), len(t), len(r))))

    with open(args.out, "w") as f:
        roots = " + ".join(f"{r.name}/" for r in args.root)
        f.write(f"# sim_opt HW_TABLE from {roots} (hw_table.py, p <= {args.pmax:g} %)\n")
        f.write("# p  e_udp  e_tcp  e_tcpr  d_tcp  d_tcpr  late   [mJ per attempt; s beyond the "
                "interval]  reps\n")
        for r in rows:
            f.write(f"{r[0]:.3f} {r[1]:.3f} {r[2]:.3f} {r[3]:.3f} {r[4]:.3f} {r[5]:.3f} {r[6]:.4f}"
                    f"  # {r[7]} rep(s)\n")
    print(open(args.out).read())


if __name__ == "__main__":
    main()
