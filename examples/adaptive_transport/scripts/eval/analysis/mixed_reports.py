"""Mixed-criticality captures: which classes carry the delivery reports, and by which transport each
class's messages were lost (the remarks of the paper's mixed-workload paragraph).

    python3 mixed_reports.py measurement_data/mixed

A TCP send requested a report when its console line shows expected_udp > 0; it is matched to the
message's class through the sequence number of its "MIX: seq=N class=C" line. Per class: messages,
the share sent as TCP, the share of all reports it carried, and its losses by transport, over the
same capture window as analyze_mixed_criticality.py.
"""
import re
import sys
from pathlib import Path

import analyze_mixed_criticality as mc

REQ_RE = re.compile(r"expected_udp=(\d+), seq=(\d+)")


def main():
    root = Path(sys.argv[1])
    rows = []
    for j in sorted(root.glob("pwr_*.json")):
        df = mc.analyze_run(j)
        if df is None:
            continue
        lines = j.with_name(j.stem + ".console.log").read_text(errors="replace").splitlines()
        boot = max((i for i, ln in enumerate(lines) if " Remote config: " in ln), default=0)
        requested = set()
        for ln in lines[boot:]:
            m = REQ_RE.search(ln)
            if m and int(m[1]) > 0:
                requested.add(int(m[2]))
        df["report"] = df["seq"].isin(requested) & (df["proto"] == "TCP")
        rows.append(df)
    import pandas as pd
    df = pd.concat(rows)
    reports = df["report"].sum()
    print(f"{len(rows)} captures, {len(df)} messages, {reports} reports")
    for cls, g in df.groupby("cls"):
        lost = g[~g.delivered]
        print(f"class {cls} (L={g.L.iloc[0]:g}): n={len(g)}  TCP {100 * (g.proto == 'TCP').mean():.1f}%  "
              f"reports carried {100 * g.report.sum() / reports:.1f}%  "
              f"lost {len(lost)} (UDP {(lost.proto == 'UDP').sum()}, TCP {(lost.proto == 'TCP').sum()})")


if __name__ == "__main__":
    main()
