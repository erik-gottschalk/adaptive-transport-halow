"""Acknowledged UDP against the single-protocol baselines (paper Section IV-E, Table I).

    python3 analyze_udp_ack.py [--root measurement_data/udp_ack]

Per capture: energy per message sent (summarize_pwr_runs, the same extraction as Table I), the
transmissions per message and the given-up messages from the console, and the server-side loss.
Prints them next to the baseline means of hw_table.py's captures (baselines/) at the
same p, as a ratio to UDP and to TCP without and with reply.

Then per p over all captures (the paper's Table I row and Section IV-E(d)): mean E_ack and its
ratio to the UDP and fresh-connection TCP means, G = E_ack - E_UDP and the most that sending every
message unacknowledged could save (G / E_ack), the pooled share of retransmitted messages, the
median time to an acknowledgment that came after the first transmission, and the messages lost:
first sent inside the capture window but never logged by the server (a message still in its
retransmissions when the window closes counts as unresolved, not lost).
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import summarize_pwr_runs as spr  # noqa: E402

DATA = HERE.parent / "measurement_data"
BASE_ROOTS = [DATA / "baselines"]
ACK_RE = re.compile(r"UDP ack for seq=(\d+) after (\d+) transmission\(s\), (\d+) ms")
GIVEUP_RE = re.compile(r"UDP ack for seq=(\d+) given up")
SENT_RE = re.compile(r"^(\S+) UDP demo packet \(raw frame\) sent .*seq=(\d+)\)")


def p_of(man):
    return int(round(man["schedule"][0][1]))


def console_stats(console: Path):
    lines = console.read_text(errors="replace").splitlines()
    boot = max((i for i, ln in enumerate(lines) if " Remote config: " in ln), default=0)
    acks = [m for m in map(ACK_RE.search, lines[boot:]) if m]
    tx = [int(m[2]) for m in acks]
    first_tx_ms = [int(m[3]) for m in acks if int(m[2]) == 1]
    gave_up = sum(bool(GIVEUP_RE.search(ln)) for ln in lines[boot:])
    return tx, gave_up, first_tx_ms


def lost_in_window(man: Path, m: dict):
    """(sent, lost, unresolved): messages first sent inside the capture window, those of them the
    server never logged, and those of the latter with no outcome on the console (still in their
    retransmissions when the capture ended). Console timestamps are local time; the offset to the
    manifest's UTC window is t_end (local) - capture_end_utc, rounded to 15 min."""
    t0 = datetime.fromisoformat(m["capture_start_utc"]).replace(tzinfo=None)
    t1 = datetime.fromisoformat(m["capture_end_utc"]).replace(tzinfo=None)
    off = (datetime.fromisoformat(m["t_end"]) - t1).total_seconds()
    off = timedelta(seconds=round(off / 900) * 900)
    lines = man.with_name(m["basename"] + ".console.log").read_text(errors="replace").splitlines()
    boot = max((i for i, ln in enumerate(lines) if " Remote config: " in ln), default=0)
    first, resolved = {}, set()
    for ln in lines[boot:]:
        if (g := SENT_RE.match(ln)):
            first.setdefault(int(g[2]), datetime.fromisoformat(g[1]) - off)
        elif (g := ACK_RE.search(ln) or GIVEUP_RE.search(ln)):
            resolved.add(int(g[1]))
    received = set()
    for ln in man.with_name(m["basename"] + ".server.log").read_text().splitlines():
        e = json.loads(ln)
        if "received" in e["event_type"] and "seq" in e.get("data", {}):
            received.add(e["data"]["seq"])
    sent = [s for s, t in first.items() if t0 <= t <= t1]
    missing = [s for s in sent if s not in received]
    unresolved = [s for s in missing if s not in resolved]
    return len(sent), len(missing) - len(unresolved), len(unresolved)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=DATA / "udp_ack")
    args = ap.parse_args()
    cache = spr.load_cache()

    base = defaultdict(list)
    for man in sorted(m for r in BASE_ROOTS for m in r.glob("pwr_*.json")):
        row = spr.process_one(man, cache)
        if "out" in row and row["out"].get("energy"):
            base[(row["manifest"]["protocol"], p_of(row["manifest"]))].append(
                row["out"]["energy"]["mJ_per_packet_attempted"])

    per_p = defaultdict(lambda: defaultdict(list))
    for man in sorted(args.root.glob("pwr_udp_ack_*.json")):
        row = spr.process_one(man, cache)
        m = row["manifest"]
        p = p_of(m)
        e = row["out"]["energy"]
        tx, gave_up, first_tx_ms = console_stats(man.with_name(m["basename"] + ".console.log"))
        n = len(tx) + gave_up
        sent, lost, unresolved = lost_in_window(man, m)
        acc = per_p[p]
        acc["E"].append(e["mJ_per_packet_attempted"])
        acc["first_tx_ms"].extend(first_tx_ms)
        for k, v in (("n", n), ("retx", sum(t > 1 for t in tx)), ("gave_up", gave_up),
                     ("sent", sent), ("lost", lost), ("unresolved", unresolved)):
            acc[k].append(v)
        print(f"{m['basename']}: in the capture window {sent} sent, {lost} lost, {unresolved} unresolved")
        print(f"{m['basename']}: p={p}%  E_ack={e['mJ_per_packet_attempted']:.2f} mJ  "
              f"(n_attempted={e.get('n_attempted')})")
        if n:
            print(f"   transmissions/message {np.mean(tx + [5] * gave_up):.3f}, retransmitted "
                  f"{sum(t > 1 for t in tx) / n:.1%}, given up {gave_up} of {n}")
        loss = row["out"].get("loss")
        if loss is not None:
            print(f"   server loss: {json.dumps(loss) if not isinstance(loss, float) else f'{loss:.4f}'}")
        for proto in ("udp", "tcp", "tcp_reply"):
            vals = base.get((proto, p))
            if vals:
                mu = float(np.mean(vals))
                print(f"   {proto:9s} baseline {mu:6.2f} mJ (n={len(vals)}, range "
                      f"{min(vals):.2f}-{max(vals):.2f})  E_ack/E = {e['mJ_per_packet_attempted'] / mu:.3f}")

    print("\nPer p over all captures (Table I row E_ack, Section IV-E(d))")
    for p in sorted(per_p):
        a = per_p[p]
        E = float(np.mean(a["E"]))
        udp, tcp = float(np.mean(base[("udp", p)])), float(np.mean(base[("tcp", p)]))
        G = E - udp
        print(f"p={p:2d}%: E_ack {E:.2f} mJ (n={len(a['E'])}, range {min(a['E']):.2f}-{max(a['E']):.2f})"
              f"  /E_UDP {E / udp:.3f}  /E_TCP0 {E / tcp:.3f}  G {G:.2f} mJ  G/E_ack {G / E:.1%}")
        print(f"        retransmitted {sum(a['retx']) / sum(a['n']):.1%} of {sum(a['n'])} messages, "
              f"given up {sum(a['gave_up'])}, median ack after first transmission "
              f"{np.median(a['first_tx_ms']):.0f} ms; in the capture windows {sum(a['sent'])} sent, "
              f"{sum(a['lost'])} lost, {sum(a['unresolved'])} unresolved")
    spr.save_cache(cache) if hasattr(spr, "save_cache") else None


if __name__ == "__main__":
    main()
