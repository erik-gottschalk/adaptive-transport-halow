"""MAC retries (Section S4-A, "MAC retries", and Table S4): how HaLow MAC retries, which injected loss
never exercises, would change G, C, G/C and the controller's saving, up to 20 % loss.

    OPT_WORKERS=8 python3 t3_mac_retry.py [runs] > opt_out/t3_mac_retry.log     (default 400 runs)

The paper's values are recorded in opt_out/t3_mac_retry.log, the tables and results it writes in
opt_out/t3_mac_retry/.

Model. The injected loss p is the loss left after the MAC's retries. If it came from a link whose
frames fail independently with probability e per attempt and the MAC gives each MPDU K attempts,
then p = e^K, and every MPDU the device sends takes (1 - e^K) / (1 - e) attempts on average, i.e.
R(p) = (1 - p) / (1 - p^(1/K)) - 1 retries. K: the Morse rate control's retry chain (mmrc.c,
umac_rc.c) has up to four rates of one or two attempts each, so K <= 8; K = 4 and K = 8 bracket it.

Uplink MPDUs per message (lwIP's fresh connection, one 32 B message): UDP 1 data frame; TCP SYN,
ACK, data, FIN, ACK = 5; the reply adds the ACK of the reply, 1; plus one power-save null frame per
message for both. TCP's segments are themselves retransmitted under the IP loss, so its count is
scaled by 1 / (1 - p). The current traces bound these counts from above (t3_frames_from_traces.py:
2.2, 7.7 and 9.8 transmit bursts per UDP, TCP and TCP-with-reply message at p = 0, the bursts
including the device's 802.11 ACKs of downlink frames).

Energy of one retry, E_r: the transmit burst measured in the traces (~0.17 mJ above the awake floor
at the link's rate) plus the awake time the retry adds (backoff, the frame, the ACK timeout: ~3 ms
at the 17.3 mA, 3.3 V awake floor, ~0.17 mJ), so E_r ~ 0.35 mJ; a degraded link would also fall
back to a slower MCS, where the 1 MHz frame takes about four times as long, E_r ~ 1.2 mJ.

Each case writes a HW_TABLE with E_UDP, E_TCP^0 and E_TCP^r raised by (MPDUs x R(p) x E_r), and
reruns the steady binding cells and the bursty schedule (paper_sweep.REF) on the firmware.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = int(sys.argv[1]) if len(sys.argv) > 1 else 400
BASE = HERE / "hw_table.txt"
OUT = HERE / "opt_out" / "t3_mac_retry"
OUT.mkdir(parents=True, exist_ok=True)

CASES = [("measured (no MAC retries)", None, None),
         ("K=4, E_r=0.35 mJ", 4, 0.35), ("K=8, E_r=0.35 mJ", 8, 0.35), ("K=8, E_r=1.2 mJ", 8, 1.2)]


def retries(p, K):
    if p <= 0:
        return 0.0
    e = p ** (1.0 / K)
    return (1 - p) / (1 - e) - 1


def table_rows():
    rows = []
    for ln in BASE.read_text().splitlines():
        if ln.strip() and not ln.lstrip().startswith("#"):
            rows.append([float(x) for x in ln.split("#")[0].split()])
    return rows


def write_table(K, er):
    path = OUT / f"hw_table_K{K}_E{er}.txt"
    lines = [f"# hw_table.txt + MAC retries: K = {K}, E_r = {er} mJ (t3_mac_retry.py)"]
    for p, eu, et, etr, dt, dtr, late in table_rows():
        r = retries(p, K) * er
        n_udp, n_tcp, n_rep = 1 + 1, 5 / (1 - p) + 1, 1 / (1 - p)
        lines.append(f"{p:.3f} {eu + n_udp * r:.3f} {et + n_tcp * r:.3f} {etr + (n_tcp + n_rep) * r:.3f} "
                     f"{dt:.3f} {dtr:.3f} {late:.4f}")
    path.write_text("\n".join(lines) + "\n")
    return path


SIM = r"""
import json, os, sys
sys.path.insert(0, %r)
from design_table import BINDING, VARIANTS
from opt_run import run, summary
from paper_sweep import REF
runs = int(sys.argv[1])
fw = VARIANTS["firmware"]
out = {}
for L, p in BINDING:
    s = summary(run([fw], chan=dict(CHAN="sched", SCHEDULE=f"3600:{p}", L=L), seeds=runs)[0], L)
    out[f"{L}:{p}"] = dict(loss_L=s["loss_L"], saving=s["saving"])
s = summary(run([fw], chan=REF, seeds=runs)[0], REF["L"])
out["bursty"] = {k: s.get(k) for k in ("loss_L", "saving", "saving_clair")}
print("JSON" + json.dumps(out))
"""

print(f"{'case':28s} " + " ".join(f"{'p=' + str(int(100 * r[0])) + '%':>22s}" for r in table_rows()))
results = {}
for name, K, er in CASES:
    rows = table_rows() if K is None else None
    path = BASE if K is None else write_table(K, er)
    if K is not None:
        rows = [[float(x) for x in ln.split()] for ln in path.read_text().splitlines()[1:]]
    gc = []
    for p, eu, et, etr, *_ in rows:
        G, C = et - eu, etr - et
        gc.append(f"G {G:5.1f} C {C:5.1f} G/C {G / C:4.2f}")
    print(f"{name:28s} " + " ".join(f"{x:>22s}" for x in gc))
    env = dict(os.environ, HW_TABLE=str(path))
    res = subprocess.run([sys.executable, "-c", SIM % str(HERE), str(RUNS)], env=env, cwd=HERE,
                         capture_output=True, text=True)
    line = [ln for ln in res.stdout.splitlines() if ln.startswith("JSON")]
    if not line:
        print(res.stdout[-2000:], res.stderr[-2000:])
        continue
    results[name] = json.loads(line[0][4:])

print(f"\nsimulation, firmware, {RUNS} runs per scenario: saving against plain TCP (loss / L)")
keys = list(next(iter(results.values())).keys()) if results else []
print(f"{'case':28s} " + " ".join(f"{k:>16s}" for k in keys))
for name, r in results.items():
    cells = []
    for k in keys:
        v = r[k]
        cells.append(f"{100 * v['saving']:5.1f}% ({v['loss_L']:.2f})")
    print(f"{name:28s} " + " ".join(f"{c:>16s}" for c in cells))
(OUT / f"results_{RUNS}.json").write_text(json.dumps(results, indent=1))
