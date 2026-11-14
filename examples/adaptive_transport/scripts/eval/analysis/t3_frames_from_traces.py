"""MAC retries (Section S4-A, "MAC retries"): transmit bursts per message and their energy, from the
current traces of the p = 0 single-protocol baselines (baselines, r01; 10 kHz). Without the raw
traces it prints the same from the stored lists (derived/t3_frames.json).

    python3 t3_frames_from_traces.py [--cycles 400] [--thr-ma 45]

Per send cycle (power_common.detect_cycles), within the first 600 ms after the send call: the
samples above --thr-ma form bursts (contiguous runs, merged across gaps <= 0.3 ms); a burst is a
radio transmission (the module receives at a lower current). Reported per protocol: bursts per
message (median, mean), burst duration, and the burst energy above the awake floor (the median
current of the awake samples below the threshold). The awake floor x a retry's extra radio-on time
is reported separately: a retried frame adds its transmission and keeps the module awake for the
retry's backoff and acknowledgment timeout.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import derived  # noqa: E402
import power_common as pc  # noqa: E402

DATA = HERE.parent / "measurement_data" / "baselines"
ap = argparse.ArgumentParser()
ap.add_argument("--cycles", type=int, default=400)
ap.add_argument("--thr-ma", type=float, default=45.0)
args = ap.parse_args()


def bursts(man, gpio):
    """Per cycle the burst count and the awake floor, per burst its duration, energy above the
    floor and peak: the lists the summary below is computed from."""
    m = json.loads(man.read_text())
    dt_ms, v = m["dt_ms"], m["tap_voltage_mV"] / 1000
    cur, d0, d1 = pc.load_current_d0(man.with_suffix(".csv"))
    cyc = pc.detect_cycles(cur, d0, d1, dt_ms=dt_ms, v_supply=v)
    cyc = cyc[cyc["protocol_gpio"] == gpio].iloc[20:20 + args.cycles]
    win = int(600 / dt_ms)
    nb, durs, energies, floors, peaks = [], [], [], [], []
    for r in cyc["rise"].astype(int):
        seg = cur[r:r + win].astype(float) / 1000          # mA
        hi = seg > args.thr_ma
        floor = np.median(seg[(~hi) & (seg > 2.0)]) if ((~hi) & (seg > 2.0)).any() else 0.0
        floors.append(float(floor))
        # bursts: rising edges of hi, merging gaps of <= 3 samples (0.3 ms)
        idx = np.flatnonzero(hi)
        if idx.size == 0:
            nb.append(0)
            continue
        splits = np.flatnonzero(np.diff(idx) > 3) + 1
        groups = np.split(idx, splits)
        nb.append(len(groups))
        for g in groups:
            s = seg[g[0]:g[-1] + 1]
            durs.append(float((g[-1] - g[0] + 1) * dt_ms))
            energies.append(float(((s - floor).clip(min=0)).sum() * dt_ms * v / 1000))   # mJ
            peaks.append(float(s.max()))
    return dict(nb=[int(x) for x in nb], durs=durs, energies=energies, floors=floors, peaks=peaks)


for proto, gpio in (("udp", "UDP"), ("tcp", "TCP"), ("tcp_reply", "TCP+reply")):
    man = DATA / f"pwr_{proto}_L025_p00_r01.json"
    csv = man.with_suffix(".csv")
    # without the trace: the stored lists (derived/t3_frames.json, see derived.py)
    b = derived.get("t3_frames", f"{derived.key_for(csv)} cycles={args.cycles} thr_ma={args.thr_ma}",
                    lambda: bursts(man, gpio), [csv])
    nb, durs, energies, floors, peaks = (b[k] for k in ("nb", "durs", "energies", "floors", "peaks"))
    nb = np.array(nb)
    print(f"{gpio:9s} cycles {len(nb)}: bursts/message median {np.median(nb):.0f}, mean {nb.mean():.2f} "
          f"(p10 {np.percentile(nb, 10):.0f}, p90 {np.percentile(nb, 90):.0f}); burst "
          f"{np.median(durs):.2f} ms median, peak {np.median(peaks):.0f} mA, energy above floor "
          f"{1000 * np.median(energies):.0f} uJ median ({1000 * np.mean(energies):.0f} mean); awake floor "
          f"{np.median(floors):.1f} mA")
