"""Power analysis of the testbed's captures: the Power Profiler's current trace of the HaLow module
and the two debug pins (D0, D1) the firmware drives during each send.

MEASUREMENT. The Power Profiler measures the HaLow module alone, at the devkit's current tap, so
there is no board current to subtract. Between sends the module's current is no stable floor
either (periodic beacon and wake-up spikes of about 200 ms), so energies and mean powers are
reported as measured, not as a level above idle; G and C, differences of two such levels, are the
same either way.

CYCLES. A cycle runs from the rise of one send's marker to the rise of the next send's, and its
energy is everything the module drew in between: the send's own burst (about 290 ms after the rise)
and whatever beacon or wake-up activity follows before the next send. The marker itself is no
energy window: it falls when the send call returns, long before the radio is done. Sends are
periodic (rise to rise 0.993-1.001 s at the 1 s interval), so the later activity falls into every
cycle alike, whatever its protocol; find_long_gaps() flags a cycle that spans a device reset.

PROTOCOL. The firmware holds a 2-bit code on (D0, D1) for the duration of each send call
(adaptive_transport.c, SET_DEBUG_PROTO): TCP = (1, 0), UDP = (0, 1), TCP carrying a delivery report
= (1, 1), idle = (0, 0). Cycles start at a rise of D0 or D1, and each cycle's protocol is read off
the code (protocol_gpio). The marker's width does not tell the protocols apart (TCP and UDP sends
both hold it for 0.2-0.6 ms); it serves only the reset check.

ALIGNMENT WITH THE RECEIVER. Each manifest records the capture's start (capture_start_utc) and the
offset between the device's and the receiver's clocks (clock_skew_s). offset_from_capture_timestamp()
turns them into the constant between a cycle's index and the sequence number the receiver logged
for that send; delivery (paired_loss_rates, n_delivered) and a cross-check of the protocol codes
against what the receiver saw (compare_to_server_log) follow from it.
"""
from __future__ import annotations

import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------------------------

# Tap voltage [V] if the caller passes none; every manifest records its own (tap_voltage_mV).
V_SUPPLY = 3.3
DT_MS = 0.1             # sample period of the stored trace [ms] (10 kHz; see the manifest's dt_ms)

# Minimum cycles of a protocol before its level is reported rather than NaN.
MIN_N = 20

# (D0, D1) -> protocol; matches adaptive_transport.c's DEBUG_PROTO_TCP/_UDP/_TCP_REPLY.
GPIO_PROTOCOL_CODES = {(1, 0): "TCP", (0, 1): "UDP", (1, 1): "TCP+reply"}

# Largest fraction of cycles with an unmapped (D0, D1) code (anything but the three above) for
# which the protocol codes are still trusted (gpio_classification_reliable).
UNRELIABLE_GPIO_FRAC = 0.01

# A cycle whose rise-to-next-rise gap is this many seconds or more almost certainly spans a
# device reset/reboot rather than one send interval; see find_long_gaps().
RESET_GAP_S = 5.0

# The largest idle time (cycle_s - width_ms) after a long send that find_long_gaps() still takes
# as "this send took long, then one ordinary send interval" rather than an outage: comfortably
# above the 1 s send interval plus scheduling jitter, comfortably below how long a reboot, HaLow
# reassociation and reconnect take.
LONG_SEND_IDLE_RESIDUAL_TOL_S = 2.0


# ---------------------------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------------------------

def load_current_d0(csv_path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A capture's current trace and its two debug pins: (current_ua as float32 microamps, d0,
    d1 as int8 0/1 arrays), from the columns "Current(uA)", "D0" and "D1"."""
    header = pd.read_csv(csv_path, nrows=0).columns
    if "D1" not in header:
        raise ValueError(f"{csv_path}: no D1 column (the second debug pin; see the module docstring)")
    d = pd.read_csv(csv_path, usecols=["Current(uA)", "D0", "D1"],
                    dtype={"Current(uA)": np.float32, "D0": "str", "D1": "str"})
    current_ua = d["Current(uA)"].to_numpy()
    d0 = (d["D0"].to_numpy() == "1").astype(np.int8)
    d1 = (d["D1"].to_numpy() == "1").astype(np.int8)
    return current_ua, d0, d1


def cycles_path_for(csv_path) -> Path:
    """The cycles file that stands in for the trace ``<basename>.csv``: ``<basename>.cycles.csv.gz``."""
    p = Path(csv_path)
    return p.with_name(p.stem + CYCLES_SUFFIX)


def write_cycles(csv_path, out_path=None) -> Path:
    """Write the cycles file of a trace (see raw_cycles). Its first line also records whether D1
    ever went high, the one fact about the trace beyond its cycles that summarize_run() needs
    (gpio_classification_reliable, gpio_pure_tcp)."""
    current_ua, d0, d1 = load_current_d0(csv_path)
    out_path = Path(out_path) if out_path is not None else cycles_path_for(csv_path)
    write_raw_cycles(raw_cycles(current_ua, d0, d1), True, int(d1.max()) if len(d1) else 0,
                     len(current_ua), out_path)
    return out_path


def write_raw_cycles(raw: pd.DataFrame, has_d1: bool, d1_max: int, n_samples: int, out_path) -> None:
    """Write raw_cycles()'s table as a cycles file (see write_cycles)."""
    import gzip
    header = f"# {CYCLES_FORMAT} has_d1={int(has_d1)} d1_max={int(d1_max)} n_samples={int(n_samples)}\n"
    # mtime=0 keeps the gzip output byte-identical across runs
    with open(out_path, "wb") as fb, gzip.GzipFile(fileobj=fb, mode="wb", mtime=0) as gz:
        gz.write(header.encode())
        gz.write(raw.to_csv(index=False, float_format=None).encode())


def read_cycles_file(path) -> tuple[pd.DataFrame, bool, int]:
    """(raw cycles, has_d1, d1_max) from a cycles file written by write_cycles()."""
    import gzip
    with gzip.open(path, "rt") as f:
        header = f.readline()
        if not header.startswith("# " + CYCLES_FORMAT):
            raise ValueError(f"{path}: not a '{CYCLES_FORMAT}' file")
        meta = dict(kv.split("=", 1) for kv in header.split() if "=" in kv)
        # round_trip: pandas' default float parser can be one ulp off what to_csv wrote
        raw = pd.read_csv(f, dtype={"rise": np.int64, "next_rise": np.int64, "fall": np.int64,
                                    "sum_ua": np.float64, "d0": np.int64, "d1": np.int64},
                          float_precision="round_trip")
    return raw, meta.get("has_d1") == "1", int(meta.get("d1_max", 0))


def load_cycles(csv_path, dt_ms: float = DT_MS, v_supply: float = V_SUPPLY):
    """detect_cycles()'s output for a capture, and its D1 channel: from the trace ``csv_path`` if it
    exists, else from its cycles file (cycles_path_for). From a cycles file, the second value is a
    one-element array holding D1's maximum, all that gpio_classification_reliable and gpio_pure_tcp
    read."""
    csv_path = Path(csv_path)
    if csv_path.exists():
        current_ua, d0, d1 = load_current_d0(csv_path)
        return detect_cycles(current_ua, d0, d1, dt_ms=dt_ms, v_supply=v_supply), d1
    cyc_path = cycles_path_for(csv_path)
    if not cyc_path.exists():
        raise FileNotFoundError(f"neither the trace {csv_path.name} nor its cycles file "
                                f"{cyc_path.name} exists in {csv_path.parent}")
    raw, has_d1, d1_max = read_cycles_file(cyc_path)
    if not has_d1:
        raise ValueError(f"{cyc_path}: capture without D1 (the second debug pin)")
    return cycles_from_raw(raw, dt_ms, v_supply), np.array([d1_max], dtype=np.int8)


def parse_server_receipts(log_path) -> pd.DataFrame:
    """The receiver's log (server.py, one JSON event per line) as one row per udp_received or
    tcp_received event, in file order. Columns: seq, event_type, expected_udp, timestamp.

    Every event is kept. A large backward jump in seq is printed as a warning: the receiver's log
    starts before the device reboots for a capture, so it normally holds the end of the previous
    run and then the restart of the sequence numbers; a jump inside a capture would be a reset
    (find_long_gaps) or a long-delayed TCP backlog arriving at once.
    """
    rows = []
    for line in Path(log_path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        ev = json.loads(line)
        if ev.get("event_type") in ("udp_received", "tcp_received"):
            rows.append({"seq": ev["data"]["seq"], "event_type": ev["event_type"],
                        "expected_udp": ev["data"].get("expected_udp") or 0,
                        "timestamp": ev["timestamp"]})
    df = pd.DataFrame(rows)
    if len(df) > 1:
        jumps = np.flatnonzero(np.diff(df["seq"].to_numpy()) < -100)
        for i in jumps:
            print(f"WARNING: {log_path}: seq dropped from {df['seq'].iloc[i]} to "
                 f"{df['seq'].iloc[i + 1]} at {df['timestamp'].iloc[i + 1]} -- possible device "
                 f"restart, or just a severely delayed TCP backlog catching up in one burst; "
                 f"all events kept, worth a manual look", file=sys.stderr)
    return df


def _dedupe_by_seq(receipts: pd.DataFrame) -> pd.DataFrame:
    """Drop duplicate ``seq`` rows (keeping the last, the more recent session), so that
    ``.set_index("seq")`` can be ``.reindex()``-ed: a device restart restarts the sequence numbers
    while the receiver keeps logging into the same file."""
    return receipts.drop_duplicates(subset="seq", keep="last")


def parse_channel_log(path) -> list[tuple[float, int]]:
    """Parse channel_conditions.log into [(unix_seconds, loss_percent), ...] loss_rate_changed
    events, in file order. Caller aligns these to the capture's own clock."""
    from datetime import datetime
    marks = []
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        ev = json.loads(line)
        if ev.get("event_type") == "loss_rate_changed":
            marks.append((datetime.fromisoformat(ev["timestamp"]).timestamp(),
                         ev["data"]["percent"]))
    return marks


# ---------------------------------------------------------------------------------------------
# Cycle detection and protocol classification
# ---------------------------------------------------------------------------------------------

def detect_cycles(current_ua: np.ndarray, d0: np.ndarray, d1: np.ndarray,
                  dt_ms: float = DT_MS, v_supply: float = V_SUPPLY) -> pd.DataFrame:
    """One row per send: from a rise of the marker (D0 or D1) to the next rise.

    Columns:
      rise            sample index of the cycle's rise
      cycle_mJ        energy over the full cycle (rise to next rise): sum(current)*dt*v_supply
      cycle_s         cycle duration, rise to next rise
      width_ms        how long the marker stayed high (the send call; see the module docstring)
      protocol_gpio   the protocol the (D0, D1) code over [rise, fall) names (GPIO_PROTOCOL_CODES),
                      or "unknown"

    cycle_mJ includes whatever beacon or wake-up activity falls inside the cycle besides the send's
    own burst (see the module docstring). The last cycle (no following rise) is dropped, and so is a
    cycle whose marker never falls.
    """
    return cycles_from_raw(raw_cycles(current_ua, d0, d1), dt_ms, v_supply)


# Columns of raw_cycles(): everything detect_cycles() takes from the trace, in samples and in
# summed microamps, so that cycles_from_raw() reproduces its output bit for bit for any dt_ms and
# v_supply. A capture's cycles file (CYCLES_SUFFIX, see load_cycles) stores this table, which is
# how the published data set replaces the raw current traces.
RAW_CYCLE_COLUMNS = ["rise", "next_rise", "fall", "sum_ua", "d0", "d1"]
CYCLES_SUFFIX = ".cycles.csv.gz"
CYCLES_FORMAT = "adaptive-transport cycles v1"


def raw_cycles(current_ua: np.ndarray, d0: np.ndarray, d1: np.ndarray) -> pd.DataFrame:
    """The trace-dependent part of detect_cycles(): per cycle the sample indices of its rise, of
    the next rise and of its fall, the sum of its current samples [uA] from rise to next rise, and
    the (D0, D1) code held over [rise, fall)."""
    active = d0 | d1
    rises = np.flatnonzero((active[1:] == 1) & (active[:-1] == 0)) + 1
    falls = np.flatnonzero((active[1:] == 0) & (active[:-1] == 1)) + 1
    rows = []
    for i, r in enumerate(rises[:-1]):
        nxt = rises[i + 1]
        fl = falls[falls > r]
        if not len(fl):
            continue
        f = fl[0]
        rows.append((int(r), int(nxt), int(f), float(current_ua[r:nxt].astype(np.float64).sum()),
                     int(d0[r:f].max()), int(d1[r:f].max())))
    return pd.DataFrame(rows, columns=RAW_CYCLE_COLUMNS)


def cycles_from_raw(raw: pd.DataFrame, dt_ms: float = DT_MS, v_supply: float = V_SUPPLY) -> pd.DataFrame:
    """detect_cycles()'s output from raw_cycles()'s (see detect_cycles for the columns)."""
    rows = []
    for r, nxt, f, sum_ua, c0, c1 in raw[RAW_CYCLE_COLUMNS].itertuples(index=False):
        cycle_uC = float(sum_ua) * dt_ms / 1000
        rows.append({
            "rise": int(r),
            "cycle_mJ": cycle_uC * v_supply / 1000,
            "cycle_s": (int(nxt) - int(r)) * dt_ms / 1000,
            "width_ms": (int(f) - int(r)) * dt_ms,
            "protocol_gpio": GPIO_PROTOCOL_CODES.get((int(c0), int(c1)), "unknown"),
        })
    if not rows:
        return pd.DataFrame({"rise": pd.Series(dtype=int), "cycle_mJ": pd.Series(dtype=float),
                             "cycle_s": pd.Series(dtype=float), "width_ms": pd.Series(dtype=float),
                             "protocol_gpio": pd.Series(dtype=str)})
    return pd.DataFrame(rows)


def gpio_classification_reliable(cycles: pd.DataFrame, d1: np.ndarray,
                                 max_unknown_frac: float = UNRELIABLE_GPIO_FRAC) -> bool:
    """Whether protocol_gpio can be trusted for this capture: D1 went high at least once (a D1
    that never does means the pin is not wired, unless the run sent only plain TCP -- see
    gpio_pure_tcp), and no more than ``max_unknown_frac`` of cycles show an unmapped code."""
    if cycles.empty or not bool(d1.max()):
        return False
    return (cycles["protocol_gpio"] == "unknown").mean() <= max_unknown_frac


def gpio_pure_tcp(cycles: pd.DataFrame, d1: np.ndarray, receipts: pd.DataFrame | None) -> bool:
    """Whether a capture whose D1 never went high is simply a run that sent only plain TCP (the tcp
    baseline, or the controller at L = 0), whose code (1, 0) never raises D1, so that protocol_gpio
    can be trusted after all. A D1 that is not wired in a run that did send UDP would hide the UDP
    sends (no rise at all), so this also requires the receiver to have received no UDP."""
    if cycles.empty or bool(d1.max()):
        return False
    if not (cycles["protocol_gpio"] == "TCP").all():
        return False
    return receipts is None or not (receipts["event_type"] == "udp_received").any()


def find_long_gaps(cycles: pd.DataFrame, threshold_s: float | None = None,
                   send_interval_s: float = 1.0) -> pd.DataFrame:
    """Cycles whose duration suggests a device reset/reboot occurred inside them (quality
    gate: discard and rerun if this is non-empty). Returns the offending rows.

    ``send_interval_s`` is the run's pause between sends (SEND_INTERVAL_MS, see
    run_power_experiment.py's --send-interval-ms). RESET_GAP_S and LONG_SEND_IDLE_RESIDUAL_TOL_S
    were set for the 1 s cadence, so both are shifted by (send_interval_s - 1): at a 30 s interval
    every ordinary cycle is ~30 s long. ``threshold_s`` overrides the shifted reset threshold.

    A long cycle is not a reset when the device spent it sending: the debug pins are held high for
    the whole send (SET_DEBUG_PROTO/CLEAR_DEBUG_PROTO in adaptive_transport.c), which under loss
    can take many seconds -- a TCP connect() that retries (no application timeout), a lost reply's
    timeout, acknowledged UDP's retransmissions with back-off -- and leaves only an ordinary send
    interval of idle time after the pins fall (LONG_SEND_IDLE_RESIDUAL_TOL_S). A reset leaves the
    pins idle for the whole outage. Every published capture's console shows a single boot.
    """
    extra_s = max(0.0, send_interval_s - 1.0)
    if threshold_s is None:
        threshold_s = RESET_GAP_S + extra_s
    idle_residual_s = cycles["cycle_s"] - cycles["width_ms"] / 1000.0
    long_send = idle_residual_s <= LONG_SEND_IDLE_RESIDUAL_TOL_S + extra_s
    return cycles[(cycles.cycle_s >= threshold_s) & ~long_send]


# ---------------------------------------------------------------------------------------------
# Alignment with the receiver's log
# ---------------------------------------------------------------------------------------------

def _epoch_utc(x) -> float:
    """An ISO timestamp as a UTC epoch, or a numeric epoch unchanged. The manifests' timestamps
    carry their offset; the receiver writes UTC without one, which ``.timestamp()`` would otherwise
    read as local time."""
    if isinstance(x, str):
        d = dt.datetime.fromisoformat(x)
        if d.tzinfo is None:
            d = d.replace(tzinfo=dt.timezone.utc)
        return d.timestamp()
    return float(x)


def offset_from_capture_timestamp(cycles: pd.DataFrame, receipts: pd.DataFrame,
                                  capture_start_utc, dt_ms: float, clock_skew_s: float = 0.0,
                                  tolerance_s: float = 0.5, min_matched: int = 20) -> int:
    """The constant between a cycle's index and the sequence number the receiver logged for that
    send, from the capture's recorded start.

    Each cycle's send time on the receiver's clock is capture_start_utc - clock_skew_s +
    rise*dt_ms/1000; the receipt nearest to it within ``tolerance_s`` (half the 1 s send interval,
    by default) gives that cycle's seq, and seq - cycle index should be one constant across
    (almost) all of them.

    ``capture_start_utc``: an ISO timestamp string or a numeric epoch, both UTC. Raises if fewer
    than ``min_matched`` cycles land within tolerance, or if the matched cycles do not agree on one
    offset by a clear majority -- either means the recorded start or clock offset is wrong.
    """
    capture_start_epoch = _epoch_utc(capture_start_utc)

    receipts = receipts.sort_values("timestamp").reset_index(drop=True)
    epochs = receipts["timestamp"].apply(_epoch_utc).to_numpy()

    server_time = capture_start_epoch + cycles["rise"].to_numpy() * dt_ms / 1000.0 - clock_skew_s
    idx = np.clip(np.searchsorted(epochs, server_time), 1, len(epochs) - 1)
    left = idx - 1
    use_left = np.abs(epochs[left] - server_time) <= np.abs(epochs[idx] - server_time)
    best_idx = np.where(use_left, left, idx)
    within_tol = np.abs(server_time - epochs[best_idx]) <= tolerance_s

    # At high loss a fresh-connection TCP send's receipt lands seconds after its rise (SYN and
    # data retransmissions while the pin is held; median 3.7 s at 40 %), so too few cycles match
    # within +-tolerance_s. Then each cycle's own span is the window: the earliest receipt in
    # [rise - tolerance_s, next rise), which a slow send's own receipt falls into because the pin
    # -- and so the cycle -- stays open while it retries.
    if within_tol.sum() < min_matched:
        span_end = server_time + np.maximum(cycles["cycle_s"].to_numpy(), tolerance_s)
        first = np.clip(np.searchsorted(epochs, server_time - tolerance_s), 0, len(epochs) - 1)
        best_idx = first
        within_tol = (epochs[first] >= server_time - tolerance_s) & (epochs[first] < span_end)

    if within_tol.sum() < min_matched:
        raise RuntimeError(f"only {within_tol.sum()} cycles matched a receipt within "
                          f"{tolerance_s}s -- capture_start_utc/clock_skew_s may be wrong")

    implied = receipts["seq"].to_numpy()[best_idx[within_tol]] - cycles.index.to_numpy()[within_tol]
    vals, counts = np.unique(implied, return_counts=True)
    order = np.argsort(counts)[::-1]
    best = int(vals[order[0]])
    agreement = counts[order[0]] / within_tol.sum()
    runner_up = counts[order[1]] if len(order) > 1 else 0
    # A clear majority, not near-unanimity: under loss a late TCP receipt makes some cycles'
    # nearest receipt the previous message's, spreading their votes over neighbouring offsets
    # (90 % agreement failed by a hair at 20 % loss).
    if agreement < 0.6 or counts[order[0]] < 3 * runner_up:
        raise RuntimeError(f"matched cycles do not converge on one offset (best guess {best}, "
                          f"only {agreement:.0%} agreement, runner-up {runner_up} votes) -- "
                          f"capture_start_utc/clock_skew_s may be wrong")
    return best


def boot_receipts(receipts: pd.DataFrame, capture_start_utc, clock_skew_s: float = 0.0,
                  fallback_s: float | None = None) -> pd.DataFrame:
    """Receipts from the boot the capture ran under, with no upper time bound -- for checks of
    the form "was this send's seq received at all" (paired_loss_rates, n_delivered,
    compare_to_server_log).

    No upper bound, because a send inside the capture can be received after it ends: a TCP
    message's receipt lags its send by up to 45 s at 20 % loss. The lower bound keeps this boot's
    sequence numbers from matching the previous boot's tail in the same log: the last receipt with
    seq <= 2 before the capture starts (the device restarts its counter at 0 on reset, and the
    capture always starts after the warm-up). Falls back to ``fallback_s`` seconds before the
    capture start if there is no such receipt, or returns ``receipts`` unchanged without a capture
    start at all."""
    if capture_start_utc is None or receipts.empty:
        return receipts
    start = _epoch_utc(capture_start_utc) - clock_skew_s
    epochs = receipts["timestamp"].apply(_epoch_utc).to_numpy()
    boots = np.flatnonzero((receipts["seq"].to_numpy() <= 2) & (epochs < start))
    if len(boots):
        lo = epochs[boots[-1]]
    elif fallback_s is not None:
        lo = start - fallback_s
    else:
        return receipts
    return receipts[epochs >= lo]


def compare_to_server_log(cycles: pd.DataFrame, receipts: pd.DataFrame, offset: int,
                          protocol_col: str = "protocol") -> dict:
    """Cross-check ``cycles[protocol_col]`` against the receiver's log for every cycle that can be
    matched to a receipt via ``offset``; it relabels nothing. The pins name the protocol directly,
    so a disagreement points at the alignment, the receiver, or the firmware or wiring.

    Returns {"n_matched", "agreement", "mismatches": list of {"cycle_index", "rise",
    protocol_col: <cycle's label>, "server_proto": <server's label>} dicts for disagreeing
    cycles -- a plain list of dicts rather than a DataFrame so this round-trips through
    summarize_pwr_runs.py's JSON result cache}.
    """
    receipts = receipts.copy()
    receipts["server_proto"] = np.where(
        receipts.event_type == "udp_received", "UDP",
        np.where(receipts.expected_udp > 0, "TCP+reply", "TCP"))
    by_seq = _dedupe_by_seq(receipts).set_index("seq")["server_proto"]

    seq = cycles.index.to_numpy() + offset
    server = by_seq.reindex(seq).to_numpy()
    matched = ~pd.isna(server)
    if matched.sum() == 0:
        return {"n_matched": 0, "agreement": float("nan"), "mismatches": []}

    agree = cycles.loc[matched, protocol_col].to_numpy() == server[matched]
    mismatched_cycles = cycles.loc[matched][~agree]
    mismatches = [
        {"cycle_index": int(idx), "rise": int(row["rise"]),
        protocol_col: row[protocol_col], "server_proto": sp}
        for (idx, row), sp in zip(mismatched_cycles.iterrows(), server[matched][~agree])
    ]
    return {"n_matched": int(matched.sum()), "agreement": float(agree.mean()),
           "mismatches": mismatches}


# ---------------------------------------------------------------------------------------------
# Power and energy
# ---------------------------------------------------------------------------------------------

def wpow(cycles: pd.DataFrame) -> float:
    """Duration-weighted mean power [mW] over a set of cycles: total energy / total time."""
    return cycles["cycle_mJ"].sum() / cycles["cycle_s"].sum()


def n_delivered(cycles: pd.DataFrame, receipts: pd.DataFrame, offset: int) -> int:
    """How many of ``cycles`` the receiver logged, whatever their protocol and whether or not the
    device itself ever learned the outcome."""
    seen = set(_dedupe_by_seq(receipts)["seq"].tolist())
    seqs = cycles.index.to_numpy() + offset
    return int(np.isin(seqs, list(seen)).sum())


def packet_normalized_energy(cycles: pd.DataFrame, n_delivered_count: int | None = None) -> dict:
    """Whole-run energy per message, with no attribution to protocols: every sample belongs to
    exactly one cycle, so the total is exact even where a send's activity (a TCP retransmission
    after the send call returned) spills into the following cycle. This is the comparison the
    paper makes between runs (energy per message sent, and per message delivered).

    Returns total_mJ, total_s, avg_mW, n_attempted (len(cycles)), mJ_per_packet_attempted (energy
    per message sent, whatever became of it), and, if ``n_delivered_count`` is given (see
    n_delivered()), n_delivered and mJ_per_packet_delivered (energy per message the receiver got,
    which does not let a policy look cheaper by delivering less).
    """
    total_mJ = float(cycles["cycle_mJ"].sum())
    total_s = float(cycles["cycle_s"].sum())
    n_attempted = len(cycles)
    out = {
        "n_attempted": n_attempted,
        "total_mJ": total_mJ,
        "total_s": total_s,
        "avg_mW": (total_mJ / total_s) if total_s else float("nan"),
        "mJ_per_packet_attempted": (total_mJ / n_attempted) if n_attempted else float("nan"),
    }
    if n_delivered_count is not None:
        out["n_delivered"] = n_delivered_count
        out["mJ_per_packet_delivered"] = (
            (total_mJ / n_delivered_count) if n_delivered_count else float("nan"))
    return out


def protocol_levels(cycles: pd.DataFrame, min_n: int = MIN_N) -> dict:
    """Per-protocol mean power and the actual (mixed) level, as measured (see the module
    docstring).

    Returns a dict with keys "UDP", "TCP", "TCP+reply" (mean power in mW, or NaN if fewer
    than min_n cycles of that protocol), "n_<protocol>" counts, "actual_mW" (all cycles, any
    protocol), and "n" (total cycles).
    """
    out = {"actual_mW": wpow(cycles), "n": len(cycles)}
    for proto in ("UDP", "TCP", "TCP+reply"):
        g = cycles[cycles.protocol == proto]
        out[f"n_{proto}"] = len(g)
        out[proto] = wpow(g) if len(g) >= min_n else float("nan")
    return out


def never_worse_metrics(levels: dict) -> dict:
    """G (probe gain), C (reply cost), their ratio, and the break-even p_opp_min derived from
    them. p_opp_min is 0 or below (always beneficial) when G >= C, in which case it is reported
    as None rather than a negative number that reads as a real threshold.
    """
    G = levels["TCP"] - levels["UDP"]
    C = levels["TCP+reply"] - levels["TCP"]
    out = {"G_mW": G, "C_mW": C, "G_over_C": G / C if C else float("nan")}
    out["p_opp_min"] = (1 - G / C) if (C and G < C) else None
    return out


def saving_percent(actual_mW: float, tcp_level_mW: float) -> float:
    """Transport saving vs. the within-run pure-TCP level, as a percentage. Positive means
    the measured mixed-protocol policy drew less power than sending everything as TCP would
    have; negative means "never worse than TCP" failed for this run."""
    return (1 - actual_mW / tcp_level_mW) * 100


# ---------------------------------------------------------------------------------------------
# Delivered loss
# ---------------------------------------------------------------------------------------------

def paired_loss_rates(cycles: pd.DataFrame, receipts: pd.DataFrame, offset: int) -> dict:
    """Delivered loss, split by protocol: for each cycle, whether the receiver logged its seq
    (cycle index + ``offset``, see offset_from_capture_timestamp()), in whatever event type.

    Only loss_udp is a quality signal against the injected loss: UDP is never retransmitted, while
    TCP's retransmissions keep loss_tcp near zero up to about 10 % (at 20 % and more,
    fresh-connection TCP does lose messages outright).
    """
    seen_seq = set(_dedupe_by_seq(receipts)["seq"].tolist())

    def _rate(mask: np.ndarray) -> tuple[int, int, float]:
        sent = int(mask.sum())
        if sent == 0:
            return 0, 0, float("nan")
        seqs = cycles.index[mask].to_numpy() + offset
        recv = int(np.isin(seqs, list(seen_seq)).sum())
        return sent, recv, 1 - recv / sent

    n_udp_sent, n_udp_recv, loss_udp = _rate((cycles.protocol == "UDP").to_numpy())
    n_tcp_sent, n_tcp_recv, loss_tcp = _rate(
        cycles.protocol.isin(["TCP", "TCP+reply"]).to_numpy())
    n_sent, n_recv = n_udp_sent + n_tcp_sent, n_udp_recv + n_tcp_recv
    return {
        "n_udp_sent": n_udp_sent, "n_udp_received": n_udp_recv, "loss_udp": loss_udp,
        "n_tcp_sent": n_tcp_sent, "n_tcp_received": n_tcp_recv, "loss_tcp": loss_tcp,
        "loss_overall": (1 - n_recv / n_sent) if n_sent else float("nan"),
        "note": ("only loss_udp is a QA signal against the nominal AP setting (and only when "
                "n_udp_sent is non-trivial). loss_tcp is near zero up to ~10 % AP loss, but "
                "fresh-connection TCP does lose messages outright at >= 20 % (connect/close "
                "giving up before retransmissions get through), so it is real data there, "
                "not noise."),
    }


# ---------------------------------------------------------------------------------------------
# Per-capture summary
# ---------------------------------------------------------------------------------------------

def summarize_run(csv_path, server_log_path=None, label: str | None = None,
                  min_n: int = MIN_N, offset: int | None = None,
                  capture_start_utc=None, capture_end_utc=None, dt_ms: float = DT_MS,
                  clock_skew_s: float = 0.0, v_supply: float = V_SUPPLY,
                  send_interval_s: float = 1.0) -> dict:
    """One capture's summary: protocol levels, G/C, saving, the UDP and reply-request shares
    (f_opp, rho), the reset check, and the energy per message; with the receiver's log
    (``server_log_path``, which then needs ``capture_start_utc`` or an explicit ``offset``) also
    the delivered loss, the energy per message delivered, and the cross-check of the protocol
    codes against the receiver (server_log_check).

    Raises if the protocol codes cannot be trusted (see gpio_classification_reliable,
    gpio_pure_tcp) or the capture cannot be aligned with the receiver's log.
    ``capture_end_utc`` is accepted for the manifests' sake and not needed.
    """
    cycles, d1 = load_cycles(csv_path, dt_ms=dt_ms, v_supply=v_supply)
    resets = find_long_gaps(cycles, send_interval_s=send_interval_s)

    receipts = None
    if server_log_path is not None:
        receipts = parse_server_receipts(server_log_path)
        if offset is None:
            if capture_start_utc is None:
                raise ValueError("a server log needs capture_start_utc (or offset) to be aligned")
            # A sparse run (e.g. 30 s between sends) has far fewer cycles to match in a short
            # capture than the default 20.
            offset = offset_from_capture_timestamp(cycles, receipts, capture_start_utc,
                                                   dt_ms, clock_skew_s,
                                                   min_matched=min(20, max(5, len(cycles) // 2)))
        # The receiver's log can begin with the previous run's tail, whose sequence numbers this
        # boot repeats; seq-membership checks use only this boot's receipts.
        receipts = boot_receipts(receipts, capture_start_utc, clock_skew_s)

    # Decided after the receipts are trimmed: gpio_pure_tcp() needs this capture's own.
    if not (gpio_classification_reliable(cycles, d1) or gpio_pure_tcp(cycles, d1, receipts)):
        raise ValueError(f"{csv_path}: the protocol codes on (D0, D1) are not usable "
                         "(D1 never high in a run that is not plain TCP, or too many unknown codes)")
    cycles["protocol"] = cycles["protocol_gpio"]

    levels = protocol_levels(cycles, min_n=min_n)
    nw = never_worse_metrics(levels)
    n = levels["n"]
    out = {
        "label": label or Path(csv_path).stem,
        "n_cycles": n,
        "f_opp": levels["n_UDP"] / n if n else float("nan"),
        "rho": levels["n_TCP+reply"] / n if n else float("nan"),
        **levels, **nw,
        "n_reset_gaps": len(resets),
    }
    if not np.isnan(levels.get("TCP", float("nan"))):
        out["saving_%"] = saving_percent(levels["actual_mW"], levels["TCP"])

    out["energy"] = packet_normalized_energy(cycles)
    if receipts is not None:
        out["server_log_check"] = compare_to_server_log(cycles, receipts, offset, protocol_col="protocol")
        out["loss"] = paired_loss_rates(cycles, receipts, offset)
        out["energy"] = packet_normalized_energy(cycles, n_delivered(cycles, receipts, offset))
    return out


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python power_common.py <capture.csv> [server log]")
        raise SystemExit(2)
    result = summarize_run(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    for k, v in result.items():
        print(f"{k:<32} {v}")
