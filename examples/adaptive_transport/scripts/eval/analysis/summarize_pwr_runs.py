"""Batch summary over the captures (pwr_<protocol>_L*_p*_r*.json) under eval/measurement_data/.

How a capture is measured and analyzed (current tap, cycles, the protocol code on the two debug
pins, alignment with the receiver's log) is described in power_common.py's module docstring.

Finds every run automatically (no run list to keep in sync), and for each one reports:
  - power results, via power_common.summarize_run: UDP/TCP/TCP+reply levels, G, C, saving%
  - adaptive-layer performance (adp runs only): how well the on-device controller's actual
    delivered loss tracked its target L (u = delivered_loss/L, eff = f/f_opt)

Also runs per-run quality gates and flags/excludes runs that fail them from the per-cell
aggregation, rather than silently averaging in a bad capture.

Usage:
    python summarize_pwr_runs.py                        # every capture under measurement_data/
    python summarize_pwr_runs.py --eval-dir ../measurement_data/grid/debt     # one folder
    python summarize_pwr_runs.py --pattern 'pwr_adp_L025_p20_*.json'
    python summarize_pwr_runs.py --csv results.csv       # also write the full per-run table
    python summarize_pwr_runs.py --no-cache              # ignore/refresh the per-run result cache
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import power_common as pc

try:
    from scipy import stats

    def _t_crit(df: int, conf: float = 0.95) -> float:
        return float(stats.t.ppf(1 - (1 - conf) / 2, df)) if df > 0 else float("nan")
except ImportError:
    def _t_crit(df: int, conf: float = 0.95) -> float:
        return 1.96  # normal approximation fallback if scipy isn't installed

EVAL = Path(__file__).resolve().parent.parent
DATA_DIR = EVAL / "measurement_data"
PLOTS_DIR = EVAL / "plots"
CACHE_PATH = PLOTS_DIR / ".pwr_summary_cache.json"
# Bump whenever power_common.summarize_run() changes what it computes, so cached results from
# the older analysis are recomputed rather than reused. 2: pure-TCP GPIO classification and the
# send-interval-aware reset check (2026-10-02). 3: majority offset vote and boot-bounded receipts
# for delivery (2026-10-05). 4: the reset check excludes every long send, not only TCP's
# (2026-11-13). 5: protocol from the pins only, no width-based fallback fields (2026-11-13).
ANALYSIS_VERSION = 5

# Count TCP messages whose connection failed to open as sent and lost even where the power trace
# has no cycle for them: a run of consecutive failed connects reads as one long stall in the trace
# (see power_common's stalled-cycle handling), so without this those messages would drop out of the
# paired loss count.
COUNT_FAILED_CONNECTS = True
FAILED_CONNECT_RE = re.compile(r"TCP connect to \S+ failed")


def with_failed_connects(out: dict, console_log_path: Path) -> dict:
    """A copy of summarize_run's output with the failed connects the paired loss count missed added
    to n_tcp_sent (and so to the losses): failed connects in the console log minus the TCP messages
    already counted lost. A failed connect is never retried (the next message has the next seq)."""
    lo = out.get("loss") or {}
    if not console_log_path.exists() or "n_tcp_sent" not in lo:
        return out
    fails = sum(1 for ln in console_log_path.read_text(errors="replace").splitlines()
                if FAILED_CONNECT_RE.search(ln))
    miss = max(0, fails - (lo["n_tcp_sent"] - lo["n_tcp_received"]))
    if miss == 0:
        return out
    lo = dict(lo, n_tcp_sent=lo["n_tcp_sent"] + miss, failed_connects_added=miss)
    n = lo["n_udp_sent"] + lo["n_tcp_sent"]
    lo["loss_tcp"] = 1 - lo["n_tcp_received"] / lo["n_tcp_sent"]
    lo["loss_overall"] = 1 - (lo["n_udp_received"] + lo["n_tcp_received"]) / n
    return dict(out, loss=lo)

# Quality-gate thresholds.
MIN_REPLY_CYCLES = 20     # fewer TCP+reply cycles than this is too few for a usable C estimate
LOSS_QA_TOL = 0.50        # loss_udp deviating from the nominal AP setting by more than this, relative
MIN_UDP_SIGNAL = 20       # below this, loss_udp has no reliable QA signal at all (f_opp -> 0)


# ---------------------------------------------------------------------------------------------
# Cache (avoids re-reading multi-hundred-MB CSVs on every invocation)
# ---------------------------------------------------------------------------------------------

def load_cache() -> dict:
    if CACHE_PATH.exists():
        try:
            return json.loads(CACHE_PATH.read_text())
        except json.JSONDecodeError:
            return {}
    return {}


def _jsonable(obj):
    """Recursively convert numpy scalars (float32 in particular -- power_common's per-cycle
    columns are float32, and unlike float64 it is not a subclass of the builtin float, so json
    rejects it as-is) to native Python types so the cache round-trips through json.dumps."""
    if isinstance(obj, dict):
        return {k: _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def save_cache(cache: dict) -> None:
    CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)     # plots/ is not in a fresh checkout
    CACHE_PATH.write_text(json.dumps(_jsonable(cache), indent=1))


# ---------------------------------------------------------------------------------------------
# Manifest helpers
# ---------------------------------------------------------------------------------------------

def send_interval_s(manifest: dict) -> float:
    """The pause between sends the run used (SEND_INTERVAL_MS, pushed via remote config since
    2026-10-02; 1 s, the firmware's compiled-in DEMO_SEND_INTERVAL_S, before that)."""
    return float((manifest.get("remote_config_pushed") or {}).get("SEND_INTERVAL_MS", 1000)) / 1000


def is_mixed(manifest: dict) -> bool:
    """Whether the run used a mixed-criticality workload (MIX_L), i.e. per-message L classes
    rather than one target L -- see analyze_mixed_criticality.py for its per-class results."""
    return bool((manifest.get("remote_config_pushed") or {}).get("MIX_L"))


def target_L(manifest: dict) -> float:
    """The L the controller actually ran with: the confirmed remote-config value if the device
    acknowledged one this replicate, else the manifest's/flashed-build's L."""
    rc = manifest.get("remote_config_confirmed")
    if rc and rc.get("ATC_L") is not None:
        return float(rc["ATC_L"])
    return float(manifest["L"])


def channel_log_check(manifest: dict, channel_log_path: Path) -> tuple[str, str | None]:
    """The manifest's scheduled loss percentages must appear, in order, as a contiguous run of
    loss_rate_changed events in the channel log."""
    want = [pct for _, pct in manifest.get("schedule", [])]
    if not want:
        return "WARN", "manifest has no schedule to check the channel log against"
    if not channel_log_path.exists():
        return "FAIL", "channel_conditions.log missing"
    marks = pc.parse_channel_log(channel_log_path)
    if not marks:
        return "FAIL", "channel_conditions.log has no loss_rate_changed events"
    got = [pct for _, pct in marks]
    n = len(want)
    for start in range(len(got) - n + 1):
        if got[start:start + n] == want:
            return "PASS", None
    return "FAIL", f"channel log loss sequence {got} does not contain expected phase transition(s) {want}"


def config_fetch_check(console_log_path: Path) -> tuple[str, str | None]:
    """The boot the capture ran under must have applied the server's remote config. The device
    fetches it over TCP right after boot, and under heavy AP loss the fetch can time out, leaving
    it on its compiled-in defaults (adaptive, L=2.5 %, change detection on) -- whatever the run is
    labelled. The manifest's remote_config_confirmed cannot show this: the server log also holds
    earlier boots' confirmations. Found on 4 of 107 pre-2026-10-02 captures (e.g. a "udp" baseline
    at 20 % that actually ran adaptive). Only the console's LAST fetch line counts; earlier ones
    belong to the previous boot's tail."""
    if not console_log_path.exists():
        return "WARN", "no console log to confirm this boot applied the remote config"
    fetches = [ln for ln in console_log_path.read_text(encoding="utf-8", errors="replace")
               .splitlines() if " Remote config: " in ln]
    if not fetches:
        return "WARN", "console log has no 'Remote config:' line for this boot"
    if "applied server-provided overrides" not in fetches[-1]:
        return "FAIL", ("this boot did not apply the remote config (console: "
                        f"'{fetches[-1].split(' Remote config: ', 1)[1][:60]}') -- it ran the "
                        "firmware's compiled-in defaults, not the labelled configuration")
    return "PASS", None


# ---------------------------------------------------------------------------------------------
# Quality gates
# ---------------------------------------------------------------------------------------------
#
# Each flag is tagged with which downstream aggregate(s) it actually invalidates, rather than
# every FAIL blanket-excluding a run from everything: a run with too few TCP+reply cycles has an
# unusable C estimate (so its saving_% shouldn't be pooled), but its UDP send/receive counts are
# unaffected, so its delivered_loss/adaptive-L-tracking numbers are still perfectly usable, and
# vice versa for a loss_udp-vs-nominal-p mismatch.
POWER_INVALIDATES = {"reset", "reply_cycles", "channel_log", "config"}
ADAPTIVE_INVALIDATES = {"reset", "loss_qa", "channel_log", "config"}


def run_qa(manifest: dict, out: dict, channel_status: tuple[str, str | None],
          config_status: tuple[str, str | None] = ("PASS", None)) -> tuple[str, list]:
    flags = []  # (level, category, message)

    cfs, cfmsg = config_status
    if cfs != "PASS":
        flags.append((cfs, "config", cfmsg))

    if out["n_reset_gaps"] > 0:
        flags.append(("FAIL", "reset",
                      f"{out['n_reset_gaps']} cycle(s) look like a device reset mid-run"))

    # L = 0 never sends UDP, so it never requests a reply -- no C to estimate, by design.
    if (manifest["protocol"] == "adp" and target_L(manifest) > 0
            and out.get("n_TCP+reply", 0) < MIN_REPLY_CYCLES):
        flags.append(("FAIL", "reply_cycles",
                      f"only {out.get('n_TCP+reply', 0)} TCP+reply cycles captured "
                      f"(<{MIN_REPLY_CYCLES}) -- too few for a usable C estimate"))

    want = [pct for _, pct in manifest.get("schedule", [])]
    p_nom = want[-1] / 100.0 if want else float("nan")
    loss = out.get("loss")
    # loss_udp only has a meaningful QA signal for the adaptive protocol, which is the only one
    # that ever sends UDP by design -- a pure-TCP control run has zero UDP sends deliberately.
    # Nor for a multi-phase schedule: loss_udp averages over phases the controller sends very
    # different amounts of UDP in, so it has no single nominal p to match.
    if len(set(want)) > 1:
        flags.append(("INFO", "loss_qa", f"multi-phase schedule {want} -- no single nominal "
                      f"loss to check loss_udp against; the channel-log check covers the "
                      f"phases"))
    elif manifest["protocol"] == "adp" and loss is not None:
        n_udp_sent = loss["n_udp_sent"]
        if n_udp_sent >= MIN_UDP_SIGNAL and p_nom == p_nom and p_nom > 0:
            rel = abs(loss["loss_udp"] - p_nom) / p_nom
            if rel > LOSS_QA_TOL:
                flags.append(("FAIL", "loss_qa",
                              f"loss_udp={loss['loss_udp']:.1%} vs nominal AP loss "
                              f"p={p_nom:.0%} (relative deviation {rel:.0%} > "
                              f"{LOSS_QA_TOL:.0%})"))
        elif n_udp_sent < MIN_UDP_SIGNAL:
            flags.append(("WARN", "loss_qa",
                          f"only {n_udp_sent} UDP sends -- no reliable loss_udp QA "
                          f"signal at this f_opp; verify against channel_scheduler.py's "
                          f"applied-command log instead"))

    cs, cmsg = channel_status
    if cs != "PASS":
        flags.append((cs, "channel_log", cmsg))

    # The pins name each send's protocol (see power_common's module docstring), so a
    # disagreement here is a WARN pointing at the alignment, the receiver or the wiring, never a
    # reason to distrust or exclude the run's power numbers.
    check = out.get("server_log_check")
    if check and check["n_matched"] >= MIN_UDP_SIGNAL and check["agreement"] < 0.95:
        flags.append(("WARN", "gpio_vs_serverlog",
                      f"protocol_gpio agrees with the server log on only "
                      f"{check['agreement']:.1%} of {check['n_matched']} matched cycles -- check "
                      f"D1 wiring / offset alignment"))

    if any(f[0] == "FAIL" for f in flags):
        status = "FAIL"
    elif any(f[0] == "WARN" for f in flags):
        status = "WARN"
    else:
        status = "PASS"     # INFO flags alone leave a run clean
    return status, flags


def excluded(row: dict, invalidating_categories: set) -> bool:
    """Whether this run's FAILs include one that invalidates the given aggregate -- a WARN, or a
    FAIL in an unrelated category, does not."""
    return any(level == "FAIL" and cat in invalidating_categories
              for level, cat, _ in row["flags"])


# ---------------------------------------------------------------------------------------------
# Adaptive-layer ("stuck to L") metrics
# ---------------------------------------------------------------------------------------------

def adaptive_metrics(manifest: dict, out: dict) -> dict | None:
    """delivered_loss/f/f_opt/margin_sigma:
      f        = fraction of packets actually sent as UDP (out['f_opp'])
      p_udp    = measured UDP delivery loss rate (out['loss']['loss_udp'])
      f_opt    = min(1, L / p_udp) -- the best UDP fraction achievable without breaking L
      delivered_loss = out['loss']['loss_overall'] -- fraction of ALL sent packets lost; equals
                 f * p_udp when TCP-carried cycles lose ~nothing
      u        = delivered_loss / L: 1.0 means the loss budget was spent exactly; <1 under-spent
                 (room to send more UDP); >1 means L was exceeded
      margin_sigma = (delivered_loss - L) / se, se = sqrt(L*(1-L)/n): how many standard errors
                 the achieved loss sits from the target, i.e. whether the miss (if any) is real
    """
    if manifest["protocol"] != "adp" or out.get("loss") is None or is_mixed(manifest):
        return None
    L = target_L(manifest)
    n = out["n_cycles"]
    f = out["f_opp"]
    p_udp = out["loss"]["loss_udp"]
    delivered_loss = out["loss"]["loss_overall"]
    f_opt = min(1.0, L / p_udp) if (p_udp == p_udp and p_udp > 0) else 1.0
    se = math.sqrt(L * (1 - L) / n) if n else float("nan")
    return {
        "L_target": L, "n": n, "f_udp": f, "p_udp": p_udp, "f_opt": f_opt,
        "f_ratio": (f / f_opt) if f_opt else float("nan"),
        "delivered_loss": delivered_loss,
        "u": (delivered_loss / L) if L else float("nan"),
        "margin_sigma": ((delivered_loss - L) / se) if se == se and se > 0 else float("nan"),
    }


# ---------------------------------------------------------------------------------------------
# Per-run processing
# ---------------------------------------------------------------------------------------------

def process_one(json_path: Path, cache: dict, use_cache: bool = True) -> dict:
    manifest = json.loads(json_path.read_text())
    basename = manifest["basename"]
    csv_path = json_path.with_name(basename + ".csv")
    server_log_path = json_path.with_name(basename + ".server.log")
    channel_log_path = json_path.with_name(basename + ".channel.log")

    # The raw trace if it is here, else its cycles file (the published data set has only those;
    # see power_common.load_cycles). The cache entry is keyed by whichever of the two is read.
    source_path = csv_path if csv_path.exists() else pc.cycles_path_for(csv_path)
    if not source_path.exists():
        return {"basename": basename, "manifest": manifest,
                "error": f"missing {csv_path.name} and {source_path.name}"}
    # The receiver's log is optional: the protocol comes from the pins; only the delivered loss
    # and adaptive_metrics() need the log, and come back as NaN/None without it (see run_qa's
    # loss_qa/reply_cycles gates).
    have_server_log = server_log_path.exists()

    stat = source_path.stat()
    # Keyed by the CSV's resolved absolute path, not just basename -- different experiment
    # directories reuse the same basename (e.g. grid/debt/ and grid/no_debt/ both have a
    # pwr_adp_L025_p05_r01), and a
    # basename-only key would let one directory's cache entry silently stand in for another's
    # whenever both happen to be read in the same process (the mtime/size check below still
    # catches this within a single script invocation that always re-derives csv_path from the
    # actual manifest being processed, but a basename-keyed cache still constantly clobbers and
    # re-clobbers the wrong directory's entry across invocations, defeating the cache for every
    # run whose basename collides, and is one bad refactor away from a real correctness bug -- see
    # e.g. any future caller that reads the cache file's entries directly instead of always going
    # through this mtime/size-gated lookup).
    cache_key = str(source_path.resolve())
    cached = cache.get(cache_key)
    if (use_cache and cached and cached.get("csv_mtime") == stat.st_mtime
            and cached.get("csv_size") == stat.st_size
            and cached.get("analysis_version") == ANALYSIS_VERSION):
        out = cached["result"]
    else:
        print(f"  analyzing {basename} ({stat.st_size / 1e6:.0f} MB)...", file=sys.stderr, flush=True)
        try:
            out = pc.summarize_run(
                csv_path, server_log_path if have_server_log else None,
                label=basename,
                capture_start_utc=manifest.get("capture_start_utc"),
                capture_end_utc=manifest.get("capture_end_utc"),
                clock_skew_s=manifest.get("clock_skew_s", 0.0),
                dt_ms=manifest.get("dt_ms", pc.DT_MS),
                v_supply=manifest.get("tap_voltage_mV", pc.V_SUPPLY * 1000) / 1000.0,
                send_interval_s=send_interval_s(manifest),
            )
        except (ValueError, RuntimeError) as exc:
            return {"basename": basename, "manifest": manifest, "error": str(exc)}
        cache[cache_key] = {"csv_mtime": stat.st_mtime, "csv_size": stat.st_size,
                            "analysis_version": ANALYSIS_VERSION, "result": out}

    console_log_path = json_path.with_name(manifest["console_name"]) if manifest.get(
        "console_name") else json_path.with_name(basename + ".console.log")
    if COUNT_FAILED_CONNECTS:
        out = with_failed_connects(out, console_log_path)
    channel_status = channel_log_check(manifest, channel_log_path)
    status, flags = run_qa(manifest, out, channel_status, config_fetch_check(console_log_path))
    adaptive = adaptive_metrics(manifest, out)
    want = [pct for _, pct in manifest.get("schedule", [])]

    return {
        "basename": basename, "manifest": manifest, "out": out,
        "p_nom": (want[-1] / 100.0) if want else float("nan"),
        "status": status, "flags": flags, "adaptive": adaptive,
    }


# ---------------------------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------------------------

def fmt_mw(x):
    return "   n/a" if x != x else f"{x:6.2f}"


def fmt_pct(x, decimals=1):
    return "  n/a" if x != x else f"{100 * x:5.{decimals}f}%"


def power_table(rows: list[dict]) -> pd.DataFrame:
    recs = []
    for r in rows:
        if "error" in r:
            continue
        m, o = r["manifest"], r["out"]
        recs.append({
            "run": r["basename"], "proto": m["protocol"], "L": m["L"],
            "p%": r["p_nom"] * 100 if r["p_nom"] == r["p_nom"] else float("nan"),
            "rep": m.get("replicate"), "status": r["status"], "n": o["n_cycles"],
            "actual_mW": o["actual_mW"], "UDP_mW": o.get("UDP"), "TCP_mW": o.get("TCP"),
            "TCPreply_mW": o.get("TCP+reply"), "G_mW": o.get("G_mW"), "C_mW": o.get("C_mW"),
            "G/C": o.get("G_over_C"), "saving_%": o.get("saving_%"),
            # Whole-run, attribution-free figures (see power_common.packet_normalized_energy()) --
            # the right numbers for a cross-run "never worse than plain TCP" comparison against a
            # separate DEMO_TCP_WAIT_REPLY=0 baseline capture, unlike TCP_mW/saving_% above (which
            # are only as accurate as each cycle's debug-pin bracket is at containing exactly the
            # activity that cycle caused -- see that function's doc).
            "mJ/pkt_attempted": o.get("energy", {}).get("mJ_per_packet_attempted"),
            "mJ/pkt_delivered": o.get("energy", {}).get("mJ_per_packet_delivered"),
        })
    df = pd.DataFrame(recs)
    if df.empty:
        return df
    return df.sort_values(["proto", "L", "p%", "rep"]).reset_index(drop=True)


def adaptive_table(rows: list[dict]) -> pd.DataFrame:
    recs = []
    for r in rows:
        if "error" in r or r["adaptive"] is None:
            continue
        m, a = r["manifest"], r["adaptive"]
        recs.append({
            "run": r["basename"], "L_target": a["L_target"],
            "p%": r["p_nom"] * 100 if r["p_nom"] == r["p_nom"] else float("nan"),
            "rep": m.get("replicate"), "status": r["status"], "n": a["n"],
            "f_udp": a["f_udp"], "p_udp_meas": a["p_udp"], "f_opt": a["f_opt"],
            "f/f_opt(eff)": a["f_ratio"], "delivered_loss": a["delivered_loss"],
            "u=deliv/L": a["u"], "margin_sigma": a["margin_sigma"],
        })
    df = pd.DataFrame(recs)
    if df.empty:
        return df
    return df.sort_values(["L_target", "p%", "rep"]).reset_index(drop=True)


def cell_aggregate(rows: list[dict]) -> pd.DataFrame:
    """Per-(protocol, L, p) aggregation of saving_%, mJ_per_packet_attempted and
    mJ_per_packet_delivered, and (for adp) delivered_loss/u.

    saving_% and delivered_loss are excluded from the pool independently -- via
    POWER_INVALIDATES / ADAPTIVE_INVALIDATES respectively -- rather than both being dropped
    together whenever a run has any FAIL. A reset or a mislabeled-cell (channel log) FAIL
    excludes a run from both, since either puts the whole capture in doubt. The per-packet energies
    share saving_%'s POWER_INVALIDATES exclusion (same underlying capture validity concerns; see
    power_common.packet_normalized_energy() for what it measures and why it's the right number for
    a cross-run, cross-protocol comparison against a separate baseline capture -- e.g. mean
    mJ/pkt_delivered for an "adp" cell against the matching (L, p) "tcp" cell's, no run-count or
    packet-count matching needed between them, per packet_normalized_energy()'s doc)."""
    recs = []
    valid = [r for r in rows if "error" not in r]
    keys = sorted({(r["manifest"]["protocol"], r["manifest"]["L"], r["p_nom"]) for r in valid},
                  key=lambda k: (k[0], k[1], k[2] if k[2] == k[2] else -1))
    for proto, L, p in keys:
        cell = [r for r in valid
               if r["manifest"]["protocol"] == proto and r["manifest"]["L"] == L and r["p_nom"] == p]
        power_cell = [r for r in cell if not excluded(r, POWER_INVALIDATES)]
        savings = np.array([r["out"]["saving_%"] for r in power_cell if "saving_%" in r["out"]])
        mj = {k: np.array([r["out"]["energy"][f"mJ_per_packet_{k}"] for r in power_cell
                           if f"mJ_per_packet_{k}" in r["out"].get("energy", {})])
              for k in ("attempted", "delivered")}
        rec = {"proto": proto, "L": L, "p%": p * 100 if p == p else float("nan"),
              "n_reps_power": len(power_cell)}
        if len(savings) >= 2:
            m, sd = savings.mean(), savings.std(ddof=1)
            hw = _t_crit(len(savings) - 1) * sd / math.sqrt(len(savings))
            rec.update({"saving_%_mean": m, "saving_%_sd": sd, "saving_%_ci95_lo": m - hw,
                       "saving_%_ci95_hi": m + hw})
        elif len(savings) == 1:
            rec["saving_%_mean"] = savings[0]
        for k, v in mj.items():
            if len(v) >= 2:
                m, sd = v.mean(), v.std(ddof=1)
                hw = _t_crit(len(v) - 1) * sd / math.sqrt(len(v))
                rec.update({f"mJ/pkt_{k}_mean": m, f"mJ/pkt_{k}_ci95_lo": m - hw,
                           f"mJ/pkt_{k}_ci95_hi": m + hw,
                           f"mJ/pkt_{k}_min": v.min(), f"mJ/pkt_{k}_max": v.max()})
            elif len(v) == 1:
                rec[f"mJ/pkt_{k}_mean"] = v[0]
        if proto == "adp":
            adaptive_cell = [r for r in cell if not excluded(r, ADAPTIVE_INVALIDATES)]
            rec["n_reps_adaptive"] = len(adaptive_cell)
            dl = np.array([r["adaptive"]["delivered_loss"] for r in adaptive_cell if r["adaptive"]])
            if len(dl) >= 2:
                m, sd = dl.mean(), dl.std(ddof=1)
                hw = _t_crit(len(dl) - 1) * sd / math.sqrt(len(dl))
                rec.update({"delivered_loss_mean": m, "delivered_loss_ci95_lo": m - hw,
                           "delivered_loss_ci95_hi": m + hw,
                           "delivered_loss_min": dl.min(), "delivered_loss_max": dl.max()})
            elif len(dl) == 1:
                rec["delivered_loss_mean"] = dl[0]
        recs.append(rec)
    return pd.DataFrame(recs)


def print_qa_detail(rows: list[dict]) -> None:
    problems = [r for r in rows if "error" in r or r["status"] != "PASS"]
    if not problems:
        print("\nAll runs passed the quality gates cleanly.")
        return
    print(f"\n{'=' * 100}\nQuality-gate detail ({len(problems)} run(s) not clean)\n{'=' * 100}")
    for r in problems:
        if "error" in r:
            print(f"\n{r['basename']}: SKIPPED -- {r['error']}")
            continue
        print(f"\n{r['basename']}: {r['status']}")
        for level, cat, msg in r["flags"]:
            print(f"    [{level}:{cat}] {msg}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-dir", default=str(DATA_DIR), help="searched recursively (default: eval/measurement_data/)")
    ap.add_argument("--pattern", default="pwr_*.json", help="glob for run manifests")
    ap.add_argument("--csv", help="also write the full merged per-run table to this CSV path")
    ap.add_argument("--no-cache", action="store_true", help="ignore/refresh the per-run cache")
    args = ap.parse_args()

    eval_dir = Path(args.eval_dir)
    manifests = sorted(eval_dir.rglob(args.pattern))
    if not manifests:
        print(f"no manifests matching {args.pattern!r} in {eval_dir}", file=sys.stderr)
        raise SystemExit(1)

    cache = load_cache() if not args.no_cache else {}
    rows = [process_one(p, cache, use_cache=not args.no_cache) for p in manifests]
    if not args.no_cache or cache:
        save_cache(cache)

    errors = [r for r in rows if "error" in r]
    if errors:
        print(f"\n{len(errors)} manifest(s) skipped:")
        for r in errors:
            print(f"  {r['basename']}: {r['error']}")

    pd.set_option("display.width", 200)
    pd.set_option("display.max_rows", 200)

    pt = power_table(rows)
    print(f"\n{'=' * 100}\nPower results ({len(pt)} runs)\n{'=' * 100}")
    if not pt.empty:
        print(pt.to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    at = adaptive_table(rows)
    print(f"\n{'=' * 100}\nAdaptive-layer performance -- how well each run stuck to its target L "
         f"({len(at)} adp runs)\n{'=' * 100}")
    if not at.empty:
        print(at.to_string(index=False, float_format=lambda x: f"{x:.4f}"))

    ct = cell_aggregate(rows)
    print(f"\n{'=' * 100}\nPer-cell aggregation over replicates that passed the quality gates "
         f"(FAIL'd runs excluded)\n{'=' * 100}")
    if not ct.empty:
        print(ct.to_string(index=False, float_format=lambda x: f"{x:.3f}"))

    print_qa_detail(rows)

    if args.csv:
        merged = pt.merge(at, on="run", how="left", suffixes=("", "_L")) if not at.empty else pt
        merged.to_csv(args.csv, index=False)
        print(f"\nWrote full per-run table to {args.csv}")


if __name__ == "__main__":
    main()
