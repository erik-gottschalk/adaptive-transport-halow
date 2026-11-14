"""
run_power_experiment.py

Runs PPK2 current captures of the adaptive_transport firmware on the Morse Micro devkit (STM32 +
MM8108 HaLow module), one replicate after another, unattended: set the access point's injected loss,
reset the device, let it connect through the warm-up, stream the HaLow module's current and the two
debug pins to CSV (decimated to 10 kHz by default), and collect the matching console, receiver and
channel logs, with a JSON manifest per replicate.

The firmware is not built or flashed here: the image is assumed to be on the device
(build_and_flash.sh). The run settings (--protocol, --L, --atc-*, --mix-*, --send-interval-ms) are
handed to the receiver (server.py --remote-config), which pushes them to the device at its boot
(remote_config.h); the device's confirmation, from the receiver's log, is recorded in the manifest
as remote_config_confirmed. The device is reset over SWD (OpenOCD), and its console is captured
over the STLink's UART.

The PPK2 runs as an ammeter on the HaLow module's own current tap (the devkit is powered separately),
so the trace holds the module's current only.

Requires ppk2-api, paramiko and pyserial (scripts/requirements.txt).

Examples
--------
Six replicates of the controller at L = 2.5 %, 20 % injected loss, 20 min each:

    python run_power_experiment.py --protocol adp --L 0.025 --loss 20 --replicates 6 \\
        --console-port /dev/ttyACM0 --server-host SERVER_HOST --server-user root

Single-protocol baselines at the same operating point: --protocol tcp, tcp_reply (every send waits
for a delivery report), udp, or udp_ack (acknowledged UDP).

Rehearse without touching the PPK2, OpenOCD, or the server:

    python run_power_experiment.py --protocol adp --L 0.025 --loss 20 --replicates 6 --dry-run \\
        --server-host x --server-user y
"""
import argparse
import datetime as dt
import json
import socket
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import mm_common as common

# PPK2 hardware ADC rate -- fixed, not configurable from the host side.
NATIVE_HZ = 100_000

DEFAULT_PWR_OUT_DIR = common.EVAL / "measurement_data" / "pwr_runs"
DEFAULT_SAMPLE_RATE_HZ = 10_000
DEFAULT_TAP_VOLTAGE_MV = 3300     # HaLow module current-measurement tap; see --voltage's help

# What each --protocol pushes to the device: (adaptive transport, fixed transport TCP, every TCP
# send waits for a delivery report). udp_ack is single-protocol UDP with UDP_ACK.
PROTOCOL_MODES = {
    "adp": (True, True, False),
    "tcp": (False, True, False),
    "tcp_reply": (False, True, True),
    "udp": (False, False, False),
    "udp_ack": (False, False, False),
}


def parse_float_list(text):
    """'0.5,0.1,0.01' -> [0.5, 0.1, 0.01]; '' -> [] (argparse type for --mix-L/--mix-share)."""
    return [float(v) for v in text.split(",") if v.strip()]


def build_remote_config(args):
    """Map --protocol/--L/--atc-* to the REMOTE_CONFIG dict server.py should hand to
    any device that queries it (see server.py's --remote-config and remote_config.h in the
    firmware)."""
    adaptive, use_tcp, wait_reply = PROTOCOL_MODES[args.protocol]
    cfg = {
        "DEMO_USE_ADAPTIVE_TRANSPORT": adaptive,
        "DEMO_PACKET_USE_TCP": use_tcp,
        "DEMO_TCP_WAIT_REPLY": wait_reply,
        "ATC_X": args.atc_x,
        "ATC_L": args.L,
        "ATC_N0": args.atc_n0,
        "ATC_EPS": args.atc_eps,
        "ATC_CALIBRATION_PACKETS": args.atc_calibration_packets,
        "ATC_USE_CHANGE_DETECTION": args.atc_use_change_detection,
        "SEND_INTERVAL_MS": args.send_interval_ms,
        # "%g" per value, the same rendering the firmware echoes back in its confirmation.
        "MIX_L": ",".join(f"{v:g}" for v in args.mix_L),
        "MIX_SHARE": ",".join(f"{v:g}" for v in args.mix_share),
    }
    # Pushed only when given, so a run without these flags uses the firmware's defaults.
    if args.atc_debt_r is not None:
        cfg["ATC_DEBT_R"] = args.atc_debt_r
    if args.atc_debt_per_class is not None:
        cfg["ATC_DEBT_PER_CLASS"] = args.atc_debt_per_class
    if args.atc_debt_tcp is not None:
        cfg["ATC_DEBT_TCP"] = args.atc_debt_tcp
    if args.atc_random_probe is not None:
        cfg["ATC_RANDOM_PROBE"] = args.atc_random_probe
    # Acknowledged UDP is a run setting of its own (UDP_ACK), pushed for udp_ack only.
    if args.protocol == "udp_ack":
        cfg["UDP_ACK"] = True
    return cfg


def log(msg, level="info"):
    common.log(msg, level)


FLASHED_INFO = HERE.parent / "targets" / "mm-mm8108-ekh05" / "build" / "flashed_info.txt"


def read_flashed_info():
    """build_and_flash.sh's record of what is on the board (commit, make variables, ELF md5), as
    {key: value} for the manifest, or None if there is none."""
    if not FLASHED_INFO.exists():
        return None
    info = {}
    for line in FLASHED_INFO.read_text(encoding="utf-8", errors="replace").splitlines():
        if ":" in line and not line.startswith(" "):
            key, _, value = line.partition(":")
            if value.strip():
                info[key.strip()] = value.strip()
    return info


def measure_clock_skew(server, dry_run=False):
    """Estimate (this machine's clock) - (the server's clock), in seconds: the server's time,
    read over SSH, against the midpoint of this machine's time before and after the call (as NTP
    does), so the SSH round trip cancels to first order."""
    if dry_run:
        return 0.0
    t0 = time.time()
    _, out, _ = server._run("date +%s.%N")
    t1 = time.time()
    skew = (t0 + t1) / 2 - float(out.strip())
    log(f"clock skew (this machine - server): {skew:+.3f}s (SSH round trip {t1 - t0:.3f}s)",
        "step")
    return skew


CONFIG_FETCH_WAIT_S = 60    # boot + HaLow association + DHCP + the query, with margin
CONFIG_FETCH_MARKER = " Remote config: "
CONFIG_FETCH_APPLIED = "applied server-provided overrides"


def wait_for_config_fetch(console_path, timeout_s):
    """Poll the console log (flushed per line by ConsoleLogger) until the freshly reset device
    logs its remote-config fetch result, applied or not. Returns the seconds waited, or None on
    timeout."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if Path(console_path).exists() and CONFIG_FETCH_MARKER in Path(console_path).read_text(
                encoding="utf-8", errors="replace"):
            return time.monotonic() - t0
        time.sleep(0.5)
    return None


def last_config_fetch_applied(console_path):
    """True/False for whether the console's LAST remote-config fetch applied the server's config
    (the boot the capture ran under -- earlier lines can belong to the previous boot), or None if
    the console has no fetch line at all."""
    if not Path(console_path).exists():
        return None
    fetches = [line for line in Path(console_path).read_text(
        encoding="utf-8", errors="replace").splitlines() if CONFIG_FETCH_MARKER in line]
    return CONFIG_FETCH_APPLIED in fetches[-1] if fetches else None


def find_last_config_confirmation(paths):
    """Scan the given server_events*.log file paths (JSON-lines, see server.py's log_event()), in
    order, for the last 'config_confirmed' event in the first one that has any, and return its
    effective_config dict -- or None if none of the files contain one. The capture's own log comes
    first; the earlier logs (from before the reset) hold confirmations of earlier boots, possibly
    with other settings, so they are only read if the capture's own log has none."""
    for path in paths:
        if not path.exists():
            continue
        last = None
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if event.get("event_type") == "config_confirmed":
                last = event.get("data", {}).get("effective_config")
        if last is not None:
            return last
    return None


# ---------------------------------------------------------------------------------------------
# decimated CSV capture
# ---------------------------------------------------------------------------------------------


class DecimatingWriter:
    """Averages the raw ~100 kHz PPK2 stream down to blocks of `factor` samples and writes each
    block as one CSV row: mean current, OR'd D0, OR'd D1 (each high if any raw sample in the
    block was high). D0/D1 are fed by the ENABLE_PWR_MEAS debug-pin marker (mmhal_set_debug_pins(),
    see adaptive_transport.c's SET_DEBUG_PROTO) -- build with ENABLE_PWR_MEAS=1 for them to carry
    any signal.

    D0 (MM_DEBUG_0/PA10) and D1 (MM_DEBUG_2/PA0, wired to the PPK2's second digital input) are
    together a 2-bit protocol code, asserted for the duration of each send call and cleared to
    (0, 0) between sends: TCP=(1,0), UDP=(0,1), TCP+reply=(1,1) -- see SET_DEBUG_PROTO's docstring
    in adaptive_transport.c for why this replaces an earlier single-pin, width-based marker (D0
    alone could mark that *a* send call was in flight, ~0.2-0.5 ms regardless of protocol, but
    never reliably which one). OR-decimating each bit independently is safe here because the two
    bits always transition together (both set/cleared in the same mmhal_set_debug_pins() call) and
    consecutive sends are seconds apart -- nowhere near close enough for two different codes' bits
    to land in the same decimated block.

    Columns: "Current(uA)", "D0", "D1" (power_common.load_current_d0 reads them).
    """

    def __init__(self, fh, factor):
        self.fh = fh
        self.factor = max(1, int(factor))
        self._cur_buf: list[float] = []
        self._d0_buf: list[int] = []
        self._d1_buf: list[int] = []
        self.samples_in = 0
        self.samples_out = 0

    def feed(self, samples, raw_digital):
        if not samples:
            return
        self._cur_buf.extend(samples)
        self._d0_buf.extend((b & 1) for b in raw_digital)
        self._d1_buf.extend((b >> 1) & 1 for b in raw_digital)
        self.samples_in += len(samples)
        n = (len(self._cur_buf) // self.factor) * self.factor
        if n == 0:
            return
        cur = np.asarray(self._cur_buf[:n], dtype=np.float64).reshape(-1, self.factor)
        d0 = np.asarray(self._d0_buf[:n], dtype=np.uint8).reshape(-1, self.factor)
        d1 = np.asarray(self._d1_buf[:n], dtype=np.uint8).reshape(-1, self.factor)
        out_cur = cur.mean(axis=1)
        out_d0 = d0.max(axis=1)
        out_d1 = d1.max(axis=1)
        self.fh.write("".join(f"{c:.4f},{d0v},{d1v}\n"
                              for c, d0v, d1v in zip(out_cur, out_d0, out_d1)))
        self.samples_out += len(out_cur)
        del self._cur_buf[:n]
        del self._d0_buf[:n]
        del self._d1_buf[:n]

    def flush_partial(self):
        if not self._cur_buf:
            return
        cur = float(np.mean(self._cur_buf))
        d0 = int(max(self._d0_buf))
        d1 = int(max(self._d1_buf))
        self.fh.write(f"{cur:.4f},{d0},{d1}\n")
        self.samples_out += 1
        self._cur_buf.clear()
        self._d0_buf.clear()
        self._d1_buf.clear()


def capture_power(ppk2, fh, schedule, decimate, set_loss_fn, dry_run=False, poll_s=0.02,
                  progress_every=300):
    """Stream PPK2 samples to `fh` for sum(schedule durations) seconds, stepping the AP loss at
    each phase boundary."""
    writer = DecimatingWriter(fh, decimate)
    phase_ends, acc = [], 0
    for secs, _ in schedule:
        acc += secs
        phase_ends.append(acc)
    total = phase_ends[-1]

    if dry_run:
        log(f"  (dry-run) would capture {total}s across {len(schedule)} phase(s)")
        return writer

    t0 = time.time()
    end = t0 + total
    next_phase = 1
    next_progress = t0 + progress_every
    ppk2.start_measuring()
    try:
        while True:
            now = time.time()
            if now >= end:
                break
            if next_phase < len(schedule) and now >= t0 + phase_ends[next_phase - 1]:
                set_loss_fn(schedule[next_phase][1])
                next_phase += 1
            data = ppk2.get_data()
            if data:
                samples, raw_digital = ppk2.get_samples(data)
                writer.feed(samples, raw_digital)
            else:
                time.sleep(poll_s)
            if now >= next_progress:
                left = int(end - now)
                log(f"  ... {left // 60} min left, {writer.samples_out} samples captured so far")
                next_progress = now + progress_every
    finally:
        ppk2.stop_measuring()
        time.sleep(0.2)                      # let the fetch thread flush its queue
        for _ in range(20):
            data = ppk2.get_data()
            if not data:
                break
            samples, raw_digital = ppk2.get_samples(data)
            writer.feed(samples, raw_digital)
        writer.flush_partial()
    return writer


# ---------------------------------------------------------------------------------------------
# one replicate
# ---------------------------------------------------------------------------------------------


def run_replicate(args, ppk2, server, chs, schedule, rep, openocd_cfg, clock_skew_s=0.0):
    L = args.L
    p_label = schedule[0][1]
    basename = f"pwr_{args.protocol}_L{round(L * 1000):03d}_p{p_label:02d}_r{rep:02d}"
    csv_path = args.out_dir / f"{basename}.csv"
    console_path = args.out_dir / f"{basename}.console.log"
    server_log_path = args.out_dir / f"{basename}.server.log"
    channel_log_path = args.out_dir / f"{basename}.channel.log"
    manifest_path = args.out_dir / f"{basename}.json"

    if not args.overwrite:
        existing = [p for p in (csv_path, console_path, server_log_path, channel_log_path,
                                manifest_path) if p.exists()]
        if existing:
            raise RuntimeError(f"{existing[0]} already exists (use --overwrite to replace)")

    log(f"replicate {rep}: {basename}", "step")

    def set_loss(pct):
        log(f"  AP loss -> {pct}%")
        if not args.dry_run and not chs.set_loss_rate(pct):
            raise RuntimeError(f"failed to set AP loss to {pct}%")

    channel_lines_before = (len(common.CHANNEL_LOG.read_text(encoding="utf-8").splitlines())
                            if common.CHANNEL_LOG.exists() else 0)

    t_start = dt.datetime.now()
    # Boot at 0 % AP loss and apply the capture's loss only once the device has fetched its remote
    # config: the query is a TCP exchange with a 3 s timeout (REMOTE_CONFIG_TIMEOUT_MS), and at
    # 40 % loss it failed on 1 of 3 boots (2026-10-02 smoke run), leaving the device on its
    # compiled-in defaults -- adaptive, L=2.5 % -- for a capture labelled e.g. "udp". The boot's
    # calibration burst then sees a clean channel, which the uncaptured warm-up at the real loss
    # washes out (300 messages at 1 s against the estimator's 20-message half-life).
    boot_loss = 0 if not args.no_reset else schedule[0][1]
    set_loss(boot_loss)
    log(f"  settling {args.settle}s before reset")
    if not args.dry_run:
        time.sleep(args.settle)

    # Start the console capture *before* resetting, so the port is already draining when the
    # firmware's boot banner appears -- see mm_common.ConsoleLogger's docstring.
    console = None
    if not args.no_console and not args.dry_run:
        console = common.ConsoleLogger(args.console_port, args.console_baud, console_path)
        console.start()
        time.sleep(0.5)
        if console.error:
            raise RuntimeError(f"console capture failed to start: {console.error}")

    if not args.no_reset:
        log("  resetting DUT via OpenOCD/STLink", "step")
        common.reset_device(openocd_cfg, args.stlink_serial, dry_run=args.dry_run)
    else:
        log("  --no-reset: reusing whatever is already running on the DUT")

    if boot_loss != schedule[0][1]:
        if console is not None:
            waited = wait_for_config_fetch(console_path, CONFIG_FETCH_WAIT_S)
            if waited is not None:
                log(f"  remote config fetch seen on the console after {waited:.0f}s")
            else:
                log(f"  no remote config fetch on the console within {CONFIG_FETCH_WAIT_S}s -- "
                    f"applying the loss anyway; the post-capture check decides", "warn")
        set_loss(schedule[0][1])

    log(f"  warm-up {args.warmup}s before capture starts (idle-connect, uncaptured)")
    if not args.dry_run:
        time.sleep(args.warmup)

    total_s = sum(d for d, _ in schedule)
    log(f"  capturing {total_s}s -> {csv_path.name}", "step")
    samples_in = samples_out = 0
    actual_hz = float("nan")
    capture_start_utc = dt.datetime.now(dt.timezone.utc).isoformat()
    if not args.dry_run:
        with open(csv_path, "w", encoding="utf-8", newline="") as fh:
            fh.write("Current(uA),D0,D1\n")
            writer = capture_power(ppk2, fh, schedule, args.decimate, set_loss,
                                   dry_run=args.dry_run)
        samples_in, samples_out = writer.samples_in, writer.samples_out
        actual_hz = samples_in / total_s if total_s else float("nan")
    capture_end_utc = dt.datetime.now(dt.timezone.utc).isoformat()
    t_end = dt.datetime.now()

    console_lines = 0
    if console is not None:
        console.stop()
        console_lines = console.lines
        log(f"  console capture stopped after {console_lines} lines")
        if console.error:
            log(f"  console capture error: {console.error}", "warn")

    log("  fetching server log for this replicate", "step")
    server_bytes = 0
    try:
        server_bytes = server.fetch_log(server_log_path)
    except Exception as exc:
        log(f"  could not fetch server log: {exc}", "warn")

    remote_confirmed = None
    if not args.dry_run:
        prereset_paths = sorted(args.out_dir.glob("server_events.prereset*.log"))
        remote_confirmed = find_last_config_confirmation([server_log_path, *prereset_paths])
        # The server log also holds earlier boots' confirmations, so a confirmation there does
        # not prove THIS boot applied the config: check the console's last fetch line too.
        if remote_confirmed is not None and console is not None:
            applied = last_config_fetch_applied(console_path)
            if applied is not True:
                log("  the device's last boot did not apply the remote config (console: "
                    f"{'no fetch line' if applied is None else 'fell back to defaults'}) -- "
                    "treating this replicate as unconfirmed", "warn")
                remote_confirmed = None
        if remote_confirmed is not None:
            log(f"  remote config confirmed by device: {remote_confirmed}", "step")
        else:
            log("  remote config was NOT confirmed for this replicate -- the device may have run "
                "on its built-in defaults instead of --L/--protocol/--atc-*", "warn")

    if not args.dry_run:
        all_lines = (common.CHANNEL_LOG.read_text(encoding="utf-8").splitlines()
                    if common.CHANNEL_LOG.exists() else [])
        mine = all_lines[channel_lines_before:]
        channel_log_path.write_text("\n".join(mine) + ("\n" if mine else ""), encoding="utf-8")
        log(f"  channel log: {len(mine)} entries this replicate")

    commit, dirty = common.git_info()
    dt_ms = 1000.0 / (NATIVE_HZ / args.decimate)
    manifest = {
        "basename": basename, "protocol": args.protocol, "L": L, "replicate": rep,
        "schedule": schedule,
        "burst": args.burst,
        "t_start": t_start.isoformat(timespec="seconds"),
        "t_end": t_end.isoformat(timespec="seconds"),
        "tz": time.strftime("%Z"),
        "capture_start_utc": capture_start_utc, "capture_end_utc": capture_end_utc,
        "clock_skew_s": clock_skew_s,
        "measurement_mode": "AMPERE_MODE",
        "tap": "halow",
        "tap_voltage_mV": args.voltage,
        "baseline_subtraction_required": False,
        "digital_channels": ["D0", "D1"],
        "flashed_info": read_flashed_info(),
        "protocol_pin_encoding": {
            "TCP": [1, 0], "UDP": [0, 1], "TCP+reply": [1, 1],
            "note": ("(D0, D1) asserted together for the duration of each send call (through "
                    "the blocking reply wait too, for TCP+reply), (0, 0) between sends -- see "
                    "adaptive_transport.c's SET_DEBUG_PROTO. Requires ENABLE_PWR_MEAS=1 and D1 "
                    "(MM_DEBUG_2/PA0) wired to the PPK2's second digital input."),
        },
        "warmup_s": args.warmup, "settle_s": args.settle,
        "reset_performed": not args.no_reset,
        "samples_in": samples_in, "samples_out": samples_out,
        "decimate_factor": args.decimate, "dt_ms": dt_ms,
        "measured_sample_rate_hz": actual_hz,
        "console_name": console_path.name if console is not None else None,
        "console_lines": console_lines,
        "server_bytes": server_bytes,
        "git_commit": commit, "git_dirty": dirty,
        "remote_config_pushed": build_remote_config(args),
        "remote_config_confirmed": remote_confirmed,
        "note": (
            "The firmware was not built or flashed for this run (flashed_info: the build on the "
            "device, where recorded). The run settings were pushed to the device through the "
            "receiver at its boot (remote_config_pushed); remote_config_confirmed is what the "
            "device confirmed it applied (None: no confirmation in the receiver's log). "
            "git_commit/git_dirty describe the working repository, not necessarily the build."
        ),
        "argv": " ".join(sys.argv),
    }
    if not args.dry_run:
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    log(f"  done: {csv_path.name}"
        + (f" ({samples_out} samples, {actual_hz:.0f} Hz measured)" if samples_out else ""),
        "step")
    return basename


# ---------------------------------------------------------------------------------------------


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--protocol", required=True, choices=["adp", "tcp", "tcp_reply", "udp", "udp_ack"],
                    help="filename/manifest label AND (via --remote-config, see the module "
                         "docstring) what's pushed to the DUT at every reset -- requires "
                         "DEMO_USE_REMOTE_CONFIG=1 in the flashed build (the default) to actually "
                         "take effect there")
    ap.add_argument("--L", type=float, required=True,
                    help="loss budget: filename/manifest label AND (like --protocol) the ATC_L "
                         "pushed to the DUT via remote config")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--loss", type=int, help="fixed AP loss percent (with --duration)")
    g.add_argument("--schedule", help="multi-phase schedule per replicate, e.g. 600:20,600:1")
    ap.add_argument("--burst", type=float, default=0,
                    help="bursty AP loss: mean loss-run length in packets, as a Gilbert-Elliott "
                         "model through netem (see channel_scheduler.BURST); the --loss/--schedule "
                         "percents stay the long-run loss rate. Default 0 = independent drops "
                         "via the nft rule")
    ap.add_argument("--duration", type=int, default=1200,
                    help="seconds per replicate, with --loss (default 1200 = 20 min)")
    ap.add_argument("--replicates", type=int, default=1)
    ap.add_argument("--send-interval-ms", type=int, default=1000,
                    help="pause after each demo send, in ms (SEND_INTERVAL_MS via remote config; "
                         "default 1000, the firmware's DEMO_SEND_INTERVAL_S)")
    ap.add_argument("--atc-debt-r", type=float, default=None,
                    help="loss-debt horizon in messages, 0 = off (ATC_DEBT_R; not pushed unless "
                         "given, so the firmware default -- 250 -- applies)")
    ap.add_argument("--atc-debt-per-class", type=lambda v: v.lower() in ("1", "true", "yes"),
                    default=None, help="one loss-debt account per mixed-workload class "
                    "(ATC_DEBT_PER_CLASS 1/0; firmware default 1)")
    ap.add_argument("--atc-debt-tcp", type=lambda v: v.lower() in ("1", "true", "yes"),
                    default=None, help="charge locally failed TCP sends to the loss debt "
                    "(ATC_DEBT_TCP 1/0; not pushed unless given, firmware default 0)")
    ap.add_argument("--atc-random-probe", type=lambda v: v.lower() in ("1", "true", "yes"),
                    default=None, help="probe as a 1/x floor on the UDP probability instead of the "
                    "forced cycle (ATC_RANDOM_PROBE 1/0; not pushed unless given, firmware "
                    "default 1)")
    ap.add_argument("--mix-L", type=parse_float_list, default=[],
                    help="mixed-criticality workload (--protocol adp): comma-separated per-class "
                         "loss tolerances, e.g. 0.5,0.1,0.01 -- each message draws a class and is "
                         "planned with its L instead of --L (MIX_L via remote config). --L then "
                         "only labels the run. Needs --mix-share of the same length")
    ap.add_argument("--mix-share", type=parse_float_list, default=[],
                    help="per-class shares for --mix-L, e.g. 0.70,0.29,0.01 (normalized on the "
                         "device)")
    ap.add_argument("--start-rep", type=int, default=1, help="first replicate number (default 1, "
                    "so a partially-completed set can be resumed without renumbering)")
    ap.add_argument("--title", default="")

    ap.add_argument("--atc-x", type=int, default=int(common.BASELINE_ATC["x"]),
                    help=f"probe parameter: UDP floor 1/x, or the forced cycle's length with "
                         f"--atc-random-probe 0 (default {common.BASELINE_ATC['x']})")
    ap.add_argument("--atc-n0", type=int, default=int(common.BASELINE_ATC["N0"]),
                    help=f"estimator confidence parameter (default {common.BASELINE_ATC['N0']})")
    ap.add_argument("--atc-eps", type=float, default=float(common.BASELINE_ATC["eps"]),
                    help="loss-budget division epsilon "
                         f"(default {float(common.BASELINE_ATC['eps']):g})")
    ap.add_argument("--atc-calibration-packets", type=int,
                    default=int(common.BASELINE_ATC["calibration_packets"]),
                    help="startup calibration burst length "
                         f"(default {common.BASELINE_ATC['calibration_packets']})")
    cd_group = ap.add_mutually_exclusive_group()
    cd_default = common.BASELINE_ATC["use_change_detection"] == "true"
    cd_group.add_argument("--atc-use-change-detection", dest="atc_use_change_detection",
                          action="store_true", default=cd_default,
                          help=f"change-detection p_udp_hat reset (default {cd_default})")
    cd_group.add_argument("--no-atc-use-change-detection", dest="atc_use_change_detection",
                          action="store_false")

    ap.add_argument("--ppk2-port", help="default: auto-detect (fails if 0 or >1 found)")
    ap.add_argument("--voltage", type=int, default=DEFAULT_TAP_VOLTAGE_MV,
                    help="voltage of whichever current-measurement tap the PPK2 is wired to right "
                         f"now (see --tap), in mV (default {DEFAULT_TAP_VOLTAGE_MV}). Required by "
                         "the PPK2 even in ampere-meter mode: its calibration model uses this as a "
                         "gain-compensation input, not to supply power -- get it wrong and every "
                         "current reading is silently off. The PPK2 supplies no power to the "
                         "devkit.")
    ap.add_argument("--sample-rate-hz", type=float, default=DEFAULT_SAMPLE_RATE_HZ,
                    help=f"effective decimated rate written to CSV (default "
                         f"{DEFAULT_SAMPLE_RATE_HZ}; the PPK2's raw ADC rate is a fixed "
                         f"~{NATIVE_HZ} Hz)")
    ap.add_argument("--warmup", type=int, default=300,
                    help="idle-connect warm-up after reset before capture starts, seconds "
                         "(default 300 = 5 min)")
    ap.add_argument("--settle", type=int, default=10,
                    help="seconds between applying AP loss and resetting the DUT (default 10)")
    ap.add_argument("--no-reset", action="store_true",
                    help="do not reset the DUT before each replicate (back-to-back windows on one "
                         "continuous boot instead of independent trials)")
    ap.add_argument("--between", type=int, default=15,
                    help="seconds to pause between replicates (default 15)")

    ap.add_argument("--platform", default=common.DEFAULT_PLATFORM,
                    help=f"devkit platform name, used to locate the default OpenOCD config under "
                         f"framework/src/platforms/<platform>/openocd.cfg (default "
                         f"{common.DEFAULT_PLATFORM})")
    ap.add_argument("--openocd-cfg", help="override the OpenOCD config file path (default: "
                    "derived from --platform)")
    ap.add_argument("--stlink-serial", help="select a specific STLink probe by serial number "
                    "(only needed if more than one is attached)")

    ap.add_argument("--console-port", help="serial port for the devkit's UART console (e.g. "
                    "/dev/tty.usbmodemXXXX or COM5) -- required unless --no-console")
    ap.add_argument("--console-baud", type=int, default=common.DEFAULT_CONSOLE_BAUD,
                    help=f"default {common.DEFAULT_CONSOLE_BAUD}")
    ap.add_argument("--no-console", action="store_true",
                    help="skip UART console capture (PPK2 + server log only)")

    ap.add_argument("--server-host", required=True)
    ap.add_argument("--server-user", required=True)
    ap.add_argument("--server-key", default=str(Path.home() / ".ssh" / "halow_lab_ed25519"))
    ap.add_argument("--server-port", type=int, default=22)
    ap.add_argument("--server-dir", default=common.DEFAULT_SERVER_DIR)
    ap.add_argument("--server-python", default="python3")

    ap.add_argument("--ap-host", default=common.DEFAULT_AP_HOST)
    ap.add_argument("--ap-user", default="root")
    ap.add_argument("--ap-key", default=str(Path.home() / ".ssh" / "halow_lab_ed25519"),
                    help="private key for AP auth (default: the same dedicated halow_lab_ed25519 "
                         "key used for --server-key) -- pass --ap-key '' to fall back to "
                         "ap_credentials.py's password instead")

    ap.add_argument("--out-dir", default=None, help="default: eval/measurement_data/pwr_runs/")
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="print every step, touch no hardware")
    return ap


def main():
    ap = build_parser()
    args = ap.parse_args()
    if len(args.mix_L) != len(args.mix_share):
        ap.error("--mix-L and --mix-share need the same number of values")
    if len(args.mix_L) > 4:
        ap.error("--mix-L: at most 4 classes (REMOTE_CONFIG_MAX_MIX_CLASSES in remote_config.h)")
    if args.mix_L and args.protocol != "adp":
        ap.error("--mix-L only applies to --protocol adp")
    if args.out_dir is None:
        args.out_dir = DEFAULT_PWR_OUT_DIR
    args.out_dir = Path(args.out_dir)

    required_mods = [("paramiko", "paramiko"), ("ppk2_api", "ppk2-api")]
    if not args.no_console:
        required_mods.append(("serial", "pyserial"))
    for mod, pkg in required_mods:
        try:
            __import__(mod)
        except ImportError:
            log(f"missing dependency {pkg!r} -- install with: pip install {pkg}", "err")
            return 2

    if not args.no_console and not args.console_port:
        log("--console-port is required unless --no-console is passed", "err")
        return 2

    args.decimate = max(1, round(NATIVE_HZ / args.sample_rate_hz))
    schedule = common.parse_schedule(args.schedule) if args.schedule \
        else [(args.duration, args.loss)]
    remote_config = build_remote_config(args)

    openocd_cfg = Path(args.openocd_cfg) if args.openocd_cfg \
        else common.openocd_cfg_path(args.platform)
    if not args.no_reset and not args.dry_run and not openocd_cfg.exists():
        log(f"OpenOCD config not found: {openocd_cfg} -- pass --openocd-cfg or check --platform",
            "err")
        return 2

    import channel_scheduler as chs
    chs.AP_HOST, chs.AP_USER = args.ap_host, args.ap_user
    chs.AP_KEY = args.ap_key or None
    chs.LOGFILE = str(common.CHANNEL_LOG)
    if args.burst:
        if args.burst < 1:
            log("--burst is a mean run length in packets, so at least 1", "err")
            return 2
        chs.BURST = args.burst
        chs.SERVER_IP = socket.gethostbyname(args.server_host)
    if chs.AP_KEY:
        if not Path(chs.AP_KEY).expanduser().exists() and not args.dry_run:
            log(f"AP key not found: {chs.AP_KEY} -- pass --ap-key, or --ap-key '' to fall back "
                "to ap_credentials.py's password", "err")
            return 2
    elif chs.AP_PASSWORD is None and not args.dry_run:
        log("no --ap-key and ap_credentials.py not found -- channel_scheduler cannot reach the "
            "AP", "err")
        return 2

    from ppk2_api.ppk2_api import PPK2_API, PPK2_MP

    port = args.ppk2_port
    if port is None and not args.dry_run:
        found = PPK2_API.list_devices()
        if len(found) == 1:
            port = found[0]
            log(f"auto-detected PPK2 on {port}")
        elif not found:
            log("no PPK2 found -- pass --ppk2-port explicitly", "err")
            return 2
        else:
            log(f"multiple PPK2 devices found {found} -- pass --ppk2-port explicitly", "err")
            return 2

    args.out_dir.mkdir(parents=True, exist_ok=True)

    ppk2 = None
    server = common.RemoteServer(args.server_host, args.server_user, args.server_key,
                                 args.server_dir, args.server_python, args.server_port,
                                 args.dry_run)
    results = []
    try:
        if not args.dry_run:
            ppk2 = PPK2_MP(port, buffer_max_size_seconds=30)
            if not ppk2.get_modifiers():
                log("could not read PPK2 calibration metadata -- check the port/cable", "err")
                return 2
            ppk2.use_ampere_meter()
            # Required even in ampere-meter mode: ppk2_api's start_measuring() raises "Input
            # voltage not set!" without it, because the calibration model (get_adc_result())
            # scales its S/I gain-compensation terms by current_vdd regardless of mode -- this is
            # NOT the PPK2 supplying power, just telling it what voltage the tap it's measuring is
            # actually on, so its ADC-to-current conversion is correct. Get this wrong (--voltage)
            # and every current reading from this capture is silently biased.
            ppk2.set_source_voltage(args.voltage)
            # The PPK2's own measurement loop still needs to be enabled once, the same call the
            # source-meter mode used to gate its regulated output with -- see the PPK2 hardware
            # guide. Not repeated per replicate: nothing here power-cycles the DUT, so there is no
            # PPK2-side state to reassert.
            ppk2.toggle_DUT_power("ON")
            log(f"PPK2 ready on {port} (ampere-meter mode, {args.voltage} mV tap)", "step")

        server.connect()
        server.start(remote_config=remote_config)
        clock_skew_s = measure_clock_skew(server, args.dry_run)

        for rep in range(args.start_rep, args.start_rep + args.replicates):
            try:
                basename = run_replicate(args, ppk2, server, chs, schedule, rep, openocd_cfg,
                                         clock_skew_s)
                results.append((rep, basename, None))
            except KeyboardInterrupt:
                log("interrupted -- stopping here", "err")
                results.append((rep, None, "interrupted"))
                break
            except Exception as exc:
                log(f"replicate {rep} failed: {exc}", "err")
                results.append((rep, None, str(exc)))
            if rep < args.start_rep + args.replicates - 1:
                log(f"pausing {args.between}s before the next replicate", "step")
                if not args.dry_run:
                    time.sleep(args.between)
    finally:
        if ppk2 is not None:
            try:
                ppk2.stop_measuring()
            except Exception:
                pass
            # Deliberately not powering the DUT off or closing ppk2.ser here: this mode never
            # supplied DUT power to begin with, and PPK2_MP.__del__() calls stop_measuring() again
            # on its own when this object is garbage-collected, which is the safer place to close
            # the port.
        try:
            server.stop()
        except Exception as exc:
            log(f"could not stop server.py: {exc}", "warn")
        server.close()

    log("===== batch summary =====", "step")
    n_ok = sum(1 for _, _, err in results if err is None)
    for rep, basename, err in results:
        if err is None:
            log(f"  OK    r{rep:02d}  {basename}")
        else:
            log(f"  FAIL  r{rep:02d}  {err}", "err")
    log(f"{n_ok}/{len(results)} replicates completed", "step")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
