#!/usr/bin/env bash
# Everything the paper reports, in one script: the hardware captures, the simulations and the
# analysis that turns both into the paper's tables, figures and numbers.
#
#   ./run_paper.sh analysis        every table, figure and number from measurement_data/ (no hardware)
#   ./run_paper.sh simulations     every simulation, into sim/opt_out/ (no hardware)
#   ./run_paper.sh captures        every hardware capture (all targets below; about 6 days)
#   ./run_paper.sh <target> ...    the captures of some targets:
#
#     baselines   UDP, TCP, TCP with reply at p = 0, 5, 10, 20 %, 3 replicates    Table I, Fig. 3, 6
#     grid        the controller over L x p (with and without the loss debt)      Fig. 3, Sec. IV-C/E
#     step        step schedule, L = 5 %, 5 replicates                            Fig. 4
#     mixed       three message classes at p = 15 %, 12 x 2 h                     Fig. 5
#     bursty      bursty, changing loss (4 h) and the binding cell under bursts   Fig. 7, Table II
#     udp_ack     acknowledged UDP at p = 0, 5, 10, 20 %, 3 replicates            Table I, Sec. IV-E
#     deep_sleep  10 s interval, 802.11 power save and WNM sleep                  Table S2
#
#   DRY_RUN=1 ./run_paper.sh captures    print the plan (one line per capture), touch no hardware
#
# The captures are exactly those of the published data: the same cells, replicates, schedules and
# controller settings (passed explicitly, as each capture ran; see "Firmware and settings" in
# measurement_data/README.md), written to the same folders under OUT_ROOT (default:
# measurement_data/, where the data repository is checked out). A capture whose manifest exists is
# skipped, so a run resumes where it stopped; to measure afresh, start from an empty OUT_ROOT. Within
# a target the cells of each replicate round run in a fixed shuffled order, so that slow drift in the
# internet path does not line up with one cell. The script builds and flashes the firmware itself
# (scripts/build_and_flash.sh; the WNM-sleep captures need DEMO_SLEEP_MODE=1) and refuses a build
# from modified firmware sources. A capture whose device did not confirm the settings it was sent is
# moved to <folder>/incomplete/ and repeated, up to MAX_ATTEMPTS times.
#
# Hardware and setup (see the top-level README): the devkit with the Power Profiler Kit II on its
# current tap and debug pins, the access point reachable over SSH (it injects the loss), and
# scripts/server.py running on the receiver in ~/adapt/. Set CONSOLE_PORT and PPK2_PORT, and
# AP_HOST, SERVER_HOST and SERVER_KEY, in the environment or in scripts/lab.local.env
# (see scripts/lab.env.example).
#
# Python: $PY, else the repository's .venv, else python3 on the PATH.
set -uo pipefail
EVAL_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_DIR="$(cd "$EVAL_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [ -z "${PY:-}" ]; then
    if [ -x "$REPO_ROOT/.venv/bin/python" ]; then PY="$REPO_ROOT/.venv/bin/python"; else PY=python3; fi
fi
FLASHED_INFO="$SCRIPT_DIR/../targets/mm-mm8108-ekh05/build/flashed_info.txt"
: "${DRY_RUN:=0}"
: "${MAX_ATTEMPTS:=2}"
: "${OUT_ROOT:=$EVAL_DIR/measurement_data}"
HW_TARGETS=(baselines grid step mixed bursty udp_ack deep_sleep)

usage() { sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
[ $# -gt 0 ] || usage 1

# ---------------------------------------------------------------------------------------------
# simulations and analysis
# ---------------------------------------------------------------------------------------------

simulations() {
    local s out="sim/opt_out/rerun_window_ge"
    cd "$EVAL_DIR/sim" || return 1
    mkdir -p "opt_out/rerun_window_ge"
    echo "=== simulations: build (gcc), then each script into $out/"
    bash build.sh > opt_out/build.log 2>&1 || { echo "build failed, see sim/opt_out/build.log"; return 1; }
    for s in design_table steady_table sched20_paper attainable_table k_sweep_final \
             binding_cell_final mix_alarm_run_length interval_wnm; do
        echo "$(date '+%a %H:%M') $s"
        nice -n 19 "$PY" "$s.py" > "opt_out/rerun_window_ge/$s.txt" 2>&1 \
            || echo "  $s failed, see $out/$s.txt"
    done
    echo "$(date '+%a %H:%M') t3_mac_retry"
    nice -n 19 "$PY" t3_mac_retry.py > opt_out/t3_mac_retry.log 2>&1 \
        || echo "  t3_mac_retry failed, see sim/opt_out/t3_mac_retry.log"
    echo "=== simulations done; compare with the paper's outputs: git diff -- sim/opt_out"
}

ANALYSES=(
    "sim/hw_table.py"
    "analysis/analyze_udp_ack.py"
    "analysis/grid_table.py"
    "analysis/paper_numbers.py"
    "analysis/analyze_step_schedule.py"
    "analysis/analyze_mixed_criticality.py"
    "analysis/mixed_reports.py measurement_data/mixed"
    "analysis/analyze_ge_schedule.py"
    "analysis/analyze_ge_schedule.py binding"
    "analysis/t1_bursty_over_15L.py"
    "analysis/analyze_deep_sleep.py -o /dev/null"
    "analysis/t3_frames_from_traces.py"
    "analysis/export_ieee_figures.py"
)

analysis() {
    local a
    cd "$EVAL_DIR" || return 1
    [ -f measurement_data/captures.csv ] || { echo "no data in measurement_data/ (git submodule update --init)"; return 1; }
    for a in "${ANALYSES[@]}"; do
        echo; echo "=== $a"
        # shellcheck disable=SC2086
        "$PY" $a 2>&1 | grep -vE '^\s*analyzing |^WARNING'
        [ "${PIPESTATUS[0]}" = 0 ] || echo "  FAILED: $a"
    done
    echo; echo "=== analysis done; figures in plots/ (the README maps each output to the paper)"
}

# ---------------------------------------------------------------------------------------------
# the captures: one line per capture, "target|folder|build|basename|run_power_experiment args"
# ---------------------------------------------------------------------------------------------

plan() {   # targets...
    "$PY" - "$EVAL_DIR" "$@" <<'EOF'
import random
import sys
from pathlib import Path

eval_dir, targets = Path(sys.argv[1]), sys.argv[2:]
STEP = "300:2,300:20,300:2,240:5,60:20,300:5,300:20,300:2"
BURSTY = (eval_dir / "ge_schedules" / "ge_regime_s1.txt").read_text().strip()   # make_ge_schedule.py
# Controller settings, as each set of captures ran (captures.csv lists them per capture).
S = "--atc-debt-per-class 1 --atc-random-probe 1"
FIXED_PROBE = ("--no-atc-use-change-detection --atc-debt-r 0 --atc-debt-per-class 0 "
               "--atc-random-probe 0 --atc-debt-tcp 0")                # no loss debt, fixed probe cycle
DEBT = f"--no-atc-use-change-detection --atc-debt-r 250 {S} --atc-debt-tcp 0"
NO_DEBT = f"--no-atc-use-change-detection --atc-debt-r 0 {S} --atc-debt-tcp 0"
FINAL = f"--atc-use-change-detection --atc-debt-r 250 {S} --atc-debt-tcp 1"   # failed sends charged

caps = []   # (target, folder, build, protocol, L, loss or schedule, rep, extra args)


def add(target, folder, protocols, L, losses, reps, extra="", build=""):
    for proto in protocols.split():
        for loss in losses:
            for rep in reps:
                caps.append((target, folder, build, proto, L, loss, rep, extra))


R3 = (1, 2, 3)
T = "--duration 1200 --warmup 300"
add("baselines", "baselines", "udp tcp tcp_reply", 0.025, (0, 5, 10, 20), R3, T)
# grid: the controller without the debt and with the fixed probe cycle (L = 0 is plain TCP) ...
g = "grid/fixed_probe"
add("grid", g, "adp", 0.0, (0, 10), (1,), f"{T} {FIXED_PROBE}")
add("grid", g, "adp", 0.025, (0, 1, 5, 10, 20), R3, f"{T} {FIXED_PROBE}")
add("grid", g, "adp", 0.05, (0, 1, 5, 10), R3, f"{T} {FIXED_PROBE}")
add("grid", g, "adp", 0.05, (20,), (1, 2), f"{T} {FIXED_PROBE}")
add("grid", g, "adp", 0.1, (0, 1, 5, 10, 20), (1, 2), f"{T} {FIXED_PROBE}")
# ... the controller with the loss debt and the random probe, and without the debt ...
g = "grid/debt"
for L in (0.025, 0.05):
    add("grid", g, "adp", L, (0, 1), (1,), f"{T} {DEBT}")
    add("grid", g, "adp", L, (5, 10, 20), R3, f"{T} {DEBT}")
    add("grid", "grid/no_debt", "adp", L, (5, 10, 20), R3, f"{T} {NO_DEBT}")
add("grid", g, "adp", 0.1, (0, 1, 5), (1,), f"{T} {DEBT}")
add("grid", g, "adp", 0.1, (10, 20), R3, f"{T} {DEBT}")
add("grid", "grid/no_debt", "adp", 0.1, (10, 20), R3, f"{T} {NO_DEBT}")
# ... and its binding cell L = 5 %, p = 20 % with failed sends charged to the debt
add("grid", "grid/binding_cell", "adp", 0.05, (20,), R3, f"{T} {FINAL}")
add("step", "step", "adp", 0.05, (STEP,), (1, 2, 3, 4, 5), f"--warmup 300 {DEBT}")
add("mixed", "mixed", "adp", 0.01, (15,), range(1, 13),
    f"--duration 7200 --warmup 300 --mix-L 0.5,0.1,0.01 --mix-share 0.6,0.3,0.1 {DEBT}")
B = "--burst 3 --warmup 300"
add("bursty", "bursty/baselines", "udp tcp", 0.05, (BURSTY,), R3, B)
add("bursty", "bursty/controller", "adp", 0.05, (BURSTY,), R3, f"{B} {FINAL}")
add("bursty", "bursty/binding_cell", "adp", 0.05, (20,), R3, f"{B} --duration 3600 {FINAL}")
add("bursty", "bursty/binding_cell_tcp", "tcp", 0.05, (20,), R3, f"{B} --duration 3600")
add("udp_ack", "udp_ack", "udp_ack", 0.025, (0, 5, 10, 20), R3, T)
D = "--duration 1200 --warmup 60 --send-interval-ms 10000"
add("deep_sleep", "deep_sleep/power_save", "udp tcp tcp_reply", 0.05, (0,), (1,), D)
add("deep_sleep", "deep_sleep/wnm_sleep", "udp tcp tcp_reply", 0.05, (0, 10), (1,), D,
    build="DEMO_SLEEP_MODE=1")

lines = []
for target in targets:
    mine = [c for c in caps if c[0] == target]
    rng = random.Random(f"run_paper {target}")
    for rep in sorted({c[6] for c in mine}):
        rnd = [c for c in mine if c[6] == rep]
        rng.shuffle(rnd)
        lines += rnd
lines.sort(key=lambda c: c[2] != "")          # one flash per build: the default build first
for target, folder, build, proto, L, loss, rep, extra in lines:
    sched = isinstance(loss, str)
    p_label = int(loss.split(",")[0].split(":")[1]) if sched else loss
    base = f"pwr_{proto}_L{round(L * 1000):03d}_p{p_label:02d}_r{rep:02d}"
    args = (f"--protocol {proto} --L {L:g} " + (f"--schedule {loss}" if sched else f"--loss {loss}")
            + f" {extra} --replicates 1 --start-rep {rep}")
    print(f"{target}|{folder}|{build}|{base}|{' '.join(args.split())}")
EOF
}

# Prints nothing if the capture is usable, else the reason.
capture_problem() {   # manifest
    [ -f "$1" ] || { echo "no manifest written"; return; }
    "$PY" - "$1" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1]))
pushed, conf = m.get("remote_config_pushed") or {}, m.get("remote_config_confirmed")
if not conf:
    print("device did not confirm the settings it was sent"); sys.exit()
norm = lambda v: str(int(v)) if isinstance(v, bool) else str(v)
bad = [f"{k}={conf.get(k)} (sent {norm(v)})" for k, v in pushed.items()
       if k not in conf or (abs(float(conf[k]) - float(v)) > 1e-9 if isinstance(v, float) else conf[k] != norm(v))]
if bad:
    print("device confirmed different settings: " + ", ".join(bad))
EOF
}

ensure_build() {   # make variables of the build the next capture needs ("" = the default build)
    local want="ENABLE_PWR_MEAS=1${1:+ $1}"
    if [ -f "$FLASHED_INFO" ] && grep -q "(clean)" "$FLASHED_INFO" &&
       [ "$(sed -n 's/^make_vars: *//p' "$FLASHED_INFO")" = "$want" ]; then
        return 0
    fi
    echo "=== $(date '+%a %H:%M') build and flash: $want"
    # shellcheck disable=SC2086
    "$SCRIPT_DIR/build_and_flash.sh" $1 > "$OUT_ROOT/build_and_flash.log" 2>&1 \
        || { echo "build or flash failed, see $OUT_ROOT/build_and_flash.log"; return 1; }
    grep -q "(clean)" "$FLASHED_INFO" \
        || { echo "the firmware sources differ from the commit (src/); commit them first"; return 1; }
}

captures() {   # targets...
    local entry target folder build base args out n=0 attempt problem d
    mapfile -t PLAN < <(plan "$@")
    echo "Plan: ${#PLAN[@]} captures into $OUT_ROOT"
    if [ "$DRY_RUN" = 1 ]; then printf '%s\n' "${PLAN[@]}"; return 0; fi

    : "${CONSOLE_PORT:?set CONSOLE_PORT, the devkit UART (/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_...-if02)}"
    : "${PPK2_PORT:?set PPK2_PORT, the Power Profiler (/dev/serial/by-id/usb-Nordic_Semiconductor_PPK2_...-if01)}"
    : "${AP_HOST:?set AP_HOST, the access point}"
    : "${SERVER_HOST:?set SERVER_HOST, the host running server.py}"
    : "${SERVER_KEY:=$HOME/.ssh/id_ed25519}"
    local l r
    l="$(md5sum < "$SCRIPT_DIR/server.py" | cut -d' ' -f1)"
    r="$(ssh -i "$SERVER_KEY" -o BatchMode=yes root@"$SERVER_HOST" 'md5sum < ~/adapt/server.py' | cut -d' ' -f1)"
    [ "$l" = "$r" ] || { echo "the receiver's ~/adapt/server.py differs from scripts/server.py"; return 1; }
    mkdir -p "$OUT_ROOT"
    local common=(--console-port "$CONSOLE_PORT" --ppk2-port "$PPK2_PORT" --server-host "$SERVER_HOST"
                  --server-user root --server-key "$SERVER_KEY" --ap-host "$AP_HOST" --ap-key "$SERVER_KEY")

    for entry in "${PLAN[@]}"; do
        IFS='|' read -r target folder build base args <<< "$entry"
        out="$OUT_ROOT/$folder"; n=$((n + 1))
        [ -f "$out/$base.json" ] && { echo "=== [$n/${#PLAN[@]}] $folder/$base exists, skipped"; continue; }
        ensure_build "$build" || return 1
        mkdir -p "$out"
        for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
            echo "=== [$n/${#PLAN[@]}] $(date '+%a %H:%M') $target: $folder/$base (attempt $attempt)"
            if compgen -G "$out/$base.*" > /dev/null; then
                d="$out/incomplete/$(date '+%Y%m%d-%H%M%S')"; mkdir -p "$d"; mv "$out/$base".* "$d/"
                echo "      files of an interrupted capture moved to $d"
            fi
            # shellcheck disable=SC2086
            "$PY" "$SCRIPT_DIR/run_power_experiment.py" $args "${common[@]}" --out-dir "$out" < /dev/null 2>&1 \
                | grep --line-buffered -E "OK|FAIL|ERROR|!!|XX|replicates completed"
            problem="$(capture_problem "$out/$base.json")"
            if [ -z "$problem" ]; then echo "      capture OK"; break; fi
            d="$out/incomplete/$(date '+%Y%m%d-%H%M%S')"; mkdir -p "$d"; mv "$out/$base".* "$d/" 2>/dev/null
            echo "      $problem -- moved to $d"
        done
    done

    echo "=== $(date '+%a %H:%M') teardown: no loss at the access point, server.py restarted idle"
    (cd "$SCRIPT_DIR" && "$PY" - "$AP_HOST" "$SERVER_KEY" <<'EOF'
import sys
import channel_scheduler as chs
chs.AP_HOST, chs.AP_KEY = sys.argv[1], sys.argv[2]
print("set_loss_rate(0):", chs.set_loss_rate(0))
EOF
)
    ssh -i "$SERVER_KEY" -o BatchMode=yes root@"$SERVER_HOST" \
        'pkill -f "^python3 -u serve[r]\.py"; cd ~/adapt && (nohup python3 -u server.py > server_stdout.idle.log 2>&1 < /dev/null &); sleep 2; pgrep -af "^python3 -u serve[r]\.py" || echo "server.py did not start"'
}

# ---------------------------------------------------------------------------------------------

# shellcheck source=/dev/null
[ -f "$SCRIPT_DIR/lab.local.env" ] && . "$SCRIPT_DIR/lab.local.env"
targets=()
for t in "$@"; do
    case "$t" in
        -h|--help) usage ;;
        analysis|simulations) ;;
        captures) targets+=("${HW_TARGETS[@]}") ;;
        *) [[ " ${HW_TARGETS[*]} " == *" $t "* ]] || { echo "unknown target: $t"; usage 2; }
           targets+=("$t") ;;
    esac
done
[ ${#targets[@]} -gt 0 ] && { captures "${targets[@]}" || exit 1; }
for t in "$@"; do
    case "$t" in
        simulations) simulations || exit 1 ;;
        analysis) analysis || exit 1 ;;
    esac
done
echo "=== all done ($(date '+%a %H:%M'))"
