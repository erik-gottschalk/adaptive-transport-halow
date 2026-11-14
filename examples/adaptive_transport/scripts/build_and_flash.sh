#!/usr/bin/env bash
# Clean-build the adaptive_transport firmware for the mm-mm8108-ekh05 and flash it with OpenOCD --
# the same steps as doing it by hand:
#   make clean && make -j$(nproc) ENABLE_PWR_MEAS=1     (in targets/mm-mm8108-ekh05)
#   openocd -c "set MM_PROGRAM 1" -f src/platforms/mm-mm8108-ekh05/openocd.cfg \
#       -c "program <elf> verify reset exit"          (in framework/)
#
# Usage:
#   ./build_and_flash.sh                        clean build with ENABLE_PWR_MEAS=1, then flash
#   ./build_and_flash.sh --build-only           same, but don't flash
#   ./build_and_flash.sh --flash-only           flash the existing build/adaptive_transport.elf
#   ./build_and_flash.sh DEMO_SLEEP_MODE=1      extra make variables are passed through (here: the
#                                               WNM-sleep build of the deep-sleep captures)
#   ./build_and_flash.sh ENABLE_PWR_MEAS=0      overrides the default
#
# Always a clean build: the Makefile doesn't track make-variable changes (ENABLE_PWR_MEAS,
# DISABLE_UART_LOG, ...) as dependencies, so an incremental build after changing one would
# silently keep the old setting in already-built objects.
#
# Writes build/build_info.txt next to the ELF (git commit, uncommitted-change status, make
# variables, the source's DEMO_* flags) and, after a successful flash, copies it to
# build/flashed_info.txt -- a record of what is actually on the board, since
# run_power_experiment.py never builds or flashes itself.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"                 # examples/adaptive_transport/scripts
EXAMPLE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"                 # examples/adaptive_transport
REPO_ROOT="$(cd "$EXAMPLE_DIR/../.." && pwd)"
TARGET_DIR="$EXAMPLE_DIR/targets/mm-mm8108-ekh05"
FRAMEWORK_DIR="$REPO_ROOT/framework"
ELF="$TARGET_DIR/build/adaptive_transport.elf"
BUILD_INFO="$TARGET_DIR/build/build_info.txt"
FLASHED_INFO="$TARGET_DIR/build/flashed_info.txt"

do_build=1
do_flash=1
make_vars=()
for arg in "$@"; do
    case "$arg" in
        --build-only) do_flash=0 ;;
        --flash-only) do_build=0 ;;
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *=*) make_vars+=("$arg") ;;
        *) echo "unknown argument: $arg (see --help)" >&2; exit 2 ;;
    esac
done
if [[ " ${make_vars[*]-} " != *" ENABLE_PWR_MEAS="* ]]; then
    make_vars=("ENABLE_PWR_MEAS=1" "${make_vars[@]+"${make_vars[@]}"}")
fi

# The Morse toolchain and OpenOCD are only added to PATH by the interactive shell setup, so fall
# back to their install location when this runs from a non-interactive shell.
for tool_glob in "/opt/morse/xpack-openocd-*/bin" "/opt/morse/xpack-arm-none-eabi-gcc-*/bin"; do
    for dir in $tool_glob; do
        [[ -d "$dir" && ":$PATH:" != *":$dir:"* ]] && PATH="$dir:$PATH"
    done
done
export PATH

step() { printf '\n==> %s\n' "$*"; }

if (( do_build )); then
    git_commit="$(git -C "$REPO_ROOT" rev-parse --short HEAD)"
    if git -C "$REPO_ROOT" diff --quiet HEAD -- "$EXAMPLE_DIR/src"; then
        git_state="clean"
    else
        git_state="dirty"
        echo "WARNING: examples/adaptive_transport/src has uncommitted changes -- this build won't"
        echo "         correspond to a commit (runs will record git_dirty=dirty)."
    fi

    step "clean build (${make_vars[*]})"
    make -C "$TARGET_DIR" clean >/dev/null
    make -C "$TARGET_DIR" -j"$(nproc)" "${make_vars[@]}"

    {
        echo "built:      $(date '+%Y-%m-%d %H:%M:%S %Z')"
        echo "git_commit: $git_commit ($git_state)"
        echo "make_vars:  ${make_vars[*]}"
        echo "elf_md5:    $(md5sum "$ELF" | cut -d' ' -f1)"
        echo "source flags:"
        grep -E '^#define (DEMO_[A-Z_]+|TCP_REPLY_[A-Z_]+_MS) ' "$EXAMPLE_DIR/src/adaptive_transport.c" \
            | sed 's/^#define /  /'
    } > "$BUILD_INFO"
fi

if (( do_flash )); then
    [[ -f "$ELF" ]] || { echo "no firmware at $ELF -- build first" >&2; exit 1; }
    command -v openocd >/dev/null || { echo "openocd not found (looked in PATH and /opt/morse)" >&2; exit 1; }

    step "flashing $(basename "$ELF")"
    # OpenOCD's config uses paths relative to framework/, hence the cd.
    if ! (cd "$FRAMEWORK_DIR" && openocd -c "set MM_PROGRAM 1" \
            -f src/platforms/mm-mm8108-ekh05/openocd.cfg \
            -c "program $ELF verify reset exit"); then
        echo >&2
        echo "Flashing failed. If OpenOCD couldn't find the ST-LINK, make sure it is attached to" >&2
        echo "WSL (Windows: usbipd list / usbipd attach --wsl --busid <id>)." >&2
        exit 1
    fi

    if [[ -f "$BUILD_INFO" ]]; then
        { cat "$BUILD_INFO"; echo "flashed:    $(date '+%Y-%m-%d %H:%M:%S %Z')"; } > "$FLASHED_INFO"
    fi
fi

step "done"
if (( do_flash )); then
    cat "$FLASHED_INFO" 2>/dev/null || true
else
    cat "$BUILD_INFO"
fi
