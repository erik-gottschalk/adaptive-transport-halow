#!/usr/bin/env bash
# The controller's footprint as the paper quotes it (Section IV-A): the controller's code linked
# into the firmware and the state of one instance on the target; also the whole application's
# flash and RAM.
#
#   make -C ../targets/mm-mm8108-ekh05 ENABLE_PWR_MEAS=1     # build first (Arm GNU toolchain on PATH)
#   ./footprint.sh
#
# Expected: 3600 B of controller code (3.6 KB), 440 B of state,
# ~828 KB flash and ~262 KB RAM (data + bss) for the application.
set -euo pipefail
cd "$(dirname "$0")/.."
BUILD=targets/mm-mm8108-ekh05/build
ELF=$BUILD/adaptive_transport.elf
OBJ=$BUILD/src/adaptive_transport_controller.o
[ -f "$ELF" ] && [ -f "$OBJ" ] || { echo "build the firmware first (see the header)"; exit 1; }

# controller code in the linked firmware: the controller object's text/rodata symbols the linker kept
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
arm-none-eabi-nm -S -t d --defined-only "$OBJ" | awk '$3 ~ /[TtRr]/ {print $4}' > "$tmp/syms"
arm-none-eabi-nm -S -t d --defined-only "$ELF" |
    awk 'NR == FNR {want[$1] = 1; next} ($4 in want) && $3 ~ /[TtRr]/ {s += $2; n++}
         END {printf "controller code linked: %d symbols, %d B\n", n, s}' "$tmp/syms" -

# state per instance: sizeof on the target, read back from a compiled constant
printf '#include <stddef.h>\n#include "adaptive_transport_controller.h"\nconst size_t atc_state_size = sizeof(struct adaptive_transport_controller);\n' > "$tmp/sz.c"
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -O2 -std=gnu11 -Iscripts/eval/sim/stub -Isrc -c "$tmp/sz.c" -o "$tmp/sz.o"
size=$(arm-none-eabi-objdump -s -j .rodata "$tmp/sz.o" | awk '/^ 0000/ {print $2}')
echo "controller state per instance: $(( 16#${size:6:2}${size:4:2}${size:2:2}${size:0:2} )) B"

arm-none-eabi-size "$ELF" | awk 'NR == 2 {printf "application: %d B flash (text), %d B RAM (data + bss)\n", $1, $2 + $3}'
