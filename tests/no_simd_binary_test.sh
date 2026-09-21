#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
        echo "usage: $0 <objdump> <elf>" >&2
        exit 2
fi

objdump=$1
elf=$2
# Hide raw instruction bytes so byte pairs such as "d5" cannot be mistaken
# for a scalar FP register token.
if ! raw_disassembly=$("$objdump" -d --no-show-raw-insn "$elf"); then
        echo "ERROR: failed to disassemble linked image" >&2
        exit 1
fi
disassembly=$(printf '%s\n' "$raw_disassembly" |
        grep -Ev '[[:space:]]\.word[[:space:]]' || true)
register_pattern='(^|[[:space:],]|\{|\[)((v|z|[qdsbh])([0-9]|[12][0-9]|3[01])|p([0-9]|1[0-5])|ffr|fpcr|fpsr)(\.|[[:space:],]|$)'

if printf '%s\n' "$disassembly" |
        grep -Eiq "$register_pattern"; then
        echo "ERROR: linked image uses SIMD/FP registers despite -mgeneral-regs-only" >&2
        printf '%s\n' "$disassembly" |
                grep -Ei "$register_pattern" |
                head -n 10 >&2
        exit 1
fi

echo "Verified linked image uses general-purpose registers only"
