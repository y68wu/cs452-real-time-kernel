#!/bin/sh
set -eu

makefile=Makefile
asm_file=syscall_asm.S
layout_file=context_frame_layout.h
memory_file=freestanding_memory.c

if ! grep -Eq '^REGISTERFLAGS:=-mgeneral-regs-only$' "$makefile"; then
        echo "all target builds must use -mgeneral-regs-only" >&2
        exit 1
fi
if ! grep -Eq '^SCALARFLAGS:=-fno-tree-vectorize -fno-tree-slp-vectorize -fno-tree-loop-distribute-patterns$' "$makefile" ||
   ! grep -Eq 'CFLAGS:.*\$\(REGISTERFLAGS\).*\$\(SCALARFLAGS\)' "$makefile"; then
        echo "all target builds must disable compiler-generated vector operations" >&2
        exit 1
fi
for primitive in memset memcpy memmove; do
        if ! grep -Eq "^void \\*${primitive}\\(" "$memory_file"; then
                echo "missing scalar freestanding ${primitive}" >&2
                exit 1
        fi
done

register_pattern='(^|[[:space:],]|\{|\[)((v|z|[qdsbh])([0-9]|[12][0-9]|3[01])|p([0-9]|1[0-5])|ffr|fpcr|fpsr)(\.|[[:space:],]|$)'
if grep -Eiq "$register_pattern" "$asm_file"; then
        echo "context assembly unexpectedly uses SIMD/FP registers" >&2
        exit 1
fi

for sample in \
        'ld1 {v0.16b}, [x0]' \
        'stp q31, q0, [sp]' \
        'fmov d5, x0' \
        'ptrue p15.b' \
        'rdvl x0, #1 // z31' \
        'mrs x0, fpcr'; do
        if ! printf '%s\n' "$sample" | grep -Eiq "$register_pattern"; then
                echo "SIMD/FP register gate missed sample: $sample" >&2
                exit 1
        fi
done

for sample in \
        'b4000100 cbz x0, target' \
        '.word 0x30d50800' \
        'add x0, x1, x2'; do
        if printf '%s\n' "$sample" | grep -Eiq "$register_pattern"; then
                echo "SIMD/FP register gate rejected integer sample: $sample" >&2
                exit 1
        fi
done

if ! grep -Eq '^#define CONTEXT_FRAME_SIZE 272$' "$layout_file"; then
        echo "general-register context frame is not 272 bytes" >&2
        exit 1
fi

for operation in stp ldp; do
        register=0
        while [ "$register" -lt 30 ]; do
                next=$((register + 1))
                offset=$((register * 8))
                if ! grep -Eq "^[[:space:]]*${operation}[[:space:]]+x${register},[[:space:]]+x${next},[[:space:]]+\\[sp,[[:space:]]*#${offset}\\]" "$asm_file"; then
                        echo "missing ${operation} for x${register}/x${next} at offset ${offset}" >&2
                        exit 1
                fi
                register=$((register + 2))
        done
done

if ! grep -Eq '^[[:space:]]*str[[:space:]]+x30,[[:space:]]+\[sp,[[:space:]]*#240\]' "$asm_file" ||
   ! grep -Eq '^[[:space:]]*ldr[[:space:]]+x30,[[:space:]]+\[sp,[[:space:]]*#240\]' "$asm_file"; then
        echo "x30 context save/restore is missing or misplaced" >&2
        exit 1
fi

for offset_name in CONTEXT_ELR_OFFSET CONTEXT_SPSR_OFFSET; do
        if [ "$(grep -c "#${offset_name}" "$asm_file")" -ne 2 ]; then
                echo "${offset_name} is not used once for save and once for restore" >&2
                exit 1
        fi
done

if ! grep -Eq '^[[:space:]]*sub[[:space:]]+sp,[[:space:]]+sp,[[:space:]]+#CONTEXT_FRAME_SIZE' "$asm_file" ||
   ! grep -Eq '^[[:space:]]*add[[:space:]]+sp,[[:space:]]+sp,[[:space:]]+#CONTEXT_FRAME_SIZE' "$asm_file"; then
        echo "context frame stack allocation/deallocation is incomplete" >&2
        exit 1
fi

echo "validated general-register-only target policy and context source"
