#!/bin/sh
set -eu

if ! grep -Eq '^[[:space:]]*__bss_start[[:space:]]*=' linker.ld ||
   ! grep -Eq '^[[:space:]]*__bss_end[[:space:]]*=' linker.ld ||
   ! grep -Fq '.bss (NOLOAD)' linker.ld ||
   ! grep -Fq 'KEEP(*(.bss*))' linker.ld ||
   ! grep -Fq 'KEEP(*(.sbss*))' linker.ld ||
   ! grep -Fq '*(COMMON)' linker.ld; then
        echo "linker does not define the complete NOLOAD BSS range" >&2
        exit 1
fi

start_line=$(grep -n 'ldr x6, =__bss_start' boot.S | cut -d: -f1)
end_line=$(grep -n 'ldr x7, =__bss_end' boot.S | cut -d: -f1)
store_line=$(grep -n 'str xzr, \[x6\], #8' boot.S | cut -d: -f1)
barrier_line=$(grep -n 'dsb sy' boot.S | tail -n 1 | cut -d: -f1)
kmain_line=$(grep -n 'bl kmain' boot.S | cut -d: -f1)

for line in "$start_line" "$end_line" "$store_line" \
            "$barrier_line" "$kmain_line"; do
        if [ -z "$line" ]; then
                echo "boot BSS zero loop is incomplete" >&2
                exit 1
        fi
done

if [ "$start_line" -ge "$store_line" ] ||
   [ "$end_line" -ge "$store_line" ] ||
   [ "$store_line" -ge "$barrier_line" ] ||
   [ "$barrier_line" -ge "$kmain_line" ] ||
   ! grep -Fq 'cmp x6, x7' boot.S ||
   ! grep -Fq 'b.hs zero_bss_done' boot.S ||
   ! grep -Fq 'b zero_bss' boot.S; then
        echo "BSS is not completely zeroed before entering C" >&2
        exit 1
fi

if ! grep -Fq 'timer_wait_head = 0;' kernel.c ||
   ! grep -Fq 'timer_next_wakeup_usec = 0;' kernel.c ||
   ! grep -Fq 'io_event_wait_head[event_type] = 0;' kernel.c; then
        echo "kernel event-list roots lack defensive startup reset" >&2
        exit 1
fi

echo "validated linker-defined BSS zeroing before C and kernel event-root reset"
