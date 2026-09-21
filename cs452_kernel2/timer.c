#include "timer.h"

uint64_t timer_get_ticks(void) {
        uint64_t value;
        asm volatile("mrs %0, cntpct_el0" : "=r"(value));
        return value;
}

uint64_t timer_get_frequency(void) {
        uint64_t value;
        asm volatile("mrs %0, cntfrq_el0" : "=r"(value));
        return value;
}

uint64_t timer_get_usec(void) {
        uint64_t ticks = timer_get_ticks();
        uint64_t freq = timer_get_frequency();

        if (freq == 0) {
                return 0;
        }

        return (ticks * 1000000u) / freq;
}
