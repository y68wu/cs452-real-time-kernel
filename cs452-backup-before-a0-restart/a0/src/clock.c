#include "clock.h"

static unsigned int minutes = 0;
static unsigned int seconds = 0;
static unsigned int tenths = 0;

void clock_init(void) {
    minutes = 0;
    seconds = 0;
    tenths = 0;
}

void clock_poll(void) {
    /*
     * TODO for hardware version:
     *
     * Read the Raspberry Pi built-in system timer.
     * Update the clock whenever at least 100 ms has elapsed.
     *
     * Important:
     * - Do not use interrupts.
     * - Do not use artificial delays to advance the clock.
     * - If the polling loop is slow, update using elapsed time so that
     *   missed tenths of seconds are caught up correctly.
     */

    tenths += 1;

    if (tenths >= 10) {
        tenths = 0;
        seconds += 1;
    }

    if (seconds >= 60) {
        seconds = 0;
        minutes += 1;
    }
}

unsigned int clock_minutes(void) {
    return minutes;
}

unsigned int clock_seconds(void) {
    return seconds;
}

unsigned int clock_tenths(void) {
    return tenths;
}
