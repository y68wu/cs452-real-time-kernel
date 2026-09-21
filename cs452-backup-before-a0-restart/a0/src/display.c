#include "display.h"
#include "clock.h"

#include <stdio.h>

static int display_ready = 0;

void display_init(void) {
    /*
     * TODO for hardware version:
     *
     * Use ASCII escape sequences to clear the terminal and draw the static
     * layout for:
     *
     * 1. current time,
     * 2. switch position table,
     * 3. recent triggered sensors,
     * 4. command prompt.
     */
    display_ready = 1;
}

void display_poll(void) {
    if (!display_ready) {
        return;
    }

    /*
     * TODO:
     * Use cursor addressing instead of printing endless lines.
     */
    printf("Time: %02u:%02u.%u\n",
           clock_minutes(),
           clock_seconds(),
           clock_tenths());
}
