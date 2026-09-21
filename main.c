#include "rpi.h"
#include "uart.h"
#include "kernel.h"
#include "tasks.h"

extern void setup_mmu(); // in mmu.S

int kmain() {
#if defined(MMU)
        setup_mmu();
#endif

        gpio_init();
        uart_config_and_enable(CONSOLE);

#ifdef MODE_TC2
        // Boot banner跟随build mode，避免TC2 image在demo时误报Part 1。
        uart_puts(CONSOLE, "\r\nCS452 Train Control Part 2\r\n");
#else
        uart_puts(CONSOLE, "\r\nCS452 Train Control Part 1\r\n");
#endif
        uart_puts(CONSOLE, "Starting kernel...\r\n");

#ifdef MODE_TC2
        /*
         * Bootstrap must outrank every service it creates. With strict
         * fixed-priority scheduling, a priority-8 parent can otherwise be
         * starved forever by the priority-1..4 clock/CAN/sensor chain before
         * it creates the dispatcher and UI.
         */
        kernel_run(FirstUserTask, TC2_FIRST_USER_PRIORITY);
#else
        kernel_run(FirstUserTask, DEFAULT_FIRST_USER_PRIORITY);
#endif

        for (;;) {
        }

        return 0;
}
