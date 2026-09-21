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

        uart_puts(CONSOLE, "\r\nCS452 Train Control Part 1\r\n");
        uart_puts(CONSOLE, "Starting kernel...\r\n");

        kernel_run(FirstUserTask, 8);

        for (;;) {
        }

        return 0;
}
