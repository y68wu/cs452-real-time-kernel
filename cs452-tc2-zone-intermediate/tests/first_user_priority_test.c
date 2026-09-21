#include <setjmp.h>
#include <stddef.h>
#include <stdio.h>

#include "tasks.h"
#include "uart.h"

static jmp_buf kernel_escape;
static int observed_priority = -1;

int kmain(void);

void gpio_init(void) {
}

void uart_config_and_enable(size_t line) {
        (void)line;
}

void uart_puts(size_t line, const char *text) {
        (void)line;
        (void)text;
}

void FirstUserTask(void) {
}

void kernel_run(void (*first_task)(void), int first_priority) {
        if (first_task == FirstUserTask) {
                observed_priority = first_priority;
        }
        longjmp(kernel_escape, 1);
}

int main(void) {
        if (setjmp(kernel_escape) == 0) {
                (void)kmain();
                puts("kmain unexpectedly returned");
                return 1;
        }

#ifdef MODE_TC2
        if (observed_priority != TC2_FIRST_USER_PRIORITY ||
            observed_priority != 0) {
                printf("TC2 bootstrap priority was %d\n",
                       observed_priority);
                return 1;
        }
        puts("validated TC2 priority-0 bootstrap");
#else
        if (observed_priority != DEFAULT_FIRST_USER_PRIORITY ||
            observed_priority != 8) {
                printf("default bootstrap priority was %d\n",
                       observed_priority);
                return 1;
        }
        puts("validated unchanged non-TC2 bootstrap priority");
#endif
        return 0;
}
