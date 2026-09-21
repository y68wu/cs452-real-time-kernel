#include "command.h"
#include "train.h"

#include <stdio.h>
#include <string.h>

void command_init(void) {
    /*
     * TODO:
     * Initialize the command buffer used for UART input.
     */
}

void command_poll(void) {
    /*
     * TODO for hardware version:
     *
     * Poll UART0 input.
     * Add typed characters to a command buffer.
     * When Enter is pressed, call command_handle_line().
     */
}

void command_handle_line(const char *line) {
    char op[8];
    int a;
    int b;
    char direction;

    if (line == NULL) {
        return;
    }

    if (sscanf(line, "%7s", op) != 1) {
        return;
    }

    if (strcmp(op, "tr") == 0) {
        if (sscanf(line, "%7s %d %d", op, &a, &b) == 3) {
            train_set_speed(a, b);
        }
    } else if (strcmp(op, "rv") == 0) {
        if (sscanf(line, "%7s %d", op, &a) == 2) {
            train_reverse(a);
        }
    } else if (strcmp(op, "sw") == 0) {
        if (sscanf(line, "%7s %d %c", op, &a, &direction) == 3) {
            switch_set_position(a, direction);
        }
    } else if (strcmp(op, "q") == 0) {
        /*
         * TODO:
         * Return to boot loader / reboot in the course environment.
         */
        printf("Quit command received.\n");
    } else {
        printf("Unknown command: %s\n", op);
    }
}
