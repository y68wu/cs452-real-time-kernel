#include "train.h"

#include <stdio.h>

#define MAX_TRAINS 128
#define MAX_SWITCHES 256

static int train_speeds[MAX_TRAINS];
static char switch_positions[MAX_SWITCHES];

static int valid_train(int train_number) {
    return train_number > 0 && train_number < MAX_TRAINS;
}

static int valid_switch(int switch_number) {
    return switch_number > 0 && switch_number < MAX_SWITCHES;
}

void train_init(void) {
    for (int i = 0; i < MAX_TRAINS; ++i) {
        train_speeds[i] = 0;
    }

    for (int i = 0; i < MAX_SWITCHES; ++i) {
        switch_positions[i] = '?';
    }
}

void train_poll(void) {
    /*
     * TODO for hardware version:
     *
     * Poll the CAN bus.
     * Read train hardware responses.
     * Update any cached train, switch, or sensor state.
     */
}

void train_set_speed(int train_number, int speed) {
    if (!valid_train(train_number)) {
        printf("Invalid train number: %d\n", train_number);
        return;
    }

    train_speeds[train_number] = speed;

    /*
     * TODO:
     * Send train speed command through CAN bus.
     */
    printf("Train %d speed set to %d\n", train_number, speed);
}

void train_reverse(int train_number) {
    int previous_speed;

    if (!valid_train(train_number)) {
        printf("Invalid train number: %d\n", train_number);
        return;
    }

    previous_speed = train_speeds[train_number];

    /*
     * Required reverse sequence:
     *
     * 1. Set train speed to 0.
     * 2. Wait until the train stops.
     * 3. Send the reverse direction command.
     * 4. Restore the previous speed in the reverse direction.
     *
     * TODO:
     * Replace this skeleton with the real hardware sequence.
     */
    train_set_speed(train_number, 0);
    printf("Reverse command for train %d\n", train_number);
    train_set_speed(train_number, previous_speed);
}

void switch_set_position(int switch_number, char direction) {
    if (!valid_switch(switch_number)) {
        printf("Invalid switch number: %d\n", switch_number);
        return;
    }

    if (direction != 'S' && direction != 'C') {
        printf("Invalid switch direction: %c\n", direction);
        return;
    }

    switch_positions[switch_number] = direction;

    /*
     * TODO:
     * Send switch command through CAN bus.
     */
    printf("Switch %d set to %c\n", switch_number, direction);
}

int train_last_speed(int train_number) {
    if (!valid_train(train_number)) {
        return 0;
    }

    return train_speeds[train_number];
}

char switch_position(int switch_number) {
    if (!valid_switch(switch_number)) {
        return '?';
    }

    return switch_positions[switch_number];
}
