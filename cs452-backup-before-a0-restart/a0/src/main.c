#include "clock.h"
#include "command.h"
#include "display.h"
#include "train.h"
#include "sensor.h"

int main(void) {
    clock_init();
    command_init();
    display_init();
    train_init();
    sensor_init();

    /*
     * Assignment 0 should eventually run as a continuous polling loop:
     *
     * while (1) {
     *     clock_poll();
     *     command_poll();
     *     train_poll();
     *     sensor_poll();
     *     display_poll();
     * }
     *
     * The finite loop below is only a host-build skeleton so that the
     * repository can compile before hardware-specific code is connected.
     */
    for (int i = 0; i < 1; ++i) {
        clock_poll();
        command_poll();
        train_poll();
        sensor_poll();
        display_poll();
    }

    return 0;
}
