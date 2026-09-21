#ifndef _train_h_
#define _train_h_ 1

#include <stdint.h>

void train_init(void);

/* Called once per polling-loop iteration. Sends queued CAN frames and completes delayed reversals. */
void train_poll(uint32_t now_us);

/* Queue train speed command. speed is assignment speed 0..14. Returns 1 on success. */
int train_set_speed(unsigned int train, unsigned int speed);

/* Queue stop now, then reverse after a delay, then restore previous speed. Returns 1 on success. */
int train_reverse(unsigned int train, uint32_t now_us);

/* Queue switch command. direction should be 'S' or 'C'. Returns 1 on success. */
int train_switch(unsigned int sw, char direction);

#endif
