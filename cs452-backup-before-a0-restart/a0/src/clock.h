#ifndef CLOCK_H
#define CLOCK_H

void clock_init(void);
void clock_poll(void);

unsigned int clock_minutes(void);
unsigned int clock_seconds(void);
unsigned int clock_tenths(void);

#endif
