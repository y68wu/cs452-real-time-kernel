#ifndef _timer_h_
#define _timer_h_ 1

#include <stdint.h>

uint64_t timer_get_ticks(void);
uint64_t timer_get_frequency(void);
uint64_t timer_get_usec(void);

#endif
