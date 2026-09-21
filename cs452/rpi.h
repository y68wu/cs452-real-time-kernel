#ifndef _rpi_h_
#define _rpi_h_ 1

#include <stdint.h>

static char* const MMIO_BASE = (char*)0xFE000000;

void gpio_init(void);

uint64_t timer_get_usec(void);
uint32_t timer_get_usec_low(void);

#endif /* _rpi_h_ */
