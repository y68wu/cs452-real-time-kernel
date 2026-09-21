#ifndef _rpi_h_
#define _rpi_h_ 1

static char* const MMIO_BASE = (char*)0xFE000000;

void gpio_init();

#endif /* _rpi_h_ */
