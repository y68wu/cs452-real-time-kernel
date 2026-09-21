#ifndef _spi_h_
#define _spi_h_ 1

#include <stdint.h>

void spi_init();
void spi_begin_transaction(void);
void spi_end_transaction(void);
uint8_t spi_transfer_one(uint8_t tx_byte);

#endif /* _spi_h_ */
