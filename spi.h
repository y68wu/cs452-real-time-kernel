#ifndef _spi_h_
#define _spi_h_ 1

#include <stdint.h>

#ifndef SPI_STATUS_POLL_LIMIT
#define SPI_STATUS_POLL_LIMIT 100000u
#endif

#define SPI_SUCCESS             0
#define SPI_ERROR_ARGUMENT     -1
#define SPI_ERROR_TX_TIMEOUT   -2
#define SPI_ERROR_RX_TIMEOUT   -3
#define SPI_ERROR_DONE_TIMEOUT -4

void spi_init(void);
void spi_begin_transaction(void);
void spi_abort_transaction(void);
int spi_end_transaction(void);
int spi_transfer_one(uint8_t tx_byte, uint8_t *rx_byte);

#endif
