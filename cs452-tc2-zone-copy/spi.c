#include <stdint.h>
#ifndef SPI_TEST_BACKEND
#include "rpi.h"
#endif
#include "spi.h"

#if !defined(SPI_READ_CS) || !defined(SPI_WRITE_CS) || \
    !defined(SPI_READ_FIFO) || !defined(SPI_WRITE_FIFO) || \
    !defined(SPI_WRITE_CLK)
static const char* SPI0_BASE = MMIO_BASE + 0x204000;
#define SPI0_REG(offset) (*(volatile uint32_t*)(SPI0_BASE + offset))

static const uintptr_t SPI_CS   = 0x00;
static const uintptr_t SPI_FIFO = 0x04;
static const uintptr_t SPI_CLK  = 0x08;
#endif

#ifndef SPI_READ_CS
#define SPI_READ_CS() SPI0_REG(SPI_CS)
#endif
#ifndef SPI_WRITE_CS
#define SPI_WRITE_CS(value) (SPI0_REG(SPI_CS) = (value))
#endif
#ifndef SPI_READ_FIFO
#define SPI_READ_FIFO() SPI0_REG(SPI_FIFO)
#endif
#ifndef SPI_WRITE_FIFO
#define SPI_WRITE_FIFO(value) (SPI0_REG(SPI_FIFO) = (value))
#endif
#ifndef SPI_WRITE_CLK
#define SPI_WRITE_CLK(value) (SPI0_REG(SPI_CLK) = (value))
#endif

static const uint32_t SPI_CS_TXD      = 0x00040000;
static const uint32_t SPI_CS_RXD      = 0x00020000;
static const uint32_t SPI_CS_DONE     = 0x00010000;
static const uint32_t SPI_CS_TA       = 0x00000080;
static const uint32_t SPI_CS_CLEAR_RX = 0x00000020;
static const uint32_t SPI_CS_CLEAR_TX = 0x00000010;
static const uint32_t SPI_CS_CPOL     = 0x00000008;
static const uint32_t SPI_CS_CPHA     = 0x00000004;
static const uint32_t SPI_CS_CS_10    = 0x00000002;
static const uint32_t SPI_CS_CS_01    = 0x00000001;

static int spi_wait_for_status(uint32_t mask) {
        for (unsigned int i = 0; i < SPI_STATUS_POLL_LIMIT; ++i) {
                if ((SPI_READ_CS() & mask) == mask) {
                        return SPI_SUCCESS;
                }
        }
        return -1;
}

static void spi_close_transaction(int clear_fifos) {
        uint32_t ctrl = SPI_READ_CS();
        ctrl &= ~SPI_CS_TA;
        if (clear_fifos) {
                ctrl |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
        }
        SPI_WRITE_CS(ctrl);
}

void spi_init(void) {
        uint32_t ctrl = SPI_READ_CS();

        ctrl &= ~(SPI_CS_CS_01 | SPI_CS_CS_10);
        ctrl &= ~SPI_CS_CPOL;
        ctrl &= ~SPI_CS_CPHA;
        ctrl |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
        SPI_WRITE_CS(ctrl);

        SPI_WRITE_CLK(50);
}

void spi_begin_transaction(void) {
        uint32_t ctrl = SPI_READ_CS();
        ctrl |= SPI_CS_TA | SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
        SPI_WRITE_CS(ctrl);
}

void spi_abort_transaction(void) {
        spi_close_transaction(1);
}

int spi_transfer_one(uint8_t tx_byte, uint8_t *rx_byte) {
        if (!rx_byte) {
                spi_abort_transaction();
                return SPI_ERROR_ARGUMENT;
        }

        if (spi_wait_for_status(SPI_CS_TXD) != SPI_SUCCESS) {
                spi_abort_transaction();
                return SPI_ERROR_TX_TIMEOUT;
        }

        SPI_WRITE_FIFO(tx_byte);

        if (spi_wait_for_status(SPI_CS_RXD) != SPI_SUCCESS) {
                spi_abort_transaction();
                return SPI_ERROR_RX_TIMEOUT;
        }

        *rx_byte = (uint8_t)SPI_READ_FIFO();
        return SPI_SUCCESS;
}

int spi_end_transaction(void) {
        if (spi_wait_for_status(SPI_CS_DONE) != SPI_SUCCESS) {
                spi_abort_transaction();
                return SPI_ERROR_DONE_TIMEOUT;
        }

        spi_close_transaction(0);
        return SPI_SUCCESS;
}
