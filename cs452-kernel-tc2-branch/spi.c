#include <stdint.h>
#include "rpi.h"
#include "spi.h"

static const char* SPI0_BASE = MMIO_BASE + 0x204000;
#define SPI0_REG(offset) (*(volatile uint32_t*)(SPI0_BASE + offset))

static const uintptr_t SPI_CS   = 0x00;
static const uintptr_t SPI_FIFO = 0x04;
static const uintptr_t SPI_CLK  = 0x08;

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

void spi_init(void) {
        uint32_t ctrl = SPI0_REG(SPI_CS);

        ctrl &= ~(SPI_CS_CS_01 | SPI_CS_CS_10);
        ctrl &= ~SPI_CS_CPOL;
        ctrl &= ~SPI_CS_CPHA;
        ctrl |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
        SPI0_REG(SPI_CS) = ctrl;

        SPI0_REG(SPI_CLK) = 50;
}

void spi_begin_transaction(void) {
        uint32_t ctrl = SPI0_REG(SPI_CS);
        ctrl |= SPI_CS_TA | SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX;
        SPI0_REG(SPI_CS) = ctrl;
}

uint8_t spi_transfer_one(uint8_t tx_byte) {
        while (!(SPI0_REG(SPI_CS) & SPI_CS_TXD)) {
        }

        SPI0_REG(SPI_FIFO) = tx_byte;

        while (!(SPI0_REG(SPI_CS) & SPI_CS_RXD)) {
        }

        return (uint8_t)SPI0_REG(SPI_FIFO);
}

void spi_end_transaction(void) {
        while (!(SPI0_REG(SPI_CS) & SPI_CS_DONE)) {
        }

        uint32_t ctrl = SPI0_REG(SPI_CS);
        ctrl &= ~SPI_CS_TA;
        SPI0_REG(SPI_CS) = ctrl;
}
