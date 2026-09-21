#include "rpi.h"
#include "spi.h"

static const char* SPI0_BASE = MMIO_BASE + 0x204000;
#define SPI0_REG(offset) (*(volatile uint32_t*)(SPI0_BASE + offset))

// SPI register offsets
static const uintptr_t SPI_CS   = 0x00;
static const uintptr_t SPI_FIFO = 0x04;
static const uintptr_t SPI_CLK  = 0x08;

// CS bit fields
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

void spi_init() {
	uint32_t ctrl = SPI0_REG(SPI_CS);
	ctrl &= ~(SPI_CS_CS_01 | SPI_CS_CS_10);    // CS = 0 (SPI0_CE0_N)
	ctrl &= ~SPI_CS_CPOL;                      // CPOL = 0
	ctrl &= ~SPI_CS_CPHA;                      // CPHA = 0
	ctrl |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX; // Clear RX and TX FIFOs
	SPI0_REG(SPI_CS) = ctrl;

	// Core clock = 500 MHz (see core_freq in https://www.raspberrypi.com/documentation/computers/config_txt.html#overclocking)
	uint32_t cdiv = 500 / 10; // Target clock is 10 MHz (MCP2515 data sheet, Page 1)
	cdiv += cdiv % 2;         // Round up to nearest even number
	SPI0_REG(SPI_CLK) = cdiv; // SCLK = Core Clock / CDIV
}

void spi_begin_transaction(void) {
	uint32_t ctrl = SPI0_REG(SPI_CS);
	ctrl |= SPI_CS_TA;                         // Transfer active
	SPI0_REG(SPI_CS) = ctrl;
}

uint8_t spi_transfer_one(uint8_t tx_byte) {
	while (!(SPI0_REG(SPI_CS) & SPI_CS_TXD));  // Wait for space in TX FIFO
	SPI0_REG(SPI_FIFO) = tx_byte;              // Write byte to TX FIFO
	while (!(SPI0_REG(SPI_CS) & SPI_CS_RXD));  // Wait for data in RX FIFO
	return SPI0_REG(SPI_FIFO);                 // Read byte from RX FIFO
}

void spi_end_transaction(void) {
	while (!(SPI0_REG(SPI_CS) & SPI_CS_DONE)); // Wait for transfer to complete
	uint32_t ctrl = SPI0_REG(SPI_CS);
	ctrl &= ~SPI_CS_TA;                        // Transfer inactive
	SPI0_REG(SPI_CS) = ctrl;
}
