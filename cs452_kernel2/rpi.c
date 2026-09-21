#include <stdint.h>
#include "rpi.h"

static char* const GPIO_BASE = (char*)(MMIO_BASE + 0x200000);

#define GPFSEL_REG(reg)             (*(volatile uint32_t*)(GPIO_BASE + reg * 4))
#define GPIO_PUP_PDN_CNTRL_REG(reg) (*(volatile uint32_t*)(GPIO_BASE + 0xe4 + reg * 4))

// function control settings for GPIO pins
static const uint32_t GPIO_INPUT  = 0x00;
static const uint32_t GPIO_OUTPUT = 0x01;
static const uint32_t GPIO_ALTFN0 = 0x04;
static const uint32_t GPIO_ALTFN1 = 0x05;
static const uint32_t GPIO_ALTFN2 = 0x06;
static const uint32_t GPIO_ALTFN3 = 0x07;
static const uint32_t GPIO_ALTFN4 = 0x03;
static const uint32_t GPIO_ALTFN5 = 0x02;

// pup/pdn resistor settings for GPIO pins
static const uint32_t GPIO_NONE = 0x00;
static const uint32_t GPIO_PUP  = 0x01;
static const uint32_t GPIO_PDP  = 0x02;

static void setup_gpio(uint32_t pin, uint32_t setting, uint32_t resistor) {
	uint32_t reg   =  pin / 10;
	uint32_t shift = (pin % 10) * 3;
	uint32_t status = GPFSEL_REG(reg);    // read status
	status &= ~(7u << shift);             // clear bits
	status |=  (setting << shift);        // set bits
	GPFSEL_REG(reg) = status;

	reg   =  pin / 16;
	shift = (pin % 16) * 2;
	status = GPIO_PUP_PDN_CNTRL_REG(reg); // read status
	status &= ~(3u << shift);             // clear bits
	status |=  (resistor << shift);       // set bits
	GPIO_PUP_PDN_CNTRL_REG(reg) = status; // write back
}

// GPIO pins 14 & 15 already configured by boot loader, but redo for clarity.
void gpio_init() {
	setup_gpio( 8, GPIO_ALTFN0, GPIO_NONE); // SPI0_CE0_N
	setup_gpio( 9, GPIO_ALTFN0, GPIO_NONE); // SPI0_MISO
	setup_gpio(10, GPIO_ALTFN0, GPIO_NONE); // SPI0_MOSI
	setup_gpio(11, GPIO_ALTFN0, GPIO_NONE); // SPI0_SCLK

	setup_gpio(14, GPIO_ALTFN0, GPIO_NONE); // UART TXD0
	setup_gpio(15, GPIO_ALTFN0, GPIO_NONE); // UART RXD0
}
