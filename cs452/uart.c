#include <stdarg.h>
#include <stdint.h>
#include "uart.h"
#include "util.h"
#include "rpi.h"

static char* const UART_BASE = (char*)(MMIO_BASE + 0x201000);
#define UART_REG(line, offset) (*(volatile uint32_t*)(UART_BASE + line * 0x200 + offset))

// UART register offsets
static const uint32_t UART_DR   = 0x00;
static const uint32_t UART_FR   = 0x18;
static const uint32_t UART_IBRD = 0x24;
static const uint32_t UART_FBRD = 0x28;
static const uint32_t UART_LCRH = 0x2c;
static const uint32_t UART_CR   = 0x30;

// masks for specific fields in the UART registers
static const uint32_t UART_FR_RXFE = 0x10;
static const uint32_t UART_FR_TXFF = 0x20;

static const uint32_t UART_CR_UARTEN =   0x01;
static const uint32_t UART_CR_TXE    =  0x100;
static const uint32_t UART_CR_RXE    =  0x200;

static const uint32_t UART_LCRH_FEN       = 0x10;
static const uint32_t UART_LCRH_WLEN_LOW  = 0x20;
static const uint32_t UART_LCRH_WLEN_HIGH = 0x40;

// Configure the line properties of a UART and ensure that it is enabled.
void uart_config_and_enable(size_t line) {
        uint32_t baud_ival, baud_fval;
        uint32_t flag = UART_LCRH_FEN;

        switch (line) {
                case CONSOLE:
                        baud_ival = 26;
                        baud_fval = 2;
                        break;
                default:
                        return;
        }

        uint32_t cr_state = UART_REG(line, UART_CR);
        UART_REG(line, UART_CR) = cr_state & ~UART_CR_UARTEN;

        UART_REG(line, UART_IBRD) = baud_ival;
        UART_REG(line, UART_FBRD) = baud_fval;
        UART_REG(line, UART_LCRH) = UART_LCRH_WLEN_HIGH | UART_LCRH_WLEN_LOW | flag;

        UART_REG(line, UART_CR) = cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

int uart_can_read(size_t line) {
        return !(UART_REG(line, UART_FR) & UART_FR_RXFE);
}

int uart_can_write(size_t line) {
        return !(UART_REG(line, UART_FR) & UART_FR_TXFF);
}

int uart_try_getc(size_t line, char *c) {
        if (!uart_can_read(line)) return 0;
        *c = (char)UART_REG(line, UART_DR);
        return 1;
}

int uart_try_putc(size_t line, char c) {
        if (!uart_can_write(line)) return 0;
        UART_REG(line, UART_DR) = (uint32_t)c;
        return 1;
}

char uart_getc(size_t line) {
        char c;
        while (!uart_try_getc(line, &c));
        return c;
}

void uart_putc(size_t line, char c) {
        while (!uart_try_putc(line, c));
}

void uart_putl(size_t line, const char* buf, size_t blen) {
        for (size_t i = 0; i < blen; i++) {
                uart_putc(line, *(buf+i));
        }
}

void uart_puts(size_t line, const char* buf) {
        while (*buf) {
                uart_putc(line, *buf);
                buf++;
        }
}

void uart_printf(size_t line, const char *fmt, ... ) {
        va_list va;
        char ch, buf[12];

        va_start(va, fmt);
        while ((ch = *(fmt++))) {
                if (ch != '%') {
                        uart_putc(line, ch);
                } else {
                        ch = *(fmt++);
                        switch(ch) {
                        case 'u':
                                ui2a(va_arg(va, unsigned int), 10, buf);
                                uart_puts(line, buf);
                                break;
                        case 'd':
                                i2a(va_arg(va, int), buf);
                                uart_puts(line, buf);
                                break;
                        case 'x':
                                ui2a(va_arg(va, unsigned int), 16, buf);
                                uart_puts(line, buf);
                                break;
                        case 's':
                                uart_puts(line, va_arg(va, char*));
                                break;
                        case 'c':
                                uart_putc(line, va_arg(va, int));
                                break;
                        case '%':
                                uart_putc(line, ch);
                                break;
                        case '\0':
                                va_end(va);
                                return;
                        }
                }
        }
        va_end(va);
}
