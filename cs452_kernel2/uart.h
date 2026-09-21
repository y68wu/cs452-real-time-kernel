#ifndef _uart_h_
#define _uart_h_ 1

#include <stddef.h>

#define CONSOLE 0

void uart_config_and_enable(size_t line);
char uart_getc(size_t line);
void uart_putc(size_t line, char c);
void uart_putl(size_t line, const char *buf, size_t blen);
void uart_puts(size_t line, const char *buf);
void uart_printf(size_t line, const char *fmt, ...);

#endif /* uart.h */
