#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>

void uart_init(void);
void uart_putc(char c);
void uart_write(const char *s);
int  uart_tx_busy(void);
int  uart_getc(char *c);  // Returns 1 if a character is received, 0 if the buffer is empty

#endif // UART_H
