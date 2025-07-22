#include "py32f030x6.h"
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include "uart.h"


void _exit(int status) {
    while (1); /* Infinite loop for exit */
}


int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        uart_putc(ptr[i]);
    }
    return len;
}

int _read(int file, char *ptr, int len) {
    if (len <= 0) return 0;

    char c;
    if (uart_getc(&c)) {
        *ptr = c;
        return 1;
    }
    return -1; // EOF — buffer empty
}

void *_sbrk(int incr) {
    extern char __heap_base; /* Defined in startup.S */
    extern char __heap_limit;
    static char *heap_end = &__heap_base;

    char *prev_heap_end = heap_end;
    if (heap_end + incr > &__heap_limit) {
        errno = ENOMEM;
        return (void *)-1;
    }
    heap_end += incr;
    return (void *)prev_heap_end;
}

int _close(int file) {
    return -1; /* Not implemented */
}

int _lseek(int file, int ptr, int dir) {
    return -1; /* Not implemented */
}

int _fstat(int file, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file) {
    return 1;
}
