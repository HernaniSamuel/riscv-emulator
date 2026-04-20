// basic_io.c — bare metal RV32I I/O test
// Tests both UART TX (write) and RX (read)

#define UART ((volatile unsigned char *)0x10000000)

void RVputchar(char c) { *UART = c; }

void RVputs(const char *s) {
    while (*s) RVputchar(*s++);
}

void RVprint_int(int x) {
    char buf[16];
    int i = 0;
    if (x == 0) { RVputchar('0'); return; }
    if (x < 0)  { RVputchar('-'); x = -x; }
    while (x > 0) { buf[i++] = '0' + (x % 10); x /= 10; }
    while (i--)   { RVputchar(buf[i]); }
}

#include <stdarg.h>
void RVprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            if      (*fmt == 'c') RVputchar(va_arg(args, int));
            else if (*fmt == 's') RVputs(va_arg(args, char *));
            else if (*fmt == 'd') RVprint_int(va_arg(args, int));
        } else {
            RVputchar(*fmt);
        }
        fmt++;
    }
    va_end(args);
}

// --- new: RX ---

char RVgetchar() {
    return (char)*UART;
}

void RVgets(char *buf, int max) {
    int i = 0;
    while (i < max - 1) {
        char c = RVgetchar();
        if (c == '\n' || c == '\r') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
}

// -----------------

int main() {
    // TX: classic hello world
    RVprintf("Hello, world!\n");
    RVprintf("Hail to King Terry the Terrible!\n");

    // RX + TX: simple echo
    RVputs("Enter your name: \n");
    char name[32];
    RVgets(name, 32);

    RVputs("Hello, ");
    RVputs(name);
    RVputs("!\n");

    // TX: confirm integer printing still works
    RVprintf("%d\n", (13 * 7) + (100 / 4) - (23 % 5));

    return 0;
}

void _start() {
    int r = main();
    asm volatile(
        "mv a0, %0\n"
        "li a7, 93\n"
        "ecall\n"
        : : "r"(r) : "a0", "a7"
    );
    while (1);
}

// generate ELF:
// wsl riscv64-unknown-elf-gcc -march=rv32im -mabi=ilp32 -O0 -nostdlib -Ttext=0x0 tests/basic_io.c -lgcc -o tests/basic_io.elf
// run:
// cargo run tests/basic_io.elf