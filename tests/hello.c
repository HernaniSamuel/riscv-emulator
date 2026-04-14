// The "hello world" from scratch
// I placed "RV" before the function names because GCC was misunderstanding my functions with its internal functions...

// macro that points to our MMIO address
#define UART ((volatile unsigned char *)0x10000000)

// let's start making a helper function to put char on the right MMIO address
void RVputchar(char c) {
    *UART = c;
}

// now it's time to put more than one char at once
void RVputs(const char *s) {
    while (*s) {
        RVputchar(*s++);
    }
}

// let's add a way to print int just because we can
void RVprint_int(int x) {
    char buf[16];
    int i = 0;

    if (x == 0) {
        RVputchar('0');
        return;
    }

    if (x < 0) {
        RVputchar('-');
        x = -x;
    }

    while (x > 0) {
        buf[i++] = '0' + (x % 10);
        x /= 10;
    }

    while (i--) {
        RVputchar(buf[i]);
    }
}

// And finally, our printf!
#include <stdarg.h>

void RVprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;

            if (*fmt == 'c') {
                char c = va_arg(args, int);
                RVputchar(c);
            }
            else if (*fmt == 's') {
                char *s = va_arg(args, char *);
                RVputs(s);
            }
            else if (*fmt == 'd') {
                int x = va_arg(args, int);
                RVprint_int(x);
            }
        } else {
            RVputchar(*fmt);
        }

        fmt++;
    }

    va_end(args);
}

int main() {
    RVprintf("Hello, world! %d \n", 42);
    RVprintf("Hail to King Terry the Terrible! \n");
    return 0;
}

void _start() {
    int r = main();

    asm volatile(
        "mv a0, %0\n"
        "li a7, 93\n"
        "ecall\n"
        :
        : "r"(r)
        : "a0", "a7"
    );

    while (1);
}

// run it using the following commands (on the root of the project)
// On windows it needs WSL, I don't know how to run it on linux but it appears to be even easier than in windows...

// generate ELF: wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Ttext=0x0 tests/hello.c -lgcc -o tests/hello.elf
// run the emulator with the generated ELF: cargo run tests/hello.elf

// run the emulator in disassemble mode: cargo run tests/hello.elf disassemble
// disassemble to compare: wsl riscv64-unknown-elf-objdump -d tests/hello.elf
// generate assembly file: wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -S tests/hello.c