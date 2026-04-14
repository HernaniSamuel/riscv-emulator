// guardrail.c
// Returns 305419896 (0x12345678) only if everything is correct, otherwise it returns 0.

typedef unsigned int  u32;
typedef signed int    i32;
typedef unsigned char u8;
typedef signed char   i8;
typedef unsigned short u16;
typedef signed short   i16;

u32 mix(u32 x) {
    // força jal/jalr + stack
    return (x << 3) ^ (x >> 2) ^ 0xA5A5A5A5;
}

int main() {
    u32 acc = 0;

    // ───────────────────────────────
    // arithmetic + shifts
    // ───────────────────────────────
    {
        u32 a = 10;
        u32 b = 20;
        u32 c = (a + b) * 3;
        acc ^= mix(c);
    }

    // ───────────────────────────────
    // signed vs unsigned compare
    // ───────────────────────────────
    {
        i32 a = -1;
        u32 b = 1;

        if (a < (i32)b) acc ^= 0x11111111; // signed
        if ((u32)a > b) acc ^= 0x22222222; // unsigned
    }

    // ───────────────────────────────
    // loads/stores + sign extension
    // ───────────────────────────────
    {
        u32 word = 0x80FF7F01;
        u8 *p = (u8*)&word;

        i8  sbyte = (i8)p[1];   // 0x7F
        i16 shalf = *(i16*)&p[2]; // 0x80FF (negative)

        acc ^= (u32)sbyte;
        acc ^= (u32)shalf;
    }

    // ───────────────────────────────
    // loop + branch
    // ───────────────────────────────
    {
        u32 sum = 0;
        for (u32 i = 0; i < 16; i++) {
            sum += (i << 1) ^ (i >> 1);
        }
        acc ^= sum;
    }

    // ───────────────────────────────
    // memory + alignment
    // ───────────────────────────────
    {
        u32 buf[4];
        buf[0] = 0xDEADBEEF;
        buf[1] = 0xCAFEBABE;
        buf[2] = buf[0] ^ buf[1];
        buf[3] = buf[2] + 0x1234;

        acc ^= buf[3];
    }

    // expected static value
    return acc == 0x7D3A02AD ? 0x12345678 : 0; // 0 = wrong | 305419896 = right
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

// generate ELF `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Ttext=0x0 tests/guardrail.c -o tests/guardrail.elf`
// run the emulator with the generated ELF `cargo run tests/guardrail.elf`
// disassemble to compare `wsl riscv64-unknown-elf-objdump -d tests/guardrail.elf`
// generate assembly `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -S tests/guardrail.c`