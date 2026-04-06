// gcc_source_test.c
// Bare-metal RV32I test for emulator

volatile int result;

int _start() {
    int a = 5;
    int b = 10;

    // ALU
    int c = a + b;
    int d = c - a;
    int e = c & b;
    int f = c | b;
    int g = c ^ b;

    // shifts
    int h = c << 2;
    int i = h >> 1;

    // branch
    int j = 0;
    if (i > 10) {
        j = 1;
    } else {
        j = 2;
    }

    // memory (forces load/store)
    volatile int mem[4];
    mem[0] = a;
    mem[1] = b;
    mem[2] = c;
    mem[3] = mem[0] + mem[1];

    result = mem[3] + j;

    // infinite loop (bare metal)
    while (1) { }
}

// run it using the following commands (on the root of the project)
// On windows it needs WSL, I don't know how to run it on linux but it appears to be even easier than in windows...

// generate assembly `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -S tests/gcc_source_test.c`
// generate ELF `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Ttext=0x0 tests/gcc_source_test.c -o tests/gcc_source_test.elf`
// run the emulator with the generated ELF `cargo run tests/gcc_source_test.elf`
// disassemble to compare `wsl riscv64-unknown-elf-objdump -d tests/gcc_source_test.elf`