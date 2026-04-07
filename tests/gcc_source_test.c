// gcc_source_test.c
// Bare-metal RV32I test for emulator

volatile int result;

int _start() {
    return 42;
}

// run it using the following commands (on the root of the project)
// On windows it needs WSL, I don't know how to run it on linux but it appears to be even easier than in windows...

// generate assembly `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -S tests/gcc_source_test.c`
// generate ELF `wsl riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -O0 -nostdlib -Ttext=0x0 tests/gcc_source_test.c -o tests/gcc_source_test.elf`
// run the emulator with the generated ELF `cargo run tests/gcc_source_test.elf`
// disassemble to compare `wsl riscv64-unknown-elf-objdump -d tests/gcc_source_test.elf`