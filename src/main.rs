use std::env;
use std::fs;

use riscv::{RiscV, read_elf};

fn main() {
    let path = env::args().nth(1).expect("missing ELF file");

    let bytes = fs::read(path).expect("failed to read file");

    let elf = read_elf(&bytes).expect("invalid ELF");

    let mut machine = RiscV::new(elf, 4096).expect("failed to create machine");

    machine.cpu.step().unwrap();
}
