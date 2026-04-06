use std::env;
use std::fs;

use riscv::{RiscV, read_elf};

fn main() {
    let path = env::args().nth(1).expect("missing ELF file");

    let bytes = fs::read(path).expect("failed to read file");

    let elf = read_elf(&bytes).expect("invalid ELF");

    let mut machine = RiscV::new(elf, 4096).expect("failed to create machine");

    loop {
        let pc = machine.cpu.get_pc();

        let raw = match machine.cpu.fetch() {
            Ok(i) => i,
            Err(_) => break,
        };

        let instr = match machine.cpu.decode(raw) {
            Ok(i) => i,
            Err(_) => break,
        };

        println!("{:#010x}: {:?}", pc, instr);

        machine.cpu.advance_pc().unwrap();
    }
}
