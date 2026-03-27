use riscv::*;

fn main() {
    let elf_file = vec![0u8; 32 * 1024];
    let mem_length: usize = 16;
    let riscv = RiscV::new(elf_file, mem_length).unwrap();
}
