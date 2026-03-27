use crate::cpu::CPU;
use crate::vm::VMError;

pub struct RiscV {
    cpu: CPU,
}

impl RiscV {
    pub fn new(elf_file: Vec<u8>, ram_length: usize) -> Result<Self, VMError> {
        let riscv = RiscV {
            cpu: CPU::new(elf_file, ram_length)?,
        };

        Ok(riscv)
    }

    // fn read_elf(...) -> Result<ELF, RiscVError> {...}
}
