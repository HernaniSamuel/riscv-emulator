use crate::risc_v::ElfImage;
use crate::vm::{VM, VMError};
pub struct CPU {
    vm: VM,
    running: bool,
    exit_code: i32,
}

impl CPU {
    // CPU::new() will receive the VM ram length and pass it to VM::new()
    pub fn new(elf_file: ElfImage, ram_length: usize) -> Result<Self, VMError> {
        let cpu = CPU {
            vm: VM::new(elf_file, ram_length)?,
            running: false,
            exit_code: 0,
        };

        Ok(cpu)
    }
}
