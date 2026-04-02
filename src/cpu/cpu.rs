use crate::risc_v::ElfImage;
use crate::vm::{VM, VMError};
pub struct CPU {
    pub vm: VM,
    pub running: bool,
    pub exit_code: i32,
}

impl CPU {
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, CPUError> {
        let vm = VM::new(elf_file, ram_length_kb)?;

        Ok(CPU {
            vm,
            running: false,
            exit_code: 0,
        })
    }

    /// Fetches the instruction at the current PC (fetch stage).
    /// Returns the 32-bit word — decoding happens in the next step.
    pub fn fetch(&self) -> Result<u32, CPUError> {
        let pc = self.vm.get_pc();

        // ISA rule: instruction must be 4-byte aligned
        if pc & 0b11 != 0 {
            return Err(CPUError::InstructionAddressMisaligned(pc));
        }

        Ok(self.vm.read_u32(pc)?)
    }

    /// Advances the PC by 4 bytes (standard RV32I instruction size).
    pub fn advance_pc(&mut self) {
        self.vm.pc = self.vm.pc.wrapping_add(4);
    }
}

#[derive(Debug)]
pub enum CPUError {
    VM(VMError),
    IllegalInstruction(u32),
    InstructionAddressMisaligned(u32),
}

impl From<VMError> for CPUError {
    fn from(e: VMError) -> Self {
        CPUError::VM(e)
    }
}
