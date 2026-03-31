use crate::risc_v::ElfImage;
use crate::vm::{VM, VMError};

pub struct CPU {
    pub vm: VM,
    pub running: bool,
    pub exit_code: i32,
}

impl CPU {
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, VMError> {
        let vm = VM::new(elf_file, ram_length_kb)?;

        Ok(CPU {
            vm,
            running: false,
            exit_code: 0,
        })
    }

    /// Busca a instrução no PC atual (fetch).
    /// Retorna a palavra de 32 bits — a decodificação fica no passo seguinte.
    pub fn fetch(&self) -> Result<u32, VMError> {
        self.vm.read_u32(self.vm.pc)
    }

    /// Avança o PC em 4 bytes (instrução padrão RV32I).
    pub fn advance_pc(&mut self) {
        self.vm.pc = self.vm.pc.wrapping_add(4);
    }
}
