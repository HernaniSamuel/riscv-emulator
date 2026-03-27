pub struct VM {
    ram: Vec<u8>,
    registers: [u32; 32],
    pc: u32,
}

impl VM {
    // fn new will load the elf file in the memory if it's not larger than the ram
    pub fn new(elf_file: Vec<u8>, ram_length: usize) -> Result<Self, VMError> {
        let vm: VM = VM {
            ram: vec![0u8; ram_length * 1024],
            registers: [0u32; 32],
            pc: 0,
        };

        if elf_file.len() > vm.ram.len() {
            Err(VMError::ELFTooLarge)
        } else {
            Ok(vm)
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum VMError {
    ELFTooLarge,
}
