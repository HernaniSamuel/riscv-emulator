use crate::risc_v::ElfImage;

#[derive(Debug)]
pub struct VM {
    pub ram: Vec<u8>,
    pub registers: [u32; 32],
    pub pc: u32,
}

impl VM {
    /// Creates the VM, verifies that all segments fit in RAM,
    /// copies each segment to the correct virtual address,
    /// and boots the PC with the ELF entry point.
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, VMError> {
        let ram_size = ram_length_kb
            .checked_mul(1024)
            .ok_or(VMError::InvalidRamSize)?;

        // Validate all segments before allocating anything
        for seg in &elf_file.segments {
            let seg_end = (seg.vaddr as usize)
                .checked_add(seg.mem_size as usize)
                .ok_or(VMError::ELFTooLarge)?;

            if seg_end > ram_size {
                return Err(VMError::ELFTooLarge);
            }

            // file_size can never be larger than mem_size (the ELF specification prohibits this)
            if seg.data.len() > seg.mem_size as usize {
                return Err(VMError::InvalidSegment);
            }
        }

        let mut ram = vec![0u8; ram_size];

        // Copies each PT_LOAD segment to its corresponding address in RAM.
        // The bytes from (filesz .. memsz) are set to zero (BSS), as already ensured
        // by vec![0u8; ram_size] above.
        for seg in &elf_file.segments {
            let start = seg.vaddr as usize;
            let end = start + seg.data.len();
            ram[start..end].copy_from_slice(&seg.data);
        }

        Ok(VM {
            ram,
            registers: [0u32; 32],
            // The program counter starts at the entry point defined by the ELF, not at 0
            // Original bug: the program counter was always set to 0
            pc: elf_file.entry,
        })
    }

    // --- Memory access helpers (required for executing instructions) ---

    /// Reads 4 bytes (little-endian) from RAM at address `addr`.
    pub fn read_u32(&self, addr: u32) -> Result<u32, VMError> {
        let a = addr as usize;
        if a + 4 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(u32::from_le_bytes(self.ram[a..a + 4].try_into().unwrap()))
    }

    /// Writes 4 bytes (little-endian) to RAM at address `addr`.
    pub fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), VMError> {
        let a = addr as usize;
        if a + 4 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a..a + 4].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    /// Reads 1 byte from RAM at address `addr`.
    pub fn read_u8(&self, addr: u32) -> Result<u8, VMError> {
        let a = addr as usize;
        if a >= self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(self.ram[a])
    }

    /// Writes 1 byte to RAM at address `addr`.
    pub fn write_u8(&mut self, addr: u32, value: u8) -> Result<(), VMError> {
        let a = addr as usize;
        if a >= self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a] = value;
        Ok(())
    }

    /// Reads 2 bytes (little-endian) — required for compressed instructions (RV32C).
    pub fn read_u16(&self, addr: u32) -> Result<u16, VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(u16::from_le_bytes(self.ram[a..a + 2].try_into().unwrap()))
    }

    pub fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a..a + 2].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum VMError {
    ELFTooLarge,
    InvalidSegment,
    InvalidRamSize,
    MemoryOutOfBounds(u32),
}
