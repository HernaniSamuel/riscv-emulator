/// A simple RV32I virtual machine.
///
/// Guarantees:
/// - x0 register is always zero
/// - PC is always 4-byte aligned
/// - Memory accesses are bounds-checked
/// - State is never modified on error
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

    /// Reads 2 bytes (little-endian) from RAM at address `addr`.
    ///
    /// Used by the `LH`, `LHU`, and `SH` RV32I instructions.
    ///
    /// # Errors
    /// Returns [`VMError::MemoryOutOfBounds`] if `addr..addr+2` exceeds RAM bounds.
    pub fn read_u16(&self, addr: u32) -> Result<u16, VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(u16::from_le_bytes(self.ram[a..a + 2].try_into().unwrap()))
    }

    /// Writes 2 bytes (little-endian) to RAM at address `addr`.
    ///
    /// # Errors
    /// Returns [`VMError::MemoryOutOfBounds`] if `addr..addr+2` exceeds RAM bounds.
    /// The memory is **not** modified if an error is returned.
    pub fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a..a + 2].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    /// Returns the value of register `index`.
    ///
    /// # Errors
    /// Returns [`VMError::InvalidRegister`] if `index >= 32`.
    pub fn get_x(&self, index: usize) -> Result<u32, VMError> {
        if index >= self.registers.len() {
            return Err(VMError::InvalidRegister(index));
        }
        Ok(self.registers[index])
    }

    /// Sets register `index` to `value`.
    ///
    /// Writes to `x0` are silently ignored, as per the RISC-V specification.
    ///
    /// # Errors
    /// Returns [`VMError::InvalidRegister`] if `index >= 32`.
    pub fn set_x(&mut self, index: usize, value: u32) -> Result<(), VMError> {
        if index >= self.registers.len() {
            return Err(VMError::InvalidRegister(index));
        }

        // x0 is hardwired to zero in RISC-V
        if index == 0 {
            return Ok(()); // ignore writes
        }

        self.registers[index] = value;
        Ok(())
    }

    /// Returns the current program counter.
    pub fn get_pc(&self) -> u32 {
        self.pc
    }

    /// Sets the program counter to `value`.
    ///
    /// The PC must be 4-byte aligned (RV32I requirement).
    ///
    /// # Errors
    /// Returns [`VMError::UnalignedPC`] if `value` is not 4-byte aligned.
    /// The PC is **not** modified if an error is returned.
    pub fn set_pc(&mut self, value: u32) -> Result<(), VMError> {
        if value % 4 != 0 {
            return Err(VMError::UnalignedPC(value));
        }

        self.pc = value;
        Ok(())
    }

    /// Advances the program counter by `offset` bytes.
    ///
    /// This method checks for overflow and alignment.
    ///
    /// # Errors
    /// Returns [`VMError::PCOverflow`] if the addition overflows.
    /// Returns [`VMError::UnalignedPC`] if the resulting PC is not aligned.
    /// The PC is **not** modified if an error is returned.
    pub fn advance_pc(&mut self, offset: u32) -> Result<(), VMError> {
        let new_pc = self.pc.checked_add(offset).ok_or(VMError::PCOverflow)?;

        self.set_pc(new_pc)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum VMError {
    ELFTooLarge,
    InvalidSegment,
    InvalidRamSize,
    MemoryOutOfBounds(u32),
    InvalidRegister(usize),
    UnalignedPC(u32),
    PCOverflow,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_vm() -> VM {
        VM {
            ram: vec![0; 1024],
            registers: [0; 32],
            pc: 0,
        }
    }

    // testing register methods
    #[test]
    fn test_set_get_register() {
        let mut vm = dummy_vm();

        vm.set_x(5, 123).unwrap();
        assert_eq!(vm.get_x(5).unwrap(), 123);
    }

    #[test]
    fn test_x0_is_immutable() {
        let mut vm = dummy_vm();

        vm.set_x(0, 999).unwrap();
        assert_eq!(vm.get_x(0).unwrap(), 0);
    }

    #[test]
    fn test_invalid_register_index() {
        let mut vm = dummy_vm();

        assert!(vm.set_x(32, 1).is_err());
        assert!(vm.get_x(32).is_err());
    }

    // testing PC methods
    #[test]
    fn test_set_pc_valid_transition() {
        let mut vm = dummy_vm();

        vm.set_pc(8).unwrap();
        assert_eq!(vm.get_pc(), 8);
    }

    #[test]
    fn test_set_pc_invalid_does_not_change_state() {
        let mut vm = dummy_vm();

        vm.set_pc(8).unwrap();
        assert!(vm.set_pc(6).is_err());

        // state must remain unchanged
        assert_eq!(vm.get_pc(), 8);
    }

    #[test]
    fn test_advance_pc_valid_transition() {
        let mut vm = dummy_vm();

        vm.set_pc(4).unwrap();
        vm.advance_pc(4).unwrap();

        assert_eq!(vm.get_pc(), 8);
    }

    #[test]
    fn test_advance_pc_unaligned_does_not_change_state() {
        let mut vm = dummy_vm();

        vm.set_pc(4).unwrap();
        assert!(vm.advance_pc(2).is_err());

        assert_eq!(vm.get_pc(), 4);
    }

    #[test]
    fn test_pc_overflow_does_not_change_state() {
        let mut vm = dummy_vm();

        vm.set_pc(u32::MAX - 3).unwrap();
        assert!(vm.advance_pc(4).is_err());

        assert_eq!(vm.get_pc(), u32::MAX - 3);
    }

    // testing RAM methods
    #[test]
    fn test_ram_valid_write_read() {
        let mut vm = dummy_vm();

        vm.write_u8(10, 42).unwrap();
        assert_eq!(vm.read_u8(10).unwrap(), 42);
    }

    #[test]
    fn test_ram_write_out_of_bounds() {
        let mut vm = dummy_vm();

        assert!(vm.write_u8(1024, 1).is_err());
    }

    #[test]
    fn test_ram_read_out_of_bounds() {
        let vm = dummy_vm();

        assert!(vm.read_u8(1024).is_err());
    }

    #[test]
    fn test_ram_invalid_write_does_not_corrupt_memory() {
        let mut vm = dummy_vm();

        vm.write_u8(0, 55).unwrap();
        assert!(vm.write_u8(1024, 99).is_err());

        assert_eq!(vm.read_u8(0).unwrap(), 55);
    }

    #[test]
    fn test_ram_u32_roundtrip() {
        let mut vm = dummy_vm();

        vm.write_u32(4, 0xDEADBEEF).unwrap();
        assert_eq!(vm.read_u32(4).unwrap(), 0xDEADBEEF);
    }

    #[test]
    fn test_ram_u32_boundary_fail() {
        let mut vm = dummy_vm();

        // last 3 bytes only, not enough for u32
        assert!(vm.write_u32(1021, 1).is_err());
    }

    #[test]
    fn test_ram_u32_boundary_success() {
        let mut vm = dummy_vm(); // 1024 bytes
        // bytes 1020..1024 — limit
        assert!(vm.write_u32(1020, 0xCAFE).is_ok());
    }

    // testing u16 methods
    #[test]
    fn test_ram_u16_roundtrip() {
        let mut vm = dummy_vm();

        vm.write_u16(10, 0xABCD).unwrap();
        assert_eq!(vm.read_u16(10).unwrap(), 0xABCD);
    }

    #[test]
    fn test_ram_u16_little_endian() {
        let mut vm = dummy_vm();

        vm.write_u16(0, 0x1234).unwrap();

        assert_eq!(vm.read_u8(0).unwrap(), 0x34);
        assert_eq!(vm.read_u8(1).unwrap(), 0x12);
    }

    #[test]
    fn test_ram_u16_boundary_valid() {
        let mut vm = dummy_vm();

        let last_valid = vm.ram.len() as u32 - 2;
        vm.write_u16(last_valid, 0xBEEF).unwrap();

        assert_eq!(vm.read_u16(last_valid).unwrap(), 0xBEEF);
    }

    #[test]
    fn test_ram_u16_boundary_invalid() {
        let mut vm = dummy_vm();

        let invalid = vm.ram.len() as u32 - 1;
        assert!(vm.write_u16(invalid, 1).is_err());
        assert!(vm.read_u16(invalid).is_err());
    }

    #[test]
    fn test_ram_u16_out_of_bounds() {
        let mut vm = dummy_vm();

        let invalid = vm.ram.len() as u32;
        assert!(vm.write_u16(invalid, 1).is_err());
        assert!(vm.read_u16(invalid).is_err());
    }

    #[test]
    fn test_ram_u16_invalid_write_does_not_corrupt() {
        let mut vm = dummy_vm();

        vm.write_u16(0, 0xAAAA).unwrap();

        let invalid = vm.ram.len() as u32 - 1;
        assert!(vm.write_u16(invalid, 0xBBBB).is_err());

        assert_eq!(vm.read_u16(0).unwrap(), 0xAAAA);
    }
}
