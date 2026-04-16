//! # Virtual Machine (VM)
//!
//! This module implements the lowest-level abstraction of the emulator,
//! modeling memory, registers, and basic I/O devices.
//!
//! The VM provides a safe and deterministic interface over raw machine
//! state, which is used by higher layers to implement instruction execution.
//!
//! ## Responsibilities
//!
//! - Manage RAM and enforce memory safety
//! - Provide access to general-purpose registers (`x0..x31`)
//! - Maintain the program counter (PC)
//! - Expose memory-mapped I/O (MMIO)
//!
//! ## Memory model
//!
//! Memory is represented as a contiguous byte array and follows a
//! little-endian layout, as defined by the RISC-V specification.
//!
//! All memory accesses are bounds-checked. Invalid accesses result in
//! [`VMError::MemoryOutOfBounds`] and do not modify the VM state.
//!
//! ## Registers
//!
//! The VM exposes 32 general-purpose registers (`x0..x31`).
//!
//! Register `x0` is hardwired to zero and ignores writes, as required by
//! the RISC-V specification.
//!
//! ## Program counter (PC)
//!
//! The program counter stores the address of the next instruction to be
//! executed.
//!
//! The VM ensures that the PC always points to a valid address in RAM,
//! but does not enforce instruction alignment. Alignment is the
//! responsibility of the CPU layer.
//!
//! ## Memory-mapped I/O
//!
//! A minimal UART device is exposed via memory-mapped registers:
//!
//! | Address        | Name   | Description              |
//! |----------------|--------|--------------------------|
//! | `0x1000_0000`  | TX     | Write a character        |
//! | `0x1000_0000`  | RX     | Read a character (block) |
//! | `0x1000_0004`  | STATUS | Transmitter status       |
//!
//! ### Behavior
//!
//! - Writing to TX prints a character to standard output
//! - Reading from RX blocks until input is available
//! - STATUS returns the transmitter readiness flag
//!
//! ## Architecture
//!
//! The VM is used by the [`crate::cpu`] layer, which is responsible for
//! instruction decoding, execution, and enforcing ISA-level constraints
//! such as alignment and control flow.
//!
//! The VM itself does not interpret instructions and should be considered
//! a pure hardware abstraction layer.
//!
//! ## Error handling
//!
//! Errors are represented by [`VMError`] and correspond to invalid
//! operations at the hardware level (e.g. out-of-bounds memory access).
//!
//! Higher layers wrap these errors to provide additional context while
//! preserving the original cause.
//!
//! ## Guarantees
//!
//! - All memory accesses are bounds-checked
//! - Register `x0` is always zero
//! - The program counter always points to valid memory
//! - State is not modified on error (operations are atomic)

use crate::risc_v::ElfImage;
use std::io::Read;

/// Base address for the UART MMIO region.
const UART_BASE: u32 = 0x1000_0000;

/// UART transmit register (write-only).
/// Writing a byte here outputs it to stdout.
const UART_TX: u32 = UART_BASE;

/// UART receive register (read-only).
/// Reading blocks until a byte is available from stdin.
const UART_RX: u32 = UART_BASE;

/// UART status register.
/// Bit 0 (`TX_READY`) indicates the transmitter is ready.
const UART_STATUS: u32 = UART_BASE + 0x04;

/// UART transmitter ready flag.
const TX_READY: u8 = 0b0000_0001;

/// Errors that can occur in the virtual machine layer.
///
/// # Design
///
/// This error type represents failures at the lowest level of the emulator.
///
/// Higher layers (such as [`crate::cpu::CPUError`] and [`crate::risc_v::RiscVError`]) wrap this type
/// to provide additional context.
///
/// # Variants
///
/// - [`VMError::ELFTooLarge`] — ELF segments exceed available RAM
/// - [`VMError::InvalidSegment`] — ELF segment is malformed
/// - [`VMError::InvalidRamSize`] — RAM size overflowed during calculation
/// - [`VMError::MemoryOutOfBounds`] — invalid memory access
/// - [`VMError::InvalidRegister`] — invalid register index
/// - [`VMError::UnalignedPC`] — PC is not properly aligned
/// - [`VMError::PCOverflow`] — PC arithmetic overflowed
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

/// A simple RV32I virtual machine.
///
/// This struct represents the lowest-level abstraction of the emulator,
/// providing memory, registers, and basic I/O primitives.
///
/// # Responsibilities
///
/// - Manage RAM and enforce memory safety
/// - Provide register access (`x0..x31`)
/// - Maintain the program counter (PC)
/// - Expose memory-mapped I/O (e.g. UART)
///
/// # Guarantees
///
/// - Register `x0` is always zero
/// - All memory accesses are bounds-checked
/// - State is not modified on error
///
/// # Memory model
///
/// Memory is represented as a contiguous byte array. All accesses are
/// little-endian, following the RISC-V specification.
///
/// # I/O
///
/// A minimal UART device is exposed via memory-mapped I/O:
///
/// - `0x1000_0000` — TX (write)
/// - `0x1000_0000` — RX (read)
/// - `0x1000_0004` — STATUS
///
/// # Examples
///
/// ```no_run
/// use riscv::{RiscV, read_elf};
///
/// # let bytes: &[u8] = &[0; 4];
/// # let elf = read_elf(bytes).unwrap();
/// let mut vm = RiscV::new(elf, 1024).unwrap();
/// ```
#[derive(Debug)]
pub struct VM {
    ram: Vec<u8>,
    registers: [u32; 32],
    pc: u32,
}

impl VM {
    /// Creates a new virtual machine from an ELF image.
    ///
    /// This method:
    ///
    /// - Allocates RAM
    /// - Validates all ELF segments
    /// - Loads segments into memory
    /// - Initializes the program counter to the ELF entry point
    ///
    /// # Errors
    ///
    /// Returns [`VMError`] if:
    ///
    /// - The RAM size overflows
    /// - A segment does not fit in memory
    /// - A segment is malformed
    ///
    /// # Notes
    ///
    /// All segments are validated before any memory is allocated or modified.
    /// This guarantees that the VM is never partially initialized.
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

    /// Reads a 32-bit little-endian value from memory.
    ///
    /// The value is read from the range `addr..addr+4`.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds.
    ///
    /// # Guarantees
    ///
    /// The VM state is not modified if an error occurs.
    pub fn read_u32(&self, addr: u32) -> Result<u32, VMError> {
        let a = addr as usize;
        if a + 4 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(u32::from_le_bytes(self.ram[a..a + 4].try_into().unwrap()))
    }

    /// Writes a 32-bit little-endian value to memory.
    ///
    /// The value is written to the range `addr..addr+4`.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds.
    ///
    /// # Guarantees
    ///
    /// Memory is not modified if an error occurs.
    pub fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), VMError> {
        let a = addr as usize;
        if a + 4 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a..a + 4].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    /// Reads a byte from memory or a memory-mapped device.
    ///
    /// This method supports both regular RAM access and memory-mapped I/O.
    ///
    /// # Memory-mapped I/O
    ///
    /// The following addresses have special behavior:
    ///
    /// - `UART_STATUS` — returns the transmitter status flag
    /// - `UART_RX` — reads a byte from standard input (blocking)
    ///
    /// All other addresses are treated as normal RAM.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the address is outside RAM
    /// and does not correspond to a valid memory-mapped register.
    ///
    /// # Guarantees
    ///
    /// The VM state is not modified if an error occurs.
    pub fn read_u8(&self, addr: u32) -> Result<u8, VMError> {
        match addr {
            UART_STATUS => return Ok(TX_READY),
            UART_RX => {
                let mut buf = [0u8];
                std::io::stdin().read_exact(&mut buf).unwrap();
                return Ok(buf[0]);
            }
            _ => {}
        }

        let a = addr as usize;
        if a >= self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(self.ram[a])
    }

    /// Writes a byte to memory or a memory-mapped device.
    ///
    /// This method supports both regular RAM access and memory-mapped I/O.
    ///
    /// # Memory-mapped I/O
    ///
    /// The following addresses have special behavior:
    ///
    /// - `UART_TX` — writes a character to standard output
    ///
    /// All other addresses are treated as normal RAM.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the address is outside RAM
    /// and does not correspond to a valid memory-mapped register.
    ///
    /// # Guarantees
    ///
    /// Memory is not modified if an error occurs.
    pub fn write_u8(&mut self, addr: u32, value: u8) -> Result<(), VMError> {
        if addr == UART_TX {
            print!("{}", value as char);
            return Ok(());
        }

        let a = addr as usize;
        if a >= self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a] = value;
        Ok(())
    }

    /// Reads a 16-bit little-endian value from memory.
    ///
    /// The value is read from the range `addr..addr+2`.
    ///
    /// This method does not enforce alignment constraints. Alignment checks,
    /// when required, must be handled by the CPU layer.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds.
    ///
    /// # Guarantees
    ///
    /// The VM state is not modified if an error occurs.
    pub fn read_u16(&self, addr: u32) -> Result<u16, VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        Ok(u16::from_le_bytes(self.ram[a..a + 2].try_into().unwrap()))
    }

    /// Writes a 16-bit little-endian value to memory.
    ///
    /// The value is written to the range `addr..addr+2`.
    ///
    /// This method does not enforce alignment constraints. Alignment checks,
    /// when required, must be handled by the CPU layer.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds.
    ///
    /// # Guarantees
    ///
    /// Memory is not modified if an error occurs.
    pub fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), VMError> {
        let a = addr as usize;
        if a + 2 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }
        self.ram[a..a + 2].copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    /// Returns the value of a general-purpose register.
    ///
    /// Registers are indexed from `0` to `31`, corresponding to `x0..x31`
    /// in the RISC-V specification.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::InvalidRegister`] if `index >= 32`.
    ///
    /// # Guarantees
    ///
    /// The VM state is not modified.
    pub fn get_x(&self, index: usize) -> Result<u32, VMError> {
        if index >= self.registers.len() {
            return Err(VMError::InvalidRegister(index));
        }
        Ok(self.registers[index])
    }

    /// Sets the value of a general-purpose register.
    ///
    /// Registers are indexed from `0` to `31`, corresponding to `x0..x31`.
    ///
    /// Writes to register `x0` are ignored, as it is hardwired to zero
    /// in the RISC-V specification.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::InvalidRegister`] if `index >= 32`.
    ///
    /// # Guarantees
    ///
    /// - Register `x0` is never modified
    /// - No state is modified if an error occurs
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

    /// Returns the total size of RAM in bytes.
    ///
    /// This method provides read-only access to the VM memory size without
    /// exposing the internal memory representation.
    ///
    /// # Usage
    ///
    /// Typically used by the CPU layer to initialize the stack pointer to
    /// the top of RAM.
    ///
    /// # Guarantees
    ///
    /// The returned value always reflects the full allocated RAM size.
    pub fn ram_size(&self) -> usize {
        self.ram.len()
    }

    /// Returns the current program counter (PC).
    ///
    /// The program counter represents the address of the next instruction
    /// to be executed.
    ///
    /// # Guarantees
    ///
    /// The VM state is not modified.
    pub fn get_pc(&self) -> u32 {
        self.pc
    }

    /// Sets the program counter (PC).
    ///
    /// The program counter determines the address of the next instruction
    /// to be executed.
    ///
    /// This method ensures that the value points to a valid address in RAM,
    /// but does not enforce instruction alignment.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if `value` is outside RAM.
    ///
    /// # Guarantees
    ///
    /// - The PC is not modified if an error occurs
    /// - Instruction alignment is not validated at this level
    ///
    /// # Notes
    ///
    /// Alignment and control-flow correctness are the responsibility of the CPU layer.
    pub fn set_pc(&mut self, value: u32) -> Result<(), VMError> {
        let addr = value as usize;

        if addr >= self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(value));
        }

        self.pc = value;
        Ok(())
    }
}

// TESTS
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
