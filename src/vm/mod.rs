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
//! The VM implements a simplified RISC-V machine model with memory,
//! registers, privileged state, and basic interrupt-capable peripherals.
//!
//! It acts as a deterministic hardware abstraction layer used by the CPU
//! execution engine.
//!
//! ### Hardware components
//!
//! The VM now models multiple subsystems:
//!
//! - **RAM** — byte-addressable main memory
//! - **Register file** — 32 general-purpose registers (`x0..x31`)
//! - **CSR bank** — machine-level control and status registers (M-mode)
//! - **CLINT** — machine timer and software interrupt controller
//! - **UART (MMIO)** — minimal serial I/O device
//!
//! ### CSR (Control and Status Registers)
//!
//! The CSR subsystem models privileged architectural state such as:
//!
//! - interrupt enable and pending bits (`mstatus`, `mie`, `mip`)
//! - trap handling (`mtvec`, `mepc`, `mcause`)
//! - machine configuration (`misa`, `mhartid`)
//! - cycle and instruction counters (`cycle`, `instret`)
//!
//! CSRs are accessed only through controlled VM interfaces and are not
//! directly exposed as raw memory.
//!
//! ### CLINT (Core Local Interruptor)
//!
//! The CLINT provides machine timer functionality:
//!
//! - `mtime` — free-running 64-bit timer
//! - `mtimecmp` — compare register for timer interrupts
//!
//! When `mtime >= mtimecmp`, the machine timer interrupt pending bit
//! (MTIP) is reflected in the CSR `mip` register.
//!
//! Time progression is explicitly simulated via `VM::tick()`.
//!
//! ### Time model
//!
//! The VM is cycle-driven rather than real-time:
//!
//! - Each call to `tick()` advances the global machine time
//! - Timer interrupts are evaluated during each tick
//! - Interrupt state is synchronized into CSR `mip`
//!
//! ### Memory-mapped I/O
//!
//! External devices (such as UART) are still exposed via MMIO regions,
//! while CLINT is handled through a dedicated internal subsystem rather
//! than general memory access.
//!
//! ## Guarantees
//!
//! - Memory accesses are bounds-checked
//! - Register `x0` remains hardwired to zero
//! - CSR state is consistent with architectural rules (M-mode)
//! - CLINT timer state is monotonically increasing
//! - Interrupt state is reflected through CSR `mip`
//! - State updates remain atomic per operation

pub mod clint;
pub mod csr;

use crate::risc_v::ElfImage;
use crate::vm::clint::*;
use crate::vm::csr::Csrs;
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
/// - [`VMError::InvalidCSR`] — invalid CSR index
/// - [`VMError::UnalignedPC`] — PC is not properly aligned
/// - [`VMError::PCOverflow`] — PC arithmetic overflowed
#[derive(Debug, Clone, PartialEq)]
pub enum VMError {
    ELFTooLarge,
    InvalidSegment,
    InvalidRamSize,
    MemoryOutOfBounds(u32),
    InvalidRegister(usize),
    InvalidCSR(u16),
    UnalignedPC(u32),
    PCOverflow,
}

/// A simple RV32IM virtual machine.
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
    csr: Csrs,
    clint: Clint,
}

impl VM {
    /// Creates a new virtual machine from an ELF image.
    ///
    /// This method performs full VM initialization in a deterministic and
    /// all-or-nothing manner.
    ///
    /// # Initialization steps
    ///
    /// - Computes and allocates RAM
    /// - Validates all ELF loadable segments before modifying state
    /// - Loads segment contents into memory
    /// - Zero-initializes uninitialized memory regions (BSS)
    /// - Clears all general-purpose registers (`x0..x31`)
    /// - Initializes all Control and Status Registers (CSRs)
    /// - Initializes the CLINT timer device
    /// - Sets the program counter (`PC`) to the ELF entry point
    ///
    /// # Hardware state after creation
    ///
    /// - `x0` is hardwired to zero
    /// - All integer registers start at `0`
    /// - RAM is zero-filled except for loaded ELF segments
    /// - CSR state starts from the default machine reset state
    /// - CLINT starts from the default timer reset state
    /// - `PC = elf_file.entry`
    ///
    /// # Default CLINT reset state
    ///
    /// - `mtime = 0`
    /// - `mtimecmp = u64::MAX`
    ///
    /// This prevents spurious timer interrupts before software programs the
    /// timer compare register.
    ///
    /// # Errors
    ///
    /// Returns [`VMError`] if:
    ///
    /// - The RAM size overflows during allocation
    /// - Any ELF segment exceeds available RAM
    /// - Any ELF segment is malformed
    ///
    /// # Notes
    ///
    /// All ELF segments are validated before memory allocation or state
    /// mutation occurs. This guarantees the VM is never partially
    /// initialized.
    ///
    /// The caller receives either a fully valid machine or an error.
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, VMError> {
        let ram_size = ram_length_kb
            .checked_mul(1024)
            .ok_or(VMError::InvalidRamSize)?;

        // Validate all segments before allocating anything.
        for seg in &elf_file.segments {
            let seg_end = (seg.vaddr as usize)
                .checked_add(seg.mem_size as usize)
                .ok_or(VMError::ELFTooLarge)?;

            if seg_end > ram_size {
                return Err(VMError::ELFTooLarge);
            }

            // ELF guarantees file_size <= mem_size.
            if seg.data.len() > seg.mem_size as usize {
                return Err(VMError::InvalidSegment);
            }
        }

        let mut ram = vec![0u8; ram_size];

        // Copy PT_LOAD contents into RAM.
        // Remaining bytes (filesz..memsz) stay zeroed as BSS.
        for seg in &elf_file.segments {
            let start = seg.vaddr as usize;
            let end = start + seg.data.len();
            ram[start..end].copy_from_slice(&seg.data);
        }

        let csr = Csrs {
            misa: (1 << 30) | (1 << 8) | (1 << 12),
            mhartid: 0,
            ..Default::default()
        };

        let clint = Clint::default();

        Ok(VM {
            ram,
            registers: [0u32; 32],
            pc: elf_file.entry,
            csr,
            clint,
        })
    }

    /// Reads the value of a Control and Status Register (CSR).
    ///
    /// CSRs are processor-internal registers used for machine control,
    /// interrupt configuration, exception handling, counters, and other
    /// architectural state.
    ///
    /// The register is selected by its 12-bit CSR address, as defined by
    /// the RISC-V privileged specification.
    ///
    /// # Parameters
    ///
    /// - `addr` — CSR address identifier.
    ///
    /// # Returns
    ///
    /// - `Ok(value)` if the CSR exists and is readable
    /// - `Err(VMError::InvalidCSR(addr))` if the CSR is not implemented
    ///   or cannot be read
    ///
    /// # Guarantees
    ///
    /// - The VM state is not modified
    /// - No side effects occur during reads unless explicitly modeled by the
    ///   underlying CSR implementation
    ///
    /// # Notes
    ///
    /// This method provides the CPU layer with controlled access to the
    /// processor CSR bank without exposing internal VM representation.
    pub fn read_csr(&self, addr: u16) -> Result<u32, VMError> {
        self.csr.read(addr).ok_or(VMError::InvalidCSR(addr))
    }

    /// Writes a value to a Control and Status Register (CSR).
    ///
    /// CSRs are processor-internal registers that control privileged state,
    /// interrupts, trap vectors, counters, and other machine-level behavior.
    ///
    /// The register is selected by its 12-bit CSR address.
    ///
    /// # Parameters
    ///
    /// - `addr` — CSR address identifier
    /// - `value` — value to be written
    ///
    /// # Returns
    ///
    /// - `Ok(())` if the CSR exists and accepted the write
    /// - `Err(VMError::InvalidCSR(addr))` if the CSR is not implemented
    ///   or does not accept writes
    ///
    /// # Guarantees
    ///
    /// - If this method returns `Err(...)`, no CSR state is modified
    /// - Write restrictions and bit masks are enforced by the CSR subsystem
    ///
    /// # Notes
    ///
    /// Some CSRs may ignore unsupported bits or apply architectural masking,
    /// depending on the simplified hardware model implemented by the VM.
    pub fn write_csr(&mut self, addr: u16, value: u32) -> Result<(), VMError> {
        self.csr
            .write(addr, value)
            .then_some(())
            .ok_or(VMError::InvalidCSR(addr))
    }

    // --- Memory access helpers (required for executing instructions) ---

    /// Reads a 32-bit little-endian value from memory or a memory-mapped device.
    ///
    /// The value is read from the range `addr..addr+4` when accessing RAM.
    ///
    /// This method also supports memory-mapped I/O devices implemented by the VM.
    ///
    /// # Memory-mapped I/O
    ///
    /// The following regions may be intercepted before RAM access:
    ///
    /// - CLINT timer registers (`mtime`, `mtimecmp`)
    ///
    /// All other addresses are treated as normal RAM.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds
    /// and does not correspond to a supported memory-mapped device.
    ///
    /// # Guarantees
    ///
    /// - The VM state is not modified if an error occurs
    /// - Reads from RAM use little-endian byte order
    pub fn read_u32(&self, addr: u32) -> Result<u32, VMError> {
        if let Some(value) = self.clint.read_u32(addr) {
            return Ok(value);
        }

        let a = addr as usize;

        if a + 4 > self.ram.len() {
            return Err(VMError::MemoryOutOfBounds(addr));
        }

        Ok(u32::from_le_bytes(self.ram[a..a + 4].try_into().unwrap()))
    }

    /// Writes a 32-bit little-endian value to memory or a memory-mapped device.
    ///
    /// The value is written to the range `addr..addr+4` when accessing RAM.
    ///
    /// This method also supports memory-mapped I/O devices implemented by the VM.
    ///
    /// # Memory-mapped I/O
    ///
    /// The following regions may be intercepted before RAM access:
    ///
    /// - CLINT timer registers (`mtime`, `mtimecmp`)
    ///
    /// All other addresses are treated as normal RAM.
    ///
    /// # Errors
    ///
    /// Returns [`VMError::MemoryOutOfBounds`] if the access exceeds RAM bounds
    /// and does not correspond to a supported memory-mapped device.
    ///
    /// # Guarantees
    ///
    /// - Memory is not modified if an error occurs
    /// - Writes to RAM use little-endian byte order
    /// - Device writes are delegated atomically to the target device
    pub fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), VMError> {
        if self.clint.write_u32(addr, value) {
            return Ok(());
        }

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

    /// Advances the virtual machine time by one tick.
    ///
    /// This method acts as the system clock for the entire emulator.
    ///
    /// # Behavior
    ///
    /// - Increments the CLINT `mtime` counter
    /// - Evaluates the timer interrupt condition (`mtime >= mtimecmp`)
    /// - Updates the CSR `mip` register (MTIP bit)
    ///
    /// # Side effects
    ///
    /// - May set or clear the machine timer interrupt pending bit (`MTIP`)
    /// - Affects interrupt delivery in the CPU layer
    ///
    /// # Notes
    ///
    /// This method must be called by the CPU execution loop (either per
    /// instruction or per cycle) to simulate passage of time.
    pub fn tick(&mut self) {
        self.clint.tick();

        if self.clint.timer_pending() {
            self.csr.mip |= 1 << 7; // MTIP
        } else {
            self.csr.mip &= !(1 << 7);
        }
    }

    /// Checks whether a machine timer interrupt is pending.
    ///
    /// This is a convenience wrapper around the CLINT timer comparison logic.
    ///
    /// # Returns
    ///
    /// - `true` if `mtime >= mtimecmp`
    /// - `false` otherwise
    ///
    /// # Notes
    ///
    /// This method does not modify VM state and is primarily intended for
    /// debugging or testing purposes. The authoritative interrupt state is
    /// exposed through the CSR `mip` register.
    pub fn timer_pending(&self) -> bool {
        self.clint.timer_pending()
    }
}

// TESTS
#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_vm() -> VM {
        let csr = Csrs {
            misa: (1 << 30) | (1 << 8) | (1 << 12),
            mhartid: 0,
            ..Default::default()
        };

        VM {
            ram: vec![0; 1024],
            registers: [0; 32],
            pc: 0,
            csr,
            clint: Clint::default(),
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

        let addr = 4;

        vm.write_u32(addr, 0xDEADBEEF).unwrap();
        assert_eq!(vm.read_u32(addr).unwrap(), 0xDEADBEEF);
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

    // testing CSR methods

    #[test]
    fn test_read_default_boot_csrs() {
        let vm = dummy_vm();

        assert_eq!(
            vm.read_csr(0x301).unwrap(),
            (1 << 30) | (1 << 8) | (1 << 12)
        ); // misa
        assert_eq!(vm.read_csr(0xF14).unwrap(), 0); // mhartid
    }

    #[test]
    fn test_write_and_read_rw_csrs() {
        let mut vm = dummy_vm();

        vm.write_csr(0x340, 0xDEADBEEF).unwrap(); // mscratch
        vm.write_csr(0x305, 0x100).unwrap(); // mtvec
        vm.write_csr(0x342, 7).unwrap(); // mcause

        assert_eq!(vm.read_csr(0x340).unwrap(), 0xDEADBEEF);
        assert_eq!(vm.read_csr(0x305).unwrap(), 0x100);
        assert_eq!(vm.read_csr(0x342).unwrap(), 7);
    }

    #[test]
    fn test_invalid_csr_read_fails() {
        let vm = dummy_vm();

        assert_eq!(vm.read_csr(0x999), Err(VMError::InvalidCSR(0x999)));
    }

    #[test]
    fn test_invalid_csr_write_fails() {
        let mut vm = dummy_vm();

        assert_eq!(vm.write_csr(0x999, 123), Err(VMError::InvalidCSR(0x999)));
    }

    #[test]
    fn test_read_only_csrs_reject_writes() {
        let mut vm = dummy_vm();

        assert!(vm.write_csr(0x301, 0).is_err()); // misa
        assert!(vm.write_csr(0xF14, 1).is_err()); // mhartid
        assert!(vm.write_csr(0xC00, 0).is_err()); // cycle
        assert!(vm.write_csr(0xC02, 0).is_err()); // instret
    }

    #[test]
    fn test_mepc_is_word_aligned_on_write() {
        let mut vm = dummy_vm();

        vm.write_csr(0x341, 0x123).unwrap();

        assert_eq!(vm.read_csr(0x341).unwrap(), 0x120);
    }

    #[test]
    fn test_mtvec_masks_reserved_mode_bit() {
        let mut vm = dummy_vm();

        vm.write_csr(0x305, 0b10).unwrap();

        assert_eq!(vm.read_csr(0x305).unwrap(), 0);
    }

    #[test]
    fn test_mstatus_masks_unsupported_bits() {
        let mut vm = dummy_vm();

        vm.write_csr(0x300, u32::MAX).unwrap();

        assert_eq!(vm.read_csr(0x300).unwrap(), 0x0001_9988);
    }

    #[test]
    fn test_mip_preserves_hardware_pending_bits() {
        let mut vm = dummy_vm();

        // bit 7 should remain controlled by hardware model
        vm.csr.mip = 1 << 7;

        vm.write_csr(0x344, 0).unwrap();

        assert_eq!(vm.read_csr(0x344).unwrap(), 1 << 7);
    }

    #[test]
    fn test_cycle_low_and_high_reads() {
        let mut vm = dummy_vm();

        vm.csr.cycle = 0x11223344_55667788;

        assert_eq!(vm.read_csr(0xC00).unwrap(), 0x55667788);
        assert_eq!(vm.read_csr(0xC80).unwrap(), 0x11223344);
    }

    #[test]
    fn test_instret_low_and_high_reads() {
        let mut vm = dummy_vm();

        vm.csr.instret = 0xAABBCCDD_EEFF0011;

        assert_eq!(vm.read_csr(0xC02).unwrap(), 0xEEFF0011);
        assert_eq!(vm.read_csr(0xC82).unwrap(), 0xAABBCCDD);
    }

    #[test]
    fn test_failed_csr_write_does_not_modify_other_state() {
        let mut vm = dummy_vm();

        vm.set_x(5, 42).unwrap();
        vm.write_csr(0x340, 99).unwrap();

        let before_x5 = vm.get_x(5).unwrap();
        let before_mscratch = vm.read_csr(0x340).unwrap();

        assert!(vm.write_csr(0x999, 1).is_err());

        assert_eq!(vm.get_x(5).unwrap(), before_x5);
        assert_eq!(vm.read_csr(0x340).unwrap(), before_mscratch);
    }

    // clint
    #[test]
    fn test_clint_mtimecmp_write_and_read() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_LO, 0x1234).unwrap();
        assert_eq!(vm.read_u32(MTIMECMP_LO).unwrap(), 0x1234);
    }

    #[test]
    fn test_clint_mtimecmp_high_write() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_HI, 0xABCD).unwrap();

        assert_eq!(vm.read_u32(MTIMECMP_HI).unwrap(), 0xABCD);
    }

    #[test]
    fn test_clint_does_not_touch_ram() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_LO, 0xFFFF).unwrap();

        // ensures that it did not accidentally write to RAM
        assert_ne!(vm.read_u32(0x0).unwrap_or(0), 0xFFFF);
    }

    #[test]
    fn test_clint_timer_pending_basic() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_HI, 0).unwrap();
        vm.write_u32(MTIMECMP_LO, 1).unwrap();

        vm.tick(); // mtime = 1

        println!("mtimecmp = {}", vm.clint.mtimecmp);
        println!("mtime = {}", vm.clint.mtime);

        assert!(vm.timer_pending());
    }

    #[test]
    fn test_mmio_precedence_over_ram() {
        let mut vm = dummy_vm();

        // se algum dia RAM overlap (defensivo)
        vm.write_u32(MTIMECMP_LO, 0xAAAA).unwrap();

        assert_eq!(vm.read_u32(MTIMECMP_LO).unwrap(), 0xAAAA);
    }

    #[test]
    fn test_clint_timer_not_pending_when_below() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_HI, 0).unwrap();
        vm.write_u32(MTIMECMP_LO, 10).unwrap();

        vm.tick(); // mtime = 1

        assert!(!vm.timer_pending());
    }

    #[test]
    fn test_clint_timer_pending_when_above() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_HI, 0).unwrap();
        vm.write_u32(MTIMECMP_LO, 1).unwrap();

        for _ in 0..5 {
            vm.tick();
        } // mtime = 5

        assert!(vm.timer_pending());
    }

    #[test]
    fn test_clint_timer_not_pending_large_mtimecmp() {
        let mut vm = dummy_vm();

        vm.write_u32(MTIMECMP_HI, 0x1).unwrap(); // mtimecmp = 0x1_0000_0000
        vm.write_u32(MTIMECMP_LO, 0x0).unwrap();

        for _ in 0..100 {
            vm.tick();
        }

        assert!(!vm.timer_pending());
    }

    #[test]
    fn test_clint_mtime_wraps_without_panic() {
        let mut vm = dummy_vm();
        vm.clint.mtime = u64::MAX;
        vm.tick(); // should be 0, without panic
        assert_eq!(vm.clint.mtime, 0);
    }
}
