//! # Control and Status Registers (CSRs)
//!
//! This module implements the RISC-V CSR file for machine-mode (`M-mode`)
//! operation, covering the subset required to run firmware such as FreeRTOS
//! on an RV32IM processor.
//!
//! ## Implemented registers
//!
//! | Address | Name        | Description                          |
//! |---------|-------------|--------------------------------------|
//! | `0x300` | `mstatus`   | Machine status                       |
//! | `0x301` | `misa`      | ISA and extensions (read-only)       |
//! | `0x304` | `mie`       | Machine interrupt enable             |
//! | `0x305` | `mtvec`     | Trap-handler base address            |
//! | `0x340` | `mscratch`  | Scratch register for trap handlers   |
//! | `0x341` | `mepc`      | Machine exception program counter    |
//! | `0x342` | `mcause`    | Machine trap cause                   |
//! | `0x344` | `mip`       | Machine interrupt pending            |
//! | `0xC00` | `cycle`     | Cycle counter (low 32 bits)          |
//! | `0xC80` | `cycleh`    | Cycle counter (high 32 bits)         |
//! | `0xC02` | `instret`   | Instructions retired (low 32 bits)   |
//! | `0xC82` | `instreth`  | Instructions retired (high 32 bits)  |
//! | `0xF14` | `mhartid`   | Hardware thread ID (read-only)       |
//!
//! ## Write semantics
//!
//! Certain registers enforce constraints on write:
//!
//! - **`mstatus`** — WPRI (reserved) bits are masked to zero via [`MSTATUS_MASK`]
//! - **`mtvec`** — bit 1 is reserved in direct mode and is always cleared
//! - **`mepc`** — bits 1:0 are always zero (instruction alignment)
//! - **`mip`** — the `MTIP` bit (bit 7) is read-only from software; it is
//!   driven exclusively by the CLINT hardware
//! - **`misa`**, **`mhartid`**, **`cycle`**, **`instret`** — read-only;
//!   write calls targeting these addresses return `false` and leave state unchanged

/// Writable bit mask for [`Csrs::mip`].
///
/// Bit 7 (`MTIP`) is driven by the CLINT and is not writable by software.
/// All other bits may be written freely.
pub const MIP_WRITABLE: u32 = !(1 << 7);

/// Write mask for [`Csrs::mstatus`].
///
/// Clears all WPRI (reserved) bits as required by the RISC-V privileged spec.
/// Only the following bits are preserved after a write:
///
/// - Bit 3 — `MIE` (Machine Interrupt Enable)
/// - Bit 7 — `MPIE` (Machine Previous Interrupt Enable)
/// - Bits 11:12 — `MPP` (Machine Previous Privilege)
pub const MSTATUS_MASK: u32 = 0x0001_9988;

/// The CSR file for a single RISC-V M-mode hart.
///
/// Holds all control and status registers needed for trap handling,
/// interrupt management, and performance counting.
///
/// # Usage
///
/// Access is always performed through [`Csrs::read`] and [`Csrs::write`],
/// which enforce the masking and read-only semantics defined by the spec.
/// Direct field access is available for internal emulator use (e.g. the CLINT
/// setting `mip.MTIP`).
///
/// # Example
///
/// ```rust
/// use riscv::vm::csr::Csrs;
/// let mut csrs = Csrs::default();
///
/// // Enable machine timer interrupts (MTIE = bit 7 of mie)
/// csrs.write(0x304, 1 << 7);
/// assert_eq!(csrs.read(0x304), Some(1 << 7));
///
/// // mhartid is read-only — writes are silently ignored
/// assert!(!csrs.write(0xF14, 0xFF));
/// assert_eq!(csrs.read(0xF14), Some(0));
/// ```
#[derive(Debug, Default)]
pub struct Csrs {
    /// Machine status register (`mstatus`, `0x300`).
    ///
    /// Controls global interrupt enable (`MIE`), previous interrupt state
    /// (`MPIE`), and previous privilege mode (`MPP`). WPRI bits are always
    /// masked to zero on write via [`MSTATUS_MASK`].
    pub mstatus: u32,

    /// ISA and extensions register (`misa`, `0x301`). Read-only.
    ///
    /// Encodes the supported ISA. Writes are silently ignored.
    pub misa: u32,

    /// Machine interrupt-enable register (`mie`, `0x304`).
    ///
    /// Each bit enables a specific interrupt source. The relevant bits for
    /// FreeRTOS are:
    /// - Bit 7 — `MTIE` (Machine Timer Interrupt Enable)
    pub mie: u32,

    /// Trap-handler base address register (`mtvec`, `0x305`).
    ///
    /// Holds the base address of the trap vector. Bit 1 is reserved in
    /// direct mode and is always cleared on write.
    pub mtvec: u32,

    /// Scratch register for machine trap handlers (`mscratch`, `0x340`).
    ///
    /// Conventionally used by the trap handler to save a register before
    /// swapping in the kernel stack pointer.
    pub mscratch: u32,

    /// Machine exception program counter (`mepc`, `0x341`).
    ///
    /// Holds the PC of the instruction that caused the trap. Bits 1:0 are
    /// always zero (instruction alignment is enforced on write).
    pub mepc: u32,

    /// Machine cause register (`mcause`, `0x342`).
    ///
    /// Encodes the reason for the most recent trap. The MSB distinguishes
    /// interrupts (1) from exceptions (0).
    pub mcause: u32,

    /// Machine interrupt-pending register (`mip`, `0x344`).
    ///
    /// Reflects which interrupts are currently pending. Bit 7 (`MTIP`) is
    /// set by the CLINT hardware and is read-only from software.
    pub mip: u32,

    /// Hardware thread ID (`mhartid`, `0xF14`). Read-only.
    ///
    /// Always `0` in a single-hart emulator. Writes are silently ignored.
    pub mhartid: u32,

    /// Cycle counter (`cycle`/`cycleh`, `0xC00`/`0xC80`). Read-only.
    ///
    /// Counts clock cycles since reset. Exposed as two 32-bit reads:
    /// low half at `0xC00`, high half at `0xC80`.
    pub cycle: u64,

    /// Instructions-retired counter (`instret`/`instreth`, `0xC02`/`0xC82`). Read-only.
    ///
    /// Incremented by the emulator after each successfully executed instruction.
    /// Exposed as two 32-bit reads: low half at `0xC02`, high half at `0xC82`.
    pub instret: u64,
}

impl Csrs {
    /// Reads a CSR by address.
    ///
    /// 64-bit counters (`cycle`, `instret`) are split across two addresses:
    /// the base address returns the low 32 bits and base `+ 0x80` returns
    /// the high 32 bits.
    ///
    /// # Parameters
    ///
    /// - `addr` — 12-bit CSR address as defined by the RISC-V privileged spec
    ///
    /// # Returns
    ///
    /// - `Some(value)` if the address maps to a supported register
    /// - `None` if the address is not implemented
    ///
    /// # Supported addresses
    ///
    /// See the [module-level register table](self).
    pub fn read(&self, addr: u16) -> Option<u32> {
        match addr {
            0x300 => Some(self.mstatus),
            0x301 => Some(self.misa),
            0x304 => Some(self.mie),
            0x305 => Some(self.mtvec),
            0x340 => Some(self.mscratch),
            0x341 => Some(self.mepc),
            0x342 => Some(self.mcause),
            0x344 => Some(self.mip),
            0xC00 => Some(self.cycle as u32),
            0xC80 => Some((self.cycle >> 32) as u32),
            0xC02 => Some(self.instret as u32),
            0xC82 => Some((self.instret >> 32) as u32),
            0xF14 => Some(self.mhartid),
            _ => None,
        }
    }

    /// Writes a value to a CSR by address.
    ///
    /// Write constraints are enforced automatically:
    ///
    /// - `mstatus` — WPRI bits are masked via [`MSTATUS_MASK`]
    /// - `mtvec` — bit 1 is cleared
    /// - `mepc` — bits 1:0 are cleared
    /// - `mip` — bit 7 (`MTIP`) is preserved from the current hardware state
    ///
    /// Read-only registers (`misa`, `mhartid`, `cycle`, `instret`) are not
    /// listed in the write match and will return `false`.
    ///
    /// # Parameters
    ///
    /// - `addr` — 12-bit CSR address
    /// - `value` — value to write
    ///
    /// # Returns
    ///
    /// - `true` if the address maps to a writable register
    /// - `false` if the address is not implemented or is read-only
    ///
    /// # Guarantees
    ///
    /// - If this method returns `false`, CSR state is unchanged
    pub fn write(&mut self, addr: u16, value: u32) -> bool {
        match addr {
            0x300 => self.mstatus = value & MSTATUS_MASK,
            0x304 => self.mie = value,
            0x305 => self.mtvec = value & !0b10,
            0x340 => self.mscratch = value,
            0x341 => self.mepc = value & !0b11,
            0x342 => self.mcause = value,
            0x344 => self.mip = (self.mip & !MIP_WRITABLE) | (value & MIP_WRITABLE),
            _ => return false,
        }
        true
    }
}
