//! RV32IM instruction decoding and intermediate representation.
//!
//! This module defines the [`Instruction`] enum, a canonical, fully-decoded
//! representation of a RISC-V RV32IM instruction.
//!
//! # Overview
//!
//! The decode stage transforms a raw 32-bit instruction word into an
//! [`Instruction`] value. This representation is **lossless at the semantic
//! level** and eliminates all encoding-specific concerns (bitfields,
//! immediate reconstruction, masking).
//!
//! The execution stage (`CPU::execute`) operates exclusively on this enum,
//! treating it as a stable intermediate representation (IR) of the program.
//!
//! # Guarantees
//!
//! Decoding enforces the following invariants:
//!
//! - Register indices (`rd`, `rs1`, `rs2`) are always in the range `0..32`
//! - Immediates are fully reconstructed and sign-extended to `i32`
//! - Shift amounts are masked to 5 bits (`0..31`)
//! - Instruction variants map 1:1 to RV32IM operations
//!
//! As a result, the execution stage does not need to perform any additional
//! decoding, masking, or sign-extension.
//!
//! # Design notes
//!
//! - The enum acts as a boundary between decoding and execution
//! - All encoding-specific complexity is confined to the decode stage
//! - Execution logic can assume well-formed inputs
//!
//! # See also
//!
//! - [`crate::cpu::CPU`]
//! - [`crate::vm::VM`]

use std::fmt;

/// A decoded RV32IM instruction.
///
/// This enum represents the result of the decode stage for the
/// RISC-V Unprivileged ISA (RV32I). Each variant corresponds to a
/// single architectural instruction, with all operands and immediates
/// fully decoded and normalized for execution.
///
/// The execution stage (`CPU::execute`) consumes this representation
/// to update architectural state (register file, memory, and program counter).
///
/// # Execution model
///
/// Instruction semantics are fully defined by each variant.
///
/// - Register writes occur only when specified by the instruction
/// - Memory access occurs only in load/store instructions
/// - Program counter updates occur only in control-flow instructions
/// - The CPU advances PC by 4 for non-control-flow instructions
///
/// No additional or implicit side effects exist.
///
/// # Field conventions
///
/// | Field   | Type  | Meaning                                               |
/// |---------|-------|-------------------------------------------------------|
/// | `rd`    | `u8`  | Destination register index (0–31)                     |
/// | `rs1`   | `u8`  | Source register 1 index (0–31)                        |
/// | `rs2`   | `u8`  | Source register 2 index (0–31)                        |
/// | `imm`   | `i32` | Immediate value, sign-extended to 32 bits             |
/// | `shamt` | `u8`  | Shift amount, masked to bits `[4:0]`                  |
///
/// # Architectural invariants
///
/// - Register `x0` is hardwired to zero; writes to `rd = 0` are discarded.
/// - All arithmetic uses two's-complement wrapping semantics.
///
/// # Specification
///
/// Semantics follow the RISC-V Unprivileged ISA specification (RV32I).
///
/// # Coverage
///
/// This enum covers the complete RV32IM base ISA, including:
/// - Integer arithmetic and logical instructions
/// - Control flow (branches and jumps)
/// - Memory access (loads and stores)
/// - System (`ECALL`, `EBREAK`) and memory-ordering (`FENCE`, `FENCE.I`)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instruction {
    // =========================================================
    // R-type
    // =========================================================
    /// Integer addition.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] + x[rs2]`
    ///
    /// # Effects
    /// - Writes the 32-bit wrapping sum to register `rd`.
    ///
    /// # Notes
    /// - Overflow wraps silently (two's-complement modular arithmetic).
    Add { rd: u8, rs1: u8, rs2: u8 },

    /// Integer subtraction.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] - x[rs2]`
    ///
    /// # Effects
    /// - Writes the 32-bit wrapping difference to register `rd`.
    ///
    /// # Notes
    /// - Underflow wraps silently (two's-complement modular arithmetic).
    Sub { rd: u8, rs1: u8, rs2: u8 },

    /// Shift left logical (register).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] << (x[rs2] & 0x1f)`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` left by the **lower 5 bits** of `x[rs2]`, filling
    ///   vacated bits with zeros, and writes the result to register `rd`.
    ///
    /// # Notes
    /// - Only the five least-significant bits of `rs2` are used as the shift
    ///   amount; the upper bits are ignored.
    Sll { rd: u8, rs1: u8, rs2: u8 },

    /// Set less than (signed).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] as i32 < x[rs2] as i32) as u32`
    ///
    /// # Effects
    /// - Writes `1` to register `rd` if `x[rs1]` is strictly less than
    ///   `x[rs2]` under **signed** interpretation; writes `0` otherwise.
    Slt { rd: u8, rs1: u8, rs2: u8 },

    /// Set less than (unsigned).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] < x[rs2]) as u32`
    ///
    /// # Effects
    /// - Writes `1` to register `rd` if `x[rs1]` is strictly less than
    ///   `x[rs2]` under **unsigned** interpretation; writes `0` otherwise.
    ///
    /// # Notes
    /// - Differs from [`Instruction::Slt`] when operands have their sign bit set, e.g.
    ///   `0xFFFF_FFFF` is `-1` (signed) but `u32::MAX` (unsigned).
    Sltu { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise XOR (register).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] ^ x[rs2]`
    ///
    /// # Effects
    /// - Writes the bitwise exclusive-OR of `x[rs1]` and `x[rs2]` to
    ///   register `rd`.
    Xor { rd: u8, rs1: u8, rs2: u8 },

    /// Shift right logical (register).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] >> (x[rs2] & 0x1f)`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` right by the **lower 5 bits** of `x[rs2]`, filling
    ///   vacated bits with **zeros**, and writes the result to register `rd`.
    ///
    /// # Notes
    /// - Logical shift: the most-significant bit is filled with `0`
    ///   regardless of the sign of `x[rs1]`.  See [`Instruction::Sra`] for
    ///   sign-preserving right shift.
    Srl { rd: u8, rs1: u8, rs2: u8 },

    /// Shift right arithmetic (register).
    ///
    /// # Semantics
    /// `x[rd] = ((x[rs1] as i32) >> (x[rs2] & 0x1f)) as u32`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` right by the **lower 5 bits** of `x[rs2]`, filling
    ///   vacated bits with the **sign bit** of `x[rs1]`, and writes the
    ///   result to register `rd`.
    ///
    /// # Notes
    /// - Arithmetic shift: preserves the sign of the original value.
    ///   Contrast with [`Instruction::Srl`], which always fills with zeros.
    Sra { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise OR (register).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] | x[rs2]`
    ///
    /// # Effects
    /// - Writes the bitwise OR of `x[rs1]` and `x[rs2]` to register `rd`.
    Or { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise AND (register).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] & x[rs2]`
    ///
    /// # Effects
    /// - Writes the bitwise AND of `x[rs1]` and `x[rs2]` to register `rd`.
    And { rd: u8, rs1: u8, rs2: u8 },

    // =========================================================
    // I-type ALU
    // =========================================================
    /// Add immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] + imm`
    ///
    /// # Effects
    /// - Writes the 32-bit wrapping sum of `x[rs1]` and the sign-extended
    ///   12-bit immediate to register `rd`.
    ///
    /// # Notes
    /// - The canonical no-op `NOP` is encoded as `ADDI x0, x0, 0`.
    /// - Overflow wraps silently (two's-complement modular arithmetic).
    Addi { rd: u8, rs1: u8, imm: i32 },

    /// Set less than immediate (signed).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] as i32 < imm) as u32`
    ///
    /// # Effects
    /// - Writes `1` to register `rd` if `x[rs1]` is strictly less than the
    ///   sign-extended immediate under **signed** comparison; writes `0`
    ///   otherwise.
    Slti { rd: u8, rs1: u8, imm: i32 },

    /// Set less than immediate (unsigned).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] < imm as u32) as u32`
    ///
    /// # Effects
    /// - Writes `1` to register `rd` if `x[rs1]` is strictly less than the
    ///   immediate reinterpreted as an **unsigned** 32-bit value; writes `0`
    ///   otherwise.
    ///
    /// # Notes
    /// - The immediate is first sign-extended to 32 bits and then compared
    ///   as an unsigned value, so `SLTIU rd, rs1, 1` tests whether `x[rs1]`
    ///   is equal to zero.
    Sltiu { rd: u8, rs1: u8, imm: i32 },

    /// XOR immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] ^ imm as u32`
    ///
    /// # Effects
    /// - Writes the bitwise XOR of `x[rs1]` and the sign-extended immediate
    ///   to register `rd`.
    ///
    /// # Notes
    /// - `XORI rd, rs1, -1` (all-ones immediate) produces the bitwise NOT
    ///   of `x[rs1]`.
    Xori { rd: u8, rs1: u8, imm: i32 },

    /// OR immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] | imm as u32`
    ///
    /// # Effects
    /// - Writes the bitwise OR of `x[rs1]` and the sign-extended immediate
    ///   to register `rd`.
    Ori { rd: u8, rs1: u8, imm: i32 },

    /// AND immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] & imm as u32`
    ///
    /// # Effects
    /// - Writes the bitwise AND of `x[rs1]` and the sign-extended immediate
    ///   to register `rd`.
    Andi { rd: u8, rs1: u8, imm: i32 },

    /// Shift left logical immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] << (shamt & 0x1f)`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` left by `shamt` bit positions, filling vacated
    ///   bits with zeros, and writes the result to register `rd`.
    ///
    /// # Notes
    /// - `shamt` is pre-masked to bits `[4:0]` during decoding; values
    ///   above 31 are not representable in the encoding.
    Slli { rd: u8, rs1: u8, shamt: u8 },

    /// Shift right logical immediate.
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] >> (shamt & 0x1f)`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` right by `shamt` bit positions, filling vacated
    ///   bits with **zeros**, and writes the result to register `rd`.
    Srli { rd: u8, rs1: u8, shamt: u8 },

    /// Shift right arithmetic immediate.
    ///
    /// # Semantics
    /// `x[rd] = ((x[rs1] as i32) >> (shamt & 0x1f)) as u32`
    ///
    /// # Effects
    /// - Shifts `x[rs1]` right by `shamt` bit positions, filling vacated
    ///   bits with the **sign bit** of `x[rs1]`, and writes the result to
    ///   register `rd`.
    Srai { rd: u8, rs1: u8, shamt: u8 },

    // =========================================================
    // Loads
    // =========================================================
    /// Load byte (sign-extended).
    ///
    /// # Semantics
    /// `x[rd] = sign_extend(mem[x[rs1] + imm][7:0])`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Reads 1 byte from memory at that address.
    /// - Sign-extends the byte to 32 bits and writes the result to register
    ///   `rd`.
    Lb { rd: u8, rs1: u8, imm: i32 },

    /// Load halfword (sign-extended).
    ///
    /// # Semantics
    /// `x[rd] = sign_extend(mem[x[rs1] + imm][15:0])`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Reads 2 bytes (a halfword) from memory at that address.
    /// - Sign-extends the 16-bit value to 32 bits and writes the result to
    ///   register `rd`.
    Lh { rd: u8, rs1: u8, imm: i32 },

    /// Load word.
    ///
    /// # Semantics
    /// `x[rd] = mem[x[rs1] + imm][31:0]`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Reads 4 bytes (a full word) from memory at that address.
    /// - Writes the 32-bit value directly to register `rd`; no
    ///   sign-extension is needed.
    Lw { rd: u8, rs1: u8, imm: i32 },

    /// Load byte (zero-extended).
    ///
    /// # Semantics
    /// `x[rd] = zero_extend(mem[x[rs1] + imm][7:0])`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Reads 1 byte from memory at that address.
    /// - **Zero-extends** the byte to 32 bits (upper 24 bits are cleared)
    ///   and writes the result to register `rd`.
    ///
    /// # Notes
    /// - Differs from [`Instruction::Lb`] only in the extension strategy: `LBU` always
    ///   produces a non-negative value in `rd`.
    Lbu { rd: u8, rs1: u8, imm: i32 },

    /// Load halfword (zero-extended).
    ///
    /// # Semantics
    /// `x[rd] = zero_extend(mem[x[rs1] + imm][15:0])`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Reads 2 bytes from memory at that address.
    /// - **Zero-extends** the halfword to 32 bits (upper 16 bits are
    ///   cleared) and writes the result to register `rd`.
    ///
    /// # Notes
    /// - Differs from [`Instruction::Lh`] only in the extension strategy: `LHU` always
    ///   produces a non-negative value in `rd`.
    Lhu { rd: u8, rs1: u8, imm: i32 },

    // =========================================================
    // Stores
    // =========================================================
    /// Store byte.
    ///
    /// # Semantics
    /// `mem[x[rs1] + imm] = x[rs2][7:0]`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Writes the **least-significant byte** of `x[rs2]` to memory at
    ///   that address.
    /// - Does not modify any register.
    Sb { rs1: u8, rs2: u8, imm: i32 },

    /// Store halfword.
    ///
    /// # Semantics
    /// `mem[x[rs1] + imm] = x[rs2][15:0]`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Writes the **least-significant halfword** (2 bytes) of `x[rs2]`
    ///   to memory at that address.
    /// - Does not modify any register.
    Sh { rs1: u8, rs2: u8, imm: i32 },

    /// Store word.
    ///
    /// # Semantics
    /// `mem[x[rs1] + imm] = x[rs2][31:0]`
    ///
    /// # Effects
    /// - Computes the effective address `x[rs1] + imm` using wrapping
    ///   addition.
    /// - Writes all 4 bytes of `x[rs2]` to memory at that address.
    /// - Does not modify any register.
    Sw { rs1: u8, rs2: u8, imm: i32 },

    // =========================================================
    // Branches
    // =========================================================
    /// Branch if equal.
    ///
    /// # Semantics
    /// `if x[rs1] == x[rs2] { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` for equality.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified; the caller advances it by 4.
    /// - Does not access memory.
    /// - Does not modify any register.
    Beq { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if not equal.
    ///
    /// # Semantics
    /// `if x[rs1] != x[rs2] { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` for inequality.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified.
    /// - Does not access memory.
    /// - Does not modify any register.
    Bne { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if less than (signed).
    ///
    /// # Semantics
    /// `if (x[rs1] as i32) < (x[rs2] as i32) { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` as **signed** 32-bit integers.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified.
    /// - Does not access memory.
    /// - Does not modify any register.
    Blt { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if greater than or equal (signed).
    ///
    /// # Semantics
    /// `if (x[rs1] as i32) >= (x[rs2] as i32) { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` as **signed** 32-bit integers.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified.
    /// - Does not access memory.
    /// - Does not modify any register.
    Bge { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if less than (unsigned).
    ///
    /// # Semantics
    /// `if x[rs1] < x[rs2] { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` as **unsigned** 32-bit integers.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified.
    /// - Does not access memory.
    /// - Does not modify any register.
    Bltu { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if greater than or equal (unsigned).
    ///
    /// # Semantics
    /// `if x[rs1] >= x[rs2] { PC = PC + imm }`
    ///
    /// # Effects
    /// - Compares `x[rs1]` and `x[rs2]` as **unsigned** 32-bit integers.
    /// - **Taken**: sets the PC to `PC + imm` (PC-relative, wrapping).
    /// - **Not taken**: PC is left unmodified.
    /// - Does not access memory.
    /// - Does not modify any register.
    Bgeu { rs1: u8, rs2: u8, imm: i32 },

    // =========================================================
    // U-type
    // =========================================================
    /// Load upper immediate.
    ///
    /// # Semantics
    /// `x[rd] = imm`
    ///
    /// # Effects
    /// - Writes the 20-bit upper immediate (already shifted left by 12
    ///   during encoding, stored here as a full `i32`) directly to register
    ///   `rd` as a `u32`.
    ///
    /// # Notes
    /// - Typically paired with [`Instruction::Addi`] to materialize a full 32-bit
    ///   constant: `LUI` loads the upper 20 bits, `ADDI` fills in the lower
    ///   12 bits.
    Lui { rd: u8, imm: i32 },

    /// Add upper immediate to PC.
    ///
    /// # Semantics
    /// `x[rd] = PC + imm`
    ///
    /// # Effects
    /// - Adds the upper immediate to the **current PC** (the address of
    ///   this instruction) using wrapping addition and writes the result to
    ///   register `rd`.
    ///
    /// # Notes
    /// - Useful for computing PC-relative addresses, e.g. for position-
    ///   independent code or to call nearby functions via `AUIPC` + `JALR`.
    Auipc { rd: u8, imm: i32 },

    // =========================================================
    // Jumps
    // =========================================================
    /// Jump and link.
    ///
    /// # Semantics
    /// `x[rd] = PC + 4; PC = PC + imm`
    ///
    /// # Effects
    /// - Saves the **return address** (`PC + 4`) to register `rd`.
    /// - Jumps unconditionally to `PC + imm` (PC-relative, wrapping).
    /// - Does not access memory.
    ///
    /// # Notes
    /// - When `rd = x0` the return address is discarded; this encodes an
    ///   unconditional jump without a link.
    /// - The 20-bit immediate allows a ±1 MiB PC-relative range.
    Jal { rd: u8, imm: i32 },

    /// Jump and link register.
    ///
    /// # Semantics
    /// `x[rd] = PC + 4; PC = (x[rs1] + imm) & !1`
    ///
    /// # Effects
    /// - Saves the **return address** (`PC + 4`) to register `rd`.
    /// - Computes the target as `x[rs1] + imm` (wrapping) and **clears
    ///   bit 0** to ensure 2-byte alignment, then sets the PC to that
    ///   target.
    /// - Does not access memory.
    ///
    /// # Notes
    /// - Clearing bit 0 is mandated by the RISC-V spec to support
    ///   compressed-instruction alignment; it is always applied even if the
    ///   C extension is absent.
    /// - `JALR x0, x1, 0` is the canonical function-return sequence.
    Jalr { rd: u8, rs1: u8, imm: i32 },

    // =========================================================
    // System / CSR
    // =========================================================
    /// Environment call.
    ///
    /// # Semantics
    /// Transfers control to the execution environment (OS / emulator) to
    /// request a service.
    ///
    /// # Effects
    /// - Reads the **syscall number** from register `x17` (`a7`).
    /// - Dispatches to the appropriate handler:
    ///   - **Syscall 93 (`exit`)**: reads the exit code from `x10` (`a0`),
    ///     stores it via `set_exit_code`, and halts the CPU (`set_running(false)`).
    ///   - **Any other number**: returns [`crate::cpu::CPUError::UnsupportedSyscall`].
    /// - Does not modify the PC directly (the CPU stops or errors before
    ///   the normal PC-advance step).
    ///
    /// # Errors
    /// Returns [`crate::cpu::CPUError::UnsupportedSyscall`] when the syscall number in
    /// `x17` is not recognised by the emulator.
    Ecall,

    /// Breakpoint.
    ///
    /// # Semantics
    /// Signals the execution environment that a breakpoint has been reached.
    ///
    /// # Effects
    /// - Currently a **no-op** in this emulator (pending a debugger
    ///   integration).
    /// - Does not modify any register, memory, or the PC.
    ///
    /// # Notes
    /// - Future implementations may pause execution and invoke a debugger
    ///   callback instead of silently continuing.
    Ebreak,

    /// CSR Read/Write (register form).
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = x[rs1]`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Reads the current CSR value
    /// - Writes register `x[rs1]` into CSR
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - If `rd == x0`, the result is discarded
    /// - Operation is atomic at architectural level
    Csrrw { rd: u8, rs1: u8, csr: u16 },

    /// CSR Read and Set Bits (register form).
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = t | x[rs1]`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Sets bits in CSR using `x[rs1]` as mask
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - If `rs1 == x0`, CSR is only read (no modification)
    Csrrs { rd: u8, rs1: u8, csr: u16 },

    /// CSR Read and Clear Bits (register form).
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = t & ~x[rs1]`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Clears bits in CSR using `x[rs1]` as mask
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - If `rs1 == x0`, CSR is only read (no modification)
    Csrrc { rd: u8, rs1: u8, csr: u16 },

    /// CSR Read/Write Immediate.
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = zimm`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Writes immediate value into CSR
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - Immediate is zero-extended (5-bit)
    Csrrwi { rd: u8, zimm: u8, csr: u16 },

    /// CSR Read and Set Bits Immediate.
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = t | zimm`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Sets bits in CSR using immediate mask
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - If `zimm == 0`, CSR is only read
    Csrrsi { rd: u8, zimm: u8, csr: u16 },

    /// CSR Read and Clear Bits Immediate.
    ///
    /// # Semantics
    /// `t = CSR[csr]`  
    /// `CSR[csr] = t & ~zimm`  
    /// `x[rd] = t`
    ///
    /// # Effects
    /// - Clears bits in CSR using immediate mask
    /// - Returns previous CSR value in `rd`
    ///
    /// # Notes
    /// - If `zimm == 0`, CSR is only read
    Csrrci { rd: u8, zimm: u8, csr: u16 },

    /// Machine Return from Trap.
    ///
    /// # Semantics
    /// `pc = mepc`
    /// `mstatus.MIE = mstatus.MPIE`
    /// `mstatus.MPIE = 1`
    /// `mstatus.MPP = U-mode (0b00)`
    ///
    /// # Effects
    /// - Restores program counter from exception return address
    /// - Restores interrupt enable state
    /// - Exits trap handler context
    ///
    /// # Notes
    /// - This instruction is only valid in Machine Mode
    /// - Critical for returning from interrupts (e.g. FreeRTOS tick handler)
    Mret,

    // =========================================================
    // Memory ordering
    // =========================================================
    /// Memory fence.
    ///
    /// # Semantics
    /// Orders memory I/O accesses visible to other hardware threads or
    /// devices.
    ///
    /// # Effects
    /// - **No-op** in this single-threaded in-order emulator.
    /// - Does not modify any register, memory, or the PC.
    ///
    /// # Notes
    /// - On a real multi-core RISC-V implementation, `FENCE` enforces
    ///   ordering of predecessor memory operations before successor ones.
    ///   In a single-threaded emulator, the sequential execution model
    ///   makes this guarantee trivially true.
    Fence,

    /// Instruction cache fence.
    ///
    /// # Semantics
    /// Ensures that any stores to instruction memory are visible to
    /// subsequent instruction fetches.
    ///
    /// # Effects
    /// - **No-op** in this emulator; instruction memory is always coherent
    ///   because there is no separate instruction cache.
    /// - Does not modify any register, memory, or the PC.
    ///
    /// # Notes
    /// - Required on real hardware after self-modifying code or dynamic
    ///   code generation before the new instructions are executed.
    FenceI,

    // =========================================================
    // RV32M extension
    // =========================================================
    /// Integer multiply (low word).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] * x[rs2])[31:0]`
    ///
    /// # Effects
    /// - Multiplies `x[rs1]` and `x[rs2]` as 32-bit integers.
    /// - Writes the least-significant 32 bits of the 64-bit product to `rd`.
    ///
    /// # Notes
    /// - Equivalent low-word result for signed and unsigned operands.
    /// - Upper 32 bits are discarded.
    Mul { rd: u8, rs1: u8, rs2: u8 },

    /// Integer multiply high (signed × signed).
    ///
    /// # Semantics
    /// `x[rd] = ((x[rs1] as i32 * x[rs2] as i32) >> 32)`
    ///
    /// # Effects
    /// - Multiplies both operands as signed 32-bit integers.
    /// - Writes the most-significant 32 bits of the 64-bit product to `rd`.
    Mulh { rd: u8, rs1: u8, rs2: u8 },

    /// Integer multiply high (signed × unsigned).
    ///
    /// # Semantics
    /// `x[rd] = ((x[rs1] as i32 * x[rs2]) >> 32)`
    ///
    /// # Effects
    /// - Multiplies `x[rs1]` as signed and `x[rs2]` as unsigned.
    /// - Writes the most-significant 32 bits of the 64-bit product to `rd`.
    Mulhsu { rd: u8, rs1: u8, rs2: u8 },

    /// Integer multiply high (unsigned × unsigned).
    ///
    /// # Semantics
    /// `x[rd] = ((x[rs1] * x[rs2]) >> 32)`
    ///
    /// # Effects
    /// - Multiplies both operands as unsigned 32-bit integers.
    /// - Writes the most-significant 32 bits of the 64-bit product to `rd`.
    Mulhu { rd: u8, rs1: u8, rs2: u8 },

    /// Integer division (signed).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] as i32) / (x[rs2] as i32)`
    ///
    /// # Effects
    /// - Divides `x[rs1]` by `x[rs2]` using signed division.
    /// - Quotient is truncated toward zero.
    /// - Writes the result to `rd`.
    ///
    /// # Special cases
    /// - If `x[rs2] == 0`, writes `0xFFFF_FFFF`.
    /// - If `x[rs1] == INT32_MIN` and `x[rs2] == -1`, writes `INT32_MIN`.
    Div { rd: u8, rs1: u8, rs2: u8 },

    /// Integer division (unsigned).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] / x[rs2]`
    ///
    /// # Effects
    /// - Divides operands as unsigned 32-bit integers.
    /// - Writes the quotient to `rd`.
    ///
    /// # Special cases
    /// - If `x[rs2] == 0`, writes `0xFFFF_FFFF`.
    Divu { rd: u8, rs1: u8, rs2: u8 },

    /// Integer remainder (signed).
    ///
    /// # Semantics
    /// `x[rd] = (x[rs1] as i32) % (x[rs2] as i32)`
    ///
    /// # Effects
    /// - Computes the signed remainder of `x[rs1] / x[rs2]`.
    /// - Writes the result to `rd`.
    ///
    /// # Special cases
    /// - If `x[rs2] == 0`, writes `x[rs1]`.
    /// - If `x[rs1] == INT32_MIN` and `x[rs2] == -1`, writes `0`.
    Rem { rd: u8, rs1: u8, rs2: u8 },

    /// Integer remainder (unsigned).
    ///
    /// # Semantics
    /// `x[rd] = x[rs1] % x[rs2]`
    ///
    /// # Effects
    /// - Computes the unsigned remainder of `x[rs1] / x[rs2]`.
    /// - Writes the result to `rd`.
    ///
    /// # Special cases
    /// - If `x[rs2] == 0`, writes `x[rs1]`.
    Remu { rd: u8, rs1: u8, rs2: u8 },
}

impl Instruction {
    fn fmt_asm(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Instruction::*;

        match *self {
            // =========================
            // R-type
            // =========================
            Add { rd, rs1, rs2 } => r(f, "add", rd, rs1, rs2),
            Sub { rd, rs1, rs2 } => r(f, "sub", rd, rs1, rs2),
            Sll { rd, rs1, rs2 } => r(f, "sll", rd, rs1, rs2),
            Slt { rd, rs1, rs2 } => r(f, "slt", rd, rs1, rs2),
            Sltu { rd, rs1, rs2 } => r(f, "sltu", rd, rs1, rs2),
            Xor { rd, rs1, rs2 } => r(f, "xor", rd, rs1, rs2),
            Srl { rd, rs1, rs2 } => r(f, "srl", rd, rs1, rs2),
            Sra { rd, rs1, rs2 } => r(f, "sra", rd, rs1, rs2),
            Or { rd, rs1, rs2 } => r(f, "or", rd, rs1, rs2),
            And { rd, rs1, rs2 } => r(f, "and", rd, rs1, rs2),

            // =========================
            // RV32M
            // =========================
            Mul { rd, rs1, rs2 } => r(f, "mul", rd, rs1, rs2),
            Mulh { rd, rs1, rs2 } => r(f, "mulh", rd, rs1, rs2),
            Mulhsu { rd, rs1, rs2 } => r(f, "mulhsu", rd, rs1, rs2),
            Mulhu { rd, rs1, rs2 } => r(f, "mulhu", rd, rs1, rs2),
            Div { rd, rs1, rs2 } => r(f, "div", rd, rs1, rs2),
            Divu { rd, rs1, rs2 } => r(f, "divu", rd, rs1, rs2),
            Rem { rd, rs1, rs2 } => r(f, "rem", rd, rs1, rs2),
            Remu { rd, rs1, rs2 } => r(f, "remu", rd, rs1, rs2),

            // =========================
            // I-type ALU
            // =========================
            Addi { rd, rs1, imm } => i(f, "addi", rd, rs1, imm),
            Slti { rd, rs1, imm } => i(f, "slti", rd, rs1, imm),
            Sltiu { rd, rs1, imm } => i(f, "sltiu", rd, rs1, imm),
            Xori { rd, rs1, imm } => i(f, "xori", rd, rs1, imm),
            Ori { rd, rs1, imm } => i(f, "ori", rd, rs1, imm),
            Andi { rd, rs1, imm } => i(f, "andi", rd, rs1, imm),

            Slli { rd, rs1, shamt } => ish(f, "slli", rd, rs1, shamt),
            Srli { rd, rs1, shamt } => ish(f, "srli", rd, rs1, shamt),
            Srai { rd, rs1, shamt } => ish(f, "srai", rd, rs1, shamt),

            // =========================
            // Loads
            // =========================
            Lb { rd, rs1, imm } => load(f, "lb", rd, rs1, imm),
            Lh { rd, rs1, imm } => load(f, "lh", rd, rs1, imm),
            Lw { rd, rs1, imm } => load(f, "lw", rd, rs1, imm),
            Lbu { rd, rs1, imm } => load(f, "lbu", rd, rs1, imm),
            Lhu { rd, rs1, imm } => load(f, "lhu", rd, rs1, imm),

            // =========================
            // Stores
            // =========================
            Sb { rs1, rs2, imm } => store(f, "sb", rs1, rs2, imm),
            Sh { rs1, rs2, imm } => store(f, "sh", rs1, rs2, imm),
            Sw { rs1, rs2, imm } => store(f, "sw", rs1, rs2, imm),

            // =========================
            // Branches
            // =========================
            Beq { rs1, rs2, imm } => branch(f, "beq", rs1, rs2, imm),
            Bne { rs1, rs2, imm } => branch(f, "bne", rs1, rs2, imm),
            Blt { rs1, rs2, imm } => branch(f, "blt", rs1, rs2, imm),
            Bge { rs1, rs2, imm } => branch(f, "bge", rs1, rs2, imm),
            Bltu { rs1, rs2, imm } => branch(f, "bltu", rs1, rs2, imm),
            Bgeu { rs1, rs2, imm } => branch(f, "bgeu", rs1, rs2, imm),

            // =========================
            // U-type
            // =========================
            Lui { rd, imm } => u(f, "lui", rd, imm),
            Auipc { rd, imm } => u(f, "auipc", rd, imm),

            // =========================
            // Jumps
            // =========================
            Jal { rd, imm } => u(f, "jal", rd, imm),
            Jalr { rd, rs1, imm } => jalr(f, rd, rs1, imm),

            // =========================
            // System / CSR
            // =========================
            Ecall => f.write_str("ecall"),
            Ebreak => f.write_str("ebreak"),
            Csrrs { rd, rs1, csr } => csr_r(f, "csrrs", rd, rs1, csr),
            Csrrc { rd, rs1, csr } => csr_r(f, "csrrc", rd, rs1, csr),
            Csrrw { rd, rs1, csr } => csr_r(f, "csrrw", rd, rs1, csr),
            Csrrsi { rd, zimm, csr } => csr_i(f, "csrrsi", rd, zimm, csr),
            Csrrci { rd, zimm, csr } => csr_i(f, "csrrci", rd, zimm, csr),
            Csrrwi { rd, zimm, csr } => csr_i(f, "csrrwi", rd, zimm, csr),
            Mret => write!(f, "mret"),

            // =========================
            // Fence
            // =========================
            Fence => f.write_str("fence"),
            FenceI => f.write_str("fence.i"),
        }
    }
}

impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.fmt_asm(f)
    }
}

// helpers
#[inline(always)]
fn r(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, rs2: u8) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, x{rs2}")
}

#[inline(always)]
fn i(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, imm: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, {imm}")
}

#[inline(always)]
fn ish(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, shamt: u8) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, {shamt}")
}

#[inline(always)]
fn load(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, base: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, {off}(x{base})")
}

#[inline(always)]
fn store(f: &mut fmt::Formatter<'_>, op: &str, base: u8, src: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{src}, {off}(x{base})")
}

#[inline(always)]
fn branch(f: &mut fmt::Formatter<'_>, op: &str, rs1: u8, rs2: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{rs1}, x{rs2}, {off}")
}

#[inline(always)]
fn u(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, imm: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, {imm}")
}

#[inline(always)]
fn jalr(f: &mut fmt::Formatter<'_>, rd: u8, rs1: u8, imm: i32) -> fmt::Result {
    write!(f, "jalr x{rd}, {imm}(x{rs1})")
}

fn csr_r(f: &mut fmt::Formatter<'_>, name: &str, rd: u8, rs1: u8, csr: u16) -> fmt::Result {
    write!(f, "{} x{}, x{}, 0x{:x}", name, rd, rs1, csr)
}

fn csr_i(f: &mut fmt::Formatter<'_>, name: &str, rd: u8, zimm: u8, csr: u16) -> fmt::Result {
    write!(f, "{} x{}, {}, 0x{:x}", name, rd, zimm, csr)
}
