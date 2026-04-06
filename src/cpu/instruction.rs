/// Represents a decoded RV32I instruction.
///
/// Each variant corresponds to a single RISC-V instruction after decoding.
/// The enum is produced by the CPU decode stage and later consumed by the
/// execution stage.
///
/// Register indices follow the RISC-V convention:
/// * `rd`  – destination register
/// * `rs1` – source register 1
/// * `rs2` – source register 2
///
/// Immediate values (`imm`) are already **sign-extended** to `i32`.
///
/// Shift amounts (`shamt`) are already masked to the valid range.
///
/// This enum only represents **RV32I base ISA** instructions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instruction {
    // =========================================================
    // R-type
    // =========================================================
    /// Add registers: `rd = rs1 + rs2`
    Add { rd: u8, rs1: u8, rs2: u8 },

    /// Subtract registers: `rd = rs1 - rs2`
    Sub { rd: u8, rs1: u8, rs2: u8 },

    /// Shift left logical: `rd = rs1 << rs2`
    Sll { rd: u8, rs1: u8, rs2: u8 },

    /// Set if less than (signed)
    Slt { rd: u8, rs1: u8, rs2: u8 },

    /// Set if less than (unsigned)
    Sltu { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise XOR
    Xor { rd: u8, rs1: u8, rs2: u8 },

    /// Shift right logical
    Srl { rd: u8, rs1: u8, rs2: u8 },

    /// Shift right arithmetic (sign-preserving)
    Sra { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise OR
    Or { rd: u8, rs1: u8, rs2: u8 },

    /// Bitwise AND
    And { rd: u8, rs1: u8, rs2: u8 },

    // =========================================================
    // I-type ALU
    // =========================================================
    /// Add immediate: `rd = rs1 + imm`
    Addi { rd: u8, rs1: u8, imm: i32 },

    /// Set if less than immediate (signed)
    Slti { rd: u8, rs1: u8, imm: i32 },

    /// Set if less than immediate (unsigned)
    Sltiu { rd: u8, rs1: u8, imm: i32 },

    /// XOR immediate
    Xori { rd: u8, rs1: u8, imm: i32 },

    /// OR immediate
    Ori { rd: u8, rs1: u8, imm: i32 },

    /// AND immediate
    Andi { rd: u8, rs1: u8, imm: i32 },

    /// Shift left logical immediate
    Slli { rd: u8, rs1: u8, shamt: u8 },

    /// Shift right logical immediate
    Srli { rd: u8, rs1: u8, shamt: u8 },

    /// Shift right arithmetic immediate
    Srai { rd: u8, rs1: u8, shamt: u8 },

    // =========================================================
    // Loads
    // =========================================================
    /// Load byte (sign-extended)
    Lb { rd: u8, rs1: u8, imm: i32 },

    /// Load halfword (sign-extended)
    Lh { rd: u8, rs1: u8, imm: i32 },

    /// Load word
    Lw { rd: u8, rs1: u8, imm: i32 },

    /// Load byte (zero-extended)
    Lbu { rd: u8, rs1: u8, imm: i32 },

    /// Load halfword (zero-extended)
    Lhu { rd: u8, rs1: u8, imm: i32 },

    // =========================================================
    // Stores
    // =========================================================
    /// Store byte
    Sb { rs1: u8, rs2: u8, imm: i32 },

    /// Store halfword
    Sh { rs1: u8, rs2: u8, imm: i32 },

    /// Store word
    Sw { rs1: u8, rs2: u8, imm: i32 },

    // =========================================================
    // Branches
    // =========================================================
    /// Branch if equal
    Beq { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if not equal
    Bne { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if less than (signed)
    Blt { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if greater or equal (signed)
    Bge { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if less than (unsigned)
    Bltu { rs1: u8, rs2: u8, imm: i32 },

    /// Branch if greater or equal (unsigned)
    Bgeu { rs1: u8, rs2: u8, imm: i32 },

    // =========================================================
    // U-type
    // =========================================================
    /// Load upper immediate
    Lui { rd: u8, imm: i32 },

    /// Add upper immediate to PC
    Auipc { rd: u8, imm: i32 },

    // =========================================================
    // Jumps
    // =========================================================
    /// Jump and link
    Jal { rd: u8, imm: i32 },

    /// Jump and link register
    Jalr { rd: u8, rs1: u8, imm: i32 },

    // =========================================================
    // System
    // =========================================================
    /// Environment call
    Ecall,

    /// Breakpoint
    Ebreak,

    // =========================================================
    // Memory ordering
    // =========================================================
    /// Memory fence
    Fence,

    /// Instruction cache fence
    FenceI,
}
