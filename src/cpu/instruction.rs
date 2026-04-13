use std::fmt;


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

impl Instruction {
    fn fmt_asm(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Instruction::*;

        match *self {
            // =========================
            // R-type
            // =========================
            Add  { rd, rs1, rs2 } => r(f, "add",  rd, rs1, rs2),
            Sub  { rd, rs1, rs2 } => r(f, "sub",  rd, rs1, rs2),
            Sll  { rd, rs1, rs2 } => r(f, "sll",  rd, rs1, rs2),
            Slt  { rd, rs1, rs2 } => r(f, "slt",  rd, rs1, rs2),
            Sltu { rd, rs1, rs2 } => r(f, "sltu", rd, rs1, rs2),
            Xor  { rd, rs1, rs2 } => r(f, "xor",  rd, rs1, rs2),
            Srl  { rd, rs1, rs2 } => r(f, "srl",  rd, rs1, rs2),
            Sra  { rd, rs1, rs2 } => r(f, "sra",  rd, rs1, rs2),
            Or   { rd, rs1, rs2 } => r(f, "or",   rd, rs1, rs2),
            And  { rd, rs1, rs2 } => r(f, "and",  rd, rs1, rs2),

            // =========================
            // I-type ALU
            // =========================
            Addi  { rd, rs1, imm } => i(f, "addi",  rd, rs1, imm),
            Slti  { rd, rs1, imm } => i(f, "slti",  rd, rs1, imm),
            Sltiu { rd, rs1, imm } => i(f, "sltiu", rd, rs1, imm),
            Xori  { rd, rs1, imm } => i(f, "xori",  rd, rs1, imm),
            Ori   { rd, rs1, imm } => i(f, "ori",   rd, rs1, imm),
            Andi  { rd, rs1, imm } => i(f, "andi",  rd, rs1, imm),

            Slli { rd, rs1, shamt } => ish(f, "slli", rd, rs1, shamt),
            Srli { rd, rs1, shamt } => ish(f, "srli", rd, rs1, shamt),
            Srai { rd, rs1, shamt } => ish(f, "srai", rd, rs1, shamt),

            // =========================
            // Loads
            // =========================
            Lb  { rd, rs1, imm } => load(f, "lb",  rd, rs1, imm),
            Lh  { rd, rs1, imm } => load(f, "lh",  rd, rs1, imm),
            Lw  { rd, rs1, imm } => load(f, "lw",  rd, rs1, imm),
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
            Beq  { rs1, rs2, imm } => branch(f, "beq",  rs1, rs2, imm),
            Bne  { rs1, rs2, imm } => branch(f, "bne",  rs1, rs2, imm),
            Blt  { rs1, rs2, imm } => branch(f, "blt",  rs1, rs2, imm),
            Bge  { rs1, rs2, imm } => branch(f, "bge",  rs1, rs2, imm),
            Bltu { rs1, rs2, imm } => branch(f, "bltu", rs1, rs2, imm),
            Bgeu { rs1, rs2, imm } => branch(f, "bgeu", rs1, rs2, imm),

            // =========================
            // U-type
            // =========================
            Lui   { rd, imm } => u(f, "lui",   rd, imm),
            Auipc { rd, imm } => u(f, "auipc", rd, imm),

            // =========================
            // Jumps
            // =========================
            Jal  { rd, imm } => u(f, "jal", rd, imm),
            Jalr { rd, rs1, imm } => jalr(f, rd, rs1, imm),

            // =========================
            // System
            // =========================
            Ecall  => f.write_str("ecall"),
            Ebreak => f.write_str("ebreak"),

            // =========================
            // Fence
            // =========================
            Fence  => f.write_str("fence"),
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