//! RV32IMA disassembler — objdump-compatible output.
//!
//! Converts a 32-bit instruction word into a human-readable string using
//! ABI register names and common pseudo-instructions. When a [`SymbolTable`]
//! is provided, branch and jump targets are annotated with symbol names.
//!
//! ## Output format
//!
//! The format matches GNU objdump (`-M no-aliases` disabled, i.e. pseudos on):
//!
//! ```text
//! 80000014  510010ef  jal     ra, <main>
//! 80001524  ff010113  addi    sp, sp, -16
//! 80001528  00112623  sw      ra, 12(sp)
//! ```
//!
//! ## Pseudo-instructions
//!
//! The following common pseudos are emitted when the encoding matches:
//!
//! | Pseudo | Canonical form |
//! |--------|---------------|
//! | `nop` | `addi zero, zero, 0` |
//! | `mv rd, rs` | `addi rd, rs, 0` |
//! | `li rd, imm` | `addi rd, zero, imm` |
//! | `not rd, rs` | `xori rd, rs, -1` |
//! | `neg rd, rs` | `sub rd, zero, rs` |
//! | `snez rd, rs` | `sltu rd, zero, rs` |
//! | `ret` | `jalr zero, 0(ra)` |
//! | `jr rs` | `jalr zero, 0(rs)` |
//! | `j offset` | `jal zero, offset` |
//! | `beqz/bnez/bltz/bgez` | branch with zero as one operand |
//!
//! ## Static disassembly
//!
//! [`disasm_elf`] disassembles an entire ELF image segment by segment, with
//! symbol labels printed before each function — equivalent to `objdump -d`.

use crate::elf::{ElfImage, SymbolTable};

// ── ABI register names ───────────────────────────────────────────────────────

const REG_NAMES: [&str; 32] = [
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3", "a4",
    "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4",
    "t5", "t6",
];

/// Return the ABI name of register `r` (masked to 5 bits).
#[inline]
fn reg(r: u32) -> &'static str {
    REG_NAMES[(r & 0x1f) as usize]
}

// ── Immediate decoders (pure functions, same logic as emulator.rs) ────────────

#[inline]
fn imm_i(ir: u32) -> i32 {
    let imm = ir >> 20;
    (imm | if imm & 0x800 != 0 { 0xffff_f000 } else { 0 }) as i32
}
#[inline]
fn imm_s(ir: u32) -> i32 {
    let imm = ((ir >> 7) & 0x1f) | ((ir & 0xfe00_0000) >> 20);
    (imm | if imm & 0x800 != 0 { 0xffff_f000 } else { 0 }) as i32
}
#[inline]
fn imm_b(ir: u32) -> i32 {
    let imm =
        ((ir & 0xf00) >> 7) | ((ir & 0x7e00_0000) >> 20) | ((ir & 0x80) << 4) | ((ir >> 31) << 12);
    (imm | if imm & 0x1000 != 0 { 0xffffe000 } else { 0 }) as i32
}
#[inline]
fn imm_j(ir: u32) -> i32 {
    let imm = ((ir & 0x8000_0000) >> 11)
        | ((ir & 0x7fe0_0000) >> 20)
        | ((ir & 0x0010_0000) >> 9)
        | (ir & 0x000f_f000);
    (imm | if imm & 0x0010_0000 != 0 {
        0xffe0_0000
    } else {
        0
    }) as i32
}
#[inline]
fn imm_u(ir: u32) -> i32 {
    (ir & 0xffff_f000) as i32
}

// ── Formatting helpers ───────────────────────────────────────────────────────

/// Format a branch/jump target address, annotating it with a symbol name if available.
///
/// - Exact match (`addr == sym.addr`): `<name>`
/// - Inside a symbol's range (`size > 0`): `<name+0xoffset>`
/// - No match or `size == 0` without exact match: `0x80001234`
fn fmt_target(addr: u32, syms: Option<&SymbolTable>) -> String {
    if let Some(sym) = syms.and_then(|t| t.lookup_addr(addr)) {
        if sym.addr == addr {
            // Exact match — always show the symbol name.
            format!("<{}>", sym.name)
        } else if sym.size > 0 {
            // Inside a known range — offset is reliable.
            format!("<{}+0x{:x}>", sym.name, addr - sym.addr)
        } else {
            // size == 0: we do not know where the symbol ends;
            // a numeric address is more honest than a made-up offset.
            format!("0x{:08x}", addr)
        }
    } else {
        format!("0x{:08x}", addr)
    }
}

/// Format a memory operand as `imm(rs)` for load/store instructions.
#[inline]
fn mem_operand(rs: u32, imm: i32) -> String {
    format!("{}({})", imm, reg(rs))
}

// ── Main disassembler ────────────────────────────────────────────────────────

/// Disassemble one RV32IMA instruction into an objdump-compatible string.
///
/// - `ir`   — the 32-bit instruction word.
/// - `pc`   — virtual address of the instruction, used to compute branch and
///   jump targets relative to PC.
/// - `syms` — optional symbol table for resolving target addresses to names.
///
/// Returns a string containing the mnemonic and operands, with no leading
/// address and no trailing newline. Example: `"jal     ra, <vTaskStartScheduler>"`.
pub fn disassemble(ir: u32, pc: u32, syms: Option<&SymbolTable>) -> String {
    let rd = (ir >> 7) & 0x1f;
    let rs1 = (ir >> 15) & 0x1f;
    let rs2 = (ir >> 20) & 0x1f;
    let funct3 = (ir >> 12) & 0x7;
    let funct7 = (ir >> 25) & 0x7f;

    match ir & 0x7f {
        // ── LUI ──────────────────────────────────────────────────────────────
        0x37 => {
            format!("lui     {}, 0x{:x}", reg(rd), (imm_u(ir) as u32) >> 12)
        }

        // ── AUIPC ────────────────────────────────────────────────────────────
        0x17 => {
            format!("auipc   {}, 0x{:x}", reg(rd), (imm_u(ir) as u32) >> 12)
        }

        // ── JAL ──────────────────────────────────────────────────────────────
        0x6f => {
            let target = pc.wrapping_add(imm_j(ir) as u32);
            let sym = fmt_target(target, syms);
            if rd == 0 {
                format!("j       {}", sym) // pseudo: j offset
            } else if rd == 1 {
                format!("jal     ra, {}", sym) // pseudo: call / jal ra
            } else {
                format!("jal     {}, {}", reg(rd), sym)
            }
        }

        // ── JALR ─────────────────────────────────────────────────────────────
        0x67 => {
            let imm = imm_i(ir);
            // ret: jalr zero, 0(ra)
            if rd == 0 && rs1 == 1 && imm == 0 {
                return "ret".to_string();
            }
            // jr rs: jalr zero, 0(rs)
            if rd == 0 && imm == 0 {
                return format!("jr      {}", reg(rs1));
            }
            format!("jalr    {}, {}({})", reg(rd), imm, reg(rs1))
        }

        // ── Branches ─────────────────────────────────────────────────────────
        0x63 => {
            let target = pc.wrapping_add(imm_b(ir) as u32);
            let sym = fmt_target(target, syms);
            let mnem = match funct3 {
                0 => "beq",
                1 => "bne",
                4 => "blt",
                5 => "bge",
                6 => "bltu",
                7 => "bgeu",
                _ => return format!("unknown.branch funct3={}", funct3),
            };
            // Pseudos: beqz / bnez / bltz / bgez / blez / bgtz
            if rs2 == 0 {
                let pseudo = match funct3 {
                    0 => Some("beqz"),
                    1 => Some("bnez"),
                    4 => Some("bltz"),
                    5 => Some("bgez"),
                    _ => None,
                };
                if let Some(p) = pseudo {
                    return format!("{:<8}{}, {}", p, reg(rs1), sym);
                }
            }
            if rs1 == 0 {
                let pseudo = match funct3 {
                    4 => Some("bgtz"), // blt zero, rs2 → bgtz rs2
                    5 => Some("blez"), // bge zero, rs2 → blez rs2
                    _ => None,
                };
                if let Some(p) = pseudo {
                    return format!("{:<8}{}, {}", p, reg(rs2), sym);
                }
            }
            format!("{:<8}{}, {}, {}", mnem, reg(rs1), reg(rs2), sym)
        }

        // ── Loads ────────────────────────────────────────────────────────────
        0x03 => {
            let imm = imm_i(ir);
            let mnem = match funct3 {
                0 => "lb",
                1 => "lh",
                2 => "lw",
                4 => "lbu",
                5 => "lhu",
                _ => return format!("unknown.load funct3={}", funct3),
            };
            format!("{:<8}{}, {}", mnem, reg(rd), mem_operand(rs1, imm))
        }

        // ── Stores ───────────────────────────────────────────────────────────
        0x23 => {
            let imm = imm_s(ir);
            let mnem = match funct3 {
                0 => "sb",
                1 => "sh",
                2 => "sw",
                _ => return format!("unknown.store funct3={}", funct3),
            };
            format!("{:<8}{}, {}", mnem, reg(rs2), mem_operand(rs1, imm))
        }

        // ── OP-IMM (0x13) ────────────────────────────────────────────────────
        0x13 => {
            let imm = imm_i(ir);
            let shamt = (ir >> 20) & 0x1f;
            match funct3 {
                0 => {
                    // nop: addi zero, zero, 0
                    if rd == 0 && rs1 == 0 && imm == 0 {
                        return "nop".to_string();
                    }
                    // mv rd, rs: addi rd, rs, 0
                    if imm == 0 {
                        return format!("mv      {}, {}", reg(rd), reg(rs1));
                    }
                    // li rd, imm: addi rd, zero, imm
                    if rs1 == 0 {
                        return format!("li      {}, {}", reg(rd), imm);
                    }
                    format!("addi    {}, {}, {}", reg(rd), reg(rs1), imm)
                }
                1 => format!("slli    {}, {}, {}", reg(rd), reg(rs1), shamt),
                2 => format!("slti    {}, {}, {}", reg(rd), reg(rs1), imm),
                3 => format!("sltiu   {}, {}, {}", reg(rd), reg(rs1), imm as u32),
                4 => {
                    // not rd, rs: xori rd, rs, -1
                    if imm == -1 {
                        return format!("not     {}, {}", reg(rd), reg(rs1));
                    }
                    format!("xori    {}, {}, {}", reg(rd), reg(rs1), imm)
                }
                5 => {
                    if funct7 == 0x20 {
                        format!("srai    {}, {}, {}", reg(rd), reg(rs1), shamt)
                    } else {
                        format!("srli    {}, {}, {}", reg(rd), reg(rs1), shamt)
                    }
                }
                6 => format!("ori     {}, {}, {}", reg(rd), reg(rs1), imm),
                7 => format!("andi    {}, {}, {}", reg(rd), reg(rs1), imm),
                _ => format!("unknown.op-imm funct3={}", funct3),
            }
        }

        // ── OP (0x33) — RV32I + RV32M ────────────────────────────────────────
        0x33 => {
            if funct7 == 0x01 {
                // RV32M
                let mnem = match funct3 {
                    0 => "mul",
                    1 => "mulh",
                    2 => "mulhsu",
                    3 => "mulhu",
                    4 => "div",
                    5 => "divu",
                    6 => "rem",
                    7 => "remu",
                    _ => return format!("unknown.m funct3={}", funct3),
                };
                return format!("{:<8}{}, {}, {}", mnem, reg(rd), reg(rs1), reg(rs2));
            }
            match funct3 {
                0 => {
                    if funct7 == 0x20 {
                        // neg rd, rs: sub rd, zero, rs
                        if rs1 == 0 {
                            return format!("neg     {}, {}", reg(rd), reg(rs2));
                        }
                        format!("sub     {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                    } else {
                        format!("add     {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                    }
                }
                1 => format!("sll     {}, {}, {}", reg(rd), reg(rs1), reg(rs2)),
                2 => {
                    // seqz is not a standard pseudo here; emit canonical form.
                    format!("slt     {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                }
                3 => {
                    // snez rd, rs: sltu rd, zero, rs
                    if rs1 == 0 {
                        return format!("snez    {}, {}", reg(rd), reg(rs2));
                    }
                    format!("sltu    {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                }
                4 => format!("xor     {}, {}, {}", reg(rd), reg(rs1), reg(rs2)),
                5 => {
                    if funct7 == 0x20 {
                        format!("sra     {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                    } else {
                        format!("srl     {}, {}, {}", reg(rd), reg(rs1), reg(rs2))
                    }
                }
                6 => format!("or      {}, {}, {}", reg(rd), reg(rs1), reg(rs2)),
                7 => format!("and     {}, {}, {}", reg(rd), reg(rs1), reg(rs2)),
                _ => format!("unknown.op funct3={}", funct3),
            }
        }

        // ── FENCE ────────────────────────────────────────────────────────────
        0x0f => "fence".to_string(),

        // ── SYSTEM ───────────────────────────────────────────────────────────
        0x73 => {
            let csrno = ir >> 20;
            let microop = funct3;
            if microop == 0 {
                return match csrno {
                    0x000 => "ecall".to_string(),
                    0x001 => "ebreak".to_string(),
                    0x302 => "mret".to_string(),
                    0x105 => "wfi".to_string(),
                    _ => format!("system  0x{:03x}", csrno),
                };
            }
            // Zicsr
            let csr_name = csr_name(csrno);
            let zimm = rs1; // rs1 field used as imm on I variants
            match microop {
                1 => {
                    // csrw csr, rs: csrrw zero, csr, rs (rd == zero, discards reading)
                    if rd == 0 {
                        return format!("csrw    {}, {}", csr_name, reg(rs1));
                    }
                    format!("csrrw   {}, {}, {}", reg(rd), csr_name, reg(rs1))
                }
                2 => {
                    // csrs csr, rs: csrrs zero, csr, rs
                    if rd == 0 {
                        return format!("csrs    {}, {}", csr_name, reg(rs1));
                    }
                    // csrr rd, csr: csrrs rd, csr, zero (rs1 == zero)
                    if rs1 == 0 {
                        return format!("csrr    {}, {}", reg(rd), csr_name);
                    }
                    format!("csrrs   {}, {}, {}", reg(rd), csr_name, reg(rs1))
                }
                3 => {
                    if rd == 0 {
                        return format!("csrc    {}, {}", csr_name, reg(rs1));
                    }
                    format!("csrrc   {}, {}, {}", reg(rd), csr_name, reg(rs1))
                }
                5 => {
                    if rd == 0 {
                        return format!("csrwi   {}, {}", csr_name, zimm);
                    }
                    format!("csrrwi  {}, {}, {}", reg(rd), csr_name, zimm)
                }
                6 => {
                    if rd == 0 {
                        return format!("csrsi   {}, {}", csr_name, zimm);
                    }
                    format!("csrrsi  {}, {}, {}", reg(rd), csr_name, zimm)
                }
                7 => {
                    if rd == 0 {
                        return format!("csrci   {}, {}", csr_name, zimm);
                    }
                    format!("csrrci  {}, {}, {}", reg(rd), csr_name, zimm)
                }
                _ => format!("unknown.csr microop={}", microop),
            }
        }

        // ── RV32A ────────────────────────────────────────────────────────────
        0x2f => {
            let funct5 = (ir >> 27) & 0x1f;
            let aq = (ir >> 26) & 1;
            let rl = (ir >> 25) & 1;
            let order = match (aq, rl) {
                (1, 1) => ".aqrl",
                (1, 0) => ".aq",
                (0, 1) => ".rl",
                _ => "",
            };
            match funct5 {
                0b00010 => format!("lr.w{}   {}, ({})", order, reg(rd), reg(rs1)),
                0b00011 => format!("sc.w{}   {}, {}, ({})", order, reg(rd), reg(rs2), reg(rs1)),
                0b00001 => format!(
                    "amoswap.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b00000 => format!(
                    "amoadd.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b00100 => format!(
                    "amoxor.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b01100 => format!(
                    "amoand.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b01000 => format!(
                    "amoor.w{}  {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b10000 => format!(
                    "amomin.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b10100 => format!(
                    "amomax.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b11000 => format!(
                    "amominu.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                0b11100 => format!(
                    "amomaxu.w{} {}, {}, ({})",
                    order,
                    reg(rd),
                    reg(rs2),
                    reg(rs1)
                ),
                _ => format!("unknown.amo funct5=0x{:02x}", funct5),
            }
        }

        _ => format!("unknown  0x{:08x}", ir),
    }
}

// ── CSR names ────────────────────────────────────────────────────────────────

/// Return the conventional name of a CSR address, or `"0xNNN"` if unknown.
fn csr_name(csr: u32) -> String {
    match csr {
        0x300 => "mstatus".into(),
        0x301 => "misa".into(),
        0x304 => "mie".into(),
        0x305 => "mtvec".into(),
        0x340 => "mscratch".into(),
        0x341 => "mepc".into(),
        0x342 => "mcause".into(),
        0x343 => "mtval".into(),
        0x344 => "mip".into(),
        0xc00 => "cycle".into(),
        0xc01 => "time".into(),
        0xc02 => "instret".into(),
        0xf11 => "mvendorid".into(),
        0xf12 => "marchid".into(),
        0xf13 => "mimpid".into(),
        0xf14 => "mhartid".into(),
        _ => format!("0x{:03x}", csr),
    }
}

// ── Static disassembly ───────────────────────────────────────────────────────

/// Disassemble an entire ELF image, segment by segment, like `objdump -d`.
///
/// Prints to stdout in the following format:
///
/// ```text
/// Disassembly of section .text:
///
/// 80000000 <_start>:
///   80000000  02000117  auipc   sp, 0x2000
/// ```
///
/// Segments shorter than 4 bytes are skipped.
pub fn disasm_elf(_data: &[u8], image: &ElfImage, syms: Option<&SymbolTable>) {
    // Use PT_LOAD segments as a proxy for code sections.
    // A true per-section disassembly would require parsing section headers,
    // but segments are sufficient for the typical use case.
    for (seg_idx, seg) in image.segments.iter().enumerate() {
        let base = seg.vaddr;
        let data_slice = &seg.data;

        // Skip segments too short to contain even one instruction.
        if data_slice.len() < 4 {
            continue;
        }

        // Section label — use a generic name; we parse segments, not sections.
        let section_label = if seg_idx == 0 {
            ".text"
        } else {
            &format!(".text.{}", seg_idx)
        };
        println!("Disassembly of section {}:\n", section_label);

        let mut i = 0usize;
        while i + 3 < data_slice.len() {
            let pc = base + i as u32;

            // Print a symbol label if this address is the start of a known symbol.
            if let Some(sym) = syms.and_then(|t| t.lookup_addr(pc)) {
                if sym.addr == pc {
                    println!("{:08x} <{}>:", pc, sym.name);
                }
            }

            let ir = u32::from_le_bytes([
                data_slice[i],
                data_slice[i + 1],
                data_slice[i + 2],
                data_slice[i + 3],
            ]);

            let mnem = disassemble(ir, pc, syms);
            println!("  {:08x}  {:08x}  {}", pc, ir, mnem);

            i += 4;
        }
        println!();
    }
}
