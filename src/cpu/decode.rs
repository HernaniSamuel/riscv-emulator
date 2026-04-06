//! RISC-V RV32I instruction decoder.
//!
//! This module converts raw 32-bit instructions into strongly typed
//! [`Instruction`] enums used by the execution stage.

use crate::cpu::cpu::{CPU, CPUError};
use crate::cpu::instruction::Instruction;

impl CPU {
    /// Decodes a raw 32-bit RISC-V instruction into a structured [Instruction] enum.
    ///
    /// This function implements the RV32I base integer instruction decoding logic.
    /// It extracts opcode, funct3, funct7, register indices, and immediates,
    /// then maps them into the corresponding [Instruction] variant.
    ///
    /// The decoder is pure: it does not modify CPU state, memory, or registers.
    /// It only interprets the provided instruction word.
    ///
    /// # Arguments
    ///
    /// * instruction - A 32-bit raw instruction fetched from memory.
    ///
    /// # Returns
    ///
    /// * Ok(Instruction) if the instruction is recognized and successfully decoded.
    /// * Err(CPUError::IllegalInstruction) if the opcode or encoding is not supported.
    ///
    /// # Supported Instruction Classes
    ///
    /// * R-type (ADD, SUB, SLL, SRL, SRA, AND, OR, XOR, SLT, SLTU)
    /// * I-type (ADDI, SLLI, SRLI, SRAI)
    /// * Loads (LB, LH, LW, LBU, LHU)
    /// * Stores (SW)
    /// * Branches (BEQ, BNE)
    /// * Upper immediates (LUI, AUIPC)
    /// * Jumps (JAL, JALR)
    /// * System (ECALL, EBREAK)
    /// * Fence (FENCE)
    ///
    /// # Errors
    ///
    /// Returns [CPUError::IllegalInstruction] when:
    ///
    /// * The opcode is not part of RV32I
    /// * funct3 or funct7 combination is invalid
    /// * The instruction belongs to an unsupported extension (e.g., RV32M)
    ///
    /// # Example
    ///
    /// /// # use riscv::*; /// # let mut cpu = /* create CPU */ unimplemented!(); /// let raw = 0x00500093; // addi x1, x0, 5 /// let instr = cpu.decode(raw).unwrap(); /// /// assert_eq!( /// instr, /// Instruction::Addi { /// rd: 1, /// rs1: 0, /// imm: 5 /// } /// ); ///
    ///
    /// # Notes
    ///
    /// * Immediates are sign-extended according to the RISC-V specification.
    /// * Register indices are not validated here; execution stage handles that.
    /// * This decoder assumes 32-bit aligned instructions (no compressed ISA).
    ///
    pub fn decode(&mut self, instruction: u32) -> Result<Instruction, CPUError> {
        let opcode = instruction & 0x7f;
        let rd = ((instruction >> 7) & 0x1f) as u8;
        let funct3 = (instruction >> 12) & 0x7;
        let rs1 = ((instruction >> 15) & 0x1f) as u8;
        let rs2 = ((instruction >> 20) & 0x1f) as u8;
        let funct7 = (instruction >> 25) & 0x7f;

        // ================= immediates =================

        // sign extend helper
        let sign_extend =
            |value: u32, bits: u32| -> i32 { ((value << (32 - bits)) as i32) >> (32 - bits) };

        let imm_i = sign_extend(instruction >> 20, 12);

        let imm_s = sign_extend(((instruction >> 25) << 5) | ((instruction >> 7) & 0x1f), 12);

        let imm_b = sign_extend(
            ((instruction >> 31) << 12)
                | (((instruction >> 7) & 0x1) << 11)
                | (((instruction >> 25) & 0x3f) << 5)
                | (((instruction >> 8) & 0xf) << 1),
            13,
        );

        let imm_u = (instruction & 0xfffff000) as i32;

        let imm_j = sign_extend(
            ((instruction >> 31) << 20)
                | (((instruction >> 12) & 0xff) << 12)
                | (((instruction >> 20) & 0x1) << 11)
                | (((instruction >> 21) & 0x3ff) << 1),
            21,
        );

        let shamt = ((instruction >> 20) & 0x1f) as u8;

        // ================= decode =================

        let instr = match opcode {
            // ================= R =================
            0b0110011 => match (funct3, funct7) {
                (0x0, 0x00) => Instruction::Add { rd, rs1, rs2 },
                (0x0, 0x20) => Instruction::Sub { rd, rs1, rs2 },
                (0x1, 0x00) => Instruction::Sll { rd, rs1, rs2 },
                (0x2, 0x00) => Instruction::Slt { rd, rs1, rs2 },
                (0x3, 0x00) => Instruction::Sltu { rd, rs1, rs2 },
                (0x4, 0x00) => Instruction::Xor { rd, rs1, rs2 },
                (0x5, 0x00) => Instruction::Srl { rd, rs1, rs2 },
                (0x5, 0x20) => Instruction::Sra { rd, rs1, rs2 },
                (0x6, 0x00) => Instruction::Or { rd, rs1, rs2 },
                (0x7, 0x00) => Instruction::And { rd, rs1, rs2 },
                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= I =================
            0b0010011 => match funct3 {
                0x0 => Instruction::Addi {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x2 => Instruction::Slti {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x3 => Instruction::Sltiu {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x4 => Instruction::Xori {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x6 => Instruction::Ori {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x7 => Instruction::Andi {
                    rd,
                    rs1,
                    imm: imm_i,
                },

                0x1 => {
                    if funct7 == 0x00 {
                        Instruction::Slli { rd, rs1, shamt }
                    } else {
                        return Err(CPUError::IllegalInstruction(instruction));
                    }
                }

                0x5 => match funct7 {
                    0x00 => Instruction::Srli { rd, rs1, shamt },
                    0x20 => Instruction::Srai { rd, rs1, shamt },
                    _ => return Err(CPUError::IllegalInstruction(instruction)),
                },

                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= LOAD =================
            0b0000011 => match funct3 {
                0x0 => Instruction::Lb {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x1 => Instruction::Lh {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x2 => Instruction::Lw {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x4 => Instruction::Lbu {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x5 => Instruction::Lhu {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= STORE =================
            0b0100011 => match funct3 {
                0x0 => Instruction::Sb {
                    rs1,
                    rs2,
                    imm: imm_s,
                },
                0x1 => Instruction::Sh {
                    rs1,
                    rs2,
                    imm: imm_s,
                },
                0x2 => Instruction::Sw {
                    rs1,
                    rs2,
                    imm: imm_s,
                },
                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= BRANCH =================
            0b1100011 => match funct3 {
                0x0 => Instruction::Beq {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x1 => Instruction::Bne {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x4 => Instruction::Blt {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x5 => Instruction::Bge {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x6 => Instruction::Bltu {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x7 => Instruction::Bgeu {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= U =================
            0b0110111 => Instruction::Lui { rd, imm: imm_u },
            0b0010111 => Instruction::Auipc { rd, imm: imm_u },

            // ================= J =================
            0b1101111 => Instruction::Jal { rd, imm: imm_j },

            0b1100111 => {
                if funct3 == 0x0 {
                    Instruction::Jalr {
                        rd,
                        rs1,
                        imm: imm_i,
                    }
                } else {
                    return Err(CPUError::IllegalInstruction(instruction));
                }
            }

            // ================= FENCE =================
            0b0001111 => match funct3 {
                0x0 => Instruction::Fence,
                0x1 => Instruction::FenceI,
                _ => return Err(CPUError::IllegalInstruction(instruction)),
            },

            // ================= SYSTEM =================
            0b1110011 => {
                let sys = (instruction >> 20) & 0xfff;
                match (funct3, sys) {
                    (0x0, 0x000) => Instruction::Ecall,
                    (0x0, 0x001) => Instruction::Ebreak,
                    _ => return Err(CPUError::IllegalInstruction(instruction)),
                }
            }

            _ => return Err(CPUError::IllegalInstruction(instruction)),
        };

        Ok(instr)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::risc_v::ElfImage;

    // ── helpers ───────────────────────────────────────────────────────────────

    fn dummy_elf() -> ElfImage {
        ElfImage {
            entry: 0,
            segments: vec![],
        }
    }

    fn dummy_cpu() -> CPU {
        CPU::new(dummy_elf(), 4).unwrap() // 4 KB RAM
    }

    // ── encoding helpers ───────────────────────────────────────────────────────

    // Assemble an R-type instruction: funct7 | rs2 | rs1 | funct3 | rd | opcode
    fn r_type(opcode: u32, funct3: u32, funct7: u32, rd: u8, rs1: u8, rs2: u8) -> u32 {
        ((funct7 as u32) << 25)
            | ((rs2 as u32) << 20)
            | ((rs1 as u32) << 15)
            | (funct3 << 12)
            | ((rd as u32) << 7)
            | opcode
    }

    // Assembles an I-type instruction. `imm` must fit into 12 signed bits.
    fn i_type(opcode: u32, funct3: u32, rd: u8, rs1: u8, imm: i32) -> u32 {
        let imm12 = (imm as u32) & 0xFFF;
        (imm12 << 20) | ((rs1 as u32) << 15) | (funct3 << 12) | ((rd as u32) << 7) | opcode
    }

    fn s_type(funct3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
        let imm12 = (imm as u32) & 0xFFF;
        let imm_11_5 = (imm12 >> 5) << 25;
        let imm_4_0 = (imm12 & 0x1F) << 7;
        imm_11_5
            | ((rs2 as u32) << 20)
            | ((rs1 as u32) << 15)
            | (funct3 << 12)
            | imm_4_0
            | 0b0100011
    }

    fn b_type(funct3: u32, rs1: u8, rs2: u8, imm: i32) -> u32 {
        // imm is a multiple of 2; bit 0 is implied
        let imm13 = (imm as u32) & 0x1FFF;
        let bit12 = (imm13 >> 12) & 0x1;
        let bit11 = (imm13 >> 11) & 0x1;
        let bits10_5 = (imm13 >> 5) & 0x3F;
        let bits4_1 = (imm13 >> 1) & 0xF;
        (bit12 << 31)
            | (bits10_5 << 25)
            | ((rs2 as u32) << 20)
            | ((rs1 as u32) << 15)
            | (funct3 << 12)
            | (bits4_1 << 8)
            | (bit11 << 7)
            | 0b1100011
    }

    fn u_type(opcode: u32, rd: u8, imm_upper: i32) -> u32 {
        // imm_upper should already be the value with the lower 12 bits set to zero
        (imm_upper as u32) | ((rd as u32) << 7) | opcode
    }

    fn j_type(rd: u8, imm: i32) -> u32 {
        let imm21 = (imm as u32) & 0x1FFFFF;
        let bit20 = (imm21 >> 20) & 0x1;
        let bits19_12 = (imm21 >> 12) & 0xFF;
        let bit11 = (imm21 >> 11) & 0x1;
        let bits10_1 = (imm21 >> 1) & 0x3FF;
        (bit20 << 31)
            | (bits10_1 << 21)
            | (bit11 << 20)
            | (bits19_12 << 12)
            | ((rd as u32) << 7)
            | 0b1101111
    }

    // ── R-type ────────────────────────────────────────────────────────────────────

    #[test]
    fn decode_add() {
        let mut cpu = dummy_cpu();
        let raw = r_type(0b0110011, 0x0, 0x00, 1, 2, 3);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Add {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }

    #[test]
    fn decode_sub() {
        let mut cpu = dummy_cpu();
        let raw = r_type(0b0110011, 0x0, 0x20, 5, 6, 7);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Sub {
                rd: 5,
                rs1: 6,
                rs2: 7
            }
        );
    }

    #[test]
    fn decode_sll() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x1, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Sll {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_srl() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x5, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Srl {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_sra() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x5, 0x20, 1, 2, 3)).unwrap(),
            Instruction::Sra {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_and() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x7, 0x00, 1, 2, 3)).unwrap(),
            Instruction::And {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_or() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x6, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Or {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_xor() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x4, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Xor {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_slt() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x2, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Slt {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }
    #[test]
    fn decode_sltu() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(r_type(0b0110011, 0x3, 0x00, 1, 2, 3)).unwrap(),
            Instruction::Sltu {
                rd: 1,
                rs1: 2,
                rs2: 3
            }
        );
    }

    // ── I-type ALU ────────────────────────────────────────────────────────────────

    #[test]
    fn decode_addi_positive() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0010011, 0x0, 1, 2, 42);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Addi {
                rd: 1,
                rs1: 2,
                imm: 42
            }
        );
    }

    #[test]
    fn decode_addi_negative() {
        // sign-extension: -1 em 12 bits → 0xFFF → sign-extendido para -1i32
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0010011, 0x0, 3, 4, -1);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Addi {
                rd: 3,
                rs1: 4,
                imm: -1
            }
        );
    }

    #[test]
    fn decode_addi_min_imm() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0010011, 0x0, 1, 0, -2048);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Addi {
                rd: 1,
                rs1: 0,
                imm: -2048
            }
        );
    }

    #[test]
    fn decode_addi_max_imm() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0010011, 0x0, 1, 0, 2047);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Addi {
                rd: 1,
                rs1: 0,
                imm: 2047
            }
        );
    }

    #[test]
    fn decode_slli() {
        let mut cpu = dummy_cpu();
        // shamt=5, funct7=0x00
        let raw = i_type(0b0010011, 0x1, 2, 3, 5);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Slli {
                rd: 2,
                rs1: 3,
                shamt: 5
            }
        );
    }

    #[test]
    fn decode_srli() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0010011, 0x5, 2, 3, 7); // funct7=0x00
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Srli {
                rd: 2,
                rs1: 3,
                shamt: 7
            }
        );
    }

    #[test]
    fn decode_srai() {
        let mut cpu = dummy_cpu();
        // funct7=0x20 para SRAI: bit 30 = 1, shamt nos bits [24:20]
        let shamt: u32 = 3;
        let raw =
            (0x20u32 << 25) | (3u32 << 15) | (0x5 << 12) | (2u32 << 7) | 0b0010011 | (shamt << 20);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Srai {
                rd: 2,
                rs1: 3,
                shamt: 3
            }
        );
    }

    // ── Loads ─────────────────────────────────────────────────────────────────────

    #[test]
    fn decode_lw() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0000011, 0x2, 1, 2, 8);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Lw {
                rd: 1,
                rs1: 2,
                imm: 8
            }
        );
    }

    #[test]
    fn decode_lw_negative_offset() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b0000011, 0x2, 1, 2, -4);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Lw {
                rd: 1,
                rs1: 2,
                imm: -4
            }
        );
    }

    #[test]
    fn decode_lb() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(i_type(0b0000011, 0x0, 1, 2, 0)).unwrap(),
            Instruction::Lb {
                rd: 1,
                rs1: 2,
                imm: 0
            }
        );
    }
    #[test]
    fn decode_lh() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(i_type(0b0000011, 0x1, 1, 2, 0)).unwrap(),
            Instruction::Lh {
                rd: 1,
                rs1: 2,
                imm: 0
            }
        );
    }
    #[test]
    fn decode_lbu() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(i_type(0b0000011, 0x4, 1, 2, 0)).unwrap(),
            Instruction::Lbu {
                rd: 1,
                rs1: 2,
                imm: 0
            }
        );
    }
    #[test]
    fn decode_lhu() {
        let mut c = dummy_cpu();
        assert_eq!(
            c.decode(i_type(0b0000011, 0x5, 1, 2, 0)).unwrap(),
            Instruction::Lhu {
                rd: 1,
                rs1: 2,
                imm: 0
            }
        );
    }

    // ── Stores ────────────────────────────────────────────────────────────────────

    #[test]
    fn decode_sw() {
        let mut cpu = dummy_cpu();
        let raw = s_type(0x2, 2, 3, 12);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Sw {
                rs1: 2,
                rs2: 3,
                imm: 12
            }
        );
    }

    #[test]
    fn decode_sw_negative_offset() {
        let mut cpu = dummy_cpu();
        let raw = s_type(0x2, 2, 3, -8);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Sw {
                rs1: 2,
                rs2: 3,
                imm: -8
            }
        );
    }

    // ── Branches ──────────────────────────────────────────────────────────────────

    #[test]
    fn decode_beq() {
        let mut cpu = dummy_cpu();
        let raw = b_type(0x0, 1, 2, 8); // offset +8
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Beq {
                rs1: 1,
                rs2: 2,
                imm: 8
            }
        );
    }

    #[test]
    fn decode_bne_negative() {
        let mut cpu = dummy_cpu();
        let raw = b_type(0x1, 1, 2, -4);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Bne {
                rs1: 1,
                rs2: 2,
                imm: -4
            }
        );
    }

    // ── U-type ────────────────────────────────────────────────────────────────────

    #[test]
    fn decode_lui() {
        let mut cpu = dummy_cpu();
        let raw = u_type(0b0110111, 5, 0x12345000u32 as i32);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Lui {
                rd: 5,
                imm: 0x12345000u32 as i32
            }
        );
    }

    #[test]
    fn decode_auipc() {
        let mut cpu = dummy_cpu();
        let raw = u_type(0b0010111, 3, 0x1000_0000u32 as i32);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Auipc {
                rd: 3,
                imm: 0x1000_0000u32 as i32
            }
        );
    }

    // ── Jumps ─────────────────────────────────────────────────────────────────────

    #[test]
    fn decode_jal() {
        let mut cpu = dummy_cpu();
        let raw = j_type(1, 256); // offset +256
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Jal { rd: 1, imm: 256 }
        );
    }

    #[test]
    fn decode_jalr() {
        let mut cpu = dummy_cpu();
        let raw = i_type(0b1100111, 0x0, 1, 2, 4);
        assert_eq!(
            cpu.decode(raw).unwrap(),
            Instruction::Jalr {
                rd: 1,
                rs1: 2,
                imm: 4
            }
        );
    }

    // ── System / Fence ────────────────────────────────────────────────────────────

    #[test]
    fn decode_ecall() {
        let mut c = dummy_cpu();
        assert_eq!(c.decode(0b1110011).unwrap(), Instruction::Ecall);
    }
    #[test]
    fn decode_ebreak() {
        let mut c = dummy_cpu();
        assert_eq!(c.decode(0x00100073).unwrap(), Instruction::Ebreak);
    }
    #[test]
    fn decode_fence() {
        let mut c = dummy_cpu();
        assert_eq!(c.decode(0b0001111).unwrap(), Instruction::Fence);
    }

    // ── Erros de decodificação ────────────────────────────────────────────────────

    #[test]
    fn decode_unknown_opcode_returns_illegal() {
        let mut cpu = dummy_cpu();
        // opcode 0b0000000 não existe em RV32I
        assert!(matches!(
            cpu.decode(0x0000_0000),
            Err(CPUError::IllegalInstruction(0x0000_0000))
        ));
    }

    #[test]
    fn decode_r_type_bad_funct7_returns_illegal() {
        let mut cpu = dummy_cpu();
        // ADD com funct7=0x01 (seria M-extension, não implementado)
        let raw = r_type(0b0110011, 0x0, 0x01, 1, 2, 3);
        assert!(matches!(
            cpu.decode(raw),
            Err(CPUError::IllegalInstruction(_))
        ));
    }
}
