//! RISC-V RV32I instruction execution engine.
//!
//! This module implements the **execution stage** of a simplified RISC-V CPU pipeline.
//! It is responsible for applying the semantic effects of a decoded
//! [`Instruction`] to the CPU state.
//!
//! # Role in the architecture
//!
//! The CPU is conceptually split into three stages:
//!
//! 1. **Fetch** – reads a 32-bit instruction from memory
//! 2. **Decode** – converts raw bits into [`Instruction`]
//! 3. **Execute** – applies the instruction effects (this module)
//!
//! This module corresponds to stage (3).
//!
//! # Responsibilities
//!
//! The execution engine is responsible for:
//!
//! - Integer ALU operations (R-type and I-type instructions)
//! - Memory access (loads and stores)
//! - Control flow (branches and jumps)
//! - System instructions (`ecall`, `ebreak`)
//! - Maintaining architectural state consistency
//!
//! # Design principles
//!
//! - **Single instruction effect model**: each call executes exactly one instruction.
//! - **Wrapping arithmetic**: all integer operations follow RV32I wrapping semantics.
//! - **Strict register semantics**: `x0` is always hardwired to zero.
//! - **Separation of concerns**: execution does not perform decoding or fetching.
//!
//! # Program Counter (PC) semantics
//!
//! - The PC is only modified by control-flow instructions:
//!   - Branches (when taken)
//!   - JAL / JALR
//! - Non-control-flow instructions do not mutate the PC.
//! - JALR always clears bit 0 of the target address (alignment rule).
//!
//! # Memory model
//!
//! - Little-endian addressing is assumed.
//! - Loads perform sign-extension or zero-extension depending on opcode.
//! - Stores truncate values to target width (byte / halfword / word).
//!
//! # System behavior
//!
//! - `ecall` is partially implemented and currently supports exit syscall.
//! - Unsupported syscalls return [`CPUError::UnsupportedSyscall`].
//! - `ebreak`, `fence`, and `fence.i` are treated as no-ops.
//!
//! # Compliance
//!
//! This implementation targets the **RV32I base integer ISA only**.
//! Extensions such as RV32M, RV32A, and compressed ISA (C) are not implemented.
//!
//! # Error handling
//!
//! Execution may fail with [`CPUError`] in cases such as:
//!
//! - Invalid memory access
//! - Unsupported system call
//! - Internal VM violations
//!
//! # Performance characteristics
//!
//! - Branches are resolved in a single step (no pipeline delay model)
//! - No speculative execution
//! - No out-of-order behavior
//!
//! This is a **functional emulator**, not a timing-accurate simulator.
//!

use crate::cpu::instruction::Instruction;
use crate::cpu::instruction::Instruction::*;
use crate::cpu::{CPU, CPUError};

impl CPU {
    /// Executes a single RV32I instruction on the CPU.
    ///
    /// This function implements the execution stage of the RISC-V pipeline.
    /// It takes a decoded [`Instruction`] and applies its semantics to the CPU state,
    /// including register file updates, memory access, and control flow changes.
    ///
    /// The function is **stateful**: it may modify:
    /// - General-purpose registers (`x0`–`x31`)
    /// - Memory
    /// - Program counter (PC)
    /// - CPU execution state (running / exit code)
    ///
    /// # Semantics
    ///
    /// - Arithmetic and logical instructions operate with wrapping semantics.
    /// - Branches modify the PC only when the condition is satisfied.
    /// - Jumps always update the PC and may write the return address.
    /// - Loads and stores perform memory access with sign/zero extension as specified by RV32I.
    /// - `x0` is always hardwired to zero (writes are ignored).
    ///
    /// # Errors
    ///
    /// Returns [`CPUError`] when:
    /// - A memory access fails (invalid address, alignment, etc.)
    /// - An unsupported or invalid syscall is executed (`ecall`)
    ///
    /// # Instruction Effects Summary
    ///
    /// ## ALU (R-type / I-type)
    /// - Perform arithmetic, logical, and shift operations
    /// - Use wrapping arithmetic (`wrapping_add`, `wrapping_sub`)
    ///
    /// ## Memory (Loads / Stores)
    /// - `lb`, `lh` sign-extend results
    /// - `lbu`, `lhu` zero-extend results
    /// - Stores write low bytes/halfwords/words to memory
    ///
    /// ## Control Flow
    /// - Branches modify PC only if condition is true
    /// - `jal` writes return address (PC + 4)
    /// - `jalr` clears bit 0 of target address
    ///
    /// ## System Instructions
    /// - `ecall` may halt execution or trigger syscalls
    /// - `ebreak` is currently a no-op
    /// - `fence` / `fence.i` are no-ops in this implementation
    ///
    /// # Notes
    ///
    /// - This function does **not fetch or decode instructions**.
    /// - It assumes the instruction is already validated.
    /// - PC management depends on the caller for non-control-flow instructions.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let mut cpu = CPU::new(elf, 4096).unwrap();
    ///
    /// cpu.execute(Instruction::Addi {
    ///     rd: 1,
    ///     rs1: 0,
    ///     imm: 10,
    /// })?;
    ///
    /// assert_eq!(cpu.get_x(1)?, 10);
    /// ```
    ///
    /// # RISC-V Compliance
    ///
    /// Implements RV32I base integer ISA (no M/A/F/D extensions).
    /// Compliant behavior includes:
    ///
    /// - Proper sign extension for loads
    /// - Proper shift masking (`shamt & 0x1f`)
    /// - Proper JALR alignment (`& !1`)
    ///
    pub fn execute(&mut self, instr: Instruction) -> Result<(), CPUError> {
        match instr {
            // ===================== R-type =====================
            Add { rd, rs1, rs2 } => {
                let val = self
                    .get_x(rs1 as usize)?
                    .wrapping_add(self.get_x(rs2 as usize)?);
                self.set_x(rd as usize, val)?;
            }
            Sub { rd, rs1, rs2 } => {
                let val = self
                    .get_x(rs1 as usize)?
                    .wrapping_sub(self.get_x(rs2 as usize)?);
                self.set_x(rd as usize, val)?;
            }
            Sll { rd, rs1, rs2 } => {
                let shamt = self.get_x(rs2 as usize)? & 0x1f;
                let val = self.get_x(rs1 as usize)? << shamt;
                self.set_x(rd as usize, val)?;
            }
            Slt { rd, rs1, rs2 } => {
                let val = (self.get_x(rs1 as usize)? as i32) < (self.get_x(rs2 as usize)? as i32);
                self.set_x(rd as usize, val as u32)?;
            }
            Sltu { rd, rs1, rs2 } => {
                let val = self.get_x(rs1 as usize)? < self.get_x(rs2 as usize)?;
                self.set_x(rd as usize, val as u32)?;
            }
            Xor { rd, rs1, rs2 } => {
                let val = self.get_x(rs1 as usize)? ^ self.get_x(rs2 as usize)?;
                self.set_x(rd as usize, val)?;
            }
            Srl { rd, rs1, rs2 } => {
                let shamt = self.get_x(rs2 as usize)? & 0x1f;
                let val = self.get_x(rs1 as usize)? >> shamt;
                self.set_x(rd as usize, val)?;
            }
            Sra { rd, rs1, rs2 } => {
                let shamt = self.get_x(rs2 as usize)? & 0x1f;
                let val = (self.get_x(rs1 as usize)? as i32 >> shamt) as u32;
                self.set_x(rd as usize, val)?;
            }
            Or { rd, rs1, rs2 } => {
                let val = self.get_x(rs1 as usize)? | self.get_x(rs2 as usize)?;
                self.set_x(rd as usize, val)?;
            }
            And { rd, rs1, rs2 } => {
                let val = self.get_x(rs1 as usize)? & self.get_x(rs2 as usize)?;
                self.set_x(rd as usize, val)?;
            }

            // ===================== I-type =====================
            Addi { rd, rs1, imm } => {
                let val = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                self.set_x(rd as usize, val)?;
            }
            Slti { rd, rs1, imm } => {
                let val = (self.get_x(rs1 as usize)? as i32) < imm;
                self.set_x(rd as usize, val as u32)?;
            }
            Sltiu { rd, rs1, imm } => {
                let val = self.get_x(rs1 as usize)? < imm as u32;
                self.set_x(rd as usize, val as u32)?;
            }
            Xori { rd, rs1, imm } => {
                let val = self.get_x(rs1 as usize)? ^ imm as u32;
                self.set_x(rd as usize, val)?;
            }
            Ori { rd, rs1, imm } => {
                let val = self.get_x(rs1 as usize)? | imm as u32;
                self.set_x(rd as usize, val)?;
            }
            Andi { rd, rs1, imm } => {
                let val = self.get_x(rs1 as usize)? & imm as u32;
                self.set_x(rd as usize, val)?;
            }
            Slli { rd, rs1, shamt } => {
                let val = self.get_x(rs1 as usize)? << (shamt & 0x1f);
                self.set_x(rd as usize, val)?;
            }
            Srli { rd, rs1, shamt } => {
                let val = self.get_x(rs1 as usize)? >> (shamt & 0x1f);
                self.set_x(rd as usize, val)?;
            }
            Srai { rd, rs1, shamt } => {
                let val = ((self.get_x(rs1 as usize)? as i32) >> (shamt & 0x1f)) as u32;
                self.set_x(rd as usize, val)?;
            }

            // ===================== Loads =====================
            Lb { rd, rs1, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                let val = self.read_u8(addr)? as i8 as i32 as u32;
                self.set_x(rd as usize, val)?;
            }
            Lh { rd, rs1, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                let val = self.read_u16(addr)? as i16 as i32 as u32;
                self.set_x(rd as usize, val)?;
            }
            Lw { rd, rs1, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                let val = self.read_u32(addr)?;
                self.set_x(rd as usize, val)?;
            }
            Lbu { rd, rs1, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                let val = self.read_u8(addr)? as u32;
                self.set_x(rd as usize, val)?;
            }
            Lhu { rd, rs1, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                let val = self.read_u16(addr)? as u32;
                self.set_x(rd as usize, val)?;
            }

            // ===================== Stores =====================
            Sb { rs1, rs2, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                self.write_u8(addr, self.get_x(rs2 as usize)? as u8)?;
            }
            Sh { rs1, rs2, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                self.write_u16(addr, self.get_x(rs2 as usize)? as u16)?;
            }
            Sw { rs1, rs2, imm } => {
                let addr = self.get_x(rs1 as usize)?.wrapping_add(imm as u32);
                self.write_u32(addr, self.get_x(rs2 as usize)?)?;
            }

            // ===================== Branches =====================
            Beq { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? == self.get_x(rs2 as usize)? {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bne { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? != self.get_x(rs2 as usize)? {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Blt { rs1, rs2, imm } => {
                if (self.get_x(rs1 as usize)? as i32) < (self.get_x(rs2 as usize)? as i32) {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bge { rs1, rs2, imm } => {
                if (self.get_x(rs1 as usize)? as i32) >= (self.get_x(rs2 as usize)? as i32) {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bltu { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? < self.get_x(rs2 as usize)? {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bgeu { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? >= self.get_x(rs2 as usize)? {
                    let pc = self.get_pc();
                    let target = (pc as i32).wrapping_add(imm) as u32;
                    self.set_pc(target)?;
                    return Ok(());
                }
            }

            // ===================== U-type =====================
            Lui { rd, imm } => {
                self.set_x(rd as usize, imm as u32)?;
            }
            Auipc { rd, imm } => {
                let val = self.get_pc().wrapping_add(imm as u32);
                self.set_x(rd as usize, val)?;
            }

            // ===================== Jumps =====================
            Jal { rd, imm } => {
                let pc = self.get_pc();
                let target = pc.wrapping_add(imm as u32);

                self.set_x(rd as usize, pc.wrapping_add(4))?;
                self.set_pc(target)?;
            }
            Jalr { rd, rs1, imm } => {
                let base = self.get_x(rs1 as usize)?;
                let target = base.wrapping_add(imm as u32) & !1;

                let ret = self.get_pc().wrapping_add(4);
                self.set_x(rd as usize, ret)?;
                self.set_pc(target)?;
            }

            // ===================== System =====================
            Ecall => {
                let syscall = self.get_x(17)?; // a7
                match syscall {
                    93 => {
                        let code = self.get_x(10)? as i32; // a0
                        self.set_exit_code(code);
                        self.set_running(false);
                    }
                    _ => {
                        return Err(CPUError::UnsupportedSyscall(syscall));
                    }
                }
            }
            Ebreak => (), // TODO: implement ebreak handler

            // ===================== Memory ordering =====================
            Fence | FenceI => (), // no-op for RV32I
        }

        Ok(())
    }
}

// TESTS
#[cfg(test)]
mod guardrail_tests {
    use super::*;
    use crate::risc_v::ElfImage;

    fn dummy_cpu() -> CPU {
        CPU::new(
            ElfImage {
                entry: 0,
                segments: vec![],
            },
            4096,
        )
        .unwrap()
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 1: x0 is hardwired to zero
    // Any instruction that writes to rd=0 must be a silent no-op.
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_x0_is_always_zero_after_write() {
        let mut cpu = dummy_cpu();
        cpu.set_x(1, 999).unwrap();

        // All of the instructions below attempt to write to x0
        cpu.execute(Add {
            rd: 0,
            rs1: 1,
            rs2: 1,
        })
        .unwrap();
        cpu.execute(Addi {
            rd: 0,
            rs1: 1,
            imm: 42,
        })
        .unwrap();
        cpu.execute(Lui { rd: 0, imm: 0xDEAD }).unwrap();
        cpu.execute(Jal { rd: 0, imm: 0 }).unwrap(); // JAL rd=0 is “call discarding link”

        assert_eq!(cpu.get_x(0).unwrap(), 0, "x0 deve ser sempre 0");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 2: Non-branch/non-jump instructions MUST NOT alter the PC
    // The PC increment (+4) must occur outside of execute(), or the caller
    // guarantees this. execute() must not change the PC for arithmetic instructions.
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_arithmetic_does_not_touch_pc() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0x100).unwrap();
        cpu.set_x(1, 10).unwrap();
        cpu.set_x(2, 20).unwrap();

        let instrs = vec![
            Add {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Sub {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            And {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Or {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Xor {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Sll {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Srl {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Sra {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Slt {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
            Sltu {
                rd: 3,
                rs1: 1,
                rs2: 2,
            },
        ];

        for instr in instrs {
            cpu.execute(instr).unwrap();
            assert_eq!(
                cpu.get_pc(),
                0x100,
                "PC should not change in R-type instructions: {:?}",
                instr
            );
        }
    }

    #[test]
    fn guardrail_immediate_ops_do_not_touch_pc() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0x200).unwrap();
        cpu.set_x(1, 5).unwrap();

        let instrs = vec![
            Addi {
                rd: 2,
                rs1: 1,
                imm: 1,
            },
            Slti {
                rd: 2,
                rs1: 1,
                imm: 10,
            },
            Sltiu {
                rd: 2,
                rs1: 1,
                imm: 10,
            },
            Andi {
                rd: 2,
                rs1: 1,
                imm: 0xF,
            },
            Ori {
                rd: 2,
                rs1: 1,
                imm: 0xF,
            },
            Xori {
                rd: 2,
                rs1: 1,
                imm: 0xF,
            },
            Slli {
                rd: 2,
                rs1: 1,
                shamt: 2,
            },
            Srli {
                rd: 2,
                rs1: 1,
                shamt: 2,
            },
            Srai {
                rd: 2,
                rs1: 1,
                shamt: 2,
            },
            Lui { rd: 2, imm: 0x1000 },
        ];

        for instr in instrs {
            cpu.execute(instr).unwrap();
            assert_eq!(
                cpu.get_pc(),
                0x200,
                "The PC should not change in I-type/U-type: {:?}",
                instr
            );
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 3: A branch that is NOT taken does not change the program counter
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_untaken_branch_preserves_pc() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0x100).unwrap();
        cpu.set_x(1, 1).unwrap();
        cpu.set_x(2, 2).unwrap();

        // Beq with rs1 ≠ rs2 → not taken
        cpu.execute(Beq {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);

        // Bne with rs1 == rs2 → not taken
        cpu.set_x(2, 1).unwrap();
        cpu.execute(Bne {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);

        // Blt with rs1 >= rs2 (signed) → not taken
        cpu.set_x(1, 5).unwrap();
        cpu.set_x(2, 3).unwrap();
        cpu.execute(Blt {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);

        // Bge with rs1 < rs2 (signed) → not taken
        cpu.set_x(1, 1).unwrap();
        cpu.set_x(2, 5).unwrap();
        cpu.execute(Bge {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);

        // Bltu with rs1 >= rs2 (unsigned) → not taken
        cpu.set_x(1, 10).unwrap();
        cpu.set_x(2, 3).unwrap();
        cpu.execute(Bltu {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);

        // Bgeu with rs1 < rs2 (unsigned) → not taken
        cpu.set_x(1, 1).unwrap();
        cpu.set_x(2, 10).unwrap();
        cpu.execute(Bgeu {
            rs1: 1,
            rs2: 2,
            imm: 100,
        })
        .unwrap();
        assert_eq!(cpu.get_pc(), 0x100);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 4: Correct sign extension in loads
    // Lb/Lh must extend the sign; Lbu/Lhu must not.
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_load_sign_extension() {
        let mut cpu = dummy_cpu();
        // Write 0xFF to a byte → since i8 is -1 → sign-extended = 0xFFFFFFFF
        cpu.set_x(1, 0).unwrap();
        cpu.set_x(2, 0xFF).unwrap();
        cpu.execute(Sb {
            rs1: 1,
            rs2: 2,
            imm: 0,
        })
        .unwrap();

        cpu.execute(Lb {
            rd: 3,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(3).unwrap(),
            0xFFFFFFFF,
            "Lb should be sign-extend 0xFF for 0xFFFFFFFF"
        );

        cpu.execute(Lbu {
            rd: 4,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(cpu.get_x(4).unwrap(), 0xFF, "Lbu should not be sign-extend");

        // 0x8000 como i16 é -32768 → sign-extended = 0xFFFF8000
        cpu.set_x(2, 0x8000).unwrap();
        cpu.execute(Sh {
            rs1: 1,
            rs2: 2,
            imm: 0,
        })
        .unwrap();

        cpu.execute(Lh {
            rd: 5,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(5).unwrap(),
            0xFFFF8000,
            "Lh should be sign-extend 0x8000"
        );

        cpu.execute(Lhu {
            rd: 6,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(6).unwrap(),
            0x8000,
            "Lhu should not be sign-extend"
        );
    }

    #[test]
    fn guardrail_lh_sign_extend_negative_halfword() {
        let mut cpu = dummy_cpu();
        // Write 0x80FF to memory (little-endian: FF 80)
        cpu.set_x(1, 0).unwrap();
        cpu.set_x(2, 0x80FF).unwrap();
        cpu.execute(Sh {
            rs1: 1,
            rs2: 2,
            imm: 0,
        })
        .unwrap();

        cpu.execute(Lh {
            rd: 3,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(3).unwrap(),
            0xFFFF80FF,
            "The 0x80FF byte must be sign-extended to 0xFFFF80FF"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 5: SRA and SRAI preserve the sign bit
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_sra_preserves_sign_bit() {
        let mut cpu = dummy_cpu();

        // Negative number: MSB = 1
        let negative: u32 = 0x8000_0000;
        cpu.set_x(1, negative).unwrap();
        cpu.set_x(2, 1).unwrap();

        cpu.execute(Sra {
            rd: 3,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(3).unwrap() & 0x8000_0000,
            0x8000_0000,
            "SRA must set MSB=1 for negative numbers"
        );

        cpu.execute(Srai {
            rd: 4,
            rs1: 1,
            shamt: 1,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(4).unwrap() & 0x8000_0000,
            0x8000_0000,
            "SRAI should set MSB=1 for negative numbers"
        );

        // An SRL with the same value should NOT be preserved (ensures that it is truly arithmetic)
        cpu.execute(Srl {
            rd: 5,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(5).unwrap() & 0x8000_0000,
            0,
            "SRL should not preserve the signal bit (logic)"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 6: shamt is masked to 5 bits (& 0x1f)
    // SLL/SRL/SRA with rs2 >= 32 must use only the lower 5 bits.
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_shamt_masked_to_5_bits() {
        let mut cpu = dummy_cpu();
        cpu.set_x(1, 1).unwrap();

        // rs2 = 32 → real shamt = 0 → result should be 1 (without shift)
        cpu.set_x(2, 32).unwrap();
        cpu.execute(Sll {
            rd: 3,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(3).unwrap(),
            1,
            "SLL com shamt=32 deve ser equivalente a shamt=0"
        );

        // rs2 = 33 → actual shamt = 1 → the result should be 2
        cpu.set_x(2, 33).unwrap();
        cpu.execute(Sll {
            rd: 4,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(4).unwrap(),
            2,
            "SLL with shamt=33 should be equivalent to shamt=1"
        );

        // rs2 = 0xFF → actual shift amount = 31 → maximum shift
        cpu.set_x(2, 0xFF).unwrap();
        cpu.execute(Srl {
            rd: 5,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(cpu.get_x(5).unwrap(), cpu.get_x(1).unwrap() >> 31);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 7: JALR sets bit 0 of the target to 0
    // JALR always produces a 2-byte aligned address (bit 0 = 0).
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_jalr_clears_bit0() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0x100).unwrap();

        // base + imm with an odd result
        cpu.set_x(1, 0x200).unwrap();
        cpu.execute(Jalr {
            rd: 2,
            rs1: 1,
            imm: 1,
        })
        .unwrap();
        assert_eq!(
            cpu.get_pc() & 1,
            0,
            "JALR must set bit 0 of the destination PC to zero"
        );

        cpu.set_pc(0x100).unwrap();
        cpu.set_x(1, 0x201).unwrap();
        cpu.execute(Jalr {
            rd: 2,
            rs1: 1,
            imm: 0,
        })
        .unwrap();
        assert_eq!(
            cpu.get_pc() & 1,
            0,
            "An odd-based JALR should set bit 0 to zero"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 8: Unknown ecall returns “Err”; do not panic
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_unknown_ecall_returns_error() {
        let mut cpu = dummy_cpu();
        cpu.set_x(17, 9999).unwrap(); // syscall not found

        let result = cpu.execute(Ecall);
        assert!(
            matches!(result, Err(CPUError::UnsupportedSyscall(9999))),
            "An invalid system call should return `UnsupportedSyscall`, not panic"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 9: ecall exit(0) for execution with correct code
    // is_running() = false and exit_code = value of a0
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_ecall_exit_halts_cpu() {
        for code in [0i32, 1, -1, 127] {
            let mut cpu = dummy_cpu();
            cpu.set_x(10, code as u32).unwrap(); // a0
            cpu.set_x(17, 93).unwrap(); // a7 = exit

            cpu.execute(Ecall).unwrap();

            assert!(
                !cpu.is_running(),
                "The CPU should stop after the ecall exit (code={code})"
            );
            assert_eq!(
                cpu.get_exit_code(),
                code,
                "The exit code must be the value of a0"
            );
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 10: AUIPC uses the PC *before* any modification
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_auipc_uses_current_pc() {
        let mut cpu = dummy_cpu();

        cpu.set_pc(0x0000_1000).unwrap();
        cpu.execute(Auipc {
            rd: 1,
            imm: 0x0000_5000,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(1).unwrap(),
            0x0000_1000_u32.wrapping_add(0x0000_5000),
            "AUIPC should be added to the current PC"
        );

        // PC = 0 → the result must be exactly 0
        cpu.set_pc(0).unwrap();
        cpu.execute(Auipc { rd: 2, imm: 0x1234 }).unwrap();
        assert_eq!(cpu.get_x(2).unwrap(), 0x1234);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 11: SLT/SLTU — signed vs. unsigned comparisons are distinct
    // The value 0xFFFFFFFF is -1 signed but u32::MAX unsigned.
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_slt_vs_sltu_signed_unsigned() {
        let mut cpu = dummy_cpu();
        cpu.set_x(1, 0xFFFF_FFFF).unwrap(); // -1 signed, u32::MAX unsigned
        cpu.set_x(2, 1).unwrap();

        // Signed: -1 < 1 → true (1)
        cpu.execute(Slt {
            rd: 3,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(cpu.get_x(3).unwrap(), 1, "SLT: -1 < 1 should be true");

        // Unsigned: u32::MAX > 1 → false (0)
        cpu.execute(Sltu {
            rd: 4,
            rs1: 1,
            rs2: 2,
        })
        .unwrap();
        assert_eq!(
            cpu.get_x(4).unwrap(),
            0,
            "SLTU: u32::MAX < 1 should be false"
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INVARIANT 12: Fence, FenceI, and Ebreak are safe no-ops (they do not change state)
    // ─────────────────────────────────────────────────────────────────────────

    #[test]
    fn guardrail_nop_instructions_do_not_change_state() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0x400).unwrap();
        cpu.set_x(1, 0xCAFE).unwrap();

        // Ensures that the CPU is running before the no-ops,
        // regardless of the default initial state
        cpu.set_running(true);

        cpu.execute(Fence).unwrap();
        cpu.execute(FenceI).unwrap();
        cpu.execute(Ebreak).unwrap();

        assert_eq!(cpu.get_pc(), 0x400, "no-ops should not change the PC");
        assert_eq!(
            cpu.get_x(1).unwrap(),
            0xCAFE,
            "no-ops should not change the registers"
        );
        assert!(cpu.is_running(), "no-ops should not stop the CPU");
    }
}
