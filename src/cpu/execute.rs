use crate::cpu::cpu::{CPU, CPUError};
use crate::cpu::instruction::Instruction;
use crate::cpu::instruction::Instruction::*;

impl CPU {
    /// Executes a single RV32I instruction on the CPU's VM.
    ///
    /// Updates registers, memory, and PC as needed. Errors from VM or illegal instructions
    /// are propagated as [`CPUError`].
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
                let shamt = (self.get_x(rs2 as usize)? & 0x1f) as u32;
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
                let shamt = (self.get_x(rs2 as usize)? & 0x1f) as u32;
                let val = self.get_x(rs1 as usize)? >> shamt;
                self.set_x(rd as usize, val)?;
            }
            Sra { rd, rs1, rs2 } => {
                let shamt = (self.get_x(rs2 as usize)? & 0x1f) as u32;
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
                let val = self.get_x(rs1 as usize)? << shamt;
                self.set_x(rd as usize, val)?;
            }
            Srli { rd, rs1, shamt } => {
                let val = self.get_x(rs1 as usize)? >> shamt;
                self.set_x(rd as usize, val)?;
            }
            Srai { rd, rs1, shamt } => {
                let val = (self.get_x(rs1 as usize)? as i32 >> shamt) as u32;
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
                    let target = self.get_pc().wrapping_add(imm as u32);
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bne { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? != self.get_x(rs2 as usize)? {
                    let target = self.get_pc().wrapping_add(imm as u32);
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Blt { rs1, rs2, imm } => {
                if (self.get_x(rs1 as usize)? as i32) < (self.get_x(rs2 as usize)? as i32) {
                    let target = self.get_pc().wrapping_add(imm as u32);
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bge { rs1, rs2, imm } => {
                if (self.get_x(rs1 as usize)? as i32) >= (self.get_x(rs2 as usize)? as i32) {
                    let target = self.get_pc().wrapping_add(imm as u32);
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bltu { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? < self.get_x(rs2 as usize)? {
                    let target = self.get_pc().wrapping_add(imm as u32);
                    self.set_pc(target)?;
                    return Ok(());
                }
            }
            Bgeu { rs1, rs2, imm } => {
                if self.get_x(rs1 as usize)? >= self.get_x(rs2 as usize)? {
                    let target = self.get_pc().wrapping_add(imm as u32);
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
                let next_pc = self.get_pc().wrapping_add(imm as u32);
                self.set_x(rd as usize, self.get_pc().wrapping_add(4))?;
                self.set_pc(next_pc)?;
                return Ok(());
            }
            Jalr { rd, rs1, imm } => { // TODO: make the return work
                let target = (self.get_x(rs1 as usize)? as i32 + imm) as u32 & !1;

                // ret (_start)
                if rd == 0 && rs1 == 1 && target == 0 {
                    self.handle_return()?;
                    return Ok(());
                }

                let ret = self.get_pc();
                self.set_x(rd as usize, ret)?;
                self.set_pc(target)?;
            }

            // ===================== System =====================
            Ecall => (),  // TODO: implement ecall handler
            Ebreak => (), // TODO: implement ebreak handler

            // ===================== Memory ordering =====================
            Fence | FenceI => (), // no-op for RV32I
        }

        // The PC advances linearly if there is no jump or branch
        self.advance_pc()?;
        Ok(())
    }

    fn handle_return(&mut self) -> Result<(), CPUError> {
        let code = self.get_x(10)? as i32; // a0
        self.set_exit_code(code);
        self.set_running(false);
        Ok(())
    }
}
