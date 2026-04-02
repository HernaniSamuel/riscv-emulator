use crate::risc_v::ElfImage;
use crate::vm::{VM, VMError};

/// Errors that can occur during CPU operation.
#[derive(Debug)]
pub enum CPUError {
    /// Wraps an error from the underlying [`VM`].
    VM(VMError),

    /// The fetched instruction opcode is not defined in RV32I.
    ///
    /// Contains the raw 32-bit word that caused the fault.
    IllegalInstruction(u32),

    /// The program counter holds an address that is not 4-byte aligned.
    ///
    /// The RISC-V ISA requires all instruction fetches to be aligned to a
    /// 4-byte boundary. Contains the offending address.
    InstructionAddressMisaligned(u32),
}

impl From<VMError> for CPUError {
    fn from(e: VMError) -> Self {
        CPUError::VM(e)
    }
}

/// A single-core RV32I CPU.
///
/// The CPU owns a [`VM`] that provides RAM and the register file. All
/// architectural state (PC, registers, memory) lives inside the VM; the CPU
/// layer adds the fetch/decode/execute pipeline and ISA-level error handling.
pub struct CPU {
    vm: VM,

    /// Whether the CPU is currently executing instructions.
    running: bool,

    /// Exit code set by the program, e.g. via an `ecall` that mimics `exit()`.
    exit_code: i32,
}

impl CPU {
    /// Creates a new CPU from an ELF image.
    ///
    /// Delegates RAM allocation and segment loading to [`VM::new`]. The
    /// program counter is initialised to the ELF entry point.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if the ELF image does not fit in `ram_length_kb`
    /// kilobytes or if the image is otherwise invalid.
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, CPUError> {
        let vm = VM::new(elf_file, ram_length_kb)?;

        Ok(CPU {
            vm,
            running: false,
            exit_code: 0,
        })
    }

    /// Fetches the instruction at the current PC (fetch stage).
    ///
    /// Enforces the RV32I alignment requirement before reading memory:
    /// if the PC is not 4-byte aligned an error is returned immediately and
    /// no memory access is attempted.
    ///
    /// Returns the raw 32-bit instruction word; decoding happens in the next
    /// pipeline stage.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::InstructionAddressMisaligned`] if the current PC
    /// is not 4-byte aligned.
    ///
    /// Returns [`CPUError::VM`] if the PC is outside RAM bounds.
    pub fn fetch(&self) -> Result<u32, CPUError> {
        let pc = self.vm.get_pc();

        // ISA rule: instruction must be 4-byte aligned
        if pc & 0b11 != 0 {
            return Err(CPUError::InstructionAddressMisaligned(pc));
        }

        Ok(self.vm.read_u32(pc)?)
    }

    /// Returns the address of the next instruction to be fetched.
    pub fn get_pc(&self) -> u32 {
        // future: trace
        self.vm.get_pc()
    }

    /// Sets the program counter (PC) to `value`.
    ///
    /// Enforces the RV32I requirement that instruction addresses must be
    /// 4-byte aligned. If `value` is not aligned the error is returned
    /// immediately and the PC is not modified.
    ///
    /// On success, the new PC value is forwarded to the underlying VM, which
    /// performs a bounds check against RAM.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::InstructionAddressMisaligned`] if `value` is not
    /// 4-byte aligned.
    ///
    /// Returns [`CPUError::VM`] if the underlying VM rejects the update
    /// (e.g. the address is outside RAM).
    pub fn set_pc(&mut self, value: u32) -> Result<(), CPUError> {
        if value & 0b11 != 0 {
            return Err(CPUError::InstructionAddressMisaligned(value));
        }

        self.vm.set_pc(value)?;
        Ok(())
    }

    /// Advances the program counter by 4 bytes (one RV32I instruction).
    ///
    /// Equivalent to `set_pc(get_pc() + 4)`. The VM's bounds check on
    /// [`set_pc`] ensures the new address remains within RAM, so no
    /// separate overflow guard is needed here.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if the resulting address would fall outside
    /// RAM bounds.
    pub fn advance_pc(&mut self) -> Result<(), CPUError> {
        self.set_pc(self.get_pc() + 4) // VM has already confirmed that the PC did not exceed the limits
    }

    /// Returns the value of register `index` (x0–x31).
    ///
    /// Forwards the request to the underlying VM.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if `index` is out of the valid range (≥ 32).
    pub fn get_x(&self, index: usize) -> Result<u32, CPUError> {
        Ok(self.vm.get_x(index)?)
    }

    /// Sets register `index` to `value`.
    ///
    /// Writes to `x0` are silently ignored, as required by the RISC-V ISA.
    /// This invariant is enforced by the underlying VM.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if `index` is out of the valid range (≥ 32).
    pub fn set_x(&mut self, index: usize, value: u32) -> Result<(), CPUError> {
        self.vm.set_x(index, value)?;
        Ok(())
    }
}

// TESTS
#[cfg(test)]
mod tests {
    use super::*;

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

    // ── fetch ─────────────────────────────────────────────────────────────────

    #[test]
    fn fetch_ok_when_pc_aligned() {
        let mut cpu = dummy_cpu();
        cpu.vm.set_pc(0).unwrap(); //  force it directly on the VM, without CPU validation
        cpu.vm.write_u32(0, 0xdeadbeef).unwrap();
        assert_eq!(cpu.fetch().unwrap(), 0xdeadbeef);
    }

    #[test]
    fn fetch_err_when_pc_misaligned_by_1() {
        let mut cpu = dummy_cpu();
        cpu.vm.set_pc(1).unwrap(); //  force it directly on the VM, without CPU validation
        assert!(matches!(
            cpu.fetch(),
            Err(CPUError::InstructionAddressMisaligned(1))
        ));
    }

    #[test]
    fn fetch_err_when_pc_misaligned_by_2() {
        let mut cpu = dummy_cpu();
        cpu.vm.set_pc(2).unwrap(); //  force it directly on the VM, without CPU validation
        assert!(matches!(
            cpu.fetch(),
            Err(CPUError::InstructionAddressMisaligned(2))
        ));
    }

    // ── set_pc ────────────────────────────────────────────────────────────────

    #[test]
    fn set_pc_ok_when_aligned() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(8).unwrap();
        assert_eq!(cpu.get_pc(), 8);
    }

    #[test]
    fn set_pc_err_when_misaligned() {
        let mut cpu = dummy_cpu();
        assert!(matches!(
            cpu.set_pc(5),
            Err(CPUError::InstructionAddressMisaligned(5))
        ));
        // The PC shouldn't have changed
        assert_eq!(cpu.get_pc(), 0);
    }

    // ── advance_pc ────────────────────────────────────────────────────────────

    #[test]
    fn advance_pc_increments_by_4() {
        let mut cpu = dummy_cpu();
        cpu.set_pc(0).unwrap();
        cpu.advance_pc().unwrap();
        assert_eq!(cpu.get_pc(), 4);
        cpu.advance_pc().unwrap();
        assert_eq!(cpu.get_pc(), 8);
    }

    // ── registers ─────────────────────────────────────────────────────────────

    #[test]
    fn set_and_get_register_roundtrip() {
        let mut cpu = dummy_cpu();
        for i in 1..32usize {
            cpu.set_x(i, i as u32 * 10).unwrap();
        }
        for i in 1..32usize {
            assert_eq!(cpu.get_x(i).unwrap(), i as u32 * 10, "x{i}");
        }
    }

    /// x0 is hardwired zero — writes should be silently ignored.
    #[test]
    fn write_to_x0_is_ignored() {
        let mut cpu = dummy_cpu();
        cpu.set_x(0, 0xDEAD_BEEF).unwrap();
        assert_eq!(cpu.get_x(0).unwrap(), 0);
    }

    #[test]
    fn get_x_out_of_range_returns_err() {
        let cpu = dummy_cpu();
        assert!(matches!(cpu.get_x(32), Err(CPUError::VM(_))));
    }

    #[test]
    fn set_x_out_of_range_returns_err() {
        let mut cpu = dummy_cpu();
        assert!(matches!(cpu.set_x(32, 42), Err(CPUError::VM(_))));
    }

    // ── initial state ─────────────────────────────────────────────────────────

    #[test]
    fn initial_pc_matches_elf_entry() {
        let elf = ElfImage {
            entry: 0x1000_0000,
            segments: vec![],
        };
        let cpu = CPU::new(elf, 4).unwrap();
        assert_eq!(cpu.get_pc(), 0x1000_0000);
    }

    #[test]
    fn all_registers_zero_on_reset() {
        let cpu = dummy_cpu();
        for i in 0..32usize {
            assert_eq!(cpu.get_x(i).unwrap(), 0, "x{i} should be 0 after reset");
        }
    }
}
