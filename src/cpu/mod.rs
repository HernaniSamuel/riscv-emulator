pub mod decode;
pub mod execute;
pub mod instruction;

use crate::cpu::instruction::Instruction;
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

    // TODO: document it later
    UnsupportedSyscall(u32),
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

/// Index of the stack pointer register (x2) in the RISC-V ABI.
const REG_SP: usize = 2;

impl CPU {
    /// Creates a new CPU from an ELF image and initialises architectural state.
    ///
    /// Delegates RAM allocation and segment loading to [`VM::new`]. After the
    /// VM is ready, performs the two hardware-level initialisations that a real
    /// RISC-V SoC reset sequence would provide:
    ///
    /// 1. **Program counter** — set to the ELF entry point (handled by the VM).
    /// 2. **Stack pointer (x2)** — set to the top of RAM, 16-byte aligned, as
    ///    required by the RISC-V psABI calling convention. Without this, the
    ///    very first function prologue (`addi sp, sp, -N`) would wrap around to
    ///    `0xFFFF_FFF0` and cause an immediate `MemoryOutOfBounds` fault.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if the ELF image does not fit in `ram_length_kb`
    /// kilobytes or if the image is otherwise invalid.
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, CPUError> {
        let mut vm = VM::new(elf_file, ram_length_kb)?;

        // Initialise the stack pointer to the top of RAM, aligned to 16 bytes.
        //
        // The RISC-V psABI requires the stack to be 16-byte aligned at function
        // entry. We align down from `ram_size` so the first `addi sp, sp, -N`
        // lands on a valid, in-bounds address rather than wrapping around.
        //
        // Note: x0 writes are ignored by the VM, so `ram_size == 0` is the only
        // pathological case — but `VM::new` already rejects zero-size RAM.
        let sp_init = vm.ram_size() as u32 & !0xF;
        vm.set_x(REG_SP, sp_init)?;

        Ok(CPU {
            vm,
            running: false,
            exit_code: 0,
        })
    }

    /// getter for exit_code (I'll document it better later...)
    pub fn get_exit_code(&self) -> i32 {
        self.exit_code
    }

    pub fn set_exit_code(&mut self, value: i32) {
        self.exit_code = value;
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    pub fn set_running(&mut self, value: bool) {
        self.running = value;
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
    /// [`VM::set_pc`] ensures the new address remains within RAM, so no
    /// separate overflow guard is needed here.
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if the resulting address would fall outside
    /// RAM bounds.
    pub fn advance_pc(&mut self) -> Result<(), CPUError> {
        self.set_pc(self.get_pc().wrapping_add(4)) // VM has already confirmed that the PC did not exceed the limits
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

    /// Reads a byte from memory at the given virtual address.
    pub fn read_u8(&self, addr: u32) -> Result<u8, CPUError> {
        Ok(self.vm.read_u8(addr)?)
    }

    /// Writes a byte to memory at the given virtual address.
    pub fn write_u8(&mut self, addr: u32, value: u8) -> Result<(), CPUError> {
        self.vm.write_u8(addr, value)?;
        Ok(())
    }

    /// Reads a halfword from memory at the given address.
    pub fn read_u16(&self, addr: u32) -> Result<u16, CPUError> {
        Ok(self.vm.read_u16(addr)?)
    }

    /// Writes a halfword to memory at the given address.
    pub fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), CPUError> {
        self.vm.write_u16(addr, value)?;
        Ok(())
    }

    /// Reads a word from memory at the given address.
    pub fn read_u32(&self, addr: u32) -> Result<u32, CPUError> {
        Ok(self.vm.read_u32(addr)?)
    }

    /// Writes a word to memory at the given address.
    pub fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), CPUError> {
        self.vm.write_u32(addr, value)?;
        Ok(())
    }

    /// Fetches, decodes and prints one instruction, then advances PC.
    /// This is a linear disassembly mode (no execute stage).
    pub fn disassemble(&mut self) -> Result<Instruction, CPUError> {
        let raw = self.fetch()?;
        let instr = self.decode(raw)?;
        self.advance_pc()?;
        Ok(instr)
    }

    /// Executes one fetch–decode–execute cycle.
    ///
    /// # PC advancement rules
    ///
    /// After `execute()` returns, the PC is advanced **only when all three
    /// conditions hold**:
    ///
    /// 1. The CPU is still running (`self.running == true`). An `ecall` that
    ///    triggers `exit` sets `running = false` before this check, so we
    ///    never attempt to advance past the last instruction.
    /// 2. The PC was **not** modified by the instruction itself. Branch and
    ///    jump instructions update the PC inside `execute()`; for all other
    ///    instructions the PC stays at `pc_before` and we step it forward here.
    /// 3. The resulting address is within RAM. If it isn't, the step fails with
    ///    [`CPUError::VM`] — which is a genuine error, not a normal halt.
    pub fn step(&mut self) -> Result<(), CPUError> {
        if !self.running {
            return Ok(());
        }
        let pc_before = self.get_pc();
        let raw = self.fetch()?;
        let instr = self.decode(raw)?;
        self.execute(instr)?;
        if self.running && self.get_pc() == pc_before {
            self.advance_pc()?;
        }
        Ok(())
    }

    pub fn run(&mut self) -> Result<(), CPUError> {
        self.running = true;

        while self.running {
            self.step()?;
        }

        Ok(())
    }

    pub fn run_disassemble(&mut self, elf: ElfImage) -> Result<(), CPUError> {
        for seg in &elf.segments {
            let start = seg.vaddr;
            let end = seg.vaddr + seg.data.len() as u32;

            self.set_pc(start)?;

            println!("\n== Segment {:#010x} - {:#010x} ==", start, end);

            while self.get_pc() < end {
                let pc = self.get_pc();

                match self.disassemble() {
                    Ok(instr) => println!("{:#010x}: {}", pc, instr),
                    Err(e) => {
                        println!("Stopped at {:#010x}: {:?}", pc, e);
                        break;
                    }
                }
            }
        }

        Ok(())
    }
}

// TESTS
#[cfg(test)]
mod tests {
    use super::*;
    use crate::risc_v::ElfSegment;

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
            // skip x2 (sp) — it is pre-initialised by CPU::new
            if i == REG_SP {
                continue;
            }
            cpu.set_x(i, i as u32 * 10).unwrap();
        }
        for i in 1..32usize {
            if i == REG_SP {
                continue;
            }
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
    fn all_registers_zero_on_reset_except_sp() {
        let cpu = dummy_cpu();
        for i in 0..32usize {
            if i == REG_SP {
                // sp must be initialised to the top of RAM (4 KB = 4096 = 0x1000)
                assert_eq!(
                    cpu.get_x(i).unwrap(),
                    0x1000,
                    "x{i} (sp) should point to top of RAM"
                );
            } else {
                assert_eq!(cpu.get_x(i).unwrap(), 0, "x{i} should be 0 after reset");
            }
        }
    }

    /// Stack pointer must be 16-byte aligned (RISC-V psABI requirement).
    #[test]
    fn initial_sp_is_16_byte_aligned() {
        let cpu = dummy_cpu();
        assert_eq!(cpu.get_x(REG_SP).unwrap() % 16, 0);
    }

    /// Stack pointer must point within RAM bounds so the first `sw` does not fault.
    #[test]
    fn initial_sp_is_within_ram() {
        // 4 KB RAM; the first prologue does `addi sp, sp, -16`, so sp-16 must be >= 0.
        let cpu = dummy_cpu();
        let sp = cpu.get_x(REG_SP).unwrap();
        assert!(sp >= 16, "sp must leave room for at least one stack frame");
    }

    // ── disassembler ──────────────────────────────────────────────────────────
    #[test]
    fn fetch_and_decode_real_instruction() {
        // addi x1, x0, 5
        let instr: u32 = 0x00500093;

        let elf = ElfImage {
            entry: 0,
            segments: vec![ElfSegment {
                vaddr: 0,
                data: instr.to_le_bytes().to_vec(),
                mem_size: 4,
            }],
        };

        let mut cpu = CPU::new(elf, 4096).unwrap();

        let raw = cpu.fetch().unwrap();
        let decoded = cpu.decode(raw).unwrap();

        assert_eq!(
            decoded,
            Instruction::Addi {
                rd: 1,
                rs1: 0,
                imm: 5
            }
        );
    }

    // ── step / run ────────────────────────────────────────────────────────────

    /// A halting ecall (syscall 93) must stop the CPU cleanly without error,
    /// even when it is the very last instruction in RAM.
    #[test]
    fn ecall_exit_at_end_of_ram_does_not_error() {
        // ecall encoding
        const ECALL: u32 = 0x0000_0073;
        // addi x17, x0, 93  (li a7, 93)
        const LI_A7_93: u32 = 0x05D00893;

        // Place the two instructions at the very end of 8-byte RAM so that
        // pc+4 after ecall would be out of bounds.
        let ram_kb = 1;
        let ram_size = ram_kb * 1024;
        let entry = (ram_size - 8) as u32;

        let mut payload = vec![0u8; 8];
        payload[0..4].copy_from_slice(&LI_A7_93.to_le_bytes());
        payload[4..8].copy_from_slice(&ECALL.to_le_bytes());

        let elf = ElfImage {
            entry,
            segments: vec![ElfSegment {
                vaddr: entry,
                data: payload,
                mem_size: 8,
            }],
        };

        let mut cpu = CPU::new(elf, ram_kb).unwrap();
        // Override sp so the test doesn't trip on stack setup
        cpu.vm.set_x(REG_SP, entry).unwrap();

        let result = cpu.run();
        assert!(
            result.is_ok(),
            "run() should return Ok after a clean exit ecall, got: {result:?}"
        );
        assert_eq!(cpu.get_exit_code(), 0);
        assert!(!cpu.is_running());
    }
}
