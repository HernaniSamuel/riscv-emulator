//! # CPU
//!
//! This module implements the RV32I CPU layer of the emulator.
//!
//! It is responsible for instruction semantics and execution flow,
//! orchestrating the underlying hardware abstraction provided by
//! [`crate::vm`].
//!
//! ## Responsibilities
//!
//! - Implement the RV32I instruction set
//! - Control execution flow (fetch–decode–execute)
//! - Enforce ISA-level constraints (alignment, control flow, syscalls)
//! - Provide a higher-level interface over the [`crate::vm::VM`]
//! - Translate low-level errors into [`CPUError`]
//!
//! ## Architecture
//!
//! The CPU operates on top of a [`crate::vm::VM`] instance, which provides:
//!
//! - Memory (RAM)
//! - Register file (`x0..x31`)
//! - Program counter (PC)
//! - Memory-mapped I/O (e.g. UART)
//!
//! The CPU does **not** store architectural state directly. Instead,
//! it encapsulates the VM and applies instruction semantics on top of it.
//!
//! This separation models a classic hardware/software boundary:
//!
//! - **Hardware layer** → [`crate::vm`]
//! - **Execution logic** → `cpu`
//!
//! ## Instruction pipeline
//!
//! The CPU follows a simple in-order 3-stage pipeline:
//!
//! 1. **Fetch** — reads a raw 32-bit instruction from memory
//! 2. **Decode** — converts the opcode into an [`instruction::Instruction`]
//! 3. **Execute** — applies the instruction's effects to the machine state
//!
//! The [`instruction::Instruction`] enum acts as a typed interface between
//! decoding, execution, and disassembly, ensuring a clear separation of concerns.
//!
//! These stages are implemented across submodules:
//!
//! - [`decode`] — instruction decoding logic
//! - [`execute`] — instruction semantics
//! - [`instruction`] — typed instruction representation
//!
//! ## Execution model
//!
//! Execution is driven by repeated calls to [`CPU::step`], which performs
//! a single fetch–decode–execute cycle.
//!
//! [`CPU::run`] builds on top of this by continuously stepping the CPU
//! until a halt condition is reached.
//!
//! The program counter (PC) is advanced automatically after each instruction,
//! except when explicitly modified by control-flow instructions (e.g. branches,
//! jumps, or system calls).
//!
//! ## Execution modes
//!
//! The CPU supports two execution modes:
//!
//! - **Normal execution** — via [`CPU::run`] and [`CPU::step`], which
//!   perform full fetch–decode–execute cycles and mutate machine state.
//!
//! - **Disassembly mode** — via [`CPU::run_disassemble`], which replaces
//!   the execute stage with a textual representation of instructions.
//!
//! In disassembly mode:
//!
//! - Instructions are fetched and decoded normally
//! - No architectural state is modified
//! - Output is written to standard output
//!
//! This mode is useful for debugging, inspection, and tooling.
//!
//! ## Error handling
//!
//! Errors are represented by [`CPUError`].
//!
//! This layer:
//!
//! - wraps [`crate::vm::VMError`]
//! - introduces ISA-level errors (e.g. misalignment, illegal instructions)
//!
//! Higher layers (such as [`crate::risc_v::RiscVError`]) wrap [`CPUError`]
//! to provide full-system context.
//!
//! ## Design notes
//!
//! - The CPU is **single-core** and executes instructions in-order
//! - No pipelining hazards or parallelism are modeled
//! - The design prioritizes clarity and correctness over performance
//! - The module is structured to mirror real CPU architecture concepts

pub mod decode;
pub mod execute;
pub mod instruction;

use crate::cpu::instruction::Instruction;
use crate::risc_v::ElfImage;
use crate::vm::{VM, VMError};

/// Errors that can occur during CPU operation.
///
/// # Design
///
/// This error type represents failures at the instruction execution layer.
///
/// It extends [`crate::vm::VMError`] by introducing errors related to
/// instruction decoding, alignment constraints, and execution semantics.
///
/// Errors from the underlying hardware abstraction ([`crate::vm::VM`])
/// are propagated via [`CPUError::VM`], preserving the original cause.
///
/// # Variants
///
/// - [`CPUError::VM`] — error originating from the VM layer
/// - [`CPUError::IllegalInstruction`] — invalid or unsupported opcode
/// - [`CPUError::InstructionAddressMisaligned`] — PC is not 4-byte aligned
/// - [`CPUError::UnsupportedSyscall`] — unimplemented system call
#[derive(Debug)]
pub enum CPUError {
    /// Wraps an error from the underlying [`crate::vm::VM`].
    ///
    /// This variant is used to propagate low-level failures such as:
    ///
    /// - Out-of-bounds memory access
    /// - Invalid register access
    /// - Invalid program counter updates
    ///
    /// The original [`crate::vm::VMError`] is preserved for precise diagnostics.
    VM(VMError),

    /// The fetched instruction opcode is not defined in the RV32I ISA.
    ///
    /// This error occurs during the **decode stage**, when the raw 32-bit
    /// instruction word cannot be mapped to a valid [`instruction::Instruction`].
    ///
    /// # Payload
    ///
    /// Contains the raw 32-bit instruction word that caused the fault.
    ///
    /// This is useful for debugging invalid binaries or incomplete ISA support.
    IllegalInstruction(u32),

    /// The program counter (PC) is not aligned to a 4-byte boundary.
    ///
    /// The RISC-V ISA requires all instruction fetches to be aligned to
    /// 4-byte boundaries. Any attempt to fetch or set a misaligned PC
    /// results in this error.
    ///
    /// This is enforced during:
    ///
    /// - [`CPU::fetch`]
    /// - [`CPU::set_pc`]
    ///
    /// # Payload
    ///
    /// Contains the misaligned address.
    InstructionAddressMisaligned(u32),

    /// A system call (`ecall`) was invoked but is not supported by the emulator.
    ///
    /// This error is raised during the **execute stage** when handling
    /// environment calls that are not implemented.
    ///
    /// # Payload
    ///
    /// Contains the syscall identifier (typically stored in a register such
    /// as `a7`, depending on the calling convention).
    ///
    /// # Notes
    ///
    /// This variant allows the emulator to explicitly fail on unsupported
    /// functionality instead of silently ignoring it.
    UnsupportedSyscall(u32),
}

impl From<VMError> for CPUError {
    /// Converts a [`crate::vm::VMError`] into a [`CPUError`].
    ///
    /// This enables automatic error propagation using the `?` operator
    /// in CPU methods that interact with the underlying [`crate::vm::VM`].
    fn from(e: VMError) -> Self {
        CPUError::VM(e)
    }
}

/// A single-core RV32I CPU.
///
/// The CPU implements the instruction execution layer of the emulator,
/// providing fetch–decode–execute semantics on top of a [`crate::vm::VM`].
///
/// All architectural state — including memory, registers, and the program
/// counter (PC) — is stored in the underlying [`crate::vm::VM`]. The CPU
/// itself is responsible for applying instruction semantics and managing
/// execution flow.
///
/// # Responsibilities
///
/// - Execute RV32I instructions
/// - Control program flow (PC updates, branching, halting)
/// - Enforce ISA-level constraints (alignment, valid opcodes)
/// - Translate [`crate::vm::VMError`] into [`CPUError`]
/// - Provide a disassembly mode for inspecting binaries
///
/// # Execution model
///
/// The CPU operates on a simple 3-stage pipeline:
///
/// 1. [`CPU::fetch`] — reads the next instruction from memory
/// 2. [`CPU::decode`] — converts it into an [`instruction::Instruction`]
/// 3. [`CPU::execute`] — applies its effects to the machine state
///
/// Higher-level methods such as [`CPU::step`] and [`CPU::run`] orchestrate
/// repeated execution of this pipeline.
///
/// ## Execution modes
///
/// The CPU supports two execution modes:
///
/// - **Normal execution** — via [`CPU::run`] and [`CPU::step`], performing
///   full fetch–decode–execute cycles.
/// - **Disassembly mode** — via [`CPU::run_disassemble`], where instructions
///   are fetched and decoded but not executed, and are instead formatted
///   and printed.
///
/// This allows the same decoding pipeline to be reused for both execution
/// and inspection of binaries.
///
/// # State
///
/// Although the CPU does not store architectural state directly, it maintains:
///
/// - `running` — whether execution should continue
/// - `exit_code` — value returned when the program terminates
///
/// # Invariants
///
/// - The program counter is always 4-byte aligned
/// - Execution stops when `running == false`
/// - All memory and register accesses are validated by the [`crate::vm::VM`]
pub struct CPU {
    vm: VM,

    /// Indicates whether the CPU is currently executing instructions.
    ///
    /// When set to `false`, execution stops on the next cycle.
    running: bool,

    /// Exit code produced by the program.
    ///
    /// This is typically set via an `ecall` that mimics a system `exit`.
    exit_code: i32,
}

/// Index of the stack pointer register (x2) in the RISC-V ABI.
const REG_SP: usize = 2;

impl CPU {
    /// Creates a new CPU from an ELF image and initialises architectural state.
    ///
    /// This constructor builds the CPU on top of a [`VM`] instance, which is
    /// responsible for loading the ELF image into memory and setting the initial
    /// program counter (PC).
    ///
    /// After the VM is initialised, this method performs additional
    /// CPU-level initialisation steps that mirror a real RISC-V reset sequence:
    ///
    /// 1. **Program counter (PC)** — initialised to the ELF entry point
    ///    (handled by [`VM::new`]).
    /// 2. **Stack pointer (x2 / `sp`)** — initialised to the top of RAM,
    ///    aligned to 16 bytes, as required by the RISC-V psABI.
    ///
    /// The stack pointer is aligned *downwards* from the total RAM size to ensure
    /// that the first stack allocation (e.g. `addi sp, sp, -N`) remains within
    /// valid memory bounds.
    ///
    /// Without this initialisation, typical compiled programs would immediately
    /// fault due to stack underflow (e.g. wrapping to `0xFFFF_FFF0`).
    ///
    /// # Errors
    ///
    /// Returns [`CPUError::VM`] if:
    ///
    /// - the ELF image is invalid
    /// - the image does not fit within the specified RAM size
    /// - the underlying [`VM`] fails to initialise
    ///
    /// # Guarantees
    ///
    /// - The CPU starts in a **stopped state** (`running = false`)
    /// - The PC points to a valid instruction within RAM
    /// - The stack pointer (`x2`) is valid and ABI-compliant
    /// - All architectural state is initialised and consistent
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, CPUError> {
        let mut vm = VM::new(elf_file, ram_length_kb)?;

        let sp_init = vm.ram_size() as u32 & !0xF;
        vm.set_x(REG_SP, sp_init)?;

        Ok(CPU {
            vm,
            running: false,
            exit_code: 0,
        })
    }

    /// Returns the program exit code.
    ///
    /// This value is typically set by the program via an `ecall` that mimics
    /// a system `exit`. It becomes meaningful after execution has stopped
    /// (i.e. when [`CPU::run`] or [`CPU::step`] completes).
    ///
    /// # Notes
    ///
    /// - The default value is `0`
    /// - The meaning of the exit code is program-defined
    pub fn get_exit_code(&self) -> i32 {
        self.exit_code
    }

    /// Sets the program exit code.
    ///
    /// This is primarily intended to be used by syscall handlers
    /// (e.g. when implementing `ecall` semantics).
    ///
    /// # Notes
    ///
    /// - Setting the exit code does **not** stop execution
    /// - Callers are expected to also update [`CPU::set_running`]
    ///   if termination is desired
    pub fn set_exit_code(&mut self, value: i32) {
        self.exit_code = value;
    }

    /// Returns whether the CPU is currently executing instructions.
    ///
    /// When this returns `false`, the CPU is considered halted and
    /// no further instructions will be executed unless restarted.
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Sets the CPU execution state.
    ///
    /// When set to `false`, execution will stop on the next cycle
    /// (e.g. in [`CPU::step`] or [`CPU::run`]).
    ///
    /// # Usage
    ///
    /// This is typically controlled internally by the CPU, for example:
    ///
    /// - When handling an `ecall` that terminates the program
    /// - When a fatal error occurs
    ///
    /// External callers may also use this to manually stop execution.
    pub fn set_running(&mut self, value: bool) {
        self.running = value;
    }

    /// Fetches the instruction at the current program counter (PC).
    ///
    /// This corresponds to the *fetch* stage of the classic
    /// fetch–decode–execute pipeline.
    ///
    /// The method enforces the RV32I alignment requirement before accessing
    /// memory: instruction addresses must be 4-byte aligned. If the PC is not
    /// properly aligned, an error is returned immediately and no memory access
    /// is performed.
    ///
    /// On success, returns the raw 32-bit instruction word. Decoding is handled
    /// by [`CPU::decode`] in the next pipeline stage.
    ///
    /// # Errors
    ///
    /// - [`CPUError::InstructionAddressMisaligned`] — if the current PC is not
    ///   4-byte aligned
    /// - [`CPUError::VM`] — if the underlying VM fails to read memory
    ///   (e.g. the PC is outside RAM bounds)
    ///
    /// # Guarantees
    ///
    /// - No memory access occurs if the PC is misaligned
    /// - The VM state is not modified by this operation
    pub fn fetch(&self) -> Result<u32, CPUError> {
        let pc = self.vm.get_pc();

        // ISA rule: instruction must be 4-byte aligned
        if pc & 0b11 != 0 {
            return Err(CPUError::InstructionAddressMisaligned(pc));
        }

        Ok(self.vm.read_u32(pc)?)
    }

    /// Returns the current program counter (PC).
    ///
    /// The PC represents the address of the next instruction to be fetched
    /// during the [`CPU::fetch`] stage.
    ///
    /// This value is stored in the underlying [`crate::vm::VM`] and reflects
    /// the current execution position of the CPU.
    ///
    /// # Notes
    ///
    /// - The PC is always expected to be 4-byte aligned (enforced by [`CPU::set_pc`])
    /// - The returned value is not validated again here
    pub fn get_pc(&self) -> u32 {
        // future: trace
        self.vm.get_pc()
    }

    /// Sets the program counter (PC) to `value`.
    ///
    /// The PC determines the address of the next instruction to be fetched.
    /// This method enforces the RV32I requirement that all instruction
    /// addresses must be 4-byte aligned.
    ///
    /// If `value` is not properly aligned, the error is returned immediately
    /// and the PC is not modified.
    ///
    /// On success, the new PC value is forwarded to the underlying
    /// [`crate::vm::VM`], which performs a bounds check against RAM.
    ///
    /// # Errors
    ///
    /// - [`CPUError::InstructionAddressMisaligned`] — if `value` is not
    ///   4-byte aligned
    /// - [`CPUError::VM`] — if the underlying VM rejects the update
    ///   (e.g. the address is outside RAM)
    ///
    /// # Guarantees
    ///
    /// - The PC is updated only if all checks succeed
    /// - The PC remains unchanged on error
    pub fn set_pc(&mut self, value: u32) -> Result<(), CPUError> {
        if value & 0b11 != 0 {
            return Err(CPUError::InstructionAddressMisaligned(value));
        }

        self.vm.set_pc(value)?;
        Ok(())
    }

    /// Advances the program counter (PC) by 4 bytes (one RV32I instruction).
    ///
    /// This corresponds to moving to the next sequential instruction in the
    /// absence of control flow changes (e.g. branches or jumps).
    ///
    /// Internally, this is equivalent to:
    ///
    /// ```text
    /// set_pc(get_pc() + 4)
    /// ```
    ///
    /// The addition uses wrapping semantics (`wrapping_add`) to avoid
    /// arithmetic overflow panics. The resulting address is then validated
    /// by [`CPU::set_pc`], which enforces alignment and delegates bounds
    /// checking to the underlying [`crate::vm::VM`].
    ///
    /// # Errors
    ///
    /// - [`CPUError::InstructionAddressMisaligned`] — if the resulting PC is
    ///   not 4-byte aligned (should not occur under normal execution)
    /// - [`CPUError::VM`] — if the resulting address is outside RAM bounds
    ///
    /// # Guarantees
    ///
    /// - The PC is advanced only if all checks succeed
    /// - The PC remains unchanged on error
    /// - No intermediate invalid state is observable
    pub fn advance_pc(&mut self) -> Result<(), CPUError> {
        self.set_pc(self.get_pc().wrapping_add(4)) // VM has already confirmed that the PC did not exceed the limits
    }

    /// Returns the value of register `index` (x0–x31).
    ///
    /// This method provides CPU-level access to the architectural register
    /// file, delegating storage and validation to the underlying
    /// [`crate::vm::VM`].
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if `index` is out of the valid range (≥ 32)
    ///
    /// # Notes
    ///
    /// - Register semantics (such as `x0` always reading as zero) are enforced
    ///   by the VM layer
    pub fn get_x(&self, index: usize) -> Result<u32, CPUError> {
        Ok(self.vm.get_x(index)?)
    }

    /// Sets register `index` to `value`.
    ///
    /// This method provides CPU-level access to the architectural register
    /// file while preserving the invariants enforced by the underlying
    /// [`crate::vm::VM`].
    ///
    /// Writes to register `x0` are silently ignored, as required by the
    /// RISC-V ISA.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if `index` is out of the valid range (≥ 32)
    ///
    /// # Guarantees
    ///
    /// - The register file is modified only if the operation succeeds
    /// - Architectural invariants (e.g. `x0 == 0`) are preserved
    pub fn set_x(&mut self, index: usize, value: u32) -> Result<(), CPUError> {
        self.vm.set_x(index, value)?;
        Ok(())
    }

    /// Reads a byte from the given virtual address.
    ///
    /// This method provides CPU-level access to memory and memory-mapped I/O
    /// (MMIO), delegating the operation to the underlying [`crate::vm::VM`].
    ///
    /// In addition to regular RAM accesses, certain address ranges correspond
    /// to hardware devices (e.g. UART). Reads from these regions may have
    /// side effects or return device-specific values.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Notes
    ///
    /// - This method does not distinguish between RAM and MMIO; that logic is
    ///   implemented in the VM layer
    pub fn read_u8(&self, addr: u32) -> Result<u8, CPUError> {
        Ok(self.vm.read_u8(addr)?)
    }

    /// Writes a byte to the given virtual address.
    ///
    /// This method provides CPU-level access to memory and memory-mapped I/O
    /// (MMIO), delegating the operation to the underlying [`crate::vm::VM`].
    ///
    /// Writes to certain address ranges may interact with hardware devices
    /// instead of RAM. For example, writing to the UART transmit register
    /// results in output being produced.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Guarantees
    ///
    /// - The underlying VM ensures memory safety and device semantics
    /// - No partial writes occur on error
    pub fn write_u8(&mut self, addr: u32, value: u8) -> Result<(), CPUError> {
        self.vm.write_u8(addr, value)?;
        Ok(())
    }

    /// Reads a 16-bit halfword from the given virtual address.
    ///
    /// This method provides CPU-level access to memory, delegating the
    /// operation to the underlying [`crate::vm::VM`].
    ///
    /// It is typically used by RV32I load instructions such as `LH` and `LHU`.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Notes
    ///
    /// - Endianness is little-endian, as defined by the RISC-V specification
    /// - Alignment requirements are not enforced here; they are handled
    ///   at the instruction level
    pub fn read_u16(&self, addr: u32) -> Result<u16, CPUError> {
        Ok(self.vm.read_u16(addr)?)
    }

    /// Writes a 16-bit halfword to the given virtual address.
    ///
    /// This method provides CPU-level access to memory, delegating the
    /// operation to the underlying [`crate::vm::VM`].
    ///
    /// It is typically used by RV32I store instructions such as `SH`.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Guarantees
    ///
    /// - The memory is modified only if the operation succeeds
    pub fn write_u16(&mut self, addr: u32, value: u16) -> Result<(), CPUError> {
        self.vm.write_u16(addr, value)?;
        Ok(())
    }

    /// Reads a 32-bit word from the given virtual address.
    ///
    /// This method provides CPU-level access to memory, delegating the
    /// operation to the underlying [`crate::vm::VM`].
    ///
    /// It is typically used by RV32I load instructions such as `LW`.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Notes
    ///
    /// - Endianness is little-endian, as defined by the RISC-V specification
    /// - Instruction fetches also rely on word-sized reads via [`CPU::fetch`]
    pub fn read_u32(&self, addr: u32) -> Result<u32, CPUError> {
        Ok(self.vm.read_u32(addr)?)
    }

    /// Writes a 32-bit word to the given virtual address.
    ///
    /// This method provides CPU-level access to memory, delegating the
    /// operation to the underlying [`crate::vm::VM`].
    ///
    /// It is typically used by RV32I store instructions such as `SW`.
    ///
    /// # Errors
    ///
    /// - [`CPUError::VM`] — if the underlying VM rejects the access
    ///   (e.g. address is outside RAM bounds)
    ///
    /// # Guarantees
    ///
    /// - The memory is modified only if the operation succeeds
    pub fn write_u32(&mut self, addr: u32, value: u32) -> Result<(), CPUError> {
        self.vm.write_u32(addr, value)?;
        Ok(())
    }

    /// Performs a single fetch–decode step and returns the decoded instruction.
    ///
    /// This method implements a *linear disassembly* mode: it fetches the
    /// instruction at the current program counter (PC), decodes it into an
    /// [`Instruction`], and advances the PC by one instruction (4 bytes),
    /// without executing it.
    ///
    /// Unlike [`CPU::step`], this method does not perform the execute stage
    /// and has no side effects on registers or memory.
    ///
    /// # Returns
    ///
    /// The decoded [`Instruction`] corresponding to the current PC.
    ///
    /// # Errors
    ///
    /// - [`CPUError::InstructionAddressMisaligned`] — if the PC is not
    ///   4-byte aligned
    /// - [`CPUError::VM`] — if memory access fails during fetch
    /// - [`CPUError::IllegalInstruction`] — if decoding fails
    ///
    /// # Guarantees
    ///
    /// - No architectural state (registers or memory) is modified
    /// - The PC is advanced only if fetch and decode succeed
    ///
    /// # Notes
    ///
    /// - This method is intended for disassembly tools and debugging
    /// - Control flow instructions (e.g. branches, jumps) are not followed;
    ///   execution proceeds linearly
    pub fn disassemble(&mut self) -> Result<Instruction, CPUError> {
        let raw = self.fetch()?;
        let instr = self.decode(raw)?;
        self.advance_pc()?;
        Ok(instr)
    }

    /// Executes a single fetch–decode–execute cycle.
    ///
    /// This method implements one full step of the CPU pipeline:
    ///
    /// ```text
    /// fetch → decode → execute → (optional PC advance)
    /// ```
    ///
    /// If the CPU is not running (`self.running == false`), this method
    /// returns immediately without performing any work.
    ///
    /// # PC advancement rules
    ///
    /// After [`CPU::execute`] returns, the program counter (PC) is advanced
    /// **only when all of the following conditions hold**:
    ///
    /// 1. The CPU is still running (`self.running == true`). Instructions such
    ///    as `ecall` may stop execution (e.g. program exit) before this step.
    /// 2. The PC was **not** modified by the executed instruction. Control-flow
    ///    instructions (e.g. branches and jumps) update the PC directly during
    ///    execution.
    /// 3. The resulting address is valid. Bounds checking is delegated to
    ///    [`CPU::advance_pc`] and ultimately to the underlying
    ///    [`crate::vm::VM`].
    ///
    /// # Errors
    ///
    /// - [`CPUError::InstructionAddressMisaligned`] — if the PC is not
    ///   4-byte aligned during fetch
    /// - [`CPUError::IllegalInstruction`] — if decoding fails
    /// - [`CPUError::VM`] — if any memory access or PC update fails
    ///
    /// # Guarantees
    ///
    /// - At most one instruction is executed
    /// - The PC is advanced at most once
    /// - No partial state updates occur on error
    /// - If `self.running == false`, no state is modified
    ///
    /// # Notes
    ///
    /// - This method is the core building block of [`CPU::run`]
    /// - Control flow instructions are responsible for updating the PC
    ///   explicitly during the execute stage
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

    /// Runs the CPU until execution stops.
    ///
    /// This method repeatedly executes instructions by calling [`CPU::step`]
    /// in a loop while the CPU is in the running state.
    ///
    /// Execution begins by setting `self.running = true` and continues until
    /// an instruction (e.g. `ecall`) sets it to `false`, or an error occurs.
    ///
    /// # Errors
    ///
    /// - Propagates any error returned by [`CPU::step`]
    ///
    /// # Guarantees
    ///
    /// - Instructions are executed sequentially according to [`CPU::step`]
    /// - Execution stops immediately on error
    /// - The CPU may stop normally by setting `self.running = false`
    ///
    /// # Notes
    ///
    /// - This is the primary execution entry point for programs
    /// - For single-step execution, use [`CPU::step`]
    /// - For disassembly mode, use [`CPU::run_disassemble`]
    pub fn run(&mut self) -> Result<(), CPUError> {
        self.running = true;

        while self.running {
            self.step()?;
        }

        Ok(())
    }

    /// Disassembles all executable segments of an ELF image.
    ///
    /// This method iterates over each loadable segment in the provided [`ElfImage`]
    /// and performs a **linear disassembly** of its contents.
    ///
    /// For each segment:
    ///
    /// 1. The program counter (PC) is set to the segment's start address.
    /// 2. Instructions are fetched and decoded using [`CPU::disassemble`].
    /// 3. Each decoded instruction is printed alongside its address.
    /// 4. The process continues until the end of the segment is reached or an error occurs.
    ///
    /// Unlike [`CPU::run`], this method does **not execute instructions** — it only
    /// decodes and displays them.
    ///
    /// # Output
    ///
    /// The disassembly is printed to standard output in the form:
    ///
    /// ```text
    /// == Segment 0xXXXXXXXX - 0xXXXXXXXX ==
    /// 0xXXXXXXXX: <instruction>
    /// ```
    ///
    /// If an error occurs during decoding, the disassembly stops for the current
    /// segment and reports the failure:
    ///
    /// ```text
    /// Stopped at 0xXXXXXXXX: <error>
    /// ```
    ///
    /// # Errors
    ///
    /// - Returns [`CPUError`] if setting the initial PC fails
    /// - Errors during disassembly are handled internally and reported via output
    ///
    /// # Notes
    ///
    /// - This method performs a **linear sweep disassembly** (no control-flow analysis)
    /// - Invalid or data regions within a segment may produce decoding errors
    /// - Each segment is processed independently
    /// - The CPU `running` state is not used in this mode
    ///
    /// # See also
    ///
    /// - [`CPU::disassemble`] — disassembles a single instruction
    /// - [`CPU::run`] — executes instructions instead of decoding them
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
