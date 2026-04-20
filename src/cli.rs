//! Command-line interface for the RISC-V emulator.
//!
//! This module defines the CLI entrypoint used by the `riscv` binary,
//! powered by [`clap`] for argument parsing.
//!
//! ## Purpose
//!
//! The CLI provides a minimal interface for running ELF binaries on the emulator
//! or inspecting them via the built-in disassembler.
//!
//! It is intentionally thin: all real logic lives in [`crate::cpu`], [`crate::vm`],
//! and [`crate::risc_v`].
//!
//! ## Usage modes
//!
//! The emulator supports two primary modes:
//!
//! - Execution mode (default): runs the ELF program
//! - Disassembly mode (`--disasm`): prints decoded instructions without executing
//!
//! ## Example usage
//!
//! ```bash
//! riscv program.elf
//!
//! riscv program.elf --disasm
//!
//! riscv program.elf --memsize 256
//! ```
//!
//! ## Design notes
//!
//! - The CLI is intentionally minimal and non-opinionated
//! - Defaults are chosen to match embedded-style constraints (small RAM)
//! - Parsing errors are handled entirely by [`clap`]
//!
//! [`clap`]: https://docs.rs/clap

use clap::Parser;

/// Command-line arguments for the RISC-V emulator.
///
/// This struct defines the public interface of the `riscv` binary.
/// It is parsed automatically by [`clap`] at startup.
///
/// # Fields
///
/// - `file`: ELF file to load and execute or disassemble
/// - `disasm`: enables disassembly mode instead of execution
/// - `memsize`: RAM size in kilobytes (default: 64 KB)
///
/// This struct does not contain any execution logic.
#[derive(Parser, Debug)]
#[command(name = "riscv")]
#[command(about = "RV32IM emulator")]
pub struct Args {
    /// Input ELF file (optional positional argument)
    pub file: Option<String>,

    /// Run disassembler instead of execution
    #[arg(long)]
    pub disasm: bool,

    /// Memory size in KB (default: 64)
    #[arg(long = "memsize", default_value_t = 64)]
    pub memsize: usize,
}
