use clap::{Parser, ValueEnum};

/// Command-line arguments for the RISC-V emulator.
#[derive(Parser, Debug)]
#[command(name = "riscv")]
#[command(about = "RV32I emulator with disassembler mode")]
pub struct Args {
    /// Path to the ELF file to execute
    pub file: String,

    /// Execution mode (default: execute)
    #[arg(value_enum, default_value_t = Mode::Execute)]
    pub mode: Mode,

    /// Memory size in kilobytes (default: 64 KB)
    #[arg(default_value_t = 64)]
    pub mem_kb: usize,
}

/// Execution mode of the CPU.
#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
pub enum Mode {
    /// Normal execution (fetch → decode → execute)
    Execute,

    /// Linear disassembly (no execution)
    Disassemble,
}