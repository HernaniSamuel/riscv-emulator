//! RISC-V Emulator binary entrypoint.
//!
//! This is the main executable for the RV32IM emulator.
//! It wires together CLI parsing, ELF loading, CPU initialization,
//! and execution or disassembly modes.
//!
//! ## Responsibilities
//!
//! This binary is intentionally thin and acts only as an orchestrator:
//!
//! - Parse CLI arguments ([`cli::Args`])
//! - Load ELF files ([`read_elf`])
//! - Initialize the virtual machine ([`RiscV`])
//! - Execute or disassemble the program
//!
//! All execution logic lives in the library modules (`cpu`, `vm`, `risc_v`).
//!
//! ## Error handling
//!
//! Errors are converted into user-facing strings at the boundary layer.
//! Internal error types are not exposed directly to the CLI output.

use clap::Parser;
use riscv::cli::Args;
use riscv::{RiscV, read_elf};
use std::{fs, process};

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        process::exit(1);
    }
}

/// Entry point logic for the emulator.
///
/// This function performs all high-level orchestration:
/// - CLI parsing
/// - file loading
/// - ELF decoding
/// - CPU execution or disassembly
///
/// It returns a user-friendly error string because this is the CLI boundary layer.
///
/// No emulator logic should exist in this function.
fn run() -> Result<(), String> {
    let args = Args::parse();

    let file = args.file.ok_or("no input file provided")?;

    let bytes = fs::read(&file).map_err(|e| format!("failed to read '{file}': {e}"))?;

    let elf = read_elf(&bytes).map_err(|e| format!("invalid ELF: {e:?}"))?;

    let mut machine = RiscV::new(elf.clone(), args.memsize)
        .map_err(|e| format!("failed to create machine: {e:?}"))?;

    if args.disasm {
        machine
            .cpu
            .run_disassemble(elf)
            .map_err(|e| format!("{e:?}"))?;
    } else {
        machine.cpu.run().map_err(|e| format!("{e:?}"))?;

        println!("Exited with code {}", machine.cpu.get_exit_code());
    }

    Ok(())
}
