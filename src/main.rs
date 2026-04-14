use clap::Parser;
use riscv::cli::{Args, Mode};
use riscv::{RiscV, read_elf};
use std::{fs, process};

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let args = Args::parse();

    let bytes = fs::read(&args.file).map_err(|e| format!("failed to read '{}': {e}", args.file))?;

    let elf = read_elf(&bytes).map_err(|e| format!("invalid ELF: {e:?}"))?;

    let mut machine = RiscV::new(elf.clone(), args.mem_kb)
        .map_err(|e| format!("failed to create machine: {e:?}"))?;

    match args.mode {
        Mode::Execute => {
            machine.cpu.run().map_err(|e| format!("{e:?}"))?;
            println!("Exited with code {}", machine.cpu.get_exit_code());
        }

        Mode::Disassemble => {
            machine
                .cpu
                .run_disassemble(elf)
                .map_err(|e| format!("{e:?}"))?;
        }
    }

    Ok(())
}
