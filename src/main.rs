// Copyright 2022 Charles Lohr — original C code under BSD/MIT/CC0.
// Rust port + ELF loader: see README for full credits.

//! Command-line interface for the mini-rv32ima emulator.
//!
//! Parses arguments, sets up the platform, and drives the
//! [`riscv_emulator::emulator::Emulator`] run loop.
//!
//! See `--help` for the full list of options.

#![allow(non_snake_case)]

// In the native binary, import everything from the library crate.
use riscv_emulator::cpu::StepResult;
use riscv_emulator::emulator::{Emulator, RunConfig};
use riscv_emulator::{disasm, elf};

use clap::{ArgGroup, Parser};

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

/// RV32IMA emulator — runs Linux kernels and bare-metal ELF images.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
#[command(group(
    ArgGroup::new("image")
        .required(true)
        .args(["file", "elf"])
))]
struct Args {
    // ── Image source ──────────────────────────────────────────────────────────
    /// Raw Linux / uClinux kernel image.
    #[arg(short = 'f', long, value_name = "FILE")]
    file: Option<String>,

    /// RV32 bare-metal or FreeRTOS ELF image.
    #[arg(short = 'e', long, value_name = "FILE")]
    elf: Option<String>,

    // ── Linux options ─────────────────────────────────────────────────────────
    /// Kernel command line string (raw images only).
    #[arg(short = 'k', long, value_name = "CMDLINE")]
    cmdline: Option<String>,

    /// External DTB file, or `'disable'` to skip DTB (raw images only).
    #[arg(short = 'b', long, value_name = "FILE")]
    dtb: Option<String>,

    // ── Execution options ─────────────────────────────────────────────────────
    /// RAM size in bytes (default: 64 MB).
    #[arg(short = 'm', long, default_value = "67108864", value_name = "BYTES")]
    ram: u32,

    /// Maximum number of instructions to execute (-1 = unlimited).
    #[arg(short = 'c', long, default_value = "-1", value_name = "N")]
    count: i64,

    /// Time divisor: slows the emulated CPU relative to wall-clock time.
    /// Useful for deterministic testing (e.g. `-t 100` = 100x slower CPU).
    #[arg(short = 't', long, default_value = "1", value_name = "N")]
    time_divisor: u32,

    /// Lock the time base to the instruction counter instead of wall-clock time.
    #[arg(short = 'l', long)]
    lock_time: bool,

    /// Disable sleep during WFI (uses 100% of host CPU for maximum throughput).
    #[arg(short = 'p', long)]
    no_sleep: bool,

    /// Execute one instruction at a time, printing the full register state after each.
    #[arg(short = 's', long)]
    single_step: bool,

    /// Halt immediately on any fault instead of dispatching the trap handler.
    #[arg(short = 'd', long)]
    fault_halt: bool,

    // ── Static analysis ───────────────────────────────────────────────────────
    /// Disassemble the entire ELF and exit without running it (requires `-e`).
    #[arg(long, requires = "elf")]
    disasm: bool,

    /// Write a compact execution trace to the given file.
    /// Each line records one instruction and the registers it modified.
    /// Compatible with both `-f` and `-e`. Can be combined with `--single-step`.
    #[arg(long, value_name = "FILE")]
    trace: Option<String>,
}

impl Args {
    /// Build a [`RunConfig`] from the parsed arguments.
    fn run_config(&self) -> RunConfig {
        RunConfig {
            instct: self.count,
            time_divisor: self.time_divisor,
            fixed_update: self.lock_time,
            do_sleep: !self.no_sleep,
            single_step: self.single_step,
            trace: None, // filled in main() after opening the trace file
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

fn main() {
    let args = Args::parse();

    if args.time_divisor == 0 {
        eprintln!("Error: --time-divisor cannot be zero");
        std::process::exit(1);
    }

    // ── Static disassembly mode ───────────────────────────────────────────────
    if args.disasm {
        let elf_file = args.elf.as_deref().unwrap();
        let data = std::fs::read(elf_file).unwrap_or_else(|_| {
            eprintln!("Error: \"{}\" not found", elf_file);
            std::process::exit(1);
        });

        let image = elf::parse_elf(&data).unwrap_or_else(|e| {
            eprintln!("Error parsing ELF: {}", e);
            std::process::exit(1);
        });

        let syms = elf::parse_symbol_table(&data).unwrap_or(None);

        disasm::disasm_elf(&data, &image, syms.as_ref());
        return;
    }

    // ── Normal emulation mode ─────────────────────────────────────────────────
    let has_elf = args.elf.is_some();
    let mut run_cfg = args.run_config();

    let mut emu = Emulator::new(args.ram);
    emu.fail_on_all_faults = args.fault_halt;

    // ── Platform setup ────────────────────────────────────────────────────────
    #[cfg(not(target_os = "windows"))]
    let mut plat = riscv_emulator::platform::posix_platform::PosixPlatform;
    #[cfg(target_os = "windows")]
    let mut plat = riscv_emulator::platform::windows_platform::WindowsPlatform::new();

    use riscv_emulator::platform::Platform as _;
    plat.capture_keyboard();

    // Restore the terminal on exit, even if the emulator panics.
    #[cfg(not(target_os = "windows"))]
    let _term_guard = {
        struct TermGuard;
        impl Drop for TermGuard {
            fn drop(&mut self) {
                riscv_emulator::platform::posix_platform::PosixPlatform.reset_keyboard();
            }
        }
        TermGuard
    };

    // ── Open trace file if requested ──────────────────────────────────────────
    if let Some(ref path) = args.trace {
        match std::fs::File::create(path) {
            Ok(f) => {
                run_cfg.trace = Some(Box::new(std::io::BufWriter::new(f)));
                eprintln!("Trace: writing to \"{}\"", path);
            }
            Err(e) => {
                eprintln!("Error: could not create \"{}\": {}", path, e);
                std::process::exit(1);
            }
        }
    }

    // ── Main loop (supports restart for raw Linux images) ─────────────────────
    let result = loop {
        let load_result = if let Some(ref ef) = args.elf {
            emu.load_elf(ef)
        } else {
            emu.load_raw(
                args.file.as_deref().unwrap(),
                args.dtb.as_deref(),
                args.cmdline.as_deref(),
            )
        };

        if let Err(code) = load_result {
            std::process::exit(code);
        }

        let r = emu.run(&mut run_cfg, &mut plat);

        // Bare-metal ELF images do not support restart; only raw Linux images do.
        if r != StepResult::Restart || has_elf {
            break r;
        }
    };

    // ── Final result ──────────────────────────────────────────────────────────
    if result == StepResult::Poweroff {
        println!("POWEROFF@0x{:08x}{:08x}", emu.cpu.cycleh, emu.cpu.cyclel);
    }
    emu.dump_state(None);
}
