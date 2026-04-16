//! # RISC-V Emulator (RV32I+)
//!
//! A lightweight RISC-V virtual machine designed for embedded systems,
//! educational use, and runtime experimentation.
//!
//! It implements a complete execution pipeline for RV32I binaries,
//! from ELF loading to instruction execution.
//!
//! ---
//!
//! ## Architecture Overview
//!
//! The emulator is structured in three layers:
//!
//! ```text
//! RiscV (runtime / system layer)
//!  └── CPU (execution engine)
//!       └── VM (memory + hardware abstraction)
//! ```
//!
//! Each layer has a clear responsibility:
//!
//! - **VM layer** — raw memory and hardware primitives
//! - **CPU layer** — instruction decode + execution logic (RV32I)
//! - **RISC-V layer** — system orchestration and ELF loading
//!
//! ---
//!
//! ## Execution Pipeline
//!
//! A program flows through the system as follows:
//!
//! ```text
//! ELF binary
//!   → read_elf (parsing + validation)
//!   → ElfImage (normalized program representation)
//!   → CPU::new (memory + register initialization)
//!   → RiscV (runtime container)
//!   → execute loop (fetch → decode → execute)
//! ```
//!
//! ---
//!
//! ## Error Model
//!
//! Errors are propagated through a layered hierarchy:
//!
//! - [`vm::VMError`] — memory and hardware faults
//! - [`cpu::CPUError`] — instruction and execution errors
//! - [`risc_v::RiscVError`] — system-level and loader errors
//!
//! Each layer wraps the one below it, preserving full context across the pipeline.
//!
//! ---
//!
//! ## Features
//!
//! - RV32I base integer instruction set
//! - ELF32 loader (little-endian only)
//! - Loadable segment mapping (PT_LOAD)
//! - Full instruction decode/execute pipeline
//! - Syscall support (basic exit syscall)
//! - Deterministic execution model
//!
//! ---
//!
//! ## Usage
//!
//! ### Run an ELF program
//!
//! ```no_run
//! use riscv::{RiscV, read_elf};
//! use std::fs;
//!
//! let bytes = fs::read("program.elf").unwrap();
//! let elf = read_elf(&bytes).unwrap();
//!
//! let mut vm = RiscV::new(elf, 1024).unwrap();
//! vm.cpu.run().unwrap();
//! ```
//!
//! ---
//!
//! ### Load and inspect program without execution
//!
//! ```no_run
//! use riscv::{read_elf};
//! use std::fs;
//!
//! let bytes = fs::read("program.elf").unwrap();
//! let elf = read_elf(&bytes).unwrap();
//!
//! println!("entry point: 0x{:08x}", elf.entry);
//! println!("segments: {}", elf.segments.len());
//! ```
//!
//! ---
//!
//! ## Design Philosophy
//!
//! This project prioritizes:
//!
//! - **Transparency** — every instruction is explicitly decoded and executed
//! - **Determinism** — no hidden state or nondeterministic behavior
//! - **Layer separation** — parsing, execution, and runtime are independent
//! - **Debuggability** — internal state is always inspectable
//!
//! ---
//!
//! ## Target Use Cases
//!
//! - Embedded systems experimentation (e.g. FreeRTOS-style workloads)
//! - Educational CPU architecture study
//! - Emulator / VM research projects
//! - Instruction-level debugging tools
//!
pub mod cli;
pub mod cpu;
pub mod risc_v;
pub mod vm;

pub use risc_v::{ElfImage, ElfSegment, RiscV, RiscVError, read_elf};

pub use vm::VM;
