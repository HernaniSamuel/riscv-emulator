//! # RISC-V Emulator (RV32I+)
//!
//! A RISC-V emulator designed to run embedded systems such as FreeRTOS.
//!
//! ## Architecture
//!
//! The emulator is composed of three layers:
//!
//! - [`vm`] — hardware abstraction
//! - [`cpu`] — instruction execution
//! - [`risc_v`] — system orchestration
//!
//! These layers are composed hierarchically:
//!
//! ```text
//! RiscV
//!  └── CPU
//!       └── VM
//! ```
//!
//! ## Error handling
//!
//! Errors follow a hierarchical structure:
//!
//! - [`vm::VMError`]
//! - [`cpu::CPUError`] (wraps [`vm::VMError`])
//! - [`risc_v::RiscVError`] (wraps [`cpu::CPUError`])
//!
//! Each layer propagates errors from the layer below.
//!
//! ## Examples
//!
//! ### Execute a program
//!
//! ```no_run
//! # use riscv::{RiscV, read_elf};
//! # use std::fs;
//! # let bytes = fs::read("program.elf").unwrap();
//! # let elf = read_elf(&bytes).unwrap();
//! let mut machine = RiscV::new(elf.clone(), 1024).unwrap();
//! machine.cpu.run().unwrap();
//! ```
//!
//! ### Disassemble a program
//!
//! ```no_run
//! # use riscv::{RiscV, read_elf};
//! # use std::fs;
//! # let bytes = fs::read("program.elf").unwrap();
//! # let elf = read_elf(&bytes).unwrap();
//! let mut machine = RiscV::new(elf.clone(), 1024).unwrap();
//! machine.cpu.run_disassemble(elf).unwrap();
//! ```
//!
pub mod cli;
pub mod cpu;
pub mod risc_v;
pub mod vm;

pub use risc_v::{ElfImage, ElfSegment, RiscV, RiscVError, read_elf};

pub use vm::VM;
