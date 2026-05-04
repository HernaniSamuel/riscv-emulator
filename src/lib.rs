//! # riscv-emulator — A RISC-V RV32IMA Emulator
//!
//! A cycle-accurate emulator for the RISC-V 32-bit base integer ISA with the
//! **M** (multiply/divide), **A** (atomics), and machine-level privileged
//! extensions. It runs real Linux kernels and FreeRTOS images, and compiles
//! to WebAssembly for in-browser execution.
//!
//! ## Architecture overview
//!
//! The emulator is split into focused modules:
//!
//! | Module | Responsibility |
//! |--------|---------------|
//! | [`cpu`] | CPU state, trap handling, timer |
//! | [`emulator`] | Main execution loop, image loading |
//! | [`mmio`] | Memory-mapped I/O (UART, CLINT, SYSCON) |
//! | [`platform`] | OS abstraction (time, keyboard, sleep) |
//! | [`elf`] | ELF32 parser and symbol table |
//! | [`disasm`] | RV32IMA disassembler |
//! | [`dtb`] | Embedded device tree blob |
//! | wasm | WebAssembly bindings (wasm32 only) |
//!
//! ## Memory map
//!
//! ```text
//! 0x1000_0000  UART 16550  TX/RX + LSR
//! 0x1100_4000  CLINT       mtimecmp low
//! 0x1100_4004  CLINT       mtimecmp high
//! 0x1100_BFF8  CLINT       mtime low  (read-only)
//! 0x1100_BFFC  CLINT       mtime high (read-only)
//! 0x1110_0000  SYSCON      0x5555 = poweroff, 0x7777 = restart
//! 0x8000_0000  RAM start   (default 64 MB)
//! ```
//!
//! ## Execution model
//!
//! Each call to [`emulator::Emulator::step`] executes up to `count`
//! instructions in a tight loop and returns a [`cpu::StepResult`] indicating
//! why it stopped. The caller is responsible for advancing wall-clock time and
//! passing `elapsed_us` (microseconds since the last call) so the CLINT timer
//! stays accurate.
//!
//! The high-level [`emulator::Emulator::run`] method wraps this loop with
//! timing, WFI sleep, and instruction-count limiting.
//!
//! ## Trap handling
//!
//! Traps (exceptions and interrupts) follow the RISC-V privileged spec 1.11.
//! When a trap is detected inside `step`, [`cpu::CpuState::commit_trap`] saves
//! the machine context into the CSRs and redirects the PC to `mtvec`. The
//! kernel's trap handler is then responsible for dispatching and returning via
//! `mret`.
//!
//! ## Extending the emulator
//!
//! - **New peripherals**: add address ranges to [`mmio`] and handle them in
//!   `handle_store` / `handle_load`.
//! - **New instructions**: add opcode arms to the `match ir & 0x7f` block in
//!   [`emulator::Emulator::step`].
//! - **New platforms**: implement the [`platform::Platform`] trait.

#![allow(non_snake_case)]

pub mod cpu;
pub mod disasm;
pub mod dtb;
pub mod elf;
pub mod emulator;
pub mod mmio;
pub mod platform;

/// WebAssembly bindings — only compiled when targeting `wasm32`.
#[cfg(target_arch = "wasm32")]
pub mod wasm;
