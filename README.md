# riscv-emulator

A RISC-V Rv32IM_Zicsr_Zifencei emulator written in Rust. Loads and executes ELF32 binaries and runs FreeRTOS.

## Features

- Complete Rv32IM base integer instruction set
- ELF32 binary loader and parser
- Instruction-level execution engine
- Static disassembly mode
- Memory-mapped execution model
- UART (MMIO) support
- Deterministic execution semantics

---

## Installation

```bash
git clone https://github.com/HernaniSamuel/riscv-emulator.git
cd riscv-emulator
cargo build --release
```

---

## Usage

```bash
# Execute an ELF binary
cargo run -- <file.elf>

---

- **Execution determinism**
  The emulator enforces deterministic execution semantics:
  - No hidden side effects in instruction execution
  - Stable CPU state transitions for identical inputs
  - Repeatable execution traces

- **ELF loading and execution model**
  Integration tests validate full program loading:
  - Correct parsing of ELF32 binaries
  - Proper segment mapping into virtual memory
  - Correct entry-point initialization
  - Zero-initialization of BSS segments
---

## Goals

This project is primarily educational, built to production-grade engineering standards. It serves as a concrete foundation for understanding CPU execution pipelines, the software/hardware boundary, and layered systems design in Rust.

Planned extensions: RV32M (multiply/divide), expanded syscall support, OS boot exploration.

---

## License

MIT