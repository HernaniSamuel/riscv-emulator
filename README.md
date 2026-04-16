# riscv-emulator

A RISC-V RV32I emulator written in Rust. Loads and executes ELF32 binaries through a clean, layered architecture with a focus on correctness, determinism, and systems-level design.

Full API documentation: https://hernanisamuel.github.io/riscv-emulator/riscv/index.html

---

## Architecture

The emulator is organized into three explicit layers, each with a single, well-defined responsibility:

```
RiscV (system orchestration)
└── CPU (instruction execution / ISA semantics)
    └── VM (hardware abstraction layer)
```

**VM layer** — models low-level hardware behavior: RAM, register file, memory-mapped I/O, and peripheral devices (UART). All hardware concerns are isolated here.

**CPU layer** — implements the RV32I execution model: instruction decoding, arithmetic and logical operations, branching, memory access (load/store), system calls (ECALL), and exception handling. Responsible for ISA correctness, not hardware simulation.

**RiscV layer** — top-level system orchestrator: initializes the CPU from a parsed ELF image and delegates execution to the selected mode (run or disassembly).

---

## Features

- Complete RV32I base integer instruction set
- ELF32 binary loader and parser
- Instruction-level execution engine
- Static disassembly mode
- Memory-mapped execution model
- UART (MMIO) support
- Deterministic execution semantics
- Explicit error propagation across all layers
- Extensive unit and integration test suite

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

# Execute with custom memory size (KB)
cargo run -- <file.elf> --memsize 1024

# Disassemble without executing
cargo run -- <file.elf> --disasm
```

---

## Design Principles

- **Strict layer separation**: hardware abstraction (VM), ISA semantics (CPU), and system orchestration (RiscV) are fully decoupled. No layer reaches into the concerns of another.
- **Deterministic execution**: given the same binary and memory configuration, execution is fully reproducible. No hidden state, no implicit side effects.
- **Explicit error handling**: errors are typed and propagated across layer boundaries. No panics in normal execution paths.
- **No implicit side effects in instruction execution**: each instruction variant in the IR documents and produces only the effects it declares. The execution stage assumes well-formed inputs.
- **Tests as specifications**: the test suite encodes RV32I compliance, CPU invariants (x0 hardwiring, PC behavior, sign extension), memory safety guarantees, and edge cases. Tests are the authoritative behavioral specification.

---

## Testing

```bash
cargo test
```

### Test Coverage Philosophy

The test suite is designed as an executable specification of the emulator’s behavior.

It validates correctness at multiple layers of abstraction:

- **ISA-level correctness (RV32I semantics)**
  Instruction decoding and execution are validated against expected architectural behavior, including arithmetic, logical, memory, and control-flow operations.

- **Architectural invariants**
  Core CPU properties are enforced as hard guarantees:
  - Register x0 remains immutable
  - Program counter updates follow strict alignment rules
  - Immediate values are correctly sign-extended
  - Control-flow instructions preserve architectural consistency

- **Memory model correctness**
  The VM layer is validated for safe and deterministic behavior:
  - Bounds-checked memory access
  - Little-endian consistency
  - Safe handling of invalid reads/writes
  - Isolation between registers and memory state

- **Control-flow semantics**
  Branching and jump instructions are tested for:
  - Correct target resolution
  - Proper PC mutation behavior
  - Preservation of state on untaken branches

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