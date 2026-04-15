# RISC-V Emulator (RV32I+)

A RISC-V emulator written in Rust, designed to explore the boundary between software and hardware.

This project implements a layered architecture capable of executing ELF binaries and is being developed with a strong focus on code quality, tooling, and engineering best practices.

---

## 📚 Documentation

Full API documentation is available here:
👉 https://hernanisamuel.github.io/riscv-emulator/riscv/index.html

---

## 🚀 Features

* RV32I instruction set support
* ELF loading and execution
* Disassembler mode
* Layered architecture (VM → CPU → RISC-V)
* Memory safety guarantees
* Memory-mapped I/O (UART)
* Comprehensive test coverage
* CI/CD integration

---

## 🧱 Architecture

The emulator is structured in three layers:

```
RiscV
 └── CPU
      └── VM
```

* **VM** — hardware abstraction (memory, registers, MMIO)
* **CPU** — instruction decoding and execution
* **RiscV** — system orchestration and integration

This separation enforces clear responsibilities and improves maintainability and correctness.

---

## ⚙️ Installation

Clone the repository:

```
git clone https://github.com/HernaniSamuel/riscv-emulator.git
cd riscv-emulator
```

---

## ▶️ Usage

Run an ELF binary:

```
cargo run <file.elf> execute <ram_kb>
```

Disassemble an ELF binary:

```
cargo run <file.elf> disassemble <ram_kb>
```

Example:

```
cargo run tests/hello.elf execute 1024
```

---

## 🎯 Goals

This project is primarily educational and aims to:

* Understand how computers work at the hardware/software boundary
* Write high-quality, idiomatic Rust
* Apply solid software engineering practices
* Learn project structuring and architecture design
* Use CI/CD effectively
* Follow best practices for Git and GitHub
* Serve as a portfolio project

---

## 🧠 Design Principles

* Clear separation of concerns
* Strong invariants and safety guarantees
* Deterministic and predictable behavior
* Explicit error handling
* Consistent and maintainable codebase

---

## 📄 License

This project is licensed under the MIT License.
