# riscv-emulator

[![Docs](https://img.shields.io/badge/docs-rustdoc-blue)](https://hernanisamuel.github.io/riscv-emulator/riscv/index.html)

A single-file, cycle-accurate **RV32IM** emulator written in Rust. Loads and executes ELF binaries bare-metal, with full machine-mode privilege support, a CLINT timer, and an NS16550-compatible UART — capable of running real operating systems such as **FreeRTOS**.

---

## Features

- **Complete RV32I base ISA** — all integer instructions, all load/store widths (`LB`, `LH`, `LW`, `LBU`, `LHU`), branches, `JAL`/`JALR`, `LUI`/`AUIPC`, `FENCE`, `ECALL`, `EBREAK`.
- **RV32M extension** — full multiply/divide suite: `MUL`, `MULH`, `MULHSU`, `MULHU`, `DIV`, `DIVU`, `REM`, `REMU`.
- **Machine-mode privileged architecture** — `mret`, `wfi`, vectored trap delivery, and the full set of M-mode CSRs: `mstatus`, `mie`, `mtvec`, `mepc`, `mcause`.
- **CLINT** — 64-bit `mtime`/`mtimecmp` timer with correct machine timer interrupt (`mcause = 0x8000_0007`) delivery, gated by `mstatus.MIE` and `mie.MTIE`.
- **NS16550-compatible UART** — byte-granular TX forwarded to stdout in real time; LSR always reports TX-ready so drivers never spin-wait.
- **ELF32 loader** — parses program headers, copies `PT_LOAD` segments, zero-fills BSS, and sets the entry point automatically.
- **32 MiB DRAM** — sufficient for FreeRTOS images with typical heap and stack requirements.
- **Zero external dependencies** — the entire emulator is a single `main.rs` using only the Rust standard library.

---

## Highlight: Running FreeRTOS

This emulator successfully boots and runs **FreeRTOS** on RISC-V — a fully preemptive, real-time operating system. The timer interrupt infrastructure (CLINT `mtime`/`mtimecmp`, vectored trap delivery, `mret`-based context restore) is implemented faithfully enough to drive the FreeRTOS tick scheduler without modification.

A ready-to-use FreeRTOS test image targeting this emulator is available at:

> **[https://github.com/HernaniSamuel/freertos-riscv-test](https://github.com/HernaniSamuel/freertos-riscv-test)**

---

## Memory Map

| Region | Base Address   | Size    | Description                              |
|--------|---------------|---------|------------------------------------------|
| UART   | `0x1000_0000` | 4 KiB   | NS16550-compatible serial port           |
| CLINT  | `0x0200_0000` | 64 KiB  | Core-Local Interruptor (timer/interrupt) |
| RAM    | `0x8000_0000` | 32 MiB  | General-purpose DRAM                     |

### UART (`0x1000_0000`)

| Register | Offset | Behaviour                                      |
|----------|--------|------------------------------------------------|
| THR/RBR  | `+0`   | Write: byte forwarded to stdout. Read: `0x00`. |
| LSR      | `+5`   | Always returns `0x20` (THRE set, TX ready).    |

### CLINT (`0x0200_0000`)

| Register    | Address        | Description                           |
|-------------|----------------|---------------------------------------|
| `mtime` lo  | `0x0200_BFF8`  | Low 32 bits of the 64-bit timer       |
| `mtime` hi  | `0x0200_BFFC`  | High 32 bits of the 64-bit timer      |
| `mtimecmp` lo | `0x0200_4000` | Low 32 bits of the compare register  |
| `mtimecmp` hi | `0x0200_4004` | High 32 bits of the compare register |

`mtime` increments by one per instruction step. A machine timer interrupt is raised whenever `mtime >= mtimecmp` (and `mtimecmp != 0`), subject to `mstatus.MIE` and `mie.MTIE`.

---

## Supported `ecall` Numbers

| `a7` (`x17`) | Behaviour                                          |
|--------------|----------------------------------------------------|
| `0`          | Environment call trap (cause 11)                   |
| `93`         | Terminate emulation; exit code taken from `a0`     |

---

## Requirements

- [Rust](https://rustup.rs/) (stable toolchain, 2021 edition or later)

No other dependencies are required.

---

## Building

```sh
git clone https://github.com/HernaniSamuel/riscv-emulator.git
cd riscv-emulator
cargo build --release
```

The compiled binary will be at `target/release/riscv-emulator`.

---

## Usage

```sh
cargo run --release -- <path/to/firmware.elf>
```

Or with the pre-built binary:

```sh
./target/release/riscv-emulator path/to/firmware.elf
```

### Example output

```
segment: vaddr=0x80000000 filesz=4096 memsz=8192
Starting emulation at 0x80000000
Hello from FreeRTOS on RISC-V!
Exit code: 0
```

---

## Running FreeRTOS

1. Clone the FreeRTOS test image repository and follow its build instructions to produce an ELF binary:

```sh
git clone https://github.com/HernaniSamuel/freertos-riscv-test.git
cd freertos-riscv-test
# follow the build instructions in that repo to produce firmware.elf
```

2. Pass the resulting ELF to the emulator:

```sh
cargo run --release -- ../freertos-riscv-test/firmware.elf
```

The emulator will boot FreeRTOS, which will drive the task scheduler via CLINT timer interrupts.

---

## Architecture

The emulator is structured around a single `CPU` struct that encodes the complete processor state:

| Field          | Description                                               |
|----------------|-----------------------------------------------------------|
| `regs[32]`     | General-purpose integer registers (`x0`–`x31`)           |
| `pc`           | Program counter                                           |
| `memory`       | Flat DRAM backing store (32 MiB `Vec<u8>`)                |
| `csr[4096]`    | Full CSR file, indexed by 12-bit address                  |
| `mtime`        | 64-bit CLINT timer counter                                |
| `mtimecmp`     | 64-bit CLINT timer compare register                       |
| `timer_pending`| Latched timer interrupt flag                              |
| `trap_count`   | Diagnostic trap counter                                   |

The execution model follows a strict **fetch → decode → execute** pipeline via `CPU::step`. Before every fetch, pending timer interrupts are checked so that an interrupt can preempt any instruction boundary — the same behaviour required by the FreeRTOS tick handler.

Memory-mapped peripheral dispatch happens inside `read_u8`/`read_u16`/`read_u32` and the corresponding write methods, in priority order: UART → CLINT → RAM → unmapped (silent zero / discard).

---

## ELF Loader

`read_elf` parses a raw byte slice as an ELF32 little-endian RISC-V binary. It validates the magic, class, endianness, and machine fields before iterating the program header table. Only `PT_LOAD` segments are processed; each is copied into emulated RAM with BSS zero-fill. The loader uses `unsafe` pointer casts to `#[repr(C)]` header structs, justified by explicit length checks performed before every cast.

---

## Limitations

- **No floating-point** — RV32F/D/Zfinx are not implemented.
- **No compressed instructions** — RV32C is not supported; the ELF must be compiled without the `C` extension.
- **No virtual memory** — only machine mode is emulated; there is no MMU or S/U-mode support.
- **No serial input** — UART RX always returns `0`; interactive console applications are not supported.
- **Single-core** — no multiprocessor or SMP support.

---

## License

This project is licensed under the [MIT License](LICENSE).