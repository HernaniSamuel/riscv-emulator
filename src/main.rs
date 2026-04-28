//! Cycle-accurate RV32IM emulator capable of loading and executing ELF binaries.
//!
//! Successfully boots and runs **FreeRTOS** using CLINT timer interrupts,
//! vectored traps, and machine-mode context restore.
//!
//! # Supported ISA
//!
//! - **RV32I** – the complete base integer instruction set (all load/store widths,
//!   branches, jumps, fence, `ecall`, `ebreak`, CSR instructions).
//! - **RV32M** – the standard integer multiply/divide extension (`mul`, `mulh`,
//!   `mulhsu`, `mulhu`, `div`, `divu`, `rem`, `remu`).
//! - **Machine-mode** privileged architecture: `mret`, `wfi`, CSRs (`mstatus`,
//!   `mie`, `mtvec`, `mepc`, `mcause`), and vectored trap delivery.
//!
//! # Memory Map
//!
//! | Region | Base address | Size | Description |
//! |--------|-------------|------|-------------|
//! | UART   | `0x1000_0000` | 4 KiB | NS16550-compatible serial port |
//! | CLINT  | `0x0200_0000` | 64 KiB | Core-Local Interruptor (timer) |
//! | RAM    | `0x8000_0000` | [`MEM_SIZE`] | General-purpose DRAM |
//!
//! # Peripherals
//!
//! ## UART (`0x1000_0000`)
//! A minimal NS16550-compatible UART. Writes to the TX register (`0x1000_0000`)
//! are forwarded directly to stdout. The LSR register always reports TX ready.
//! RX reads always return `0` (no input support).
//!
//! ## CLINT (`0x0200_0000`)
//! Implements `mtime` (a 64-bit monotonically increasing counter) and `mtimecmp`
//! (the 64-bit compare threshold). When `mtime >= mtimecmp`, a machine timer
//! interrupt (cause `0x8000_0007`) is raised, subject to `mstatus.MIE` and
//! `mie.MTIE`. The counter increments by one on every call to [`CPU::tick`].
//!
//! # Execution Model
//!
//! The emulator follows a strict fetch → decode → execute pipeline executed one
//! instruction at a time via [`CPU::step`]. Timer interrupts are checked before
//! every fetch so that an interrupt can preempt any instruction.
//!
//! # Supported `ecall` Numbers
//!
//! | `a7` (x17) | Behaviour |
//! |-----------|-----------|
//! | `0`       | Triggers a trap with cause 11 (environment call) |
//! | `93`      | Terminates emulation; exit code is taken from `a0` (x10) |
//!
//! # Example
//!
//! ```text
//! $ cargo run -- path/to/firmware.elf
//! segment: vaddr=0x80000000 filesz=4096 memsz=8192
//! Starting emulation at 0x80000000
//! Hello, world!
//! Exit code: 0
//! ```

use std::env;
use std::fmt;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args().nth(1).expect("usage: emulator <elf>");

    let data = fs::read(path)?;
    let elf = read_elf(&data).expect("invalid ELF");

    let mut cpu = CPU::new(MEM_SIZE);

    cpu.regs[2] = (RAM_BASE as u32) + (MEM_SIZE as u32) - 4;

    cpu.load_elf(&elf).expect("failed to load elf");

    cpu.pc = elf.entry;

    println!("Starting emulation at 0x{:08x}", cpu.pc);

    cpu.running = true;
    while cpu.running {
        cpu.step();
    }
    println!("Exit code: {}", cpu.exit_code);

    Ok(())
}

// ── CSR addresses ────────────────────────────────────────────────────────────

/// CSR address of the Machine Status register (`mstatus`).
///
/// Controls global interrupt enable and records prior privilege state on trap
/// entry. The emulator reads and writes the `MIE`, `MPIE`, and `MPP` fields.
pub const MSTATUS: usize = 0x300;

/// CSR address of the Machine Interrupt-Enable register (`mie`).
///
/// Each bit enables a specific interrupt source. The emulator only uses
/// [`MIE_MTIE`] (bit 7) to gate machine timer interrupts.
pub const MIE_CSR: usize = 0x304;

/// CSR address of the Machine Trap-Vector Base-Address register (`mtvec`).
///
/// Holds the base address of the trap handler. The lowest two bits encode the
/// trap mode; the emulator masks them off and jumps to `mtvec & !0b11` on any
/// trap or interrupt. If `mtvec` is zero the trap is silently ignored.
pub const MTVEC: usize = 0x305;

/// CSR address of the Machine Exception Program Counter (`mepc`).
///
/// Saved automatically by [`CPU::trap`] to hold the PC of the instruction that
/// was interrupted or caused the exception. Restored by `mret`.
pub const MEPC: usize = 0x341;

/// CSR address of the Machine Cause register (`mcause`).
///
/// Written by [`CPU::trap`] with the exception or interrupt cause code.
/// Bit 31 is set for asynchronous interrupts (`0x8000_0007` for machine timer).
pub const MCAUSE: usize = 0x342;

/// CSR address of the `mtime` low-word alias (read-only shadow, non-standard).
///
/// Not used for writes; the authoritative timer state lives in [`CPU::mtime`]
/// and is accessed through the CLINT memory-mapped registers.
pub const MTIME: usize = 0xC01;

/// CSR address of the `mtimecmp` low-word alias (non-standard).
///
/// Not used for writes; the authoritative compare value lives in
/// [`CPU::mtimecmp`] and is accessed through the CLINT memory-mapped registers.
pub const MTIMECMP: usize = 0xC80;

// ── Memory sizing ─────────────────────────────────────────────────────────────

/// Size of the emulated DRAM in bytes (32 MiB).
///
/// Chosen to be large enough to host a FreeRTOS image with typical stack and
/// heap requirements. The RAM region begins at [`RAM_BASE`].
pub const MEM_SIZE: usize = 32 * 1024 * 1024;

// ── CLINT memory-mapped register addresses ───────────────────────────────────

/// Memory-mapped address of the low 32 bits of `mtime`.
///
/// Reading returns bits `[31:0]` of the 64-bit `mtime` counter. Writing updates
/// those bits while preserving the high half.
pub const CLINT_MTIME: u32 = 0x0200_BFF8;

/// Memory-mapped address of the high 32 bits of `mtime` (`CLINT_MTIME + 4`).
///
/// Reading returns bits `[63:32]` of `mtime`. Writing updates those bits while
/// preserving the low half.
pub const CLINT_MTIME_PLUS_4: u32 = CLINT_MTIME + 4;

/// Memory-mapped address of the low 32 bits of `mtimecmp`.
///
/// Writing this register may clear [`CPU::timer_pending`] if the resulting
/// 64-bit `mtimecmp` value is strictly greater than the current `mtime`.
/// Software typically writes the low half last to avoid a spurious interrupt
/// window; beware that writing only the low half leaves the high half intact.
pub const CLINT_MTIMECMP: u32 = 0x0200_4000;

/// Memory-mapped address of the high 32 bits of `mtimecmp` (`CLINT_MTIMECMP + 4`).
///
/// Writing this half clears [`CPU::timer_pending`] only when the combined
/// 64-bit `mtimecmp` (with the existing low half) exceeds `mtime`. Because the
/// low half is unchanged at this point, software must ensure the final combined
/// value is correct before relying on the pending-clear behavior.
pub const CLINT_MTIMECMP_PLUS_4: u32 = CLINT_MTIMECMP + 4;

/// Inclusive start address of the CLINT memory-mapped region.
pub const CLINT_BASE: u32 = 0x0200_0000;

/// Exclusive end address of the CLINT memory-mapped region.
pub const CLINT_END: u32 = 0x0201_0000;

/// Base address of the emulated DRAM.
///
/// All ELF segments with a virtual address at or above this value are loaded
/// into the backing [`CPU::memory`] buffer. The stack pointer is initialized to
/// `RAM_BASE + MEM_SIZE - 4` before execution begins.
pub const RAM_BASE: usize = 0x8000_0000;

// ── UART memory-mapped register addresses ────────────────────────────────────

/// Base address of the NS16550-compatible UART peripheral.
pub const UART_BASE: u32 = 0x1000_0000;

/// Size of the UART address window in bytes (4 KiB).
pub const UART_SIZE: u32 = 0x1000;

/// Address of the UART Transmit Holding Register.
///
/// A byte written here is immediately forwarded to stdout. Writes of the null
/// byte (`0x00`) are suppressed.
pub const UART_TX: u32 = UART_BASE;

/// Address of the UART Receive Buffer Register.
///
/// Reads always return `0`; the emulator does not model serial input.
pub const UART_RX: u32 = UART_BASE;

/// Address of the UART Line Status Register (LSR).
///
/// Reads always return [`TX_READY`], indicating the transmitter is never busy.
pub const UART_LSR: u32 = UART_BASE + 5;

/// Value returned by reads from [`UART_LSR`].
///
/// Bit 5 (`THRE` – Transmitter Holding Register Empty) is permanently set,
/// telling the driver that the UART is always ready to accept a new byte.
pub const TX_READY: u8 = 0x20;

// ── mstatus bit masks ─────────────────────────────────────────────────────────

/// `mstatus` bit 3 – Machine Interrupt Enable.
///
/// When set, machine-mode interrupts are globally enabled. [`CPU::trap`] clears
/// this bit and saves its previous value in [`MSTATUS_MPIE`]. `mret` restores
/// it from `MPIE`.
pub const MSTATUS_MIE: u32 = 1 << 3;

/// `mstatus` bit 7 – Machine Previous Interrupt Enable.
///
/// Captures the state of [`MSTATUS_MIE`] at the moment a trap is taken.
/// `mret` copies this bit back into `MIE`, effectively restoring the prior
/// interrupt-enable state.
pub const MSTATUS_MPIE: u32 = 1 << 7;

/// `mstatus` bits `[12:11]` mask – Machine Previous Privilege mode.
///
/// Set to `0b11` (Machine mode) by [`CPU::trap`] and cleared to `0b00` by
/// `mret`. The emulator operates exclusively in Machine mode, so this field
/// is always forced to `0b11` on trap entry.
pub const MSTATUS_MPP_MASK: u32 = 0b11 << 11;

// ── mie bit masks ─────────────────────────────────────────────────────────────

/// `mie` bit 7 – Machine Timer Interrupt Enable.
///
/// When set together with [`MSTATUS_MIE`], a pending timer interrupt
/// (signalled by [`CPU::timer_pending`]) will cause the CPU to enter the trap
/// handler with cause `0x8000_0007`.
pub const MIE_MTIE: u32 = 1 << 7;

// ── Address-space helpers ─────────────────────────────────────────────────────

/// Returns `true` if `addr` falls within the UART peripheral window.
fn is_uart(addr: u32) -> bool {
    (UART_BASE..UART_BASE + UART_SIZE).contains(&addr)
}

/// Returns `true` if `addr` falls within the CLINT peripheral window.
fn is_clint(addr: u32) -> bool {
    (CLINT_BASE..CLINT_END).contains(&addr)
}

/// Returns `true` if `addr` falls within the emulated DRAM region.
fn is_ram(addr: u32, mem_size: usize) -> bool {
    let start = RAM_BASE as u32;
    let end = start + mem_size as u32;
    addr >= start && addr < end
}

/// Converts an absolute guest address into a byte index into [`CPU::memory`].
///
/// The caller must verify `is_ram(addr, ...)` before calling this function;
/// no bounds checking is performed here.
fn to_ram_index(addr: u32) -> usize {
    (addr as usize).wrapping_sub(RAM_BASE)
}

// ── CPU ───────────────────────────────────────────────────────────────────────

/// The RISC-V RV32IM processor state and execution engine.
///
/// `CPU` combines architectural registers, a flat byte-addressed memory image,
/// CSR state, and CLINT timer counters into a single structure. All peripheral
/// accesses are dispatched from the memory-read/write methods rather than
/// through a separate bus abstraction.
///
/// # Invariants
///
/// - `regs[0]` is always forced back to `0` at the end of every [`CPU::step`]
///   call to uphold the `x0` hard-wired-zero invariant.
/// - `pc` must never be `0` or `0xffff_ffff` when [`CPU::fetch`] is called;
///   violating this causes a panic.
pub struct CPU {
    /// The 32 general-purpose integer registers (`x0`–`x31`).
    ///
    /// `regs[0]` (`x0`) is hard-wired to zero and is restored to `0` at the
    /// end of every step even if an instruction writes to it.
    pub regs: [u32; 32],

    /// The program counter: address of the instruction to be fetched next.
    ///
    /// Updated to `pc + 4` after a sequential instruction, or to the branch/
    /// jump target otherwise. On trap entry it is saved to `mepc` and replaced
    /// with the handler address from `mtvec`.
    pub pc: u32,

    /// Backing store for the emulated DRAM region.
    ///
    /// Indexed via [`to_ram_index`]; its length equals the `mem_size` passed to
    /// [`CPU::new`]. ELF segments are copied here by [`CPU::load_elf`].
    pub memory: Vec<u8>,

    /// Controls the main execution loop in `main`.
    ///
    /// Set to `true` before the loop begins; cleared when an `ebreak`
    /// instruction is encountered or when `ecall` with `a7 = 93` is executed.
    pub running: bool,

    /// The exit code reported when emulation terminates.
    ///
    /// Initialized to `1000` as a sentinel. Set to the value of `a0` (x10)
    /// when `ecall 93` is executed. Printed to stdout after the loop exits.
    pub exit_code: i32,

    /// Indicates that a machine timer interrupt is waiting to be delivered.
    ///
    /// Set to `true` by [`CPU::tick`] when `mtime >= mtimecmp`. Cleared when
    /// the interrupt is actually taken (subject to `mstatus.MIE` and `mie.MTIE`
    /// being set), or when software programs a new `mtimecmp` value strictly
    /// greater than the current `mtime`.
    pub timer_pending: bool,

    /// The complete 4096-entry CSR file indexed by 12-bit CSR address.
    ///
    /// Only a handful of entries are architecturally meaningful to the emulator
    /// (`mstatus`, `mie`, `mtvec`, `mepc`, `mcause`). All other locations are
    /// read/write but have no side effects.
    pub csr: [u32; 4096],

    /// The 64-bit machine timer counter, memory-mapped at [`CLINT_MTIME`].
    ///
    /// Incremented by one on every call to [`CPU::tick`] (i.e., once per
    /// instruction step). Wraps on overflow.
    pub mtime: u64,

    /// The 64-bit machine timer compare register, memory-mapped at
    /// [`CLINT_MTIMECMP`].
    ///
    /// When `mtime >= mtimecmp` (and `mtimecmp != 0`), [`CPU::timer_pending`]
    /// is set. Software clears a pending interrupt by writing a future deadline
    /// to this register.
    pub mtimecmp: u64,
}

impl CPU {
    /// Creates a new `CPU` with `mem_size` bytes of zeroed DRAM and all
    /// registers, CSRs, and counters initialised to zero.
    ///
    /// The stack pointer (`x2`) and program counter are **not** set here;
    /// the caller is responsible for initialising them before calling
    /// [`CPU::step`].
    pub fn new(mem_size: usize) -> Self {
        Self {
            regs: [0; 32],
            pc: 0,
            memory: vec![0; mem_size],
            csr: [0; 4096],
            mtime: 0,
            mtimecmp: 0,
            running: false,
            exit_code: 1000,
            timer_pending: false,
        }
    }

    /// Reads a 32-bit value from a CLINT memory-mapped register.
    ///
    /// Only [`CLINT_MTIME`], [`CLINT_MTIME_PLUS_4`], [`CLINT_MTIMECMP`], and
    /// [`CLINT_MTIMECMP_PLUS_4`] are meaningful; all other addresses return `0`.
    fn clint_read(&self, addr: u32) -> u32 {
        match addr {
            CLINT_MTIME => self.mtime as u32,
            CLINT_MTIME_PLUS_4 => (self.mtime >> 32) as u32,
            CLINT_MTIMECMP => self.mtimecmp as u32,
            CLINT_MTIMECMP_PLUS_4 => (self.mtimecmp >> 32) as u32,
            _ => 0,
        }
    }

    /// Writes a 32-bit value to a CLINT memory-mapped register.
    ///
    /// Writing [`CLINT_MTIMECMP`] or [`CLINT_MTIMECMP_PLUS_4`] may clear
    /// [`CPU::timer_pending`] if the resulting 64-bit `mtimecmp` is strictly
    /// greater than `mtime`.
    ///
    /// # Notes
    ///
    /// When writing `mtimecmp` as two 32-bit halves, software should write the
    /// high half first and the low half last to avoid a window where `mtimecmp`
    /// transiently equals `mtime` and triggers a spurious interrupt.
    fn clint_write(&mut self, addr: u32, val: u32) {
        match addr {
            CLINT_MTIME => {
                self.mtime = (self.mtime & 0xFFFF_FFFF_0000_0000) | val as u64;
            }

            CLINT_MTIME_PLUS_4 => {
                self.mtime = (self.mtime & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32);
            }

            CLINT_MTIMECMP => {
                self.mtimecmp = (self.mtimecmp & 0xFFFF_FFFF_0000_0000) | val as u64;
                if self.mtimecmp > self.mtime {
                    self.timer_pending = false;
                }
            }
            CLINT_MTIMECMP_PLUS_4 => {
                self.mtimecmp = (self.mtimecmp & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32);
                if self.mtimecmp > self.mtime {
                    self.timer_pending = false;
                }
            }

            _ => {}
        }
    }

    /// Reads a 32-bit value from a UART memory-mapped register.
    ///
    /// - [`UART_LSR`] always returns [`TX_READY`], indicating the transmitter
    ///   is idle.
    /// - [`UART_RX`] always returns `0`; serial input is not emulated.
    /// - All other UART addresses return `0`.
    fn uart_read(&self, addr: u32) -> u32 {
        match addr {
            UART_LSR => TX_READY as u32,
            UART_RX => 0,
            _ => 0,
        }
    }

    /// Writes a 32-bit value to a UART memory-mapped register.
    ///
    /// Only writes to [`UART_TX`] have an effect: the low byte of `val` is
    /// written to stdout. Null bytes are silently discarded. The write is
    /// immediately flushed so that output appears in real time.
    fn uart_write(&mut self, addr: u32, val: u32) {
        if addr == UART_TX {
            let ch = (val & 0xFF) as u8;
            if ch != 0 {
                let stdout = std::io::stdout();
                let mut out = stdout.lock();
                std::io::Write::write_all(&mut out, &[ch]).ok();
                std::io::Write::flush(&mut out).ok();
            }
        }
    }

    /// Reads one byte from the guest address space.
    ///
    /// Dispatch order: UART → CLINT → RAM → zero (unmapped).
    ///
    /// Reads from CLINT and UART addresses return only the low byte of the
    /// corresponding 32-bit register value.
    pub fn read_u8(&self, addr: u32) -> u8 {
        if is_uart(addr) {
            return self.uart_read(addr) as u8;
        }

        if is_clint(addr) {
            return self.clint_read(addr) as u8;
        }

        if is_ram(addr, self.memory.len()) {
            return self.memory[to_ram_index(addr)];
        }

        0
    }

    /// Writes one byte to the guest address space.
    ///
    /// Dispatch order: UART → CLINT → RAM → silent discard (unmapped).
    pub fn write_u8(&mut self, addr: u32, val: u8) {
        if is_uart(addr) {
            self.uart_write(addr, val as u32);
            return;
        }

        if is_clint(addr) {
            self.clint_write(addr, val as u32);
            return;
        }

        if is_ram(addr, self.memory.len()) {
            self.memory[to_ram_index(addr)] = val;
        }
    }

    /// Reads a little-endian 16-bit value from the guest address space.
    ///
    /// Both bytes must map to the same region (UART, CLINT, or RAM); a
    /// cross-region halfword read is not supported and returns `0`.
    pub fn read_u16(&self, addr: u32) -> u16 {
        if is_uart(addr) {
            return self.uart_read(addr) as u16;
        }

        if is_clint(addr) {
            return self.clint_read(addr) as u16;
        }

        if is_ram(addr, self.memory.len()) && is_ram(addr.wrapping_add(1), self.memory.len()) {
            let a = to_ram_index(addr);

            return u16::from_le_bytes([self.memory[a], self.memory[a + 1]]);
        }

        0
    }

    /// Writes a little-endian 16-bit value to the guest address space.
    ///
    /// Both bytes must map to the same region; cross-region writes are silently
    /// discarded.
    pub fn write_u16(&mut self, addr: u32, val: u16) {
        if is_uart(addr) {
            self.uart_write(addr, val as u32);
            return;
        }

        if is_clint(addr) {
            self.clint_write(addr, val as u32);
            return;
        }

        if is_ram(addr, self.memory.len()) && is_ram(addr.wrapping_add(1), self.memory.len()) {
            let a = to_ram_index(addr);
            let b = val.to_le_bytes();

            self.memory[a] = b[0];
            self.memory[a + 1] = b[1];
        }
    }

    /// Reads a little-endian 32-bit value from the guest address space.
    ///
    /// All four bytes must fall within the same region; an out-of-bounds or
    /// cross-region read returns `0`. This method is also used by [`CPU::fetch`]
    /// to retrieve instruction words.
    pub fn read_u32(&self, addr: u32) -> u32 {
        if is_uart(addr) {
            return self.uart_read(addr);
        }

        if is_clint(addr) {
            return self.clint_read(addr);
        }

        if is_ram(addr, self.memory.len()) && is_ram(addr.wrapping_add(3), self.memory.len()) {
            let a = to_ram_index(addr);

            return u32::from_le_bytes([
                self.memory[a],
                self.memory[a + 1],
                self.memory[a + 2],
                self.memory[a + 3],
            ]);
        }

        0
    }

    /// Writes a little-endian 32-bit value to the guest address space.
    ///
    /// All four bytes must fall within the same region; out-of-bounds or
    /// cross-region writes are silently discarded.
    pub fn write_u32(&mut self, addr: u32, val: u32) {
        if is_uart(addr) {
            self.uart_write(addr, val);
            return;
        }

        if is_clint(addr) {
            self.clint_write(addr, val);
            return;
        }

        if is_ram(addr, self.memory.len()) && is_ram(addr.wrapping_add(3), self.memory.len()) {
            let a = to_ram_index(addr);

            self.memory[a..a + 4].copy_from_slice(&val.to_le_bytes());
        }
    }

    /// Reads the value of a CSR by its 12-bit address.
    ///
    /// No side effects are produced; this is a plain array lookup.
    pub fn csr_read(&self, addr: u16) -> u32 {
        self.csr[addr as usize]
    }

    /// Writes a value to a CSR by its 12-bit address.
    ///
    /// No masking or field validation is performed; the raw 32-bit value is
    /// stored directly. Architecturally reserved bits are silently accepted.
    pub fn csr_write(&mut self, addr: u16, val: u32) {
        self.csr[addr as usize] = val;
    }

    /// Delivers a synchronous exception or asynchronous interrupt to the CPU.
    ///
    /// This method performs the full RISC-V machine-mode trap sequence:
    ///
    /// 1. Saves `pc` to `mepc`.
    /// 2. Writes `cause` to `mcause`.
    /// 3. In `mstatus`: copies `MIE` → `MPIE`, clears `MIE`, and sets `MPP`
    ///    to `0b11` (Machine mode).
    /// 4. Sets `pc` to the handler address `mtvec & !0b11`.
    ///
    /// If `mtvec` is zero the trap is silently ignored and the CPU state is
    /// not modified (other than the `mcause` and `mepc` writes).
    ///
    /// # Parameters
    ///
    /// - `cause` – The value written to `mcause`. For interrupts, bit 31 must
    ///   be set (e.g., `0x8000_0007` for a machine timer interrupt). For
    ///   synchronous exceptions, bit 31 is clear (e.g., `11` for an
    ///   environment call).
    pub fn trap(&mut self, cause: u32) {
        let mtvec = self.csr_read(MTVEC as u16);
        let handler = mtvec & !0b11;

        if handler == 0 {
            println!("mtvec = 0, trap ignored");
            return;
        }

        self.csr_write(MEPC as u16, self.pc);
        self.csr_write(MCAUSE as u16, cause);

        let mut mstatus = self.csr_read(MSTATUS as u16);
        let mie_bit = (mstatus & MSTATUS_MIE) != 0;

        mstatus &= !MSTATUS_MIE;
        if mie_bit {
            mstatus |= MSTATUS_MPIE;
        } else {
            mstatus &= !MSTATUS_MPIE;
        }
        mstatus = (mstatus & !MSTATUS_MPP_MASK) | (0b11 << 11);

        self.csr_write(MSTATUS as u16, mstatus);
        self.pc = handler;
    }

    /// Advances the timer by one tick and delivers a pending timer interrupt
    /// if interrupts are enabled.
    ///
    /// Each call:
    ///
    /// 1. Increments `mtime` by one (wrapping on overflow).
    /// 2. Sets [`CPU::timer_pending`] if `mtimecmp != 0` and
    ///    `mtime >= mtimecmp`.
    /// 3. If `timer_pending` is set and both `mstatus.MIE` and `mie.MTIE` are
    ///    set, clears `timer_pending` and calls [`CPU::trap`] with cause
    ///    `0x8000_0007`.
    ///
    /// `tick` is called at the start of every [`CPU::step`], so `mtime`
    /// effectively counts executed instructions.
    pub fn tick(&mut self) {
        self.mtime = self.mtime.wrapping_add(1);

        if self.mtimecmp != 0 && self.mtime >= self.mtimecmp {
            self.timer_pending = true;
        }

        if self.timer_pending {
            let mstatus = self.csr_read(MSTATUS as u16);
            let mie = self.csr_read(MIE_CSR as u16);
            if (mstatus & MSTATUS_MIE) != 0 && (mie & MIE_MTIE) != 0 {
                self.timer_pending = false;
                self.trap(0x8000_0007);
            }
        }
    }

    /// Loads all `PT_LOAD` segments from `elf` into the emulated DRAM and sets
    /// `pc` to the ELF entry point.
    ///
    /// Segments whose virtual address falls below [`RAM_BASE`] are skipped with
    /// a diagnostic message. Segments that extend beyond the end of the backing
    /// memory buffer return an error immediately without partial modification.
    ///
    /// BSS-style zero padding (`memsz > filesz`) is applied after copying file
    /// data.
    ///
    /// # Errors
    ///
    /// Returns [`RiscVError::InvalidProgramHeader`] if any `PT_LOAD` segment
    /// would overflow the backing memory buffer.
    pub fn load_elf(&mut self, elf: &ElfImage) -> Result<(), RiscVError> {
        self.pc = elf.entry;
        for seg in &elf.segments {
            println!(
                "segment: vaddr=0x{:08X} filesz={} memsz={}",
                seg.vaddr,
                seg.data.len(),
                seg.mem_size
            );

            if (seg.vaddr as usize) < RAM_BASE {
                println!("  -> ignored (out of RAM)");
                continue;
            }

            let start = to_ram_index(seg.vaddr);
            let end = start + seg.data.len();

            if end > self.memory.len() {
                println!(
                    "  -> out of bounds! start={} end={} mem={}",
                    start,
                    end,
                    self.memory.len()
                );
                return Err(RiscVError::InvalidProgramHeader);
            }

            self.memory[start..end].copy_from_slice(&seg.data);
            for i in end..(start + seg.mem_size as usize) {
                self.memory[i] = 0;
            }
        }
        Ok(())
    }

    /// Fetches the 32-bit instruction word at the current `pc`.
    ///
    /// Delegates to [`CPU::read_u32`], which handles peripheral and RAM
    /// dispatch. The instruction is returned in its raw encoded form for
    /// subsequent decoding by [`CPU::decode`].
    ///
    /// # Panics
    ///
    /// Panics if `pc` is `0` or `0xffff_ffff`, which indicates a control-flow
    /// error in the emulated program.
    pub fn fetch(&self) -> u32 {
        let pc = self.pc;
        if pc == 0 || pc == 0xffff_ffff {
            panic!("invalid pc");
        }

        self.read_u32(pc)
    }

    /// Decodes a raw 32-bit instruction word into a typed [`Instruction`].
    ///
    /// All standard RV32I and RV32M encodings are supported. Immediate values
    /// are sign-extended to 32 bits where required by the ISA specification.
    ///
    /// # Panics
    ///
    /// Panics with `"illegal instruction 0x{:08x}"` if the `opcode`, `funct3`,
    /// or `funct7` fields do not match any known encoding. This terminates
    /// emulation immediately rather than delivering an illegal-instruction trap.
    pub fn decode(&mut self, instruction: u32) -> Instruction {
        let opcode = instruction & 0x7f;
        let rd = ((instruction >> 7) & 0x1f) as u8;
        let funct3 = (instruction >> 12) & 0x7;
        let rs1 = ((instruction >> 15) & 0x1f) as u8;
        let rs2 = ((instruction >> 20) & 0x1f) as u8;
        let funct7 = (instruction >> 25) & 0x7f;

        // ================= immediates =================

        let sign_extend =
            |value: u32, bits: u32| -> i32 { ((value << (32 - bits)) as i32) >> (32 - bits) };

        let imm_i = sign_extend(instruction >> 20, 12);

        let imm_s = sign_extend(((instruction >> 25) << 5) | ((instruction >> 7) & 0x1f), 12);

        let imm_b = sign_extend(
            ((instruction >> 31) << 12)
                | (((instruction >> 7) & 0x1) << 11)
                | (((instruction >> 25) & 0x3f) << 5)
                | (((instruction >> 8) & 0xf) << 1),
            13,
        );

        let imm_u = (instruction & 0xffff_f000) as i32;

        let imm_j = sign_extend(
            ((instruction >> 31) << 20)
                | (((instruction >> 12) & 0xff) << 12)
                | (((instruction >> 20) & 0x1) << 11)
                | (((instruction >> 21) & 0x3ff) << 1),
            21,
        );

        let shamt = ((instruction >> 20) & 0x1f) as u8;

        match opcode {
            // ================= R =================
            0b0110011 => match (funct3, funct7) {
                (0x0, 0x00) => Instruction::Add { rd, rs1, rs2 },
                (0x0, 0x20) => Instruction::Sub { rd, rs1, rs2 },
                (0x1, 0x00) => Instruction::Sll { rd, rs1, rs2 },
                (0x2, 0x00) => Instruction::Slt { rd, rs1, rs2 },
                (0x3, 0x00) => Instruction::Sltu { rd, rs1, rs2 },
                (0x4, 0x00) => Instruction::Xor { rd, rs1, rs2 },
                (0x5, 0x00) => Instruction::Srl { rd, rs1, rs2 },
                (0x5, 0x20) => Instruction::Sra { rd, rs1, rs2 },
                (0x6, 0x00) => Instruction::Or { rd, rs1, rs2 },
                (0x7, 0x00) => Instruction::And { rd, rs1, rs2 },

                // RV32M
                (0x0, 0x01) => Instruction::Mul { rd, rs1, rs2 },
                (0x1, 0x01) => Instruction::Mulh { rd, rs1, rs2 },
                (0x2, 0x01) => Instruction::Mulhsu { rd, rs1, rs2 },
                (0x3, 0x01) => Instruction::Mulhu { rd, rs1, rs2 },
                (0x4, 0x01) => Instruction::Div { rd, rs1, rs2 },
                (0x5, 0x01) => Instruction::Divu { rd, rs1, rs2 },
                (0x6, 0x01) => Instruction::Rem { rd, rs1, rs2 },
                (0x7, 0x01) => Instruction::Remu { rd, rs1, rs2 },

                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= I =================
            0b0010011 => match funct3 {
                0x0 => Instruction::Addi {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x2 => Instruction::Slti {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x3 => Instruction::Sltiu {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x4 => Instruction::Xori {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x6 => Instruction::Ori {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x7 => Instruction::Andi {
                    rd,
                    rs1,
                    imm: imm_i,
                },

                0x1 if funct7 == 0x00 => Instruction::Slli { rd, rs1, shamt },
                0x1 => panic!("illegal instruction 0x{:08x}", instruction),

                0x5 => match funct7 {
                    0x00 => Instruction::Srli { rd, rs1, shamt },
                    0x20 => Instruction::Srai { rd, rs1, shamt },
                    _ => panic!("illegal instruction 0x{:08x}", instruction),
                },

                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= LOAD =================
            0b0000011 => match funct3 {
                0x0 => Instruction::Lb {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x1 => Instruction::Lh {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x2 => Instruction::Lw {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x4 => Instruction::Lbu {
                    rd,
                    rs1,
                    imm: imm_i,
                },
                0x5 => Instruction::Lhu {
                    rd,
                    rs1,
                    imm: imm_i,
                },

                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= STORE =================
            0b0100011 => match funct3 {
                0x0 => Instruction::Sb {
                    rs1,
                    rs2,
                    imm: imm_s,
                },
                0x1 => Instruction::Sh {
                    rs1,
                    rs2,
                    imm: imm_s,
                },
                0x2 => Instruction::Sw {
                    rs1,
                    rs2,
                    imm: imm_s,
                },

                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= BRANCH =================
            0b1100011 => match funct3 {
                0x0 => Instruction::Beq {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x1 => Instruction::Bne {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x4 => Instruction::Blt {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x5 => Instruction::Bge {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x6 => Instruction::Bltu {
                    rs1,
                    rs2,
                    imm: imm_b,
                },
                0x7 => Instruction::Bgeu {
                    rs1,
                    rs2,
                    imm: imm_b,
                },

                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= U =================
            0b0110111 => Instruction::Lui { rd, imm: imm_u },
            0b0010111 => Instruction::Auipc { rd, imm: imm_u },

            // ================= J =================
            0b1101111 => Instruction::Jal { rd, imm: imm_j },

            0b1100111 if funct3 == 0x0 => Instruction::Jalr {
                rd,
                rs1,
                imm: imm_i,
            },

            0b1100111 => panic!("illegal instruction 0x{:08x}", instruction),

            // ================= FENCE =================
            0b0001111 => match funct3 {
                0x0 => Instruction::Fence,
                0x1 => Instruction::FenceI,
                _ => panic!("illegal instruction 0x{:08x}", instruction),
            },

            // ================= SYSTEM / CSR =================
            0b1110011 => {
                let csr = ((instruction >> 20) & 0xfff) as u16;
                let zimm = ((instruction >> 15) & 0x1f) as u8;

                match funct3 {
                    0x0 => match csr {
                        0x000 => Instruction::Ecall,
                        0x001 => Instruction::Ebreak,
                        0x302 => Instruction::Mret,
                        0x105 => Instruction::Wfi,

                        _ => panic!("illegal instruction 0x{:08x}", instruction),
                    },

                    0x1 => Instruction::Csrrw { rd, rs1, csr },
                    0x2 => Instruction::Csrrs { rd, rs1, csr },
                    0x3 => Instruction::Csrrc { rd, rs1, csr },

                    0x5 => Instruction::Csrrwi { rd, zimm, csr },
                    0x6 => Instruction::Csrrsi { rd, zimm, csr },
                    0x7 => Instruction::Csrrci { rd, zimm, csr },

                    _ => panic!("illegal instruction 0x{:08x}", instruction),
                }
            }

            _ => panic!("illegal instruction 0x{:08x}", instruction),
        }
    }

    /// Executes a decoded instruction and returns a [`StepResult`] describing
    /// how the program counter should be updated.
    ///
    /// This method implements the full RV32I and RV32M execute stage. Side
    /// effects include:
    ///
    /// - Register file writes (suppressed for `rd = 0`).
    /// - Memory reads and writes through the peripheral dispatch layer.
    /// - CSR reads and writes.
    /// - Trap delivery via [`CPU::trap`] (for `ecall 0` and timer-related
    ///   paths).
    /// - Halting the emulator (`running = false`) for `ebreak` and `ecall 93`.
    ///
    /// # Returns
    ///
    /// - [`StepResult::Next`] – the caller should advance `pc` by 4.
    /// - [`StepResult::Jump`] – the caller should set `pc` to `addr`.
    /// - [`StepResult::Halt`] – the caller should stop the execution loop.
    ///
    /// # Panics
    ///
    /// Panics with `"unsupported syscall {}"` if `ecall` is executed with an
    /// `a7` value other than `0` or `93`.
    ///
    /// # Notes on specific instructions
    ///
    /// - **`wfi`**: Instead of spinning, this fast-forwards `mtime` to
    ///   `mtimecmp - 1` when a future deadline is pending, allowing the timer
    ///   interrupt to fire on the very next tick.
    /// - **`mret`**: Restores `pc` from `mepc`, restores `MIE` from `MPIE`,
    ///   sets `MPIE` to `1`, and clears `MPP`.
    /// - **Division by zero**: Follows the RISC-V specification — signed and
    ///   unsigned division/remainder by zero return defined results rather than
    ///   trapping.
    pub fn execute(&mut self, instr: Instruction) -> StepResult {
        #[inline(always)]
        fn idx(r: u8) -> usize {
            r as usize
        }

        #[inline(always)]
        fn wr(cpu: &mut CPU, rd: u8, val: u32) {
            if rd != 0 {
                cpu.regs[rd as usize] = val;
            }
        }

        match instr {
            // =====================================================
            // RV32I - R TYPE
            // =====================================================
            Instruction::Add { rd, rs1, rs2 } => {
                wr(
                    self,
                    rd,
                    self.regs[idx(rs1)].wrapping_add(self.regs[idx(rs2)]),
                );
            }

            Instruction::Sub { rd, rs1, rs2 } => {
                wr(
                    self,
                    rd,
                    self.regs[idx(rs1)].wrapping_sub(self.regs[idx(rs2)]),
                );
            }

            Instruction::Sll { rd, rs1, rs2 } => {
                let shamt = self.regs[idx(rs2)] & 0x1f;
                wr(self, rd, self.regs[idx(rs1)] << shamt);
            }

            Instruction::Slt { rd, rs1, rs2 } => {
                wr(
                    self,
                    rd,
                    ((self.regs[idx(rs1)] as i32) < (self.regs[idx(rs2)] as i32)) as u32,
                );
            }

            Instruction::Sltu { rd, rs1, rs2 } => {
                wr(self, rd, (self.regs[idx(rs1)] < self.regs[idx(rs2)]) as u32);
            }

            Instruction::Xor { rd, rs1, rs2 } => {
                wr(self, rd, self.regs[idx(rs1)] ^ self.regs[idx(rs2)]);
            }

            Instruction::Srl { rd, rs1, rs2 } => {
                let shamt = self.regs[idx(rs2)] & 0x1f;
                wr(self, rd, self.regs[idx(rs1)] >> shamt);
            }

            Instruction::Sra { rd, rs1, rs2 } => {
                let shamt = self.regs[idx(rs2)] & 0x1f;
                wr(self, rd, ((self.regs[idx(rs1)] as i32) >> shamt) as u32);
            }

            Instruction::Or { rd, rs1, rs2 } => {
                wr(self, rd, self.regs[idx(rs1)] | self.regs[idx(rs2)]);
            }

            Instruction::And { rd, rs1, rs2 } => {
                wr(self, rd, self.regs[idx(rs1)] & self.regs[idx(rs2)]);
            }

            // =====================================================
            // RV32M
            // =====================================================
            Instruction::Mul { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as i32 as i64;
                let b = self.regs[idx(rs2)] as i32 as i64;
                wr(self, rd, a.wrapping_mul(b) as u32);
            }

            Instruction::Mulh { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as i32 as i64;
                let b = self.regs[idx(rs2)] as i32 as i64;
                wr(self, rd, ((a.wrapping_mul(b) >> 32) as u64) as u32);
            }

            Instruction::Mulhsu { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as i32 as i64;
                let b = self.regs[idx(rs2)] as u64 as i64;
                wr(self, rd, ((a.wrapping_mul(b) >> 32) as u64) as u32);
            }

            Instruction::Mulhu { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as u64;
                let b = self.regs[idx(rs2)] as u64;
                wr(self, rd, (a.wrapping_mul(b) >> 32) as u32);
            }

            Instruction::Div { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as i32;
                let b = self.regs[idx(rs2)] as i32;

                let v = if b == 0 {
                    u32::MAX
                } else if a == i32::MIN && b == -1 {
                    a as u32
                } else {
                    (a / b) as u32
                };

                wr(self, rd, v);
            }

            Instruction::Divu { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)];
                let b = self.regs[idx(rs2)];
                wr(self, rd, a.checked_div(b).unwrap_or(u32::MAX));
            }

            Instruction::Rem { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)] as i32;
                let b = self.regs[idx(rs2)] as i32;

                let v = if b == 0 {
                    a as u32
                } else if a == i32::MIN && b == -1 {
                    0
                } else {
                    (a % b) as u32
                };

                wr(self, rd, v);
            }

            Instruction::Remu { rd, rs1, rs2 } => {
                let a = self.regs[idx(rs1)];
                let b = self.regs[idx(rs2)];
                wr(self, rd, if b == 0 { a } else { a % b });
            }

            // =====================================================
            // I TYPE
            // =====================================================
            Instruction::Addi { rd, rs1, imm } => {
                wr(self, rd, self.regs[idx(rs1)].wrapping_add(imm as u32));
            }

            Instruction::Slti { rd, rs1, imm } => {
                wr(self, rd, ((self.regs[idx(rs1)] as i32) < imm) as u32);
            }

            Instruction::Sltiu { rd, rs1, imm } => {
                wr(self, rd, (self.regs[idx(rs1)] < imm as u32) as u32);
            }

            Instruction::Xori { rd, rs1, imm } => {
                wr(self, rd, self.regs[idx(rs1)] ^ (imm as u32));
            }

            Instruction::Ori { rd, rs1, imm } => {
                wr(self, rd, self.regs[idx(rs1)] | (imm as u32));
            }

            Instruction::Andi { rd, rs1, imm } => {
                wr(self, rd, self.regs[idx(rs1)] & (imm as u32));
            }

            Instruction::Slli { rd, rs1, shamt } => {
                wr(self, rd, self.regs[idx(rs1)] << (shamt & 0x1f));
            }

            Instruction::Srli { rd, rs1, shamt } => {
                wr(self, rd, self.regs[idx(rs1)] >> (shamt & 0x1f));
            }

            Instruction::Srai { rd, rs1, shamt } => {
                wr(
                    self,
                    rd,
                    ((self.regs[idx(rs1)] as i32) >> (shamt & 0x1f)) as u32,
                );
            }

            // =====================================================
            // LOAD
            // =====================================================
            Instruction::Lb { rd, rs1, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                wr(self, rd, self.read_u8(addr) as i8 as i32 as u32);
            }

            Instruction::Lh { rd, rs1, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                wr(self, rd, self.read_u16(addr) as i16 as i32 as u32);
            }

            Instruction::Lw { rd, rs1, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                wr(self, rd, self.read_u32(addr));
            }

            Instruction::Lbu { rd, rs1, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                wr(self, rd, self.read_u8(addr) as u32);
            }

            Instruction::Lhu { rd, rs1, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                wr(self, rd, self.read_u16(addr) as u32);
            }

            // =====================================================
            // STORE
            // =====================================================
            Instruction::Sb { rs1, rs2, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                self.write_u8(addr, self.regs[idx(rs2)] as u8);
            }

            Instruction::Sh { rs1, rs2, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                self.write_u16(addr, self.regs[idx(rs2)] as u16);
            }

            Instruction::Sw { rs1, rs2, imm } => {
                let addr = self.regs[idx(rs1)].wrapping_add(imm as u32);
                self.write_u32(addr, self.regs[idx(rs2)]);
            }

            // =====================================================
            // BRANCH
            // =====================================================
            Instruction::Beq { rs1, rs2, imm } => {
                if self.regs[idx(rs1)] == self.regs[idx(rs2)] {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            Instruction::Bne { rs1, rs2, imm } => {
                if self.regs[idx(rs1)] != self.regs[idx(rs2)] {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            Instruction::Blt { rs1, rs2, imm } => {
                if (self.regs[idx(rs1)] as i32) < (self.regs[idx(rs2)] as i32) {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            Instruction::Bge { rs1, rs2, imm } => {
                if (self.regs[idx(rs1)] as i32) >= (self.regs[idx(rs2)] as i32) {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            Instruction::Bltu { rs1, rs2, imm } => {
                if self.regs[idx(rs1)] < self.regs[idx(rs2)] {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            Instruction::Bgeu { rs1, rs2, imm } => {
                if self.regs[idx(rs1)] >= self.regs[idx(rs2)] {
                    return StepResult::Jump((self.pc as i32).wrapping_add(imm) as u32);
                }
            }

            // =====================================================
            // U TYPE
            // =====================================================
            Instruction::Lui { rd, imm } => wr(self, rd, imm as u32),

            Instruction::Auipc { rd, imm } => {
                wr(self, rd, self.pc.wrapping_add(imm as u32));
            }

            // =====================================================
            // JUMPS
            // =====================================================
            Instruction::Jal { rd, imm } => {
                wr(self, rd, self.pc.wrapping_add(4));
                return StepResult::Jump(self.pc.wrapping_add(imm as u32));
            }

            Instruction::Jalr { rd, rs1, imm } => {
                wr(self, rd, self.pc.wrapping_add(4));
                return StepResult::Jump(self.regs[idx(rs1)].wrapping_add(imm as u32) & !1);
            }

            // =====================================================
            // FENCE
            // =====================================================
            Instruction::Fence => {}
            Instruction::FenceI => {}

            // =====================================================
            // SYSTEM
            // =====================================================
            Instruction::Ecall => {
                let syscall = self.regs[17];

                match syscall {
                    0 => {
                        self.trap(11);
                        return StepResult::Jump(self.pc);
                    }

                    93 => {
                        self.exit_code = self.regs[10] as i32;
                        return StepResult::Halt;
                    }

                    _ => panic!("unsupported syscall {}", syscall),
                }
            }

            Instruction::Ebreak => {
                return StepResult::Halt;
            }

            Instruction::Mret => {
                let mepc = self.csr_read(MEPC as u16);
                let mut mstatus = self.csr_read(MSTATUS as u16);

                let mpie = (mstatus & MSTATUS_MPIE) != 0;

                if mpie {
                    mstatus |= MSTATUS_MIE;
                } else {
                    mstatus &= !MSTATUS_MIE;
                }

                mstatus |= MSTATUS_MPIE;
                mstatus &= !MSTATUS_MPP_MASK;

                self.csr_write(MSTATUS as u16, mstatus);
                self.csr_write(MCAUSE as u16, 0);

                return StepResult::Jump(mepc);
            }

            Instruction::Wfi => {
                if self.mtimecmp > self.mtime {
                    self.mtime = self.mtimecmp.saturating_sub(1);
                }

                return StepResult::Next;
            }

            // =====================================================
            // CSR
            // =====================================================
            Instruction::Csrrw { rd, rs1, csr } => {
                let old = self.csr_read(csr);
                self.csr_write(csr, self.regs[idx(rs1)]);
                wr(self, rd, old);
            }

            Instruction::Csrrs { rd, rs1, csr } => {
                let old = self.csr_read(csr);
                let val = self.regs[idx(rs1)];

                if val != 0 {
                    self.csr_write(csr, old | val);
                }

                wr(self, rd, old);
            }

            Instruction::Csrrc { rd, rs1, csr } => {
                let old = self.csr_read(csr);
                let val = self.regs[idx(rs1)];

                if val != 0 {
                    self.csr_write(csr, old & !val);
                }

                wr(self, rd, old);
            }

            Instruction::Csrrwi { rd, zimm, csr } => {
                let old = self.csr_read(csr);
                self.csr_write(csr, zimm as u32);
                wr(self, rd, old);
            }

            Instruction::Csrrsi { rd, zimm, csr } => {
                let old = self.csr_read(csr);

                if zimm != 0 {
                    self.csr_write(csr, old | zimm as u32);
                }

                wr(self, rd, old);
            }

            Instruction::Csrrci { rd, zimm, csr } => {
                let old = self.csr_read(csr);

                if zimm != 0 {
                    self.csr_write(csr, old & !(zimm as u32));
                }

                wr(self, rd, old);
            }
        }

        StepResult::Next
    }

    /// Executes one complete instruction cycle: tick → interrupt check →
    /// fetch → decode → execute → PC update.
    ///
    /// The sequence is:
    ///
    /// 1. Call [`CPU::tick`] to advance `mtime` and set `timer_pending` if the
    ///    deadline has passed.
    /// 2. If `timer_pending` is set and interrupts are enabled (`mstatus.MIE`
    ///    and `mie.MTIE`), deliver the interrupt via [`CPU::trap`] and return
    ///    without executing the current instruction.
    /// 3. Fetch the instruction word at `pc` via [`CPU::fetch`].
    /// 4. Decode it via [`CPU::decode`].
    /// 5. Execute it via [`CPU::execute`] and update `pc` based on the returned
    ///    [`StepResult`].
    /// 6. Force `regs[0]` back to `0`.
    ///
    /// This method drives the outer `while cpu.running` loop in `main`.
    pub fn step(&mut self) {
        self.tick();

        if self.timer_pending {
            let mstatus = self.csr_read(MSTATUS as u16);
            let mie = self.csr_read(MIE_CSR as u16);
            if (mstatus & MSTATUS_MIE) != 0 && (mie & MIE_MTIE) != 0 {
                self.timer_pending = false;
                self.trap(0x8000_0007);
                self.regs[0] = 0;
                return;
            }
        }

        let raw = self.fetch();
        let inst = self.decode(raw);

        match self.execute(inst) {
            StepResult::Next => self.pc = self.pc.wrapping_add(4),
            StepResult::Jump(addr) => self.pc = addr,
            StepResult::Halt => self.running = false,
        }

        self.regs[0] = 0;
    }
}

// ── StepResult ────────────────────────────────────────────────────────────────

/// Outcome of executing a single instruction, used by [`CPU::step`] to update
/// the program counter.
pub enum StepResult {
    /// Advance `pc` by 4 (sequential execution).
    Next,
    /// Set `pc` to the contained absolute address (branches, jumps, traps).
    Jump(u32),
    /// Stop the execution loop (`ebreak` or `ecall 93`).
    Halt,
}

// ── Instruction ───────────────────────────────────────────────────────────────

/// A decoded RV32IM instruction with all fields extracted and sign-extended.
///
/// Each variant carries only the fields relevant to that instruction format.
/// Immediate values are stored as `i32` with sign extension already applied;
/// shift amounts are stored as `u8` (5-bit unsigned). CSR addresses are stored
/// as `u16` (12-bit unsigned).
///
/// The `Display` implementation formats each variant as canonical RISC-V
/// assembly syntax suitable for disassembly output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instruction {
    // ── R-type ────────────────────────────────────────────────────────────────
    /// `ADD rd, rs1, rs2` – wrapping integer addition.
    Add { rd: u8, rs1: u8, rs2: u8 },
    /// `SUB rd, rs1, rs2` – wrapping integer subtraction.
    Sub { rd: u8, rs1: u8, rs2: u8 },
    /// `SLL rd, rs1, rs2` – logical left shift; shift amount from `rs2[4:0]`.
    Sll { rd: u8, rs1: u8, rs2: u8 },
    /// `SLT rd, rs1, rs2` – signed less-than comparison; result is 0 or 1.
    Slt { rd: u8, rs1: u8, rs2: u8 },
    /// `SLTU rd, rs1, rs2` – unsigned less-than comparison; result is 0 or 1.
    Sltu { rd: u8, rs1: u8, rs2: u8 },
    /// `XOR rd, rs1, rs2` – bitwise exclusive OR.
    Xor { rd: u8, rs1: u8, rs2: u8 },
    /// `SRL rd, rs1, rs2` – logical right shift; shift amount from `rs2[4:0]`.
    Srl { rd: u8, rs1: u8, rs2: u8 },
    /// `SRA rd, rs1, rs2` – arithmetic right shift (sign-extending); shift
    /// amount from `rs2[4:0]`.
    Sra { rd: u8, rs1: u8, rs2: u8 },
    /// `OR rd, rs1, rs2` – bitwise OR.
    Or { rd: u8, rs1: u8, rs2: u8 },
    /// `AND rd, rs1, rs2` – bitwise AND.
    And { rd: u8, rs1: u8, rs2: u8 },

    // ── I-type ALU ────────────────────────────────────────────────────────────
    /// `ADDI rd, rs1, imm` – wrapping addition with a 12-bit sign-extended
    /// immediate. The canonical `NOP` is encoded as `ADDI x0, x0, 0`.
    Addi { rd: u8, rs1: u8, imm: i32 },
    /// `SLTI rd, rs1, imm` – signed less-than with immediate; result is 0 or 1.
    Slti { rd: u8, rs1: u8, imm: i32 },
    /// `SLTIU rd, rs1, imm` – unsigned less-than with sign-extended immediate;
    /// result is 0 or 1.
    Sltiu { rd: u8, rs1: u8, imm: i32 },
    /// `XORI rd, rs1, imm` – bitwise XOR with sign-extended immediate.
    /// `XORI rd, rs1, -1` is the canonical one-complement idiom.
    Xori { rd: u8, rs1: u8, imm: i32 },
    /// `ORI rd, rs1, imm` – bitwise OR with sign-extended immediate.
    Ori { rd: u8, rs1: u8, imm: i32 },
    /// `ANDI rd, rs1, imm` – bitwise AND with sign-extended immediate.
    Andi { rd: u8, rs1: u8, imm: i32 },
    /// `SLLI rd, rs1, shamt` – logical left shift by a 5-bit immediate amount.
    Slli { rd: u8, rs1: u8, shamt: u8 },
    /// `SRLI rd, rs1, shamt` – logical right shift by a 5-bit immediate amount.
    Srli { rd: u8, rs1: u8, shamt: u8 },
    /// `SRAI rd, rs1, shamt` – arithmetic right shift by a 5-bit immediate
    /// amount; the sign bit is replicated into vacated positions.
    Srai { rd: u8, rs1: u8, shamt: u8 },

    // ── Loads ─────────────────────────────────────────────────────────────────
    /// `LB rd, imm(rs1)` – load byte, sign-extended to 32 bits.
    Lb { rd: u8, rs1: u8, imm: i32 },
    /// `LH rd, imm(rs1)` – load halfword (16-bit), sign-extended to 32 bits.
    Lh { rd: u8, rs1: u8, imm: i32 },
    /// `LW rd, imm(rs1)` – load word (32-bit).
    Lw { rd: u8, rs1: u8, imm: i32 },
    /// `LBU rd, imm(rs1)` – load byte, zero-extended to 32 bits.
    Lbu { rd: u8, rs1: u8, imm: i32 },
    /// `LHU rd, imm(rs1)` – load halfword (16-bit), zero-extended to 32 bits.
    Lhu { rd: u8, rs1: u8, imm: i32 },

    // ── Stores ────────────────────────────────────────────────────────────────
    /// `SB rs2, imm(rs1)` – store the low byte of `rs2`.
    Sb { rs1: u8, rs2: u8, imm: i32 },
    /// `SH rs2, imm(rs1)` – store the low halfword (16 bits) of `rs2`.
    Sh { rs1: u8, rs2: u8, imm: i32 },
    /// `SW rs2, imm(rs1)` – store the full 32-bit word from `rs2`.
    Sw { rs1: u8, rs2: u8, imm: i32 },

    // ── Branches ──────────────────────────────────────────────────────────────
    /// `BEQ rs1, rs2, imm` – branch if `rs1 == rs2`; target is `pc + imm`.
    Beq { rs1: u8, rs2: u8, imm: i32 },
    /// `BNE rs1, rs2, imm` – branch if `rs1 != rs2`.
    Bne { rs1: u8, rs2: u8, imm: i32 },
    /// `BLT rs1, rs2, imm` – branch if `rs1 < rs2` (signed comparison).
    Blt { rs1: u8, rs2: u8, imm: i32 },
    /// `BGE rs1, rs2, imm` – branch if `rs1 >= rs2` (signed comparison).
    Bge { rs1: u8, rs2: u8, imm: i32 },
    /// `BLTU rs1, rs2, imm` – branch if `rs1 < rs2` (unsigned comparison).
    Bltu { rs1: u8, rs2: u8, imm: i32 },
    /// `BGEU rs1, rs2, imm` – branch if `rs1 >= rs2` (unsigned comparison).
    Bgeu { rs1: u8, rs2: u8, imm: i32 },

    // ── U-type ────────────────────────────────────────────────────────────────
    /// `LUI rd, imm` – loads a 20-bit upper immediate into `rd[31:12]`,
    /// zeroing the low 12 bits.
    Lui { rd: u8, imm: i32 },
    /// `AUIPC rd, imm` – adds a 20-bit upper immediate to `pc` and writes the
    /// result to `rd`. Used for PC-relative addressing.
    Auipc { rd: u8, imm: i32 },

    // ── Jumps ─────────────────────────────────────────────────────────────────
    /// `JAL rd, imm` – unconditional jump to `pc + imm`; saves `pc + 4` in
    /// `rd` as the return address. `imm` is a 21-bit sign-extended offset.
    Jal { rd: u8, imm: i32 },
    /// `JALR rd, imm(rs1)` – jump to `(rs1 + imm) & !1`; saves `pc + 4` in
    /// `rd`. The LSB of the target is always cleared per the ISA specification.
    Jalr { rd: u8, rs1: u8, imm: i32 },

    // ── System ────────────────────────────────────────────────────────────────
    /// `ECALL` – environment call. The emulator dispatches on `a7` (x17):
    /// `0` delivers a trap with cause 11; `93` halts with the exit code in
    /// `a0` (x10).
    Ecall,
    /// `EBREAK` – breakpoint. Halts emulation immediately via
    /// [`StepResult::Halt`].
    Ebreak,

    // ── CSR instructions ──────────────────────────────────────────────────────
    /// `CSRRW rd, csr, rs1` – atomically reads `csr` into `rd` and writes
    /// `rs1` to `csr`.
    Csrrw { rd: u8, rs1: u8, csr: u16 },
    /// `CSRRS rd, csr, rs1` – reads `csr` into `rd` and sets bits in `csr`
    /// corresponding to set bits in `rs1`. No write if `rs1 = x0`.
    Csrrs { rd: u8, rs1: u8, csr: u16 },
    /// `CSRRC rd, csr, rs1` – reads `csr` into `rd` and clears bits in `csr`
    /// corresponding to set bits in `rs1`. No write if `rs1 = x0`.
    Csrrc { rd: u8, rs1: u8, csr: u16 },
    /// `CSRRWI rd, csr, zimm` – reads `csr` into `rd` and writes the 5-bit
    /// zero-extended immediate `zimm` to `csr`.
    Csrrwi { rd: u8, zimm: u8, csr: u16 },
    /// `CSRRSI rd, csr, zimm` – reads `csr` into `rd` and sets bits in `csr`
    /// from the 5-bit zero-extended immediate `zimm`. No write if `zimm = 0`.
    Csrrsi { rd: u8, zimm: u8, csr: u16 },
    /// `CSRRCI rd, csr, zimm` – reads `csr` into `rd` and clears bits in `csr`
    /// from the 5-bit zero-extended immediate `zimm`. No write if `zimm = 0`.
    Csrrci { rd: u8, zimm: u8, csr: u16 },

    // ── Privileged ────────────────────────────────────────────────────────────
    /// `MRET` – return from machine-mode trap. Restores `pc` from `mepc` and
    /// `mstatus.MIE` from `mstatus.MPIE`.
    Mret,

    // ── Fence ─────────────────────────────────────────────────────────────────
    /// `FENCE` – memory ordering fence. Treated as a no-op in this
    /// single-cycle, single-hart emulator.
    Fence,
    /// `FENCE.I` – instruction-fetch fence. Treated as a no-op because the
    /// emulator has no instruction cache to invalidate.
    FenceI,

    // ── RV32M ─────────────────────────────────────────────────────────────────
    /// `MUL rd, rs1, rs2` – lower 32 bits of the signed×signed product.
    Mul { rd: u8, rs1: u8, rs2: u8 },
    /// `MULH rd, rs1, rs2` – upper 32 bits of the signed×signed 64-bit product.
    Mulh { rd: u8, rs1: u8, rs2: u8 },
    /// `MULHSU rd, rs1, rs2` – upper 32 bits of the signed×unsigned 64-bit
    /// product (`rs1` is treated as signed, `rs2` as unsigned).
    Mulhsu { rd: u8, rs1: u8, rs2: u8 },
    /// `MULHU rd, rs1, rs2` – upper 32 bits of the unsigned×unsigned 64-bit
    /// product.
    Mulhu { rd: u8, rs1: u8, rs2: u8 },
    /// `DIV rd, rs1, rs2` – signed integer division (truncated toward zero).
    /// Division by zero produces `u32::MAX` (all ones). Overflow (`i32::MIN /
    /// -1`) produces `i32::MIN`.
    Div { rd: u8, rs1: u8, rs2: u8 },
    /// `DIVU rd, rs1, rs2` – unsigned integer division. Division by zero
    /// produces `u32::MAX`.
    Divu { rd: u8, rs1: u8, rs2: u8 },
    /// `REM rd, rs1, rs2` – signed remainder. Division by zero returns the
    /// dividend unchanged. Overflow (`i32::MIN % -1`) returns `0`.
    Rem { rd: u8, rs1: u8, rs2: u8 },
    /// `REMU rd, rs1, rs2` – unsigned remainder. Division by zero returns the
    /// dividend unchanged.
    Remu { rd: u8, rs1: u8, rs2: u8 },

    /// `WFI` – wait for interrupt. Instead of spinning, the emulator
    /// fast-forwards `mtime` to `mtimecmp - 1` so the timer fires on the next
    /// tick, avoiding a busy-wait loop in the host process.
    Wfi,
}

impl Instruction {
    fn fmt_asm(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        use Instruction::*;

        match *self {
            // =========================
            // R-type
            // =========================
            Add { rd, rs1, rs2 } => r(f, "add", rd, rs1, rs2),
            Sub { rd, rs1, rs2 } => r(f, "sub", rd, rs1, rs2),
            Sll { rd, rs1, rs2 } => r(f, "sll", rd, rs1, rs2),
            Slt { rd, rs1, rs2 } => r(f, "slt", rd, rs1, rs2),
            Sltu { rd, rs1, rs2 } => r(f, "sltu", rd, rs1, rs2),
            Xor { rd, rs1, rs2 } => r(f, "xor", rd, rs1, rs2),
            Srl { rd, rs1, rs2 } => r(f, "srl", rd, rs1, rs2),
            Sra { rd, rs1, rs2 } => r(f, "sra", rd, rs1, rs2),
            Or { rd, rs1, rs2 } => r(f, "or", rd, rs1, rs2),
            And { rd, rs1, rs2 } => r(f, "and", rd, rs1, rs2),

            // =========================
            // RV32M
            // =========================
            Mul { rd, rs1, rs2 } => r(f, "mul", rd, rs1, rs2),
            Mulh { rd, rs1, rs2 } => r(f, "mulh", rd, rs1, rs2),
            Mulhsu { rd, rs1, rs2 } => r(f, "mulhsu", rd, rs1, rs2),
            Mulhu { rd, rs1, rs2 } => r(f, "mulhu", rd, rs1, rs2),
            Div { rd, rs1, rs2 } => r(f, "div", rd, rs1, rs2),
            Divu { rd, rs1, rs2 } => r(f, "divu", rd, rs1, rs2),
            Rem { rd, rs1, rs2 } => r(f, "rem", rd, rs1, rs2),
            Remu { rd, rs1, rs2 } => r(f, "remu", rd, rs1, rs2),

            // =========================
            // I-type ALU
            // =========================
            Addi { rd, rs1, imm } => i(f, "addi", rd, rs1, imm),
            Slti { rd, rs1, imm } => i(f, "slti", rd, rs1, imm),
            Sltiu { rd, rs1, imm } => i(f, "sltiu", rd, rs1, imm),
            Xori { rd, rs1, imm } => i(f, "xori", rd, rs1, imm),
            Ori { rd, rs1, imm } => i(f, "ori", rd, rs1, imm),
            Andi { rd, rs1, imm } => i(f, "andi", rd, rs1, imm),

            Slli { rd, rs1, shamt } => ish(f, "slli", rd, rs1, shamt),
            Srli { rd, rs1, shamt } => ish(f, "srli", rd, rs1, shamt),
            Srai { rd, rs1, shamt } => ish(f, "srai", rd, rs1, shamt),

            // =========================
            // Loads
            // =========================
            Lb { rd, rs1, imm } => load(f, "lb", rd, rs1, imm),
            Lh { rd, rs1, imm } => load(f, "lh", rd, rs1, imm),
            Lw { rd, rs1, imm } => load(f, "lw", rd, rs1, imm),
            Lbu { rd, rs1, imm } => load(f, "lbu", rd, rs1, imm),
            Lhu { rd, rs1, imm } => load(f, "lhu", rd, rs1, imm),

            // =========================
            // Stores
            // =========================
            Sb { rs1, rs2, imm } => store(f, "sb", rs1, rs2, imm),
            Sh { rs1, rs2, imm } => store(f, "sh", rs1, rs2, imm),
            Sw { rs1, rs2, imm } => store(f, "sw", rs1, rs2, imm),

            // =========================
            // Branches
            // =========================
            Beq { rs1, rs2, imm } => branch(f, "beq", rs1, rs2, imm),
            Bne { rs1, rs2, imm } => branch(f, "bne", rs1, rs2, imm),
            Blt { rs1, rs2, imm } => branch(f, "blt", rs1, rs2, imm),
            Bge { rs1, rs2, imm } => branch(f, "bge", rs1, rs2, imm),
            Bltu { rs1, rs2, imm } => branch(f, "bltu", rs1, rs2, imm),
            Bgeu { rs1, rs2, imm } => branch(f, "bgeu", rs1, rs2, imm),

            // =========================
            // U-type
            // =========================
            Lui { rd, imm } => u(f, "lui", rd, imm),
            Auipc { rd, imm } => u(f, "auipc", rd, imm),

            // =========================
            // Jumps
            // =========================
            Jal { rd, imm } => u(f, "jal", rd, imm),
            Jalr { rd, rs1, imm } => jalr(f, rd, rs1, imm),

            // =========================
            // System / CSR
            // =========================
            Ecall => f.write_str("ecall"),
            Ebreak => f.write_str("ebreak"),
            Csrrs { rd, rs1, csr } => csr_r(f, "csrrs", rd, rs1, csr),
            Csrrc { rd, rs1, csr } => csr_r(f, "csrrc", rd, rs1, csr),
            Csrrw { rd, rs1, csr } => csr_r(f, "csrrw", rd, rs1, csr),
            Csrrsi { rd, zimm, csr } => csr_i(f, "csrrsi", rd, zimm, csr),
            Csrrci { rd, zimm, csr } => csr_i(f, "csrrci", rd, zimm, csr),
            Csrrwi { rd, zimm, csr } => csr_i(f, "csrrwi", rd, zimm, csr),
            Mret => write!(f, "mret"),
            Wfi => f.write_str("wfi"),

            // =========================
            // Fence
            // =========================
            Fence => f.write_str("fence"),
            FenceI => f.write_str("fence.i"),
        }
    }
}

/// Formats the instruction as canonical RISC-V assembly syntax.
///
/// Register names use the numeric ABI (`x0`–`x31`). Immediates are printed
/// as decimal integers. CSR addresses are printed as lowercase hexadecimal
/// with a `0x` prefix.
impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.fmt_asm(f)
    }
}

// ── Assembly formatting helpers ───────────────────────────────────────────────

/// Formats an R-type instruction: `op xrd, xrs1, xrs2`.
#[inline(always)]
fn r(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, rs2: u8) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, x{rs2}")
}

/// Formats an I-type ALU instruction: `op xrd, xrs1, imm`.
#[inline(always)]
fn i(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, imm: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, {imm}")
}

/// Formats an I-type shift instruction: `op xrd, xrs1, shamt`.
#[inline(always)]
fn ish(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, rs1: u8, shamt: u8) -> fmt::Result {
    write!(f, "{op} x{rd}, x{rs1}, {shamt}")
}

/// Formats a load instruction: `op xrd, off(xbase)`.
#[inline(always)]
fn load(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, base: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, {off}(x{base})")
}

/// Formats a store instruction: `op xsrc, off(xbase)`.
#[inline(always)]
fn store(f: &mut fmt::Formatter<'_>, op: &str, base: u8, src: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{src}, {off}(x{base})")
}

/// Formats a branch instruction: `op xrs1, xrs2, off`.
#[inline(always)]
fn branch(f: &mut fmt::Formatter<'_>, op: &str, rs1: u8, rs2: u8, off: i32) -> fmt::Result {
    write!(f, "{op} x{rs1}, x{rs2}, {off}")
}

/// Formats a U-type instruction: `op xrd, imm`.
#[inline(always)]
fn u(f: &mut fmt::Formatter<'_>, op: &str, rd: u8, imm: i32) -> fmt::Result {
    write!(f, "{op} x{rd}, {imm}")
}

/// Formats a `JALR` instruction: `jalr xrd, imm(xrs1)`.
#[inline(always)]
fn jalr(f: &mut fmt::Formatter<'_>, rd: u8, rs1: u8, imm: i32) -> fmt::Result {
    write!(f, "jalr x{rd}, {imm}(x{rs1})")
}

/// Formats a CSR register-based instruction: `name xrd, xrs1, 0xcsr`.
fn csr_r(f: &mut fmt::Formatter<'_>, name: &str, rd: u8, rs1: u8, csr: u16) -> fmt::Result {
    write!(f, "{} x{}, x{}, 0x{:x}", name, rd, rs1, csr)
}

/// Formats a CSR immediate instruction: `name xrd, zimm, 0xcsr`.
fn csr_i(f: &mut fmt::Formatter<'_>, name: &str, rd: u8, zimm: u8, csr: u16) -> fmt::Result {
    write!(f, "{} x{}, {}, 0x{:x}", name, rd, zimm, csr)
}

// ── ELF loader ────────────────────────────────────────────────────────────────

/// Errors that can occur while parsing or loading an ELF binary.
#[derive(Debug)]
pub enum RiscVError {
    /// The file does not begin with the ELF magic bytes (`\x7FELF`), or is
    /// too short to contain a complete ELF header.
    NotElf,
    /// The ELF class field (`e_ident[4]`) is not `1` (ELF32). 64-bit ELF
    /// binaries are not supported.
    Not32Bit,
    /// The ELF data encoding field (`e_ident[5]`) is not `1` (little-endian).
    /// Big-endian binaries are not supported.
    WrongEndian,
    /// The ELF machine field (`e_machine`) is not [`EM_RISCV`] (`243`). Only
    /// RISC-V ELF binaries can be loaded.
    NotRiscV,
    /// A program header is malformed: offset/size arithmetic overflowed, the
    /// segment extends beyond the ELF file, or a `PT_LOAD` segment would
    /// exceed the emulated DRAM.
    InvalidProgramHeader,
}

/// ELF machine type for RISC-V (`e_machine = 243`).
pub const EM_RISCV: u16 = 243;

/// ELF program-header type for a loadable segment.
///
/// Only segments with `p_type == PT_LOAD` are copied into emulated RAM;
/// all others are silently skipped.
pub const PT_LOAD: u32 = 1;

/// The 52-byte ELF32 file header, read directly from the binary via a raw
/// pointer cast.
///
/// Fields follow the ELF32 specification layout. All multi-byte fields are
/// interpreted as little-endian values after the endianness check in
/// [`read_elf`].
#[repr(C)]
#[derive(Debug)]
struct Elf32Header {
    /// ELF identification bytes: magic, class, data, version, OS/ABI, padding.
    e_ident: [u8; 16],
    /// Object file type (executable, shared object, etc.).
    e_type: u16,
    /// Target ISA; must equal [`EM_RISCV`].
    e_machine: u16,
    /// ELF version; always `1`.
    e_version: u32,
    /// Virtual address of the program entry point, copied to [`CPU::pc`].
    e_entry: u32,
    /// Byte offset of the program header table within the file.
    e_phoff: u32,
    /// Byte offset of the section header table (unused by the loader).
    e_shoff: u32,
    /// Processor-specific flags (RISC-V ABI and ISA extension bits).
    e_flags: u32,
    /// Size of this header in bytes (always 52 for ELF32).
    e_ehsize: u16,
    /// Size of one program header entry in bytes.
    e_phentsize: u16,
    /// Number of entries in the program header table.
    e_phnum: u16,
    /// Size of one section header entry in bytes (unused by the loader).
    e_shentsize: u16,
    /// Number of section header entries (unused by the loader).
    e_shnum: u16,
    /// Section header index of the section name string table (unused).
    e_shstrndx: u16,
}

/// A single ELF32 program header entry describing one memory segment.
///
/// The loader iterates all entries and processes those with
/// `p_type == PT_LOAD`.
#[repr(C)]
#[derive(Debug)]
struct Elf32ProgramHeader {
    /// Segment type; only [`PT_LOAD`] segments are copied into RAM.
    p_type: u32,
    /// Byte offset of the segment data within the ELF file.
    p_offset: u32,
    /// Virtual address at which the segment is mapped in the guest.
    p_vaddr: u32,
    /// Physical address (identical to `p_vaddr` for most RISC-V linker scripts).
    p_paddr: u32,
    /// Number of bytes of file data to copy into RAM.
    p_filesz: u32,
    /// Total size of the segment in memory; bytes beyond `p_filesz` are zeroed
    /// (BSS padding).
    p_memsz: u32,
    /// Segment permission flags (read/write/execute).
    p_flags: u32,
    /// Required alignment of the segment in memory and in the file.
    p_align: u32,
}

/// A loaded ELF segment ready to be copied into emulated RAM.
#[derive(Debug, Clone)]
pub struct ElfSegment {
    /// Guest virtual address at which `data` should be placed.
    ///
    /// Must be at or above [`RAM_BASE`]; segments below that address are
    /// skipped by [`CPU::load_elf`].
    pub vaddr: u32,

    /// Raw bytes read from the ELF file for this segment (`p_filesz` bytes).
    pub data: Vec<u8>,

    /// Total size of the segment in guest memory (`p_memsz`).
    ///
    /// When `mem_size > data.len()`, the gap is zero-filled by
    /// [`CPU::load_elf`] to handle BSS sections.
    pub mem_size: u32,
}

/// A parsed ELF binary reduced to the information needed for loading and
/// execution.
#[derive(Debug, Clone)]
pub struct ElfImage {
    /// Virtual address of the program entry point (`e_entry`).
    ///
    /// Copied to [`CPU::pc`] by both [`read_elf`] (indirectly) and
    /// [`CPU::load_elf`] before execution begins.
    pub entry: u32,

    /// All `PT_LOAD` segments extracted from the program header table,
    /// in the order they appear in the file.
    pub segments: Vec<ElfSegment>,
}

/// Parses a byte slice as an ELF32 little-endian RISC-V binary and returns an
/// [`ElfImage`] containing the entry point and all loadable segments.
///
/// The function performs the following validation steps in order:
///
/// 1. Verifies the file is large enough to hold an [`Elf32Header`].
/// 2. Checks the ELF magic bytes.
/// 3. Checks for ELF32 class (`e_ident[4] == 1`).
/// 4. Checks for little-endian encoding (`e_ident[5] == 1`).
/// 5. Checks that the target machine is RISC-V (`e_machine == 243`).
/// 6. Validates that the program header table fits within the file.
/// 7. For each `PT_LOAD` segment, validates that the file data range is in
///    bounds and copies it into an [`ElfSegment`].
///
/// # Errors
///
/// Returns the first [`RiscVError`] encountered during validation.
///
/// # Safety
///
/// Internally uses `unsafe` pointer casts to reinterpret the raw byte slice
/// as `Elf32Header` and `Elf32ProgramHeader` slices. This is safe because:
/// - The byte-length of the slice is verified before casting.
/// - Both types are `#[repr(C)]` with no padding-sensitive fields.
/// - All field accesses go through the cast references, not raw pointer
///   arithmetic.
pub fn read_elf(data: &[u8]) -> Result<ElfImage, RiscVError> {
    if data.len() < std::mem::size_of::<Elf32Header>() {
        return Err(RiscVError::NotElf);
    }
    let header = unsafe { &*(data.as_ptr() as *const Elf32Header) };
    if &header.e_ident[0..4] != b"\x7FELF" {
        return Err(RiscVError::NotElf);
    }
    if header.e_ident[4] != 1 {
        return Err(RiscVError::Not32Bit);
    }
    if header.e_ident[5] != 1 {
        return Err(RiscVError::WrongEndian);
    }
    if header.e_machine != EM_RISCV {
        return Err(RiscVError::NotRiscV);
    }
    let phoff = header.e_phoff as usize;
    let phnum = header.e_phnum as usize;
    let phentsize = header.e_phentsize as usize;
    let ph_end = phoff
        .checked_add(
            phnum
                .checked_mul(phentsize)
                .ok_or(RiscVError::InvalidProgramHeader)?,
        )
        .ok_or(RiscVError::InvalidProgramHeader)?;
    if ph_end > data.len() {
        return Err(RiscVError::InvalidProgramHeader);
    }
    let program_headers = unsafe {
        std::slice::from_raw_parts(data[phoff..].as_ptr() as *const Elf32ProgramHeader, phnum)
    };
    let mut segments = Vec::new();
    for ph in program_headers {
        if ph.p_type != PT_LOAD {
            continue;
        }
        let start = ph.p_offset as usize;
        let filesz = ph.p_filesz as usize;
        let end = start
            .checked_add(filesz)
            .ok_or(RiscVError::InvalidProgramHeader)?;
        if end > data.len() {
            return Err(RiscVError::InvalidProgramHeader);
        }
        segments.push(ElfSegment {
            vaddr: ph.p_vaddr,
            data: data[start..end].to_vec(),
            mem_size: ph.p_memsz,
        });
    }
    Ok(ElfImage {
        entry: header.e_entry,
        segments,
    })
}
