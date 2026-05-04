//! Memory-mapped I/O — peripheral registers and RAM access helpers.
//!
//! Any address in `0x1000_0000..0x1200_0000` is treated as MMIO.
//! Reads and writes to RAM use `ptr::read_unaligned` and `ptr::write_unaligned` to handle
//! unaligned accesses, which are legal in RV32I.
//!
//! ## Peripheral map
//!
//! | Address | Peripheral | Notes |
//! |---------|-----------|-------|
//! | `0x1000_0000` | UART TX/RX | Write = transmit byte; read = receive byte |
//! | `0x1000_0005` | UART LSR | Bit 5 = TX ready, bit 0 = RX ready |
//! | `0x1100_4000` | CLINT mtimecmp low | Written by kernel to schedule next timer IRQ |
//! | `0x1100_4004` | CLINT mtimecmp high | |
//! | `0x1100_BFF8` | CLINT mtime low | Read-only; driven by [`CpuState::tick_timer`] |
//! | `0x1100_BFFC` | CLINT mtime high | |
//! | `0x1110_0000` | SYSCON | `0x5555` = poweroff, `0x7777` = restart |
//!
//! ## UART
//!
//! The UART is a simplified NS16550-compatible device. TX is always ready
//! (`LSR` bits 5 and 6 hardwired to 1). RX readiness reflects whether the
//! host has a key waiting in stdin.
//!
//! In WASM mode, TX bytes are pushed to an `output_buf` instead of being
//! printed to stdout. This lets `EmulatorWasm::step_batch`
//! collect output and hand it back to the JavaScript caller.
//!
//! ## CLINT
//!
//! The Core Local Interruptor provides `mtime` (a free-running 64-bit counter)
//! and `mtimecmp` (the comparison threshold). When `mtime > mtimecmp`, MTIP
//! fires. The actual increment happens in [`CpuState::tick_timer`]; this
//! module only handles the MMIO read/write interface.
//!
//! [`CpuState::tick_timer`]: crate::cpu::CpuState::tick_timer

use crate::cpu::{CpuState, Csr, StepResult};
use crate::platform::Platform;
use std::io::{self, Write};

// ── UART NS16550 ──────────────────────────────────────────────────────────────

/// UART TX register (write) / RX register (read).
pub const UART_TX: u32 = 0x1000_0000;
/// UART Line Status Register.
pub const UART_LSR: u32 = 0x1000_0005;
/// LSR value when TX is ready: THRE (bit 5) + TEMT (bit 6).
const UART_LSR_TX_READY: u32 = 0x60;
/// LSR DR bit (bit 0) — set when a byte is available to read.
const UART_LSR_RX_READY: u32 = 0x01;

// ── CLINT ─────────────────────────────────────────────────────────────────────

/// CLINT `mtimecmp` — low 32 bits (write-only from software perspective).
pub const CLINT_MTIMECMP_LO: u32 = 0x1100_4000;
/// CLINT `mtimecmp` — high 32 bits.
pub const CLINT_MTIMECMP_HI: u32 = 0x1100_4004;
/// CLINT `mtime` — low 32 bits (read-only).
pub const CLINT_MTIME_LO: u32 = 0x1100_BFF8;
/// CLINT `mtime` — high 32 bits (read-only).
pub const CLINT_MTIME_HI: u32 = 0x1100_BFFC;

// ── SYSCON ────────────────────────────────────────────────────────────────────

/// System controller address. Writing `0x5555` powers off; `0x7777` restarts.
pub const SYSCON_ADDR: u32 = 0x1110_0000;

/// Returns `true` if `addr` falls in the MMIO region (`0x1000_0000..0x1200_0000`).
#[inline]
pub fn is_mmio(addr: u32) -> bool {
    (0x1000_0000..0x1200_0000).contains(&addr)
}

/// Handle an MMIO store (write).
///
/// Returns [`StepResult::Ok`] in all normal cases. Returns
/// [`StepResult::Restart`] or [`StepResult::Poweroff`] when the kernel writes
/// to SYSCON, which causes the run loop to exit immediately.
///
/// The `output_buf` parameter controls UART output routing:
/// - `None` — bytes are printed directly to stdout (native binary mode).
/// - `Some(buf)` — bytes are appended to the buffer (WASM mode).
pub fn handle_store(
    cpu: &mut CpuState,
    addr: u32,
    val: u32,
    output_buf: Option<&mut Vec<u8>>,
) -> StepResult {
    match addr {
        // UART TX — send the low byte to the console.
        UART_TX => {
            let byte = val as u8;
            if let Some(buf) = output_buf {
                buf.push(byte);
            } else {
                print!("{}", byte as char);
                let _ = io::stdout().flush();
            }
        }

        // CLINT mtimecmp — schedule the next timer interrupt.
        CLINT_MTIMECMP_LO => {
            cpu.timermatchl = val;
        }
        CLINT_MTIMECMP_HI => {
            cpu.timermatchh = val;
        }

        // SYSCON — power control.
        SYSCON_ADDR => {
            // Advance the PC past the store instruction before returning so
            // the state is consistent if the caller inspects it after halt.
            cpu.pc += 4;
            return match val {
                0x7777 => StepResult::Restart,
                0x5555 => StepResult::Poweroff,
                _ => StepResult::Fault,
            };
        }

        _ => {}
    }
    StepResult::Ok
}

/// Handle an MMIO load (read).
///
/// Returns `0` for unmapped addresses, which is harmless and consistent with
/// how real hardware behaves on unimplemented registers.
pub fn handle_load(cpu: &CpuState, addr: u32, plat: &mut dyn Platform) -> u32 {
    match addr {
        // UART LSR — TX is always ready; RX ready bit reflects stdin.
        UART_LSR => {
            let rx = if plat.is_kb_hit() != 0 {
                UART_LSR_RX_READY
            } else {
                0
            };
            UART_LSR_TX_READY | rx
        }

        // UART RX — consume one byte from stdin if available.
        UART_TX if plat.is_kb_hit() != 0 => plat.read_kb_byte() as u32,

        // CLINT mtime — the kernel reads these to get the current time.
        CLINT_MTIME_LO => cpu.timerl,
        CLINT_MTIME_HI => cpu.timerh,

        _ => 0,
    }
}

/// Handle a write to an unrecognized CSR number.
///
/// The standard CSRs are handled directly in the decode loop. This function
/// covers the custom debug CSRs (`0x136`–`0x139`) that the original
/// mini-rv32ima added for bare-metal printf-style debugging without a UART
/// driver.
pub fn handle_csr_write(ram: &[u8], ram_size: u32, csrno: u32, value: u32) {
    const RAM_BASE: u32 = 0x8000_0000;
    match csrno {
        x if x == Csr::PrintInt as u32 => {
            print!("{}", value as i32);
            let _ = io::stdout().flush();
        }
        x if x == Csr::PrintHex as u32 => {
            print!("{:08x}", value);
            let _ = io::stdout().flush();
        }
        x if x == Csr::PrintChar as u32 => {
            print!("{}", value as u8 as char);
            let _ = io::stdout().flush();
        }
        x if x == Csr::PrintStr as u32 => {
            let offset = value.wrapping_sub(RAM_BASE) as usize;
            if offset >= ram_size as usize {
                eprintln!("DEBUG: invalid PrintStr pointer ({:08x})", value);
                return;
            }
            let slice = &ram[offset..];
            let len = slice.iter().position(|&b| b == 0).unwrap_or(slice.len());
            let _ = io::stdout().write_all(&slice[..len]);
        }
        _ => {}
    }
}

/// Handle a read from an unrecognized CSR number.
///
/// Currently only `ReadKbd` (`0x140`) is handled here. It returns the next
/// byte from stdin, or `-1` cast to `i32` if none is available.
pub fn handle_csr_read(csrno: u32, plat: &mut dyn Platform) -> i32 {
    if csrno == Csr::ReadKbd as u32 {
        if plat.is_kb_hit() == 0 {
            return -1;
        }
        return plat.read_kb_byte();
    }
    0
}

// ─────────────────────────────────────────────────────────────────────────────
// RAM access helpers
//
// RV32I allows unaligned loads and stores. We use ptr::read/write_unaligned
// to implement these without undefined behaviour. All offsets are validated
// against ram_size before these functions are called.
// ─────────────────────────────────────────────────────────────────────────────

/// Load 1 byte, zero-extended to 32 bits (`LBU`).
#[inline]
pub fn mem_load1(ram: &[u8], ofs: u32) -> u32 {
    ram[ofs as usize] as u32
}

/// Load 2 bytes, zero-extended to 32 bits (`LHU`). Handles unaligned offsets.
#[inline]
pub fn mem_load2(ram: &[u8], ofs: u32) -> u32 {
    let v = unsafe { (ram.as_ptr().add(ofs as usize) as *const u16).read_unaligned() };
    v as u32
}

/// Load 4 bytes (`LW`). Handles unaligned offsets.
#[inline]
pub fn mem_load4(ram: &[u8], ofs: u32) -> u32 {
    unsafe { (ram.as_ptr().add(ofs as usize) as *const u32).read_unaligned() }
}

/// Load 1 byte, sign-extended to 32 bits (`LB`).
#[inline]
pub fn mem_load1s(ram: &[u8], ofs: u32) -> u32 {
    ram[ofs as usize] as i8 as i32 as u32
}

/// Load 2 bytes, sign-extended to 32 bits (`LH`). Handles unaligned offsets.
#[inline]
pub fn mem_load2s(ram: &[u8], ofs: u32) -> u32 {
    let v = unsafe { (ram.as_ptr().add(ofs as usize) as *const u16).read_unaligned() };
    v as i16 as i32 as u32
}

/// Store 1 byte (`SB`).
#[inline]
pub fn mem_store1(ram: &mut [u8], ofs: u32, val: u32) {
    ram[ofs as usize] = val as u8;
}

/// Store 2 bytes (`SH`). Handles unaligned offsets.
#[inline]
pub fn mem_store2(ram: &mut [u8], ofs: u32, val: u32) {
    unsafe {
        (ram.as_mut_ptr().add(ofs as usize) as *mut u16).write_unaligned(val as u16);
    }
}

/// Store 4 bytes (`SW`). Handles unaligned offsets.
#[inline]
pub fn mem_store4(ram: &mut [u8], ofs: u32, val: u32) {
    unsafe {
        (ram.as_mut_ptr().add(ofs as usize) as *mut u32).write_unaligned(val);
    }
}
