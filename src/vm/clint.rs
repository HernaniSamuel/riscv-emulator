//! # Core-Local Interruptor (CLINT)
//!
//! ## Overview
//!
//! This module implements a minimal **CLINT (Core-Local Interruptor)**
//! device for **RISC-V** systems, designed for **single-hart**
//! environments.
//!
//! The CLINT provides local CPU timing services through
//! **memory-mapped I/O (MMIO)** registers and is primarily responsible
//! for machine timer interrupts.
//!
//! This implementation is intended for lightweight emulators,
//! educational projects, experimental kernels, and embedded software
//! such as FreeRTOS.
//!
//! ---
//!
//! ## Responsibilities
//!
//! The device maintains two 64-bit registers:
//!
//! - **mtime**: monotonically increasing machine timer counter.
//! - **mtimecmp**: comparison value used to trigger timer interrupts.
//!
//! Whenever:
//!
//! `mtime >= mtimecmp`
//!
//! a machine timer interrupt is considered pending.
//!
//! ---
//!
//! ## Implemented Memory Map
//!
//! | Offset | Final Address | Register | Description |
//! |--------|---------------|----------|-------------|
//! | `0x4000` | `0x0200_4000` | `mtimecmp low` | bits 31:0 |
//! | `0x4004` | `0x0200_4004` | `mtimecmp high` | bits 63:32 |
//! | `0xBFF8` | `0x0200_BFF8` | `mtime low` | bits 31:0 |
//! | `0xBFFC` | `0x0200_BFFC` | `mtime high` | bits 63:32 |
//!
//! Since accesses are 32-bit wide, each 64-bit register is exposed
//! as two independent halves.
//!
//! ---
//!
//! ## General Behavior
//!
//! Time advances through [`Clint::tick()`], which increments `mtime`
//! by one using wrapping arithmetic.
//!
//! This means that after reaching `u64::MAX`, the counter rolls back
//! to `0` without panicking.
//!
//! ---
//!
//! ## Default Reset State
//!
//! On initialization:
//!
//! - `mtime = 0`
//! - `mtimecmp = u64::MAX`
//!
//! This prevents an immediate timer interrupt at boot.
//!
//! ---
//!
//! ## Intentional Limitations
//!
//! This implementation only models the features required by simple
//! software environments.
//!
//! Unsupported features:
//!
//! - Multi-hart systems
//! - Software interrupts (`msip`)
//! - Privilege delegation
//! - External interrupt routing
//! - Real-time clock synchronization
//!
//! ---
//!
//! ## Typical Emulator Usage
//!
//! Common per-cycle flow:
//!
//! 1. CPU executes an instruction
//! 2. Emulator calls `clint.tick()`
//! 3. Check `clint.timer_pending()`
//! 4. If true, raise machine timer interrupt
//!
//! ---
//!
//! ## Design Goals
//!
//! This code prioritizes:
//!
//! - simplicity
//! - predictability
//! - basic RISC-V software compatibility
//! - maintainability
//!
//! It does not attempt full hardware-level fidelity.

/// Base address of the CLINT MMIO region.
pub const CLINT_BASE: u32 = 0x0200_0000;

/// Low 32 bits of `mtimecmp`.
///
/// Controls bits `31:0` of the compare register.
pub const MTIMECMP_LO: u32 = CLINT_BASE + 0x4000;

/// High 32 bits of `mtimecmp`.
pub const MTIMECMP_HI: u32 = CLINT_BASE + 0x4004;

/// Low 32 bits of `mtime`.
pub const MTIME_LO: u32 = CLINT_BASE + 0xBFF8;

/// High 32 bits of `mtime`.
pub const MTIME_HI: u32 = CLINT_BASE + 0xBFFC;

/// Minimal CLINT device state.
///
/// ## Internal Registers
///
/// - `mtime`: current timer counter
/// - `mtimecmp`: interrupt compare threshold
///
/// ## Interrupt Rule
///
/// A timer interrupt is pending whenever:
///
/// `mtime >= mtimecmp`
#[derive(Debug, Clone, PartialEq)]
pub struct Clint {
    pub mtime: u64,
    pub mtimecmp: u64,
}

impl Clint {
    /// Advances the timer by one tick.
    ///
    /// Uses wrapping arithmetic.
    ///
    /// ## Effects
    ///
    /// - increments `mtime`
    /// - leaves `mtimecmp` unchanged
    pub fn tick(&mut self) {
        self.mtime = self.mtime.wrapping_add(1);
    }

    /// Returns whether a timer interrupt is pending.
    ///
    /// ## Return Value
    ///
    /// - `true` if `mtime >= mtimecmp`
    /// - `false` otherwise
    pub fn timer_pending(&self) -> bool {
        self.mtime >= self.mtimecmp
    }

    /// Reads a 32-bit MMIO register.
    ///
    /// ## Parameters
    ///
    /// - `addr`: physical address
    ///
    /// ## Returns
    ///
    /// - `Some(value)` if supported
    /// - `None` otherwise
    pub fn read_u32(&self, addr: u32) -> Option<u32> {
        match addr {
            MTIMECMP_LO => Some(self.mtimecmp as u32),
            MTIMECMP_HI => Some((self.mtimecmp >> 32) as u32),

            MTIME_LO => Some(self.mtime as u32),
            MTIME_HI => Some((self.mtime >> 32) as u32),

            _ => None,
        }
    }

    /// Writes a 32-bit MMIO register.
    ///
    /// ## Parameters
    ///
    /// - `addr`: physical address
    /// - `value`: 32-bit value
    ///
    /// ## Returns
    ///
    /// - `true` if write succeeded
    /// - `false` if address is unsupported
    ///
    /// ## Notes
    ///
    /// 64-bit registers are updated as independent halves.
    pub fn write_u32(&mut self, addr: u32, value: u32) -> bool {
        match addr {
            MTIMECMP_LO => {
                self.mtimecmp = (self.mtimecmp & 0xFFFF_FFFF_0000_0000) | value as u64;
            }

            MTIMECMP_HI => {
                self.mtimecmp = (self.mtimecmp & 0x0000_0000_FFFF_FFFF) | ((value as u64) << 32);
            }

            MTIME_LO => {
                self.mtime = (self.mtime & 0xFFFF_FFFF_0000_0000) | value as u64;
            }

            MTIME_HI => {
                self.mtime = (self.mtime & 0x0000_0000_FFFF_FFFF) | ((value as u64) << 32);
            }

            _ => return false,
        }

        true
    }
}

impl Default for Clint {
    /// Safe reset state.
    fn default() -> Self {
        Self {
            mtime: 0,
            mtimecmp: u64::MAX,
        }
    }
}
