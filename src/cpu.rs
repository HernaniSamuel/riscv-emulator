//! CPU state, trap handling, and timer logic.
//!
//! This module defines the complete architectural state of the emulated
//! RV32IMA hart: general-purpose registers, program counter, all machine-mode
//! CSRs, and the CLINT timer counters.
//!
//! ## Key types
//!
//! - [`CpuState`] — the full register file and CSR set for one hart.
//! - [`Trap`] — every exception and interrupt cause the CPU can raise.
//! - [`StepResult`] — what the execution loop returns to its caller.
//! - [`Csr`] — symbolic names for CSR addresses.
//!
//! ## Design note: `extraflags`
//!
//! To stay compatible with the original C implementation, three logical fields
//! are packed into the single `extraflags` word:
//!
//! ```text
//! bits [1:0]  — privilege mode  (0 = U-mode, 3 = M-mode)
//! bit  [2]    — WFI sleep flag
//! bits [31:3] — LR/SC reservation address (RAM offset >> 3)
//! ```
//!
//! Accessor methods ([`CpuState::get_privilege`], [`CpuState::get_wfi`], etc.)
//! hide this packing from the rest of the emulator.

/// Outcome of a single execution batch ([`crate::emulator::Emulator::step`]).
///
/// The run loop inspects this value after every `step` call to decide what to
/// do next (sleep, restart, stop, etc.).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum StepResult {
    /// Normal execution — at least one instruction retired.
    Ok = 0,
    /// CPU executed a WFI and is sleeping until the next interrupt.
    Wfi = 1,
    /// Unrecoverable fault (only raised when `fail_on_all_faults` is set).
    Fault = 3,
    /// Kernel requested a system restart via SYSCON (`0x7777`).
    Restart = 0x7777,
    /// Kernel requested a clean shutdown via SYSCON (`0x5555`).
    Poweroff = 0x5555,
}

/// Trap cause codes, as defined by the RISC-V privileged specification.
///
/// `Trap::None` is an internal sentinel meaning "no trap is pending"; it is
/// never written to `mcause`. Interrupt causes have bit 31 set (e.g.
/// [`Trap::IntTimer`] = `0x8000_0007`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Trap {
    /// Sentinel — no trap pending.
    None = 0xffff_ffff,
    /// Instruction address misaligned.
    ExcInsnMisaligned = 0,
    /// Instruction access fault (PC outside RAM).
    ExcInsnAccessFault = 1,
    /// Illegal instruction encoding.
    ExcIllegalInsn = 2,
    /// `ebreak` breakpoint.
    ExcBreakpoint = 3,
    /// Load access fault (address outside RAM and not MMIO).
    ExcLoadAccessFault = 5,
    /// Store/AMO access fault.
    ExcStoreAccessFault = 7,
    /// `ecall` from U-mode.
    ExcEcallU = 8,
    /// `ecall` from M-mode.
    ExcEcallM = 11,
    /// Machine timer interrupt (MTIP — bit 7 of `mip`/`mie`).
    IntTimer = 0x8000_0007,
}

impl Trap {
    /// Returns `true` if this is an interrupt (bit 31 set in mcause).
    #[inline]
    pub fn is_interrupt(self) -> bool {
        (self as u32) & 0x8000_0000 != 0
    }
    /// The value to write directly into `mcause`.
    #[inline]
    pub fn to_mcause(self) -> u32 {
        self as u32
    }
    /// Returns `true` if this is the no-trap sentinel.
    #[inline]
    pub fn is_none(self) -> bool {
        self == Trap::None
    }
}

/// CSR addresses used by the emulator.
///
/// The standard machine-mode CSRs follow the RISC-V privileged spec.
/// The `Print*` and `ReadKbd` entries (`0x136`–`0x140`) are custom debug
/// extensions inherited from the original mini-rv32ima: they let bare-metal
/// code print integers, hex values, strings, and characters without needing a
/// UART driver.
#[repr(u32)]
pub enum Csr {
    /// Machine status register.
    Mstatus = 0x300,
    /// Machine ISA register (reports RV32IMA to the kernel).
    Misa = 0x301,
    /// Machine interrupt-enable register.
    Mie = 0x304,
    /// Machine trap-handler base address.
    Mtvec = 0x305,
    /// Machine scratch register (used by trap handlers for context save).
    Mscratch = 0x340,
    /// Machine exception PC — address of the trapping instruction.
    Mepc = 0x341,
    /// Machine cause register.
    Mcause = 0x342,
    /// Machine trap value (faulting address or instruction).
    Mtval = 0x343,
    /// Machine interrupt-pending register.
    Mip = 0x344,
    /// Cycle counter (lower 32 bits).
    Cycle = 0xc00,
    /// Vendor ID (returns `0xff0ff0ff`).
    Mvendorid = 0xf11,
    // ── Debug extensions (mini-rv32ima custom CSRs) ───────────────────────
    /// Write a signed integer to stdout.
    PrintInt = 0x136,
    /// Write a hex word to stdout.
    PrintHex = 0x137,
    /// Write a null-terminated string from RAM to stdout.
    PrintStr = 0x138,
    /// Write a single character to stdout.
    PrintChar = 0x139,
    /// Read one byte from stdin (-1 if none available).
    ReadKbd = 0x140,
}

/// Complete architectural state of one RV32IMA hart.
///
/// All fields that map directly to ISA registers are `pub` so that the
/// emulator core in [`crate::emulator`] and the MMIO layer in
/// [`crate::mmio`] can access them without indirection. Fields that pack
/// multiple logical values (only `extraflags`) expose typed accessors instead.
///
/// ## Timer fields
///
/// The CLINT timer is implemented with four 32-bit fields that form two 64-bit
/// values:
///
/// - `timerl` / `timerh` — the running `mtime` counter, incremented by
///   [`CpuState::tick_timer`] on every `step` call.
/// - `timermatchl` / `timermatchh` — the `mtimecmp` threshold written by the
///   kernel via MMIO. When `mtime > mtimecmp`, MTIP fires.
#[derive(Default)]
pub struct CpuState {
    /// General-purpose registers x0–x31. x0 is architecturally zero; the
    /// decode loop enforces this by skipping writes when `rdid == 0`.
    pub regs: [u32; 32],
    /// Program counter.
    pub pc: u32,
    /// `mstatus` CSR.
    pub mstatus: u32,
    /// Cycle counter — low 32 bits.
    pub cyclel: u32,
    /// Cycle counter — high 32 bits.
    pub cycleh: u32,
    /// `mtime` — low 32 bits. Incremented each `step` by elapsed microseconds.
    pub timerl: u32,
    /// `mtime` — high 32 bits.
    pub timerh: u32,
    /// `mtimecmp` — low 32 bits. Written by the kernel via CLINT MMIO.
    pub timermatchl: u32,
    /// `mtimecmp` — high 32 bits.
    pub timermatchh: u32,
    /// `mscratch` CSR.
    pub mscratch: u32,
    /// `mtvec` CSR — trap handler base address.
    pub mtvec: u32,
    /// `mie` CSR — interrupt enable bits.
    pub mie: u32,
    /// `mip` CSR — interrupt pending bits.
    pub mip: u32,
    /// `mepc` CSR — exception program counter.
    pub mepc: u32,
    /// `mtval` CSR — trap value (faulting address or bad instruction).
    pub mtval: u32,
    /// `mcause` CSR — trap cause code.
    pub mcause: u32,
    /// Packed field — see module-level docs for the bit layout.
    pub extraflags: u32,
}

impl CpuState {
    // ── Privilege mode ────────────────────────────────────────────────────────

    /// Current privilege level: `0` = U-mode, `3` = M-mode.
    ///
    /// This emulator only uses M-mode (3) and, when the kernel is running,
    /// U-mode (0). S-mode is not implemented.
    #[inline]
    pub fn get_privilege(&self) -> u32 {
        self.extraflags & 0x3
    }

    /// Set the current privilege level.
    #[inline]
    pub fn set_privilege(&mut self, p: u32) {
        self.extraflags = (self.extraflags & !0x3) | (p & 0x3);
    }

    // ── WFI sleep flag ────────────────────────────────────────────────────────

    /// Returns `true` if the CPU is in WFI (Wait-For-Interrupt) sleep.
    ///
    /// While sleeping, `step` returns [`StepResult::Wfi`] immediately without
    /// executing any instructions. The flag is cleared by [`tick_timer`] when
    /// MTIP fires.
    ///
    /// [`tick_timer`]: CpuState::tick_timer
    #[inline]
    pub fn get_wfi(&self) -> bool {
        (self.extraflags >> 2) & 1 != 0
    }

    /// Set or clear the WFI sleep flag.
    #[inline]
    pub fn set_wfi(&mut self, v: bool) {
        if v {
            self.extraflags |= 4;
        } else {
            self.extraflags &= !4;
        }
    }

    // ── LR/SC reservation ─────────────────────────────────────────────────────

    /// Returns the current LR/SC reservation address (RAM offset).
    ///
    /// Set by `LR.W`, consumed and cleared by `SC.W`.
    #[inline]
    pub fn get_reservation(&self) -> u32 {
        self.extraflags >> 3
    }

    /// Record a new LR/SC reservation at the given RAM offset.
    #[inline]
    pub fn set_reservation(&mut self, ofs: u32) {
        self.extraflags = (self.extraflags & 0x7) | (ofs << 3);
    }

    // ── 64-bit cycle counter ──────────────────────────────────────────────────

    /// Read the full 64-bit cycle counter.
    #[inline]
    pub fn get_cycle64(&self) -> u64 {
        ((self.cycleh as u64) << 32) | self.cyclel as u64
    }

    /// Write the full 64-bit cycle counter.
    #[inline]
    pub fn set_cycle64(&mut self, v: u64) {
        self.cyclel = v as u32;
        self.cycleh = (v >> 32) as u32;
    }

    // ── Timer ─────────────────────────────────────────────────────────────────

    /// Advance `mtime` by `elapsed_us` microseconds and update MTIP.
    ///
    /// Called at the start of every [`crate::emulator::Emulator::step`] with
    /// the number of microseconds that have passed since the previous call.
    /// This keeps the CLINT timer accurate relative to wall-clock time.
    ///
    /// **MTIP** (Machine Timer Interrupt Pending, `mip` bit 7) is set when
    /// `mtime > mtimecmp` and `mtimecmp != 0`, and cleared otherwise. Setting
    /// MTIP also clears the WFI flag so the CPU wakes up.
    pub fn tick_timer(&mut self, elapsed_us: u32) {
        // Increment mtime with carry propagation between the two 32-bit halves.
        let new = self.timerl.wrapping_add(elapsed_us);
        if new < self.timerl {
            self.timerh = self.timerh.wrapping_add(1);
        }
        self.timerl = new;

        if (self.timerh > self.timermatchh
            || (self.timerh == self.timermatchh && self.timerl > self.timermatchl))
            && (self.timermatchh != 0 || self.timermatchl != 0)
        {
            self.set_wfi(false);
            self.mip |= 1 << 7; // set MTIP
        } else {
            self.mip &= !(1 << 7); // clear MTIP
        }
    }

    // ── Commit Trap ───────────────────────────────────────────────────────────

    /// Save machine context and redirect the PC to `mtvec`.
    ///
    /// This is the final step of trap handling inside `step`. It writes the
    /// standard trap CSRs and returns the new PC value (always `mtvec`).
    ///
    /// ## What gets saved
    ///
    /// | CSR | Value written |
    /// |-----|--------------|
    /// | `mepc` | PC of the trapping instruction (PC+4 for interrupts) |
    /// | `mcause` | Trap cause code from [`Trap::to_mcause`] |
    /// | `mtval` | Faulting address (load/store faults) or trapping PC |
    /// | `mstatus` | MIE → MPIE, current privilege → MPP, MIE cleared |
    ///
    /// The privilege mode is set to M-mode (3) regardless of where the trap
    /// originated. The kernel's trap handler is expected to inspect `mcause`,
    /// handle the event, and return with `mret`.
    pub fn commit_trap(&mut self, trap: Trap, rval: u32, mut pc: u32) -> u32 {
        if trap.is_interrupt() {
            self.mcause = trap.to_mcause();
            self.mtval = 0;
            // Interrupts return to the instruction *after* the one that was
            // about to execute, so the kernel adjusts mepc += 4 before mret.
            pc = pc.wrapping_add(4);
        } else {
            self.mcause = trap.to_mcause();
            // For memory faults, mtval holds the faulting address; for all
            // other exceptions it holds the PC of the bad instruction.
            self.mtval = if trap == Trap::ExcLoadAccessFault || trap == Trap::ExcStoreAccessFault {
                rval
            } else {
                pc
            };
        }

        let cur_priv = self.get_privilege();
        self.mepc = pc;
        // Pack MIE into MPIE (bit 3 → bit 7) and privilege into MPP (bits 11:12).
        self.mstatus = ((self.mstatus & 0x08) << 4) | (cur_priv << 11);
        self.set_privilege(3); // enter M-mode

        self.mtvec // caller sets pc = mtvec
    }
}
