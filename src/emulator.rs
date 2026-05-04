//! The emulator core — CPU execution loop, image loading, and run control.
//!
//! [`Emulator`] is the central type. It owns the [`CpuState`], the RAM buffer,
//! and optional debug state (symbol table, output buffer). Everything else —
//! time, keyboard, sleep — is delegated to a [`Platform`] implementation.
//!
//! ## Execution flow
//!
//! ```text
//! Emulator::run()
//!   └─ loop {
//!        tick_timer(elapsed_us)       // advance mtime, check MTIP
//!        check WFI / pending IRQ
//!        'iloop: for 0..count {
//!            fetch instruction
//!            decode opcode
//!            execute → update regs / pc / trap
//!        }
//!        commit_trap() if any trap fired
//!        update cycle counter
//!      }
//! ```
//!
//! ## Loading images
//!
//! | Method | Use case |
//! |--------|---------|
//! | [`Emulator::load_raw`] | Linux kernel raw image + DTB |
//! | [`Emulator::load_raw_from_bytes`] | Same, but from an in-memory slice (WASM) |
//! | [`Emulator::load_elf`] | Bare-metal ELF or FreeRTOS |
//!
//! ## ISA coverage
//!
//! | Extension | Instructions |
//! |-----------|-------------|
//! | RV32I | All base integer instructions |
//! | RV32M | MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU |
//! | RV32A | LR.W, SC.W, AMOSWAP/ADD/XOR/AND/OR/MIN/MAX/MINU/MAXU.W |
//! | Zicsr | CSRRW, CSRRS, CSRRC, CSRRWI, CSRRSI, CSRRCI |
//! | Privileged | MRET, WFI, ECALL, EBREAK |
//! | FENCE | Treated as no-op (no cache model) |

use crate::cpu::{CpuState, Csr, StepResult, Trap};
use crate::elf::SymbolTable;
use crate::mmio;
use crate::platform::Platform;
use crate::{dtb, elf};

/// Physical base address of emulated RAM.
///
/// All kernel images and ELF segments are loaded at or above this address.
pub const RAM_BASE: u32 = 0x8000_0000;

/// A complete RV32IMA emulator instance.
///
/// Owns the CPU state, the RAM buffer, and optional debug/WASM state.
/// Peripheral I/O (time, keyboard, sleep) is delegated to a [`Platform`].
pub struct Emulator {
    /// The CPU register file and CSR set.
    pub cpu: CpuState,
    /// Flat emulated RAM buffer. Indexed as `ram[addr - RAM_BASE]`.
    pub ram: Vec<u8>,
    /// Size of `ram` in bytes.
    pub ram_size: u32,
    /// When `true`, any fault halts immediately instead of invoking the trap
    /// handler. Enabled by the `-d` CLI flag for debugging.
    pub fail_on_all_faults: bool,
    /// Symbol table from the loaded ELF, if available.
    /// Populated by [`load_elf`] when the image is not stripped.
    ///
    /// [`load_elf`]: Emulator::load_elf
    pub symbols: Option<SymbolTable>,
    /// UART output buffer. In WASM mode, bytes written to the UART go here
    /// instead of stdout. Drained by [`drain_output`] after each batch.
    ///
    /// [`drain_output`]: Emulator::drain_output
    pub output_buf: Vec<u8>,
    /// When `true`, UART bytes go to `output_buf` instead of stdout.
    /// Set automatically by [`load_raw_from_bytes`].
    ///
    /// [`load_raw_from_bytes`]: Emulator::load_raw_from_bytes
    pub wasm_mode: bool,
}

impl Emulator {
    /// Create a new emulator with `ram_size` bytes of RAM, all zeroed.
    pub fn new(ram_size: u32) -> Self {
        Emulator {
            cpu: CpuState::default(),
            ram: vec![0u8; ram_size as usize],
            ram_size,
            fail_on_all_faults: false,
            symbols: None,
            output_buf: Vec::new(),
            wasm_mode: false,
        }
    }

    /// Drain and return the accumulated UART output buffer.
    ///
    /// After this call, `output_buf` is empty and ready for the next batch.
    /// Only meaningful in WASM mode; always returns an empty vec otherwise.
    pub fn drain_output(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.output_buf)
    }

    /// Load a raw kernel image from an in-memory byte slice.
    ///
    /// Equivalent to [`load_raw`] but without disk I/O. Used by the WASM
    /// target where the kernel image is embedded with `include_bytes!`.
    ///
    /// [`load_raw`]: Emulator::load_raw
    pub fn load_raw_from_bytes(
        &mut self,
        image_data: &[u8],
        kernel_cmdline: Option<&str>,
    ) -> Result<(), i32> {
        if image_data.len() as u32 > self.ram_size {
            return Err(-6);
        }
        self.ram.fill(0);
        self.ram[..image_data.len()].copy_from_slice(image_data);

        // Use the embedded default DTB.
        let dtb_blob = crate::dtb::DEFAULT64MB_DTB;
        let dtb_ptr = self.ram_size - dtb_blob.len() as u32;
        self.ram[dtb_ptr as usize..dtb_ptr as usize + dtb_blob.len()].copy_from_slice(dtb_blob);

        if let Some(cmdline) = kernel_cmdline {
            const CMDLINE_OFFSET: u32 = 0xc0;
            const CMDLINE_MAX_LEN: usize = 54;
            let dst = &mut self.ram[dtb_ptr as usize + CMDLINE_OFFSET as usize
                ..dtb_ptr as usize + CMDLINE_OFFSET as usize + CMDLINE_MAX_LEN];
            let src = cmdline.as_bytes();
            let n = src.len().min(CMDLINE_MAX_LEN - 1);
            dst[..n].copy_from_slice(&src[..n]);
            dst[n] = 0;
        }

        // patch_dtb_ram_size is defined as a local function below.
        fn patch(ram: &mut [u8], dtb_ptr: u32) {
            const OFFSET: u32 = 0x13c;
            const SENTINEL: u32 = 0x00c0ff03;
            if crate::mmio::mem_load4(ram, dtb_ptr + OFFSET) == SENTINEL {
                crate::mmio::mem_store4(ram, dtb_ptr + OFFSET, dtb_ptr.to_be());
            }
        }
        patch(&mut self.ram, dtb_ptr);

        self.cpu = CpuState::default();
        self.cpu.pc = RAM_BASE;
        self.cpu.regs[10] = 0;
        self.cpu.regs[11] = dtb_ptr + RAM_BASE;
        self.cpu.set_privilege(3);
        self.wasm_mode = true;
        Ok(())
    }
}

/// Parameters that control the [`Emulator::run`] loop.
pub struct RunConfig {
    /// Maximum number of instructions to execute. `-1` means unlimited.
    pub instct: i64,
    /// Time divisor applied to wall-clock time before feeding the CLINT timer.
    /// `1` = real time. Values > 1 make the CPU appear slower, which is useful
    /// for deterministic testing (`-l` locks it to the instruction counter).
    pub time_divisor: u32,
    /// When `true`, the cycle counter is used as the time base instead of
    /// wall-clock time. Produces fully deterministic execution.
    pub fixed_update: bool,
    /// When `true`, the run loop calls [`Platform::mini_sleep`] during WFI
    /// to avoid burning host CPU. Disable with `-p` for maximum throughput.
    pub do_sleep: bool,
    /// When `true`, execute one instruction per loop iteration and call
    /// [`Emulator::dump_state`] after each one. Enabled by `-s`.
    pub single_step: bool,
    /// Optional writer for the compact execution trace (`--trace <FILE>`).
    /// When `Some`, every instruction is logged with the registers it modified.
    pub trace: Option<Box<dyn std::io::Write>>,
}

impl Default for RunConfig {
    fn default() -> Self {
        RunConfig {
            instct: -1,
            time_divisor: 1,
            fixed_update: false,
            do_sleep: true,
            single_step: false,
            trace: None,
        }
    }
}

// ── Immediate decoders ───────────────────────────────────────────────────────

#[inline]
fn decode_imm_i(ir: u32) -> i32 {
    let imm = ir >> 20;
    (imm | if imm & 0x800 != 0 { 0xffff_f000 } else { 0 }) as i32
}
#[inline]
fn decode_imm_s(ir: u32) -> i32 {
    let imm = ((ir >> 7) & 0x1f) | ((ir & 0xfe00_0000) >> 20);
    (imm | if imm & 0x800 != 0 { 0xffff_f000 } else { 0 }) as i32
}
#[inline]
fn decode_imm_b(ir: u32) -> i32 {
    let imm =
        ((ir & 0xf00) >> 7) | ((ir & 0x7e00_0000) >> 20) | ((ir & 0x80) << 4) | ((ir >> 31) << 12);
    (imm | if imm & 0x1000 != 0 { 0xffffe000 } else { 0 }) as i32
}
#[inline]
fn decode_imm_j(ir: u32) -> i32 {
    let imm = ((ir & 0x8000_0000) >> 11)
        | ((ir & 0x7fe0_0000) >> 20)
        | ((ir & 0x0010_0000) >> 9)
        | (ir & 0x000f_f000);
    (imm | if imm & 0x0010_0000 != 0 {
        0xffe0_0000
    } else {
        0
    }) as i32
}
#[inline]
fn decode_imm_u(ir: u32) -> i32 {
    (ir & 0xffff_f000) as i32
}

// ── DTB helpers ──────────────────────────────────────────────────────────────

/// Patch the RAM size field in the embedded DTB.
///
/// The default DTB contains the sentinel value `0x00c0ff03` at offset `0x13c`.
/// This function replaces it with the actual DTB pointer (= RAM size − DTB
/// size) in big-endian, which the kernel uses to determine how much memory
/// is available.
fn patch_dtb_ram_size(ram: &mut [u8], dtb_ptr: u32) {
    const OFFSET: u32 = 0x13c;
    const SENTINEL: u32 = 0x00c0ff03;
    if mmio::mem_load4(ram, dtb_ptr + OFFSET) == SENTINEL {
        mmio::mem_store4(ram, dtb_ptr + OFFSET, dtb_ptr.to_be());
    }
}

impl Emulator {
    /// Load a raw Linux kernel image and a Device Tree Blob into RAM.
    ///
    /// - `dtb_file = None` — use the embedded 64 MB DTB from [`crate::dtb`].
    /// - `dtb_file = Some("disable")` — skip DTB entirely (`a1 = 0`).
    /// - `dtb_file = Some(path)` — load DTB from the given file.
    ///
    /// On success the CPU is reset with:
    /// - `pc = RAM_BASE` (kernel entry point)
    /// - `a0 = 0` (hart ID)
    /// - `a1 = dtb_ptr + RAM_BASE` (DTB physical address)
    /// - privilege = M-mode
    pub fn load_raw(
        &mut self,
        image_file: &str,
        dtb_file: Option<&str>,
        kernel_cmdline: Option<&str>,
    ) -> Result<(), i32> {
        let image_data = std::fs::read(image_file).map_err(|_| {
            eprintln!("Error: \"{}\" not found", image_file);
            -5i32
        })?;

        if image_data.len() as u32 > self.ram_size {
            eprintln!(
                "Error: image ({} bytes) does not fit in {} bytes of RAM",
                image_data.len(),
                self.ram_size
            );
            return Err(-6);
        }

        self.ram.fill(0);
        self.ram[..image_data.len()].copy_from_slice(&image_data);

        let dtb_ptr: u32 = match dtb_file {
            Some("disable") => 0,

            Some(path) => {
                let dtb_data = std::fs::read(path).map_err(|_| {
                    eprintln!("Error: \"{}\" not found", path);
                    -5i32
                })?;
                let ptr = self.ram_size - dtb_data.len() as u32;
                self.ram[ptr as usize..ptr as usize + dtb_data.len()].copy_from_slice(&dtb_data);
                ptr
            }

            None => {
                let blob = dtb::DEFAULT64MB_DTB;
                let ptr = self.ram_size - blob.len() as u32;
                self.ram[ptr as usize..ptr as usize + blob.len()].copy_from_slice(blob);

                if let Some(cmdline) = kernel_cmdline {
                    const CMDLINE_OFFSET: u32 = 0xc0;
                    const CMDLINE_MAX_LEN: usize = 54;
                    let dst = &mut self.ram[ptr as usize + CMDLINE_OFFSET as usize
                        ..ptr as usize + CMDLINE_OFFSET as usize + CMDLINE_MAX_LEN];
                    let src = cmdline.as_bytes();
                    let n = src.len().min(CMDLINE_MAX_LEN - 1);
                    dst[..n].copy_from_slice(&src[..n]);
                    dst[n] = 0;
                }

                patch_dtb_ram_size(&mut self.ram, ptr);
                ptr
            }
        };

        // RV32 Linux boot convention:
        //   a0 = hart ID (0)
        //   a1 = physical address of the DTB
        self.cpu = CpuState::default();
        self.cpu.pc = RAM_BASE;
        self.cpu.regs[10] = 0;
        self.cpu.regs[11] = if dtb_ptr != 0 { dtb_ptr + RAM_BASE } else { 0 };
        self.cpu.set_privilege(3);
        Ok(())
    }

    /// Load an RV32 bare-metal ELF or FreeRTOS image.
    ///
    /// Only `PT_LOAD` segments with `vaddr >= RAM_BASE` are copied into RAM.
    /// Bytes between `p_filesz` and `p_memsz` are zero-filled (BSS). The
    /// symbol table is loaded automatically if the ELF is not stripped.
    ///
    /// On success the CPU is reset with:
    /// - `pc = elf.entry`
    /// - `sp = RAM_BASE + ram_size - 16` (top of RAM, 16-byte aligned per ABI)
    /// - `a0 = a1 = 0` (no DTB — FreeRTOS does not use it)
    /// - privilege = M-mode
    pub fn load_elf(&mut self, elf_file: &str) -> Result<(), i32> {
        let data = std::fs::read(elf_file).map_err(|_| {
            eprintln!("Error: \"{}\" not found", elf_file);
            -5i32
        })?;

        let image = elf::parse_elf(&data).map_err(|e| {
            eprintln!("Error parsing ELF \"{}\": {}", elf_file, e);
            -7i32
        })?;

        // Attempt to load the symbol table; silently skip if the ELF is stripped.
        self.symbols = elf::parse_symbol_table(&data).unwrap_or(None);
        if let Some(ref syms) = self.symbols {
            eprintln!("Symbol table loaded: {} symbols", syms.len());
        }

        self.ram.fill(0);

        for seg in &image.segments {
            if (seg.vaddr as usize) < RAM_BASE as usize {
                eprintln!(
                    "Warning: ELF segment vaddr=0x{:08x} below RAM base — skipped",
                    seg.vaddr
                );
                continue;
            }

            let ofs = (seg.vaddr as usize) - RAM_BASE as usize;
            let end = ofs + seg.data.len();
            let zend = ofs + seg.mem_size as usize;

            if end > self.ram_size as usize {
                eprintln!(
                    "Error: ELF segment [0x{:08x}..+{}] overflows RAM ({} bytes)",
                    seg.vaddr,
                    seg.data.len(),
                    self.ram_size
                );
                return Err(-8);
            }

            self.ram[ofs..end].copy_from_slice(&seg.data);

            // Zero-fill the BSS region (mem_size > file_size).
            if zend > end && zend <= self.ram_size as usize {
                self.ram[end..zend].fill(0);
            }
        }

        self.cpu = CpuState::default();
        self.cpu.pc = image.entry;
        self.cpu.regs[2] = (RAM_BASE + self.ram_size - 16) & !0xf; // sp
        self.cpu.regs[10] = 0; // a0
        self.cpu.regs[11] = 0; // a1
        self.cpu.set_privilege(3);

        eprintln!(
            "ELF loaded: entry=0x{:08x} sp=0x{:08x}",
            self.cpu.pc, self.cpu.regs[2]
        );
        Ok(())
    }

    /// Run the emulator until it stops or the instruction limit is reached.
    ///
    /// Calls [`step`] in a loop, advancing the time base between calls.
    /// Returns the reason the loop exited as a [`StepResult`].
    ///
    /// [`step`]: Emulator::step
    pub fn run(&mut self, cfg: &mut RunConfig, plat: &mut dyn Platform) -> StepResult {
        // trace and single_step both require 1 instruction per flip to capture every step.
        let instrs_per_flip: i32 = if cfg.single_step || cfg.trace.is_some() {
            1
        } else {
            1024
        };

        let mut last_time: u64 = if cfg.fixed_update {
            0
        } else {
            plat.get_time_microseconds() / cfg.time_divisor as u64
        };

        // Previous register state for single-step diff highlighting.
        // Zeroed so the first instruction shows no changes.
        let mut prev_regs: [u32; 32] = [0; 32];

        let mut rt: u64 = 0;
        loop {
            if cfg.instct >= 0 && rt > cfg.instct as u64 {
                break;
            }

            let cycle = self.cpu.get_cycle64();

            let elapsed_us: u32 = if cfg.fixed_update {
                (cycle / cfg.time_divisor as u64).wrapping_sub(last_time) as u32
            } else {
                (plat.get_time_microseconds() / cfg.time_divisor as u64).wrapping_sub(last_time)
                    as u32
            };
            last_time = last_time.wrapping_add(elapsed_us as u64);

            if cfg.single_step {
                // Print state *before* this step, diff against previous step.
                self.dump_state(Some(&prev_regs));
                // Save current state for the next dump.
                prev_regs = self.cpu.regs;
            }

            // Capture PC and regs before the step so the trace line shows
            // what the instruction at trace_pc did.
            let trace_pc = self.cpu.pc;
            if cfg.trace.is_some() {
                prev_regs = self.cpu.regs;
            }

            let step_result = self.step(elapsed_us, instrs_per_flip, plat);

            // Write trace line using the pre-step PC and register snapshot.
            if let Some(ref mut writer) = cfg.trace {
                self.write_trace_line(writer, trace_pc, &prev_regs);
            }

            match step_result {
                StepResult::Ok => {}
                StepResult::Wfi => {
                    if cfg.do_sleep {
                        plat.mini_sleep();
                    }
                    self.cpu.set_cycle64(cycle + instrs_per_flip as u64);
                }
                other => return other,
            }

            rt += instrs_per_flip as u64;
        }
        StepResult::Ok
    }

    /// Execute up to `count` instructions, advancing the timer by `elapsed_us` µs.
    ///
    /// This is the hot path of the emulator. It:
    /// 1. Calls [`CpuState::tick_timer`] to update `mtime` and check MTIP.
    /// 2. Checks for a pending timer interrupt before the instruction loop.
    /// 3. Fetches, decodes, and executes up to `count` instructions.
    /// 4. Commits any trap that fired via [`CpuState::commit_trap`].
    /// 5. Updates the cycle counter.
    ///
    /// Returns [`StepResult::Ok`] in the normal case. Returns
    /// [`StepResult::Wfi`] if the CPU is sleeping. Returns
    /// [`StepResult::Restart`] or [`StepResult::Poweroff`] if the kernel
    /// wrote to SYSCON.
    pub fn step(&mut self, elapsed_us: u32, count: i32, plat: &mut dyn Platform) -> StepResult {
        let ram_size = self.ram_size;

        self.cpu.tick_timer(elapsed_us);

        // Early exit if the CPU is sleeping.
        if self.cpu.get_wfi() {
            return StepResult::Wfi;
        }

        let mut trap = Trap::None;
        let mut rval: u32 = 0;
        let mut pc = self.cpu.pc;
        let cycle_start = self.cpu.cyclel;
        let mut cycle_counter = cycle_start;

        // ── 2. Check for a pending timer interrupt ──────────────────────────────
        if (self.cpu.mip & (1 << 7)) != 0
            && (self.cpu.mie & (1 << 7)) != 0
            && (self.cpu.mstatus & 0x8) != 0
        {
            trap = Trap::IntTimer;
            pc = pc.wrapping_sub(4);
        } else {
            // ── Instruction dispatch loop ────────────────────────────────
            'iloop: for _ in 0..count {
                rval = 0;
                cycle_counter = cycle_counter.wrapping_add(1);

                let ofs_pc = pc.wrapping_sub(RAM_BASE);
                if ofs_pc >= ram_size {
                    trap = Trap::ExcInsnAccessFault;
                    break;
                } else if ofs_pc & 3 != 0 {
                    trap = Trap::ExcInsnMisaligned;
                    break;
                }

                let ir = mmio::mem_load4(&self.ram, ofs_pc);
                let mut rdid = (ir >> 7) & 0x1f;

                match ir & 0x7f {
                    // ── LUI ──────────────────────────────────────────────────
                    0x37 => {
                        rval = decode_imm_u(ir) as u32;
                    }

                    // ── AUIPC ────────────────────────────────────────────────
                    0x17 => {
                        rval = pc.wrapping_add(decode_imm_u(ir) as u32);
                    }

                    // ── JAL ──────────────────────────────────────────────────
                    0x6f => {
                        rval = pc.wrapping_add(4);
                        pc = pc.wrapping_add(decode_imm_j(ir) as u32).wrapping_sub(4);
                    }

                    // ── JALR ─────────────────────────────────────────────────
                    0x67 => {
                        rval = pc.wrapping_add(4);
                        pc = (self.cpu.regs[((ir >> 15) & 0x1f) as usize]
                            .wrapping_add(decode_imm_i(ir) as u32)
                            & !1)
                            .wrapping_sub(4);
                    }

                    // ── Branches (BEQ BNE BLT BGE BLTU BGEU) ─────────────────
                    0x63 => {
                        let rs1 = self.cpu.regs[((ir >> 15) & 0x1f) as usize] as i32;
                        let rs2 = self.cpu.regs[((ir >> 20) & 0x1f) as usize] as i32;
                        let target = pc.wrapping_add(decode_imm_b(ir) as u32).wrapping_sub(4);
                        rdid = 0;
                        let taken = match (ir >> 12) & 0x7 {
                            0 => rs1 == rs2,                   // beq
                            1 => rs1 != rs2,                   // bne
                            4 => rs1 < rs2,                    // blt
                            5 => rs1 >= rs2,                   // bge
                            6 => (rs1 as u32) < (rs2 as u32),  // bltu
                            7 => (rs1 as u32) >= (rs2 as u32), // bgeu
                            _ => {
                                trap = Trap::ExcIllegalInsn;
                                false
                            }
                        };
                        if taken {
                            pc = target;
                        }
                    }

                    // ── Loads (LB LH LW LBU LHU) ─────────────────────────────
                    0x03 => {
                        let addr = self.cpu.regs[((ir >> 15) & 0x1f) as usize]
                            .wrapping_add(decode_imm_i(ir) as u32);
                        let ofs = addr.wrapping_sub(RAM_BASE);
                        if ofs >= ram_size - 3 {
                            if mmio::is_mmio(addr) {
                                rval = mmio::handle_load(&self.cpu, addr, plat);
                            } else {
                                trap = Trap::ExcLoadAccessFault;
                                rval = addr;
                            }
                        } else {
                            rval = match (ir >> 12) & 0x7 {
                                0 => mmio::mem_load1s(&self.ram, ofs), // lb
                                1 => mmio::mem_load2s(&self.ram, ofs), // lh
                                2 => mmio::mem_load4(&self.ram, ofs),  // lw
                                4 => mmio::mem_load1(&self.ram, ofs),  // lbu
                                5 => mmio::mem_load2(&self.ram, ofs),  // lhu
                                _ => {
                                    trap = Trap::ExcIllegalInsn;
                                    0
                                }
                            };
                        }
                    }

                    // ── Stores (SB SH SW) ─────────────────────────────────────
                    0x23 => {
                        let rs1 = self.cpu.regs[((ir >> 15) & 0x1f) as usize];
                        let rs2 = self.cpu.regs[((ir >> 20) & 0x1f) as usize];
                        let addr = rs1.wrapping_add(decode_imm_s(ir) as u32);
                        let ofs = addr.wrapping_sub(RAM_BASE);
                        rdid = 0;
                        if ofs >= ram_size - 3 {
                            if mmio::is_mmio(addr) {
                                let buf = if self.wasm_mode {
                                    Some(&mut self.output_buf)
                                } else {
                                    None
                                };
                                let sr = mmio::handle_store(&mut self.cpu, addr, rs2, buf);
                                if sr != StepResult::Ok {
                                    return sr;
                                }
                            } else {
                                trap = Trap::ExcStoreAccessFault;
                                rval = addr;
                            }
                        } else {
                            match (ir >> 12) & 0x7 {
                                0 => mmio::mem_store1(&mut self.ram, ofs, rs2), // sb
                                1 => mmio::mem_store2(&mut self.ram, ofs, rs2), // sh
                                2 => mmio::mem_store4(&mut self.ram, ofs, rs2), // sw
                                _ => {
                                    trap = Trap::ExcIllegalInsn;
                                }
                            }
                        }
                    }

                    // ── OP-IMM and OP (RV32I + RV32M) ──────────────────────────
                    0x13 | 0x33 => {
                        let imm = decode_imm_i(ir);
                        let rs1 = self.cpu.regs[((ir >> 15) & 0x1f) as usize];
                        let is_reg = (ir & 0x20) != 0;
                        let rs2: u32 = if is_reg {
                            self.cpu.regs[(imm & 0x1f) as usize]
                        } else {
                            imm as u32
                        };

                        if is_reg && (ir & 0x0200_0000) != 0 {
                            // RV32M
                            rval = match (ir >> 12) & 0x7 {
                                0 => rs1.wrapping_mul(rs2),                                      // mul
                                1 => (((rs1 as i32 as i64) * (rs2 as i32 as i64)) >> 32) as u32, // mulh
                                2 => (((rs1 as i32 as i64) * (rs2 as u64 as i64)) >> 32) as u32, // mulhsu
                                3 => (((rs1 as u64) * (rs2 as u64)) >> 32) as u32, // mulhu
                                4 => {
                                    // div
                                    if rs2 == 0 {
                                        u32::MAX
                                    } else if rs1 as i32 == i32::MIN && rs2 as i32 == -1 {
                                        rs1
                                    } else {
                                        ((rs1 as i32) / (rs2 as i32)) as u32
                                    }
                                }
                                5 => {
                                    // divu
                                    if rs2 == 0 {
                                        u32::MAX
                                    } else {
                                        rs1.checked_div(rs2).unwrap()
                                    }
                                }
                                6 => {
                                    // rem
                                    if rs2 == 0 {
                                        rs1
                                    } else if rs1 as i32 == i32::MIN && rs2 as i32 == -1 {
                                        0
                                    } else {
                                        ((rs1 as i32) % (rs2 as i32)) as u32
                                    }
                                }
                                7 => {
                                    // remu
                                    if rs2 == 0 {
                                        rs1
                                    } else {
                                        rs1 % rs2
                                    }
                                }
                                _ => 0,
                            };
                        } else {
                            // RV32I
                            rval = match (ir >> 12) & 0x7 {
                                0 => {
                                    if is_reg && (ir & 0x4000_0000) != 0 {
                                        rs1.wrapping_sub(rs2) // SUB
                                    } else {
                                        rs1.wrapping_add(rs2) // ADD / ADDI
                                    }
                                }
                                1 => rs1 << (rs2 & 0x1f), // SLL / SLLI
                                2 => ((rs1 as i32) < (rs2 as i32)) as u32, // SLT / SLTI
                                3 => (rs1 < rs2) as u32,  // SLTU / SLTIU
                                4 => rs1 ^ rs2,           // XOR / XORI
                                5 => {
                                    if ir & 0x4000_0000 != 0 {
                                        ((rs1 as i32) >> (rs2 & 0x1f)) as u32 // SRA / SRAI
                                    } else {
                                        rs1 >> (rs2 & 0x1f) // SRL / SRLI
                                    }
                                }
                                6 => rs1 | rs2, // OR / ORI
                                7 => rs1 & rs2, // AND / ANDI
                                _ => 0,
                            };
                        }
                    }

                    // ── FENCE — no-op on this emulator (without real cache) ─────────
                    0x0f => {
                        rdid = 0;
                    }

                    // ── SYSTEM (Zicsr + privileged instructions) ─────────────
                    0x73 => {
                        let csrno = ir >> 20;
                        let microop = (ir >> 12) & 0x7;

                        if microop & 3 != 0 {
                            // ── Zicsr ─────────────────────────────────────────
                            let rs1imm = (ir >> 15) & 0x1f;
                            let rs1 = self.cpu.regs[rs1imm as usize];

                            let rval_csr = match csrno {
                                x if x == Csr::Mscratch as u32 => self.cpu.mscratch,
                                x if x == Csr::Mtvec as u32 => self.cpu.mtvec,
                                x if x == Csr::Mie as u32 => self.cpu.mie,
                                x if x == Csr::Cycle as u32 => cycle_counter,
                                x if x == Csr::Mip as u32 => self.cpu.mip,
                                x if x == Csr::Mepc as u32 => self.cpu.mepc,
                                x if x == Csr::Mstatus as u32 => self.cpu.mstatus,
                                x if x == Csr::Mcause as u32 => self.cpu.mcause,
                                x if x == Csr::Mtval as u32 => self.cpu.mtval,
                                x if x == Csr::Mvendorid as u32 => 0xff0f_f0ff,
                                x if x == Csr::Misa as u32 => 0x4040_1101,
                                _ => mmio::handle_csr_read(csrno, plat) as u32,
                            };
                            rval = rval_csr;

                            let writeval = match microop {
                                1 => rs1,
                                2 => rval_csr | rs1,
                                3 => rval_csr & !rs1,
                                5 => rs1imm,
                                6 => rval_csr | rs1imm,
                                7 => rval_csr & !rs1imm,
                                _ => rs1,
                            };

                            match csrno {
                                x if x == Csr::Mscratch as u32 => {
                                    self.cpu.mscratch = writeval;
                                }
                                x if x == Csr::Mtvec as u32 => {
                                    self.cpu.mtvec = writeval;
                                }
                                x if x == Csr::Mie as u32 => {
                                    self.cpu.mie = writeval;
                                }
                                x if x == Csr::Mip as u32 => {
                                    self.cpu.mip = writeval;
                                }
                                x if x == Csr::Mepc as u32 => {
                                    self.cpu.mepc = writeval;
                                }
                                x if x == Csr::Mstatus as u32 => {
                                    self.cpu.mstatus = writeval;
                                }
                                x if x == Csr::Mcause as u32 => {
                                    self.cpu.mcause = writeval;
                                }
                                x if x == Csr::Mtval as u32 => {
                                    self.cpu.mtval = writeval;
                                }
                                _ => {
                                    mmio::handle_csr_write(
                                        &self.ram,
                                        self.ram_size,
                                        csrno,
                                        writeval,
                                    );
                                }
                            }
                        } else if microop == 0 {
                            // ── Privileged instructions ──────────────────────
                            rdid = 0;
                            if (csrno & 0xff) == 0x02 {
                                // MRET — return from trap handler
                                let ms = self.cpu.mstatus;
                                let prev = (ms >> 11) & 3;
                                let cur_priv = self.cpu.get_privilege();
                                // MIE = MPIE, MPIE = 1, MPP = privilege atual
                                self.cpu.mstatus = ((ms & 0x80) >> 4) | (cur_priv << 11) | 0x80;
                                self.cpu.set_privilege(prev);
                                pc = self.cpu.mepc.wrapping_sub(4);
                            } else {
                                match csrno {
                                    0x000 => {
                                        trap = if self.cpu.get_privilege() != 0 {
                                            Trap::ExcEcallM
                                        } else {
                                            Trap::ExcEcallU
                                        };
                                    }
                                    0x001 => {
                                        trap = Trap::ExcBreakpoint;
                                    }
                                    0x105 => {
                                        // WFI — suspend until next interrupt
                                        self.cpu.mstatus |= 8; // enable MIE
                                        self.cpu.set_wfi(true);
                                        if self.cpu.cyclel > cycle_counter {
                                            self.cpu.cycleh = self.cpu.cycleh.wrapping_add(1);
                                        }
                                        self.cpu.cyclel = cycle_counter;
                                        self.cpu.pc = pc.wrapping_add(4);
                                        return StepResult::Wfi;
                                    }
                                    _ => {
                                        trap = Trap::ExcIllegalInsn;
                                    }
                                }
                            }
                        } else {
                            trap = Trap::ExcIllegalInsn;
                        }
                    }

                    // ── RV32A  ──────────────────────────────
                    0x2f => {
                        let rs1 = self.cpu.regs[((ir >> 15) & 0x1f) as usize];
                        let mut rs2 = self.cpu.regs[((ir >> 20) & 0x1f) as usize];
                        let irmid = (ir >> 27) & 0x1f;
                        let ofs = rs1.wrapping_sub(RAM_BASE);

                        if ofs >= ram_size - 3 {
                            trap = Trap::ExcStoreAccessFault;
                            rval = rs1;
                        } else {
                            rval = mmio::mem_load4(&self.ram, ofs);
                            let mut dowrite = true;
                            match irmid {
                                2 => {
                                    // LR.W
                                    dowrite = false;
                                    self.cpu.set_reservation(ofs);
                                }
                                3 => {
                                    // SC.W
                                    rval =
                                        (self.cpu.get_reservation() != (ofs & 0x1fff_ffff)) as u32;
                                    dowrite = rval == 0;
                                }
                                1 => {
                                    // AMOSWAP.W
                                }
                                0 => {
                                    // AMOADD.W
                                    rs2 = rs2.wrapping_add(rval);
                                }
                                4 => {
                                    // AMOXOR.W
                                    rs2 ^= rval;
                                }
                                12 => {
                                    // AMOAND.W
                                    rs2 &= rval;
                                }
                                8 => {
                                    // AMOOR.W
                                    rs2 |= rval;
                                }
                                16 => {
                                    // AMOMIN.W
                                    rs2 = if (rs2 as i32) < (rval as i32) {
                                        rs2
                                    } else {
                                        rval
                                    };
                                }
                                20 => {
                                    // AMOMAX.W
                                    rs2 = if (rs2 as i32) > (rval as i32) {
                                        rs2
                                    } else {
                                        rval
                                    };
                                }
                                24 => {
                                    // AMOMINU.W
                                    rs2 = rs2.min(rval);
                                }
                                28 => {
                                    // AMOMAXU.W
                                    rs2 = rs2.max(rval);
                                }
                                _ => {
                                    trap = Trap::ExcIllegalInsn;
                                    dowrite = false;
                                }
                            }
                            if dowrite {
                                mmio::mem_store4(&mut self.ram, ofs, rs2);
                            }
                        }
                    }

                    _ => {
                        trap = Trap::ExcIllegalInsn;
                    }
                } // end opcode dispatch

                if !trap.is_none() {
                    self.cpu.pc = pc;
                    if self.fail_on_all_faults {
                        eprintln!("FAULT");
                        return StepResult::Fault;
                    }
                    break 'iloop;
                }

                if rdid != 0 {
                    self.cpu.regs[rdid as usize] = rval;
                }
                pc = pc.wrapping_add(4);
            } // end instruction loop
        }

        // ── 4. Commit trap ─────────────────────────────────────────────────────
        if !trap.is_none() {
            pc = self.cpu.commit_trap(trap, rval, pc);
        }

        if self.cpu.cyclel > cycle_counter {
            self.cpu.cycleh = self.cpu.cycleh.wrapping_add(1);
        }
        self.cpu.cyclel = cycle_counter;
        self.cpu.pc = pc;
        StepResult::Ok
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Debug
    // ─────────────────────────────────────────────────────────────────────────

    /// Write one line of compact execution trace to `writer`.
    ///
    /// Format: `<pc>  <ir>  <mnem padded to 36 chars>  [reg=val ...]`
    ///
    /// Only registers that changed relative to `prev_regs` are listed.
    /// Instructions with no register writes (branches, stores) produce a line
    /// that ends after the mnemonic.
    ///
    /// ## Example output
    ///
    /// ```text
    /// 80000014  510010ef  jal     ra, <main>                    ra=80000018
    /// 80001524  ff010113  addi    sp, sp, -16                   sp=81fffff0
    /// 80001528  00112623  sw      ra, 12(sp)
    /// ```
    pub fn write_trace_line(
        &self,
        writer: &mut dyn std::io::Write,
        pc: u32,
        prev_regs: &[u32; 32],
    ) {
        use crate::disasm;

        const NAMES: [&str; 32] = [
            "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3",
            "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
            "t3", "t4", "t5", "t6",
        ];

        let pc_ofs = pc.wrapping_sub(RAM_BASE);

        // Disassemble the instruction at the captured PC.
        let (ir, mnem) = if pc_ofs < self.ram_size - 3 {
            let ir = mmio::mem_load4(&self.ram, pc_ofs);
            let mnem = disasm::disassemble(ir, pc, self.symbols.as_ref());
            (ir, mnem)
        } else {
            (0, "[out of RAM]".to_string())
        };

        // Registers that have changed
        let changes: Vec<String> = (0..32)
            .filter(|&i| self.cpu.regs[i] != prev_regs[i])
            .map(|i| format!("{}={:08x}", NAMES[i], self.cpu.regs[i]))
            .collect();

        // Compact line: pc  ir  mnem  [changed registers...]
        // mnem padded to 36 characters to simplify fixed-column parsing
        let _ = writeln!(
            writer,
            "{:08x}  {:08x}  {:<36}{}",
            pc,
            ir,
            mnem,
            changes.join("  ")
        );
    }

    /// Print the PC, the disassembled instruction, and the full register grid.
    ///
    /// `prev_regs` — register state *before* the last step.
    /// Registers that changed are prefixed with `*`.
    ///
    /// Format:
    /// ```text
    /// 80001234  510010ef  jal     ra, <main>
    ///   zero=00000000 *ra=80000018   sp=83fffff0  gp=00000000
    ///   ...
    /// ```
    pub fn dump_state(&self, prev_regs: Option<&[u32; 32]>) {
        use crate::disasm;

        let pc = self.cpu.pc;
        let pc_ofs = pc.wrapping_sub(RAM_BASE);

        // ── Instruction line ────────────────────────────────────────────────────
        if pc_ofs < self.ram_size - 3 {
            let ir = mmio::mem_load4(&self.ram, pc_ofs);
            let mnem = disasm::disassemble(ir, pc, self.symbols.as_ref());
            eprintln!("{:08x}  {:08x}  {}", pc, ir, mnem);
        } else {
            eprintln!("{:08x}  [out of RAM]", pc);
        }

        // ── Register grid — 4 per row ──────────────────────────────────────────
        // ABI names in the same order as the classic mini-rv32ima dump.
        const NAMES: [&str; 32] = [
            "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0", "a1", "a2", "a3",
            "a4", "a5", "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
            "t3", "t4", "t5", "t6",
        ];

        let r = &self.cpu.regs;
        let prev = prev_regs.unwrap_or(r); // sem prev → nada marcado

        // Each cell has a fixed width: marker (1) + name (4) + "=" (1) + value (8) = 14
        // The longest name is "zero" (4 chars), so {:>4} right-aligns them all.
        // The marker is part of the label: "*ra " or " ra " — always 5 chars total.
        let mut line = String::with_capacity(80);
        for i in 0..32 {
            let changed = r[i] != prev[i];
            // Label = marker + name, left-aligned in a 5-character field:
            //   " zero", "  ra ", "  sp ", " *a3 " etc.
            let label = format!("{}{}", if changed { "*" } else { " " }, NAMES[i]);
            let cell = format!("{:>5}={:08x}", label, r[i]);
            line.push_str(&cell);

            if (i + 1) % 4 == 0 {
                eprintln!("  {}", line.trim_end());
                line.clear();
            } else {
                line.push_str("  ");
            }
        }
    }
}
