use std::env;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args().nth(1).expect("usage: emulator <elf>");

    let data = fs::read(path)?;
    let elf = read_elf(&data).expect("invalid ELF");

    let mut cpu = CPU::new(MEM_SIZE);

    // stack pointer (igual seu)
    cpu.regs[2] = (RAM_BASE as u32) + (MEM_SIZE as u32) - 4;

    // load program
    cpu.load_elf(&elf).expect("failed to load elf");

    // set PC (caso load_elf não faça)
    cpu.pc = elf.entry;

    println!("Starting emulation at 0x{:08x}", cpu.pc);

    // main loop
    cpu.running = true;
    while cpu.running {
        cpu.tick();
        cpu.step();
    }
    println!("Exit code: {}", cpu.exit_code);

    Ok(())
}

pub const MSTATUS: usize = 0x300;
pub const MIE_CSR: usize = 0x304; // machine interrupt-enable register
pub const MTVEC: usize = 0x305;
pub const MEPC: usize = 0x341;
pub const MCAUSE: usize = 0x342;
pub const MTIME: usize = 0xC01;
pub const MTIMECMP: usize = 0xC80;

pub const MEM_SIZE: usize = 32 * 1024 * 1024; // 32MB, suficiente pro FreeRTOS

pub const CLINT_MTIME: u32 = 0x0200_BFF8;
pub const CLINT_MTIME_PLUS_4: u32 = CLINT_MTIME + 4;
pub const CLINT_MTIMECMP: u32 = 0x0200_4000;
pub const CLINT_MTIMECMP_PLUS_4: u32 = CLINT_MTIMECMP + 4;
pub const CLINT_BASE: u32 = 0x0200_0000;
pub const CLINT_END: u32 = 0x0201_0000;

pub const RAM_BASE: usize = 0x8000_0000;

pub const UART_BASE: u32 = 0x1000_0000;
pub const UART_SIZE: u32 = 0x1000;
pub const UART_TX: u32 = UART_BASE;
pub const UART_RX: u32 = UART_BASE;
pub const UART_LSR: u32 = UART_BASE + 5;
pub const TX_READY: u8 = 0x20;

// Bits de mstatus
pub const MSTATUS_MIE: u32 = 1 << 3; // Machine Interrupt Enable
pub const MSTATUS_MPIE: u32 = 1 << 7; // Machine Previous Interrupt Enable
pub const MSTATUS_MPP_MASK: u32 = 0b11 << 11;

// Bits de mie (0x304)
pub const MIE_MTIE: u32 = 1 << 7; // Machine Timer Interrupt Enable

fn is_uart(addr: u32) -> bool {
    (UART_BASE..UART_BASE + UART_SIZE).contains(&addr)
}

fn is_clint(addr: u32) -> bool {
    (CLINT_BASE..CLINT_END).contains(&addr)
}

fn is_ram(addr: u32, mem_size: usize) -> bool {
    let start = RAM_BASE as u32;
    let end = start + mem_size as u32;
    addr >= start && addr < end
}

fn to_ram_index(addr: u32) -> usize {
    (addr as usize).wrapping_sub(RAM_BASE)
}

#[inline(always)]
fn sign_extend(value: u32, bits: u32) -> i32 {
    ((value << (32 - bits)) as i32) >> (32 - bits)
}

fn imm_i(inst: u32) -> i32 {
    sign_extend(inst >> 20, 12)
}

fn imm_s(inst: u32) -> i32 {
    let v = ((inst >> 25) << 5) | ((inst >> 7) & 0x1f);
    sign_extend(v, 12)
}

fn imm_b(inst: u32) -> i32 {
    let v = ((inst >> 31) << 12)
        | (((inst >> 7) & 1) << 11)
        | (((inst >> 25) & 0x3f) << 5)
        | (((inst >> 8) & 0xf) << 1);

    sign_extend(v, 13)
}

fn imm_u(inst: u32) -> i32 {
    (inst & 0xffff_f000) as i32
}

fn imm_j(inst: u32) -> i32 {
    let v = ((inst >> 31) << 20)
        | (((inst >> 12) & 0xff) << 12)
        | (((inst >> 20) & 1) << 11)
        | (((inst >> 21) & 0x3ff) << 1);

    sign_extend(v, 21)
}
fn shamt(inst: u32) -> u32 {
    (inst >> 20) & 0x1f
}

pub struct CPU {
    pub regs: [u32; 32],
    pub pc: u32,
    pub memory: Vec<u8>,

    pub running: bool,
    pub exit_code: i32,
    pub timer_pending: bool,

    pub csr: [u32; 4096],
    pub mtime: u64,
    pub mtimecmp: u64,

    pub trap_count: u64,
    pub step_log: u64,
}

impl CPU {
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
            trap_count: 0,
            step_log: 0,
        }
    }

    fn clint_read(&self, addr: u32) -> u32 {
        match addr {
            CLINT_MTIME => self.mtime as u32,
            CLINT_MTIME_PLUS_4 => (self.mtime >> 32) as u32,
            CLINT_MTIMECMP => self.mtimecmp as u32,
            CLINT_MTIMECMP_PLUS_4 => (self.mtimecmp >> 32) as u32,
            _ => 0,
        }
    }

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
                // Só limpa pending se o novo mtimecmp está no futuro
                if self.mtimecmp > self.mtime {
                    self.timer_pending = false;
                }
            }
            CLINT_MTIMECMP_PLUS_4 => {
                self.mtimecmp = (self.mtimecmp & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32);
                if self.mtimecmp > self.mtime {
                    // mtimecmp ainda tem lo=0xFFFFFFFF aqui!
                    self.timer_pending = false;
                }
            }

            _ => {}
        }
    }

    pub fn csr_read(&self, addr: u16) -> u32 {
        self.csr[addr as usize]
    }

    pub fn csr_write(&mut self, addr: u16, val: u32) {
        self.csr[addr as usize] = val;
    }

    pub fn trap(&mut self, cause: u32) {
        let mtvec = self.csr_read(MTVEC as u16);
        let handler = mtvec & !0b11;

        if handler == 0 {
            println!("mtvec = 0, trap ignorado");
            return;
        }

        if self.timer_pending {
            let mstatus = self.csr_read(MSTATUS as u16);
            let mie = self.csr_read(MIE_CSR as u16);
            if (mstatus & MSTATUS_MIE) != 0 && (mie & MIE_MTIE) != 0 {
                self.timer_pending = false;

                self.trap(0x8000_0007);
            }
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

    pub fn load_elf(&mut self, elf: &ElfImage) -> Result<(), RiscVError> {
        self.pc = elf.entry;
        for seg in &elf.segments {
            println!(
                "segmento: vaddr=0x{:08X} filesz={} memsz={}",
                seg.vaddr,
                seg.data.len(),
                seg.mem_size
            );

            if (seg.vaddr as usize) < RAM_BASE {
                println!("  -> ignorado (fora da RAM)");
                continue;
            }

            let start = to_ram_index(seg.vaddr);
            let end = start + seg.data.len();

            if end > self.memory.len() {
                println!(
                    "  -> fora dos limites! start={} end={} mem={}",
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

    fn uart_read(&self, addr: u32) -> u32 {
        match addr {
            UART_LSR => TX_READY as u32,
            UART_RX => 0,
            _ => 0,
        }
    }

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
    pub fn step(&mut self) {
        if self.pc == 0 || self.pc == 0xFFFFFFFF {
            panic!("PC inválido: 0x{:08x}", self.pc);
        }

        let inst = self.read_u32(self.pc);

        let opcode = inst & 0x7f;
        let rd = ((inst >> 7) & 0x1f) as usize;
        let funct3 = (inst >> 12) & 7;
        let rs1 = ((inst >> 15) & 0x1f) as usize;
        let rs2 = ((inst >> 20) & 0x1f) as usize;
        let funct7 = (inst >> 25) & 0x7f;

        let imm_i = imm_i(inst);
        let imm_s = imm_s(inst);
        let imm_b = imm_b(inst);
        let imm_u = imm_u(inst);
        let imm_j = imm_j(inst);
        let shamt = shamt(inst);

        match opcode {
            0b0110011 => match (funct3, funct7) {
                // ADD
                (0x0, 0x00) => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1].wrapping_add(self.regs[rs2]);
                    }
                }

                // SUB
                (0x0, 0x20) => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1].wrapping_sub(self.regs[rs2]);
                    }
                }

                // SLL
                (0x1, 0x00) => {
                    if rd != 0 {
                        let shamt = self.regs[rs2] & 0x1f;
                        self.regs[rd] = self.regs[rs1] << shamt;
                    }
                }

                // SLT
                (0x2, 0x00) => {
                    if rd != 0 {
                        let val = (self.regs[rs1] as i32) < (self.regs[rs2] as i32);
                        self.regs[rd] = val as u32;
                    }
                }

                // SLTU
                (0x3, 0x00) => {
                    if rd != 0 {
                        self.regs[rd] = (self.regs[rs1] < self.regs[rs2]) as u32;
                    }
                }

                // XOR
                (0x4, 0x00) => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] ^ self.regs[rs2];
                    }
                }

                // SRL
                (0x5, 0x00) => {
                    if rd != 0 {
                        let shamt = self.regs[rs2] & 0x1f;
                        self.regs[rd] = self.regs[rs1] >> shamt;
                    }
                }

                // SRA
                (0x5, 0x20) => {
                    if rd != 0 {
                        let shamt = self.regs[rs2] & 0x1f;
                        self.regs[rd] = ((self.regs[rs1] as i32) >> shamt) as u32;
                    }
                }

                // OR
                (0x6, 0x00) => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] | self.regs[rs2];
                    }
                }

                // AND
                (0x7, 0x00) => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] & self.regs[rs2];
                    }
                }
                // MUL
                (0x0, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as i32 as i64;
                        let b = self.regs[rs2] as i32 as i64;
                        self.regs[rd] = (a.wrapping_mul(b) as u64) as u32;
                    }
                }

                // MULH
                (0x1, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as i32 as i64;
                        let b = self.regs[rs2] as i32 as i64;
                        self.regs[rd] = ((a.wrapping_mul(b) >> 32) as u64) as u32;
                    }
                }

                // MULHSU
                (0x2, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as i32 as i64;
                        let b = self.regs[rs2] as u64 as i64;
                        self.regs[rd] = ((a.wrapping_mul(b) >> 32) as u64) as u32;
                    }
                }

                // MULHU
                (0x3, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as u64;
                        let b = self.regs[rs2] as u64;
                        self.regs[rd] = (a.wrapping_mul(b) >> 32) as u32;
                    }
                }

                // DIV
                (0x4, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as i32;
                        let b = self.regs[rs2] as i32;

                        let val = if b == 0 {
                            u32::MAX
                        } else if a == i32::MIN && b == -1 {
                            a as u32
                        } else {
                            (a / b) as u32
                        };

                        self.regs[rd] = val;
                    }
                }

                // DIVU
                (0x5, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1];
                        let b = self.regs[rs2];

                        self.regs[rd] = a.checked_div(b).unwrap_or(u32::MAX);
                    }
                }

                // REM
                (0x6, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1] as i32;
                        let b = self.regs[rs2] as i32;

                        let val = if b == 0 {
                            a as u32
                        } else if a == i32::MIN && b == -1 {
                            0
                        } else {
                            (a % b) as u32
                        };

                        self.regs[rd] = val;
                    }
                }

                // REMU
                (0x7, 0x01) => {
                    if rd != 0 {
                        let a = self.regs[rs1];
                        let b = self.regs[rs2];

                        self.regs[rd] = if b == 0 { a } else { a % b };
                    }
                }

                _ => panic!("Invalid R instruction"),
            },

            0b0010011 => match funct3 {
                // ADDI
                0x0 => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1].wrapping_add(imm_i as u32);
                    }
                }

                // SLTI
                0x2 => {
                    if rd != 0 {
                        let val = (self.regs[rs1] as i32) < imm_i;
                        self.regs[rd] = val as u32;
                    }
                }

                // SLTIU
                0x3 => {
                    if rd != 0 {
                        self.regs[rd] = (self.regs[rs1] < imm_i as u32) as u32;
                    }
                }

                // XORI
                0x4 => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] ^ (imm_i as u32);
                    }
                }

                // ORI
                0x6 => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] | (imm_i as u32);
                    }
                }

                // ANDI
                0x7 => {
                    if rd != 0 {
                        self.regs[rd] = self.regs[rs1] & (imm_i as u32);
                    }
                }

                // ================= SHIFT (caso especial) =================
                0x1 => {
                    if rd != 0 {
                        if funct7 != 0x00 {
                            panic!("illegal SLLI");
                        }
                        self.regs[rd] = self.regs[rs1] << (shamt & 0x1f);
                    }
                }

                0x5 => {
                    if rd != 0 {
                        match funct7 {
                            // SRLI
                            0x00 => {
                                self.regs[rd] = self.regs[rs1] >> (shamt & 0x1f);
                            }

                            // SRAI
                            0x20 => {
                                self.regs[rd] = ((self.regs[rs1] as i32) >> (shamt & 0x1f)) as u32;
                            }

                            _ => panic!("illegal shift"),
                        }
                    }
                }

                _ => panic!("invalid I-type"),
            },

            0b0000011 => match funct3 {
                // LB (signed byte)
                0x0 => {
                    if rd != 0 {
                        let addr = self.regs[rs1].wrapping_add(imm_i as u32);
                        let val = self.read_u8(addr) as i8 as i32 as u32;
                        self.regs[rd] = val;
                    }
                }

                // LH (signed halfword)
                0x1 => {
                    if rd != 0 {
                        let addr = self.regs[rs1].wrapping_add(imm_i as u32);
                        let val = self.read_u16(addr) as i16 as i32 as u32;
                        self.regs[rd] = val;
                    }
                }

                // LW (word)
                0x2 => {
                    if rd != 0 {
                        let addr = self.regs[rs1].wrapping_add(imm_i as u32);
                        self.regs[rd] = self.read_u32(addr);
                    }
                }

                // LBU (unsigned byte)
                0x4 => {
                    if rd != 0 {
                        let addr = self.regs[rs1].wrapping_add(imm_i as u32);
                        self.regs[rd] = self.read_u8(addr) as u32;
                    }
                }

                // LHU (unsigned halfword)
                0x5 => {
                    if rd != 0 {
                        let addr = self.regs[rs1].wrapping_add(imm_i as u32);
                        self.regs[rd] = self.read_u16(addr) as u32;
                    }
                }

                _ => panic!("invalid LOAD instruction"),
            },

            0b0100011 => match funct3 {
                // SB (store byte)
                0x0 => {
                    let addr = self.regs[rs1].wrapping_add(imm_s as u32);
                    self.write_u8(addr, self.regs[rs2] as u8);
                }

                // SH (store halfword)
                0x1 => {
                    let addr = self.regs[rs1].wrapping_add(imm_s as u32);
                    self.write_u16(addr, self.regs[rs2] as u16);
                }

                // SW (store word)
                0x2 => {
                    let addr = self.regs[rs1].wrapping_add(imm_s as u32);
                    self.write_u32(addr, self.regs[rs2]);
                }

                _ => panic!("invalid STORE instruction"),
            },

            0b1100011 => {
                match funct3 {
                    // BEQ
                    0x0 => {
                        if self.regs[rs1] == self.regs[rs2] {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }

                    // BNE
                    0x1 => {
                        if self.regs[rs1] != self.regs[rs2] {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }

                    // BLT (signed)
                    0x4 => {
                        if (self.regs[rs1] as i32) < (self.regs[rs2] as i32) {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }
                    // BGE (signed)
                    0x5 => {
                        if (self.regs[rs1] as i32) >= (self.regs[rs2] as i32) {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }

                    // BLTU (unsigned)
                    0x6 => {
                        if self.regs[rs1] < self.regs[rs2] {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }

                    // BGEU (unsigned)
                    0x7 => {
                        if self.regs[rs1] >= self.regs[rs2] {
                            self.pc = (self.pc as i32).wrapping_add(imm_b) as u32;
                            return;
                        }
                    }

                    _ => panic!("invalid BRANCH"),
                }
            }

            // ================= U =================

            // LUI
            0b0110111 => {
                if rd != 0 {
                    self.regs[rd] = imm_u as u32;
                }
            }

            // AUIPC
            0b0010111 => {
                if rd != 0 {
                    self.regs[rd] = self.pc.wrapping_add(imm_u as u32);
                }
            }

            // ================= J =================

            // JAL
            0b1101111 => {
                if rd != 0 {
                    self.regs[rd] = self.pc.wrapping_add(4);
                }
                self.pc = self.pc.wrapping_add(imm_j as u32);
                return;
            }

            // JALR
            0b1100111 => {
                if funct3 == 0x0 {
                    if rd != 0 {
                        self.regs[rd] = self.pc.wrapping_add(4);
                    }

                    let base = self.regs[rs1];
                    self.pc = base.wrapping_add(imm_i as u32) & !1;

                    return;
                }

                panic!("invalid JALR");
            }

            // ================= FENCE =================
            0b0001111 => match funct3 {
                0x0 | 0x1 => (),
                _ => panic!("invalid fence"),
            },

            0b1110011 => {
                let csr = ((inst >> 20) & 0xfff) as u16;
                let zimm = (inst >> 15) & 0x1f;

                match funct3 {
                    // =========================================================
                    // SYSTEM
                    // =========================================================
                    0x0 => match csr {
                        // ECALL
                        0x000 => {
                            let syscall = self.regs[17];

                            match syscall {
                                0 => {
                                    self.trap(11);
                                    return;
                                }

                                93 => {
                                    self.exit_code = self.regs[10] as i32;
                                    self.running = false;
                                    return;
                                }

                                _ => {
                                    panic!(
                                        "unsupported syscall={} a0={} a1={} a2={} pc=0x{:08x}",
                                        self.regs[17],
                                        self.regs[10],
                                        self.regs[11],
                                        self.regs[12],
                                        self.pc
                                    );
                                }
                            }
                        }

                        // EBREAK
                        0x001 => {
                            return;
                        }

                        // MRET
                        0x302 => {
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

                            self.pc = mepc;
                            return;
                        }

                        // WFI
                        0x105 => {
                            if self.mtimecmp > self.mtime {
                                self.mtime = self.mtimecmp.saturating_sub(1);
                            }
                            self.pc = self.pc.wrapping_add(4);
                            return;
                        }

                        _ => panic!("invalid system instruction"),
                    },

                    // =========================================================
                    // CSRRW
                    // =========================================================
                    0x1 => {
                        let old = self.csr_read(csr);
                        let val = self.regs[rs1];

                        self.csr_write(csr, val);

                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    // =========================================================
                    // CSRRS
                    // =========================================================
                    0x2 => {
                        let old = self.csr_read(csr);
                        let val = self.regs[rs1];

                        if val != 0 {
                            self.csr_write(csr, old | val);
                        }

                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    // =========================================================
                    // CSRRC
                    // =========================================================
                    0x3 => {
                        let old = self.csr_read(csr);
                        let val = self.regs[rs1];

                        if val != 0 {
                            self.csr_write(csr, old & !val);
                        }

                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    // =========================================================
                    // CSRRWI
                    // =========================================================
                    0x5 => {
                        let old = self.csr_read(csr);

                        self.csr_write(csr, zimm);

                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    // =========================================================
                    // CSRRSI
                    // =========================================================
                    0x6 => {
                        let old = self.csr_read(csr);

                        if zimm != 0 {
                            self.csr_write(csr, old | zimm);
                        }

                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    // =========================================================
                    // CSRRCI
                    // =========================================================
                    0x7 => {
                        let old = self.csr_read(csr);
                        if zimm != 0 {
                            let novo = old & !zimm;
                            self.csr_write(csr, novo);
                        }
                        if rd != 0 {
                            self.regs[rd] = old;
                        }
                    }

                    _ => panic!("invalid CSR instruction"),
                }
            }

            _ => panic!(
                "opcode invalido=0x{:02x} inst=0x{:08x} pc=0x{:08x}",
                opcode, inst, self.pc
            ),
        }
        self.pc = self.pc.wrapping_add(4);
    }
}

// ================== ELF STUFF ===================================
#[derive(Debug)]
pub enum RiscVError {
    NotElf,
    Not32Bit,
    WrongEndian,
    NotRiscV,
    InvalidProgramHeader,
}

pub const EM_RISCV: u16 = 243;
pub const PT_LOAD: u32 = 1;

#[repr(C)]
#[derive(Debug)]
struct Elf32Header {
    e_ident: [u8; 16],
    e_type: u16,
    e_machine: u16,
    e_version: u32,
    e_entry: u32,
    e_phoff: u32,
    e_shoff: u32,
    e_flags: u32,
    e_ehsize: u16,
    e_phentsize: u16,
    e_phnum: u16,
    e_shentsize: u16,
    e_shnum: u16,
    e_shstrndx: u16,
}

#[repr(C)]
#[derive(Debug)]
struct Elf32ProgramHeader {
    p_type: u32,
    p_offset: u32,
    p_vaddr: u32,
    p_paddr: u32,
    p_filesz: u32,
    p_memsz: u32,
    p_flags: u32,
    p_align: u32,
}

#[derive(Debug, Clone)]
pub struct ElfSegment {
    pub vaddr: u32,
    pub data: Vec<u8>,
    pub mem_size: u32,
}

#[derive(Debug, Clone)]
pub struct ElfImage {
    pub entry: u32,
    pub segments: Vec<ElfSegment>,
}

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
