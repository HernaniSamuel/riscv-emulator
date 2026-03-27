use crate::cpu::CPU;
use crate::vm::VMError;

pub struct RiscV {
    cpu: CPU,
}

impl RiscV {
    pub fn new(elf_file: ElfImage, ram_length: usize) -> Result<Self, VMError> {
        let riscv = RiscV {
            cpu: CPU::new(elf_file, ram_length)?,
        };

        Ok(riscv)
    }
}

// ChatGPT made this code, I copied manually to understand better
#[derive(Debug)]
pub enum RiscVError {
    NotElf,
    Not32Bit,
    WrongEndian,
    NotRiscV,
    InvalidProgramHeader,
}

const EM_RISCV: u16 = 243;
const PT_LOAD: u32 = 1;

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
    p_addr: u32,
    p_filesz: u32,
    p_memsz: u32,
    p_flags: u32,
    p_align: u32,
}

pub struct ElfSegment {
    pub vaddr: u32,
    pub data: Vec<u8>,
    pub mem_size: u32,
}

pub struct ElfImage {
    pub entry: u32,
    pub segments: Vec<ElfSegment>,
}

pub fn read_elf(data: &[u8]) -> Result<ElfImage, RiscVError> {
    let header = unsafe { &*(data.as_ptr() as *const Elf32Header) };

    // 1. ELF signature
    if &header.e_ident[0..4] != b"\x7FELF" {
        return Err(RiscVError::NotElf);
    }

    // 2. 32-bit
    if header.e_ident[5] != 1 {
        return Err(RiscVError::Not32Bit);
    }

    // 3. little endian
    if header.e_ident[5] != 1 {
        return Err(RiscVError::WrongEndian);
    }

    // 4. RISC-V
    if header.e_machine != EM_RISCV {
        return Err(RiscVError::NotRiscV);
    }

    let mut segments = Vec::new();

    let phoff = header.e_phoff as usize;
    let phnum = header.e_phnum as usize;

    let program_headers = unsafe {
        std::slice::from_raw_parts(data[phoff..].as_ptr() as *const Elf32ProgramHeader, phnum)
    };

    for ph in program_headers {
        if ph.p_type == PT_LOAD {
            let start = ph.p_offset as usize;
            let end = start + ph.p_filesz as usize;

            let segment_data = data[start..end].to_vec();

            segments.push(ElfSegment {
                vaddr: ph.p_vaddr,
                data: segment_data,
                mem_size: ph.p_memsz,
            });
        }
    }

    Ok(ElfImage {
        entry: header.e_entry,
        segments,
    })
}
