use crate::cpu::CPU;
use crate::vm::VMError;

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

    // 1. Assinatura ELF
    if &header.e_ident[0..4] != b"\x7FELF" {
        return Err(RiscVError::NotElf);
    }

    // 2. 32-bit — e_ident[4] == 1 (ELFCLASS32)
    //    BUG original: usava e_ident[5] (campo errado — era o de endianness)
    if header.e_ident[4] != 1 {
        return Err(RiscVError::Not32Bit);
    }

    // 3. Little-endian — e_ident[6] == 1 (ELFDATA2LSB)
    //    BUG original: usava e_ident[5] de novo → check duplicado, nunca detectava big-endian
    if header.e_ident[6] != 1 {
        return Err(RiscVError::WrongEndian);
    }

    // 4. Arquitetura RISC-V
    if header.e_machine != EM_RISCV {
        return Err(RiscVError::NotRiscV);
    }

    let phoff = header.e_phoff as usize;
    let phnum = header.e_phnum as usize;
    let phentsize = header.e_phentsize as usize;

    // Valida que os program headers cabem no buffer (overflow-safe)
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

// ---------------------------------------------------------------------------

pub struct RiscV {
    pub cpu: CPU,
}

impl RiscV {
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, VMError> {
        Ok(RiscV {
            cpu: CPU::new(elf_file, ram_length_kb)?,
        })
    }
}
