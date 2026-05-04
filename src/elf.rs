//! ELF32 parser and symbol table loader.
//!
//! Parses RISC-V 32-bit little-endian ELF files and extracts two things:
//!
//! 1. **Loadable segments** ([`ElfImage`]) — `PT_LOAD` program headers copied
//!    into RAM by [`crate::emulator::Emulator::load_elf`].
//! 2. **Symbol table** ([`SymbolTable`]) — `STT_FUNC` and `STT_OBJECT` symbols
//!    used by the disassembler to resolve addresses to names.
//!
//! ## Usage
//!
//! ```ignore
//! let data  = std::fs::read("freertos.elf")?;
//! let image = elf::parse_elf(&data)?;
//! let syms  = elf::parse_symbol_table(&data)?; // None if stripped
//! ```
//!
//! ## Symbol lookup
//!
//! [`SymbolTable`] supports two lookup directions:
//!
//! - **Address → symbol**: [`SymbolTable::lookup_addr`] — used by the
//!   disassembler to annotate branch targets and call destinations.
//! - **Name → symbol**: [`SymbolTable::lookup_name`] — reserved for future
//!   breakpoint-by-name support.
//!
//! Symbols with `size == 0` (common in hand-written assembly) only match on
//! an exact address. Symbols with a known `size` match any address inside
//! `[sym.addr, sym.addr + sym.size)`.

#[derive(Debug)]
pub enum ElfError {
    /// File is too short to contain an ELF header.
    TooShort,
    /// Magic bytes (`\x7fELF`) do not match.
    BadMagic,
    /// ELF class is not 32-bit.
    Not32Bit,
    /// ELF data encoding is not little-endian.
    WrongEndian,
    /// `e_machine` is not `EM_RISCV` (243).
    NotRiscV,
    /// A program header is out of bounds or overflows.
    InvalidProgramHeader,
    /// A section header is out of bounds or overflows.
    InvalidSectionHeader,
}

impl std::fmt::Display for ElfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElfError::TooShort => write!(f, "file too short to be ELF"),
            ElfError::BadMagic => write!(f, "bad ELF magic bytes"),
            ElfError::Not32Bit => write!(f, "not a 32-bit ELF (EI_CLASS != 1)"),
            ElfError::WrongEndian => write!(f, "not a little-endian ELF (EI_DATA != 1)"),
            ElfError::NotRiscV => write!(f, "not a RISC-V ELF (e_machine != 243)"),
            ElfError::InvalidProgramHeader => write!(f, "invalid or out-of-bounds program header"),
            ElfError::InvalidSectionHeader => write!(f, "invalid or out-of-bounds section header"),
        }
    }
}

impl std::error::Error for ElfError {}

// ── ELF constants ────────────────────────────────────────────────────────────

const EM_RISCV: u16 = 243;
const PT_LOAD: u32 = 1;
const SHT_SYMTAB: u32 = 2; // symbol table section

const STT_OBJECT: u8 = 1; // data / global variable symbol
const STT_FUNC: u8 = 2; // function symbol

const STB_LOCAL: u8 = 0; // local symbol
const STB_GLOBAL: u8 = 1; // global symbol
const STB_WEAK: u8 = 2; // weak symbol

// ── ELF32 on-disk structures (repr(C) for zero-copy parsing) ─────────────────

#[repr(C)]
struct Elf32Ehdr {
    e_ident: [u8; 16],
    e_type: u16,
    e_machine: u16,
    e_version: u32,
    e_entry: u32,
    e_phoff: u32, // offset da program header table
    e_shoff: u32, // offset da section header table
    _e_flags: u32,
    _e_ehsize: u16,
    e_phentsize: u16,
    e_phnum: u16,
    e_shentsize: u16,
    e_shnum: u16,
    e_shstrndx: u16, // index of the section name string table
}

#[repr(C)]
struct Elf32Phdr {
    p_type: u32,
    p_offset: u32,
    p_vaddr: u32,
    _p_paddr: u32,
    p_filesz: u32,
    p_memsz: u32,
    _p_flags: u32,
    _p_align: u32,
}

#[repr(C)]
struct Elf32Shdr {
    sh_name: u32, // offset do nome na string table
    sh_type: u32,
    _sh_flags: u32,
    _sh_addr: u32,
    sh_offset: u32, // offset no arquivo
    sh_size: u32,
    sh_link: u32, // for SHT_SYMTAB: index of the associated string table
    _sh_info: u32,
    _sh_addralign: u32,
    sh_entsize: u32, // tamanho de cada entrada (para tabelas)
}

/// One entry in the ELF32 symbol table (`Elf32_Sym`).
#[repr(C)]
struct Elf32Sym {
    st_name: u32,  // offset do nome na string table
    st_value: u32, // symbol virtual address
    st_size: u32,  // tamanho em bytes (0 se desconhecido)
    st_info: u8,   // type (bits 3:0) + binding (bits 7:4)
    _st_other: u8,
    _st_shndx: u16,
}

impl Elf32Sym {
    fn stt(&self) -> u8 {
        self.st_info & 0xf
    } // tipo
    fn stb(&self) -> u8 {
        (self.st_info >> 4) & 0xf
    } // binding
}

// ── Public types — segments ──────────────────────────────────────────────────

/// A loadable segment (`PT_LOAD`) from an ELF file.
#[derive(Debug, Clone)]
pub struct ElfSegment {
    /// Virtual address where this segment should be loaded.
    pub vaddr: u32,
    /// Raw bytes from the ELF file (`p_filesz` bytes).
    pub data: Vec<u8>,
    /// Total size in memory (`p_memsz`). Bytes beyond `data.len()` are zero-filled (BSS).
    pub mem_size: u32,
}

/// A parsed ELF image ready to be loaded into emulator RAM.
#[derive(Debug, Clone)]
pub struct ElfImage {
    /// Entry point virtual address (`e_entry`).
    pub entry: u32,
    /// All `PT_LOAD` segments, in file order.
    pub segments: Vec<ElfSegment>,
}

// ── Public types — symbol table ──────────────────────────────────────────────

/// The kind of an ELF symbol, derived from `st_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SymbolKind {
    /// Function symbol (`STT_FUNC`).
    Func,
    /// Data / global variable symbol (`STT_OBJECT`).
    Object,
}

/// A resolved symbol from the `.symtab` section.
#[derive(Debug, Clone)]
pub struct Symbol {
    /// Symbol name (e.g. `"vTaskStartScheduler"`).
    pub name: String,
    /// Virtual address.
    pub addr: u32,
    /// Size in bytes. `0` means unknown — common for hand-written assembly.
    pub size: u32,
    /// Whether this is a function or a data object.
    pub kind: SymbolKind,
}

/// Symbol table extracted from an ELF `.symtab` section.
///
/// Internally maintained as two sorted indices for O(log n) lookup in both
/// directions:
/// - `by_addr` — sorted by address, used by [`lookup_addr`].
/// - `by_name` — sorted by name, used by [`lookup_name`].
///
/// [`lookup_addr`]: SymbolTable::lookup_addr
/// [`lookup_name`]: SymbolTable::lookup_name
#[derive(Debug, Default)]
pub struct SymbolTable {
    by_addr: Vec<Symbol>,
    by_name: Vec<(String, usize)>, // (name, index into by_addr)
}

impl SymbolTable {
    /// Returns `true` if the table contains no symbols.
    pub fn is_empty(&self) -> bool {
        self.by_addr.is_empty()
    }

    /// Number of symbols in the table.
    pub fn len(&self) -> usize {
        self.by_addr.len()
    }

    /// Find the symbol that contains `addr`.
    ///
    /// Returns the symbol whose range `[sym.addr, sym.addr + sym.size)` covers
    /// `addr`. If `sym.size == 0`, only an exact address match is accepted.
    /// Returns `None` if no symbol covers the address.
    pub fn lookup_addr(&self, addr: u32) -> Option<&Symbol> {
        // Binary search: count = number of symbols with addr <= target.
        // If none, there is no candidate below addr.
        let count = self.by_addr.partition_point(|s| s.addr <= addr);
        if count == 0 {
            return None;
        }

        let sym = &self.by_addr[count - 1];

        if sym.addr == addr {
            return Some(sym); // exact match
        }

        // Range match: only valid when size is known and addr falls within it.
        if sym.size > 0 && addr < sym.addr.saturating_add(sym.size) {
            Some(sym)
        } else {
            None
        }
    }

    /// Find a symbol by its exact name.
    ///
    /// Returns `None` if no symbol with that name exists.
    pub fn lookup_name(&self, name: &str) -> Option<&Symbol> {
        let idx = self
            .by_name
            .binary_search_by_key(&name, |(n, _)| n.as_str())
            .ok()?;
        let sym_idx = self.by_name[idx].1;
        Some(&self.by_addr[sym_idx])
    }

    /// Iterate over all symbols in ascending address order.
    pub fn iter(&self) -> impl Iterator<Item = &Symbol> {
        self.by_addr.iter()
    }

    fn build(mut symbols: Vec<Symbol>) -> Self {
        // Sort by address for binary search in lookup_addr.
        symbols.sort_by_key(|s| s.addr);

        // Build name index for binary search in lookup_name.
        let mut by_name: Vec<(String, usize)> = symbols
            .iter()
            .enumerate()
            .map(|(i, s)| (s.name.clone(), i))
            .collect();
        by_name.sort_by(|a, b| a.0.cmp(&b.0));

        SymbolTable {
            by_addr: symbols,
            by_name,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers internos de parse
/// Reinterpret a byte slice as a slice of `repr(C)` structs.
///
/// # Safety
/// The caller must ensure `offset + count * size_of::<T>() <= data.len()`
/// and that the alignment requirements of `T` are satisfied. ELF32 structs
/// are guaranteed to meet both conditions when parsed from a valid ELF file.
unsafe fn read_slice<T>(data: &[u8], offset: usize, count: usize) -> &[T] {
    std::slice::from_raw_parts(data[offset..].as_ptr() as *const T, count)
}

/// Read a null-terminated UTF-8 string from a string table at `offset`.
fn read_cstr(strtab: &[u8], offset: usize) -> &str {
    let slice = &strtab[offset..];
    let len = slice.iter().position(|&b| b == 0).unwrap_or(slice.len());
    std::str::from_utf8(&slice[..len]).unwrap_or("<invalid utf8>")
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Parse an ELF32 RV32 little-endian file and return its loadable segments.
///
/// Only `PT_LOAD` program headers are extracted. Section headers are ignored.
pub fn parse_elf(data: &[u8]) -> Result<ElfImage, ElfError> {
    let hdr = parse_header(data)?;

    let segments = parse_segments(data, hdr)?;

    Ok(ElfImage {
        entry: hdr.e_entry,
        segments,
    })
}

/// Parse the `.symtab` section of an ELF32 RV32 file.
///
/// Extracts `STT_FUNC` and `STT_OBJECT` symbols with `global`, `weak`, or
/// `local` binding. Linker-internal names (starting with `.` or `$`) are
/// filtered out.
///
/// Returns `Ok(None)` if the ELF has no `.symtab` (i.e. it was stripped).
/// Returns `Err` only if the file is not a valid ELF32 RV32 binary.
pub fn parse_symbol_table(data: &[u8]) -> Result<Option<SymbolTable>, ElfError> {
    let hdr = parse_header(data)?;

    let shoff = hdr.e_shoff as usize;
    let shnum = hdr.e_shnum as usize;
    let shentsize = hdr.e_shentsize as usize;

    // No section headers — the ELF is stripped or is a relocatable object.
    if shoff == 0 || shnum == 0 {
        return Ok(None);
    }

    let sh_end = shoff
        .checked_add(
            shnum
                .checked_mul(shentsize)
                .ok_or(ElfError::InvalidSectionHeader)?,
        )
        .ok_or(ElfError::InvalidSectionHeader)?;

    if sh_end > data.len() {
        return Err(ElfError::InvalidSectionHeader);
    }

    // SAFETY: bounds verificados acima.
    let shdrs: &[Elf32Shdr] = unsafe { read_slice(data, shoff, shnum) };

    // Find the SHT_SYMTAB section.
    let symtab_shdr = shdrs.iter().find(|s| s.sh_type == SHT_SYMTAB);
    let symtab_shdr = match symtab_shdr {
        Some(s) => s,
        None => return Ok(None), // ELF stripped
    };

    // The associated string table is pointed to by sh_link.
    let strtab_idx = symtab_shdr.sh_link as usize;
    if strtab_idx >= shnum {
        return Err(ElfError::InvalidSectionHeader);
    }
    let strtab_shdr = &shdrs[strtab_idx];

    // Validate and extract the section data.
    let sym_offset = symtab_shdr.sh_offset as usize;
    let sym_size = symtab_shdr.sh_size as usize;
    let sym_entsize = symtab_shdr.sh_entsize as usize;

    if sym_entsize == 0 {
        return Err(ElfError::InvalidSectionHeader);
    }

    let sym_end = sym_offset
        .checked_add(sym_size)
        .ok_or(ElfError::InvalidSectionHeader)?;
    if sym_end > data.len() {
        return Err(ElfError::InvalidSectionHeader);
    }

    let str_offset = strtab_shdr.sh_offset as usize;
    let str_end = str_offset
        .checked_add(strtab_shdr.sh_size as usize)
        .ok_or(ElfError::InvalidSectionHeader)?;
    if str_end > data.len() {
        return Err(ElfError::InvalidSectionHeader);
    }

    let strtab = &data[str_offset..str_end];

    let sym_count = sym_size / sym_entsize;

    // SAFETY: bounds verificados acima.
    let raw_syms: &[Elf32Sym] = unsafe { read_slice(data, sym_offset, sym_count) };

    let mut symbols = Vec::new();
    for sym in raw_syms {
        // Filtra por tipo
        let kind = match sym.stt() {
            STT_FUNC => SymbolKind::Func,
            STT_OBJECT => SymbolKind::Object,
            _ => continue,
        };

        // Include global, weak, and local bindings. Local functions like
        // uart_putc are useful for the disassembler.
        match sym.stb() {
            STB_GLOBAL | STB_WEAK | STB_LOCAL => {}
            _ => continue,
        }

        // Skip undefined symbols (address 0).
        if sym.st_value == 0 {
            continue;
        }

        let name_offset = sym.st_name as usize;
        if name_offset >= strtab.len() {
            continue;
        }

        let name = read_cstr(strtab, name_offset).to_owned();
        if name.is_empty() {
            continue;
        }

        // Filter out linker/compiler noise:
        //   .Lxxx — local assembler labels
        //   $x    — ARM/RISC-V mapping symbols
        if name.starts_with('.') || name.starts_with('$') {
            continue;
        }

        symbols.push(Symbol {
            name,
            addr: sym.st_value,
            size: sym.st_size,
            kind,
        });
    }

    if symbols.is_empty() {
        return Ok(None);
    }

    Ok(Some(SymbolTable::build(symbols)))
}

// ── Internal helpers ─────────────────────────────────────────────────────────

/// Validate the ELF header and return a typed reference into the data slice.
fn parse_header(data: &[u8]) -> Result<&Elf32Ehdr, ElfError> {
    if data.len() < std::mem::size_of::<Elf32Ehdr>() {
        return Err(ElfError::TooShort);
    }

    // SAFETY: tamanho verificado acima.
    let hdr = unsafe { &*(data.as_ptr() as *const Elf32Ehdr) };

    if &hdr.e_ident[0..4] != b"\x7fELF" {
        return Err(ElfError::BadMagic);
    }
    if hdr.e_ident[4] != 1 {
        return Err(ElfError::Not32Bit);
    }
    if hdr.e_ident[5] != 1 {
        return Err(ElfError::WrongEndian);
    }
    if hdr.e_machine != EM_RISCV {
        return Err(ElfError::NotRiscV);
    }

    Ok(hdr)
}

/// Extract all `PT_LOAD` segments from the ELF file.
fn parse_segments(data: &[u8], hdr: &Elf32Ehdr) -> Result<Vec<ElfSegment>, ElfError> {
    let phoff = hdr.e_phoff as usize;
    let phnum = hdr.e_phnum as usize;
    let phentsize = hdr.e_phentsize as usize;

    let ph_end = phoff
        .checked_add(
            phnum
                .checked_mul(phentsize)
                .ok_or(ElfError::InvalidProgramHeader)?,
        )
        .ok_or(ElfError::InvalidProgramHeader)?;

    if ph_end > data.len() {
        return Err(ElfError::InvalidProgramHeader);
    }

    // SAFETY: bounds verificados acima.
    let phdrs: &[Elf32Phdr] = unsafe { read_slice(data, phoff, phnum) };

    let mut segments = Vec::new();
    for ph in phdrs {
        if ph.p_type != PT_LOAD {
            continue;
        }

        let start = ph.p_offset as usize;
        let end = start
            .checked_add(ph.p_filesz as usize)
            .ok_or(ElfError::InvalidProgramHeader)?;

        if end > data.len() {
            return Err(ElfError::InvalidProgramHeader);
        }

        segments.push(ElfSegment {
            vaddr: ph.p_vaddr,
            data: data[start..end].to_vec(),
            mem_size: ph.p_memsz,
        });
    }

    Ok(segments)
}
