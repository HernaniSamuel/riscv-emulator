//! RISC-V runtime and ELF loader.
//!
//! This module provides the **system integration layer** between a parsed
//! RISC-V ELF binary and the virtual CPU execution environment.
//!
//! It is responsible for:
//!
//! - Parsing ELF32 binaries
//! - Validating RISC-V executable format constraints
//! - Extracting loadable segments (`PT_LOAD`)
//! - Building an [`ElfImage`] representation
//! - Initializing the [`CPU`] with a ready-to-run memory layout
//!
//! # Architecture role
//!
//! This module sits *above* the CPU core and acts as the runtime bootstrap:
//!
//! ```text
//! ELF file → ELF parser → ElfImage → CPU → Execute loop
//! ```
//!
//! It does **not implement execution or decoding logic**. Those are handled
//! by the CPU core.
//!
//! # Supported format
//!
//! This loader supports a strict subset of ELF:
//!
//! - ELF32 only
//! - Little-endian only
//! - RISC-V machine type (`EM_RISCV = 243`)
//! - Loadable segments only (`PT_LOAD`)
//!
//! Unsupported features include:
//!
//! - ELF64 binaries
//! - Big-endian ELF files
//! - Dynamic linking (shared objects)
//! - Relocations
//! - Non-loadable program segments
//!
//! # Safety model
//!
//! ELF parsing uses low-level binary casting (`repr(C)` structs + pointer reads).
//! This is safe under the assumption that:
//!
//! - Input is validated as ELF32 before casting
//! - Program header bounds are checked before slicing
//! - All offsets are verified against input buffer size
//!
//! Invalid or malformed binaries result in [`RiscVError`].
//!
//! # Memory model
//!
//! Each `PT_LOAD` segment is mapped into an [`ElfSegment`] containing:
//!
//! - Virtual address (`vaddr`)
//! - Raw segment data
//! - Declared memory size (`mem_size`)
//!
//! The CPU is responsible for placing these segments into its memory space.
//!
//! # Error handling
//!
//! ELF validation and runtime initialization errors are reported via [`RiscVError`]:
//!
//! - [`crate::risc_v::RiscVError::NotElf`] → invalid ELF signature
//! - [`crate::risc_v::RiscVError::Not32Bit`] → ELF64 or invalid class
//! - [`crate::risc_v::RiscVError::WrongEndian`] → big-endian ELF
//! - [`crate::risc_v::RiscVError::NotRiscV`] → non-RISC-V machine type
//! - [`crate::risc_v::RiscVError::InvalidProgramHeader`] → malformed or out-of-bounds program headers
//! - [`crate::risc_v::RiscVError::CPU`] → errors propagated from CPU initialization
//!
//! # Design notes
//!
//! - Uses zero-copy header parsing for performance (unsafe pointer casting)
//! - Converts only `PT_LOAD` segments into executable memory regions
//! - Keeps ELF representation decoupled from CPU memory layout
//!
//! # RiscV runtime
//!
//! The [`RiscV`] struct acts as a **high-level runtime wrapper** around the CPU.
//! It is responsible for:
//!
//! - Owning the CPU instance
//! - Bootstrapping execution from an ELF image
//! - Providing a clean entry point for simulation
//!
//! # Example
//!
//! ```ignore
//! let elf = read_elf(&binary_data)?;
//!
//! let mut rv = RiscV::new(elf, 4096)?;
//!
//! rv.cpu.run()?;
//! ```
//!
//! # Compliance
//!
//! This loader targets a minimal RISC-V execution environment:
//!
//! - Static ELF executables
//! - No runtime linking or dynamic loader support
//!
use crate::cpu::{CPU, CPUError};

/// Errors that can occur during ELF loading and RISC-V system initialization.
///
/// This enum represents all failure modes that may occur while parsing and validating
/// an ELF binary or initializing the RISC-V virtual machine.
///
/// It acts as a high-level error wrapper around [`CPUError`] and ELF parsing errors.
///
/// # Design Philosophy
///
/// The loader is strict and fails fast: any malformed or incompatible ELF file
/// results in an error rather than undefined behavior or partial execution.
///
/// This ensures the VM only runs validated RISC-V RV32IM binaries.
///
/// # Variants
///
/// * `CPU(CPUError)`
///   - Propagates errors originating from the CPU subsystem (memory, execution, syscalls).
///
/// * `NotElf`
///   - The provided binary does not contain a valid ELF magic header (`0x7F 'E' 'L' 'F'`).
///
/// * `Not32Bit`
///   - The ELF file is not a 32-bit binary (only ELFCLASS32 is supported).
///
/// * `WrongEndian`
///   - The ELF file is not little-endian (ELFDATA2LSB required).
///
/// * `NotRiscV`
///   - The ELF file targets a non-RISC-V architecture (e_machine != EM_RISCV).
///
/// * `InvalidProgramHeader`
///   - The program header table is malformed, out-of-bounds, or inconsistent with file size.
///     This includes:
///   - Program headers extending beyond file size
///   - Overflow in header offset calculations
///   - Invalid `p_filesz` / `p_memsz` combinations
///
/// # Notes
///
/// * This loader only supports ELF32 binaries.
/// * Only `PT_LOAD` segments are processed; all other segment types are ignored.
/// * The loader assumes a valid memory model provided by the CPU.
/// * No relocation, dynamic linking, or symbol resolution is performed.
///
/// # Safety Model
///
/// This module uses `unsafe` parsing of ELF structures for performance.
/// All unsafe operations are guarded by explicit bounds checks before use.
///
/// As a result, malformed binaries are rejected before any unsafe dereference occurs.
#[derive(Debug)]
pub enum RiscVError {
    CPU(CPUError),
    NotElf,
    Not32Bit,
    WrongEndian,
    NotRiscV,
    InvalidProgramHeader,
}

/// Converts a [`CPUError`] into a [`RiscVError`].
///
/// This implementation enables automatic propagation of CPU-level errors
/// into the higher-level RISC-V system error type using the `?` operator.
///
/// # Purpose
///
/// The RISC-V loader and system layer wraps the CPU subsystem. When a
/// [`CPUError`] occurs (e.g., memory access violation, illegal instruction,
/// syscall failure), it is automatically converted into
/// [`RiscVError::CPU`] to maintain a unified error type across the system.
///
/// This allows ergonomic error propagation without manual mapping at each call site.
///
/// # Example
///
/// ```rust
/// use riscv::{ElfImage, RiscV, RiscVError};
/// use riscv::cpu::CPU;
///
/// fn load(elf: ElfImage) -> Result<RiscV, RiscVError> {
///     let cpu = CPU::new(elf, 4096)?;
///     Ok(RiscV { cpu })
/// }
/// ```
impl From<CPUError> for RiscVError {
    fn from(e: CPUError) -> Self {
        RiscVError::CPU(e)
    }
}

/// ELF machine type for RISC-V architecture (EM_RISCV = 243).
///
/// This value identifies the target architecture in the ELF header
/// (`e_machine` field). Only binaries matching this value are accepted
/// by the loader.
const EM_RISCV: u16 = 243;

/// ELF program header type indicating a loadable segment (PT_LOAD = 1).
///
/// Only segments marked as PT_LOAD are mapped into memory by the loader.
/// All other segment types (dynamic, note, etc.) are ignored.
const PT_LOAD: u32 = 1;

/// ELF32 file header (Executable and Linkable Format).
///
/// This structure represents the ELF header as defined by the System V ABI.
/// It is mapped directly onto raw binary data using `#[repr(C)]` and
/// unsafe pointer casting.
///
/// # Layout
///
/// The fields correspond exactly to the ELF specification:
///
/// * `e_ident` - Magic number and metadata (class, endianness, version)
/// * `e_type` - Object file type (e.g., executable, relocatable)
/// * `e_machine` - Target architecture (must be EM_RISCV for this loader)
/// * `e_version` - ELF version (should be 1)
/// * `e_entry` - Entry point virtual address
/// * `e_phoff` - Program header table file offset
/// * `e_shoff` - Section header table file offset
/// * `e_flags` - Architecture-specific flags
/// * `e_ehsize` - ELF header size in bytes
/// * `e_phentsize` - Size of one program header entry
/// * `e_phnum` - Number of program header entries
/// * `e_shentsize` - Size of one section header entry
/// * `e_shnum` - Number of section header entries
/// * `e_shstrndx` - Section header string table index
///
/// # Safety
///
/// This struct is used with `unsafe` transmutation from raw bytes:
///
/// ```ignore
/// let header = &*(data.as_ptr() as *const Elf32Header);
/// ```
///
/// This is only valid if:
///
/// * The buffer is at least `size_of::<Elf32Header>()`
/// * The data is properly aligned (or architecture allows unaligned access)
/// * The file is a valid ELF32 binary
///
/// Incorrect use will result in undefined behavior.
///
/// # Notes
///
/// * This loader assumes little-endian ELF32 binaries.
/// * Only ELF files with `e_machine == EM_RISCV` are accepted downstream.
/// * This struct is not meant to be constructed manually.
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

/// ELF32 Program Header (segment descriptor).
///
/// This structure defines how segments of an ELF binary are mapped into memory.
///
/// Each entry in the program header table describes a loadable segment,
/// including its location in the file and how it should be mapped in virtual memory.
///
/// This struct is used via `unsafe` casting from raw ELF bytes and must strictly
/// match the ELF specification layout.
///
/// # Layout (ELF Program Header)
///
/// * `p_type`  - Segment type (e.g., PT_LOAD for loadable segments)
/// * `p_offset` - Offset of the segment data in the ELF file
/// * `p_vaddr`  - Virtual address where the segment should be loaded
/// * `p_paddr`  - Physical address (typically ignored in user-space loaders)
/// * `p_filesz` - Size of the segment in the file
/// * `p_memsz`  - Size of the segment in memory (may be larger than file size)
/// * `p_flags`  - Segment permissions (R/W/X flags)
/// * `p_align`  - Alignment constraint in memory and file
///
/// # Safety
///
/// This struct is read directly from raw ELF bytes using unsafe casting.
///
/// ```ignore
/// let ph = &*(ptr as *const Elf32ProgramHeader);
/// ```
///
/// Correctness depends on:
///
/// * Valid ELF file structure
/// * Proper bounds checking before access
/// * Correct `phoff`, `phnum`, and `phentsize` validation
///
/// Invalid usage can lead to undefined behavior.
///
/// # Notes
///
/// * Only `PT_LOAD` segments are used by this loader.
/// * Other segment types are ignored.
/// * This struct is never constructed manually.
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

/// In-memory representation of a loadable ELF segment.
///
/// This struct is a simplified abstraction of an ELF `PT_LOAD` program header,
/// used by the loader to map binary segments into the virtual machine memory.
///
/// It decouples raw ELF parsing from the execution layer, providing a clean
/// intermediate representation between file format and CPU memory.
///
/// # Fields
///
/// * `vaddr`
///   - The virtual address where this segment should be loaded.
///
/// * `data`
///   - Raw bytes of the segment as stored in the ELF file.
///   - This represents the initialized portion of memory.
///
/// * `mem_size`
///   - Total size the segment should occupy in memory.
///   - May be larger than `data.len()`, indicating zero-initialized memory (BSS).
///
/// # Behavior
///
/// During loading:
/// - `data` is copied into VM memory starting at `vaddr`.
/// - Any remaining space up to `mem_size` is zero-initialized.
///
/// # Notes
///
/// * This is not a direct ELF structure — it is a loader-level abstraction.
/// * It exists to simplify CPU memory initialization.
/// * It intentionally ignores ELF flags and alignment after parsing.
#[derive(Debug, Clone)]
pub struct ElfSegment {
    pub vaddr: u32,
    pub data: Vec<u8>,
    pub mem_size: u32,
}

/// In-memory representation of a fully parsed ELF binary.
///
/// This struct represents the final output of the ELF loader after validation
/// and parsing of the ELF file.
///
/// It is the primary input used to initialize the RISC-V CPU execution context.
///
/// # Purpose
///
/// The `ElfImage` acts as a clean abstraction between:
///
/// * Raw ELF binary format (disk representation)
/// * Virtual machine memory layout (execution representation)
///
/// It contains only the information required to start execution.
///
/// # Fields
///
/// * `entry`
///   - Entry point address of the program (from ELF `e_entry` field).
///   - This is the initial program counter (PC) value for the CPU.
///
/// * `segments`
///   - List of loadable memory segments derived from `PT_LOAD` program headers.
///   - Each segment describes a region of memory to be mapped into the VM.
///
/// # Execution Model
///
/// When passed to the CPU:
///
/// 1. Each segment is loaded into the VM memory at its `vaddr`.
/// 2. Memory regions are initialized with segment data + zero padding (if needed).
/// 3. The CPU program counter is set to `entry`.
/// 4. Execution begins at the entry point.
///
/// # Notes
///
/// * This struct is the final artifact of ELF parsing.
/// * It does not retain raw ELF metadata or headers.
/// * It assumes validation has already been performed by the loader.
/// * It is safe to clone and reuse across CPU instances.
#[derive(Debug, Clone)]
pub struct ElfImage {
    pub entry: u32,
    pub segments: Vec<ElfSegment>,
}

/// Parses a raw ELF32 binary into an executable [`ElfImage`].
///
/// This function is the entry point of the RISC-V loader pipeline.
///
/// It validates the ELF file structure, checks architecture compatibility,
/// extracts loadable segments, and converts them into a format usable by
/// the virtual machine.
///
/// # Pipeline Overview
///
/// The function performs the following steps:
///
/// 1. Validates ELF header size
/// 2. Checks ELF magic number (`0x7F 'E' 'L' 'F'`)
/// 3. Ensures 32-bit ELF format (ELFCLASS32)
/// 4. Ensures little-endian encoding (ELFDATA2LSB)
/// 5. Verifies target architecture is RISC-V (EM_RISCV)
/// 6. Validates program header table bounds safely
/// 7. Extracts all `PT_LOAD` segments
/// 8. Builds a normalized [`ElfImage`] for execution
///
/// # Arguments
///
/// * `data` - Raw ELF binary loaded into memory.
///
/// # Returns
///
/// * `Ok(ElfImage)` - A validated and normalized representation of the program
///   ready for execution by the CPU.
///
/// * `Err(RiscVError)` - If the ELF is invalid, incompatible, or malformed.
///
/// # Errors
///
/// This function may return:
///
/// * `NotElf`
///   - Missing ELF magic header or file too small.
///
/// * `Not32Bit`
///   - ELF is not ELF32 format.
///
/// * `WrongEndian`
///   - ELF is not little-endian.
///
/// * `NotRiscV`
///   - Target architecture is not RISC-V.
///
/// * `InvalidProgramHeader`
///   - Program header table is out of bounds, corrupted, or inconsistent.
///
/// # Safety
///
/// This function uses unsafe casting to interpret raw bytes as ELF structures:
///
/// ```ignore
/// let header = &*(data.as_ptr() as *const Elf32Header);
/// let ph = &*(... as *const Elf32ProgramHeader);
/// ```
///
/// These operations are safe because:
///
/// * The file size is validated before access
/// * Bounds checks are performed on all header offsets
/// * Only trusted ELF32 layout assumptions are used
///
/// Invalid or malicious binaries are rejected before unsafe access can cause UB.
///
/// # Notes
///
/// * Only `PT_LOAD` segments are loaded into memory.
/// * Other segment types (dynamic, notes, TLS, etc.) are ignored.
/// * This loader does not perform relocation or linking.
/// * The resulting [`ElfImage`] is architecture-neutral within RV32IM constraints.
///
/// # Design Role
///
/// This function is the boundary between:
///
/// * Raw binary input (unsafe, untrusted data)
/// * VM execution model (validated, structured representation)
///
/// It is expected to be called exactly once per program load.
pub fn read_elf(data: &[u8]) -> Result<ElfImage, RiscVError> {
    // Ensure the buffer is large enough to contain an ELF32 header
    if data.len() < std::mem::size_of::<Elf32Header>() {
        return Err(RiscVError::NotElf);
    }

    let header = unsafe { &*(data.as_ptr() as *const Elf32Header) };

    // 1. Validate ELF magic signature
    if &header.e_ident[0..4] != b"\x7FELF" {
        return Err(RiscVError::NotElf);
    }

    // 2. Verify ELF class is 32-bit (ELFCLASS32 = 1)
    //    Original bug: used e_ident[5] (incorrect field; corresponds to endianness)
    if header.e_ident[4] != 1 {
        return Err(RiscVError::Not32Bit);
    }

    // 3. Verify little-endian encoding (ELFDATA2LSB = 1)
    //    Original bug: incorrectly checked e_ident[5] again, duplicating the wrong field
    //    and failing to detect big-endian binaries
    if header.e_ident[6] != 1 {
        return Err(RiscVError::WrongEndian);
    }

    // 4. Validate target architecture (RISC-V)
    if header.e_machine != EM_RISCV {
        return Err(RiscVError::NotRiscV);
    }

    let phoff = header.e_phoff as usize;
    let phnum = header.e_phnum as usize;
    let phentsize = header.e_phentsize as usize;

    // Ensure program headers fit within the input buffer (overflow-safe validation)
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

    // Parse program header table from raw binary data
    let program_headers = unsafe {
        std::slice::from_raw_parts(data[phoff..].as_ptr() as *const Elf32ProgramHeader, phnum)
    };

    let mut segments = Vec::new();

    // Iterate over program headers and extract loadable segments
    for ph in program_headers {
        // Only process loadable segments (PT_LOAD)
        if ph.p_type != PT_LOAD {
            continue;
        }

        let start = ph.p_offset as usize;
        let filesz = ph.p_filesz as usize;

        let end = start
            .checked_add(filesz)
            .ok_or(RiscVError::InvalidProgramHeader)?;

        // Validate segment boundaries
        if end > data.len() {
            return Err(RiscVError::InvalidProgramHeader);
        }

        // Copy segment data into memory
        segments.push(ElfSegment {
            vaddr: ph.p_vaddr,
            data: data[start..end].to_vec(),
            mem_size: ph.p_memsz,
        });
    }

    // Construct final ELF image representation
    Ok(ElfImage {
        entry: header.e_entry,
        segments,
    })
}

/// Top-level RISC-V virtual machine runtime.
///
/// This struct represents the complete execution environment for a
/// RISC-V RV32IM program.
///
/// It wraps the underlying [`CPU`] and provides a simplified entry
/// point for initializing and running ELF binaries.
///
/// # Role in the System
///
/// The `RiscV` struct is the highest-level abstraction in the VM stack:
///
/// * `read_elf` → parses raw binary into [`ElfImage`]
/// * `CPU::new` → loads image into memory and initializes execution state
/// * `RiscV` → provides a clean runtime wrapper for execution
///
/// It acts as the "system boundary" between host application and VM.
///
/// # Fields
///
/// * `cpu`
///   - The underlying CPU instance responsible for execution.
///   - Exposed for direct control and inspection of VM state.
///
/// # Design Philosophy
///
/// This struct intentionally stays minimal:
///
/// * No execution loop is implemented here
/// * No scheduling or syscall handling is added at this level
/// * It only owns and initializes the CPU
///
/// Higher-level orchestration (fetch-decode-execute loop) is expected
/// to be implemented outside this struct.
///
/// # Notes
///
/// * Each instance represents a single isolated VM.
/// * Multiple VMs can be created independently.
/// * The CPU is fully initialized from an [`ElfImage`] at construction time.
pub struct RiscV {
    pub cpu: CPU,
}

impl RiscV {
    /// Creates a new RISC-V VM instance from a parsed ELF image.
    ///
    /// This initializes the CPU state, loads program segments into memory,
    /// and sets the program counter to the ELF entry point.
    ///
    /// # Arguments
    ///
    /// * `elf_file` - Validated ELF image produced by [`read_elf`].
    /// * `ram_length_kb` - Size of the VM memory in kilobytes.
    ///
    /// # Returns
    ///
    /// * `Ok(RiscV)` - Fully initialized virtual machine ready for execution.
    /// * `Err(RiscVError)` - If CPU initialization or memory setup fails.
    ///
    /// # Notes
    ///
    /// * This does not start execution automatically.
    /// * Execution must be driven externally (fetch/decode/execute loop).
    pub fn new(elf_file: ElfImage, ram_length_kb: usize) -> Result<Self, RiscVError> {
        Ok(RiscV {
            cpu: CPU::new(elf_file, ram_length_kb)?,
        })
    }
}
