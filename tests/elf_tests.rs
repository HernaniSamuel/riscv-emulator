//! Integration tests for read_elf + VM::new
//!
//! Place this file in `tests/elf_tests.rs` or include it as a
//! test module inside any file using `#[cfg(test)]`.
//!
//! Run with: cargo test

#[cfg(test)]
mod tests {
    use riscv::{
        risc_v::{RiscVError, read_elf},
        vm::{VM, VMError},
    };

    // -----------------------------------------------------------------------
    // Minimal valid ELF builder for RV32I (little-endian, 32-bit)
    // Generates an ELF with a single PT_LOAD segment containing `payload`.
    // -----------------------------------------------------------------------
    fn build_elf(entry: u32, load_vaddr: u32, payload: &[u8]) -> Vec<u8> {
        // Fixed sizes of an ELF32
        const ELF_HEADER_SIZE: u32 = 52;
        const PHDR_SIZE: u32 = 32;
        const PHDR_COUNT: u16 = 1;

        // The segment starts right after the ELF header + program header
        let segment_offset: u32 = ELF_HEADER_SIZE + PHDR_SIZE;
        let filesz: u32 = payload.len() as u32;
        // memsz may be larger (BSS) — we keep it equal for simplicity
        let memsz: u32 = filesz;

        let mut elf = Vec::new();

        // --- ELF Header (52 bytes) ---
        // e_ident (16 bytes)
        elf.extend_from_slice(b"\x7FELF"); // magic
        elf.push(1); // EI_CLASS:   ELFCLASS32
        elf.push(1); // EI_DATA:    ELFDATA2LSB (little-endian)
        elf.push(1); // EI_VERSION: EV_CURRENT
        elf.push(0); // EI_OSABI:   ELFOSABI_NONE
        elf.extend_from_slice(&[0u8; 8]); // EI_ABIVERSION + padding

        elf.extend_from_slice(&2u16.to_le_bytes()); // e_type:      ET_EXEC
        elf.extend_from_slice(&243u16.to_le_bytes()); // e_machine:   EM_RISCV
        elf.extend_from_slice(&1u32.to_le_bytes()); // e_version:   EV_CURRENT
        elf.extend_from_slice(&entry.to_le_bytes()); // e_entry
        elf.extend_from_slice(&ELF_HEADER_SIZE.to_le_bytes()); // e_phoff
        elf.extend_from_slice(&0u32.to_le_bytes()); // e_shoff
        elf.extend_from_slice(&0u32.to_le_bytes()); // e_flags
        elf.extend_from_slice(&(ELF_HEADER_SIZE as u16).to_le_bytes()); // e_ehsize
        elf.extend_from_slice(&(PHDR_SIZE as u16).to_le_bytes()); // e_phentsize
        elf.extend_from_slice(&PHDR_COUNT.to_le_bytes()); // e_phnum
        elf.extend_from_slice(&64u16.to_le_bytes()); // e_shentsize
        elf.extend_from_slice(&0u16.to_le_bytes()); // e_shnum
        elf.extend_from_slice(&0u16.to_le_bytes()); // e_shstrndx

        // --- Program Header (32 bytes) ---
        elf.extend_from_slice(&1u32.to_le_bytes()); // p_type:   PT_LOAD
        elf.extend_from_slice(&segment_offset.to_le_bytes()); // p_offset
        elf.extend_from_slice(&load_vaddr.to_le_bytes()); // p_vaddr
        elf.extend_from_slice(&load_vaddr.to_le_bytes()); // p_paddr
        elf.extend_from_slice(&filesz.to_le_bytes()); // p_filesz
        elf.extend_from_slice(&memsz.to_le_bytes()); // p_memsz
        elf.extend_from_slice(&5u32.to_le_bytes()); // p_flags: PF_R | PF_X
        elf.extend_from_slice(&0x1000u32.to_le_bytes()); // p_align

        // --- Payload ("code") ---
        elf.extend_from_slice(payload);

        elf
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// RV32I NOP instruction: ADDI x0, x0, 0  →  0x00000013
    fn nop() -> [u8; 4] {
        0x00000013u32.to_le_bytes()
    }

    // -----------------------------------------------------------------------
    // read_elf tests — ELF parsing
    // -----------------------------------------------------------------------

    #[test]
    fn valid_elf_opens_without_error() {
        let payload: Vec<u8> = nop().repeat(4); // 4 NOPs
        let data = build_elf(0x1000, 0x1000, &payload);
        let result = read_elf(&data);
        assert!(
            result.is_ok(),
            "Valid ELF should open without error: {:?}",
            result
        );
    }

    #[test]
    fn entry_point_read_correctly() {
        let entry = 0x20004000u32;
        let data = build_elf(entry, 0x2000_0000, &nop());
        let image = read_elf(&data).unwrap();
        assert_eq!(
            image.entry, entry,
            "entry point must match e_entry from header"
        );
    }

    #[test]
    fn segment_loaded_with_correct_data() {
        let payload = vec![0xAA, 0xBB, 0xCC, 0xDD];
        let data = build_elf(0x1000, 0x1000, &payload);
        let image = read_elf(&data).unwrap();

        assert_eq!(
            image.segments.len(),
            1,
            "must have exactly 1 PT_LOAD segment"
        );
        assert_eq!(image.segments[0].vaddr, 0x1000);
        assert_eq!(
            image.segments[0].data, payload,
            "segment bytes must match payload"
        );
        assert_eq!(image.segments[0].mem_size, payload.len() as u32);
    }

    #[test]
    fn elf_without_magic_returns_not_elf() {
        let mut data = build_elf(0x1000, 0x1000, &nop());
        data[0] = 0x00; // corrupt magic
        let err = read_elf(&data).unwrap_err();
        assert!(matches!(err, RiscVError::NotElf));
    }

    #[test]
    fn elf_64bit_returns_not_32bit() {
        let mut data = build_elf(0x1000, 0x1000, &nop());
        data[4] = 2; // EI_CLASS = ELFCLASS64
        let err = read_elf(&data).unwrap_err();
        assert!(matches!(err, RiscVError::Not32Bit));
    }

    #[test]
    fn elf_big_endian_returns_wrong_endian() {
        let mut data = build_elf(0x1000, 0x1000, &nop());
        data[6] = 2; // EI_DATA = ELFDATA2MSB
        let err = read_elf(&data).unwrap_err();
        assert!(matches!(err, RiscVError::WrongEndian));
    }

    #[test]
    fn elf_wrong_architecture_returns_not_riscv() {
        let mut data = build_elf(0x1000, 0x1000, &nop());
        let em_x86_64: u16 = 62;
        let bytes = em_x86_64.to_le_bytes();
        data[18] = bytes[0];
        data[19] = bytes[1];
        let err = read_elf(&data).unwrap_err();
        assert!(matches!(err, RiscVError::NotRiscV));
    }

    #[test]
    fn empty_elf_returns_not_elf() {
        let err = read_elf(&[]).unwrap_err();
        assert!(matches!(err, RiscVError::NotElf));
    }

    // -----------------------------------------------------------------------
    // VM::new tests — RAM loading
    // -----------------------------------------------------------------------

    #[test]
    fn vm_loads_segment_at_correct_vaddr() {
        let payload = vec![0x13, 0x00, 0x00, 0x00]; // NOP
        let vaddr = 0x1000u32;
        let data = build_elf(vaddr, vaddr, &payload);
        let image = read_elf(&data).unwrap();

        let vm = VM::new(image, 64).expect("VM should be created without error");

        for (i, byte) in payload.iter().enumerate() {
            assert_eq!(
                vm.read_u8(vaddr + i as u32).unwrap(),
                *byte,
                "segment bytes must be at the correct vaddr in RAM"
            );
        }
    }

    #[test]
    fn vm_pc_starts_at_entry_point() {
        let entry = 0x1000u32;
        let data = build_elf(entry, entry, &nop());
        let image = read_elf(&data).unwrap();
        let vm = VM::new(image, 64).unwrap();

        assert_eq!(
            vm.get_pc(),
            entry,
            "PC must start exactly at ELF entry point"
        );
    }

    #[test]
    fn vm_segment_larger_than_ram_returns_error() {
        let payload = vec![0u8; 1024 * 1024];
        let data = build_elf(0x1000, 0x1000, &payload);
        let image = read_elf(&data).unwrap();
        let err = VM::new(image, 64).unwrap_err();
        assert_eq!(err, VMError::ELFTooLarge);
    }

    #[test]
    fn vm_bss_zeroed_after_filesz() {
        let filesz = 4usize;
        let memsz = 16u32;
        let vaddr = 0x1000u32;
        let payload = vec![0xAA; filesz];

        let mut data = build_elf(vaddr, vaddr, &payload);

        let memsz_offset = 52 + 20;
        data[memsz_offset..memsz_offset + 4].copy_from_slice(&memsz.to_le_bytes());

        let image = read_elf(&data).unwrap();
        assert_eq!(image.segments[0].mem_size, memsz);

        let vm = VM::new(image, 64).unwrap();

        // payload
        for i in 0..filesz {
            assert_eq!(vm.read_u8(vaddr + i as u32).unwrap(), 0xAA);
        }

        // BSS zeroed
        for i in filesz as u32..memsz {
            assert_eq!(vm.read_u8(vaddr + i).unwrap(), 0x00);
        }
    }

    #[test]
    fn vm_fetch_reads_instruction_from_pc() {
        let nop_bytes = nop();
        let entry = 0x1000u32;
        let data = build_elf(entry, entry, &nop_bytes);
        let image = read_elf(&data).unwrap();
        let cpu = riscv::cpu::CPU::new(image, 64).unwrap();

        let instruction = cpu.fetch().expect("fetch should not fail");
        assert_eq!(instruction, 0x00000013, "fetched instruction must be NOP");
    }
}
