# RISC-V Implementation Guide

## The Virtual Machine (VM)

The RV32I VM specifies the following components:

- **Memory:** Is the ram memory, an array of bytes (u8), the length will be defined using an compile flag. Default value: `[u8; 65536]` (64kb).
- **Program Counter (PC)**: Always points to the next instruction on memory. Type `u32`.
- **Registers**: 32 registers named x<0-31> with `u32` type. **x0** is always **0** after each instruction.

A general overview of the VM:

```rust
struct VM {
    ram: [u8; x * 1024], // x = 64 by default
    x: [u32; 32],
    pc: u32,
}

impl VM {
    pub fn new() -> Result<Self, VMError> {...}

    pub fn get_ram(index: usize) -> Result<u8, VMError> {...}
    pub fn set_ram(index: usize, value: u8) -> Result<(), VMError> {...}

    pub fn get_pc() -> u32 {...}
    pub fn advance_pc(offset: u32) -> Result<(), VMError> {...}
    pub fn set_pc(offset: u32) -> Result<(), VMError> {...}
    
    pub fn get_x(index: usize) -> Result<u32, VMError> {..}
    pub fn set_x(index: usize, value: u32) -> Result<(), VMError> {...}
}
```

The methods are important for good practices of encapsulation, can be a bit verbose but it’s safe.