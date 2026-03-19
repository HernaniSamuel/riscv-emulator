# RISC-V Implementation guide

## The Virtual Machine (VM)

The **RV32I VM** consists of the following components:

- **Memory:** Represents **RAM** as an array of bytes (`u8`). Its **length** is defined by a **compile-time flag**. The **default** is `[u8; 65536]` (**64KB**).
- **Program Counter (PC)**: Always **points** to the **next instruction** on **memory**. Type `u32`.
- **Registers**: 32 registers, named x<0-31>, of type `u32`. **x0** is always **zero**.

A general overview of the VM:

```rust
pub struct VM {
    ram: [u8; x * 1024], // x = 64 by default
    x: [u32; 32],
    pc: u32,
    // CSR here later...
}

impl VM {
    pub fn new(elf_file: Vec<u8>) -> Result<Self, VMError> {...}
    
    pub fn get_ram(&self, index: usize) -> Result<u8, VMError> {...}
    pub fn set_ram(&mut self, index: usize, value: u8) -> Result<(), VMError> {...}
    
    pub fn get_pc(&self) -> u32 {...}
    pub fn advance_pc(&mut self, offset: u32) -> Result<(), VMError> {...}
    pub fn set_pc(&mut self, offset: u32) -> Result<(), VMError> {...}
    
    pub fn get_x(&self, index: usize) -> Result<u32, VMError> {...}
    pub fn set_x(&mut self, index: usize, value: u32) -> Result<(), VMError> {...}
    
    // CSR getters and setters
}
```

These methods are important for good encapsulation practices. These methods enforce memory and register safety at the cost of some verbosity.

The expected behavior of each VM method is described below.

- **new**: Creates a **new VM instance**, loads the ELF into memory, and returns `Self`. It can **fail** if the **ELF** is **larger** than the **RAM length**, in which case it propagates the `VMError` described in the **RAM methods** below.
- **get_ram**: If `ram.get(index)` returns `Some(&u8)`, it returns `Ok(u8)`. Otherwise, it returns a `VMError` indicating an attempt to access an **invalid memory address**.
- **set_ram**: If `ram.get_mut(index)` returns `None`, it returns a `VMError` indicating an attempt to access an **invalid memory address**. If it returns `Some(&u8)`, the value is **dereferenced** and updated to match the value provided in the **value** parameter.
- **get_pc**: This method only returns the **current** PC value as a `u32`. It can’t return an error, and it exists only because the VM’s PC attribute **can’t be public**. Otherwise, it could be **modified freely** and get **out of control**.
- **set_pc**: If the offset is greater than the memory length, it returns a `VMError` indicating that the PC is **out of bounds**. This may seem redundant since RAM already has a bounds checker, but it helps catch when the PC goes out of bounds and is useful for **debugging**.
- **advance_pc**: If the function argument plus the current PC value is greater than the memory length minus 4 (to ensure all 4 bytes of the instruction fit within memory), it returns a `VMError` indicating that the PC is **out of bounds**. Otherwise, it sets the PC to the current value plus the provided offset.
- **get_x**: If `x.get(index)` returns `Some(&u32)`, it returns `Ok(u32)`. Otherwise, it returns a `VMError` indicating an attempt to access an **invalid register address**.
- **set_x:** If `x.get_mut(index)` returns `None`, it returns a `VMError` indicating an attempt to access an **invalid register address**. If it returns `Some(&u32)` and the **index is not zero**, the value is **dereferenced** and updated to match the value provided in the **value** parameter. If the **index** is **zero**, it **does not modify the register value** (since **x0 is always zero**).

## The Central Processing Unit (CPU)

The RV32I CPU consists of the following components:

- **Virtual Machine (VM):** An instance of the Virtual Machine struct that serves as the virtual hardware the CPU uses to operate.
- **Running**: A boolean value that indicates whether the CPU is still running.
- **Exit Code**: A `i32` value that indicates the exit code returned by the program when it halts.

A general overview of the CPU:

```rust
pub struct CPU {
	vm: VM,
	running: bool,
	exit_code: i32,
}

impl CPU {
	pub fn new(elf_file: Vec<u8>) -> Result<Self, VMError> {...}
	
	pub fn step(&mut self) -> Result<(), GeneralError> {...}
	fn fetch(&mut self) -> Result<u32, VMError> {...}
	fn decode(&self, opcode: u32) -> Result<Instruction, GeneralError> {...}
	// decode method has some utility functions
	fn execute(&mut self, instruction: Instruction) -> Result<(), VMError> {...}
	
	pub fn get_exit_code(&self) -> i32 {...}
	pub fn set_exit_code(&mut self, value: i32) -> () {...}
	
	pub fn is_running(&self) -> bool {...}
	pub fn set_running(&mut self, value: bool) -> () {...}
}
```

This is the list of the CPU methods and how each one should work:

- **new**: Creates a new VM instance and assigns it to the CPU struct. Returns `Self` or propagates the `VMError`.

- **fetch**: Constructs the instruction using bitwise operations to concatenate 4 bytes into a `u32` instruction:
    1. Use `self.vm.get_ram` with the indexes `pc`, `pc + 1`, `pc + 2`, and `pc + 3`. Then, store the bytes in 4 `u8` variables.
    2. Use a bit shift operation to combine the four bytes into a `u32` instruction value:
    
    ```rust
    let instruction = (byte1 as u32)
               | (byte2 as u32) << 8
               | (byte3 as u32) << 16
               | (byte4 as u32) << 24;
    ```
    
    1. Call `self.vm.advance_pc(4)` so the PC always points to the next instruction after fetching.
    
- **decode:** All RV32I instructions have the same length of 32 bits, changing only the format. By decoding the instruction, we want to extract some information values to discover what the machine should do. The values are:
    - **opcode:** The **opcode** indicates the **format** of an **instruction**. The **seven lower bits** of an instruction **contains the opcode**.
    - **Destination Register (rd)**: As the name suggests, this is the **register** where the **result** of the operation **will be stored**.
    - **Source Register 1 (rs1):** Source register 1 provides the **first input value** to the **ALU (Arithmetic Logic Unit)**.
    - **Source Register 2 (rs2)**: Source register 2 provides the **second value** to the **ALU**.
    - **Function 3 (func3)**: func3 is a **three-bit field** that helps the **CPU** identify the **instruction format** (R, I, S, B, U or J).
    - **Function 7 (func7)**: funct7 is a **7-bit field** that helps the **processor** determine the exact operation to perform in **R-type** instructions.
    - **Immediate value (imm):** imm is a **constant value** that is passed by an **instruction**. It is useful for **avoiding unnecessary register usage**.
    
    These values can be extracted using **bitmasks**. It is possible to extract all the values described (except imm) using this procedure.
    
    ```rust
    // Utility function of the decode method
    fn get_bits(value: u32, shift: u8, mask: u8) -> u32 {
        ((value >> shift) & mask as u32)
    }
    
    // Utility function of the decode method
    fn sign_ext(value: u32, bits: u32) -> i32 {
        let shift = 32 - bits;
        ((value << shift) as i32) >> shift
    }
    
    let opcode: u32 = (instruction & 0x7F) as u32;
    let rd: u32     = get_bits(instruction, 7, 0x1F);
    let rs1: u32    = get_bits(instruction, 15, 0x1F);
    let rs2: u32    = get_bits(instruction, 20, 0x1F);
    let funct3: u32 = get_bits(instruction, 12, 0x7);
    ```
    
    The imm field bit positions change according to the instruction type, so it should be extracted according to the instruction type as well. This can be done using the following steps.
    
    ```rust
    let imm_i = sign_ext(instruction >> 20, 12);
    
    let imm_s = sign_ext(
        (get_bits(instruction, 25, 0x7F) << 5)
        | get_bits(instruction, 7, 0x1F),
        12,
    );
    
    let imm_b = sign_ext(
        (get_bits(instruction, 31, 0x1) << 12)
        | (get_bits(instruction, 7,  0x1) << 11)
        | (get_bits(instruction, 25, 0x3F) << 5)
        | (get_bits(instruction, 8,  0xF) << 1),
        13,
    );
    
    let imm_u = (instruction & 0xFFFFF000) as i32;
    
    let imm_j = sign_ext(
        (get_bits(instruction, 31, 0x1)  << 20)
        | (get_bits(instruction, 12, 0xFF) << 12)
        | (get_bits(instruction, 20, 0x1)  << 11)
        | (get_bits(instruction, 21, 0x3FF) << 1),
        21,
    );
    
    let shamt = get_bits(instruction, 20, 0x1F) as u8;
    ```
    
    The **decoder** matches the **opcode** to determine the **instruction type**, then uses **funct3** and **funct7** to identify the **exact instruction**.
    
    ```rust
    match opcode {
        // ================= R =================
        0b0110011 => match (funct3, funct7) {
            (0x0, 0x00) => Instr::Add  { rd, rs1, rs2 },
            (0x0, 0x20) => Instr::Sub  { rd, rs1, rs2 },
            (0x1, 0x00) => Instr::Sll  { rd, rs1, rs2 },
            (0x2, 0x00) => Instr::Slt  { rd, rs1, rs2 },
            (0x3, 0x00) => Instr::Sltu { rd, rs1, rs2 },
            (0x4, 0x00) => Instr::Xor  { rd, rs1, rs2 },
            (0x5, 0x00) => Instr::Srl  { rd, rs1, rs2 },
            (0x5, 0x20) => Instr::Sra  { rd, rs1, rs2 },
            (0x6, 0x00) => Instr::Or   { rd, rs1, rs2 },
            (0x7, 0x00) => Instr::And  { rd, rs1, rs2 },
            _ => Instr::Illegal(instruction),
        },
    
        // ================= I (ALU) =================
        0b0010011 => match funct3 {
            0x0 => Instr::Addi  { rd, rs1, imm: imm_i },
            0x2 => Instr::Slti  { rd, rs1, imm: imm_i },
            0x3 => Instr::Sltiu { rd, rs1, imm: imm_i },
            0x4 => Instr::Xori  { rd, rs1, imm: imm_i },
            0x6 => Instr::Ori   { rd, rs1, imm: imm_i },
            0x7 => Instr::Andi  { rd, rs1, imm: imm_i },
    
            0x1 => match funct7 {
                0x00 => Instr::Slli { rd, rs1, shamt },
                _ => Instr::Illegal(instruction),
            },
    
            0x5 => match funct7 {
                0x00 => Instr::Srli { rd, rs1, shamt },
                0x20 => Instr::Srai { rd, rs1, shamt },
                _ => Instr::Illegal(instruction),
            },
    
            _ => Instr::Illegal(instruction),
        },
    
        // ================= LOAD =================
        0b0000011 => match funct3 {
            0x0 => Instr::Lb  { rd, rs1, imm: imm_i },
            0x1 => Instr::Lh  { rd, rs1, imm: imm_i },
            0x2 => Instr::Lw  { rd, rs1, imm: imm_i },
            0x4 => Instr::Lbu { rd, rs1, imm: imm_i },
            0x5 => Instr::Lhu { rd, rs1, imm: imm_i },
            _ => Instr::Illegal(instruction),
        },
    
        // ================= STORE =================
        0b0100011 => match funct3 {
            0x0 => Instr::Sb { rs1, rs2, imm: imm_s },
            0x1 => Instr::Sh { rs1, rs2, imm: imm_s },
            0x2 => Instr::Sw { rs1, rs2, imm: imm_s },
            _ => Instr::Illegal(instruction),
        },
    
        // ================= BRANCH =================
        0b1100011 => match funct3 {
            0x0 => Instr::Beq  { rs1, rs2, imm: imm_b },
            0x1 => Instr::Bne  { rs1, rs2, imm: imm_b },
            0x4 => Instr::Blt  { rs1, rs2, imm: imm_b },
            0x5 => Instr::Bge  { rs1, rs2, imm: imm_b },
            0x6 => Instr::Bltu { rs1, rs2, imm: imm_b },
            0x7 => Instr::Bgeu { rs1, rs2, imm: imm_b },
            _ => Instr::Illegal(instruction),
        },
    
        // ================= U =================
        0b0110111 => Instr::Lui   { rd, imm: imm_u },
        0b0010111 => Instr::Auipc { rd, imm: imm_u },
    
        // ================= J =================
        0b1101111 => Instr::Jal  { rd, imm: imm_j },
        0b1100111 => match funct3 {
            0x0 => Instr::Jalr { rd, rs1, imm: imm_i },
            _ => Instr::Illegal(instruction),
        },
    
        // ================= FENCE =================
        0b0001111 => match funct3 {
            0x0 => Instr::Fence,
            0x1 => Instr::FenceI,
            _ => Instr::Illegal(instruction),
        },
    
        // ================= SYSTEM =================
        0b1110011 => match funct3 {
            0x0 => match imm_i {
                0 => Instr::Ecall,
                1 => Instr::Ebreak,
                _ => Instr::Illegal(instruction),
            },
    
            0x1 => Instr::Csrrw  { rd, rs1, csr },
            0x2 => Instr::Csrrs  { rd, rs1, csr },
            0x3 => Instr::Csrrc  { rd, rs1, csr },
            0x5 => Instr::Csrrwi { rd, zimm: rs1, csr },
            0x6 => Instr::Csrrsi { rd, zimm: rs1, csr },
            0x7 => Instr::Csrrci { rd, zimm: rs1, csr },
    
            _ => Instr::Illegal(instruction),
        },
    
        _ => Instr::Illegal(instruction),
    }
    ```