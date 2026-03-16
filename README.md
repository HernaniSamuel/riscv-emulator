# RISC-V Implementation guide

## The Virtual Machine (VM)

The RV32I VM consists of the following components:

- **Memory:** Represents **RAM** as an array of bytes (`u8`). Its **length** is defined by a **compile-time flag**. The **default** is `[u8; 65536]` (**64KB**).
- **Program Counter (PC)**: Always **points** to the **next instruction** on **memory**. Type `u32`.
- **Registers**: 32 registers, named x<0-31>, of type `u32`. **x0** is always **zero**.

A general overview of the VM:

```rust
pub struct VM {
    ram: [u8; x * 1024], // x = 64 by default
    x: [u32; 32],
    pc: u32,
}

impl VM {
    pub fn new(elf_file: Vec<u8>) -> Result<Self, VMError> {...}
    
    pub fn get_ram(&self, index: usize) -> Result<u8, VMError> {...}
    pub fn set_ram(&mut self, index: usize, value: u8) -> Result<(), VMError> {...}
    
    pub fn get_pc(&self) -> u32 {...}
    pub fn advance_pc(&mut self, offset: u32) -> Result<(), VMError> {...}
    pub fn set_pc(&mut self, offset: u32) -> Result<(), VMError> {...}
    
    pub fn get_x(&self, index: usize) -> Result<u32, VMError> {..}
    pub fn set_x(&mut self, index: usize, value: u32) -> Result<(), VMError> {...}
}
```

These methods are important for good encapsulation practices. They can be a bit verbose, but they are safe.

The list of VM methods and how each one should work:

- **new**: Creates a **new VM instance**, loads the ELF into memory, and returns `Self`. It can **fail** if the **ELF** is **larger** than the **RAM length**, in which case it propagates the `VMError` described in the **RAM methods** below.
- **get_ram**: If `ram.get(index)` returns `Some(&u8)`, it returns `Ok(u8)`. Otherwise, it returns a `VMError` indicating an attempt to access an **invalid memory address**.
- **set_ram**: If `ram.get_mut(index)` returns `None`, it returns a `VMError` indicating an attempt to access an **invalid memory address**. If it returns `Some(&u8)`, the value is **dereferenced** and updated to match the value provided in the **value** parameter.
- **get_pc**: This method only returns the **current** PC value as a `u32`. It can’t return an error, and it exists only because the VM’s PC attribute **can’t be public**. Otherwise, it could be **modified freely** and get **out of control**.
- **set_pc**: If the offset is greater than the memory length, it returns a `VMError` indicating that the PC is **out of bounds**. This may seem redundant since RAM already has a bounds checker, but it helps catch when the PC goes out of bounds and is useful for **debugging**.
- **advance_pc**: If the function argument plus the current PC value is greater than the memory length minus 5 (to ensure all 4 bytes of the instruction fit within memory), it returns a `VMError` indicating that the PC is **out of bounds**. Otherwise, it sets the PC to the current value plus the provided offset.
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
	fn execute(&mut self, instruction: Instruction) -> Result<(), VMError> {...}
	
	pub fn get_exit_code(&self) -> i32 {...}
	pub fn set_exit_code(&mut self, value: i32) -> () {...}
	
	pub fn is_running(&self) -> bool
	pub fn set_running(&mut self, value: bool) -> () {...}
}
```

This is the list of the CPU methods and how each one should work: