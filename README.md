# Implementation-of-RISCV-Single-Cycle-Datapath-using-Verilog

This project implements a RISC-V single-cycle processor using Verilog. The processor is designed to execute basic arithmetic and logic operations, as well as memory access instructions. It demonstrates fundamental concepts of digital design and microprocessor architecture, making it suitable for educational purposes.

# Overview
The single-cycle RISC-V processor is implemented using a modular design, where each module represents a specific functional block of the processor. The processor supports a subset of RISC-V instructions, including arithmetic operations (addition, subtraction), logical operations (AND, OR), and memory operations (load word). The design is verified using a testbench that simulates the operation of the processor and provides waveform analysis to confirm the correct behavior of each module.

# Modules Description
## 1. Single_Cycle_Top
The top-level module integrates all the components of the processor, connecting them through appropriate control and data signals. It includes modules such as the program counter, instruction memory, register file, ALU, data memory, control unit, and multiplexers.

## 2. PC_Module
The Program Counter (PC) module holds the address of the current instruction. It is updated every clock cycle to point to the next instruction, which is typically the current address plus 4 bytes (assuming each instruction is 32-bit).

## 3. PC_Adder
This module increments the program counter by 4, effectively moving to the next instruction in the instruction memory.

## 4. Instruction_Memory
The instruction memory module reads the instruction from the specified address (PC). The instructions are loaded from an external file (memfile2.hex) during initialization.

## 5. Register_File
This module represents the register file, which consists of 32 registers, each 32-bits wide. It supports two read ports and one write port, allowing simultaneous reading of two registers and writing to a third.

## 6. Sign_Extend
The sign extension module extends the immediate values from the instructions to 32 bits, as required by the processor's ALU for operations.

## 7. Mux
Multiplexers are used to select between different inputs based on control signals. In this design, they are used to select between register values and immediate values, as well as between ALU results and memory data.

## 8. ALU
The Arithmetic Logic Unit (ALU) performs arithmetic and logical operations based on the control signals. It supports operations such as addition, subtraction, AND, OR, and set less than.

## 9. Data_Memory
This module represents the data memory where load (lw) and store (sw) operations are performed. Data can be written to and read from specific addresses.

## 10. Control_Unit_Top
The control unit generates the necessary control signals based on the opcode and function fields of the current instruction. It coordinates the operation of the processor's datapath.

## 11. Main_Decoder and ALU_Decoder
The main decoder interprets the instruction opcode and generates control signals for different components (e.g., register write, memory write). The ALU decoder generates specific control signals for the ALU based on the operation required.

# Assembly Code Execution
The following assembly instructions are executed on this single-cycle RISC-V processor to verify its functionality:

### AND Operation: and x10, x1, x2

Performs a bitwise AND between registers x1 and x2 and stores the result in x10.
### OR Operation: or x11, x3, x4

Performs a bitwise OR between registers x3 and x4 and stores the result in x11.
### Addition: add x12, x5, x6

Adds the values of registers x5 and x6 and stores the result in x12.
### Set Less Than: slt x13, x7, x8

Sets x13 to 1 if x7 is less than x8, otherwise sets it to 0.
### Load Word: lw x14, 0(x9)

Loads the word from the memory address in x9 into register x14.
### Subtraction: sub x15, x1, x2

Subtracts the value in x2 from x1 and stores the result in x15.

# Verification and Testing
The project includes a testbench (Single_Cycle_Top_Tb) that initializes the processor, applies reset conditions, and generates clock signals.
The testbench also initializes some register and memory values to observe the behavior of the processor during the execution of the test instructions.
The results are analyzed using waveform analysis, and the functionality is verified by observing the outputs of each instruction.

# Conclusion
This single-cycle RISC-V datapath project provides a hands-on approach to understanding microprocessor design. By executing basic arithmetic, logical, and memory operations, the project demonstrates the core functionalities of a processor, making it a valuable learning tool for digital design and computer architecture concepts.
