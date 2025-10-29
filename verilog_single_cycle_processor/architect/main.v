`default_nettype none

// ALU - done
// Control Unit - done
// Immediate Decoder - done
// Multiplexer - done
// Reg_file - done

// srcA and srcB are the two 32-bit inputs to the ALU
// alu_control is a 3-bit control signal that determines the operation to be performed
// alu_out is the 32-bit output of the ALU
// zero is 1 when alu_out == 0 that is used by Control Unit
// compare = 1 when srcA == srcB
module alu32(
    input [31:0] srcA, srcB,
    input [2:0] alu_control,
    output reg [31:0] alu_out,
    output alu_compare,
    output alu_zero);

  always @(*) begin //nastava zmena hodnot
    case (alu_control)
      3'b000: alu_out = srcA + srcB; // 000 - add      
      3'b001: alu_out = srcA - srcB; // 001 - sub
      3'b010: alu_out = srcA & srcB; // 010 - and
      3'b011: alu_out = srcA >> srcB[4:0]; // 011 - srl
      3'b100: begin // 100 - add_v
        alu_out = {(srcA[31:24] + srcB[31:24]),
             (srcA[23:16] + srcB[23:16]),
             (srcA[15:8]  + srcB[15:8]),
             (srcA[7:0]   + srcB[7:0])};
      end
      3'b101: begin // 101 - avg_v
        alu_out = {((srcA[31:24] + srcB[31:24]) >> 1),
             ((srcA[23:16] + srcB[23:16]) >> 1),
             ((srcA[15:8]  + srcB[15:8])  >> 1),
             ((srcA[7:0]   + srcB[7:0])   >> 1)};
      end
      default: alu_out = 0;
    endcase
  end

  assign alu_compare = (srcA == srcB);
  assign alu_zero = (alu_out == 0);

endmodule

// opcode - 7-bit opcode from instruction
// funct3 - 3-bit funct3 from instruction
// funct7 - 7-bit funct7 from instruction

module control_unit(
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg       BranchBeq,
    output reg       BranchJal,
    output reg       BranchJalr,
    output reg       RegWrite,
    output reg       MemToReg,
    output reg       MemWrite,
    output reg [2:0] ALUControl,
    output reg       ALUSrc,
    output reg [2:0] immControl);

// block defines opcodes (more readable xd)
localparam R_TYPE   = 7'b0110011;
localparam I_TYPE   = 7'b0010011;
localparam I_LOAD   = 7'b0000011;
localparam S_TYPE   = 7'b0100011;
localparam B_TYPE   = 7'b1100011;  
localparam JAL      = 7'b1101111;
localparam JALR     = 7'b1100111;
localparam ADD_V    = 7'b0001011;
localparam AVG_V    = 7'b0001011;

always @(*) 
begin
  BranchBeq   = 0;
  BranchJal   = 0;
  BranchJalr  = 0;
  RegWrite    = 0;
  MemToReg    = 0;
  MemWrite    = 0;
  ALUControl  = 3'b000;
  ALUSrc      = 0;
  immControl  = 3'b000;

  case (opcode)
    R_TYPE: begin
      RegWrite = 1;
      ALUSrc   = 0;
      if (funct3 == 3'b000) begin
        if (funct7 == 7'b0100000)
          ALUControl = 3'b001; // sub
        else
          ALUControl = 3'b000; // add
      end
      else if (funct3 == 3'b111)
        ALUControl = 3'b010; // and
      else if (funct3 == 3'b101)
        ALUControl = 3'b011; // srl
    end

    I_TYPE: begin
      RegWrite = 1;
      ALUSrc   = 1;
      ALUControl = 3'b000;
      immControl = 3'b001;
    end

    I_LOAD: begin
      RegWrite = 1;
      ALUSrc   = 1;
      MemToReg = 1;
      ALUControl = 3'b000;
      immControl = 3'b001;
    end

    S_TYPE: begin
      MemWrite = 1;
      ALUSrc   = 1;
      ALUControl = 3'b000;
      immControl = 3'b010; 
    end

    B_TYPE: begin
      if (funct3 == 3'b000)
        BranchBeq = 1;
      else if (funct3 == 3'b100)
        BranchBeq = 1;
      immControl = 3'b011;
    end

    JAL: begin
      RegWrite  = 1;
      BranchJal = 1;
      immControl = 3'b101;
    end

    JALR: begin
      RegWrite  = 1;
      BranchJalr = 1;
      ALUSrc = 1;
      immControl = 3'b001;
    end

    ADD_V: begin
      RegWrite = 1;
      ALUSrc   = 0;
      if (funct3 == 3'b000)
        ALUControl = 3'b100;
      else if (funct3 == 3'b001)
        ALUControl = 3'b101;
    end

    default: begin
      RegWrite   = 0;
      MemToReg   = 0;
      MemWrite   = 0;
      BranchBeq  = 0;
      BranchJal  = 0;
      BranchJalr = 0;
      ALUSrc     = 0;
      ALUControl = 3'b000;
      immControl = 3'b000;
    end

  endcase
end

endmodule

module imm_decode(
    input  [31:0] instr,
    input  [2:0]  immControl,
    output reg [31:0] imm_out);

always @(*)
begin
    case (immControl)
        3'b000: imm_out = 0; // R-type
        3'b001: imm_out =  {{20{instr[31]}}, instr[31:20]}; // I-type
        3'b010: imm_out = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
        3'b011: imm_out = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
        3'b100: imm_out = {instr[31:12], 12'b0}; // U-type
        3'b101: imm_out = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
        default: imm_out = 32'b0;
    endcase
end
endmodule

module reg_file(
    input clk,
    input is_signal,
    input [4:0] rs1, rs2, res,
    input [31:0] wd,
    output [31:0] rd1, rd2);

  reg [31:0] regs [31:0];

  always @(posedge clk)
    if (is_signal && (res != 0))
      regs[res] <= wd;

  assign rd1 = (rs1 == 0) ? 0 : regs[rs1];
  assign rd2 = (rs2 == 0) ? 0 : regs[rs2];
endmodule

module mux2_1(
  input  [31:0] a, 
  input  [31:0] b, 
  input  select,
  output [31:0] y
);
  assign y = (select == 0) ? a : b;
endmodule

module adder32(
    input  [31:0] a, b,
    output [31:0] y
);
  assign y = a + b;
endmodule

module and2(input a, b, output y);
  assign y = a & b;
endmodule

module or2(input a, b, output y);
  assign y = a | b;
endmodule

module pc_reg(
    input        clk,
    input        reset,
    input [31:0] pc_next,
    output reg [31:0] pc
);
  always @(posedge clk or posedge reset) begin
    if (reset)
      pc <= 32'h0000_0000;
    else
      pc <= pc_next;
  end
endmodule

module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
);

wire [6:0]  opcode = instruction[6:0];
wire [2:0]  funct3 = instruction[14:12];
wire [6:0]  funct7 = instruction[31:25];
wire [4:0]  rs1    = instruction[19:15];
wire [4:0]  rs2    = instruction[24:20];
wire [4:0]  rd     = instruction[11:7];

wire RegWrite, MemToReg, MemWrite, ALUSrc, ALUControl, immControl, BranchJal, BranchJalr, BranchBeq;
wire [2:0] ALUControl, immControl;

control_unit cu_res (opcode, funct3, funct7, BranchBeq, BranchJal, BranchJalr, RegWrite, MemToReg, MemWrite, ALUControl, ALUSrc, immControl);

reg [31:0] imm_out;

imm_decode dec_res (instruction, immControl, imm_out);

wire [31:0] rd1, rd2, wd;

reg_file reg_res (clk, RegWrite, rs1, rs2, rd, wd, rd1, rd2);

wire [31:0] SrcA = rd1;
wire [31:0] SrcB;

mux2_1 srcb_mux (rd2, imm_out, ALUSrc, SrcB);

alu32


endmodule

//... add new modules here ...
`default_nettype wire