`default_nettype none

// ALU - done
// Control Unit - done
// Immediate Decoder - done
// Multiplexer - done
// Reg_file - done
// PC Register - done
// small operations - done
// processor - done(?s)

// srcA and srcB are the two 32-bit inputs to the ALU
// alu_control is a 3-bit control signal that determines the operation to be performed
// alu_out is the 32-bit output of the ALU
// zero is 1 when alu_out == 0 that is used by Control Unit
// compare = 1 when srcA == srcB
module alu32(
    input [31:0] srcA, srcB,
    input [2:0] alu_control,
    output reg [31:0] alu_out,
    output reg alu_zero,
    output reg alu_less
);

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
      3'b110: alu_out = 32'b0; // blt
      default: alu_out = 32'b0;
    endcase
    alu_zero = (alu_out == 32'b0);
    alu_less = ($signed(srcA) < $signed(srcB));
  end
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
    output reg [2:0] immControl,
    output reg      BranchBLT
    );

// block defines opcodes (more readable xd)
localparam R_TYPE   = 7'b0110011;
localparam addi_instr   = 7'b0010011;
localparam lw_instr   = 7'b0000011;
localparam sw_instr   = 7'b0100011;
localparam beq_blt_instr   = 7'b1100011;  
localparam JAL      = 7'b1101111;
localparam JALR     = 7'b1100111;
localparam ADD_V    = 7'b0001011;

  always @(*) begin
    BranchBeq   = 0;
    BranchJal   = 0;
    BranchJalr  = 0;
    RegWrite    = 0;
    MemToReg    = 0;
    MemWrite    = 0;
    ALUControl  = 3'b000;
    ALUSrc      = 0;
    immControl  = 3'b000;
    BranchBLT   = 0;

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
        else if (funct3 == 3'b111) begin
          ALUControl = 3'b010; // and
        end
        else if (funct3 == 3'b101) begin
          ALUControl = 3'b011; // srl
        end
      end

      addi_instr: begin
        RegWrite = 1;
        ALUSrc   = 1;
        ALUControl = 3'b000;
        immControl = 3'b001;
      end

      lw_instr: begin
        RegWrite = 1;
        ALUSrc   = 1;
        MemToReg = 1;
        ALUControl = 3'b000;
        immControl = 3'b001;
      end

      sw_instr: begin
        MemWrite = 1;
        ALUSrc   = 1;
        ALUControl = 3'b000;
        immControl = 3'b010; 
      end

      beq_blt_instr: begin
        if (funct3 == 3'b000) begin // beq
          BranchBeq = 1;
          ALUControl = 3'b001; 
        end
        else if (funct3 == 3'b100) begin //blt
          BranchBLT = 1;
          ALUControl = 3'b110; // set less than
        end
        immControl = 3'b011;
      end

      JAL: begin
        RegWrite  = 1;
        BranchJal = 1;
        ALUSrc = 1;
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
        if (funct3 == 3'b000) begin
          ALUControl = 3'b100;
        end
        else if (funct3 == 3'b001) begin
          ALUControl = 3'b101;
        end
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
        BranchBLT  = 0;
      end
    endcase
  end
endmodule

module imm_decode(
    input  [31:0] instr,
    input  [2:0]  immControl,
    output reg [31:0] imm_out);

  always @(*) begin
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
  input regWrite,
  input [4:0] rs1, rs2, addressWrite,
  input [31:0] wd,
  output reg [31:0] rd1, rd2);

  reg [31:0] regs [31:0];

  always @(posedge clk) begin
    if (regWrite && (addressWrite != 0))
      regs[addressWrite] <= wd;
  end

  always @(*) begin
    rd1 = (rs1 == 0) ? 32'b0 : regs[rs1];
    rd2 = (rs2 == 0) ? 32'b0 : regs[rs2];
  end
endmodule

module mux2_1(
  input  [31:0] a, 
  input  [31:0] b, 
  input  select,
  output [31:0] y
);
  assign y = (select == 0) ? a : b;
endmodule

module pc_reg(
    input        clk,
    input        reset,
    input [31:0] out,
    output reg [31:0] pc
);
  always @(posedge clk) begin
    if (reset)
      pc <= 32'b0;
    else
      pc <= out;
  end
endmodule


module add_operation(
    input  [31:0] a,
    input  [31:0] b,
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

module or3(input a, b, c, output y);
    assign y = a | b | c;
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
wire [4:0]  a1    = instruction[19:15];
wire [4:0]  a2    = instruction[24:20];
wire [4:0]  a3     = instruction[11:7];


//values from control unit
wire RegWrite, MemToReg, MemWrite, ALUSrc, BranchJal, BranchJalr, BranchBeq, BranchJalrx, branchBLT;
wire [2:0] ALUControl, immControl;
wire alu_zero, alu_less;
wire [31:0] ALUOut;

control_unit cu_res (opcode, funct3, funct7, BranchBeq, BranchJal, BranchJalr, RegWrite, MemToReg, MemWrite, ALUControl, ALUSrc, immControl, branchBLT);

wire [31:0] imm_out;
imm_decode dec_res (instruction, immControl, imm_out);

wire [31:0] rd1, rd2, writeData;

reg_file reg_res (clk, RegWrite, a1, a2, a3, writeData, rd1, rd2);

wire [31:0] SrcA = rd1;
wire [31:0] SrcB;

mux2_1 alu_operand_mux (rd2, imm_out, ALUSrc, SrcB);
alu32 alu_res (SrcA, SrcB, ALUControl, ALUOut, alu_zero, alu_less);

assign WE             = MemWrite;
assign address_to_mem = ALUOut; 
assign data_to_mem    = rd2;

wire [31:0] PCplus4;
wire [31:0] jalr_target;
wire [31:0] regWriteData;

mux2_1 regWriteMux1 (ALUOut, PCplus4, BranchJalrx, jalr_target);
mux2_1 regWriteMux2 (jalr_target, data_from_mem, MemToReg, writeData);
//==================================================

wire [31:0] branchAdderOutput;
wire[31:0] branchTargetOutput;
add_operation branchAdder (imm_out, PC, branchAdderOutput);
mux2_1 branchTarget_mux (branchAdderOutput, ALUOut, BranchJalr, branchTargetOutput);

add_operation pcPlus4Adder (PC, 32'd4, PCplus4);

wire BranchOutcome;
wire[31:0] Pc_n;
mux2_1 leftestMux (PCplus4, branchTargetOutput, BranchOutcome, Pc_n);
pc_reg pc_register (clk, reset, Pc_n, PC);

or2 branchOrJalr (BranchJal, BranchJalr, BranchJalrx);

wire branchBeqAndZeroResult;
wire branchBLTandLessResult;
and2 branchAndZero (BranchBeq, alu_zero, branchBeqAndZeroResult);
and2 branchBltAndLess (branchBLT, alu_less, branchBLTandLessResult);

or3 branchOutcomeOrJal (branchBeqAndZeroResult, BranchJalrx, branchBLTandLessResult, BranchOutcome);

endmodule

//... add new modules here ...
`default_nettype wire