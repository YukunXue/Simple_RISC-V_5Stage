`timescale 1ps/1ps

// riscvsingle.sv

// RISC-V single-cycle processor
// From Section 7.6 of Digital Design & Computer Architecture
// 27 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)

// Single-cycle implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate



module top(
          // input  logic        clk_in1_p, 
          // input  logic        clk_in1_n,
           input  logic        clk,
           input  logic        reset,                         
          // output logic [31:0] WriteData, DataAdr, 
          // output logic        MemWrite,
           output logic [7:0] led);

  logic [31:0] PC, Instr, ReadData;
  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  //logic        clk;
  // instantiate processor and memories
  //clk_wiz_0 u_clk_wiz
  //    (
  //     // Clock out ports
  //     .clk_out1(clk),     // output clk_out1
  //    // Clock in ports
  //     .clk_in1_p(clk_in1_p),    // input clk_in1_p
  //     .clk_in1_n(clk_in1_n));    // input clk_in1_n

  riscvsingle rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, 
                       WriteData, ReadData,led);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscvsingle(input  logic        clk, reset,
                   output logic [31:0] PCF,
                   input  logic [31:0] Instr,
                   output logic        MemWriteM,
                   output logic [31:0] ALUResultM, WriteDataM,
                   input  logic [31:0] ReadDataM,
                   output logic [7:0] led);

  logic       ZeroE,flushE, ALUSrcE,ResultSrcE0,PCSrcE,RegWriteM,  RegWriteW;
  logic [1:0] ImmSrcD,ResultSrcW;
  logic [2:0] alucontrolE;
  logic [31:0]InstrF,InstrD;

  assign InstrF=Instr;
  controller c(clk, reset,InstrD[6:0], InstrD[14:12], InstrD[30], ZeroE,flushE,
               ImmSrcD,ALUSrcE,ResultSrcE0,PCSrcE,alucontrolE,
               RegWriteM, MemWriteM,
               ResultSrcW,RegWriteW
               );

  datapath dp(clk, reset, ImmSrcD, ALUSrcE,ResultSrcE0,PCSrcE,alucontrolE,RegWriteM,
              ResultSrcW,  RegWriteW,ZeroE,flushE,PCF,InstrF,InstrD,ALUResultM, WriteDataM,ReadDataM,led);

endmodule

module controller(input  logic       clk, reset,
                  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       ZeroE,
                  input  logic       flushE,
                  output logic [1:0] ImmSrcD,
                  output logic       ALUSrcE,ResultSrcE0,PCSrcE,
                  output logic [2:0] alucontrolE,
                  output logic       RegWriteM, MemWriteM,
                  output logic [1:0] ResultSrcW,
                  output logic       RegWriteW
                  );

  logic [1:0] ALUOpD, ResultSrcD; //,ImmSrcD
  logic [2:0] alucontrolD;
  logic       MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD; 

  logic [1:0] ResultSrcE;
  //logic [2:0] alucontrolE;
  logic       MemWriteE, BranchE,  RegWriteE, JumpE; //ALUSrcE,

  logic [1:0] ResultSrcM;
 // logic       MemWriteM, RegWriteM; 

 // logic [1:0] ResultSrcW;
 // logic       RegWriteW;       

  maindec md(op, ResultSrcD, MemWriteD, BranchD,
             ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD);
  aludec  ad(op[5], funct3, funct7b5, ALUOpD, alucontrolD);

  

  //pipeline registers
  floprc #(10) regE(clk, reset, flushE,
                  {ResultSrcD,alucontrolD, MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD}, 
                  {ResultSrcE,alucontrolE, MemWriteE, BranchE, ALUSrcE, RegWriteE, JumpE});
  flopr #(4) regM(clk, reset, 
                  {ResultSrcE, MemWriteE, RegWriteE},
                  {ResultSrcM, MemWriteM, RegWriteM});
  flopr #(3) regW(clk, reset, 
                  {ResultSrcM, RegWriteM},
                  {ResultSrcW, RegWriteW});  

  assign PCSrcE = BranchE & ZeroE | JumpE;
  assign ResultSrcE0=ResultSrcE[0];   

endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrc,
               output logic       RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
          ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
                 3'b101:    ALUControl = 3'b111; // srl
                 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ImmSrcD,
                input  logic        ALUSrcE,ResultSrcE0,PCSrcE,
                input  logic [2:0]  alucontrolE,
                input  logic        RegWriteM,
                input  logic [1:0]  ResultSrcW,
                input  logic        RegWriteW,
                output logic        ZeroE,
                output logic        flushE,
                output logic [31:0] PCF,
                input  logic [31:0] instrF,
                output logic [31:0] instrD,
                output logic [31:0] ALUResultM, WriteDataM,
                input  logic [31:0] ReadDataM,
                output logic [7:0] led);

  logic [4:0]  rs1D, rs2D, rs1E, rs2E,rdD, rdE, rdM, rdW;
  logic [31:0] PCNext, PCD,PCE,PCPlus4F, PCPlus4D,PCPlus4E,PCPlus4M,PCPlus4W, PCTargetE;
 // logic [31:0] instrD;
  logic [31:0] ImmExtD,ImmExtE;
  logic [31:0] RD1D,RD2D,RD1E,RD2E, SrcAE, SrcBE;
  logic [31:0] ALUResultE, ALUResultW, WriteDataE, ReadDataW,ResultW;
  logic [1:0]  forwardaE, forwardbE;    
  logic        stallF, stallD, flushD;  

   // hazard detection
  hazard h(rs1D, rs2D, rs1E, rs2E, rdE, rdM, rdW,
         RegWriteM, RegWriteW, ResultSrcE0,PCSrcE,
         forwardaE, forwardbE, 
         stallF, stallD, rdstall, flushD,flushE);

  // next PC logic
  mux2 #(32)  pcbrmux(PCPlus4F, PCTargetE, PCSrcE, PCNext);

  // Fetch stage logic
  flopenr #(32) pcreg(clk, reset, ~stallF,PCNext,PCF);
  adder       pcadd4(PCF, 32'd4, PCPlus4F);

  // Decode stage
  flopenrc #(32) pcd(clk, reset, ~stallD, flushD, PCF,PCD);
  flopenrc #(32) PC4D(clk, reset, ~stallD, flushD, PCPlus4F,PCPlus4D);
      //need to reset to addi X0, X0, 0
  flopenrc #(32,32'b0010011) INSTD(clk, reset, ~stallD, flushD, instrF, instrD);
  
  assign rs1D = instrD[19:15];
  assign rs2D = instrD[24:20];
  assign rdD = instrD[11:7];

  // register file logic
  regfile     rf(clk, RegWriteW,rs1D, rs2D, rdW,
                  ResultW,RD1D, RD2D,led);
  extend      ext(instrD[31:7], ImmSrcD, ImmExtD);

  //Execute stage 
  floprc #(32) rdata1E(clk, reset, flushE, RD1D, RD1E);
  floprc #(32) rdata2E(clk, reset, flushE, RD2D, RD2E);
  floprc #(32) immE(clk, reset, flushE, ImmExtD, ImmExtE);
  floprc #(32) pcplus4E(clk, reset, flushE, PCPlus4D, PCPlus4E);
  floprc #(32) pcE(clk, reset, flushE, PCD, PCE);
  floprc #(5)  rs11E(clk, reset, flushE, rs1D, rs1E);
  floprc #(5)  rs22E(clk, reset, flushE, rs2D, rs2E);
  floprc #(5)  rddE(clk, reset, flushE, rdD, rdE);
  //mux4 #(32)  forwardaemux(RD1E, ResultW, ALUResultM, ReadDataW, {rdstall,forwardaE}, SrcAE);
  //mux4 #(32)  forwardbemux(RD2E, ResultW, ALUResultM, ReadDataW, {rdstall,forwardbE}, WriteDataE);

  mux3 #(32)  forwardaemux(RD1E, ResultW, ALUResultM, forwardaE, SrcAE);
  mux3 #(32)  forwardbemux(RD2E, ResultW, ALUResultM, forwardbE, WriteDataE);  
  mux2 #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);

  // ALU logic
  alu         alu(SrcAE, SrcBE, alucontrolE, ALUResultE, ZeroE);
  adder       pcaddJAL(PCE, ImmExtE, PCTargetE);
  
  // Memory stage
  flopr #(32) r1M(clk, reset, WriteDataE, WriteDataM);
  flopr #(32) r2M(clk, reset, ALUResultE, ALUResultM);
  flopr #(32) pcplus4M(clk, reset, PCPlus4E, PCPlus4M);
  flopr #(5)  rdm(clk, reset, rdE, rdM);

  //writeback stage
  flopr #(32) aluM(clk, reset, ALUResultM, ALUResultW);
  flopr #(32) pcplus4w(clk, reset, PCPlus4M, PCPlus4W);
  flopr #(32) resultW(clk, reset, ReadDataM, ReadDataW);
  flopr #(5)  rdw(clk, reset, rdM, rdW);
  mux3 #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W,  ResultSrcW, ResultW);


endmodule

module hazard(
    input logic   [4:0] rs1D,
    input logic   [4:0] rs2D,
    input logic   [4:0] rs1E,
    input logic   [4:0] rs2E,
    input logic   [4:0] rdE,
    input logic   [4:0] rdM,
    input logic   [4:0] rdW,
    input logic   RegWriteM,
    input logic   RegWriteW,
    input logic   ResultSrcE0,
    input logic   PCSrcE,
    output logic  [1:0] forwardaE,
    output logic  [1:0] forwardbE, 
    output logic  stallF,
    output logic  stallD,
    output logic  rdstall,
    output logic  flushD,
    output logic  flushE
);


//forwarding src to E stage(ALU)
always_comb
  begin
    forwardaE = 2'b00; forwardbE = 2'b00;
    if(rs1E != 0)
      if(rs1E == rdM & RegWriteM) forwardaE = 2'b10;
      else if(rs1E == rdW & RegWriteW) forwardaE = 2'b01;
    if(rs2E != 0)
      if(rs2E == rdM & RegWriteM) forwardbE = 2'b10;
      else if(rs2E == rdW & RegWriteW) forwardbE = 2'b01;
  end


//stalls load word stall logic
assign #1 lwstall = ResultSrcE0 & (rdE == rs1D | rdE == rs2D);
//assign #1 flushE = lwstall
assign #1 stallD = lwstall ;
assign #1 stallF = stallD  ;

// RF posedge harzard
assign #1 rdstall = (RegWriteW & ( rs1D == rdW || rs2D == rdW) & rs1D !=0 & rs2D!=0)? 1 : 0;
//assign #1 rdstall = 1'b0;

//contorl hazard flush
assign #1 flushD = PCSrcE;
assign #1 flushE = lwstall | PCSrcE ;

endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2,
               output logic [7:0]  led);

  logic [31:0] rf[31:0];
  
  logic [31:0] sim_x1;
  logic [31:0] sim_x2;
  logic [31:0] sim_x3;
  logic [31:0] sim_x5;
  logic [31:0] sim_x6;
  logic [31:0] sim_x13;
  logic [31:0] sim_x12;
  logic [31:0] sim_t3;
  logic [31:0] sim_t4;
  logic [31:0] sim_t5;
  logic [31:0] sim_t6;
  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  //always_ff @(negedge clk)
  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  //assign rd1 = (a1 != 0) ? rf[a1] : 0;
  //assign rd2 = (a2 != 0) ? rf[a2] : 0;
  assign rd1 = (a1 == 0)? 0 : ((a1 == a3 & we3)? wd3 : rf[a1]);
  assign rd2 = (a2 == 0)? 0 : ((a2 == a3 & we3)? wd3 : rf[a2]);

  assign sim_t3 = rf[28];
  assign sim_t4 = rf[29];
  assign sim_t5 = rf[30];
  assign sim_t6 = rf[31];
  assign sim_x1 = rf[1];
  assign sim_x2 = rf[2];
  assign sim_x3 = rf[3];
  assign sim_x5 = rf[5];
  assign sim_x6 = rf[6];

  assign sim_x12 = rf[12];
  assign sim_x13 = rf[13];
  assign led = sim_t4[7:0];
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always@(*)
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             
endmodule
//r
module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule
//r clear  flush  Decode/Eex
module floprc #(parameter WIDTH = 8)
              (input                  clk, reset, clear,
               input      [WIDTH-1:0] d, 
               output reg [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)      q <= #1 0;
    else if (clear) q <= #1 0;
    else            q <= #1 d;
endmodule
//en r   stall  FETCH
module flopenr #(parameter WIDTH = 8)
                (input                  clk, reset,
                 input                  en,
                 input      [WIDTH-1:0] d, 
                 output reg [WIDTH-1:0] q);
 
  always_ff @(posedge clk, posedge reset)
    if      (reset) q <= #1 0;
    else if (en)    q <= #1 d;
endmodule
//en r clear  flush/stall/   Fetch/Decode
module flopenrc #(parameter WIDTH = 8, parameter VALUE_0 = 32'b0)
                 (input                  clk, reset,
                  input                  en, clear,
                  input      [WIDTH-1:0] d, 
                  output reg [WIDTH-1:0] q);
 
  always_ff @(posedge clk, posedge reset)
    if      (reset) q <= #1 VALUE_0;
    else if (clear) q <= #1 VALUE_0;
    else if (en)    q <= #1 d;
endmodule


module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0);   //01: d1  1X: d2  00:d0    s?a:b  | s=1:a  s=0:b
endmodule

module mux4 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2, d3,
              input  logic [2:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[2] ? d3 : (s[1] ? d2 : (s[0] ? d1 : d0));   //01: d1  1X: d2  00:d0    s?a:b  | s=1:a  s=0:b
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

 // initial
 //     $readmemh("/project/users/PKUSOC-18/work/cpu_for_fpga_class/riscv_sources/riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
  //assign RAM[0]  = 32'h00500093;
  //assign RAM[1]  = 32'h00300193;
  //assign RAM[2]  = 32'h00300293;
  //assign RAM[3]  = 32'h00500313;
  //assign RAM[4]  = 32'h40308133;
  //assign RAM[5]  = 32'h00517633;
  //assign RAM[6]  = 32'h002366B3;
  //-----------------------//
  //assign RAM[7]  = 32'h01EF0F33;
  //assign RAM[8]  = 32'h01EF0F33;
  //assign RAM[9]  = 32'h01EF0F33;
  //assign RAM[10] = 32'h01EF0F33;
  //assign RAM[11] = 32'h01EF0F33;
  //assign RAM[12] = 32'h01EF0F33;
  //assign RAM[13] = 32'h01EF0F33;
  //assign RAM[14] = 32'h10000f93;
  //assign RAM[15] = 32'h00100e93;
  //assign RAM[16] = 32'h00000e13;
  //assign RAM[17] = 32'h001E0E13;
  //assign RAM[18] = 32'h01EE0463;
  //assign RAM[19] = 32'hFF9FF06F;
  //assign RAM[20] = 32'h01DE8EB3;
  //assign RAM[21] = 32'hFFFE84E3;
  //assign RAM[22] = 32'hFE9FF06F;

  assign RAM[0]  = 32'h7FF00F13;
  assign RAM[1]  = 32'h01EF0F33;
  assign RAM[2]  = 32'h01EF0F33;
  assign RAM[3]  = 32'h01EF0F33;
  assign RAM[4]  = 32'h01EF0F33;
  assign RAM[5]  = 32'h01EF0F33;
  assign RAM[6]  = 32'h01EF0F33;
  assign RAM[7]  = 32'h01EF0F33;
  assign RAM[8]  = 32'h01EF0F33;
  assign RAM[9]  = 32'h01EF0F33;
  assign RAM[10] = 32'h01EF0F33;
  assign RAM[11] = 32'h01EF0F33;
  assign RAM[12] = 32'h01EF0F33;
  assign RAM[13] = 32'h01EF0F33;
  assign RAM[14] = 32'h00200F13;
  assign RAM[15] = 32'h00100113;
  assign RAM[16] = 32'h10000f93;
  assign RAM[17] = 32'h00100e93;
  assign RAM[18] = 32'h00000e13;
  assign RAM[19] = 32'h001E0E13;
  assign RAM[20] = 32'h01EE0463;
  assign RAM[21] = 32'hFF9FF06F;
  assign RAM[22] = 32'h01DE8EB3;
  assign RAM[23] = 32'h01FE8463;
  assign RAM[24] = 32'hFE9FF06F;
  assign RAM[25] = 32'h002EDEB3;
  assign RAM[26] = 32'hFC2E8EE3;
  assign RAM[27] = 32'h00000e13;
  assign RAM[28] = 32'h001E0E13;
  assign RAM[29] = 32'hFFEE08E3;
  assign RAM[30] = 32'hFF9FF06F;
  //-------------------------------
  //assign RAM[0]  = 32'h7FF00F13;
  //assign RAM[1]  = 32'h01EF0F33;
  //assign RAM[2]  = 32'h01EF0F33;
  //assign RAM[3]  = 32'h01EF0F33;
  //assign RAM[4]  = 32'h01EF0F33;
  //assign RAM[5]  = 32'h01EF0F33;
  //assign RAM[6]  = 32'h01EF0F33;
  //assign RAM[7]  = 32'h01EF0F33;
  //assign RAM[8]  = 32'h01EF0F33;
  //assign RAM[9]  = 32'h01EF0F33;
  //assign RAM[10] = 32'h01EF0F33;
  //assign RAM[11] = 32'h01EF0F33;
  //assign RAM[12] = 32'h01EF0F33;
  //assign RAM[13] = 32'h01EF0F33;
  //assign RAM[14] = 32'h00100113;
  //assign RAM[15] = 32'h20000f93;
  //assign RAM[16] = 32'h00100e93;
  //assign RAM[17] = 32'h00000e13;
  //assign RAM[18] = 32'h001E0E13;
  //assign RAM[19] = 32'h01EE0463;
  //assign RAM[20] = 32'hFF9FF06F;
  //assign RAM[21] = 32'h01DE8EB3;
  //assign RAM[22] = 32'h01FE8463;
  //assign RAM[23] = 32'hFE9FF06F;
  //assign RAM[24] = 32'h002EDEB3;
  //assign RAM[25] = 32'hFC2E8EE3;
  //assign RAM[26] = 32'h00000e13;
  //assign RAM[27] = 32'h001E0E13;
  //assign RAM[28] = 32'hFFEE08E3;
  //assign RAM[29] = 32'hFF9FF06F;
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always@(*)
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule
