
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

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
   logic [7:0] led;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite,led);
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
     //    #1000 $finish;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end
  //dump fsdb 
     initial begin 
       $fsdbDumpfile("riscvpipe.fsdb");
           $fsdbDumpvars(0);
     end 

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 25) begin
          $display("Simulation succeeded");
           $finish();//$stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $finish(); //$stop;
        end
      end
    end
  initial begin

      #10000ns;

      $finish ( );//主动的结束仿真

    end

endmodule

module top(input  logic       clk, 
           
           input  logic        reset,                         
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite,
           output logic [7:0] led);

  logic [31:0] PC, Instr, ReadData;
  //logic [31:0] WriteData, DataAdr;
 
  // instantiate processor and memories

  riscvpipe rvsingle(clk, reset, PC, Instr, MemWrite, DataAdr, 
                       WriteData, ReadData,led);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscvpipe(input  logic        clk, reset,
                   output logic [31:0] PCF,
                   input  logic [31:0] Instr,
                   output logic        MemWriteM,
                   output logic [31:0] ALUResultM, WriteDataM,
                   input  logic [31:0] ReadDataM,
                   output logic [7:0] led);

  logic       ZeroE,div_stallE,flushE, ALUSrcE,ResultSrcE0,PCSrcE,
              RegWriteM,  RegWriteW;
  logic [1:0] ImmSrcD,ALUME,ResultSrcW;
  logic [2:0] alucontrolE;
  logic [31:0]InstrF,InstrD;

  assign InstrF=Instr;
  controller c(clk, reset,InstrD[6:0], InstrD[14:12], InstrD[30],InstrD[25],
                ZeroE,flushE,div_stallE,
               ImmSrcD,ALUSrcE,ResultSrcE0,PCSrcE,alucontrolE,ALUME,
               RegWriteM, MemWriteM,
               ResultSrcW,RegWriteW
               );

  datapath dp(clk, reset, ImmSrcD, ALUSrcE,ResultSrcE0,PCSrcE,alucontrolE,ALUME,RegWriteM,
              ResultSrcW,  RegWriteW,ZeroE,flushE, div_stallE, 
              PCF,InstrF,InstrD,ALUResultM, WriteDataM,ReadDataM,led);

endmodule

module controller(input  logic       clk, reset,
                  input  logic [6:0] op,  
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       funct7b0,
                  input  logic       ZeroE,
                  input  logic       flushE,
                  input  logic       div_stallE,                 
                  output logic [1:0] ImmSrcD,
                  output logic       ALUSrcE,ResultSrcE0,PCSrcE,
                  output logic [2:0] alucontrolE,
                  output logic [1:0] ALUME,
                  output logic       RegWriteM, MemWriteM,
                  output logic [1:0] ResultSrcW,
                  output logic       RegWriteW
                  );

  logic [1:0] ALUOpD, ResultSrcD,ALUMD; //,ImmSrcD
  logic [2:0] alucontrolD;
  logic       MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD; 

  logic [1:0] ResultSrcE;
  //logic [2:0] alucontrolE;
  logic       MemWriteE, BranchE,  RegWriteE, JumpE; //ALUSrcE,

  logic [1:0] ResultSrcM;
  logic       div_flushE; 
  assign      div_flushE=ALUME[0];

 // logic [1:0] ResultSrcW;
 // logic       RegWriteW;       

  maindec md(op, ResultSrcD, MemWriteD, BranchD,
             ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD);
  aludec  ad(op[5], funct3, funct7b5,funct7b0, ALUOpD, alucontrolD, ALUMD);

  

  //pipeline registers
  flopenrc #(11) regE(clk, reset,~div_stallE, flushE,
                  {ResultSrcD,alucontrolD[2],alucontrolD[0], MemWriteD, BranchD, ALUSrcD, RegWriteD, JumpD, ALUMD}, 
                  {ResultSrcE,alucontrolE[2],alucontrolE[0], MemWriteE, BranchE, ALUSrcE, RegWriteE, JumpE, ALUME});
   //for div reset_i_start;
  floprc #(1) reset_i_startE(clk, reset, div_flushE|flushE, alucontrolD[1],alucontrolE[1]);//下周期div_i_startE=0
  flopenr #(4) regM(clk, reset, ~div_stallE,
                  {ResultSrcE, MemWriteE, RegWriteE},
                  {ResultSrcM, MemWriteM, RegWriteM});
  flopenr #(3) regW(clk, reset, ~div_stallE,
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
              input  logic       funct7b0, 
              input  logic [1:0] ALUOp,
              output reg [2:0] ALUControl,
              output logic [1:0]     ALU_M);
//TODO

endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  ImmSrcD,
                input  logic        ALUSrcE,ResultSrcE0,PCSrcE,
                input  logic [2:0]  alucontrolE,
                input  logic [1:0]  ALUME,   
                input  logic        RegWriteM,
                input  logic [1:0]  ResultSrcW,
                input  logic        RegWriteW,
                output logic        ZeroE,
                output logic        flushE, div_stall,          
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
  logic        stallF, stallD,flushD;  
  logic        DIV_flushE,DIV_busyE,DIV_validE,div_stallE;
  logic        RtypedivE,div_i_startE;
  
  assign div_stall=div_stallE;

  assign     RtypedivE=ALUME[0];
  assign     div_i_startE=RtypedivE & ALUME[0];
   // hazard detection
  hazard h(rs1D, rs2D, rs1E, rs2E, rdE, rdM, rdW,
         RegWriteM, RegWriteW, ResultSrcE0,PCSrcE,RtypedivE,DIV_validE,
         div_stallE,
         forwardaE, forwardbE, 
         stallF, stallD,flushD,flushE);

  // next PC logic
  mux2 #(32)  pcbrmux(PCPlus4F, PCTargetE, PCSrcE, PCNext);

  // Fetch stage logic
  flopenr #(32) pcreg(clk, reset, ~(stallD|div_stallE),PCNext,PCF);
  adder       pcadd4(PCF, 32'd4, PCPlus4F);

  // Decode stage
  flopenrc #(32) pcd(clk, reset, ~(stallD|div_stallE), flushD, PCF,PCD);
  flopenrc #(32) PC4D(clk, reset, ~(stallD|div_stallE), flushD, PCPlus4F,PCPlus4D);
      //need to reset to addi X0, X0, 0
  flopenrc #(32,32'b0010011) INSTD(clk, reset, ~(stallD|div_stallE), flushD, instrF, instrD);
  
  assign rs1D = instrD[19:15];
  assign rs2D = instrD[24:20];
  assign rdD = instrD[11:7];
  // register file logic
  regfile     rf(clk, RegWriteW,rs1D, rs2D, rdW,
                  ResultW,RD1D, RD2D,led);
  extend      ext(instrD[31:7], ImmSrcD, ImmExtD);

  //Execute stage 
  flopenrc #(32) rdata1E(clk, reset, ~div_stallE, flushE, RD1D, RD1E);
  flopenrc #(32) rdata2E(clk, reset, ~div_stallE, flushE, RD2D, RD2E);
  flopenrc #(32) immE   (clk, reset, ~div_stallE, flushE, ImmExtD, ImmExtE);
  flopenrc #(32) pcplus4E(clk, reset,~div_stallE, flushE, PCPlus4D, PCPlus4E);
  flopenrc #(32) pcE(clk, reset,     ~div_stallE, flushE, PCD, PCE);
  flopenrc #(5)  rs11E(clk, reset,   ~div_stallE, flushE, rs1D, rs1E);
  flopenrc #(5)  rs22E(clk, reset,   ~div_stallE, flushE, rs2D, rs2E);
  flopenrc #(5)  rddE(clk, reset,    ~div_stallE, flushE, rdD, rdE);
 

  mux3 #(32)  forwardaemux(RD1E, ResultW, ALUResultM, forwardaE, SrcAE);
  mux3 #(32)  forwardbemux(RD2E, ResultW, ALUResultM, forwardbE, WriteDataE);
  mux2 #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);

  // ALU logic
  //assign DIV_flushE = flushE;
  alu         alu(clk, reset,SrcAE, SrcBE, alucontrolE, ALUME,flushE,DIV_busyE,DIV_validE, ALUResultE, ZeroE);
  adder       pcaddJAL(PCE, ImmExtE, PCTargetE);
  
  // Memory stage 
  //add div  when div is valid, continue the pipeline 
 
  flopenr #(32) r1M    (clk, reset,  ~div_stallE  , WriteDataE, WriteDataM);
  flopenr #(32) r2M    (clk, reset,  ~div_stallE  ,ALUResultE, ALUResultM);
  flopenr #(32) pcplus4M(clk, reset, ~div_stallE  ,  PCPlus4E, PCPlus4M);
  flopenr #(5)  rdm(clk, reset,      ~div_stallE  , rdE, rdM);
  
  //flopr #(1) con_pipe(clk, reset, Con_pipeM, Con_pipeW);
  //writeback stage
  flopenr #(32) aluM(clk, reset,     ~div_stallE ,ALUResultM, ALUResultW);
  flopenr #(32) pcplus4w(clk, reset, ~div_stallE ,PCPlus4M, PCPlus4W);
  flopenr #(32) resultW(clk, reset,  ~div_stallE  ,ReadDataM, ReadDataW);
  flopenr #(5)  rdw(clk, reset,      ~div_stallE ,rdM, rdW);
  mux3 #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W,  ResultSrcW, ResultW);

endmodule

//hazard detection added 
module hazard(input  [4:0] rs1D, rs2D, rs1E, rs2E, 
              input  [4:0] rdE, rdM, rdW,
              input        regwriteM, regwriteW, ResultSrcE0,PCSrcE,RtypedivE,DIV_validE,
              output  logic     div_stallE,
              output reg [1:0] forwardaE, forwardbE,
              output       stallF, stallD,flushD,flushE);

  wire lwstall;
 
  //TO ADD DIV STALL

  // forwarding sources to D stage (branch equality)
 // assign forwardaD = (rsD !=0 & rsD == writeregM & regwriteM);
 // assign forwardbD = (rtD !=0 & rtD == writeregM & regwriteM);

  // forwarding sources to E stage (ALU)
  always_comb 
    begin
      forwardaE = 2'b00; forwardbE = 2'b00;
      if (rs1E != 0)
        if (rs1E == rdM & regwriteM ) forwardaE = 2'b10;
        else if (rs1E == rdW & regwriteW) forwardaE = 2'b01;
      if (rs2E != 0)
        if (rs2E == rdM & regwriteM ) forwardbE = 2'b10;
        else if (rs2E == rdW & regwriteW ) forwardbE = 2'b01;
    end

  // stalls  Load word stall logic:
  assign #1 lwstall = ResultSrcE0 & (rdE == rs1D | rdE == rs2D);
  //assign #1 flushE = lwstall  //combed in next hazard
  assign #1 stallD = lwstall ;
  assign #1 stallF = stallD; // stalling D stalls all previous stages

  //Control hazard flush
  assign #1 flushD = PCSrcE;
  assign #1 flushE = lwstall|PCSrcE; // stalling D flushes next stage

  // *** not necessary to stall D stage on store if source comes from load;
  // *** instead, another bypass network could be added from W to M
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2,
               output logic [7:0] led);

  logic [31:0] rf[31:0];
  logic [31:0] sim_t3;
  logic [31:0] sim_t4;
  logic [31:0] sim_t5;
  logic [31:0] sim_t6;
  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(negedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;

  assign sim_t3 = rf[28];
  assign sim_t4 = rf[29];
  assign sim_t5 = rf[30];
  assign sim_t6 = rf[31];

  assign led = sim_t4[7:0];
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
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

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

 // initial
 //     $readmemh("/project/users/PKUSOC-18/work/cpu_for_fpga_class/riscv_sources/riscvtest.txt",RAM);
/* # riscvtest.s
# Sarah.Harris@unlv.edu
# David_Harris@hmc.edu
# 27 Oct 2020
#
# Test the RISC-V processor.  
#  add, sub, and, or, slt, addi, lw, sw, beq, jal
# If successful, it should write the value 25 to address 100

#       RISC-V Assembly         Description               Address   Machine Code   binary
main:   addi x2, x0, 5          # x2 = 5                  0         00500113   
        addi x3, x0, 12         # x3 = 12                 4         00C00193
        mul  x7, x3, x2         # x7 = (12 * 5) = 60      8         023103B3      0000001 00010 00011 000 00111 01100 11 
        div  x4, x7, x2         # x4 = (60 / 5) = 12      C         0223C233      0000001 00010 00111 100 00100 01100 11
        addi x2, x0, 5          # x2=5                    10        00500113      0000001 00100 00010 110 00101 01100 11
        add  x5, x5, x4         # x5 = (2 + 12) = 14      14        004282B3
        beq  x5, x7, end        # shouldn't be taken      18        02728863
        slt  x4, x3, x4         # x4 = (12 < 12) = 0      1C        0041A233
        beq  x4, x0, around     # should be taken         20        00020463
        addi x5, x0, 0          # shouldn't happen        24        00000293
around: slt  x4, x7, x2         # x4 = (3 < 5)  = 1       28        0023A233
        add  x7, x4, x5         # x7 = (1 + 11) = 12      2C        005203B3
        sub  x7, x7, x2         # x7 = (12 - 5) = 7       30        402383B3
        sw   x7, 84(x3)         # [96] = 7                34        0471AA23 
        lw   x2, 96(x0)         # x2 = [96] = 7           38        06002103 
        add  x9, x2, x5         # x9 = (7 + 11) = 18      3C        005104B3
        jal  x3, end            # jump to end, x3 = 0x44  40        008001EF
        addi x2, x0, 1          # shouldn't happen        44        00100113
end:    add  x2, x2, x9         # x2 = (7 + 18)  = 25     48        00910133
        sw   x2, 0x20(x3)       # mem[100] = 25           4C        0221A023 
done:   beq  x2, x2, done       # infinite loop           50        00210063
*/		
		

  assign rd = RAM[a[31:2]]; // word aligned

  assign RAM[0]  = 32'h00500113;
  assign RAM[1]  = 32'h00C00193;
  assign RAM[2]  = 32'h023103B3;
  assign RAM[3]  = 32'h0223C233;
  assign RAM[4]  = 32'h00500113;
  assign RAM[5]  = 32'h004282B3;
  assign RAM[6]  = 32'h02728863;
  assign RAM[7]  = 32'h0041A233;
  assign RAM[8]  = 32'h00020463;
  assign RAM[9]  = 32'h00000293;
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
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic        clk   ,
           input  logic        rst   ,
           input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           input  logic [1:0]  alum,
           input  logic        i_div_flush,
           output logic        o_div_busy,
           output logic        o_div_valid,
           output logic [31:0] result_out,
           output logic        zero);

  logic [31:0] condinvb, sum, result, 
               quotient , remainder,  
               mult_high, mult_low;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation
 // logic        div_i_start;    // div start

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];
  //serdiv
  serdiv #(32) div1 (
    clk  , rst   ,
    i_div_flush, alucontrol[1]&alum[0], o_div_busy, o_div_valid, alucontrol[2],
    a, b, quotient , remainder );
  //mul
  mult #(32) mult1 (
    alucontrol[2], alucontrol[1],
     a, b,   
     mult_high, mult_low);

  //reset div  i_start

  always_comb
    case ({alum,alucontrol}) inside
      5'b00000:  result_out = sum;         // add
      5'b00001:  result_out = sum;         // subtract
      5'b00010:  result_out = a & b;       // and
      5'b00011:  result_out = a | b;       // or
      5'b00100:  result_out = a ^ b;       // xor
      5'b00101:  result_out = sum[31] ^ v; // slt
      5'b00110:  result_out = a << b[4:0]; // sll
      5'b00111:  result_out = a >> b[4:0]; // srl
      5'b11??0:  result_out= quotient;
      5'b11??1:  result_out= remainder;
      5'b10??0:  result_out= mult_low;
      5'b10??1:  result_out= mult_high;
      default:  result_out = 32'bx;
    endcase
  assign zero = (result_out == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule

