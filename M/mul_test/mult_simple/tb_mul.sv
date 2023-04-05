`timescale 1ps/1ps
`include "defines.v"
module tb_mul();

  logic x_sign;
  logic y_sign;
  
  logic [31:0] x;
  logic [31:0] y;
 
  logic [5:0] x_b;
  logic [5:0] y_b;

  logic [5:0] hi_bb;
  logic [5:0] lo_bb;

  logic [31:0] hi;
  logic [31:0] lo;

  logic [31:0] hi_b;
  logic [31:0] lo_b;
  
  mult_simple mult1(
	x_sign,
	y_sign,
	x,
	y,
	hi,
	lo
  );

  mult_booth mult2(
	x_sign,
	y_sign,
	x,
	y,
	hi_b,
	lo_b	
  );

  mult_booth #(6)mult3(
	x_sign,
	y_sign,
	x_b,
	y_b,
	hi_bb,
	lo_bb	
  );

  initial begin

	{x_sign, y_sign} = 2'b11;
	//x = $random % 4294967296;
	//y = $random % 4294967296;
	//x = 32'd4294967295;
	//y = 32'd1; 
	x = 32'hFFFFFFF8;
	y = 32'd1; 

	x_b = 6'b111110;
	y_b = 6'b001111; 

	#10
	{x_sign, y_sign} = 2'b10;
	//x = $random % 4294967296;
	//y = $random % 4294967296;
	x = 32'hFFFFFFF8;
	y = 32'd1;

	x_b = 6'b111110;
	y_b = 6'b001111; 

	#10
	{x_sign, y_sign} = 2'b00;
	//x = $random % 4294967296;
	//y = $random % 4294967296;
	x = 32'hFFFFFFF8;
	y = 32'd1;

	x_b = 6'b111110;
	y_b = 6'b001111;  
	
	#10
	$finish;
  end


  initial begin
        $dumpfile("tb_mul.vcd");
        $dumpvars(0, tb_mul);    //tb模块名称
   end
endmodule


module mult_simple #(
  parameter W = 32          // width should not be changed, only support 64 now.
)(
  input             i_x_sign,
  input             i_y_sign,
  input 	[W-1:0]   i_x     ,
  input 	[W-1:0]   i_y     ,
  output 	[W-1:0]   o_hi_res,
  output 	[W-1:0]   o_lw_res
);

  reg [63:0] mul;  

  wire [`MUL_OP-1:0] mul_op ;
  assign mul_op = {i_x_sign, i_y_sign};


  always @(*)begin
      case(mul_op)
      `MULH :  mul = $signed(i_x) * $signed(i_y);

      `MULHU:  mul = $unsigned(i_x) * $unsigned(i_y);

      `MULHSU: mul = $signed(i_x) * $unsigned(i_y);

      `MULHUS: mul = $unsigned(i_x) * $signed(i_y);

      endcase
  end
  
  assign o_hi_res = mul[63:32];
  assign o_lw_res = mul[31:0];
   
endmodule

module mult_booth #(
  parameter W = 32          // width should not be changed, only support 64 now.
)(
  input             i_x_sign,
  input             i_y_sign,
  input 	[W-1:0]   i_x     ,
  input 	[W-1:0]   i_y     ,
  output 	[W-1:0]   o_hi_res,
  output 	[W-1:0]   o_lw_res
);

  localparam TOTAL_W = W + 2 ; // 2 for signed extension, 34 totally.
  localparam PNUM = TOTAL_W/2; // 17

  logic	[TOTAL_W-1:0]   x  ; //34bit
  logic	[TOTAL_W-1:0]   y  ; //34bit
  logic [2*TOTAL_W-1:0] res; //68bit

  assign x =  {i_x_sign ? {2{i_x[W-1]}} : 2'b0, i_x[W-1:0]}; // 34bit 扩展两位符号数
  assign y =  {i_y_sign ? {2{i_y[W-1]}} : 2'b0, i_y[W-1:0]};

  // 1. generate partial product://///////////////////////////////////////////////////////////
  wire  [TOTAL_W:0] p[PNUM-1:0];   //17个 34bit部分积
  wire  c [PNUM-1:0];             //扩展位

  booth #(.WIDTH (TOTAL_W)) B_0(.x (x),.s ({y[1:0], 1'b0}),.p (p[0]),.c (c[0]));     //p[0] 34bit
  for(genvar i=1; i<PNUM; i=i+1)begin:Booths
    booth #(.WIDTH (TOTAL_W)) B_(.x (x),.s (y[2*i+1 : 2*i-1]),.p (p[i]),.c (c[i]));
  end

  // 2. use wallace tree to generate result://////////////////////////////////////////////////
  wire [2*TOTAL_W-1:0] tree_in [PNUM-1:0];	// with modified sign extension  扩展成68bit
  wire [2*TOTAL_W-1:0] tree_out [1:0];      //压缩成两个68bit相加
  assign tree_in[ 0] = {{(TOTAL_W-1){c[0]}} , p[0]	};
  for(genvar i=1; i<PNUM; i=i+1)begin:gen_tree_in
    //assign tree_in[i] = {{(TOTAL_W-1-2*i){c[i]}}, p[i], 1'b0, c[i-1], {(2*i-2){1'b0}}};
    assign tree_in[i] = {{(TOTAL_W-1-2*i){c[i]}}, p[i], {(2*i){1'b0}}};
  end

	wire [2*TOTAL_W-1:0] tree_in0 = tree_in[0];
	wire [2*TOTAL_W-1:0] tree_in1 = tree_in[1];
	wire [2*TOTAL_W-1:0] tree_in2 = tree_in[2];
	wire [2*TOTAL_W-1:0] tree_in3 = tree_in[3];
	assign tree_out[0] = tree_in[0] + tree_in[1];
	assign tree_out[1] = tree_in[2] + tree_in[3];

	
//  wallace_tree_17 #(2*TOTAL_W) wallace_tree (.in(tree_in),.out(tree_out));

  // 3. full connect adder://///////////////////////////////////////////////////////////////////
  logic carry;
  rca_nbit #(.N (2*TOTAL_W)) u_rca_nbit(.i_a (tree_out[1]),.i_b (tree_out[0]), .i_c (1'b0),.o_s (res), .o_c(carry));
  assign {o_hi_res, o_lw_res} = res[2*W-1:0];

endmodule

module booth #(parameter WIDTH=32) (
  input [WIDTH-1:0] x,
  input [2:0] s,
  output wire [WIDTH:0] p,  //部分积
  output wire c             //负1正0
);

  wire y_add,y,y_sub; // y+1,y,y-1
  wire sel_negative,sel_double_negative,sel_positive,sel_double_positive;

  assign {y_add,y,y_sub} = s;

  assign sel_negative =  y_add & (y & ~y_sub | ~y & y_sub);
  assign sel_positive = ~y_add & (y & ~y_sub | ~y & y_sub);
  assign sel_double_negative =  y_add & ~y & ~y_sub;
  assign sel_double_positive = ~y_add &  y &  y_sub;

  wire [WIDTH-1:0] inv_x;
  assign inv_x = ~x+1'b1;
  assign p = sel_double_negative ? {inv_x, 1'b0} : 
  			//sel_double_negative ? ~{x, 1'b0} : 
            (sel_double_positive ? {x, 1'b0} :
            //(sel_negative ? ~{1'b0,x}:
			(sel_negative ? {inv_x[WIDTH-1],inv_x}:
            (sel_positive ?  {x[WIDTH-1],x} : {(WIDTH+1){1'b0}})));
//  assign c = sel_double_negative | sel_negative ? 1'b1 : 1'b0;
  assign c = p[WIDTH];

endmodule

module wallace_tree_17 #(
  parameter WIDTH=64
) (
  input  [WIDTH-1:0] in [16:0],
  output [WIDTH-1:0] out[1:0]
);

  wire  [WIDTH-1:0] s_row1[4:0];
  wire  [WIDTH-1:0] c_row1[4:0];
  wire  [WIDTH-1:0] c_row1_shift[4:0];

  wire  [WIDTH-1:0] s_row2[3:0];
  wire  [WIDTH-1:0] c_row2[3:0];
  wire  [WIDTH-1:0] c_row2_shift[3:0];
  
  wire  [WIDTH-1:0] s_row3[1:0];
  wire  [WIDTH-1:0] c_row3[1:0];
  wire  [WIDTH-1:0] c_row3_shift[1:0];

  wire  [WIDTH-1:0] s_row4[1:0];
  wire  [WIDTH-1:0] c_row4[1:0];
  wire  [WIDTH-1:0] c_row4_shift[1:0];

  wire  [WIDTH-1:0] s_row5;
  wire  [WIDTH-1:0] c_row5;
  wire  [WIDTH-1:0] c_row5_shift;

  wire  [WIDTH-1:0] s_row6, c_row6, c_row6_shift;
 
 

  // 1. first level, 5 csa, 17p -> 2x5+2=12p://////////////////////////////////////////////////////////////
  // left in15 in16
  for(genvar i=0; i<5; i=i+1)begin: csa_row1
    csa_nbit #(.N(WIDTH)) csa_row1  (in[3*i] , in[3*i+1] , in[3*i+2] , s_row1[i], c_row1[i]);
    assign c_row1_shift[i] = {c_row1[i][WIDTH-2:0], 1'b0};
  end

  // 2. second level, 4 csa, 12p -> 8p :///////////////////////////////////////////////////////////
  //add in15 in16;
  csa_nbit #(.N(WIDTH)) csa_row2_0  (s_row1[0]      , c_row1_shift[0],  s_row1[1]      ,  s_row2[0], c_row2[0]);
  csa_nbit #(.N(WIDTH)) csa_row2_1  (c_row1_shift[1], s_row1[2]      ,  c_row1_shift[2],  s_row2[1], c_row2[1]);
  csa_nbit #(.N(WIDTH)) csa_row2_2  (s_row1[3]      , c_row1_shift[3],  s_row1[4]      ,  s_row2[2], c_row2[2]);
  csa_nbit #(.N(WIDTH)) csa_row2_3  (c_row1_shift[4], in[15]      ,  in[16],              s_row2[3], c_row2[3]);
  for(genvar i=0; i<4; i=i+1)begin: csa_row2_shift
    assign c_row2_shift[i] = {c_row2[i][WIDTH-2:0], 1'b0};
  end

  // 3. third level, 2 csa, 8p -> 6p: ///////////////////////////////////////////////////////////////////
  //left s_row2[3], c_row2_shift[3]
  csa_nbit #(.N(WIDTH)) csa_row3_0  (s_row2[0],     s_row2[1]    , c_row2_shift[0], s_row3[0], c_row3[0]);
  csa_nbit #(.N(WIDTH)) csa_row3_1  (s_row2[2]   , c_row2_shift[1], c_row2_shift[2]  , s_row3[1], c_row3[1]);
  for(genvar i=0; i<2; i=i+1)begin: csa_row3_shift
    assign c_row3_shift[i] = {c_row3[i][WIDTH-2:0], 1'b0};
  end

  // 4. fourth level, 2csa, 6p -> 4p : ////////////////////////////////////////////////////////////
  csa_nbit #(.N(WIDTH)) csa_row4_0  (s_row3[0]       , s_row3[1],   s_row2[3]      , s_row4[0], c_row4[0]);
  csa_nbit #(.N(WIDTH)) csa_row4_1  (c_row2_shift[3] ,c_row3_shift[0]  , c_row3_shift[1], s_row4[1], c_row4[1]);
  for(genvar i=0; i<2; i=i+1)begin: csa_row4_shift
    assign c_row4_shift[i] = {c_row4[i][WIDTH-2:0], 1'b0};
  end

  // 5. fifth level, 1 csa, 4p -> 3p (2+1): //////////////////////////////////////////////////////////////
  csa_nbit #(.N(WIDTH)) csa_row5_0  (c_row4_shift[0], s_row4[0]   , s_row4[1], s_row5, c_row5);
  
    assign c_row5_shift = {c_row5[WIDTH-2:0], 1'b0};
 

  // 6. sixth level, 1 csa, 3p -> 2p : ////////////////////////////////////////////////////////////////
  csa_nbit #(.N(WIDTH)) csa_row6    (c_row4_shift[1], s_row5, c_row5_shift, s_row6, c_row6);
  assign c_row6_shift = {c_row6[WIDTH-2:0], 1'b0};

  // 9. output 2p :////////////////////////////////////////////////////////////////////////////////////////
  assign out[1] = c_row6_shift;
  assign out[0] = s_row6;

endmodule

module csa_nbit#(
  parameter N = 64
)(
  input  [N-1:0] i_a, 
  input  [N-1:0] i_b, 
  input  [N-1:0] i_c, 
  output [N-1:0] o_s,  //sum
  output [N-1:0] o_c  //count
);

  wire [N-1:0] p;
  wire [N-1:0] g;

  genvar i;
  generate
    for(i=0; i<N; i=i+1)begin:csa
      assign g[i] = i_a[i] & i_b[i];
      assign p[i] = i_a[i] ^ i_b[i];
      assign o_s[i] = p[i] ^ i_c[i];
      assign o_c[i] = i_c[i] & p[i] | g[i] ; 
    end
  endgenerate

endmodule

module rca_nbit#(
  parameter N = 64
)(
  input  [N-1:0] i_a, 
  input  [N-1:0] i_b, 
  input          i_c, 
  output [N-1:0] o_s, 
  output         o_c
);

  wire [N-1:0] p;
  wire [N-1:0] g;
  // verilator lint_off UNOPTFLAT
  wire [N:0] c;
  // verilator lint_on UNOPTFLAT

  for(genvar i=0; i<N; i=i+1)begin:csa
    assign g[i]   = i_a[i] & i_b[i];
    assign p[i]   = i_a[i] ^ i_b[i];
    assign c[i+1] = c[i] & p[i] | g[i];
    assign o_s[i] = p[i] ^ c[i];
  end

  assign c[0] = i_c;
  assign o_c = c[N];

endmodule