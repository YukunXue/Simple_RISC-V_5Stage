
module serdiv#(
  parameter WIDTH = 32
)(
  input                     i_clk         ,
  input                     i_rst        ,
  input                     i_flush       , //取消当前计算
  input                     i_start       , //开始计算
  output                    o_busy        , //计算中
  output logic              o_end_valid   , //计算结束
  input                     i_signed      , //有符号数1 无符号数0
  input  logic [WIDTH-1:0]  i_dividend    ,
  input  logic [WIDTH-1:0]  i_divisor     ,
  output logic [WIDTH-1:0]  o_quotient    ,
  output logic [WIDTH-1:0]  o_remainder
);

  // 1. control signal:///////////////////////////////////////////////////////////////////////
  localparam CNT_W = $clog2(WIDTH); 
  
  logic [CNT_W-1:0] cnt;

  wire cntneq0 = |cnt; // cnt != 0
  
  assign o_busy = (cntneq0) | o_end_valid;

  always@(posedge i_clk or negedge i_rst)begin
    if(i_rst)begin
      cnt <= {CNT_W{1'b0}};
    end else if(i_flush) begin
      cnt <= {CNT_W{1'b0}};
    end else if(i_start) begin
      cnt <= {CNT_W{1'b1}}; //  63.
    end else if(cntneq0) begin // cnt != 0
      cnt <= cnt - 1;
    end
  end

  
  always@(posedge i_clk or negedge i_rst)begin
    if(i_rst)begin
      o_end_valid <= 1'b0;
    end else if(i_flush)begin
      o_end_valid <= 1'b0;
    end else if(cnt == {{(CNT_W-1){1'b0}},1'b1}) begin  //1
      o_end_valid <= 1'b1;
    end else if(o_end_valid) begin
      o_end_valid <= 1'b0;
    end
  end

  // 2. deal input signals://///////////////////////////////////////////////////////////////////
  wire [WIDTH-1:0] i_dividend_wrapper =  i_dividend;
  wire [WIDTH-1:0] i_divisor_wrapper  =  i_divisor ;

  wire dividend_positive = i_signed ? ~i_dividend_wrapper[WIDTH-1] : 1;
  wire divisor_positive  = i_signed ? ~i_divisor_wrapper [WIDTH-1] : 1;

  wire [WIDTH-1:0] i_dividend_abs = dividend_positive ? i_dividend_wrapper : ~i_dividend_wrapper + 1'b1;  
  wire [WIDTH-1:0] i_divisor_abs  = divisor_positive  ? i_divisor_wrapper  : ~i_divisor_wrapper  + 1'b1;

  // 3. div:///////////////////////////////////////////////////////////////////////////////////
  logic [WIDTH-1  :0] divisor_r;
  logic [2*WIDTH-1:0] dividend , dividend_r , dividend_r_shift ;
  logic [WIDTH-1  :0] quotient , quotient_r ;

  always@(posedge i_clk or negedge i_rst)begin
    if(i_rst)begin
      dividend_r  <= {2*WIDTH{1'b0}};
      divisor_r   <= {  WIDTH{1'b0}};
      quotient_r  <= {  WIDTH{1'b0}};
    end else if(i_flush) begin
      dividend_r  <= {2*WIDTH{1'b0}};
      divisor_r   <= {  WIDTH{1'b0}};
      quotient_r  <= {  WIDTH{1'b0}};
    end else if(i_start) begin  //初始化
      dividend_r  <= {{WIDTH{1'b0}}, i_dividend_abs};
      divisor_r   <= i_divisor_abs;
      quotient_r  <= {WIDTH{1'b0}};
    end else if(cntneq0) begin
      dividend_r  <= dividend ;
      quotient_r  <= quotient ;
    end
  end

  logic [WIDTH-1:0] div_sub, mask;
  logic sub_positive, sub_negative;

  assign dividend_r_shift = {dividend_r >> cnt};
  assign {sub_negative,div_sub} = dividend_r_shift[WIDTH:0] - {1'b0,divisor_r};
  assign sub_positive = ~sub_negative;

  assign mask = ~({WIDTH{1'b1}} << cnt); // low bit mask.
  assign dividend = sub_negative ? dividend_r : {{WIDTH{1'b0}}, {(div_sub<<cnt) | (dividend_r[WIDTH-1:0] & mask)}};//补全没用到的位

  for(genvar i=0; i<WIDTH; i++)begin
    assign quotient[i]  = (i == cnt) ? sub_positive : quotient_r[i];
  end

  // 4. output: //cnt == 0输出结果
  assign o_quotient  = cntneq0 ? {WIDTH{1'b0}} : (~(dividend_positive^divisor_positive) ? quotient : ~quotient + 1'b1)  ;
  assign o_remainder = cntneq0 ? {WIDTH{1'b0}} : (dividend_positive ? dividend[WIDTH-1:0] : ~dividend[WIDTH-1:0] + 1'b1); 

endmodule



module mult1 #(
  parameter W = 32          // width should not be changed, only support 64 now.
)(
  input             i_x_sign,
  input             i_y_sign,
  input 	[W-1:0]   i_x     ,
  input 	[W-1:0]   i_y     ,
  output 	[W-1:0]   o_hi_res,
  output 	[W-1:0]   o_lw_res
);

  logic [63:0] mul;  
  assign mul = i_x * i_y;
  
  assign o_hi_res = mul[63:32];
  assign o_lw_res = mul[31:0];
   
endmodule



module mult #(
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
  assign y = {i_y_sign ? {2{i_y[W-1]}} : 2'b0, i_y[W-1:0]};

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
    assign tree_in[i] = {{(TOTAL_W-1-2*i){c[i]}}, p[i], 1'b0, c[i-1], {(2*i-2){1'b0}}};
  end

  wallace_tree_17 #(2*TOTAL_W) wallace_tree (.in(tree_in),.out(tree_out));

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

  assign p = sel_double_negative ? ~{x, 1'b0} : 
            (sel_double_positive ? {x, 1'b0} :
            (sel_negative ? ~{1'b0,x}:
            (sel_positive ?  {1'b0,x} : {(WIDTH+1){1'b0}})));
  assign c = sel_double_negative | sel_negative ? 1'b1 : 1'b0;

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
