`timescale 1ps/1ps
`include "defines.v"
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

  wire [63:0] mul;  

  wire [`MUL_OP-1:0] mul_op = {i_x_sign, i_y_sign};


  always @(*)begin
      case(mul_op)
      `MULH : mul = $signed(i_x) * $signed(i_y);

      `MULHU: mul = $unsigned(i_x) * $unsigned(i_y);

      `MULHSU: mul = $signed(i_x) * $unsigned(i_y);

      `MULHUS: mul = $unsigned(i_x) * $signed(i_y);

      endcase
  end

  assign mul = i_x * i_y;
  
  assign o_hi_res = mul[63:32];
  assign o_lw_res = mul[31:0];
   
endmodule