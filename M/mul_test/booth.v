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