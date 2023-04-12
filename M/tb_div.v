module testbench();

  logic        clk;
  logic        reset;

  logic   i_flush     ;
  logic   i_start ,i_start_real    ;
  logic   o_busy      ;
  logic   o_end_valid ;
  logic    i_signed    ;

  logic [31:0]  i_dividend ,x,y,s,r;
  logic [31:0]  i_divisor  ;
  logic [31:0]  o_quotient ;
  logic [31:0]  o_remainder;

  wire div_signed;
  assign div_signed = i_signed;
    assign x = i_dividend;
    assign y = i_divisor;
    assign s = o_quotient;
    assign r = o_remainder;

//-----{计算参考结果}begin
//第一步，求 x 和 y 的绝对值，并判断商和余数的符号
wire x_signed = x[31] & div_signed;               //x 的符号位，做无符号时认为是 0
wire y_signed = y[31] & div_signed;               //y 的符号位，做无符号时认为是 0
wire [31:0] x_abs;
wire [31:0] y_abs;
assign x_abs = ({32{x_signed}}^x) + x_signed;     //此处异或运算必须加括号
assign y_abs = ({32{y_signed}}^y) + y_signed;     //因为 verilog 中+的优先级更高
wire s_ref_signed = (x[31]^y[31]) & div_signed;   //运算结果商的符号位，做无符号时认为是 0
wire r_ref_signed = x[31] & div_signed;           //运算结果余数的符号位，做无符号时认为是 0

//第二步，求得商和余数的绝对值
reg [31:0] s_ref_abs;
reg [31:0] r_ref_abs;
always @(clk)
begin
    s_ref_abs <= x_abs/y_abs;
    r_ref_abs <= x_abs-s_ref_abs*y_abs; 
end

//第三步，依据商和余数的符号位调整
wire [31:0] s_ref;
wire [31:0] r_ref;
///此处异或运算必须加括号，因为 verilog 中+的优先级更高
assign s_ref = ({32{s_ref_signed}}^s_ref_abs) + {30'd0,s_ref_signed}; 
assign r_ref = ({32{r_ref_signed}}^r_ref_abs) + r_ref_signed;
//-----{计算参考结果}end
//判断结果是否正确
wire s_ok;
wire r_ok;
assign s_ok = s_ref==s;
assign r_ok = r_ref==r;
reg [5:0] time_out;

////输出结果,将各 32 位(不论是有符号还是无符号数)扩展成 33 位有符号数，以便以 16 进制形式打印
wire signed [32:0] x_d     = {div_signed&x[31],x};
wire signed [32:0] y_d     = {div_signed&y[31],y};
wire signed [32:0] s_d     = {div_signed&s[31],s};
wire signed [32:0] r_d     = {div_signed&r[31],r};
wire signed [32:0] s_ref_d = {div_signed&s_ref[31],s_ref};
wire signed [32:0] r_ref_d = {div_signed&r_ref[31],r_ref};

  // instantiate device to be tested
  serdiv dut(clk,
             reset, 
             i_flush    ,
             i_start_real    ,
             o_busy     ,
             o_end_valid,
             i_signed   ,
                i_dividend ,
                i_divisor  ,
                o_quotient ,
                o_remainder
             );
  
  // initialize test
  initial
    begin
      reset <= 1; # 22; reset <= 0;
     //    #1000 $finish;
    end

  initial
  begin
    i_flush <= 0;
     i_start <= 0;
     # 30;i_start <= 1;
     #10; i_start <= 0;
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
  always@(posedge clk)
  begin
    i_start_real <= i_start;
  end
  // check results
  always @(posedge clk)
    begin
      if(i_start) begin
          i_start <= 0;
          i_signed <= {$random}%2; 
		  i_dividend <= $random; 
		  i_divisor  <= $random;  ////被除数随机产生 0 的概率很小，基本可忽略
      end
        if(o_end_valid) begin
            i_start <= 1;
            if (s_ok && r_ok)
		  begin
		      $display("[x=%x, y=%x, signed=%x, s=%x, r=%x, s_OK=%b, r_OK=%b",
                      x_d,y_d,div_signed,s_d,r_d,s_ok,r_ok);
		  end
		  else 
		  if (~(s_ok && r_ok))
		  begin
		      $display("Error: x=%x, y=%x, signed=%x, s=%x, r=%x, s_ref=%x, r_ref=%x, s_OK=%b, r_OK=%b",
                      x_d,y_d,div_signed,s_d,r_d,s_ref_d,r_ref_d,s_ok,r_ok);
	         $finish;
		  end
        end
    end
  initial begin

      #100000ns;

      $finish ( );//主动的结束仿真

    end

endmodule
