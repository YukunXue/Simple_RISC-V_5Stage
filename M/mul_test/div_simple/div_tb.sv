`timescale 1ps/1ps
module div_tb(
);

	logic [7:0] a;
	logic [7:0] b;
	logic [7:0] c;

	initial begin
		a = 8'b11111000;
		b = 8'b10000;
		#10
		$finish;
	end

	always @(*) begin
		c = $signed(a) % $signed(b);
	end

	initial begin
        $dumpfile("div_tb.vcd");
        $dumpvars(0, div_tb);    //tb模块名称
   end

endmodule