`timescale 1ps/1ps
module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
   logic [7:0] led;

  // instantiate device to be tested
  top dut(clk, reset, led);
  
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
  //   initial begin 
   //    $fsdbDumpfile("riscvpipe.fsdb");
   //       $fsdbDumpvars(0);
   //  end 

  // check results
  //always @(negedge clk)
  //  begin
  //    if(MemWrite) begin
  //      if(DataAdr === 100 & WriteData === 25) begin
  //        $display("Simulation succeeded");
  //         $finish();//$stop;
  //      end else if (DataAdr !== 96) begin
  //        $display("Simulation failed");
  //        $finish(); //$stop;
  //      end
  //    end
  //  end

  initial begin

      #10800;

      $finish;//主动的结束仿真

    end

  initial begin
        $dumpfile("testbench.vcd");
        $dumpvars(0, testbench);    //tb模块名称
    end
endmodule