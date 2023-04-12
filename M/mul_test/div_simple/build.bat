set testbentch_module=div_tb
set rtl_file="*.sv"
set gtkw_file="%testbentch_module%.gtkw"

iverilog -g2012 -o "%testbentch_module%.vvp" %rtl_file% 
vvp -n "%testbentch_module%.vvp" -lxt2

IF EXIST %gtkw_file%  (
        gtkwave %gtkw_file% 
) ELSE (
        gtkwave "%testbentch_module%.vcd"
)  
pause