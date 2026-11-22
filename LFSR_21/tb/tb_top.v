`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;

    reg clk;
    reg arst_n;
    wire [2:0] q;

    galois_lfsr dut(.clk(clk), .arst_n(arst_n), .q(q));

   initial 
   begin
      clk = 0;
      forever #4 clk = ~clk;
    end

	initial 
		begin
       $dumpfile("waveform.vcd");
       $dumpvars(0,tb_top);
    end

	initial
		begin
       	arst_n = 0; //initialize registers to "001" first
       	#6;
        	arst_n = 1'b1; //deassert reset
       	#3;
        	arst_n = 1'b0; //assert reset to re-initialize to "001" again
       	#3;
        	arst_n = 1'b1; //deassert reset
			#2000;
       	$finish;
    end

    initial
         begin
			#1;
         $monitor("Time=%0t | clk=%b | Reset=%b | Q=%b", $time, clk, arst_n, q);
    end

endmodule
