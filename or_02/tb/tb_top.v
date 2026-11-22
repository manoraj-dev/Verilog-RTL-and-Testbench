`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

// vars declare
reg A, B, C, D;
wire Y;

// DUT instantiate
NAND4 u_dut(
	.Y(Y),
	.A(A),
	.B(B),
	.C(C),
	.D(D)
);

// initialize signals
initial begin
 A=1'b0;
 B=1'b0;
 C=1'b0;
 D=1'b0;
end

// Add signals to VCD waveforms
initial begin
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end

//monitor singnals
initial begin
  $display("==================================================================");
  $monitor("| %4t | Y=%b| A=%b| B=%b| C=%b| D=%b |", $time, Y , A, B, C, D);
end

// Procedural signals driven
always begin
 #2;
 A=1'b0;
 B=1'b1;
 C=1'b0;
 D=1'b1;
 #5;
 A=1'b0;
 B=1'b1;
 C=1'b1;
 D=1'b1;
 #5;
 A=1'b0;
 B=1'b1;
 C=1'b0;
 D=1'b1;
 #5;
 A=1'b1;
 B=1'b0;
 C=1'b1;
 D=1'b0;
 #5;
 A=1'b1;
 B=1'b1;
 C=1'b1;
 D=1'b1;
 #1;
 $finish;
end

endmodule
