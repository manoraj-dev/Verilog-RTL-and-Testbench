`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

// vars declare
reg A, B;
wire sum,carry;

// DUT instantiate
halfadder_b u_dut(
	.sum(sum),
	.carry(carry),
	.A(A),
	.B(B)
);

// initialize signals
initial begin
 A=1'b0;
 B=1'b0;
end

// Add signals to VCD waveforms
initial begin
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end

//monitor singnals
initial begin
  $display("==================================================================");
  $monitor("| %4t | sum=%b|carry=%b| A=%b| B=%b|", $time, sum,carry, A, B);
end

// Procedural signals driven
always begin
 #2;
 A=1'b0;
 B=1'b1;
 #5;
 A=1'b0;
 B=1'b1;
 #5;
 A=1'b0;
 B=1'b1;
 #5;
 A=1'b1;
 B=1'b0;
 #5;
 A=1'b1;
 B=1'b1;
 #1;
 $finish;
end

endmodule
