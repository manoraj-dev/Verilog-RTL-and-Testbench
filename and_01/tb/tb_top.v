`timescale 1ns/1ns

`include "../src/test.v"

module tb_top;

// vars declare
reg A, B;
wire Y_AND, Y_OR, Y_NAND, Y_NOR;
wire Y_NOT, Y_BUF, Y_XOR, Y_XNOR;


// DUT instantiate
test  dut(
	.A(A),
	.B(B),
	.Y_AND(Y_AND),
   .Y_OR(Y_OR),
   .Y_NAND(Y_NAND),
   .Y_NOR(Y_NOR),
   .Y_NOT(Y_NOT),
   .Y_BUF(Y_BUF),
   .Y_XOR(Y_XOR),
   .Y_XNOR(Y_XNOR)
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
  $monitor("Time=%0t | A=%b B=%b | AND=%b OR=%b NAND=%b NOR=%b NOT=%b BUF=%b XOR=%b XNOR=%b",
    $time, A, B, Y_AND, Y_OR, Y_NAND, Y_NOR, Y_NOT, Y_BUF, Y_XOR, Y_XNOR);
end

//Checker
//always begin
//  if(A=0) || (B==0)
//		$display("Test Passed!!!"); 
//  else()
//		$display("Test Failed!!!"); 
//end
`
// Procedural signals driven
always begin
	#2;
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
 #4;
	//repeat(10) begin
	//	#2;
	//	A=$random();
	//	B=$random();
	end

$finish;

endmodule
