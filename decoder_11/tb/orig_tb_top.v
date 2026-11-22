`timescale 1ns/1ps


`include "../src/dut.v"

module tb_top;

reg [2:0] A;
reg expected;
wire [7:0] Y;

decoder_3to8 dut (
    .A(A),
    .Y(Y)
);

initial begin

    $dumpfile("waveform.vcd");
    $dumpvars(0, tb_top);

    $monitor("Time=%0t | A=%b | Y=%b",
              $time, A, Y);
//self checker 
 A = 3'b000;
    expected = 8'b00000001;
    #10;

    if(Y == expected)
        $display("TEST 1 : PASS");
    else
        $display("TEST 1 : FAIL");
A = 3'b001;
	expected =8'b00000010;
	#10;
	if(Y == expected)
	     $display("TEST 2 : PASS");
    else
        $display("TEST 2 : FAIL");
A = 3'b010;
	expected =8'b00000100;
	 #10;
    if(Y == expected)
	     $display("TEST 3 : PASS");
    else
        $display("TEST 3 : FAIL");

A = 3'b110;
  expected =8'b00001000;
 #10;
     if(Y == expected)
       $display("TEST 4 : PASS");
     else
       $display("TEST 4 : FAIL");
$finish;
end 
endmodule
