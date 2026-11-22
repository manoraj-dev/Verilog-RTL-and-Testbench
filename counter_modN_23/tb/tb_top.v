`timescale 1ns/1ns

`include "../src/dut.v"

module tb_top;
localparam WIDTH=4;
reg clk;
reg rstn;
wire [WIDTH-1:0] y;

initial 
begin
	clk = 0;
	rstn = 0;
	$dumpfile("waveform.vcd");
	$dumpvars(0, tb_top);
end 

counter_mod_N #(.N(10),.WIDTH(4)) dut (.clk(clk),.rstn(rstn),.y(y));

always begin
	#2 clk = ~clk;
end

initial begin
	rstn = 0;
	#10;
	rstn = 1;
end

always begin
	#1;
	$monitor("Time=%t rstn=%1b count=%4b",$time, rstn, y);
	#200;
	$finish;
end

endmodule
