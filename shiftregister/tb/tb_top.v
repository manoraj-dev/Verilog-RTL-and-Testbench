`timescale 1ns/1ns

//`include "../src/dut.v"
`include "./gates.v"

module tb_top;

reg clk;
reg rst_n;
reg serial_in;
wire [2:0] y;
 
shiftregister_3b  dut(.clk(clk), .rst_n(rst_n), .serial_in(serial_in), .y(y));
 
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
           clk=0;
           serial_in=0;
           rst_n = 0;
           #10;
           rst_n = 1;
			  repeat(200)
			  begin
           #8;
           serial_in=$random();
           end
end
initial
begin
		 #1;
       $monitor("time=%t | rst_n=%b | y=%3b | serial_in=%b",$time, rst_n, y,serial_in);
       #2000;
       $finish;
end

endmodule
